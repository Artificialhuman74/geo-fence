#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <TinyGPS++.h>

#include "config.h"

TinyGPSPlus gps;

static unsigned long lastReportMs = 0;
static unsigned long lastHistoryMs = 0;
static unsigned long lastStatusMs = 0;
static unsigned long lastCommandPollMs = 0;
static unsigned long lastWifiAttemptMs = 0;

// Non-blocking buzzer state machine so beeping never stalls GPS/network work.
static bool buzzerActive = false;
static bool buzzerOn = false;
static int buzzerBeepsLeft = 0;
static unsigned long buzzerNextToggleMs = 0;

void startAlarmBeeps() {
  buzzerActive = true;
  buzzerOn = false;
  buzzerBeepsLeft = BUZZER_BEEP_COUNT;
  buzzerNextToggleMs = millis();
}

void serviceBuzzer() {
  if (!buzzerActive) return;
  unsigned long now = millis();
  if (now < buzzerNextToggleMs) return;

  if (buzzerOn) {
    digitalWrite(BUZZER_PIN, LOW);
    buzzerOn = false;
    buzzerBeepsLeft--;
    if (buzzerBeepsLeft <= 0) {
      buzzerActive = false;
      return;
    }
    buzzerNextToggleMs = now + BUZZER_GAP_MS;
  } else {
    digitalWrite(BUZZER_PIN, HIGH);
    buzzerOn = true;
    buzzerNextToggleMs = now + BUZZER_BEEP_MS;
  }
}

void ensureWifiConnected() {
  if (WiFi.status() == WL_CONNECTED) return;

  unsigned long now = millis();
  if (now - lastWifiAttemptMs < WIFI_RECONNECT_INTERVAL_MS) return;
  lastWifiAttemptMs = now;

  Serial.println("WiFi not connected, (re)connecting...");
  WiFi.disconnect();
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_CONNECT_TIMEOUT_MS) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi connected, IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi connect attempt timed out.");
  }
}

// Firebase REST calls skip TLS certificate validation (setInsecure()) to
// avoid bundling/pinning Google's root CA on the ESP32 — acceptable for a
// hobby device pushing to your own database, but it means a network-level
// attacker could in principle MITM these requests. Pin the root CA instead
// if that risk matters for your deployment.
//
// One TLS connection is kept alive and reused across requests (setReuse):
// the handshake, not the transfer, dominates per-publish latency, so this
// is what makes sub-second reporting possible on a hotspot link.
static WiFiClientSecure tlsClient;
static HTTPClient httpClient;
static bool httpClientInit = false;

static bool firebaseBegin(const String &path) {
  if (!httpClientInit) {
    tlsClient.setInsecure();
    httpClient.setReuse(true);
    httpClientInit = true;
  }
  String url = "https://" + String(FIREBASE_HOST) + path + ".json?auth=" + String(FIREBASE_AUTH);
  return httpClient.begin(tlsClient, url);
}

bool firebasePut(const String &path, const String &jsonBody) {
  if (!firebaseBegin(path)) {
    Serial.println("firebasePut: http.begin failed");
    return false;
  }
  httpClient.addHeader("Content-Type", "application/json");
  int code = httpClient.PUT(jsonBody);
  httpClient.end();  // keeps the socket open thanks to setReuse(true)

  if (code < 200 || code >= 300) {
    Serial.printf("firebasePut(%s) failed, HTTP %d\n", path.c_str(), code);
    return false;
  }
  return true;
}

String firebaseGet(const String &path) {
  if (!firebaseBegin(path)) {
    Serial.println("firebaseGet: http.begin failed");
    return "";
  }
  int code = httpClient.GET();
  String body;
  if (code == 200) {
    body = httpClient.getString();
  } else {
    Serial.printf("firebaseGet(%s) failed, HTTP %d\n", path.c_str(), code);
  }
  httpClient.end();
  return body;
}

unsigned long currentUnixTimestamp() {
  if (!(gps.date.isValid() && gps.time.isValid())) return 0;
  // GPS time is already UTC, but ESP32's newlib has no timegm(), so convert
  // civil date -> days since epoch directly (Howard Hinnant's algorithm).
  int y = gps.date.year();
  unsigned m = gps.date.month();
  unsigned d = gps.date.day();
  y -= m <= 2;
  const int era = (y >= 0 ? y : y - 399) / 400;
  const unsigned yoe = (unsigned)(y - era * 400);
  const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  const long days = (long)era * 146097 + (long)doe - 719468;
  return (unsigned long)days * 86400UL + gps.time.hour() * 3600UL +
         gps.time.minute() * 60UL + gps.time.second();
}

void publishFix() {
  unsigned long ts = currentUnixTimestamp();
  unsigned long now = millis();

  String payload = "{";
  payload += "\"lat\":" + String(gps.location.lat(), 6) + ",";
  payload += "\"lon\":" + String(gps.location.lng(), 6) + ",";
  payload += "\"sats\":" + String(gps.satellites.isValid() ? gps.satellites.value() : 0) + ",";
  payload += "\"speed_kmh\":" + String(gps.speed.isValid() ? gps.speed.kmph() : 0.0, 2) + ",";
  payload += "\"ts\":" + String(ts);
  payload += "}";

  Serial.println(payload);
  firebasePut("/devices/" DEVICE_ID "/current", payload);

  // /current carries the real-time position; history and status are archival
  // and only need occasional writes — this is what frees the link for a
  // higher fix rate.
  if (ts != 0 && now - lastHistoryMs >= HISTORY_INTERVAL_MS) {
    lastHistoryMs = now;
    firebasePut("/devices/" DEVICE_ID "/history/" + String(ts), payload);
  }
  if (now - lastStatusMs >= STATUS_INTERVAL_MS) {
    lastStatusMs = now;
    String statusPayload = "{\"status\":\"ok\",\"sats\":" +
                            String(gps.satellites.isValid() ? gps.satellites.value() : 0) + "}";
    firebasePut("/devices/" DEVICE_ID "/status", statusPayload);
  }
}

void publishNoFixStatus() {
  // Distinguish "module not talking to us at all" (wiring/baud problem —
  // usually TX/RX swapped on NEO-6M breakouts) from "talking but no
  // satellite fix yet" (normal for the first minutes after cold start).
  bool gpsSilent = (gps.charsProcessed() < 10) && (millis() > GPS_DATA_TIMEOUT_MS);
  const char *status = gpsSilent ? "no_gps_data" : "no_fix";
  if (gpsSilent) {
    Serial.printf("No NMEA data from GPS module — check wiring "
                  "(GPS TX -> GPIO%d, GPS RX -> GPIO%d) and baud rate.\n",
                  GPS_RX_PIN, GPS_TX_PIN);
  }
  String statusPayload = "{\"status\":\"" + String(status) + "\",\"sats\":" +
                          String(gps.satellites.isValid() ? gps.satellites.value() : 0) + "}";
  Serial.println(statusPayload);
  firebasePut("/devices/" DEVICE_ID "/status", statusPayload);
}

void pollAlarmCommand() {
  String body = firebaseGet("/devices/" DEVICE_ID "/command");
  body.trim();
  if (body == "\"ALARM\"") {
    Serial.println("ALARM command received, beeping.");
    startAlarmBeeps();
    firebasePut("/devices/" DEVICE_ID "/command", "null");
  }
}

void setup() {
  Serial.begin(115200);
  // Bigger RX buffer: NMEA keeps streaming while a blocking HTTPS publish is
  // in flight, and the default 256 bytes would overflow at 5 Hz.
  Serial2.setRxBufferSize(2048);
  Serial2.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  // NEO-6M ships at 1 fix/s with the full NMEA sentence set. Keep only
  // GGA+RMC (everything TinyGPS++ needs) so 5 Hz fits within 9600 baud,
  // then raise the navigation rate to 5 Hz (UBX-CFG-RATE, measRate=200ms).
  // Settings are not persisted, so this runs on every boot.
  delay(200);
  Serial2.print(F("$PUBX,40,GLL,0,0,0,0*5C\r\n"));
  Serial2.print(F("$PUBX,40,GSV,0,0,0,0*59\r\n"));
  Serial2.print(F("$PUBX,40,GSA,0,0,0,0*4E\r\n"));
  Serial2.print(F("$PUBX,40,VTG,0,0,0,0*5E\r\n"));
  delay(100);
  static const uint8_t UBX_CFG_RATE_5HZ[] = {
      0xB5, 0x62, 0x06, 0x08, 0x06, 0x00,
      0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A};
  Serial2.write(UBX_CFG_RATE_5HZ, sizeof(UBX_CFG_RATE_5HZ));

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  Serial.println("ESP32 GPS Geofence firmware starting...");
  WiFi.mode(WIFI_STA);
  ensureWifiConnected();
}

void loop() {
  // Feed TinyGPS++ from the GPS module's UART.
  while (Serial2.available() > 0) {
    gps.encode(Serial2.read());
  }

  ensureWifiConnected();

  unsigned long now = millis();
  if (WiFi.status() == WL_CONNECTED) {
    if (gps.location.isValid()) {
      // Publish each fresh fix, throttled by REPORT_INTERVAL_MS.
      if (gps.location.isUpdated() && now - lastReportMs >= REPORT_INTERVAL_MS) {
        lastReportMs = now;
        publishFix();
      }
    } else if (now - lastStatusMs >= STATUS_INTERVAL_MS) {
      lastStatusMs = now;
      publishNoFixStatus();
    }

    if (now - lastCommandPollMs >= COMMAND_POLL_INTERVAL_MS) {
      lastCommandPollMs = now;
      pollAlarmCommand();
    }
  }

  serviceBuzzer();
}
