# GPS Geofence Alarm

Track a NEO-6M GPS tag from anywhere, watch it on a laptop web dashboard,
set a geofence by clicking a map, and get alarms (laptop sound + the
tracker's own buzzer) when it leaves the fence.

## How it works — the tracker never talks to the laptop directly

```
 ESP32 + NEO-6M ──WiFi (phone hotspot)──▶ Firebase RTDB ◀──internet── laptop web dashboard
   (tracker)                               (cloud relay)                http://127.0.0.1:8000
```

Both sides talk to Firebase in the cloud, so the tracker can be any
distance from the laptop. The phone only provides a hotspot — no app runs
on it. (For connectivity without any phone, swap WiFi for a GSM module
like a SIM800L — not implemented here.)

Data paths in Firebase, per device:

| Path                          | Writer    | Reader  | Purpose                          |
|-------------------------------|-----------|---------|----------------------------------|
| `/devices/<id>/current`       | ESP32     | laptop  | latest fix (lat/lon/sats/speed)  |
| `/devices/<id>/history/<ts>`  | ESP32     | —       | fix archive                      |
| `/devices/<id>/status`        | ESP32     | laptop  | `ok` / `no_fix` / `no_gps_data`  |
| `/devices/<id>/command`       | laptop    | ESP32   | `"ALARM"` → tracker beeps        |
| `/devices/<id>/geofence`      | laptop    | laptop  | fence saved from the web UI      |

## 1. Firebase setup (once)

1. Create a Firebase project → Realtime Database.
2. For the ESP32: Project Settings → Service Accounts → Database Secrets
   (legacy) → copy the secret into `esp32-firmware/src/config.h`.
3. For the laptop: Project Settings → Service Accounts → Generate new
   private key → save as `laptop-dashboard/firebase-service-account.json`.
4. Put the database URL in both `config.h` (host only) and
   `laptop-dashboard/config.yaml`.

## 2. Tracker (ESP32 + NEO-6M)

Wiring: GPS VCC→5V, GND→GND, GPS TX→GPIO12, GPS RX→GPIO13, buzzer on
GPIO4. The module's micro-USB port is not used by the ESP32. (Pins are
set in `config.h` — GPS_RX_PIN/GPS_TX_PIN — adjust there if you rewire.)

Fill in WiFi hotspot SSID/password in `esp32-firmware/src/config.h`, then:

```sh
cd esp32-firmware
pio run -t upload
```

First satellite fix takes 1–5 minutes outdoors from cold start.

## 3. Laptop dashboard

```sh
cd laptop-dashboard
pip install -r requirements.txt
python server.py            # open http://127.0.0.1:8000
```

- Live map with the tracker (pulsing dot), its trail, and the fence circle.
- **Kalman-filtered position** (nav-app style): a constant-velocity Kalman
  filter smooths GPS jitter, and chi-square innovation gating rejects
  multipath outliers outright — statistically impossible jumps never move
  the marker or the geofence logic. The bold green trail is the filtered
  path; raw fixes are drawn as a faint dashed red trace behind it.
- **95% error ellipse** (dashed cyan) from the filter covariance. It doubles
  as the alarm margin: a breach only fires once the position is outside the
  fence by more than the current uncertainty, so noise near the boundary
  can't ring false alarms. Re-entry needs only distance ≤ radius
  (hysteresis).
- **Google Maps buttons** — open the tracker's position in Google Maps, or
  get turn-by-turn directions to it.
- Click the map to place a fence center, set the radius, **Save geofence**
  (persisted to Firebase, survives restarts).
- Breach → laptop sound + desktop notification + red banner in the browser
  (click *enable breach siren* once for in-browser audio) + the tracker's
  buzzer beeps via the `ALARM` command.
- **Ring tracker buzzer** button to find the tag / test the link.
- Event log + `geofence_log.csv` record every EXIT/ENTER.

### Try it with zero hardware

```sh
python server.py --demo     # simulated tracker orbits in and out of the fence
```

## 4. Hosted deployment (Railway backend + Netlify frontend)

The dashboard can run in the cloud so any browser can watch the tracker —
no laptop required.

**Backend → Railway.** Deploy this repo (the root `Procfile` +
`requirements.txt` drive the build). Set these variables on the service:

| Variable | Value |
|---|---|
| `FIREBASE_SERVICE_ACCOUNT` | the full service-account JSON, pasted as one line |
| `FIREBASE_DB_URL` | `https://<project>-default-rtdb.<region>.firebasedatabase.app` |
| `DEVICE_ID` | e.g. `freebot-01` |
| `SERVER_AUDIO` | `off` (no speakers in the cloud; the browser siren alerts instead) |

**Frontend → Netlify.** Import the same repo; `netlify.toml` builds the
static page and proxies `/api/*` to the Railway URL (edit the `to =` line
if the Railway domain changes). No CORS setup needed.

**Firebase rules.** With a public repo the database URL is public, so the
database must not be in test mode: apply `database.rules.json` (deny all
client access) in Firebase Console → Realtime Database → Rules. The ESP32
(legacy secret) and the server (service account) both bypass rules, so
they keep working.

Notes for hosted mode: the local sound/notification alarms are disabled
(`SERVER_AUDIO=off`) — breach alerts come from the browser banner/siren and
the tracker's own buzzer. The CSV log is ephemeral on Railway's filesystem.

`dashboard.py` is the old headless (no web UI) listener; `server.py`
replaces it and does everything it did.
