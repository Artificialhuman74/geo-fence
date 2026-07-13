"""GPS Geofence web dashboard.

Runs the Firebase listener (same job dashboard.py did headlessly) AND serves
a local web UI at http://127.0.0.1:8000 with a live map, click-to-place
geofence editor, event log, and a remote "ring buzzer" button.

The tracker and this laptop never talk directly — both talk to Firebase —
so the tracker can be any distance away as long as it has internet
(e.g. a phone hotspot travelling with it).

Usage:
    python server.py            # real mode: listens to Firebase
    python server.py --demo     # no Firebase/credentials needed: simulated
                                # tracker walks in and out of the fence
"""

import argparse
import json
import math
import os
import sys
import threading
import time
from collections import deque
from datetime import datetime, timezone

from flask import Flask, jsonify, render_template, request

import alarm
from dashboard import CONFIG_PATH, load_config, log_row
from geofence import GeofenceEvent, GeofenceState, KalmanTracker2D

METERS_PER_DEGREE_LAT = 111320.0


class AppState:
    """All mutable state shared between the fix-listener thread, the demo
    simulator thread, and Flask request handlers. Guarded by one lock."""

    def __init__(self, cfg: dict):
        self.lock = threading.Lock()
        self.cfg = cfg
        self.geofence = {
            "lat": cfg["home"]["lat"],
            "lon": cfg["home"]["lon"],
            "radius_m": cfg["radius_m"],
        }
        self._new_gf_state()

        self.latest = None          # last fix dict
        self.last_result = None     # last GeofenceResult
        self.device_status = None   # dict from /status (ok/no_fix/no_gps_data)
        self.trail = deque(maxlen=600)      # Kalman-filtered path (drawn bold)
        self.raw_trail = deque(maxlen=300)  # raw fixes (drawn faint, shows noise)
        self.events = deque(maxlen=60)
        # Kalman filter with outlier gating — smooths jitter and rejects
        # multipath jumps the way phone nav apps do.
        self.kalman = KalmanTracker2D()
        self.filtered = None        # filtered position (lat, lon)
        self.ellipse = None         # 95% error ellipse from filter covariance

        # set in real mode so alarms also ring the tracker's buzzer
        self.command_ref = None
        self.geofence_ref = None

    def _new_gf_state(self):
        self.gf_state = GeofenceState(
            home_lat=self.geofence["lat"],
            home_lon=self.geofence["lon"],
            radius_m=self.geofence["radius_m"],
            cooldown_seconds=self.cfg["geofence"]["alarm_cooldown_seconds"],
        )

    def set_geofence(self, lat: float, lon: float, radius_m: float):
        with self.lock:
            self.geofence = {"lat": lat, "lon": lon, "radius_m": radius_m}
            # Fresh state: the next fix re-evaluates inside/outside against
            # the new fence (and alarms immediately if it's outside it).
            self._new_gf_state()
            self._log_event("FENCE_SET",
                            f"center {lat:.5f},{lon:.5f} r={radius_m:.0f}m")
        # Resetting gf_state above means the next fix won't necessarily emit
        # an ENTER event even if the tracker is now inside the new fence, so
        # a siren left looping from the old fence wouldn't otherwise stop.
        alarm.stop_siren()
        if self.geofence_ref is not None:
            self.geofence_ref.set(self.geofence)  # persists across restarts

    def silence_alarm(self):
        """Manually stop a currently-sounding siren (e.g. the editor's Reset
        button) without needing to physically re-enter the fence."""
        with self.lock:
            self.gf_state.snooze()
            self._log_event("SILENCED", "alarm manually silenced")
        alarm.stop_siren()

    def _log_event(self, kind: str, detail: str):
        # caller holds the lock
        self.events.appendleft({
            "t": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "kind": kind,
            "detail": detail,
        })

    def process_fix(self, lat, lon, sats, speed_kmh, ts):
        """Ingest one fix; returns (event, distance) for alarm handling."""
        with self.lock:
            # The Kalman filter smooths jitter and rejects outlier fixes;
            # geofence decisions use the filtered position, gated by the
            # filter's own 95% uncertainty as the breach margin.
            est = self.kalman.process(lat, lon, time.time())
            flat, flon = est["lat"], est["lon"]
            self.filtered = (flat, flon)
            self.ellipse = dict(est["ellipse"], lat=flat, lon=flon)
            margin = est["ellipse"]["semi_major_m"]

            result = self.gf_state.update(flat, flon, margin_m=margin)
            self.latest = {
                "lat": lat, "lon": lon, "sats": sats,
                "speed_kmh": speed_kmh, "ts": ts,
                "received_at": time.time(),
            }
            self.last_result = result
            if not self.raw_trail or self.raw_trail[-1] != [lat, lon]:
                self.raw_trail.append([lat, lon])
            if not est["rejected"] and (
                    not self.trail or self.trail[-1] != [flat, flon]):
                self.trail.append([flat, flon])

            if result.event == GeofenceEvent.EXIT:
                self._log_event("BREACH", f"{result.distance_m:.0f}m from center")
            elif result.event == GeofenceEvent.REPEAT_ALARM:
                self._log_event("STILL_OUT", f"{result.distance_m:.0f}m from center")
            elif result.event == GeofenceEvent.ENTER:
                self._log_event("BACK_IN", f"{result.distance_m:.0f}m from center")
        return result

    def handle_alarm(self, result, lat, lon):
        """Sound/notify/CSV-log/ring-device for breach events. Called outside
        the lock — starting/stopping the siren thread must not block the
        Firebase listener."""
        if result.event == GeofenceEvent.ENTER:
            alarm.stop_siren()
            return
        if result.event not in (GeofenceEvent.EXIT, GeofenceEvent.REPEAT_ALARM):
            return

        now_str = datetime.now(timezone.utc).isoformat(timespec="seconds")
        tag = "EXIT" if result.event == GeofenceEvent.EXIT else "STILL_OUTSIDE"
        log_row(self.cfg["log_file"], now_str, lat, lon, result.distance_m, tag)
        if self.command_ref is not None:
            self.command_ref.set("ALARM")  # ESP32 polls this and beeps

        # Looping siren starts on EXIT and keeps playing for the whole
        # breach — REPEAT_ALARM just confirms it's still running. Skipped
        # on hosted deployments (SERVER_AUDIO=off): there the browser
        # siren and tracker buzzer are the alert channels.
        if _server_audio_enabled():
            alarm.start_siren(self.cfg["alarm"]["sound_file"])
            alarm.notify(
                title="Geofence Alert",
                message=f"Device is {result.distance_m:.0f}m from home — outside geofence!",
            )

    def snapshot(self) -> dict:
        with self.lock:
            return {
                "geofence": dict(self.geofence),
                "fix": dict(self.latest) if self.latest else None,
                "device_id": self.cfg["firebase"]["device_id"],
                "distance_m": self.last_result.distance_m if self.last_result else None,
                "is_outside": self.last_result.is_outside if self.last_result else None,
                "filtered": list(self.filtered) if self.filtered else None,
                "ellipse": dict(self.ellipse) if self.ellipse else None,
                "rejected_fixes": self.kalman.rejected_total,
                "raw_trail": list(self.raw_trail),
                "device_status": dict(self.device_status) if self.device_status else None,
                "trail": list(self.trail),
                "events": list(self.events),
                "now": time.time(),
            }


def load_runtime_config() -> dict:
    """config.yaml, with hosted-deployment overrides from the environment:
    FIREBASE_DB_URL, DEVICE_ID. Credentials come from
    FIREBASE_SERVICE_ACCOUNT (see start_firebase) so no secret ever needs
    to exist as a file in the deployed repo."""
    cfg = load_config(CONFIG_PATH)
    fb = cfg["firebase"]
    fb["database_url"] = os.environ.get("FIREBASE_DB_URL", fb["database_url"])
    fb["device_id"] = os.environ.get("DEVICE_ID", fb["device_id"])
    return cfg


def _server_audio_enabled() -> bool:
    # Sound on the *server* only makes sense when the server is the laptop
    # in front of you. Hosted deployments set SERVER_AUDIO=off; the browser
    # siren + tracker buzzer still alert.
    return os.environ.get("SERVER_AUDIO", "on") != "off"


# ---------------------------------------------------------------- real mode

def start_firebase(state: AppState):
    import firebase_admin
    from firebase_admin import credentials, db

    svc_json = os.environ.get("FIREBASE_SERVICE_ACCOUNT")
    if svc_json:
        cred = credentials.Certificate(json.loads(svc_json))
    else:
        path = state.cfg["firebase"]["service_account_path"]
        if not os.path.isfile(path):
            sys.exit(
                f"Firebase credentials not found: set the FIREBASE_SERVICE_ACCOUNT "
                f"env var (JSON string) or place the key file at '{path}'.")
        cred = credentials.Certificate(path)
    firebase_admin.initialize_app(
        cred, {"databaseURL": state.cfg["firebase"]["database_url"]})

    device = state.cfg["firebase"]["device_id"]
    state.command_ref = db.reference(f"/devices/{device}/command")
    state.geofence_ref = db.reference(f"/devices/{device}/geofence")

    # A geofence saved from the web UI outlives restarts; fall back to the
    # config.yaml home/radius if none was ever saved.
    saved = state.geofence_ref.get()
    if isinstance(saved, dict) and {"lat", "lon", "radius_m"} <= set(saved):
        with state.lock:
            state.geofence = {k: float(saved[k]) for k in ("lat", "lon", "radius_m")}
            state._new_gf_state()
        print(f"Loaded saved geofence: {state.geofence}")

    def on_current(event):
        data = event.data
        if not isinstance(data, dict) or "lat" not in data or "lon" not in data:
            return
        result = state.process_fix(
            data["lat"], data["lon"],
            data.get("sats", 0), data.get("speed_kmh", 0.0), data.get("ts", 0),
        )
        state.handle_alarm(result, data["lat"], data["lon"])

    def on_status(event):
        if isinstance(event.data, dict):
            with state.lock:
                state.device_status = event.data

    db.reference(f"/devices/{device}/current").listen(on_current)
    db.reference(f"/devices/{device}/status").listen(on_status)
    print(f"Listening on /devices/{device}/current ...")


# ---------------------------------------------------------------- demo mode

def start_demo(state: AppState):
    """Simulated tracker: orbits the fence center with GPS-like gaussian
    noise, drifting well outside the boundary and back roughly once a
    minute, so the whole UI/alarm/ellipse path can be tried with zero
    hardware or Firebase setup."""

    import random

    def run():
        t = 0.0
        while True:
            with state.lock:
                gf = dict(state.geofence)
            dist = gf["radius_m"] * (0.7 + 0.9 * math.sin(t / 28.0))
            bearing = t / 40.0
            noise_m = 3.5  # per-axis sigma, typical NEO-6M jitter
            north = dist * math.cos(bearing) + random.gauss(0.0, noise_m)
            east = dist * math.sin(bearing) + random.gauss(0.0, noise_m)
            if random.random() < 0.06:
                # occasional multipath-style outlier so the Kalman gate has
                # something to reject (shows up on the raw trail only)
                north += random.choice([-1, 1]) * random.uniform(30, 90)
                east += random.choice([-1, 1]) * random.uniform(30, 90)
            lat = gf["lat"] + north / METERS_PER_DEGREE_LAT
            lon = gf["lon"] + east / (
                METERS_PER_DEGREE_LAT * math.cos(math.radians(gf["lat"])))
            result = state.process_fix(lat, lon, sats=9, speed_kmh=3.5,
                                       ts=int(time.time()))
            with state.lock:
                state.device_status = {"status": "ok", "sats": 9}
            state.handle_alarm(result, lat, lon)
            t += 0.5
            time.sleep(0.5)

    threading.Thread(target=run, daemon=True, name="demo-tracker").start()
    print("DEMO MODE: simulated tracker running, no Firebase used.")


# -------------------------------------------------------------------- flask

def create_app(state: AppState) -> Flask:
    app = Flask(__name__)

    @app.get("/")
    def index():
        # The template is plain HTML (device id arrives via /api/state), so
        # the identical file also deploys as a static page on Netlify.
        return render_template("index.html")

    @app.get("/api/state")
    def api_state():
        return jsonify(state.snapshot())

    @app.post("/api/geofence")
    def api_geofence():
        body = request.get_json(force=True)
        try:
            lat = float(body["lat"])
            lon = float(body["lon"])
            radius_m = float(body["radius_m"])
        except (KeyError, TypeError, ValueError):
            return jsonify({"error": "need numeric lat, lon, radius_m"}), 400
        if not (-90 <= lat <= 90 and -180 <= lon <= 180 and 5 <= radius_m <= 50000):
            return jsonify({"error": "out of range (radius 5–50000 m)"}), 400
        state.set_geofence(lat, lon, radius_m)
        return jsonify({"ok": True})

    @app.post("/api/ring")
    def api_ring():
        if state.command_ref is not None:
            state.command_ref.set("ALARM")
        with state.lock:
            state._log_event("RING", "manual buzzer ring sent")
        return jsonify({"ok": True})

    @app.post("/api/silence")
    def api_silence():
        state.silence_alarm()
        return jsonify({"ok": True})

    return app


def create_wsgi_app() -> Flask:
    """Production entry point (Railway etc.):
        gunicorn -w 1 --threads 8 'server:create_wsgi_app()'
    Exactly one worker — tracker state lives in this process. Set DEMO=1
    to run the simulated tracker instead of Firebase."""
    cfg = load_runtime_config()
    state = AppState(cfg)
    if os.environ.get("DEMO") == "1":
        start_demo(state)
    else:
        start_firebase(state)
    return create_app(state)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--demo", action="store_true",
                        help="run with a simulated tracker (no Firebase)")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=int(os.environ.get("PORT", 8000)))
    args = parser.parse_args()

    cfg = load_runtime_config()
    state = AppState(cfg)

    if args.demo:
        start_demo(state)
    else:
        start_firebase(state)

    app = create_app(state)
    print(f"Dashboard: http://{args.host}:{args.port}")
    app.run(host=args.host, port=args.port, debug=False, use_reloader=False)


if __name__ == "__main__":
    main()
