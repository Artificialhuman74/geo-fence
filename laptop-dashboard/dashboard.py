"""Laptop dashboard: listens to Firebase for GPS readings, checks the
geofence, and triggers alarms on boundary crossings.
"""

import csv
import os
import sys
from datetime import datetime, timezone

import yaml
import firebase_admin
from firebase_admin import credentials, db

import alarm
from geofence import GeofenceEvent, GeofenceState

CONFIG_PATH = os.path.join(os.path.dirname(__file__), "config.yaml")


def load_config(path: str = CONFIG_PATH) -> dict:
    with open(path, "r") as f:
        return yaml.safe_load(f)


def init_firebase(cfg: dict):
    service_account_path = cfg["firebase"]["service_account_path"]
    if not os.path.isfile(service_account_path):
        sys.exit(
            f"Firebase service account key not found at '{service_account_path}'. "
            "Set firebase.service_account_path in config.yaml."
        )
    cred = credentials.Certificate(service_account_path)
    firebase_admin.initialize_app(cred, {"databaseURL": cfg["firebase"]["database_url"]})


def log_row(log_file: str, timestamp: str, lat: float, lon: float, distance_m: float, event: str) -> None:
    is_new = not os.path.isfile(log_file)
    with open(log_file, "a", newline="") as f:
        writer = csv.writer(f)
        if is_new:
            writer.writerow(["timestamp", "lat", "lon", "distance_m", "event"])
        writer.writerow([timestamp, f"{lat:.6f}", f"{lon:.6f}", f"{distance_m:.1f}", event])


def make_listener(cfg: dict, state: GeofenceState, command_ref):
    log_file = cfg["log_file"]
    sound_file = cfg["alarm"]["sound_file"]

    def on_update(event):
        data = event.data
        if not isinstance(data, dict) or "lat" not in data or "lon" not in data:
            return  # partial update or not a reading we understand

        lat, lon = data["lat"], data["lon"]
        result = state.update(lat, lon)
        now_str = datetime.now(timezone.utc).isoformat(timespec="seconds")
        status = "OUTSIDE" if result.is_outside else "inside"

        print(f"[{now_str}] connected | lat={lat:.6f} lon={lon:.6f} "
              f"distance={result.distance_m:.1f}m | {status}")

        if result.event == GeofenceEvent.EXIT:
            print("  -> GEOFENCE BREACH: alarm triggered")
            log_row(log_file, now_str, lat, lon, result.distance_m, "EXIT")
            alarm.trigger(sound_file, result.distance_m)
            command_ref.set("ALARM")  # ESP32 polls this and beeps its onboard buzzer
        elif result.event == GeofenceEvent.REPEAT_ALARM:
            print("  -> still outside: alarm repeated")
            log_row(log_file, now_str, lat, lon, result.distance_m, "STILL_OUTSIDE")
            alarm.trigger(sound_file, result.distance_m)
            command_ref.set("ALARM")
        elif result.event == GeofenceEvent.ENTER:
            print("  -> back inside geofence")
            log_row(log_file, now_str, lat, lon, result.distance_m, "ENTER")

    return on_update


def main():
    cfg = load_config()
    init_firebase(cfg)

    state = GeofenceState(
        home_lat=cfg["home"]["lat"],
        home_lon=cfg["home"]["lon"],
        radius_m=cfg["radius_m"],
        cooldown_seconds=cfg["geofence"]["alarm_cooldown_seconds"],
    )

    device_id = cfg["firebase"]["device_id"]
    ref = db.reference(f"/devices/{device_id}/current")
    command_ref = db.reference(f"/devices/{device_id}/command")

    print(f"Listening on /devices/{device_id}/current ...")
    print(f"Home: {cfg['home']['lat']}, {cfg['home']['lon']} | radius: {cfg['radius_m']}m")

    ref.listen(make_listener(cfg, state, command_ref))


if __name__ == "__main__":
    main()
