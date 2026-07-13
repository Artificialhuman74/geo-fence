"""Writes simulated GPS readings to Firebase so the geofence dashboard can be
tested end-to-end without the ESP32/GPS hardware.

Usage:
    python fake_data_writer.py                 # walk from home to outside and back
    python fake_data_writer.py --stay-outside   # walk out and stay out (tests cooldown repeats)
"""

import argparse
import time
from datetime import datetime, timezone

import yaml
import firebase_admin
from firebase_admin import credentials, db

from dashboard import CONFIG_PATH

METERS_PER_DEGREE_LAT = 111320.0


def offset_lat(lat: float, meters: float) -> float:
    return lat + meters / METERS_PER_DEGREE_LAT


def push_reading(ref, lat: float, lon: float, sats: int = 8, speed_kmh: float = 0.0):
    ts = int(time.time())
    reading = {"lat": lat, "lon": lon, "sats": sats, "speed_kmh": speed_kmh, "ts": ts}
    ref.child("current").set(reading)
    ref.child("history").child(str(ts)).set(reading)
    print(f"[{datetime.now(timezone.utc).isoformat(timespec='seconds')}] wrote {reading}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stay-outside", action="store_true",
                         help="walk outside the radius and remain there (tests alarm cooldown)")
    parser.add_argument("--interval", type=float, default=2.0, help="seconds between writes")
    args = parser.parse_args()

    with open(CONFIG_PATH, "r") as f:
        cfg = yaml.safe_load(f)

    cred = credentials.Certificate(cfg["firebase"]["service_account_path"])
    firebase_admin.initialize_app(cred, {"databaseURL": cfg["firebase"]["database_url"]})
    ref = db.reference(f"/devices/{cfg['firebase']['device_id']}")

    home_lat = cfg["home"]["lat"]
    home_lon = cfg["home"]["lon"]
    radius_m = cfg["radius_m"]

    # Step distances (meters north of home) chosen to straddle the radius.
    inside_offset = radius_m * 0.4
    outside_offset = radius_m * 1.8

    steps = [inside_offset, outside_offset]
    if not args.stay_outside:
        steps += [outside_offset, inside_offset]
    else:
        steps += [outside_offset, outside_offset, outside_offset]

    for offset in steps:
        push_reading(ref, offset_lat(home_lat, offset), home_lon)
        time.sleep(args.interval)


if __name__ == "__main__":
    main()
