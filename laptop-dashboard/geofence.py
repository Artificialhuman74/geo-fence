"""Pure geofence math, GPS filtering, and state-transition logic (no I/O,
easy to test)."""

import math
import time
from dataclasses import dataclass
from enum import Enum, auto

import numpy as np

EARTH_RADIUS_M = 6371000.0
METERS_PER_DEGREE_LAT = 111320.0
# 95% confidence for a 2-DOF chi-square distribution — scales the covariance
# eigenvalues into the ellipse semi-axes.
CHI2_95_2DOF = 5.991


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle distance between two lat/lon points, in meters."""
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    d_phi = math.radians(lat2 - lat1)
    d_lambda = math.radians(lon2 - lon1)

    a = (
        math.sin(d_phi / 2) ** 2
        + math.cos(phi1) * math.cos(phi2) * math.sin(d_lambda / 2) ** 2
    )
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return EARTH_RADIUS_M * c


def _ellipse_from_cov(pxx: float, pyy: float, pxy: float) -> dict:
    """95% confidence ellipse of a 2x2 covariance (east/north, m²), via
    analytic eigen-decomposition. angle_deg is the semi-major axis
    direction, counterclockwise from east."""
    half_trace = (pxx + pyy) / 2.0
    root = math.sqrt(max(0.0, ((pxx - pyy) / 2.0) ** 2 + pxy * pxy))
    return {
        "semi_major_m": math.sqrt(CHI2_95_2DOF * (half_trace + root)),
        "semi_minor_m": math.sqrt(CHI2_95_2DOF * max(0.0, half_trace - root)),
        "angle_deg": math.degrees(0.5 * math.atan2(2.0 * pxy, pxx - pyy)),
    }


class KalmanTracker2D:
    """Constant-velocity Kalman filter over GPS fixes with innovation
    gating — the same core technique phone nav apps use (minus road
    snapping, which needs map data).

    Three behaviours fall out of it:
    - smoothing: each fix is blended with where the motion model predicted
      the tracker should be, so jitter cancels instead of drawing zigzags;
    - outlier rejection: a fix whose normalized innovation exceeds the
      chi-square gate (a statistically impossible jump, e.g. multipath
      bounce) is discarded outright — position coasts on the prediction;
    - honest uncertainty: the filter covariance gives the 95% error
      ellipse directly, and it grows while fixes are being rejected.

    State is [east_m, north_m, v_east, v_north] in a local tangent plane
    anchored at the first fix. If MAX_CONSECUTIVE_REJECTS fixes in a row
    fail the gate, the jump is treated as real (vehicle, long outage) and
    the track restarts at the new position.
    """

    GATE_NIS = 13.82              # chi-square 2 DOF @ 99.9%
    MAX_CONSECUTIVE_REJECTS = 5
    ACCEL_PSD = 1.0               # (m/s²)² process noise: pedestrian/slow vehicle
    MEAS_SIGMA_M = 5.0            # typical NEO-6M horizontal noise, 1 sigma
    MAX_DT_S = 10.0

    def __init__(self):
        self._origin = None       # (lat, lon) tangent-plane anchor
        self._x = None            # state vector
        self._P = None            # state covariance
        self._t = None
        self._rejects = 0
        self.rejected_total = 0

    def _to_m(self, lat: float, lon: float):
        lat0, lon0 = self._origin
        k_lon = METERS_PER_DEGREE_LAT * math.cos(math.radians(lat0))
        return np.array([(lon - lon0) * k_lon,
                         (lat - lat0) * METERS_PER_DEGREE_LAT])

    def _to_latlon(self, east: float, north: float):
        lat0, lon0 = self._origin
        k_lon = METERS_PER_DEGREE_LAT * math.cos(math.radians(lat0))
        return (lat0 + north / METERS_PER_DEGREE_LAT, lon0 + east / k_lon)

    def _reset_at(self, z):
        self._x = np.array([z[0], z[1], 0.0, 0.0])
        # position known to measurement accuracy; velocity unknown (~5 m/s std)
        self._P = np.diag([self.MEAS_SIGMA_M ** 2, self.MEAS_SIGMA_M ** 2,
                           25.0, 25.0])

    def process(self, lat: float, lon: float, t: float) -> dict:
        """Ingest one raw fix at wall time t (seconds). Returns the filtered
        position, whether this fix was rejected as an outlier, the 95%
        error ellipse, and the estimated speed."""
        if self._origin is None:
            self._origin = (lat, lon)
        z = self._to_m(lat, lon)
        if self._x is None:
            self._reset_at(z)
            self._t = t
            return self._result(rejected=False)

        dt = min(max(t - self._t, 1e-3), self.MAX_DT_S)
        self._t = t

        # predict
        F = np.eye(4)
        F[0, 2] = F[1, 3] = dt
        dt2, dt3 = dt * dt, dt * dt * dt
        q = self.ACCEL_PSD
        Q = q * np.array([[dt3 / 3, 0, dt2 / 2, 0],
                          [0, dt3 / 3, 0, dt2 / 2],
                          [dt2 / 2, 0, dt, 0],
                          [0, dt2 / 2, 0, dt]])
        x = F @ self._x
        P = F @ self._P @ F.T + Q

        # gate + update
        H = np.array([[1.0, 0, 0, 0], [0, 1.0, 0, 0]])
        R = np.eye(2) * self.MEAS_SIGMA_M ** 2
        y = z - H @ x
        S = H @ P @ H.T + R
        nis = float(y @ np.linalg.solve(S, y))

        if nis > self.GATE_NIS and self._rejects < self.MAX_CONSECUTIVE_REJECTS:
            self._rejects += 1
            self.rejected_total += 1
            self._x, self._P = x, P  # coast on the prediction; P keeps growing
            return self._result(rejected=True)

        if nis > self.GATE_NIS:
            self._reset_at(z)  # persistent disagreement: the jump was real
        else:
            K = P @ H.T @ np.linalg.inv(S)
            self._x = x + K @ y
            self._P = (np.eye(4) - K @ H) @ P
        self._rejects = 0
        return self._result(rejected=False)

    def _result(self, rejected: bool) -> dict:
        lat, lon = self._to_latlon(self._x[0], self._x[1])
        Ppos = self._P[:2, :2]
        return {
            "lat": lat,
            "lon": lon,
            "rejected": rejected,
            "ellipse": _ellipse_from_cov(Ppos[0, 0], Ppos[1, 1], Ppos[0, 1]),
            "speed_ms": float(math.hypot(self._x[2], self._x[3])),
        }


class GeofenceEvent(Enum):
    INSIDE = auto()       # still inside, nothing to do
    EXIT = auto()         # just crossed inside -> outside: alarm
    STILL_OUTSIDE = auto()  # outside, cooldown not yet elapsed: no alarm
    REPEAT_ALARM = auto()   # outside, cooldown elapsed: alarm again
    ENTER = auto()         # just crossed outside -> inside: log only, no alarm


@dataclass
class GeofenceResult:
    distance_m: float
    is_outside: bool
    event: GeofenceEvent


class GeofenceState:
    """Tracks inside/outside transitions and alarm cooldown across updates."""

    def __init__(self, home_lat: float, home_lon: float, radius_m: float,
                 cooldown_seconds: float, clock=time.monotonic):
        self.home_lat = home_lat
        self.home_lon = home_lon
        self.radius_m = radius_m
        self.cooldown_seconds = cooldown_seconds
        self._clock = clock

        self._was_outside = False
        self._last_alarm_at = None

    def snooze(self) -> None:
        """Manually silence an active alarm without waiting for re-entry.
        Stays flagged as outside (so re-entry still logs ENTER correctly),
        but restarts the cooldown clock so REPEAT_ALARM doesn't fire again
        for a fresh cooldown_seconds window."""
        if self._was_outside:
            self._last_alarm_at = self._clock()

    def update(self, lat: float, lon: float, margin_m: float = 0.0) -> GeofenceResult:
        """margin_m widens the exit threshold (radius + margin) while inside,
        but re-entry still only needs distance <= radius. The asymmetry gives
        hysteresis: a position must be outside by more than its measurement
        uncertainty before a breach fires, so GPS jitter near the boundary
        can't ring the alarm."""
        distance = haversine_m(self.home_lat, self.home_lon, lat, lon)
        threshold = self.radius_m + (0.0 if self._was_outside else margin_m)
        is_outside = distance > threshold
        now = self._clock()

        if is_outside:
            if not self._was_outside:
                event = GeofenceEvent.EXIT
                self._last_alarm_at = now
            elif self._last_alarm_at is None or (now - self._last_alarm_at) >= self.cooldown_seconds:
                event = GeofenceEvent.REPEAT_ALARM
                self._last_alarm_at = now
            else:
                event = GeofenceEvent.STILL_OUTSIDE
            self._was_outside = True
        else:
            event = GeofenceEvent.ENTER if self._was_outside else GeofenceEvent.INSIDE
            self._was_outside = False
            self._last_alarm_at = None

        return GeofenceResult(distance_m=distance, is_outside=is_outside, event=event)
