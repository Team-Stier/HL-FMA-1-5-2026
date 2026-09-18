"""Validate only the path source explicitly requested by State Manager."""

import math
import hashlib
import json
from dataclasses import dataclass
from typing import Dict, Optional, Sequence, Tuple


@dataclass(frozen=True)
class State:
    stamp: float
    received: float
    decision_id: int
    route_name: str
    path_mode: str
    direction: int
    valid: bool = True
    stop_requested: bool = False


@dataclass(frozen=True)
class Pose:
    frame_id: str
    position: Tuple[float, float, float]
    orientation: Tuple[float, float, float, float]


@dataclass(frozen=True)
class Candidate:
    stamp: float
    received: float
    decision_id: int
    route_name: str
    direction: int
    frame_id: str
    path_frame_id: str
    path_stamp: float
    poses: Sequence[Pose]


@dataclass(frozen=True)
class Selection:
    ready: bool
    reason: str
    source: str
    candidate: Optional[Candidate] = None


def path_fingerprint(candidate):
    """Bind approval to exact validated geometry and its mission request.

    Call only after SelectorCore has accepted geometry and freshness. Header and
    receipt times are independently validated and excluded here: refreshing an
    identical path should not invalidate a still-fresh geometric safety check.
    A replan, changed orientation, cropped endpoints or new request does change
    this fingerprint even if the path source remains the same.
    """
    data = [candidate.decision_id, candidate.route_name, candidate.direction, candidate.frame_id,
            candidate.path_frame_id,
            [[pose.frame_id, pose.position, pose.orientation] for pose in candidate.poses]]
    encoded = json.dumps(data, ensure_ascii=False, separators=(',', ':'), allow_nan=False).encode('utf-8')
    return hashlib.sha256(encoded).hexdigest()


def fresh(stamp, now, timeout, future_tolerance):
    return (math.isfinite(stamp) and math.isfinite(now) and stamp > 0
            and -future_tolerance <= now - stamp <= timeout)


class SelectorCore:
    SOURCES = {'RDDF': 'RDDF', 'LOCAL': 'LOCAL', 'PARKING': 'PARKING'}

    def __init__(self, timeout=0.5, future_tolerance=0.05,
                 frame_id='map', max_pose_spacing_m=2.0):
        if not (math.isfinite(timeout) and timeout > 0
                and math.isfinite(future_tolerance) and future_tolerance >= 0
                and math.isfinite(max_pose_spacing_m) and max_pose_spacing_m > 0):
            raise ValueError('Invalid selector validation limits')
        self.timeout = timeout
        self.future_tolerance = future_tolerance
        self.frame_id = frame_id
        self.max_pose_spacing_m = max_pose_spacing_m

    def _fresh(self, stamp, now):
        return fresh(stamp, now, self.timeout, self.future_tolerance)

    def evaluate(self, state: Optional[State], candidates: Dict[str, Candidate], now):
        source = state.path_mode if state else ''

        def reject(reason):
            return Selection(False, reason, source)

        if state is None:
            return reject('STATE_MISSING')
        if not self._fresh(state.stamp, now) or not self._fresh(state.received, now):
            return reject('STATE_STALE_OR_FUTURE')
        if source not in self.SOURCES:
            return reject('PATH_MODE_UNKNOWN')
        if not state.route_name or state.direction not in (-1, 1):
            return reject('STATE_ROUTE_INVALID')
        candidate = candidates.get(source)
        if candidate is None:
            return reject('REQUESTED_PATH_MISSING')
        if not all(self._fresh(stamp, now) for stamp in
                   (candidate.stamp, candidate.received, candidate.path_stamp)):
            return reject('PATH_STALE_OR_FUTURE')
        if (candidate.decision_id, candidate.route_name, candidate.direction) != (
                state.decision_id, state.route_name, state.direction):
            return reject('PATH_REQUEST_MISMATCH')
        if candidate.frame_id != self.frame_id or candidate.path_frame_id != self.frame_id:
            return reject('PATH_FRAME_INVALID')
        if len(candidate.poses) < 2:
            return reject('PATH_TOO_SHORT')
        previous = None
        length = 0.0
        for pose in candidate.poses:
            if pose.frame_id != self.frame_id:
                return reject('POSE_FRAME_INVALID')
            if (len(pose.position) != 3 or len(pose.orientation) != 4
                    or not all(math.isfinite(v) for v in pose.position + pose.orientation)):
                return reject('POSE_NONFINITE')
            norm_squared = sum(q * q for q in pose.orientation)
            if abs(norm_squared - 1.0) > 0.002:
                return reject('POSE_QUATERNION_INVALID')
            if previous is not None:
                dx, dy, dz = (b - a for a, b in zip(previous, pose.position))
                if math.hypot(dx, dy, dz) > self.max_pose_spacing_m:
                    return reject('PATH_DISCONTINUITY')
                step = math.hypot(dx, dy)
                length += step
            previous = pose.position
        if length < 0.001:
            return reject('PATH_DEGENERATE')
        # Readiness describes geometry even while the mission waits/stops. This
        # allows WAIT_PATH_READY to complete without a valid/ready cycle.
        return Selection(True, 'PATH_READY', source, candidate)
