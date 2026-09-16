"""A command is eligible only if received while every dependency is healthy.

Inputs becoming stale, a transient invalid callback, a new route decision, or
a backwards clock invalidates that eligibility. Recovery requires a new raw
command; an old command cannot become live when a stop flag clears.
"""

import math
from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class Mission:
    stamp: float
    decision_id: int
    route_name: str
    path_mode: str
    direction: int
    speed_limit_mps: float
    stop_requested: bool = False
    valid: bool = True
    finished: bool = False


@dataclass(frozen=True)
class SafetyInput:
    stamp: float
    stop: bool
    sensor_valid: bool
    reason: str = ''
    clearance_m: float = 0.0
    path_fingerprint: str = ''


@dataclass(frozen=True)
class PathReady:
    stamp: float
    decision_id: int
    route_name: str
    source: str
    direction: int
    ready: bool
    path_fingerprint: str = ''


@dataclass(frozen=True)
class RawCommand:
    kph: int
    deg: int
    brake: int


@dataclass(frozen=True)
class GateDecision:
    kph: int = 0
    deg: int = 0
    brake: int = 1
    allowed: bool = False
    reason: str = 'INPUTS_MISSING'


class SafetyGate:
    def __init__(self, input_timeout=0.5, command_timeout=0.25,
                 future_tolerance=0.05, steering_limit_deg=25):
        if not all(math.isfinite(v) and v > 0 for v in (input_timeout, command_timeout)):
            raise ValueError('Timeouts must be finite and positive')
        if not math.isfinite(future_tolerance) or future_tolerance < 0:
            raise ValueError('Invalid future timestamp tolerance')
        if not 0 < steering_limit_deg <= 25:
            raise ValueError('Steering limit must be in (0, 25] degrees')
        self.input_timeout = input_timeout
        self.command_timeout = command_timeout
        self.future_tolerance = future_tolerance
        self.steering_limit_deg = int(steering_limit_deg)
        self.last_time = None
        self._reset()

    def _reset(self):
        self.mission = self.safety = self.path = self.localization = self.raw = None
        self.raw_eligible = False

    def _clock(self, now):
        if not math.isfinite(now) or (self.last_time is not None and now < self.last_time):
            self._reset()
        self.last_time = now

    def _fresh(self, stamp, now, timeout=None):
        return (math.isfinite(stamp) and math.isfinite(now) and stamp > 0
                and -self.future_tolerance <= now - stamp <= (
                    self.input_timeout if timeout is None else timeout))

    def _envelope_fresh(self, envelope, now):
        msg, receipt = envelope
        return self._fresh(msg.stamp, now) and self._fresh(receipt, now)

    @staticmethod
    def _mission_key(msg):
        # A moving braking cap is not a new route request. Positive valid caps
        # are applied at every evaluation, without pulsing stop between raw
        # controller callbacks. Zero/nonfinite caps still invalidate via guard.
        return (msg.decision_id, msg.route_name, msg.path_mode, msg.direction,
                msg.stop_requested, msg.valid, msg.finished)

    @staticmethod
    def _path_key(msg):
        return (msg.decision_id, msg.route_name, msg.source, msg.direction, msg.ready,
                msg.path_fingerprint)

    def _update(self, attribute, msg, now, key=None):
        self._clock(now)
        # Catch expired dependencies before replacing a stale envelope with a
        # fresh one. Otherwise an old raw command could be revived by refresh.
        if self._guard(now):
            self.raw_eligible = False
        previous = getattr(self, attribute)
        if key and (previous is None or key(previous[0]) != key(msg)):
            self.raw_eligible = False
        setattr(self, attribute, (msg, now))
        if self._guard(now):
            self.raw_eligible = False

    def update_mission(self, msg: Mission, now):
        self._update('mission', msg, now, self._mission_key)

    def update_safety(self, msg: SafetyInput, now):
        self._update('safety', msg, now)

    def update_path(self, msg: PathReady, now):
        self._update('path', msg, now, self._path_key)

    def update_localization(self, valid: bool, now):
        self._clock(now)
        if self._guard(now):
            self.raw_eligible = False
        self.localization = (bool(valid), now)
        if self._guard(now):
            self.raw_eligible = False

    def update_raw(self, msg: RawCommand, now):
        self._clock(now)
        self.raw = (msg, now)
        self.raw_eligible = self._guard(now) is None and self._raw_valid(msg)

    @staticmethod
    def _raw_valid(msg):
        return (all(type(v) is int for v in (msg.kph, msg.deg, msg.brake))
                and 0 <= msg.kph <= 65535 and -32768 <= msg.deg <= 32767
                and 0 <= msg.brake <= 255)

    def _guard(self, now) -> Optional[str]:
        if not math.isfinite(now) or now <= 0:
            return 'CLOCK_INVALID'
        if self.mission is None:
            return 'MISSION_MISSING'
        if not self._envelope_fresh(self.mission, now):
            return 'MISSION_STALE_OR_FUTURE'
        mission = self.mission[0]
        if not mission.valid:
            return 'MISSION_INVALID'
        if mission.finished:
            return 'MISSION_FINISHED'
        if mission.stop_requested:
            return 'MISSION_STOP'
        if mission.direction == -1:
            return 'REVERSE_INTERFACE_UNAVAILABLE'
        if mission.direction != 1 or not mission.route_name:
            return 'MISSION_ROUTE_INVALID'
        if mission.path_mode not in ('RDDF', 'LOCAL', 'PARKING'):
            return 'MISSION_PATH_MODE_INVALID'
        if not math.isfinite(mission.speed_limit_mps) or mission.speed_limit_mps < 0:
            return 'MISSION_SPEED_INVALID'
        if mission.speed_limit_mps * 3.6 < 1:
            return 'SPEED_LIMIT_BELOW_INTERFACE_RESOLUTION'
        if self.localization is None or not self._fresh(self.localization[1], now):
            return 'LOCALIZATION_MISSING_OR_STALE'
        if not self.localization[0]:
            return 'LOCALIZATION_INVALID'
        if self.safety is None or not self._envelope_fresh(self.safety, now):
            return 'SAFETY_MISSING_STALE_OR_FUTURE'
        if not self.safety[0].sensor_valid:
            return 'LIDAR_INVALID'
        if self.safety[0].stop:
            return 'LIDAR_STOP:' + self.safety[0].reason
        if self.path is None or not self._envelope_fresh(self.path, now):
            return 'PATH_STATUS_MISSING_STALE_OR_FUTURE'
        path = self.path[0]
        if not path.ready:
            return 'PATH_NOT_READY'
        if (path.decision_id, path.route_name, path.source, path.direction) != (
                mission.decision_id, mission.route_name, mission.path_mode, mission.direction):
            return 'PATH_REQUEST_MISMATCH'
        if not isinstance(path.path_fingerprint, str) or not path.path_fingerprint:
            return 'PATH_FINGERPRINT_MISSING'
        approved = self.safety[0].path_fingerprint
        if not isinstance(approved, str) or not approved:
            return 'SAFETY_PATH_FINGERPRINT_MISSING'
        if path.path_fingerprint != approved:
            return 'PATH_FINGERPRINT_MISMATCH'
        return None

    def evaluate(self, now):
        self._clock(now)
        reason = self._guard(now)
        if reason:
            self.raw_eligible = False
            return GateDecision(reason=reason)
        if self.raw is None or not self._fresh(self.raw[1], now, self.command_timeout):
            self.raw_eligible = False
            return GateDecision(reason='RAW_COMMAND_MISSING_OR_STALE')
        command = self.raw[0]
        if not self._raw_valid(command):
            self.raw_eligible = False
            return GateDecision(reason='RAW_COMMAND_INVALID')
        if not self.raw_eligible:
            return GateDecision(reason='FRESH_RAW_COMMAND_REQUIRED')
        if command.brake:
            return GateDecision(reason='CONTROLLER_STOP')
        maximum = int(math.floor(min(self.mission[0].speed_limit_mps, 65535 / 3.6) * 3.6))
        speed = min(command.kph, maximum)
        if speed == 0:
            return GateDecision(reason='ZERO_SPEED')
        steering = max(-self.steering_limit_deg, min(self.steering_limit_deg, command.deg))
        return GateDecision(speed, steering, 0, True, 'DRIVE_ALLOWED')
