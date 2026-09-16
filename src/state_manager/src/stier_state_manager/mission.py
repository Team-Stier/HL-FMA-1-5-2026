"""Deterministic mission decisions; no ROS, vehicle commands, or geometry I/O.

Distances are metres along the active route. ``s`` may be monotonic tracker
progress; ``raw_s`` must retain backwards motion for the hill rollback check.
Landmarks refer to the tracking reference unless a documented axle/bumper
offset is configured. The ROS adapter is responsible for timestamped perception,
collision checks, matching planner acknowledgements, and applying stop requests.
"""

import math


RULE_DEFAULTS = {
    "standstill_speed_mps": 0.05,
    "hill_hold_s": 3.0,
    "hill_hold_position_tolerance_m": 0.02,
    "parking_hold_s": 0.5,
    "sensor_timeout_s": 0.5,
    "max_update_gap_s": 0.5,
    "parking_stable_observations": 3,
    "stop_tolerance_m": 0.2,
    "front_bumper_offset_m": 0.0,
    "rear_axle_offset_m": 0.0,
    "finish_clearance_m": 0.5,
    "hill_rollback_limit_m": 0.5,
    "hill_clearance_timeout_s": 30.0,
    "mission_deadline_s": 480.0,
    "no_motion_timeout_s": 60.0,
    "intersection_stop_penalty_s": 3.0,
    "intersection_stop_timeout_s": 20.0,
    "intersection_clearance_timeout_s": 30.0,
    "parking_yaw_tolerance_rad": math.radians(10.0),
}

SPEED_DEFAULTS = {
    "normal": 2.0, "hill": 1.0, "static": 1.0,
    "intersection": 1.0, "parking": 0.5,
}

ROUTES = {
    2: "2", 3: "3_s-static-obstacle", 4: "4", 7: "7",
    8: "8_dynamic-obstacle", 9: "9", 12: "12",
}
PARKING_ROUTES = {
    "t": {
        "left": ("5_T-left-in", "6-T-left-out"),
        "right": ("5_T-right-in", "6_T-right-out"),
    },
    "parallel": {
        "left": ("10_parallel-left-in", "11-parallel-left-out"),
        "right": ("10_parallel-right-in", "11_parallel-right-out"),
    },
}
REQUIRED_LANDMARKS = {
    1: ("hill_start_s", "hill_stop_s", "hill_top_s"),
    2: ("stop_line_s", "intersection_exit_s"),
    4: ("stop_line_s", "intersection_exit_s"),
    5: ("parking_confirm_s",), 6: ("parking_exit_s",),
    7: ("stop_line_s", "intersection_exit_s"),
    10: ("parking_confirm_s",), 11: ("parking_exit_s",),
    12: ("finish_branch_s",), 13: ("finish_s",),
}


def _number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


HILL_ZONE_KEYS = ("hill_zone_start_s", "hill_zone_end_s")


def landmark_keys(section, marks):
    """A measured stop-zone pair replaces the legacy ramp/target triple."""
    if section == 1 and any(marks.get(k) is not None for k in HILL_ZONE_KEYS):
        return HILL_ZONE_KEYS
    return REQUIRED_LANDMARKS.get(section, ())


def hill_target(marks):
    """Return allowed zone edges and the rear-axle target, in route arc length.

    Zone markers already delimit the permitted stop area: do not inset them
    by another metre. Legacy ramp boundaries retain their one-metre insets.
    """
    if landmark_keys(1, marks) == HILL_ZONE_KEYS:
        a, b = (marks.get(k) for k in HILL_ZONE_KEYS)
        if not _number(a) or not _number(b) or a >= b:
            raise ValueError("HILL_ZONE_MARKERS_INVALID")
        return a, (a + b) / 2.0, b
    a, target, b = (marks.get(k) for k in REQUIRED_LANDMARKS[1])
    if not all(_number(v) for v in (a, target, b)) or not a + 1 <= target <= b - 1:
        raise ValueError("HILL_STOP_OUTSIDE_RULE_ZONE")
    return a + 1, target, b - 1


def _branch(route):
    if "left" in route:
        return "left"
    if "right" in route:
        return "right"
    return None


class MissionEngine:
    """Stateful rule engine driven by explicit, replayable input snapshots.

    ``next_route`` is an authorization for the route tracker, never an implicit
    route change. Invalid data clears continuous dwell evidence and suppresses
    transitions. Completed mission timestamps and committed branch choices are
    retained on re-entry; construct a new engine to begin a new competition run.
    """

    def __init__(self, config=None):
        self.config = config or {}
        self.rules = dict(RULE_DEFAULTS)
        self.rules.update(self.config.get("rules", {}))
        # Configuration may be more conservative than the regulations; it may
        # not shorten mandated stops or relax maximum rollback/deadline values.
        for name, value in self.rules.items():
            if name in RULE_DEFAULTS and (not _number(value) or value < 0):
                if name not in ("front_bumper_offset_m", "rear_axle_offset_m") or not _number(value):
                    raise ValueError("invalid mission rule: " + name)
        if self.rules["hill_hold_s"] < 3:
            raise ValueError("hill hold must be at least 3 seconds")
        if not 0 < self.rules["hill_rollback_limit_m"] <= 0.5:
            raise ValueError("hill rollback limit must be in (0, 0.5] metres")
        if not 0 < self.rules["hill_clearance_timeout_s"] <= 30:
            raise ValueError("hill clearance timeout must be in (0, 30] seconds")
        if not 0 < self.rules["mission_deadline_s"] <= 480:
            raise ValueError("mission deadline must be in (0, 480] seconds")
        if not 0 < self.rules["no_motion_timeout_s"] <= 60:
            raise ValueError("no-motion timeout must be in (0, 60] seconds")
        for name, maximum in (("intersection_stop_penalty_s", 3),
                              ("intersection_stop_timeout_s", 20),
                              ("intersection_clearance_timeout_s", 30)):
            if not 0 < self.rules[name] <= maximum:
                raise ValueError("invalid intersection rule: " + name)
        for name in ("sensor_timeout_s", "max_update_gap_s", "parking_stable_observations",
                     "finish_clearance_m", "hill_hold_position_tolerance_m"):
            if self.rules[name] <= 0:
                raise ValueError("mission rule must be positive: " + name)
        for name in ("parking_stable_observations",):
            if int(self.rules[name]) != self.rules[name]:
                raise ValueError("observation counts must be integers")
        self.finish_branch = self.config.get("finish_branch", "left")
        if self.finish_branch not in ("left", "right"):
            raise ValueError("finish_branch must be left or right")
        self.speeds = dict(SPEED_DEFAULTS)
        self.speeds.update(self.config.get("speeds", {}))
        if any(not _number(value) or value <= 0 for value in self.speeds.values()):
            raise ValueError("mission speed limits must be positive finite numbers")
        self.states = {}
        self.branches = {}
        self.committed_branches = set()
        self.completed_missions = {}
        self.counters = {}
        self._reported = set()
        self._candidate_evidence = {}
        self._start_time = None
        self._last_time = None
        self._last_moving_time = None
        self._last_route = None
        self._last_red_wait = None
        self._excluded_signal_wait_s = 0.0

    def _once(self, name, key):
        token = (name, key)
        if token not in self._reported:
            self._reported.add(token)
            self.counters[name] = self.counters.get(name, 0) + 1

    def _fresh(self, observation, now, route=None):
        if not isinstance(observation, dict):
            return False
        stamp = observation.get("stamp")
        if not _number(stamp):
            return False
        age = now - stamp
        if age < 0.0 or age > float(self.rules["sensor_timeout_s"]):
            return False
        return route is None or observation.get("route") == route

    def _reset_continuous(self):
        for state in self.states.values():
            state["dwell_since"] = None
            state.pop("hill_hold_anchor", None)
            state["intersection_stop_since"] = None
        self._candidate_evidence.clear()

    def _complete(self, key, now):
        self.completed_missions.setdefault(key, now)

    def _stop(self, out, reason, phase=None):
        out["stop_requested"] = True
        out["speed_limit"] = 0.0
        out["reason"] = reason
        out["next_route"] = None
        if phase:
            out["phase"] = phase

    def _next(self, out, route):
        out["next_route"] = self.config.get("transitions", {}).get(out["route"], route)

    def _dwell(self, state, now, condition, duration):
        if not condition:
            state["dwell_since"] = None
            return False
        if state.get("dwell_since") is None:
            state["dwell_since"] = now
        return now - state["dwell_since"] >= duration

    def _landmarks(self, snapshot, section, length):
        landmarks = snapshot.get("landmarks", {})
        if not isinstance(landmarks, dict):
            return None, "LANDMARKS_MISSING"
        # The public contract is route -> fields; a flat current-route mapping
        # is accepted for adapters that already resolve the active route.
        if snapshot["route"] in landmarks:
            landmarks = landmarks[snapshot["route"]]
        if not isinstance(landmarks, dict):
            return None, "LANDMARKS_MISSING"
        for key in landmark_keys(section, landmarks):
            if not _number(landmarks.get(key)):
                return None, "CALIBRATION_REQUIRED:" + key
            if landmarks[key] < 0 or landmarks[key] > length:
                return None, "LANDMARK_OUTSIDE_ROUTE:" + key
        if section == 1:
            try:
                hill_target(landmarks)
            except ValueError as error:
                return None, str(error)
        if section in (2, 4, 7):
            if landmarks["stop_line_s"] >= landmarks["intersection_exit_s"]:
                return None, "INTERSECTION_LANDMARK_ORDER_INVALID"
        return landmarks, None

    def _base(self, snapshot):
        route = str(snapshot.get("route", ""))
        section = snapshot.get("section", 0)
        mission = {
            1: "HILL_STOP", 2: "TRAFFIC_STRAIGHT", 3: "STATIC_AVOIDANCE",
            4: "TRAFFIC_STRAIGHT", 5: "T_PARKING", 6: "T_PARKING",
            7: "TRAFFIC_LEFT", 8: "RDDF_TRANSIT", 9: "PARALLEL_APPROACH",
            10: "PARALLEL_PARKING", 11: "PARALLEL_PARKING",
            12: "FINISH_APPROACH", 13: "FINISH",
        }.get(section, "UNKNOWN")
        speed_name = "normal"
        if section == 1:
            speed_name = "hill"
        elif section in (2, 4, 7):
            speed_name = "intersection"
        elif section == 3:
            speed_name = "static"
        elif section in (5, 6, 10, 11):
            speed_name = "parking"
        mode = "LOCAL" if section == 3 else "PARKING" if section in (5, 6, 10, 11) else "RDDF"
        return {
            "route": route, "section": section, "mission": mission,
            "phase": "APPROACH", "selected_branch": _branch(route),
            "branch": _branch(route), "path_mode": mode,
            "stop_requested": False, "speed_limit": float(self.speeds[speed_name]),
            "direction": self.parking_leg(route).get("direction", 1),
            "reason": "", "next_route": None, "remaining_stop_m": None,
            "counters": {}, "diagnostics": [], "completed_missions": {},
            "parking_candidates": {}, "virtual_stop": None,
            "hill_target_s": None, "hill_hold_elapsed_s": 0.0,
            "parking_leg_index": self.parking_leg(route).get("leg_index", -1),
            "parking_leg_phase": self.parking_leg(route).get("phase", ""),
            "parking_leg_target_s": self.parking_leg(route).get("target_s"),
        }

    def _finish_output(self, out):
        out["branch"] = out["selected_branch"]
        out["counters"] = dict(self.counters)
        out["completed_missions"] = dict(self.completed_missions)
        out["excluded_signal_wait_s"] = self._excluded_signal_wait_s
        out["run_started"] = self._start_time is not None
        out["elapsed_time_s"] = 0.0
        if self._start_time is not None and self._last_time is not None:
            clock_end = self.completed_missions.get("finish", self._last_time)
            out["elapsed_time_s"] = max(0.0, clock_end - self._start_time - self._excluded_signal_wait_s)
        return out

    def _qualifying_red_wait(self, snap):
        """Only observed red-signal waiting within 1 m earns excluded time."""
        if (snap.get("section") not in (2, 4, 7) or not snap.get("healthy")
                or not snap.get("calibrated") or not _number(snap.get("speed"))
                or abs(snap["speed"]) > self.rules["standstill_speed_mps"]
                or not _number(snap.get("s")) or not _number(snap.get("length"))
                or not self._fresh(snap.get("signal"), snap["now"], snap.get("route"))
                or snap["signal"].get("value") != "RED"):
            return False
        marks, error = self._landmarks(snap, snap["section"], snap["length"])
        if error:
            return False
        distance = marks["stop_line_s"] - snap["s"] - self.rules["front_bumper_offset_m"]
        return 0.0 <= distance <= 1.0

    def update(self, snapshot):
        """Return a decision without mutating the caller's snapshot."""
        out = self._base(snapshot)
        now = snapshot.get("now")
        if not _number(now):
            self._reset_continuous()
            self._stop(out, "INVALID_CLOCK", "UNAVAILABLE")
            return self._finish_output(out)
        if self._last_time is not None and now < self._last_time:
            self._reset_continuous()
            self._once("clock_regression", self._last_time)
            self._stop(out, "CLOCK_REGRESSION", "UNAVAILABLE")
            return self._finish_output(out)
        previous_time = self._last_time
        if previous_time is not None and now - previous_time > self.rules["max_update_gap_s"]:
            self._reset_continuous()
        self._last_time = now
        speed = snapshot.get("speed")
        moving = _number(speed) and abs(speed) > self.rules["standstill_speed_mps"]
        # Daemon startup, calibration, and localization acquisition are not
        # competition driving time. A future stamped start-command adapter may
        # explicitly set run_started; it still requires valid calibrated input.
        start_requested = moving or snapshot.get("run_started") is True
        if (self._start_time is None and start_requested and snapshot.get("healthy")
                and snapshot.get("calibrated")):
            self._start_time = now
            self._last_moving_time = now
        red_wait = self._start_time is not None and self._qualifying_red_wait(snapshot)
        if (red_wait and self._last_red_wait == snapshot.get("route") and previous_time is not None
                and 0 <= now - previous_time <= self.rules["max_update_gap_s"]):
            self._excluded_signal_wait_s += now - previous_time
        self._last_red_wait = snapshot.get("route") if red_wait else None
        if self._start_time is not None and "finish" not in self.completed_missions and (red_wait or moving):
            self._last_moving_time = now
        clock_end = self.completed_missions.get("finish", now)
        if (self._start_time is not None
                and clock_end - self._start_time - self._excluded_signal_wait_s > self.rules["mission_deadline_s"]):
            self._once("mission_deadline_exceeded", "run")
            out["diagnostics"].append("MISSION_DEADLINE_EXCEEDED")
        if ("finish" not in self.completed_missions and self._last_moving_time is not None
                and now - self._last_moving_time >= self.rules["no_motion_timeout_s"]):
            self._once("no_motion_timeout", self._last_moving_time)
            out["diagnostics"].append("NO_MOTION_TIMEOUT")
        if not snapshot.get("healthy", False):
            self._reset_continuous()
            self._stop(out, snapshot.get("reason") or "LOCALIZATION_OR_SENSORS_INVALID", "UNAVAILABLE")
            return self._finish_output(out)
        if snapshot.get("collision", False):
            self._reset_continuous()
            self._stop(out, "COLLISION_STOP", "SAFETY_STOP")
            return self._finish_output(out)
        route, section = out["route"], out["section"]
        if (not route or section not in range(1, 14)
                or not _number(speed) or not _number(snapshot.get("s"))
                or not _number(snapshot.get("length")) or snapshot["length"] <= 0
                or not _number(snapshot.get("yaw"))):
            self._reset_continuous()
            self._stop(out, "INVALID_ROUTE_OR_VEHICLE_STATE", "UNAVAILABLE")
            return self._finish_output(out)
        if not snapshot.get("calibrated", False):
            self._reset_continuous()
            self._stop(out, "CALIBRATION_REQUIRED", "UNAVAILABLE")
            return self._finish_output(out)
        landmarks, error = self._landmarks(snapshot, section, snapshot["length"])
        if error:
            self._reset_continuous()
            self._stop(out, error, "UNAVAILABLE")
            return self._finish_output(out)
        state = self.states.setdefault(route, {"dwell_since": None})
        if self._last_route != route:
            state["dwell_since"] = None
            self._last_route = route
        standing = abs(speed) <= self.rules["standstill_speed_mps"]
        if section == 1:
            self._hill(snapshot, landmarks, state, out, standing)
        elif section in (2, 4, 7):
            self._traffic(snapshot, landmarks, state, out)
            if section == 4:
                self._parking_preview(snapshot, out, "t")
                if snapshot.get("at_end") and state.get("crossed") and not out["stop_requested"]:
                    self._parking_handoff(snapshot, out, "t")
        elif section == 3:
            out["phase"] = "AVOIDING"
            if snapshot.get("at_end"):
                self._complete("static", now)
                out["phase"] = "COMPLETE"
                self._next(out, ROUTES[4])
        elif section in (5, 6, 10, 11):
            self._parking(snapshot, landmarks, state, out, standing)
        elif section == 8:
            out["phase"] = "FOLLOW_RDDF"
            if snapshot.get("at_end"):
                self._complete("section_8_transit", now)
                out["phase"] = "COMPLETE"
                self._next(out, ROUTES[9])
        elif section == 9:
            self._parking_preview(snapshot, out, "parallel")
            if snapshot.get("at_end"):
                self._parking_handoff(snapshot, out, "parallel")
        elif section == 12:
            self._finish_approach(snapshot, landmarks, out)
        elif section == 13:
            self._finish(snapshot, landmarks, out)
        if not snapshot.get("path_ready", False) and not out["stop_requested"]:
            self._stop(out, "REQUESTED_PATH_UNAVAILABLE", "WAIT_PATH")
        return self._finish_output(out)

    def _hill(self, snap, marks, state, out, standing):
        now, s = snap["now"], snap["s"]
        raw_s = snap.get("raw_s", s)
        if not _number(raw_s):
            self._stop(out, "HILL_RAW_PROGRESS_INVALID", "UNAVAILABLE")
            return
        zone_start, stop, zone_end = hill_target(marks)
        paired = landmark_keys(1, marks) == HILL_ZONE_KEYS
        start = zone_start if paired else marks["hill_start_s"]
        top = zone_end if paired else marks["hill_top_s"]
        out["hill_target_s"] = stop
        zone_ok = zone_start <= raw_s <= zone_end
        near_stop = abs(raw_s - stop) <= self.rules["stop_tolerance_m"]
        if standing and zone_ok and near_stop:
            state.setdefault("first_stop_time", now)
        if (not state.get("hill_cleared") and state.get("first_stop_time") is not None
                and now - state["first_stop_time"] > self.rules["hill_clearance_timeout_s"]):
            self._once("hill_clearance_timeout", out["route"])
            out["diagnostics"].append("HILL_CLEARANCE_TIMEOUT")
        if start <= raw_s <= top and state.get("hill_entered") is None:
            state["hill_entered"] = now
        if state.get("hill_entered") is not None and not state.get("hill_cleared"):
            state["furthest_raw_s"] = max(state.get("furthest_raw_s", raw_s), raw_s)
            if state["furthest_raw_s"] - raw_s >= self.rules["hill_rollback_limit_m"]:
                state["rollback_fault"] = True
                self._once("hill_rollback", out["route"])
            if state.get("rollback_fault"):
                self._stop(out, "HILL_ROLLBACK_LIMIT_EXCEEDED", "FAULT")
                return
        if "hill" not in self.completed_missions:
            out["remaining_stop_m"] = max(0.0, stop - s)
            if raw_s > stop + self.rules["stop_tolerance_m"]:
                self._once("hill_stop_missed", out["route"])
                self._stop(out, "HILL_STOP_ZONE_MISSED", "FAULT")
                return
            # Signed reverse speed is never accepted as a stationary hold.
            # Position also catches slow drift hidden by the speed deadband.
            stationary = standing and snap["speed"] >= 0 and zone_ok and near_stop
            position = (raw_s, snap.get("x"), snap.get("y"))
            anchor = state.get("hill_hold_anchor")
            drift = abs(raw_s - anchor[0]) if anchor else 0.0
            if anchor and all(_number(v) for v in position[1:] + anchor[1:]):
                drift = max(drift, math.hypot(position[1] - anchor[1], position[2] - anchor[2]))
            if not stationary or drift > self.rules["hill_hold_position_tolerance_m"]:
                state["dwell_since"] = None
                state.pop("hill_hold_anchor", None)
            if stationary and state.get("hill_hold_anchor") is None:
                state["hill_hold_anchor"] = position
            held = self._dwell(state, now, stationary, self.rules["hill_hold_s"])
            out["hill_hold_elapsed_s"] = (now - state["dwell_since"]
                                            if state.get("dwell_since") is not None else 0.0)
            if held:
                self._complete("hill", now)
                state["hill_release"] = now
                out["phase"] = "CLIMBING"
                out["remaining_stop_m"] = None
            elif near_stop:
                self._stop(out, "HILL_REQUIRED_HOLD", "HOLD" if stationary else "STOPPING")
                return
        else:
            out["phase"] = "CLIMBING"
            out["hill_hold_elapsed_s"] = self.rules["hill_hold_s"]
        if "hill" in self.completed_missions:
            # The 30-second rule begins at first standstill, including the
            # mandatory hold. A diagnostic never bypasses an independent stop.
            if s >= top:
                state["hill_cleared"] = True
                self._complete("hill_clearance", now)
                out["phase"] = "COMPLETE"
            if snap.get("at_end") and state.get("hill_cleared"):
                self._next(out, ROUTES[2])

    def traffic_constraint(self, route, section, length, now, signal):
        """Read-only shared contract for the RDDF generator and mission output.

        None means this route has no traffic constraint. An invalid constraint
        never authorizes a path; unknown/stale observations keep a valid wall.
        """
        if section not in (2, 4, 7):
            return None
        required = "LEFT_ARROW" if section == 7 else "GREEN"
        marks, error = self._landmarks({"route": route, "landmarks": self.config.get("landmarks", {})}, section, length)
        return self._traffic_constraint(route, required, marks, error, now, signal)

    def _traffic_constraint(self, route, required, marks, error, now, signal):
        state = self.states.get(route, {})
        entered = state.get("authorized") or state.get("crossed") or "intersection:" + route in self.completed_missions
        permitted = self._fresh(signal, now, route) and signal.get("value") == required
        return {"valid": error is None, "active": not (entered or permitted) or error is not None,
                "stop_line_s": marks["stop_line_s"] if marks else None,
                "required_signal": required,
                "reason": error or ("CROSSING_AUTHORIZED" if entered else "SIGNAL_PERMITTED" if permitted else "WAIT_" + required)}

    def _traffic(self, snap, marks, state, out):
        now, s = snap["now"], snap["s"]
        front_s = s + self.rules["front_bumper_offset_m"]
        stop, exit_s = marks["stop_line_s"], marks["intersection_exit_s"]
        required = "LEFT_ARROW" if snap["section"] == 7 else "GREEN"
        signal = snap.get("signal", {})
        permitted = self._fresh(signal, now, snap["route"]) and signal.get("value") == required
        out["virtual_stop"] = self._traffic_constraint(snap["route"], required, marks, None, now, signal)
        mission_key = "intersection:" + snap["route"]
        if mission_key in self.completed_missions:
            state["crossed"] = True
        if not state.get("authorized") and not state.get("crossed"):
            out["remaining_stop_m"] = max(0.0, stop - front_s)
            if front_s >= stop and permitted:
                state["authorized"] = True
                state.setdefault("intersection_entered", now)
            elif front_s > stop + self.rules["stop_tolerance_m"]:
                self._once("unauthorized_intersection_entry", snap["route"])
                self._stop(out, "INTERSECTION_ENTERED_WITHOUT_PERMISSION", "FAULT")
                return
            elif not permitted:
                out["reason"] = "WAIT_" + required
                if front_s >= stop - self.rules["stop_tolerance_m"]:
                    self._stop(out, "WAIT_" + required, "WAIT_SIGNAL")
                    return
            else:
                # Green may permit a rolling approach. A red before the front
                # crosses still revokes entry; no early authorization latch.
                out["remaining_stop_m"] = None
                out["phase"] = "APPROACH_PERMITTED"
        if state.get("authorized") or state.get("crossed"):
            out["remaining_stop_m"] = None
            out["phase"] = "CROSSING"
            rear_s = s + self.rules["rear_axle_offset_m"]
            if not state.get("crossed"):
                if abs(snap["speed"]) <= self.rules["standstill_speed_mps"]:
                    if state.get("intersection_stop_since") is None:
                        state["intersection_stop_since"] = now
                    stopped_for = now - state["intersection_stop_since"]
                    if stopped_for >= self.rules["intersection_stop_penalty_s"]:
                        self._once("intersection_stop_penalty", snap["route"])
                        out["diagnostics"].append("INTERSECTION_STOP_AT_LEAST_3S")
                    if stopped_for >= self.rules["intersection_stop_timeout_s"]:
                        self._once("intersection_stop_timeout", snap["route"])
                        out["diagnostics"].append("INTERSECTION_STOP_AT_LEAST_20S")
                else:
                    state["intersection_stop_since"] = None
                entered = state.get("intersection_entered")
                if entered is not None and now - entered > self.rules["intersection_clearance_timeout_s"]:
                    self._once("intersection_clearance_timeout", snap["route"])
                    out["diagnostics"].append("INTERSECTION_CLEARANCE_TIMEOUT")
            if rear_s >= exit_s:
                state["crossed"] = True
                self._complete(mission_key, now)
                out["phase"] = "COMPLETE"
        if snap.get("at_end") and state.get("crossed") and snap["section"] != 4:
            self._next(out, ROUTES[3 if snap["section"] == 2 else 8])

    def _parking_observations(self, snap, kind):
        parking = snap.get("parking", {})
        if not isinstance(parking, dict):
            return {}
        if kind in parking and isinstance(parking[kind], dict):
            parking = parking[kind]
        observations = {}
        for side in ("left", "right"):
            value = parking.get(side)
            if isinstance(value, dict):
                observations[side] = value
            else:
                observations[side] = {"stamp": parking.get("stamp"), "value": value}
        return observations

    def _stable_candidate(self, key, side, observation, now, allowed, count):
        evidence_key = (key, side)
        if not self._fresh(observation, now) or observation.get("value") != allowed:
            self._candidate_evidence.pop(evidence_key, None)
            return False
        evidence = self._candidate_evidence.get(evidence_key)
        stamp = observation["stamp"]
        if evidence is None or stamp < evidence["stamp"] or stamp - evidence["stamp"] > self.rules["sensor_timeout_s"]:
            evidence = {"stamp": stamp, "count": 1}
        elif stamp > evidence["stamp"]:
            evidence = {"stamp": stamp, "count": evidence["count"] + 1}
        self._candidate_evidence[evidence_key] = evidence
        return evidence["count"] >= count

    def _parking_preview(self, snap, out, kind):
        observations = self._parking_observations(snap, kind)
        stable = []
        for side in ("left", "right"):
            observation = observations.get(side, {})
            fresh = self._fresh(observation, snap["now"])
            out["parking_candidates"][side] = observation.get("value") if fresh else "UNKNOWN"
            if self._stable_candidate("parking:" + kind, side, observation, snap["now"], "CLEAR",
                                      self.rules["parking_stable_observations"]):
                stable.append(side)
        selected = self.branches.get(kind)
        selected_observation = observations.get(selected, {})
        selected_clear = (self._fresh(selected_observation, snap["now"])
                          and selected_observation.get("value") == "CLEAR")
        if kind not in self.committed_branches and stable and (not selected or not selected_clear):
            preferred = self.config.get("preferred_parking_branch", "left")
            self.branches[kind] = preferred if preferred in stable else stable[0]
        if kind in self.branches:
            out["selected_branch"] = self.branches[kind]
        if snap["section"] == 9:
            out["phase"] = "SPACE_SELECTED" if kind in self.branches else "SEARCH_SPACE"

    def _parking_handoff(self, snap, out, kind):
        side = self.branches.get(kind)
        if not side:
            self._stop(out, "PARKING_SPACE_UNCONFIRMED", "WAIT_SPACE")
            return
        observation = self._parking_observations(snap, kind).get(side, {})
        if not self._fresh(observation, snap["now"]) or observation.get("value") != "CLEAR":
            self._stop(out, "SELECTED_PARKING_SPACE_NOT_CLEAR", "WAIT_SPACE")
            return
        out["selected_branch"] = side
        self._next(out, PARKING_ROUTES[kind][side][0])

    def parking_leg(self, route):
        """Return the accepted immutable leg, or an empty mapping before planning.

        A leg's direction is a planner request, not a substitute for checking
        the measured vehicle heading against the selected path in the runtime.
        """
        return dict(self.states.get(route, {}).get("parking_leg", {}))

    def _parking_leg(self, snap, marks, state, out, standing, entry, kind):
        """Accept a stamped leg only at its measured, stopped start pose.

        RDDFs have no gear-switch annotations. A planner must advertise those
        bounds explicitly. Entry supports forward setup then reverse parking;
        parallel entry may begin in reverse after setup on the approach route.
        All entry legs advance along the ordered RDDF, even when the body is
        travelling backwards. Final entry still requires the measured checkline.
        """
        observation = snap.get("parking_maneuver", {})
        active = state.get("parking_leg")
        if (not self._fresh(observation, snap["now"], snap["route"])
                or type(snap.get("decision_id")) is not int
                or type(observation.get("decision_id")) is not int
                or observation.get("decision_id") != snap["decision_id"]):
            state["dwell_since"] = None
            self._stop(out, "PARKING_MANEUVER_STALE_OR_MISMATCHED", "WAIT_PARKING_PLAN")
            return None
        index = observation.get("leg_index")
        phase, direction = observation.get("phase"), observation.get("direction")
        start, target = observation.get("start_s"), observation.get("target_s")
        final = observation.get("final_leg")
        expected_directions = {"FORWARD_APPROACH": 1, "REVERSE_ENTRY": -1, "FORWARD_EXIT": 1}
        if (type(index) is not int or index < 0 or type(direction) is not int
                or phase not in expected_directions or direction != expected_directions.get(phase)
                or not _number(start) or not _number(target) or start < 0
                or target <= start + 0.01 or target > snap["length"]
                or type(final) is not bool):
            state["dwell_since"] = None
            self._stop(out, "PARKING_MANEUVER_INVALID", "WAIT_PARKING_PLAN")
            return None
        tolerance = self.rules["stop_tolerance_m"]
        checkpoint = marks["parking_confirm_s" if entry else "parking_exit_s"]
        if entry:
            valid_phase = ((phase == "FORWARD_APPROACH" and not final
                            and target < checkpoint - tolerance)
                           or (phase == "REVERSE_ENTRY" and final
                               and abs(target - checkpoint) <= 1e-6))
        else:
            valid_phase = (phase == "FORWARD_EXIT" and final
                           and abs(target - checkpoint) <= 1e-6)
        if not valid_phase:
            state["dwell_since"] = None
            self._stop(out, "PARKING_MANEUVER_CHECKPOINT_INVALID", "WAIT_PARKING_PLAN")
            return None
        leg = {name: observation[name] for name in
               ("leg_index", "phase", "direction", "start_s", "target_s", "final_leg")}
        if active and index == active["leg_index"]:
            if leg != active:
                state["dwell_since"] = None
                self._stop(out, "PARKING_ACTIVE_LEG_CHANGED", "WAIT_PARKING_PLAN")
                return None
            return active
        raw_s = snap.get("raw_s", snap["s"])
        if not _number(raw_s):
            self._stop(out, "PARKING_RAW_PROGRESS_INVALID", "UNAVAILABLE")
            return None
        if active:
            ordered = (index == active["leg_index"] + 1
                       and active["phase"] == "FORWARD_APPROACH" and phase == "REVERSE_ENTRY"
                       and abs(start - active["target_s"]) <= 1e-6)
            if not ordered:
                self._stop(out, "PARKING_LEG_SEQUENCE_INVALID", "WAIT_PARKING_PLAN")
                return None
            near_start = abs(raw_s - active["target_s"]) <= tolerance
        else:
            allowed_first = ((entry and phase == "FORWARD_APPROACH")
                             or (entry and kind == "parallel" and phase == "REVERSE_ENTRY")
                             or (not entry and phase == "FORWARD_EXIT"))
            if index != 0 or not allowed_first or abs(start) > 1e-6:
                self._stop(out, "PARKING_INITIAL_LEG_INVALID", "WAIT_PARKING_PLAN")
                return None
            near_start = abs(raw_s - start) <= tolerance
        if not standing:
            self._stop(out, "WAIT_STANDSTILL_FOR_PARKING_LEG", "WAIT_GEAR_CHANGE")
            return None
        if not near_start:
            self._stop(out, "PARKING_LEG_START_POSE_MISMATCH", "WAIT_PARKING_PLAN")
            return None
        state["parking_leg"] = leg
        state["dwell_since"] = None
        out.update(direction=direction, parking_leg_index=index,
                   parking_leg_phase=phase, parking_leg_target_s=target)
        # A new leg must receive a new decision epoch and matching path before
        # movement, including when the first leg uses the default +1 direction.
        self._stop(out, "PARKING_LEG_ACCEPTED", "WAIT_PATH")
        return None

    def _parking(self, snap, marks, state, out, standing):
        kind = "t" if snap["section"] in (5, 6) else "parallel"
        side = _branch(snap["route"])
        if not side or self.branches.get(kind) != side:
            self._stop(out, "PARKING_BRANCH_NOT_AUTHORIZED", "UNAVAILABLE")
            return
        self.committed_branches.add(kind)
        entry = snap["section"] in (5, 10)
        if not entry and "parking:" + kind + ":entry" not in self.completed_missions:
            self._stop(out, "PARKING_ENTRY_NOT_COMPLETED", "UNAVAILABLE")
            return
        out["selected_branch"] = side
        if state.get("parking_fault"):
            self._stop(out, state["parking_fault"], "FAULT")
            return
        leg = self._parking_leg(snap, marks, state, out, standing, entry, kind)
        if leg is None:
            return
        raw_s = snap.get("raw_s", snap["s"])
        if not _number(raw_s):
            state["dwell_since"] = None
            self._stop(out, "PARKING_RAW_PROGRESS_INVALID", "UNAVAILABLE")
            return
        out.update(direction=leg["direction"], parking_leg_index=leg["leg_index"],
                   parking_leg_phase=leg["phase"], parking_leg_target_s=leg["target_s"])
        key = "parking:" + kind + ":" + ("entry" if entry else "exit")
        out["phase"] = leg["phase"]
        if leg["phase"] == "FORWARD_APPROACH":
            out["remaining_stop_m"] = max(0.0, leg["target_s"] - raw_s)
            if raw_s > leg["target_s"] + self.rules["stop_tolerance_m"]:
                state["parking_fault"] = "PARKING_LEG_TARGET_MISSED"
                self._stop(out, state["parking_fault"], "FAULT")
            elif raw_s >= leg["target_s"] - self.rules["stop_tolerance_m"]:
                self._stop(out, "WAIT_REVERSE_PARKING_LEG", "WAIT_GEAR_CHANGE")
            return
        position = marks["parking_confirm_s" if entry else "parking_exit_s"]
        if entry:
            out["remaining_stop_m"] = max(0.0, position - raw_s)
            target_yaw = marks.get("parking_yaw_rad")
            yaw_ok = target_yaw is None or (_number(target_yaw) and abs(
                math.atan2(math.sin(snap["yaw"] - target_yaw), math.cos(snap["yaw"] - target_yaw))
            ) <= self.rules["parking_yaw_tolerance_rad"])
            at_confirmation = abs(raw_s - position) <= self.rules["stop_tolerance_m"]
            if raw_s > position + self.rules["stop_tolerance_m"] and key not in self.completed_missions:
                self._once("parking_confirmation_missed", kind)
                state["parking_fault"] = "PARKING_CONFIRMATION_MISSED"
                self._stop(out, state["parking_fault"], "FAULT")
                return
            if key not in self.completed_missions:
                if self._dwell(state, snap["now"], standing and at_confirmation and yaw_ok,
                               self.rules["parking_hold_s"]):
                    self._complete(key, snap["now"])
                elif at_confirmation:
                    self._stop(out, "PARKING_CONFIRMATION_HOLD" if yaw_ok else "PARKING_POSE_MISMATCH",
                               "CONFIRM_PARKED")
                    return
            if key in self.completed_missions:
                out["phase"] = "PARKED"
                if not standing:
                    self._stop(out, "WAIT_STANDSTILL_FOR_EXIT", "PARKED")
                    return
                self._next(out, PARKING_ROUTES[kind][side][1])
        else:
            out["remaining_stop_m"] = None
            if raw_s >= position:
                self._complete(key, snap["now"])
            if snap.get("at_end") and key in self.completed_missions:
                out["phase"] = "COMPLETE"
                self._next(out, ROUTES[7 if kind == "t" else 12])

    def _finish_approach(self, snap, marks, out):
        """Follow the configured finish branch without camera lane control."""
        side = self.finish_branch
        self.branches["finish"] = side
        out["selected_branch"] = side
        out["phase"] = "FINISH_BRANCH_SELECTED"
        if side == "left" and snap["s"] >= marks["finish_branch_s"]:
            self._next(out, "13_left")
        elif side == "right" and snap.get("at_end"):
            self._next(out, "13_right")

    def _finish(self, snap, marks, out):
        side = _branch(snap["route"])
        if not side or self.finish_branch != side:
            self._stop(out, "FINISH_BRANCH_NOT_CONFIGURED", "UNAVAILABLE")
            return
        self.branches["finish"] = side
        self.committed_branches.add("finish")
        rear_s = snap["s"] + self.rules["rear_axle_offset_m"]
        # Give the stopping controller runoff beyond the scoring line. Using
        # the scoring line itself as a stop target can halt before the rear
        # axle crosses because of the controller's stopping buffer.
        out["remaining_stop_m"] = max(0.0, marks["finish_s"] + self.rules["finish_clearance_m"] - rear_s)
        out["phase"] = "FINISH_APPROACH"
        if rear_s >= marks["finish_s"]:
            self._complete("finish", snap["now"])
            self._stop(out, "COURSE_COMPLETE", "COMPLETE")
