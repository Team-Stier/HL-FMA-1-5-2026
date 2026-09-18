"""Deterministic mission decisions; no ROS, vehicle commands, or geometry I/O.

Distances are metres along the active route. ``s`` may be monotonic tracker
progress; ``raw_s`` must retain backwards motion for the hill rollback check.
Landmarks refer to the tracking reference unless a documented axle/bumper
offset is configured. The ROS adapter is responsible for timestamped perception,
collision checks, matching planner acknowledgements, and applying stop requests.
"""

import math


RULE_DEFAULTS = {
    # One shared standstill definition is used by every mission.
    "standstill_speed_mps": 0.5,
    "hill_hold_s": 3.0,
    "hill_hold_position_tolerance_m": 0.02,
    "parking_hold_s": 1.0,
    "sensor_timeout_s": 0.5,
    "max_update_gap_s": 0.5,
    "parking_stable_observations": 3,
    "finish_sign_stable_observations": 3,
    "stop_tolerance_m": 0.2,
    "traffic_tracking_stop_m": 0.5,
    "front_bumper_offset_m": 0.0,
    "rear_axle_offset_m": 0.0,
    "finish_runout_m": 3.0,
    "hill_rollback_limit_m": 0.5,
    "hill_clearance_timeout_s": 30.0,
    "mission_deadline_s": 480.0,
    "no_motion_timeout_s": 60.0,
    "intersection_stop_penalty_s": 3.0,
    "intersection_stop_timeout_s": 20.0,
    "intersection_clearance_timeout_s": 30.0,
    "traffic_force_departure_s": 20.0,
    "parking_yaw_tolerance_rad": math.radians(10.0),
}

SPEED_DEFAULTS = {
    "normal": 8.0 / 3.6, "hill": 1.0, "static": 1.0,
    "intersection": 1.0, "parking": 6.0 / 3.6,
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
PARALLEL_PROFILE_DEFAULTS = {
    "10_parallel-left-in": {
        "initial_direction": 1,
        "changes": [{"s": 9.337439695228316, "direction": -1}],
    },
    "10_parallel-right-in": {
        "initial_direction": 1,
        "changes": [{"s": 6.614062696750327, "direction": -1},
                    {"s": 16.56201644939806, "direction": 1}],
    },
    "11-parallel-left-out": {
        "initial_direction": -1,
        "changes": [{"s": 0.7236489020465036, "direction": 1}],
    },
    "11_parallel-right-out": {
        "initial_direction": -1,
        "changes": [{"s": 2.237988918723955, "direction": 1}],
    },
}
REQUIRED_LANDMARKS = {
    1: ("hill_start_s", "hill_stop_s", "hill_top_s"),
    2: ("stop_line_s",),
    4: ("stop_line_s",),
    7: ("stop_line_s",),
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
                     "finish_sign_stable_observations",
                     "finish_runout_m", "hill_hold_position_tolerance_m",
                     "traffic_force_departure_s", "parking_hold_s",
                     "traffic_tracking_stop_m"):
            if self.rules[name] <= 0:
                raise ValueError("mission rule must be positive: " + name)
        for name in ("parking_stable_observations", "finish_sign_stable_observations"):
            if int(self.rules[name]) != self.rules[name]:
                raise ValueError("observation counts must be integers")
        self.finish_fallback_branch = self.config.get(
            "finish_fallback_branch", self.config.get("finish_branch", "left"))
        if self.finish_fallback_branch not in ("left", "right"):
            raise ValueError("finish_fallback_branch must be left or right")
        configured_parking = self.config.get("parking_branches", {})
        if not isinstance(configured_parking, dict):
            raise ValueError("parking_branches must be an object")
        self.parking_branches = {
            "t": configured_parking.get("t", "left"),
            "parallel": configured_parking.get("parallel", "left"),
        }
        if any(side not in ("left", "right") for side in self.parking_branches.values()):
            raise ValueError("parking branches must be left or right")
        profiles = self.config.get("parallel_parking_profiles", PARALLEL_PROFILE_DEFAULTS)
        if not isinstance(profiles, dict):
            raise ValueError("parallel_parking_profiles must be an object")
        self.parallel_profiles = {}
        for route, profile in profiles.items():
            if not isinstance(profile, dict):
                raise ValueError("parallel parking profile must be an object: " + route)
            direction = profile.get("initial_direction")
            changes = profile.get("changes", [])
            if type(direction) is not int or direction not in (-1, 1) or not isinstance(changes, list):
                raise ValueError("invalid parallel parking profile: " + route)
            parsed, previous_s, previous_direction = [], 0.0, direction
            for change in changes:
                if not isinstance(change, dict):
                    raise ValueError("invalid parallel parking change: " + route)
                position, next_direction = change.get("s"), change.get("direction")
                if (not _number(position) or position <= previous_s
                        or type(next_direction) is not int or next_direction not in (-1, 1)
                        or next_direction == previous_direction):
                    raise ValueError("invalid parallel parking change: " + route)
                parsed.append((float(position), next_direction))
                previous_s, previous_direction = position, next_direction
            self.parallel_profiles[route] = (direction, tuple(parsed))
        self.speeds = dict(SPEED_DEFAULTS)
        self.speeds.update(self.config.get("speeds", {}))
        if any(not _number(value) or value <= 0 for value in self.speeds.values()):
            raise ValueError("mission speed limits must be positive finite numbers")
        self.states = {}
        self.branches = dict(self.parking_branches)
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
            state["signal_wait_since"] = None
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
        return landmarks, None

    def _base(self, snapshot):
        route = str(snapshot.get("route", ""))
        section = snapshot.get("section", 0)
        mission = {
            1: "HILL_STOP", 2: "TRAFFIC_STRAIGHT", 3: "STATIC_AVOIDANCE",
            4: "TRAFFIC_STRAIGHT", 5: "T_PARKING", 6: "T_PARKING",
            7: "TRAFFIC_LEFT", 8: "DYNAMIC_OBSTACLE", 9: "PARALLEL_APPROACH",
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
        mode = "LOCAL" if section == 3 else "RDDF"
        direction = -1 if section in (5, 11) else 1
        return {
            "route": route, "section": section, "mission": mission,
            "phase": "APPROACH", "selected_branch": _branch(route),
            "branch": _branch(route), "path_mode": mode,
            "stop_requested": False, "emergency_stop_requested": False,
            "speed_limit": float(self.speeds[speed_name]),
            "direction": direction,
            "reason": "", "next_route": None, "remaining_stop_m": None,
            "counters": {}, "diagnostics": [], "completed_missions": {},
            "parking_candidates": {}, "virtual_stop": None,
            "hill_target_s": None, "hill_hold_elapsed_s": 0.0,
            "traffic_wait_elapsed_s": 0.0,
            "traffic_force_departure_s": self.rules["traffic_force_departure_s"],
            "parking_leg_index": -1,
            "parking_leg_phase": "",
            "parking_leg_target_s": None,
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
                if snapshot.get("at_end") and state.get("authorized"):
                    # Once the end of the forward approach has been observed,
                    # retain the handoff even if projection jitters off the
                    # endpoint while the vehicle is braking.
                    state["t_reverse_handoff_pending"] = True
                if (state.get("t_reverse_handoff_pending")
                        and not out["stop_requested"]):
                    self._t_reverse_handoff(snapshot, state, out, standing)
        elif section == 3:
            out["phase"] = "AVOIDING"
            if snapshot.get("at_end"):
                self._complete("static", now)
                out["phase"] = "COMPLETE"
                self._next(out, ROUTES[4])
        elif section in (5, 6):
            self._t_parking(snapshot, state, out, standing)
        elif section in (10, 11):
            self._parallel_parking(snapshot, state, out, standing)
        elif section == 8:
            out["phase"] = "MONITORING_DYNAMIC"
            if snapshot.get("at_end"):
                self._complete("section_8_transit", now)
                out["phase"] = "COMPLETE"
                self._next(out, ROUTES[9])
        elif section == 9:
            self._parking_preview(snapshot, out, "parallel")
            if snapshot.get("at_end"):
                self._next(out, PARKING_ROUTES["parallel"][self.branches["parallel"]][0])
        elif section == 12:
            self._finish_approach(snapshot, out)
        elif section == 13:
            self._finish(snapshot, out)
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
            # Accept speed noise in both directions within the standstill range.
            # Position also catches slow drift hidden by the speed deadband.
            stationary = standing and zone_ok and near_stop
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
        entered = state.get("authorized") or "intersection:" + route in self.completed_missions
        permitted = self._fresh(signal, now, route) and signal.get("value") == required
        return {"valid": error is None, "active": not (entered or permitted) or error is not None,
                "stop_line_s": marks["stop_line_s"] if marks else None,
                "required_signal": required,
                "reason": error or ("CROSSING_AUTHORIZED" if entered else "SIGNAL_PERMITTED" if permitted else "WAIT_" + required)}

    def _traffic(self, snap, marks, state, out):
        now, s = snap["now"], snap["s"]
        front_s = s + self.rules["front_bumper_offset_m"]
        stop = marks["stop_line_s"]
        required = "LEFT_ARROW" if snap["section"] == 7 else "GREEN"
        signal = snap.get("signal", {})
        permitted = self._fresh(signal, now, snap["route"]) and signal.get("value") == required
        out["virtual_stop"] = self._traffic_constraint(snap["route"], required, marks, None, now, signal)
        mission_key = "intersection:" + snap["route"]
        waiting = state.get("traffic_wait_latched", False)
        since = state.get("signal_wait_since")
        if since is not None:
            out["traffic_wait_elapsed_s"] = max(0.0, now - since)
        if mission_key in self.completed_missions:
            state["authorized"] = True
        if not state.get("authorized"):
            out["remaining_stop_m"] = max(0.0, stop - front_s)
            if (front_s >= stop or waiting) and permitted:
                state["authorized"] = True
                state.setdefault("intersection_entered", now)
                state["signal_wait_since"] = None
            elif not waiting and front_s > stop + self.rules["stop_tolerance_m"]:
                self._once("unauthorized_intersection_entry", snap["route"])
                self._stop(out, "INTERSECTION_ENTERED_WITHOUT_PERMISSION", "FAULT")
                return
            elif not permitted:
                out["reason"] = "WAIT_" + required
                if waiting or out["remaining_stop_m"] <= self.rules["traffic_tracking_stop_m"]:
                    # Once waiting at the line, pose jitter must not command
                    # another approach. Start the timer at first standstill.
                    state["traffic_wait_latched"] = True
                    standing = abs(snap["speed"]) <= self.rules["standstill_speed_mps"]
                    if standing and state.get("signal_wait_since") is None:
                        state["signal_wait_since"] = now
                    since = state.get("signal_wait_since")
                    waited = max(0.0, now - since) if since is not None else 0.0
                    out["traffic_wait_elapsed_s"] = waited
                    if standing and waited >= self.rules["traffic_force_departure_s"]:
                        state["authorized"] = True
                        state["intersection_entered"] = now
                        self._once("traffic_force_departure", snap["route"])
                        out["diagnostics"].append("TRAFFIC_FORCE_DEPARTURE_AFTER_TIMEOUT")
                        out["reason"] = "TRAFFIC_FORCE_DEPARTURE_AFTER_TIMEOUT"
                        out["virtual_stop"] = self._traffic_constraint(
                            snap["route"], required, marks, None, now, signal)
                    else:
                        self._stop(out, "WAIT_" + required, "WAIT_SIGNAL")
                        return
                else:
                    state["signal_wait_since"] = None
            else:
                state["signal_wait_since"] = None
                # Green may permit a rolling approach. A red before the front
                # crosses still revokes entry; no early authorization latch.
                out["remaining_stop_m"] = None
                out["phase"] = "APPROACH_PERMITTED"
        if state.get("authorized"):
            out["remaining_stop_m"] = None
            out["phase"] = "CROSSING"
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
            if snap.get("at_end"):
                self._complete(mission_key, now)
                out["phase"] = "COMPLETE"
        if snap.get("at_end") and state.get("authorized") and snap["section"] != 4:
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
        """Expose the configured branch; parking-space perception is not used."""
        observations = self._parking_observations(snap, kind)
        for side in ("left", "right"):
            observation = observations.get(side, {})
            fresh = self._fresh(observation, snap["now"])
            out["parking_candidates"][side] = observation.get("value") if fresh else "UNKNOWN"
        out["selected_branch"] = self.branches[kind]
        if snap["section"] == 9:
            out["phase"] = "SPACE_SELECTED"

    def _t_reverse_handoff(self, snap, state, out, standing):
        held = self._dwell(state, snap["now"], standing,
                           self.rules["parking_hold_s"])
        self._stop(out, "T_PARKING_REVERSE_HOLD", "WAIT_GEAR_CHANGE")
        if held:
            self._next(out, PARKING_ROUTES["t"][self.branches["t"]][0])

    def _parallel_parking(self, snap, state, out, standing):
        """Follow the recorded parallel-parking RDDF and change gear at its configured points."""
        side = _branch(snap["route"])
        if not side or self.branches.get("parallel") != side:
            self._stop(out, "PARKING_BRANCH_NOT_AUTHORIZED", "UNAVAILABLE")
            return
        self.committed_branches.add("parallel")
        out["selected_branch"] = side
        profile = self.parallel_profiles.get(snap["route"])
        if profile is None:
            self._stop(out, "PARALLEL_RDDF_PROFILE_MISSING", "UNAVAILABLE")
            return
        initial_direction, changes = profile
        if changes and changes[-1][0] >= snap["length"]:
            self._stop(out, "PARALLEL_RDDF_PROFILE_INVALID", "UNAVAILABLE")
            return
        raw_s = snap.get("raw_s", snap["s"])
        if not _number(raw_s):
            self._stop(out, "PARKING_RAW_PROGRESS_INVALID", "UNAVAILABLE")
            return
        legs = [(0.0, initial_direction)] + list(changes)
        if "parallel_leg_index" not in state:
            state["parallel_leg_index"] = max(
                index for index, (start, _) in enumerate(legs)
                if raw_s > start + self.rules["stop_tolerance_m"] or index == 0)
        index = min(state["parallel_leg_index"], len(legs) - 1)
        start_s, direction = legs[index]
        end_s = legs[index + 1][0] if index + 1 < len(legs) else snap["length"]
        phase = "FORWARD" if direction > 0 else "REVERSE"
        out.update(path_mode="RDDF", direction=direction, phase=phase,
                   parking_leg_index=index, parking_leg_phase=phase,
                   parking_leg_target_s=end_s, rddf_start_s=start_s,
                   rddf_end_s=end_s)

        if index + 1 < len(legs):
            out["remaining_stop_m"] = max(0.0, end_s - raw_s)
            if raw_s >= end_s - self.rules["stop_tolerance_m"]:
                held = self._dwell(state, snap["now"], standing,
                                   self.rules["parking_hold_s"])
                self._stop(out, "PARALLEL_GEAR_CHANGE", "WAIT_GEAR_CHANGE")
                if held:
                    state["parallel_leg_index"] = index + 1
                    next_start, next_direction = legs[index + 1]
                    next_end = (legs[index + 2][0]
                                if index + 2 < len(legs) else snap["length"])
                    next_phase = "FORWARD" if next_direction > 0 else "REVERSE"
                    out.update(direction=next_direction,
                               parking_leg_index=index + 1,
                               parking_leg_phase=next_phase,
                               parking_leg_target_s=next_end,
                               rddf_start_s=next_start, rddf_end_s=next_end)
                    state["dwell_since"] = None
            else:
                state["dwell_since"] = None
            return

        if snap["section"] == 10:
            out["remaining_stop_m"] = max(0.0, snap["length"] - raw_s)
            if raw_s >= snap["length"] - self.rules["stop_tolerance_m"]:
                self._stop(out, "PARALLEL_ENTRY_COMPLETE", "PARKED")
                if self._dwell(state, snap["now"], standing,
                               self.rules["parking_hold_s"]):
                    self._complete("parking:parallel:entry", snap["now"])
                    self._next(out, PARKING_ROUTES["parallel"][side][1])
            else:
                state["dwell_since"] = None
            return

        out["remaining_stop_m"] = None
        if snap.get("at_end"):
            self._complete("parking:parallel:exit", snap["now"])
            out["phase"] = "COMPLETE"
            self._next(out, ROUTES[12])

    def _t_parking(self, snap, state, out, standing):
        """Follow the recorded T RDDF: reverse in, configured hold, forward out."""
        side = _branch(snap["route"])
        if not side or self.branches.get("t") != side:
            self._stop(out, "PARKING_BRANCH_NOT_AUTHORIZED", "UNAVAILABLE")
            return
        self.committed_branches.add("t")
        out["selected_branch"] = side
        out["parking_leg_index"] = -1
        out["parking_leg_target_s"] = -1.0
        raw_s = snap.get("raw_s", snap["s"])
        if not _number(raw_s):
            self._stop(out, "PARKING_RAW_PROGRESS_INVALID", "UNAVAILABLE")
            return
        if snap["section"] == 5:
            out.update(direction=-1, path_mode="RDDF", phase="REVERSE_ENTRY",
                       parking_leg_phase="REVERSE_ENTRY",
                       remaining_stop_m=max(0.0, snap["length"] - raw_s))
            if snap.get("at_end"):
                state["t_exit_handoff_pending"] = True
            if state.get("t_exit_handoff_pending"):
                self._stop(out, "T_PARKING_ENTRY_COMPLETE", "WAIT_GEAR_CHANGE")
                if self._dwell(state, snap["now"], standing,
                               self.rules["parking_hold_s"]):
                    self._complete("parking:t:entry", snap["now"])
                    self._next(out, PARKING_ROUTES["t"][side][1])
            return

        out.update(direction=1, path_mode="RDDF", phase="FORWARD_EXIT",
                   parking_leg_phase="FORWARD_EXIT", remaining_stop_m=None)
        if snap.get("at_end"):
            self._complete("parking:t:exit", snap["now"])
            out["phase"] = "COMPLETE"
            self._next(out, ROUTES[7])

    def _finish_approach(self, snap, out):
        """Select the section-13 branch from stable DOWN/X camera signs."""
        lanes = snap.get("lane", {})
        fresh = self._fresh(lanes, snap["now"], snap["route"])
        stable = []
        for side in ("left", "right"):
            observation = ({"stamp": lanes.get("stamp"), "value": lanes.get(side)}
                           if fresh else {})
            if self._stable_candidate("finish", side, observation, snap["now"], "DOWN",
                                      self.rules["finish_sign_stable_observations"]):
                stable.append(side)
        if "finish" not in self.branches and stable:
            self.branches["finish"] = (self.finish_fallback_branch
                                       if self.finish_fallback_branch in stable else stable[0])
        side = self.branches.get("finish")
        out["selected_branch"] = side
        out["phase"] = "FINISH_SIGN_SELECTED" if side else "READ_FINISH_SIGN"
        branch_s = snap.get("finish_branch_s")
        if not _number(branch_s):
            self._stop(out, "FINISH_BRANCH_GEOMETRY_INVALID", "UNAVAILABLE")
            return
        if not side:
            out["remaining_stop_m"] = max(0.0, branch_s - snap["s"])
            if snap["s"] >= branch_s - self.rules["stop_tolerance_m"]:
                side = self.finish_fallback_branch
                self.branches["finish"] = side
                out["selected_branch"] = side
                out["phase"] = "FINISH_FALLBACK_SELECTED"
                out["reason"] = "FINISH_SIGN_FALLBACK"
                out["diagnostics"].append("FINISH_SIGN_FALLBACK:" + side)
            else:
                return
        if side == "left":
            if snap["s"] >= branch_s:
                self._next(out, "13_left")
        elif side == "right" and snap.get("at_end"):
            self._next(out, "13_right")

    def _finish(self, snap, out):
        side = _branch(snap["route"])
        selected = self.branches.get("finish")
        if selected is None and side:
            # Direct section-13 testing remains possible without replaying 12.
            self.branches["finish"] = side
            selected = side
        if not side or selected != side:
            self._stop(out, "FINISH_BRANCH_NOT_SELECTED", "UNAVAILABLE")
            return
        self.branches["finish"] = side
        self.committed_branches.add("finish")
        # Runtime extends only section 13 by finish_runout_m. Therefore this
        # route endpoint is exactly the requested point beyond the RDDF end.
        out["remaining_stop_m"] = max(0.0, snap["length"] - snap["s"])
        out["phase"] = "FINISH_APPROACH"
        if snap["s"] >= snap["length"] - self.rules["stop_tolerance_m"]:
            self._complete("finish", snap["now"])
            self._stop(out, "COURSE_COMPLETE", "COMPLETE")
