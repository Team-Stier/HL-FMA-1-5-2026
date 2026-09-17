"""Consume Localization route matches and run missions without ROS."""
import math

from .geometry import Route, braking_distance, project
from .mission import MissionEngine


def fresh(stamp, now, timeout=0.5):
    return (isinstance(stamp, (int, float)) and math.isfinite(stamp)
            and stamp > 0 and 0 <= now - stamp <= timeout)


def validate_vehicle(config):
    vehicle = config.get('vehicle', {})
    if vehicle.get('validated') is not True:
        return False
    positive = ('front_m', 'deceleration_mps2', 'reaction_s', 'max_speed_mps')
    return all(type(vehicle.get(k)) in (int, float) and math.isfinite(vehicle[k])
               and vehicle[k] > 0 for k in positive)


class MissionRuntime:
    def __init__(self, routes, config):
        self.config = config
        self.engine = MissionEngine(config)
        self.source_routes = dict(routes)
        self.routes = self._with_finish_runout(routes, self.engine.rules['finish_runout_m'])
        self.active_route_name = config.get('start_route', '1_right')
        if self.active_route_name not in self.routes:
            raise ValueError('unknown active route: ' + self.active_route_name)
        self.active_route = self.routes[self.active_route_name]
        self.progress_s = 0.0
        tracker_config = config.get('tracker', {})
        self.end_tolerance_m = float(tracker_config.get('end_tolerance_m', 0.8))
        if not math.isfinite(self.end_tolerance_m) or self.end_tolerance_m <= 0:
            raise ValueError('tracker.end_tolerance_m must be positive')
        self.finish_left_branch_s = self._finish_left_branch()
        self.decision_id = 0
        self.request = None
        self.parking_request = None
        self.last_time = None
        self.clock_fault = False
        self.vehicle_ok = validate_vehicle(config)
        if self.vehicle_ok:
            # Localization base_link is at the rear axle. The traffic stop
            # reference must use the actual measured front bumper overhang.
            self.engine.rules['front_bumper_offset_m'] = config['vehicle']['front_m']
            minimum_stop = braking_distance(1.0/3.6, config['vehicle']['deceleration_mps2'],
                                            config['vehicle']['reaction_s'], config.get('stop_buffer_m', .05))
            if minimum_stop > self.engine.rules['stop_tolerance_m']:
                self.vehicle_ok = False

    def activate_route(self, route_name, progress_s=0.0):
        """Accept Localization's active RDDF without resetting mission state."""
        route = self.routes.get(route_name)
        if route is None:
            raise ValueError('unknown active route: ' + route_name)
        progress_s = float(progress_s)
        if not math.isfinite(progress_s):
            raise ValueError('route progress must be finite')
        changed = route_name != self.active_route_name
        self.active_route_name = route_name
        self.active_route = route
        self.progress_s = min(route.length, max(0.0, progress_s))
        if changed:
            self.request = None
            self.parking_request = None
        return changed

    @staticmethod
    def _with_finish_runout(routes, distance):
        """Append a straight control path after each section-13 RDDF endpoint."""
        extended = dict(routes)
        for name, route in routes.items():
            if route.section != 13:
                continue
            x, y, yaw = route.end
            endpoint = (x + distance * math.cos(yaw),
                        y + distance * math.sin(yaw), yaw)
            extended[name] = Route(name, list(route.points) + [endpoint], route.direction)
        return extended

    def _finish_left_branch(self):
        source, target = self.routes.get('12'), self.routes.get('13_left')
        if source is None or target is None:
            return None
        matched = project(source, target.start[0], target.start[1])
        tolerance = float(self.config.get('tracker', {}).get(
            'transition_join_tolerance_m', 2.5))
        if matched is None or matched['distance'] > tolerance:
            return None
        return matched['s']

    def _dynamic_obstacle(self, now, tracked, data):
        """Return an E-Stop request only for a cluster on a dynamic RDDF route."""
        config = self.config.get('dynamic_obstacle', {})
        token = str(config.get('route_token', 'dynamic')).strip().lower()
        required = bool(token) and token in str(tracked.get('route', '')).lower()
        result = {'required': required, 'valid': not required, 'active': False,
                  'reason': 'NOT_DYNAMIC_ROUTE', 'clearance_m': -1.0}
        if not required:
            return result
        observation = data.get('clusters', {})
        timeout = float(config.get('input_timeout_s', self.config.get('input_timeout_s', .5)))
        if (not observation.get('valid')
                or not fresh(observation.get('stamp'), now, timeout)
                or not fresh(observation.get('receipt_stamp'), now, timeout)
                or observation.get('frame') != 'map'):
            result['reason'] = observation.get('reason', 'DYNAMIC_OBSTACLE_CLUSTERS_UNAVAILABLE')
            return result
        lookahead = float(config.get('lookahead_m', self.config.get('path_lookahead_m', 20.0)))
        half_width = float(config.get('corridor_half_width_m', math.nan))
        if not (math.isfinite(lookahead) and lookahead > 0
                and math.isfinite(half_width) and half_width > 0):
            result['reason'] = 'DYNAMIC_OBSTACLE_CONFIG_INVALID'
            return result
        route = self.active_route
        minimum_s = max(0.0, tracked['s'])
        maximum_s = min(route.length, tracked['s'] + lookahead)
        nearest = None
        for cluster in observation.get('clusters', []):
            points = list(cluster)
            if points:
                points.append((sum(p[0] for p in points) / len(points),
                               sum(p[1] for p in points) / len(points)))
            for x, y in points:
                matched = project(route, x, y, minimum_s, maximum_s)
                if matched is not None and matched['distance'] <= half_width:
                    clearance = max(0.0, matched['s'] - tracked['s'])
                    nearest = clearance if nearest is None else min(nearest, clearance)
        result.update(valid=True, reason='DYNAMIC_OBSTACLE_CLEAR')
        if nearest is not None:
            result.update(active=True, reason='DYNAMIC_OBSTACLE_ON_RDDF', clearance_m=nearest)
        return result

    def _health(self, data, now):
        timeout = self.config.get('input_timeout_s', 0.5)
        odom = data.get('odom', {})
        valid = data.get('localization', {})
        if not fresh(odom.get('stamp'), now, timeout):
            return False, 'ODOMETRY_STALE'
        if not fresh(valid.get('stamp'), now, timeout) or valid.get('valid') is not True:
            return False, 'LOCALIZATION_INVALID'
        if odom.get('frame') != 'map' or odom.get('child_frame') != 'base_link':
            return False, 'ODOMETRY_FRAME_INVALID'
        if not all(math.isfinite(odom.get(k, math.nan)) for k in
                   ('x', 'y', 'yaw', 'speed')):
            return False, 'ODOMETRY_NONFINITE'
        return True, 'OK'

    def _matched_progress(self, data, now):
        """Consume Localization's projection for the active RDDF."""
        observation = data.get('rddf_match', {})
        timeout = self.config.get('input_timeout_s', 0.5)
        if (not fresh(observation.get('stamp'), now, timeout)
                or not fresh(observation.get('received'), now, timeout)
                or not fresh(observation.get('pose_stamp'), now, timeout)):
            return None, 'RDDF_MATCH_STALE'
        if observation.get('frame') != 'map':
            return None, 'RDDF_MATCH_FRAME_INVALID'

        expected = self.active_route_name
        if not observation.get('matched'):
            return None, observation.get('reason') or 'RDDF_MATCH_INVALID'
        if observation.get('route') != expected:
            return None, 'RDDF_ROUTE_MISMATCH'
        route = self.source_routes.get(expected)
        segment = observation.get('segment_index')
        fraction = observation.get('segment_fraction')
        distance = observation.get('distance_m')
        if (route is None or type(segment) is not int
                or not 0 <= segment < len(route.points) - 1
                or not all(isinstance(value, (int, float)) and math.isfinite(value)
                           for value in (fraction, distance))
                or not 0.0 <= fraction <= 1.0 or distance < 0):
            return None, 'RDDF_MATCH_INVALID'
        raw_s = (route.s[segment] + fraction *
                 (route.s[segment + 1] - route.s[segment]))

        # Section 13 has a State-Manager-owned 3 m straight runout that is not
        # present in Localization's source CSV.  Past the source endpoint, use
        # only terminal along-track displacement for that synthetic segment.
        if self.active_route.length > route.length and raw_s >= route.length - 1e-6:
            odom = data.get('odom', {})
            end_x, end_y, end_yaw = route.end
            along = ((odom.get('x', end_x) - end_x) * math.cos(end_yaw)
                     + (odom.get('y', end_y) - end_y) * math.sin(end_yaw))
            if math.isfinite(along) and along > 0:
                raw_s = min(self.active_route.length, route.length + along)
        self.progress_s = raw_s
        return {
            'route': expected, 'section': self.active_route.section,
            's': raw_s, 'raw_s': raw_s, 'length': self.active_route.length,
            'progress': raw_s/self.active_route.length,
            'cross_track': distance, 'healthy': True, 'reason': 'ok',
            'at_end': raw_s >= self.active_route.length-self.end_tolerance_m,
        }, 'OK'

    def _path_selection(self, data, now):
        """Consume the external Selector's decision without revalidating geometry."""
        status = data.get('selector_status', {})
        result = {'ready': False, 'reason': 'SELECTOR_STATUS_MISSING',
                  'path_fingerprint': ''}
        if self.request is None:
            return result
        timeout = self.config.get('input_timeout_s', 0.5)
        if (not fresh(status.get('stamp'), now, timeout)
                or not fresh(status.get('receipt_stamp'), now, timeout)):
            result['reason'] = 'SELECTOR_STATUS_STALE'
            return result
        route, mode, direction = self.request
        if (status.get('decision_id'), status.get('route'),
                status.get('source'), status.get('direction')) != (
                    self.decision_id, route, mode, direction):
            result['reason'] = 'SELECTOR_STATUS_REQUEST_MISMATCH'
            return result
        if status.get('ready') is not True:
            result['reason'] = status.get('reason') or 'REQUESTED_PATH_UNAVAILABLE'
            return result
        fingerprint = status.get('path_fingerprint')
        if not isinstance(fingerprint, str) or not fingerprint:
            result['reason'] = 'SELECTOR_FINGERPRINT_MISSING'
            return result
        result.update(ready=True, reason='PATH_ACCEPTED',
                      path_fingerprint=fingerprint)
        return result

    def traffic_constraint(self, now, signal):
        route = self.active_route
        wall = self.engine.traffic_constraint(route.name, route.section, route.length, now, signal)
        if wall and wall['valid']:
            wall['target_s'] = wall['stop_line_s'] - self.engine.rules['front_bumper_offset_m'] - self.config.get('stop_buffer_m', .05)
            wall['pose'] = route.pose_at(wall['stop_line_s'])
            if wall['target_s'] <= 0:
                wall.update(valid=False, active=True, reason='STOP_LINE_HAS_NO_APPROACH_SPACE')
        return wall

    def rddf_points(self, now, signal):
        """Generate only the allowed RDDF prefix; green restores full lookahead.

        Evaluate the current observation, not last tick's decision, so a red
        update cannot publish another unrestricted path. Missing calibration
        produces no traffic path. The normal mission gate still checks health.
        """
        route = self.active_route
        start = max(0.0, self.progress_s - 1.0)
        end = min(route.length, self.progress_s + max(20.0, self.config.get('path_lookahead_m', 20.0)))
        wall = self.traffic_constraint(now, signal)
        if wall:
            if not wall['valid']:
                return []
            if wall['active']:
                end = min(end, wall['target_s'])
        if end <= start:
            return []
        # Preserve original corners as well as interpolated endpoints.
        samples = route.slice(start, end)
        points = [samples[0]]
        for a, b in zip(samples, samples[1:]):
            steps = max(1, int(math.ceil(math.hypot(b[0]-a[0], b[1]-a[1]) / .5)))
            yaw_delta = math.atan2(math.sin(b[2]-a[2]), math.cos(b[2]-a[2]))
            for i in range(1, steps+1):
                t = i / steps
                points.append((a[0]+t*(b[0]-a[0]), a[1]+t*(b[1]-a[1]), a[2]+t*yaw_delta))
        return points

    def step(self, now, data, selector_status=None):
        if selector_status is not None:
            data = dict(data)
            data['selector_status'] = selector_status
        if self.last_time is not None and now < self.last_time:
            self.clock_fault = True
        self.last_time = now
        healthy, reason = self._health(data, now)
        if self.clock_fault:
            healthy, reason = False, 'CLOCK_REGRESSION_RESTART_REQUIRED'
        odom = data.get('odom', {})
        tracked = {'route': self.active_route_name, 'section': self.active_route.section,
                   's': self.progress_s, 'raw_s': self.progress_s,
                   'length': self.active_route.length,
                   'progress': self.progress_s/self.active_route.length, 'at_end': False}
        if healthy:
            matched, match_reason = self._matched_progress(data, now)
            if matched is None:
                healthy, reason = False, match_reason
            else:
                tracked.update(matched)
                healthy, reason = tracked['healthy'], tracked['reason']
        if self.request is None:
            mode = 'LOCAL' if tracked['section'] == 3 else 'PARKING' if tracked['section'] in (10, 11) else 'RDDF'
            self._set_request(tracked['route'], mode, -1 if tracked['section'] == 5 else 1)
        selection = self._path_selection(data, now)
        snapshot = dict(tracked, now=now, healthy=healthy, reason=reason,
                        speed=odom.get('speed', 0), yaw=odom.get('yaw', 0),
                        x=odom.get('x'), y=odom.get('y'),
                        # Landmark completeness is checked for the active RDDF
                        # by MissionEngine._landmarks().  Vehicle geometry is
                        # required only by features that actually consume it.
                        calibrated=not self.config.get('calibration_mode', False),
                        landmarks=self.config.get('landmarks', {}),
                        signal=data.get('signal', {}), path_ready=selection['ready'],
                        decision_id=self.decision_id,
                        parking_maneuver=data.get('parking_maneuver', {}),
                        parking=data.get('parking', {}) if healthy else {},
                        finish_branch_s=self.finish_left_branch_s)
        # Dynamic-obstacle E-Stop evaluation remains independent of mission
        # dwell accounting, so it cannot reset a route-specific hold timer.
        decision = self.engine.update(snapshot)
        decision['virtual_stop'] = self.traffic_constraint(now, data.get('signal', {}))
        dynamic_token = str(self.config.get('dynamic_obstacle', {}).get(
            'route_token', 'dynamic')).strip().lower()
        dynamic_obstacle = self._dynamic_obstacle(now, tracked, data) if healthy else {
            'required': bool(dynamic_token) and dynamic_token in str(tracked['route']).lower(),
            'valid': False, 'active': False, 'reason': reason, 'clearance_m': -1.0}
        decision['dynamic_obstacle'] = dynamic_obstacle
        request = (tracked['route'], decision['path_mode'], decision['direction'])
        parking_request = ((tracked['route'], decision['parking_leg_index'], decision['parking_leg_phase'],
                            decision['parking_leg_target_s']) if decision.get('parking_leg_index', -1) >= 0 else None)
        if request != self.request or parking_request != self.parking_request:
            self._set_request(*request)
            self.parking_request = parking_request
            selection = self._path_selection(data, now)
            decision.update(stop_requested=True, speed_limit=0.0, next_route=None, reason='WAIT_NEW_PATH', phase='WAIT_PATH')
        safety = {'stop': True, 'sensor_valid': healthy, 'reason': reason, 'clearance_m': -1.0,
                  'path_fingerprint': ''}
        if healthy and selection['ready']:
            safety.update(stop=False, reason='PATH_ACCEPTED',
                          path_fingerprint=selection['path_fingerprint'])
        elif healthy:
            safety['reason'] = selection['reason']
        remaining_stop = decision.get('remaining_stop_m')
        if remaining_stop is not None and self.vehicle_ok:
            available = max(0.0, remaining_stop - self.config.get('stop_buffer_m', 0.05))
            a, reaction = self.config['vehicle']['deceleration_mps2'], self.config['vehicle']['reaction_s']
            cap = max(0.0, math.sqrt((a*reaction)**2 + 2*a*available) - a*reaction)
            decision['speed_limit'] = min(decision['speed_limit'], cap)
        if self.vehicle_ok:
            decision['speed_limit'] = min(decision['speed_limit'], self.config['vehicle']['max_speed_mps'])
        # Localization alone owns active-RDDF handoff. ``next_route`` remains a
        # mission hint for diagnostics; it never changes tracking state here.
        decision.update(decision_id=self.decision_id, valid=healthy and snapshot['calibrated']
                        and decision['phase'] not in ('UNAVAILABLE', 'FAULT'),
                        finished='finish' in decision.get('completed_missions', {}),
                        progress=tracked['progress'], distance_m=tracked['s'], safety=safety,
                        tracking=tracked)
        if dynamic_obstacle.get('active'):
            safety.update(stop=True, sensor_valid=True, reason='DYNAMIC_OBSTACLE_ON_RDDF',
                          clearance_m=dynamic_obstacle['clearance_m'])
            decision.update(emergency_stop_requested=True, stop_requested=True,
                            speed_limit=0.0, reason='DYNAMIC_OBSTACLE_ON_RDDF',
                            phase='EMERGENCY_STOP')
        elif dynamic_obstacle.get('required') and not dynamic_obstacle.get('valid'):
            safety.update(stop=True, sensor_valid=False, reason=dynamic_obstacle['reason'],
                          clearance_m=-1.0)
            decision.update(stop_requested=True, speed_limit=0.0,
                            reason=dynamic_obstacle['reason'],
                            phase='WAIT_DYNAMIC_OBSERVATION')
        if not decision['valid'] or safety['stop']:
            decision['stop_requested'] = True
            decision['speed_limit'] = 0.0
        return decision

    def _set_request(self, route, mode, direction):
        self.decision_id += 1
        self.request = (route, mode, direction)
