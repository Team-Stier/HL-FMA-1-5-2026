"""Orchestrate route tracking, missions and observed-space vetoes without ROS."""
import math

from selector.core import SelectorCore, State, path_fingerprint
from .geometry import Route, RouteTracker, braking_distance, corridor_status, project
from .mission import MissionEngine, PARKING_ROUTES


def fresh(stamp, now, timeout=0.5):
    return (isinstance(stamp, (int, float)) and math.isfinite(stamp)
            and stamp > 0 and 0 <= now - stamp <= timeout)


def validate_vehicle(config):
    vehicle = config.get('vehicle', {})
    if vehicle.get('validated') is not True or vehicle.get('curb_visibility_validated') is not True:
        return False
    positive = ('width_m', 'front_m', 'rear_m', 'deceleration_mps2', 'reaction_s',
                'max_speed_mps', 'max_yaw_rate_rps')
    return all(type(vehicle.get(k)) in (int, float) and math.isfinite(vehicle[k])
               and vehicle[k] > 0 for k in positive)


def footprint(pose, vehicle):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + a*c - b*s, y + a*s + b*c)
            for a, b in ((vehicle['front_m'], vehicle['width_m']/2),
                         (vehicle['front_m'], -vehicle['width_m']/2),
                         (-vehicle['rear_m'], -vehicle['width_m']/2),
                         (-vehicle['rear_m'], vehicle['width_m']/2))]


class MissionRuntime:
    def __init__(self, routes, config):
        self.routes, self.config = routes, config
        self.tracker = RouteTracker(routes, config.get('start_route', '1_right'), config.get('tracker'))
        self.engine = MissionEngine(config)
        self.selector = SelectorCore(timeout=config.get('input_timeout_s', 0.5), future_tolerance=0,
                                     max_path_heading_error_rad=config.get('max_path_heading_error_rad', 1.0))
        self.decision_id = 0
        self.request = None
        self.parking_request = None
        self.last_time = None
        self.clock_fault = False
        self.transition_fault = ''
        self.vehicle_ok = validate_vehicle(config)
        self.vehicle_error = 'VEHICLE_CALIBRATION_REQUIRED'
        self.preview_cache = None
        if self.vehicle_ok:
            # Localization base_link is at the rear axle. The traffic stop
            # reference must use the actual measured front bumper overhang.
            self.engine.rules['front_bumper_offset_m'] = config['vehicle']['front_m']
            minimum_stop = braking_distance(1.0/3.6, config['vehicle']['deceleration_mps2'],
                                            config['vehicle']['reaction_s'], config.get('stop_buffer_m', .05))
            if minimum_stop > self.engine.rules['stop_tolerance_m']:
                self.vehicle_ok = False
                self.vehicle_error = 'STOP_PRECISION_BELOW_COMMAND_RESOLUTION'

    def _dynamic_obstacle(self, now, tracked, data):
        """Return an E-Stop request only for a cluster on a dynamic RDDF route."""
        config = self.config.get('dynamic_obstacle', {})
        token = str(config.get('route_token', 'dynamic')).strip().lower()
        required = bool(token) and token in str(tracked.get('route', '')).lower()
        result = {'required': required, 'valid': not required, 'active': False,
                  'reason': 'NOT_DYNAMIC_ROUTE', 'clearance_m': -1.0}
        if not required:
            return result
        if not self.vehicle_ok:
            result['reason'] = 'DYNAMIC_OBSTACLE_VEHICLE_CALIBRATION_REQUIRED'
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
        margin = float(config.get('path_margin_m', self.config['vehicle'].get('margin_m', .25)))
        if not (math.isfinite(lookahead) and lookahead > 0 and math.isfinite(margin) and margin >= 0):
            result['reason'] = 'DYNAMIC_OBSTACLE_CONFIG_INVALID'
            return result
        route = self.tracker.current
        minimum_s = max(0.0, tracked['s'] - self.config['vehicle']['rear_m'])
        maximum_s = min(route.length, tracked['s'] + lookahead)
        corridor_half_width = self.config['vehicle']['width_m'] * .5 + margin
        nearest = None
        for cluster in observation.get('clusters', []):
            points = list(cluster)
            if points:
                points.append((sum(p[0] for p in points) / len(points),
                               sum(p[1] for p in points) / len(points)))
            for x, y in points:
                matched = project(route, x, y, minimum_s, maximum_s)
                if matched is not None and matched['distance'] <= corridor_half_width:
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
        scan = data.get('scan', {})
        if not fresh(odom.get('stamp'), now, timeout):
            return False, 'ODOMETRY_STALE'
        if not fresh(valid.get('stamp'), now, timeout) or valid.get('valid') is not True:
            return False, 'LOCALIZATION_INVALID'
        if odom.get('frame') != 'map' or odom.get('child_frame') != 'base_link':
            return False, 'ODOMETRY_FRAME_INVALID'
        if not all(math.isfinite(odom.get(k, math.nan)) for k in ('x', 'y', 'yaw', 'speed', 'position_variance', 'yaw_variance')):
            return False, 'ODOMETRY_NONFINITE'
        if not (0 <= odom['position_variance'] <= self.config.get('max_position_variance_m2', 0.25)
                and 0 <= odom['yaw_variance'] <= self.config.get('max_yaw_variance_rad2', 0.08)):
            return False, 'LOCALIZATION_UNCERTAIN'
        if not fresh(scan.get('stamp'), now, timeout) or not scan.get('valid'):
            return False, scan.get('reason', 'LIDAR_STALE_OR_INVALID')
        if abs(scan['stamp'] - odom['stamp']) > self.config.get('max_pose_scan_skew_s', 0.2):
            return False, 'POSE_LIDAR_TIME_SKEW'
        if self.vehicle_ok and (not math.isfinite(odom.get('yaw_rate', math.nan))
                or abs(odom['speed']) > self.config['vehicle']['max_speed_mps']
                or abs(odom['yaw_rate']) > self.config['vehicle']['max_yaw_rate_rps']):
            return False, 'MOTION_OUTSIDE_SCAN_CALIBRATION'
        return True, 'OK'

    def _corridor(self, points, data, direction=1):
        vehicle, scan = self.config['vehicle'], data['scan']
        pose = tuple(data['odom'][key] for key in ('x', 'y', 'yaw'))
        # A scan transformed at its first beam is not deskewed. Enlarge the
        # swept footprint by an explicit bound on possible acquisition motion,
        # including rotation of the farthest return, and localization error.
        duration = scan.get('duration', 0.0)
        rotation = min(math.pi, vehicle['max_yaw_rate_rps'] * duration)
        motion_margin = vehicle['max_speed_mps'] * duration + 2*scan.get('range_max', 0)*math.sin(rotation/2)
        localization_margin = self.config.get('localization_sigma_margin', 2.0) * math.sqrt(data['odom']['position_variance'])
        return corridor_status(points, scan['hits'], scan['rays'],
                               vehicle_width=vehicle['width_m'], front=vehicle['front_m'],
                               rear=vehicle['rear_m'], margin=vehicle.get('margin_m', 0.25) + motion_margin + localization_margin,
                               direction=direction, coverage_exclusion=footprint(pose, vehicle))

    def _parking_preview(self, data, section):
        result = {}
        if section not in (4, 9) or not self.vehicle_ok:
            return result
        kind = 't' if section == 4 else 'parallel'
        cache_key = (section, data['scan']['stamp'])
        if self.preview_cache and self.preview_cache[0] == cache_key:
            return self.preview_cache[1]
        for side, names in PARKING_ROUTES[kind].items():
            statuses = []
            for name in names:
                # Test both orientations: metadata only describes initial yaw.
                # This conservative envelope cannot replace a parking planner.
                for direction in (1, -1):
                    statuses.append(self._corridor(self.routes[name].points, data, direction)['status'])
            value = 'BLOCKED' if 'BLOCKED' in statuses else 'CLEAR' if all(s == 'CLEAR' for s in statuses) else 'UNKNOWN'
            result[side] = {'stamp': data['scan']['stamp'], 'value': value}
        self.preview_cache = (cache_key, result)
        return result

    def traffic_constraint(self, now, signal):
        route = self.tracker.current
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
        route = self.tracker.current
        start = max(0.0, self.tracker.s - 1.0)
        end = min(route.length, self.tracker.s + max(20.0, self.config.get('path_lookahead_m', 20.0)))
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

    def _crosses_virtual_stop(self, points, wall):
        if not wall or not wall['active']:
            return False
        if not wall['valid']:
            return True
        # Traffic requests use the RDDF corridor. Check segment interiors too,
        # preventing a sparse response from skipping a stop-line restriction.
        for a, b in zip(points, points[1:]):
            steps = max(1, int(math.ceil(math.hypot(b[0]-a[0], b[1]-a[1]) / .1)))
            for i in range(steps+1):
                t = i / steps
                matched = project(self.tracker.current, a[0]+t*(b[0]-a[0]), a[1]+t*(b[1]-a[1]))
                if matched is None or matched['s'] > wall['target_s'] + 1e-6:
                    return True
        return False

    def step(self, now, data, candidates):
        if self.last_time is not None and now < self.last_time:
            self.clock_fault = True
        self.last_time = now
        healthy, reason = self._health(data, now)
        if self.clock_fault or self.transition_fault:
            healthy, reason = False, self.transition_fault or 'CLOCK_REGRESSION_RESTART_REQUIRED'
        odom = data.get('odom', {})
        tracked = {'route': self.tracker.route_name, 'section': self.tracker.current.section,
                   's': self.tracker.s, 'raw_s': self.tracker.s, 'length': self.tracker.current.length,
                   'progress': self.tracker.s/self.tracker.current.length, 'at_end': False}
        if healthy:
            parking = tracked['section'] in (5, 6, 10, 11)
            tracked.update(self.tracker.update(odom['x'], odom['y'], odom['yaw'], now,
                                               check_heading=not parking))
            healthy, reason = tracked['healthy'], tracked['reason']
        if self.request is None:
            mode = 'LOCAL' if tracked['section'] == 3 else 'PARKING' if tracked['section'] in (5, 6, 10, 11) else 'RDDF'
            self._set_request(tracked['route'], mode, 1)
        selection = self.selector.evaluate(State(now, now, self.decision_id, *self.request), candidates, now)
        snapshot = dict(tracked, now=now, healthy=healthy, reason=reason,
                        speed=odom.get('speed', 0), yaw=odom.get('yaw', 0),
                        x=odom.get('x'), y=odom.get('y'),
                        calibrated=self.vehicle_ok and self.config.get('landmarks_validated') is True
                        and not self.config.get('calibration_mode', False),
                        landmarks=self.config.get('landmarks', {}),
                        signal=data.get('signal', {}), path_ready=selection.ready,
                        decision_id=self.decision_id,
                        parking_maneuver=data.get('parking_maneuver', {}),
                        parking=self._parking_preview(data, tracked['section']) if healthy else {})
        # Collision veto is independent of mission dwell accounting. In
        # particular a dummy occupying the corridor must not reset its 3 s hold.
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
            selection = self.selector.evaluate(State(now, now, self.decision_id, *self.request), candidates, now)
            decision.update(stop_requested=True, speed_limit=0.0, next_route=None, reason='WAIT_NEW_PATH', phase='WAIT_PATH')
        safety = {'stop': True, 'sensor_valid': healthy, 'reason': reason, 'clearance_m': -1.0,
                  'path_fingerprint': ''}
        if healthy and self.vehicle_ok and selection.ready:
            candidate = selection.candidate
            safety['path_fingerprint'] = path_fingerprint(candidate)
            points = []
            for pose in candidate.poses:
                qx, qy, qz, qw = pose.orientation
                yaw = math.atan2(2*(qw*qz + qx*qy), 1-2*(qy*qy+qz*qz))
                # PlannedPath pose orientations describe body heading.
                points.append((pose.position[0], pose.position[1], yaw + (math.pi if decision['direction'] < 0 else 0)))
            path = Route('1_safety_path', points, decision['direction'])
            matched = project(path, odom['x'], odom['y'])
            distance = braking_distance(odom['speed'], self.config['vehicle']['deceleration_mps2'],
                                        self.config['vehicle']['reaction_s'], self.config['vehicle'].get('margin_m', .25))
            remaining = path.length - matched['s']
            body_yaw = matched['yaw'] + (math.pi if decision['direction'] < 0 else 0)
            heading_error = abs(math.atan2(math.sin(odom['yaw']-body_yaw), math.cos(odom['yaw']-body_yaw)))
            if self._crosses_virtual_stop(points, decision.get('virtual_stop')):
                safety['reason'] = 'PATH_CROSSES_VIRTUAL_STOP'
            elif matched['distance'] > self.config.get('max_path_start_offset_m', 1.0):
                safety['reason'] = 'SELECTED_PATH_TOO_FAR'
            elif heading_error > self.config.get('max_path_heading_error_rad', 1.0):
                safety['reason'] = 'SELECTED_PATH_HEADING_MISMATCH'
            elif remaining < distance and not tracked['at_end'] and decision.get('remaining_stop_m') is None:
                safety['reason'] = 'PATH_SHORTER_THAN_STOPPING_DISTANCE'
            else:
                horizon = path.slice(matched['s'], matched['s'] + max(distance, 0.5))
                # Join measured vehicle pose to the selected path, checking
                # swept space even when a local avoidance path is offset.
                horizon.insert(0, (odom['x'], odom['y'], odom['yaw'] + (math.pi if decision['direction'] < 0 else 0)))
                check = self._corridor(horizon, data, decision['direction'])
                safety.update(stop=check['status'] != 'CLEAR', reason=check['status'] + ':' + check.get('reason', ''),
                              clearance_m=check.get('nearest_obstacle_s') if check.get('nearest_obstacle_s') is not None else -1.0)
        elif healthy:
            safety['reason'] = self.vehicle_error if not self.vehicle_ok else 'REQUESTED_PATH_UNAVAILABLE'
        remaining_stop = decision.get('remaining_stop_m')
        if remaining_stop is not None and self.vehicle_ok:
            available = max(0.0, remaining_stop - self.config.get('stop_buffer_m', 0.05))
            a, reaction = self.config['vehicle']['deceleration_mps2'], self.config['vehicle']['reaction_s']
            cap = max(0.0, math.sqrt((a*reaction)**2 + 2*a*available) - a*reaction)
            decision['speed_limit'] = min(decision['speed_limit'], cap)
        if self.vehicle_ok:
            decision['speed_limit'] = min(decision['speed_limit'], self.config['vehicle']['max_speed_mps'])
        # A route handoff also invalidates the previous planner response. The
        # next cycle acquires the successor before any new movement is allowed.
        successor = decision.get('next_route')
        if successor and not safety['stop'] and healthy:
            if self.tracker.transition(successor):
                section = self.tracker.current.section
                mode = 'LOCAL' if section == 3 else 'PARKING' if section in (5, 6, 10, 11) else 'RDDF'
                self._set_request(successor, mode, 1)
                self.parking_request = None
                tracked = dict(route=successor, section=section, s=0.0, raw_s=0.0,
                               length=self.tracker.current.length, progress=0.0, at_end=False)
                decision.update(route=successor, section=section, path_mode=mode, direction=self.request[2],
                                mission='TRANSITION', selected_branch=self.tracker.current.branch,
                                parking_leg_index=-1, parking_leg_phase='', parking_leg_target_s=-1.0,
                                stop_requested=True, emergency_stop_requested=False,
                                speed_limit=0, remaining_stop_m=None,
                                reason='ROUTE_HANDOFF', phase='HANDOFF', virtual_stop=None)
                healthy = False
            else:
                self.transition_fault = 'ROUTE_TRANSITION_REJECTED:' + self.tracker.transition_reason
                decision.update(stop_requested=True, speed_limit=0, reason=self.transition_fault)
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
