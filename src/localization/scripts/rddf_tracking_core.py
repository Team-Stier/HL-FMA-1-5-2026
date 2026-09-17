#!/usr/bin/env python3
"""Shared RDDF route identification and live-input freshness checks, without ROS."""
import math
import re


def _angle_error(first, second):
    return abs(math.atan2(math.sin(first-second), math.cos(first-second)))


def _grouped(route_map, candidate):
    source = candidate['route']
    return dict(candidate, source_route=source,
                route=route_map.route_group_names.get(source, source))


def current_rddf_members(route_map, names):
    """Expand display groups to separate CSV polylines without joining endpoints."""
    return list(dict.fromkeys(member for name in names
                             for member in route_map.route_groups.get(name, [name])))


def current_rddf_match(route_map, position, localization_valid, max_distance_m=5.0,
                       ambiguity_distance_m=0.5):
    """Identify public RDDF groups while preserving each source segment's geometry."""
    if position is None:
        return dict(accepted=False, reason='NO_FRESH_GLOBAL', routes=[],
                    text='RDDF: waiting for Global position')
    if not localization_valid:
        return dict(accepted=False, reason='LOCALIZATION_INVALID', routes=[],
                    text='RDDF: localization invalid / stale')
    result = route_map.match(position[0], position[1], max_distance_m,
                             ambiguity_distance_m)
    if 'route' in result:
        result = _grouped(route_map, result)
    if 'candidates' in result:
        result['candidates'] = [_grouped(route_map, candidate)
                                for candidate in result['candidates']]
    names = sorted({candidate['route'] for candidate in result.get('candidates', [])})
    if len(names) > 1:
        result.update(accepted=False, reason='AMBIGUOUS_ROUTE')
    elif (len(names) == 1 and names[0] in route_map.route_groups
          and result['reason'] == 'AMBIGUOUS_ROUTE'):
        # Different in/out legs still identify one parking group. Their source
        # segments/headings remain separate candidates, including at reversals.
        result.update(accepted=True, reason='MATCHED')
    result['routes'] = names
    if result['accepted']:
        result['text'] = 'RDDF: {} | {:.2f} m'.format(result['route'], result['distance'])
    elif result['reason'] == 'AMBIGUOUS_ROUTE':
        result['text'] = 'RDDF candidates: {} | {:.2f} m'.format(' / '.join(names), result['distance'])
    elif result['reason'] == 'TOO_FAR':
        result['text'] = 'RDDF: off route | nearest {}: {:.2f} m'.format(result['route'], result['distance'])
    else:
        result['text'] = 'RDDF: position unavailable'
    return result


class RddfTracker:
    """Own the active RDDF and preserve it through crossings.

    Initial acquisition may use every route.  Once acquired, ordinary matching
    stays on the active source route and can move only to a geometrically joined
    next-numbered route.  Mission behavior remains outside Localization.
    """
    def __init__(self, route_map, map_frame='map', max_distance_m=5.0,
                 ambiguity_distance_m=0.5, pose_stale_sec=0.5, valid_stale_sec=1.0,
                 heading_tolerance_rad=1.0, heading_switch_margin_rad=0.35,
                 transition_radius_m=5.0, transition_join_tolerance_m=2.5,
                 rollback_segments=2.0):
        self.route_map = route_map
        self.map_frame = map_frame.lstrip('/')
        self.maximum = float(max_distance_m)
        self.margin = float(ambiguity_distance_m)
        self.pose_stale = float(pose_stale_sec)
        self.valid_stale = float(valid_stale_sec)
        self.heading_tolerance = float(heading_tolerance_rad)
        self.heading_margin = float(heading_switch_margin_rad)
        self.transition_radius = float(transition_radius_m)
        self.join_tolerance = float(transition_join_tolerance_m)
        self.rollback_segments = float(rollback_segments)
        if (not self.map_frame or not all(math.isfinite(v) and v > 0 for v in
                (self.maximum, self.pose_stale, self.valid_stale,
                 self.heading_tolerance, self.transition_radius, self.join_tolerance))
                or not all(math.isfinite(v) and v >= 0 for v in
                           (self.margin, self.heading_margin, self.rollback_segments))):
            raise ValueError('invalid RDDF tracker configuration')
        self.pose = self.valid = self.last_now = None
        self.active_source = None
        self.active_progress = None
        self.requested_successor = None

    def _observe_time(self, now):
        if self.last_now is not None and now < self.last_now:
            self.pose = self.valid = None
            self.active_source = None
            self.active_progress = None
            self.requested_successor = None
        self.last_now = now

    def update_pose(self, x, y, stamp, received, frame_id, yaw=None):
        self._observe_time(received)
        self.pose = dict(x=x, y=y, yaw=yaw, stamp=stamp,
                         received=received, frame_id=frame_id)

    def update_valid(self, valid, received):
        self._observe_time(received)
        self.valid = (bool(valid), received)

    def update_successor_request(self, source):
        requested = source.strip() if isinstance(source, str) else None
        if (requested and self.active_source and requested != self.active_source
                and self._section(requested) == 1 and self._section(self.active_source) == 1):
            self.active_source = None
            self.active_progress = None
        self.requested_successor = requested

    @staticmethod
    def _section(name):
        match = re.match(r'(\d+)', name or '')
        return int(match.group(1)) if match else None

    @staticmethod
    def _branch(name):
        return 'left' if 'left' in name else 'right' if 'right' in name else None

    def _successors(self, source):
        section = self._section(source)
        if section is None or section >= 13:
            return []
        branch = self._branch(source)
        result = []
        for name in self.route_map.routes:
            if self._section(name) != section + 1:
                continue
            if section in (5, 10) and self._branch(name) != branch:
                continue
            result.append(name)
        return result

    def _heading_choice(self, candidates, yaw):
        if yaw is None or not math.isfinite(yaw) or not candidates:
            return None
        ranked = sorted(((_angle_error(yaw, candidate['yaw']), candidate)
                         for candidate in candidates), key=lambda item: item[0])
        if ranked[0][0] > self.heading_tolerance:
            return None
        if len(ranked) > 1 and ranked[1][0] - ranked[0][0] < self.heading_margin:
            return None
        return ranked[0][1]

    def _accepted(self, candidate, reason='MATCHED'):
        result = (dict(candidate) if 'source_route' in candidate
                  else _grouped(self.route_map, candidate))
        result.pop('_connection_at_end', None)
        result.pop('_distance_from_start', None)
        result.pop('candidates', None)
        result.pop('candidate_count', None)
        result.update(accepted=True, reason=reason, routes=[result['route']])
        result['candidates'] = [dict(result)]
        return result

    @staticmethod
    def _progress(candidate):
        return float(candidate['index']) + float(candidate.get('fraction', 0.0))

    def _initial_match(self, x, y, yaw):
        result = current_rddf_match(self.route_map, (x, y), True,
                                    self.maximum, self.margin)
        requested = next((candidate for candidate in result.get('candidates', [])
                          if candidate['source_route'] == self.requested_successor), None)
        if requested is not None:
            self.active_source = requested['source_route']
            self.active_progress = self._progress(requested)
            self.requested_successor = None
            return self._accepted(requested, 'MATCHED_BY_REQUEST')
        if result.get('accepted'):
            candidates = result.get('candidates', [])
            sources = {item['source_route'] for item in candidates}
            chosen = (self._heading_choice(candidates, yaw)
                      if len(sources) > 1 else result)
            if chosen is not None:
                self.active_source = chosen['source_route']
                self.active_progress = self._progress(chosen)
                return (self._accepted(chosen, 'MATCHED_BY_HEADING')
                        if chosen is not result else result)
            # Keep the public group match for callers without body yaw. The
            # ROS node always supplies yaw and therefore never guesses an
            # in/out source leg inside an overlapping parking group.
            if yaw is not None:
                result.update(accepted=False, reason='AMBIGUOUS_SOURCE_ROUTE')
                return result
            self.active_source = result['source_route']
            self.active_progress = self._progress(result)
            return result
        chosen = self._heading_choice(result.get('candidates', []), yaw)
        if chosen is not None:
            self.active_source = chosen['source_route']
            self.active_progress = self._progress(chosen)
            return self._accepted(chosen, 'MATCHED_BY_HEADING')
        return result

    def _joined_successor(self, x, y, yaw, active, rolled_back):
        """Select only a connected next-numbered route near its entry point."""
        current_points = self.route_map.routes[self.active_source]
        current_last_segment = len(current_points) - 2
        candidates = []
        for source in self._successors(self.active_source):
            start = self.route_map.routes[source][0]
            connection = self.route_map.match(
                float(start[0]), float(start[1]), self.join_tolerance, 0.0,
                route_name=self.active_source)
            if connection.get('distance', math.inf) > self.join_tolerance:
                continue
            distance_from_start = math.hypot(x-float(start[0]), y-float(start[1]))
            connection_at_end = (connection.get('index') == current_last_segment
                                 and connection.get('fraction', 0.0) >= 0.9)
            if distance_from_start > self.transition_radius:
                continue
            target = self.route_map.match(x, y, self.maximum, 0.0, route_name=source)
            if target.get('distance', math.inf) > self.maximum:
                continue
            target['_connection_at_end'] = connection_at_end
            target['_distance_from_start'] = distance_from_start
            candidates.append(target)
        if not candidates:
            return None
        requested = next((candidate for candidate in candidates
                          if candidate['route'] == self.requested_successor), None)
        if requested is not None:
            return requested
        if len(candidates) > 1:
            if not rolled_back and active.get('distance', 0.0) < self.margin:
                return None
            by_distance = sorted(candidates, key=lambda item: item['distance'])
            if by_distance[1]['distance']-by_distance[0]['distance'] >= self.margin:
                return by_distance[0]
            # At a shared branch entry the current-route tangent can resemble
            # one branch before the vehicle has actually selected it. Wait
            # until the pose leaves or rolls back along the current route.
            return self._heading_choice(candidates, yaw)
        if len(candidates) == 1:
            target = candidates[0]
            if target['_connection_at_end']:
                endpoint_radius = min(self.transition_radius, max(0.5, self.margin))
                return target if (rolled_back or
                                  target['_distance_from_start'] <= endpoint_radius) else None
            active_error = (_angle_error(yaw, active['yaw'])
                            if yaw is not None and math.isfinite(yaw) else math.inf)
            target_error = (_angle_error(yaw, target['yaw'])
                            if yaw is not None and math.isfinite(yaw) else math.inf)
            distance_better = target['distance'] + self.margin < active.get('distance', math.inf)
            return target if (target_error + self.heading_margin < active_error
                              or rolled_back and distance_better) else None
        return None

    def _tracked_match(self, x, y, yaw):
        active = self.route_map.match(x, y, self.maximum, self.margin,
                                      route_name=self.active_source)
        raw_progress = self._progress(active)
        rolled_back = (self.active_progress is not None
                       and raw_progress+self.rollback_segments < self.active_progress)
        successor = self._joined_successor(x, y, yaw, active, rolled_back)
        if successor is not None:
            self.active_source = successor['route']
            self.active_progress = self._progress(successor)
            self.requested_successor = None
            return self._accepted(successor, 'ROUTE_TRANSITION')
        if active.get('distance', math.inf) > self.maximum:
            return _grouped(self.route_map, active)
        if not active.get('accepted'):
            chosen = self._heading_choice(active.get('candidates', []), yaw)
            if chosen is None:
                result = _grouped(self.route_map, active)
                result['candidates'] = [_grouped(self.route_map, item)
                                        for item in active.get('candidates', [])]
                result['routes'] = [self.route_map.route_group_names.get(
                    self.active_source, self.active_source)]
                return result
            self.active_progress = max(self.active_progress or 0.0,
                                       self._progress(chosen))
            return self._accepted(chosen, 'MATCHED_BY_HEADING')
        self.active_progress = max(self.active_progress or 0.0,
                                   self._progress(active))
        return self._accepted(active)

    def evaluate(self, now):
        self._observe_time(now)
        def rejected(reason):
            return dict(accepted=False, reason=reason, routes=[])
        if self.pose is None:
            return rejected('NO_GLOBAL')
        pose = self.pose
        if not all(math.isfinite(pose[key]) for key in ('x', 'y', 'stamp', 'received')):
            return rejected('INVALID_INPUT')
        if pose['stamp'] <= 0 or pose['stamp'] > now:
            return rejected('INVALID_STAMP')
        if pose['frame_id'].lstrip('/') != self.map_frame:
            return rejected('FRAME_MISMATCH')
        if now-pose['stamp'] > self.pose_stale or now-pose['received'] > self.pose_stale:
            return rejected('STALE_GLOBAL')
        if self.valid is None or now-self.valid[1] > self.valid_stale:
            return rejected('STALE_VALID')
        if not self.valid[0]:
            return rejected('LOCALIZATION_INVALID')
        yaw = pose.get('yaw')
        if yaw is not None and not math.isfinite(yaw):
            return rejected('INVALID_INPUT')
        if self.active_source is None:
            return self._initial_match(pose['x'], pose['y'], yaw)
        return self._tracked_match(pose['x'], pose['y'], yaw)
