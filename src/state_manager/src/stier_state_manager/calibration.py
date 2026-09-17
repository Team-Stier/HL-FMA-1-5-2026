"""Measured mission landmarks; JSON storage and checks have no ROS dependency.

Each ``*_s`` is metres along the named RDDF at its tracking reference.
``parking_confirm_s`` is the position of base_link when the rear axle reaches
the measured parking confirmation line. The current vehicle model places
base_link at the rear axle; a different reference needs an explicitly measured
offset, not a click on the physical line treated as the body centre.
``hill_start_s``/``hill_top_s`` locate the actual ramp boundaries, and
``hill_stop_s`` locates the rear axle at its intended stop at least 1 m inside
both boundaries. Alternatively hill_zone_start_s/hill_zone_end_s mark the
permitted stop-zone boundaries directly; their arc-length midpoint is the
rear-axle target, without another one-metre inset. Traffic stop lines refer to the physical line; the mission's
front-bumper offset controls how far before that line base_link stops.

Clicks set distances only. Parking body yaw must be independently measured and
entered as ``landmarks[route]['parking_yaw_rad']`` in radians. Calling validate
is the operator's explicit confirmation of these physical reference meanings;
geometry validation alone cannot identify painted lines or ramp boundaries.
"""

import copy
import fcntl
import hashlib
import json
import math
import os
import tempfile
from pathlib import Path

from .geometry import project
from .mission import REQUIRED_LANDMARKS, HILL_ZONE_KEYS, landmark_keys, hill_target


DEFAULT_END_TOLERANCE_M = 0.8
DEFAULT_TRANSITION_JOIN_TOLERANCE_M = 2.5


def number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def merge_overlay(base, overlay):
    """Merge calibration overrides without discarding base rules or metadata."""
    if not isinstance(base, dict) or not isinstance(overlay, dict):
        raise ValueError('Base configuration and calibration overlay must be objects')
    result = copy.deepcopy(base)
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = merge_overlay(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def route_catalogue_digest(routes):
    """Stable fingerprint of sorted RDDF names, direction and full geometry."""
    data = [[name, route.direction, route.points] for name, route in sorted(routes.items())]
    encoded = json.dumps(data, ensure_ascii=False, separators=(',', ':'), allow_nan=False).encode('utf-8')
    return hashlib.sha256(encoded).hexdigest()


def target_route(routes, route_name, landmark):
    if route_name not in routes:
        raise ValueError('Unknown route: ' + str(route_name))
    route = routes[route_name]
    if landmark not in (REQUIRED_LANDMARKS.get(route.section, ()) + (HILL_ZONE_KEYS if route.section == 1 else ())):
        raise ValueError('Landmark is not a click target for this section: ' + str(landmark))
    return route


def snap_landmark(routes, route_name, landmark, east_m, north_m, max_distance_m=2.0):
    """Snap to the nearest actual CSV waypoint; index is zero based.

    This deliberately does not manufacture an interpolated CSV index. If two
    points are equally close the earliest point wins; review a self-crossing
    route in RViz before accepting a target near coincident points.
    """
    route = target_route(routes, route_name, landmark)
    if not all(number(v) for v in (east_m, north_m, max_distance_m)) or max_distance_m <= 0:
        raise ValueError('Click coordinates and maximum distance must be finite')
    index = min(range(len(route.points)), key=lambda i: math.hypot(
        route.points[i][0] - east_m, route.points[i][1] - north_m))
    x, y, _ = route.points[index]
    snap_error = math.hypot(x - east_m, y - north_m)
    if snap_error > max_distance_m:
        raise ValueError('Click is {:.3f} m from the route; limit is {:.3f} m'.format(
            snap_error, max_distance_m))
    return {'index': index, 'east_m': x, 'north_m': y, 'distance_m': route.s[index]}


def record_landmark(config, routes, route_name, landmark, point):
    """Return a new config preserving unrelated fields and revoking validation."""
    target_route(routes, route_name, landmark)
    if not isinstance(config, dict):
        raise ValueError('Calibration JSON must be an object')
    route = routes[route_name]
    index = point.get('index')
    if (type(index) is not int or index < 0 or index >= len(route.points)
            or not all(number(point.get(key)) for key in ('east_m', 'north_m', 'distance_m'))):
        raise ValueError('Invalid snapped landmark metadata')
    if (abs(point['distance_m'] - route.s[index]) > 1e-6
            or math.hypot(point['east_m'] - route.points[index][0],
                          point['north_m'] - route.points[index][1]) > 1e-6):
        raise ValueError('Snapped landmark does not match the route index')
    result = copy.deepcopy(config)
    for name in ('landmarks', 'landmark_points'):
        if name not in result:
            result[name] = {}
        if not isinstance(result[name], dict):
            raise ValueError(name + ' must be an object')
        if route_name not in result[name]:
            result[name][route_name] = {}
        if not isinstance(result[name][route_name], dict):
            raise ValueError(name + '.' + route_name + ' must be an object')
    result['landmarks'][route_name][landmark] = float(point['distance_m'])
    result['landmark_points'][route_name][landmark] = dict(point)
    result['landmarks_validated'] = False
    return result


def validate_landmarks(config, routes, explicit_validation=False):
    """Return every discovered error; never mark the document valid implicitly."""
    errors = []
    if not isinstance(config, dict):
        return ['Calibration JSON must be an object']
    if not routes:
        return ['Route catalogue is unavailable']
    if config.get('landmarks_validated') is True and not explicit_validation:
        if config.get('route_catalogue_digest') != route_catalogue_digest(routes):
            errors.append('Validated calibration is not bound to this RDDF catalogue; explicit recalibration is required')
    marks = config.get('landmarks', {})
    if not isinstance(marks, dict):
        return ['landmarks must be an object']
    records = config.get('landmark_points', {})
    if not isinstance(records, dict):
        errors.append('landmark_points must be an object')
    else:
        for name, entries in records.items():
            if name not in routes or not isinstance(entries, dict):
                errors.append('landmark_points has an unknown route or invalid object: ' + str(name))
                continue
            route = routes[name]
            for key, point in entries.items():
                label = name + '.' + str(key)
                if key not in (REQUIRED_LANDMARKS.get(route.section, ()) + (HILL_ZONE_KEYS if route.section == 1 else ())) or not isinstance(point, dict):
                    errors.append(label + ': invalid landmark_points entry')
                    continue
                index = point.get('index')
                values = marks.get(name, {})
                distance = values.get(key) if isinstance(values, dict) else None
                if (type(index) is not int or not 0 <= index < len(route.points)
                        or not number(distance)
                        or not all(number(point.get(k)) for k in ('east_m', 'north_m', 'distance_m'))):
                    errors.append(label + ': invalid landmark_points index/coordinates/distance')
                    continue
                expected = route.points[index]
                if (math.hypot(point['east_m'] - expected[0], point['north_m'] - expected[1]) > 1e-5
                        or abs(point['distance_m'] - route.s[index]) > 1e-5
                        or abs(distance - point['distance_m']) > 1e-5):
                    errors.append(label + ': RDDF or landmark changed; remeasure this recorded point')
    tracker = config.get('tracker', {})
    if not isinstance(tracker, dict):
        return ['tracker must be an object']
    rules = config.get('rules', {})
    if not isinstance(rules, dict):
        return ['rules must be an object']
    tolerance = tracker.get('end_tolerance_m', DEFAULT_END_TOLERANCE_M)
    if not number(tolerance) or tolerance <= 0 or tolerance > 0.8:
        errors.append('tracker.end_tolerance_m must be within (0, 0.8] m')
        tolerance = 0.8
    for name in marks:
        if name not in routes:
            errors.append('Unknown landmark route: ' + str(name))
    for name, route in routes.items():
        required = REQUIRED_LANDMARKS.get(route.section, ())
        values = marks.get(name, {})
        if not isinstance(values, dict):
            errors.append(name + ': landmarks must be an object')
            continue
        required = landmark_keys(route.section, values)
        allowed = set(REQUIRED_LANDMARKS.get(route.section, ()))
        if route.section == 1:
            allowed.update(HILL_ZONE_KEYS)
        if route.section == 5:
            # Legacy values remain accepted but T parking now stops at the
            # recorded RDDF endpoint and needs no measured parking landmark.
            allowed.update(('parking_confirm_s', 'parking_yaw_rad'))
        if route.section == 6:
            allowed.add('parking_exit_s')
        for key in values:
            if key not in allowed:
                errors.append(name + ': unknown landmark ' + str(key))
        for key in required:
            value = values.get(key)
            if not number(value):
                errors.append(name + ': missing/nonfinite ' + key)
            elif value < 0 or value > route.length:
                errors.append(name + ': ' + key + ' is outside the route')
        complete = all(number(values.get(key)) and 0 <= values[key] <= route.length
                       for key in required)
        if not complete:
            continue
        if route.section == 1:
            try:
                hill_target(values)
            except ValueError as error:
                errors.append(name + ': hill stop must be at least 1 m inside both ramp boundaries; ' + str(error))
        if route.section == 12:
            left = routes.get('13_left')
            if left is None:
                errors.append(name + ': 13_left route is required to verify the fork')
            else:
                match = project(route, left.start[0], left.start[1])
                join = tracker.get('transition_join_tolerance_m',
                                   DEFAULT_TRANSITION_JOIN_TOLERANCE_M)
                if not number(join) or join <= 0:
                    errors.append('tracker.transition_join_tolerance_m must be positive')
                elif match is None or match['distance'] > join:
                    errors.append(name + ': 13_left fork does not match RDDF geometry')
    return errors


def atomic_update(filename, transform):
    """Read/modify/replace JSON under a sidecar lock; errors preserve the file."""
    path = Path(filename).expanduser()
    if not path.name or path.is_symlink():
        raise ValueError('Calibration file must be a regular file, not a symlink')
    path.parent.mkdir(parents=True, exist_ok=True)
    with (path.parent / (path.name + '.lock')).open('a', encoding='utf-8') as lock:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX)
        if path.exists():
            with path.open(encoding='utf-8') as stream:
                config = json.load(stream)
            if not isinstance(config, dict):
                raise ValueError('Calibration JSON must be an object')
        else:
            config = {}
        result = transform(config)
        if not isinstance(result, dict):
            raise ValueError('Calibration update must return an object')
        payload = json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False) + '\n'
        fd, temporary = tempfile.mkstemp(prefix='.' + path.name + '.', suffix='.tmp', dir=str(path.parent))
        try:
            with os.fdopen(fd, 'w', encoding='utf-8') as stream:
                stream.write(payload)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, path)
        except BaseException:
            if os.path.exists(temporary):
                os.unlink(temporary)
            raise
        return result
