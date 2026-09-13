"""Pure, bounded replay geometry. No ROS, TF, hardware or file writes."""
import csv
import json
import math
from collections import OrderedDict, deque
from dataclasses import dataclass
from pathlib import Path

DEFAULT_ORIGIN = (37.288731, 127.1072336)
MAX_ROUTES = 100
MAX_POINTS = 200000
MAX_FILE_BYTES = 32 * 1024 * 1024
WGS84_A = 6378137.0
WGS84_E2 = 6.6943799901413165e-3


@dataclass(frozen=True)
class Route:
    id: str
    name: str
    closed: bool
    points: tuple


@dataclass(frozen=True)
class RouteProject:
    origin: tuple
    routes: tuple


def finite_number(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(label + ': a finite number is required')
    return float(value)


def coordinate(lat, lon):
    lat, lon = finite_number(lat, 'latitude'), finite_number(lon, 'longitude')
    if not -90 <= lat <= 90 or not -180 <= lon <= 180:
        raise ValueError('Latitude/longitude is outside WGS84 ranges')
    return lat, lon


def origin_coordinate(origin):
    lat, lon = coordinate(*origin)
    if abs(lat) >= 89.999:
        raise ValueError('Local EN projection does not support a polar origin')
    return lat, lon


def project_point(lat, lon, origin=DEFAULT_ORIGIN):
    """Same WGS84 curvature approximation as the satellite route editor."""
    lat, lon = coordinate(lat, lon)
    lat0, lon0 = origin_coordinate(origin)
    phi = math.radians(lat0)
    denominator = 1 - WGS84_E2 * math.sin(phi) ** 2
    east = math.radians(lon - lon0) * WGS84_A / math.sqrt(denominator) * math.cos(phi)
    north = math.radians(lat - lat0) * WGS84_A * (1 - WGS84_E2) / denominator ** 1.5
    return east, north


def _route_identity(identifier, name):
    if not isinstance(identifier, str) or not identifier.strip() or len(identifier) > 200:
        raise ValueError('Route id must contain 1 to 200 characters')
    if not isinstance(name, str) or len(name) > 200:
        raise ValueError('Route name must contain at most 200 characters')
    return identifier, name


def _json_routes(value):
    if not isinstance(value, dict) or value.get('version') != 1:
        raise ValueError('Expected a version 1 route-editor project JSON')
    datum = value.get('origin')
    if not isinstance(datum, dict):
        raise ValueError('Project JSON must provide its geographic origin')
    origin = origin_coordinate((datum.get('lat'), datum.get('lng')))
    raw_routes = value.get('routes')
    if not isinstance(raw_routes, list) or len(raw_routes) > MAX_ROUTES:
        raise ValueError('Project must contain at most 100 routes')
    if 'spacingM' in value and not .1 <= finite_number(value['spacingM'], 'spacingM') <= 20:
        raise ValueError('Project spacingM must be between 0.1 and 20')
    routes, seen, count = [], set(), 0
    for item in raw_routes:
        if not isinstance(item, dict):
            raise ValueError('Each route must be an object')
        identifier, name = _route_identity(item.get('id'), item.get('name'))
        if identifier in seen:
            raise ValueError('Duplicate route id: ' + identifier)
        seen.add(identifier)
        raw_points = item.get('points')
        if type(item.get('closed')) is not bool or not isinstance(raw_points, list):
            raise ValueError('Route requires boolean closed and a points array')
        count += len(raw_points)
        if count > MAX_POINTS:
            raise ValueError('Route point count exceeds 200000')
        points = []
        for point in raw_points:
            if not isinstance(point, dict):
                raise ValueError('Each point requires lat and lng')
            points.append(coordinate(point.get('lat'), point.get('lng')))
        routes.append(Route(identifier, name, item['closed'], tuple(points)))
    return RouteProject(origin, tuple(routes))


def _csv_routes(stream, origin):
    reader = csv.DictReader(stream)
    required = {'route_id', 'route_name', 'closed', 'index', 'latitude', 'longitude'}
    if (not reader.fieldnames or not required.issubset(reader.fieldnames)
            or len(set(reader.fieldnames)) != len(reader.fieldnames)):
        raise ValueError('Expected route-editor CSV with latitude/longitude; UTM RDDF text is not supported')
    groups, count = OrderedDict(), 0
    for row in reader:
        if None in row or any(row.get(key) is None for key in required):
            raise ValueError('Malformed CSV row {}'.format(reader.line_num))
        identifier, name = _route_identity(row['route_id'], row['route_name'])
        closed_text = row['closed'].strip().lower()
        if closed_text not in ('true', 'false'):
            raise ValueError('CSV closed must be true or false')
        closed = closed_text == 'true'
        if identifier not in groups:
            if len(groups) >= MAX_ROUTES:
                raise ValueError('CSV exceeds 100 routes')
            groups[identifier] = {'name': name, 'closed': closed, 'points': []}
        route = groups[identifier]
        if route['name'] != name or route['closed'] != closed:
            raise ValueError('Inconsistent CSV metadata for route ' + identifier)
        index = row['index'].strip()
        if not index.isascii() or not index.isdigit() or int(index) != len(route['points']):
            raise ValueError('CSV indices must start at 0 and follow traversal order for each route')
        try:
            point = coordinate(float(row['latitude']), float(row['longitude']))
        except (ValueError, TypeError) as error:
            raise ValueError('Invalid CSV coordinate at row {}'.format(reader.line_num)) from error
        count += 1
        if count > MAX_POINTS:
            raise ValueError('CSV point count exceeds 200000')
        route['points'].append(point)
    return RouteProject(origin_coordinate(origin), tuple(
        Route(identifier, item['name'], item['closed'], tuple(item['points']))
        for identifier, item in groups.items()))


def load_routes(path, csv_origin=DEFAULT_ORIGIN):
    """Parse the complete file before returning; caller state is never mutated."""
    path = Path(path).expanduser()
    try:
        if path.stat().st_size > MAX_FILE_BYTES:
            raise ValueError('Route file exceeds 32 MiB')
        with path.open('r', encoding='utf-8-sig', newline='') as stream:
            if path.suffix.lower() == '.json':
                return _json_routes(json.load(stream))
            if path.suffix.lower() == '.csv':
                return _csv_routes(stream, csv_origin)
            raise ValueError('route_file must be an editor .json or .csv file')
    except (OSError, UnicodeError, csv.Error) as error:
        raise ValueError('Unable to read route file: ' + str(error)) from error


@dataclass(frozen=True)
class GpsSample:
    lat: object
    lon: object
    status: str
    hacc_m: object
    satellites: int
    valid: bool


def navpvt_sample(message):
    """Coordinate and quality come from the same NavPVT epoch, not cached status."""
    try:
        lat, lon = coordinate(message.lat * 1e-7, message.lon * 1e-7)
    except (AttributeError, TypeError, ValueError, OverflowError):
        lat = lon = None
    flags, fix_type = getattr(message, 'flags', 0), getattr(message, 'fixType', 0)
    valid = lat is not None and bool(flags & 1) and fix_type in (2, 3, 4)
    carrier = (flags >> 6) & 3
    status = ('NO_FIX' if not valid else '2D_FIX' if fix_type == 2 else 'RTK_FIXED' if carrier == 2 else
              'RTK_FLOAT' if carrier == 1 else 'UNKNOWN' if carrier == 3 else
              'DGNSS' if flags & 2 else 'GNSS_DR' if fix_type == 4 else '3D_FIX')
    hacc = getattr(message, 'hAcc', None)
    hacc = hacc * .001 if isinstance(hacc, (int, float)) and math.isfinite(hacc) and hacc >= 0 else None
    return GpsSample(lat, lon, status, hacc, int(getattr(message, 'numSV', 0)), valid)


@dataclass(frozen=True)
class TrackPoint:
    stamp: float
    x: float
    y: float
    status: str
    segment: int


class GpsTrack:
    def __init__(self, origin=DEFAULT_ORIGIN, max_points=20000, gap_seconds=2., max_step_m=50.):
        self.origin = origin_coordinate(origin)
        if not isinstance(max_points, int) or isinstance(max_points, bool) or not 2 <= max_points <= MAX_POINTS:
            raise ValueError('max_points must be an integer from 2 to 200000')
        self.gap_seconds = finite_number(gap_seconds, 'gap_seconds')
        self.max_step_m = finite_number(max_step_m, 'max_step_m')
        if self.gap_seconds <= 0 or self.max_step_m <= 0:
            raise ValueError('Gap and jump thresholds must be positive')
        self.points = deque(maxlen=max_points)
        self.current = None
        self._clock = None
        self._previous = None
        self._segment = 0
        self.reset_count = 0

    def clock(self, stamp):
        stamp = finite_number(stamp, 'time')
        if stamp < 0:
            raise ValueError('Time cannot be negative')
        reset = self._clock is not None and stamp < self._clock - 1e-9
        if reset:
            self.points.clear()
            self.current = None
            self._previous = None
            self._segment += 1
            self.reset_count += 1
        self._clock = stamp
        return reset

    def add(self, message, stamp):
        self.clock(stamp)
        sample = navpvt_sample(message)
        self.current = sample
        if not sample.valid:
            self._previous = None
            self._segment += 1
            return sample
        x, y = project_point(sample.lat, sample.lon, self.origin)
        previous = self._previous
        if previous:
            if stamp == previous.stamp and x == previous.x and y == previous.y:
                return sample
            if stamp - previous.stamp > self.gap_seconds or math.hypot(x - previous.x, y - previous.y) > self.max_step_m:
                self._segment += 1
        point = TrackPoint(stamp, x, y, sample.status, self._segment)
        self.points.append(point)
        self._previous = point
        return sample
