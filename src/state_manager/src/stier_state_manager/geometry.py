"""ROS-independent RDDF tracking and conservative, observed-space checks.

All distances use metres and all angles radians.  Callers must transform scan
geometry into the same frame as the route and reject stale sensor messages.
These 2-D checks detect returns intersecting a footprint; they do not establish
that a scanner can see every curb, nor replace physical vehicle validation.
"""

import bisect
import functools
import math
import re


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _finite(*values):
    return all(math.isfinite(value) for value in values)


class Route:
    def __init__(self, name, points, direction=1):
        if direction not in (-1, 1):
            raise ValueError("direction must be +1 or -1")
        self.name = str(name)
        self.points = [tuple(float(v) for v in p[:3]) for p in points]
        if len(self.points) < 2 or any(len(p) != 3 or not _finite(*p) for p in self.points):
            raise ValueError("a route needs at least two finite (x, y, path_yaw) points")
        self.direction = direction
        self.s = [0.0]
        for a, b in zip(self.points, self.points[1:]):
            self.s.append(self.s[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
        self.length = self.s[-1]
        if self.length <= 1e-9:
            raise ValueError("route length must be positive")
        self.start, self.end = self.points[0], self.points[-1]

    @property
    def section(self):
        match = re.match(r"(?:yongin_)?(\d+)", self.name)
        if match is None:
            raise ValueError("route name must begin with a section number")
        return int(match.group(1))

    @property
    def branch(self):
        return "left" if "left" in self.name else "right" if "right" in self.name else None

    def pose_at(self, distance):
        distance = min(self.length, max(0.0, float(distance)))
        i = min(len(self.points) - 2, max(0, bisect.bisect_right(self.s, distance) - 1))
        a, b = self.points[i], self.points[i + 1]
        length = self.s[i + 1] - self.s[i]
        t = (distance - self.s[i]) / length if length > 1e-9 else 0.0
        return (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]),
                wrap_angle(a[2] + t * wrap_angle(b[2] - a[2])))

    def slice(self, start_s=0.0, end_s=None):
        """Return interpolated endpoints and interior points in travel order."""
        start_s = min(self.length, max(0.0, start_s))
        end_s = self.length if end_s is None else min(self.length, max(start_s, end_s))
        return ([self.pose_at(start_s)] +
                [p for p, s in zip(self.points, self.s) if start_s < s < end_s] +
                [self.pose_at(end_s)])


def project(route, x, y, min_s=0.0, max_s=None):
    """Closest projection in a progress window, never across other routes."""
    upper = route.length if max_s is None else min(route.length, max_s)
    lower = max(0.0, min_s)
    best = None
    for i, (a, b) in enumerate(zip(route.points, route.points[1:])):
        length = route.s[i + 1] - route.s[i]
        if length <= 1e-9 or route.s[i + 1] < lower or route.s[i] > upper:
            continue
        t = ((x - a[0]) * (b[0] - a[0]) + (y - a[1]) * (b[1] - a[1])) / length ** 2
        lo, hi = max(0.0, (lower - route.s[i]) / length), min(1.0, (upper - route.s[i]) / length)
        t = max(lo, min(hi, t))
        px, py = a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])
        distance = math.hypot(x - px, y - py)
        candidate = {"s": route.s[i] + t * length, "distance": distance,
                     "yaw": wrap_angle(a[2] + t * wrap_angle(b[2] - a[2])),
                     "x": px, "y": py, "segment": i}
        if best is None or distance < best["distance"] - 1e-9:
            best = candidate
    return best


def braking_distance(speed_mps, deceleration_mps2, reaction_s, margin_m=0.0):
    if not _finite(speed_mps, deceleration_mps2, reaction_s, margin_m):
        raise ValueError("braking inputs must be finite")
    if deceleration_mps2 <= 0 or reaction_s < 0 or margin_m < 0:
        raise ValueError("deceleration must be positive; reaction and margin nonnegative")
    speed = abs(speed_mps)
    return speed * speed / (2.0 * deceleration_mps2) + reaction_s * speed + margin_m


braking_clearance = braking_distance


class FreeRays(list):
    """Free ray list retaining beam spacing so missing returns stay unknown."""

    def __init__(self, max_gap_rad, min_range=0.0):
        super().__init__()
        self.max_gap_rad = max_gap_rad
        self.min_range = min_range
        self.beam_indices = []
        self.beam_count = 0
        self.wrap_scan = False


def scan_to_geometry(ranges, angle_min, angle_increment, range_min, range_max,
                     scanner_pose=(0.0, 0.0, 0.0), max_range=None):
    """Build map-frame hits and observed-free rays from a validated scan.

    NaN/-Inf/zero/below-minimum returns contribute no coverage. +Inf is a
    no-return ray only when the sensor's maximum range is finite and valid.
    Range caps restrict evidence, rather than manufacturing obstacle returns.
    """
    ox, oy, yaw = scanner_pose
    if (not _finite(ox, oy, yaw, angle_min, angle_increment, range_min, range_max)
            or range_min < 0 or range_max <= range_min or angle_increment == 0):
        return [], []
    bound = range_max if max_range is None else min(range_max, max_range)
    if not math.isfinite(bound) or bound <= range_min:
        return [], []
    hits, rays = [], FreeRays(abs(angle_increment) * 1.5, range_min)
    rays.beam_count = len(ranges)
    rays.wrap_scan = abs(angle_increment) * len(ranges) >= 2.0 * math.pi - abs(angle_increment) * 1.5
    for i, distance in enumerate(ranges):
        if math.isnan(distance) or distance == -math.inf or distance <= 0 or distance < range_min:
            continue
        if math.isfinite(distance) and distance > range_max:
            continue
        angle = yaw + angle_min + i * angle_increment
        observed = min(distance, bound)
        ex, ey = ox + observed * math.cos(angle), oy + observed * math.sin(angle)
        rays.append((ox, oy, ex, ey))
        rays.beam_indices.append(i)
        if math.isfinite(distance) and distance <= bound:
            hits.append((ex, ey))
    return hits, rays


def transform_scan_to_geometry(ranges, angle_min, angle_increment, range_min, range_max,
                               translation=(0.0, 0.0, 0.0), rotation=(0.0, 0.0, 0.0, 1.0),
                               max_range=None):
    """Project a full 3-D TF transform of planar laser rays into map XY.

    Quaternion order is (x, y, z, w). The caller supplies a transform at the scan
    timestamp and must independently validate scan-plane height/tilt visibility.
    rays.origin_z, rays.endpoint_z and rays.plane_normal expose that geometry;
    a point's XY clearance alone cannot establish visibility of a low curb.
    """
    if len(translation) != 3 or len(rotation) != 4 or not _finite(*translation, *rotation):
        return [], []
    norm = math.sqrt(sum(q * q for q in rotation))
    if norm <= 1e-12:
        return [], []
    qx, qy, qz, qw = (q / norm for q in rotation)
    tx, ty, tz = translation
    r00, r01 = 1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw)
    r10, r11 = 2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz)
    r20, r21 = 2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw)
    hits, local_rays = scan_to_geometry(ranges, angle_min, angle_increment, range_min,
                                        range_max, max_range=max_range)
    if not isinstance(local_rays, FreeRays):
        return [], []
    rays = FreeRays(local_rays.max_gap_rad, local_rays.min_range)
    rays.beam_indices = list(local_rays.beam_indices)
    rays.beam_count, rays.wrap_scan = local_rays.beam_count, local_rays.wrap_scan
    rays.origin_z = tz
    rays.endpoint_z = []
    rays.plane_normal = (2 * (qx * qz + qy * qw), 2 * (qy * qz - qx * qw),
                         1 - 2 * (qx * qx + qy * qy))
    for _, _, x, y in local_rays:
        rays.append((tx, ty, tx + r00 * x + r01 * y, ty + r10 * x + r11 * y))
        rays.endpoint_z.append(tz + r20 * x + r21 * y)
    hits = [(tx + r00 * x + r01 * y, ty + r10 * x + r11 * y) for x, y in hits]
    return hits, rays


def _cross(a, b, c):
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _hull(points):
    points = sorted(set(points))
    if len(points) <= 1:
        return points
    lower, upper = [], []
    for p in points:
        while len(lower) >= 2 and _cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(points):
        while len(upper) >= 2 and _cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def _inside(point, polygon):
    return all(_cross(a, b, point) >= -1e-8
               for a, b in zip(polygon, polygon[1:] + polygon[:1]))


def _area(polygon):
    return abs(sum(a[0] * b[1] - a[1] * b[0]
                   for a, b in zip(polygon, polygon[1:] + polygon[:1]))) * 0.5


def _clip(subject, clip_polygon):
    """Intersection of convex counterclockwise polygons."""
    output = list(subject)
    for a, b in zip(clip_polygon, clip_polygon[1:] + clip_polygon[:1]):
        if not output:
            return []
        incoming, output = output, []
        previous = incoming[-1]
        previous_side = _cross(a, b, previous)
        for current in incoming:
            current_side = _cross(a, b, current)
            current_inside, previous_inside = current_side >= -1e-10, previous_side >= -1e-10
            if current_inside != previous_inside:
                ratio = previous_side / (previous_side - current_side)
                output.append((previous[0] + ratio * (current[0] - previous[0]),
                               previous[1] + ratio * (current[1] - previous[1])))
            if current_inside:
                output.append(current)
            previous, previous_side = current, current_side
    return output


def _rectangle(point, half_width, front, rear, direction):
    x, y, yaw = point
    yaw += math.pi if direction == -1 else 0.0
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + lon * c - lat * s, y + lon * s + lat * c)
            for lon, lat in ((-rear, -half_width), (front, -half_width),
                             (front, half_width), (-rear, half_width))]


class _FreeSpace:
    def __init__(self, rays, max_gap):
        grouped = {}
        beam_indices = getattr(rays, "beam_indices", [])
        beam_count, wrap_scan = getattr(rays, "beam_count", 0), getattr(rays, "wrap_scan", False)
        for i, (ox, oy, ex, ey) in enumerate(rays):
            if not _finite(ox, oy, ex, ey) or math.hypot(ex - ox, ey - oy) <= 1e-8:
                continue
            beam_id = beam_indices[i] if len(beam_indices) == len(rays) else None
            grouped.setdefault((ox, oy), []).append((math.atan2(ey - oy, ex - ox), (ex, ey), beam_id))
        self.groups = []
        for origin, ends in grouped.items():
            ends.sort(key=lambda item: item[0])
            if len(ends) < 2:
                continue
            angles = [item[0] for item in ends]
            self.groups.append((origin, ends, angles))
        self.max_gap = min(max_gap, getattr(rays, "max_gap_rad", max_gap))
        self.min_range = getattr(rays, "min_range", 0.0)
        self.regions = []
        for origin, ends, _ in self.groups:
            group_regions = []
            for left, right in zip(ends, ends[1:] + ends[:1]):
                gap = (right[0] - left[0]) % (2.0 * math.pi)
                if gap <= 1e-9 or gap > self.max_gap:
                    continue
                if left[2] is not None and right[2] is not None:
                    separation = abs(left[2] - right[2])
                    if separation != 1 and not (wrap_scan and separation == beam_count - 1):
                        continue
                # A chord outside the near-range blind circle is conservative:
                # the quadrilateral never credits evidence inside range_min.
                inner_radius = self.min_range / math.cos(gap / 2.0)
                if any(math.hypot(e[1][0] - origin[0], e[1][1] - origin[1]) <= inner_radius for e in (left, right)):
                    continue
                if inner_radius:
                    inner_left = (origin[0] + inner_radius * math.cos(left[0]), origin[1] + inner_radius * math.sin(left[0]))
                    inner_right = (origin[0] + inner_radius * math.cos(right[0]), origin[1] + inner_radius * math.sin(right[0]))
                    region = [inner_left, left[1], right[1], inner_right]
                else:
                    region = [origin, left[1], right[1]]
                box = (min(p[0] for p in region), max(p[0] for p in region),
                       min(p[1] for p in region), max(p[1] for p in region))
                group_regions.append((region, box))
            self.regions.append(group_regions)

    def coverage(self, polygon, excluded):
        """Continuous area coverage, so even sub-grid blind wedges stay unknown.

        Fan regions for a single scanner origin do not overlap in area. For
        multiple origins require one origin to cover each swept polygon, avoiding
        an incorrect sum that credits overlapping observations twice.
        """
        area = _area(polygon)
        excluded_area = _area(_clip(polygon, excluded)) if excluded else 0.0
        required = max(0.0, area - excluded_area)
        if required <= 1e-10:
            return 1.0
        box = (min(p[0] for p in polygon), max(p[0] for p in polygon),
               min(p[1] for p in polygon), max(p[1] for p in polygon))
        best = 0.0
        for group in self.regions:
            covered = 0.0
            for region, rbox in group:
                if rbox[1] < box[0] or rbox[0] > box[1] or rbox[3] < box[2] or rbox[2] > box[3]:
                    continue
                overlap = _clip(polygon, region)
                if len(overlap) < 3:
                    continue
                observed = _area(overlap)
                if excluded:
                    observed -= _area(_clip(overlap, excluded))
                covered += max(0.0, observed)
            best = max(best, min(1.0, covered / required))
        return best

def _prepared_free_space(rays, max_gap):
    # Cache on one scan object, verifying content/metadata before reuse.
    if isinstance(rays, FreeRays):
        key = (tuple(rays), tuple(rays.beam_indices), rays.beam_count, rays.wrap_scan,
               rays.min_range, rays.max_gap_rad, max_gap)
        cached = getattr(rays, '_prepared', None)
        if cached is not None and cached[0] == key:
            return cached[1]
        prepared = _FreeSpace(rays, max_gap)
        rays._prepared = (key, prepared)
        return prepared
    return _FreeSpace(rays, max_gap)


@functools.lru_cache(maxsize=64)
def _swept_polygons(points, vehicle_width, margin, front, rear, sample_step, direction, chunk_m):
    poses = [(points[0], 0.0)]
    distance = 0.0
    for a, b in zip(points, points[1:]):
        segment = math.hypot(b[0] - a[0], b[1] - a[1])
        turn = wrap_angle(b[2] - a[2])
        count = max(1, int(math.ceil(segment / sample_step)), int(math.ceil(abs(turn) / 0.08)))
        for i in range(1, count + 1):
            t = i / float(count)
            poses.append(((a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]),
                           wrap_angle(a[2] + t * turn)), distance + t * segment))
        distance += segment
    half_width = vehicle_width / 2.0 + margin
    rectangles = [_rectangle(pose, half_width, front + margin, rear + margin, direction)
                  for pose, _ in poses]
    # A chunk hull contains every small consecutive sweep. It may conservatively
    # veto a tight bend, but cannot discard a detected collision.
    polygons, start = [], 0
    while start < len(poses):
        end = start
        while end + 1 < len(poses) and poses[end + 1][1] - poses[start][1] <= chunk_m:
            end += 1
        if end == start and end + 1 < len(poses):
            end += 1
        polygon = _hull([p for rectangle in rectangles[start:end + 1] for p in rectangle])
        box = (min(p[0] for p in polygon), max(p[0] for p in polygon),
               min(p[1] for p in polygon), max(p[1] for p in polygon))
        polygons.append((polygon, poses[start][1], box))
        if end == len(poses) - 1:
            break
        start = end
    return polygons


def corridor_status(route_points, hits, rays, vehicle_width=1.2, margin=0.25,
                    front=1.5, rear=0.6, sample_step=0.25, direction=1,
                    max_ray_gap_rad=math.radians(1.5), coverage_exclusion=None,
                    sweep_chunk_m=2.0):
    """Check swept rectangles against returns and actually observed free space.

    ``route_points`` are (x, y, path_yaw) in travel order; use ``direction=-1``
    for reverse. ``rays`` are (origin_x, origin_y, free_end_x, free_end_y).
    Adjacent valid rays form free-space triangles only below max_ray_gap_rad;
    missing rays, out-of-view areas and occlusions therefore stay UNKNOWN.
    CLEAR requires continuous area coverage of every swept-footprint polygon.
    A finite hit in the swept footprint wins even when coverage is incomplete.
    ``coverage_exclusion`` may be a calibrated current chassis polygon in the
    map frame: its interior needs no free-space evidence. It never excludes
    obstacle hits and must not include the future path or clearance margin.
    ``sweep_chunk_m`` bounds conservative convex-hull groups. Smaller values
    reduce false vetoes in tight bends at increased cost. Returned obstacle s
    is a conservative chunk-start lower bound, not an exact bumper distance.
    """
    result = {"status": "UNKNOWN", "reason": "invalid_geometry", "coverage": 0.0,
              "nearest_obstacle_s": None}
    if (not _finite(vehicle_width, margin, front, rear, sample_step, max_ray_gap_rad, sweep_chunk_m)
            or vehicle_width <= 0 or min(margin, front, rear) < 0 or front + rear <= 0 or sample_step <= 0
            or max_ray_gap_rad <= 0 or max_ray_gap_rad >= math.pi or sweep_chunk_m <= 0 or direction not in (-1, 1)):
        return result
    points = [tuple(p[:3]) for p in route_points]
    if not points or any(len(p) != 3 or not _finite(*p) for p in points):
        return result
    if coverage_exclusion is not None:
        if (len(coverage_exclusion) < 3
                or any(len(p) != 2 or not _finite(*p) for p in coverage_exclusion)
                or _area(_hull(coverage_exclusion)) <= 1e-8):
            return result
    polygons = _swept_polygons(tuple(points), vehicle_width, margin, front, rear,
                              sample_step, direction, sweep_chunk_m)
    finite_hits = [p[:2] for p in hits if len(p) >= 2 and _finite(*p[:2])]
    excluded = _hull(coverage_exclusion) if coverage_exclusion else []
    # Complete the collision pass before early-returning UNKNOWN: unknown near
    # space must not hide a known obstacle later in the manoeuvre.
    for polygon, route_s, (min_x, max_x, min_y, max_y) in polygons:
        if any(min_x <= p[0] <= max_x and min_y <= p[1] <= max_y and _inside(p, polygon)
               for p in finite_hits):
            result.update(status="BLOCKED", reason="return_inside_swept_footprint", nearest_obstacle_s=route_s)
            return result
    free = _prepared_free_space(rays, max_ray_gap_rad)
    for polygon, _, _ in polygons:
        coverage = free.coverage(polygon, excluded)
        if coverage < 1.0 - 1e-8:
            result.update(reason="insufficient_free_space_coverage", coverage=coverage)
            return result
    result.update(status="CLEAR", reason="swept_footprint_observed_free", coverage=1.0)
    return result
