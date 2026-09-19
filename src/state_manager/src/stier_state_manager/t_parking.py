"""Select a T-parking branch from DBSCAN entrance occupancy."""
import bisect
import math


class TParkingSelector:
    def __init__(self, routes, config):
        self.config = config
        self.rois = {}
        for side, distance in config['entry_s'].items():
            route = routes['5_T-' + side + '-in']
            x, y, _ = route.pose_at(distance)
            index = bisect.bisect_right(route.s, distance) - 1
            a, b = route.points[index:index + 2]
            yaw = math.atan2(b[1] - a[1], b[0] - a[0])
            self.rois[side] = (x, y, math.cos(yaw), math.sin(yaw))
        self.candidate = None
        self.count = 0
        self.selected = None

    def observe(self, section, distance, clusters):
        if self.selected is not None:
            return self.selected
        candidate = None
        config = self.config
        if section == 4 and config['start_s'] <= distance <= config['end_s']:
            counts = {}
            for side, (x, y, cosine, sine) in self.rois.items():
                counts[side] = sum(
                    abs(cosine * (px - x) + sine * (py - y)) <= config['length_m'] / 2
                    and abs(-sine * (px - x) + cosine * (py - y)) <= config['width_m'] / 2
                    for cluster in clusters for px, py in cluster)
            if counts['left'] >= config['blocked_min_points'] and counts['right'] <= config['clear_max_points']:
                candidate = 'right'
            elif counts['right'] >= config['blocked_min_points'] and counts['left'] <= config['clear_max_points']:
                candidate = 'left'
        self.count = self.count + 1 if candidate and candidate == self.candidate else int(bool(candidate))
        self.candidate = candidate
        if self.count >= config['consecutive_frames']:
            self.selected = candidate
        return self.selected
