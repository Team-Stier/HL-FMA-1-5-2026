#!/usr/bin/env python3
"""Offline rigid placement of a school RDDF at a measured rear-axle pose.

GPS arguments locate the antenna, not base_link. Heading is measured ENU yaw
(east=0, north=90 degrees), never the old RDDF-initialized IMU heading.
No ROS publishers, vehicle commands or changes to the source map.
"""
import argparse
import copy
import csv
import json
import math
from pathlib import Path

import yaml

from rddf_initialization_core import RddfRouteMap, WGS84_A_M, WGS84_E2


def gps_heading(route_map, start_lat, start_lon, end_lat, end_lon):
    start = route_map.project_gps(start_lat, start_lon)
    end = route_map.project_gps(end_lat, end_lon)
    dx, dy = end[0] - start[0], end[1] - start[1]
    return math.atan2(dy, dx), math.hypot(dx, dy)


class Placement:
    def __init__(self, route_map, route, antenna_lat, antenna_lon, yaw,
                 lever_x, lever_y):
        self.map = route_map
        self.source = route_map.routes[route][0]
        segment = next(s for s in route_map._segments if s['route'] == route)
        # _segments yaw is body heading, including reverse-route direction.
        self.rotation = yaw - segment['yaw']
        self.c, self.s = math.cos(self.rotation), math.sin(self.rotation)
        ax, ay = route_map.project_gps(antenna_lat, antenna_lon)
        self.target = (ax - math.cos(yaw) * lever_x + math.sin(yaw) * lever_y,
                       ay - math.sin(yaw) * lever_x - math.cos(yaw) * lever_y)
        lat = math.radians(route_map.origin['lat'])
        den = math.sqrt(1.0 - WGS84_E2 * math.sin(lat) ** 2)
        self.east_scale = WGS84_A_M / den * math.cos(lat)
        self.north_scale = WGS84_A_M * (1.0 - WGS84_E2) / den ** 3

    def xy(self, x, y):
        dx, dy = x - self.source[0], y - self.source[1]
        return (self.target[0] + self.c * dx - self.s * dy,
                self.target[1] + self.s * dx + self.c * dy)

    def latlon(self, x, y):
        return (self.map.origin['lat'] + math.degrees(y / self.north_scale),
                self.map.origin['lng'] + math.degrees(x / self.east_scale))

    def geographic(self, lat, lon):
        return self.latlon(*self.xy(*self.map.project_gps(lat, lon)))

    def yaw(self, yaw):
        return math.atan2(math.sin(yaw + self.rotation),
                          math.cos(yaw + self.rotation))

    def metadata(self, value):
        if isinstance(value, list):
            return [self.metadata(v) for v in value]
        if not isinstance(value, dict):
            return value
        # Keep the original ENU datum. Transform geometry, not map coordinates.
        result = {k: copy.deepcopy(v) if k == 'origin' else self.metadata(v)
                  for k, v in value.items()}
        if 'east_m' in value and 'north_m' in value:
            x, y = self.xy(float(value['east_m']), float(value['north_m']))
            result['east_m'], result['north_m'] = x, y
            if 'latitude' in value and 'longitude' in value:
                result['latitude'], result['longitude'] = self.latlon(x, y)
        elif 'lat' in value and 'lng' in value:
            result['lat'], result['lng'] = self.geographic(value['lat'], value['lng'])
        elif 'latitude' in value and 'longitude' in value:
            result['latitude'], result['longitude'] = self.geographic(
                value['latitude'], value['longitude'])
        if 'path_yaw_rad' in value:
            result['path_yaw_rad'] = self.yaw(value['path_yaw_rad'])
        return result


def prepare(source, output, route, latitude, longitude, yaw, lever_x, lever_y):
    source, output = Path(source).resolve(), Path(output).resolve()
    route_map = RddfRouteMap(source)
    placement = Placement(route_map, route, latitude, longitude, yaw, lever_x, lever_y)
    # A new directory per placement leaves both original and previous tests intact.
    output.mkdir(parents=True, exist_ok=False)
    for path in source.rglob('*.csv'):
        target = output / path.relative_to(source)
        target.parent.mkdir(parents=True, exist_ok=True)
        with path.open(encoding='utf-8-sig', newline='') as stream:
            reader = csv.DictReader(stream)
            fields, rows = reader.fieldnames, list(reader)
        for row in rows:
            x, y = placement.xy(float(row['east_m']), float(row['north_m']))
            lat, lon = placement.latlon(x, y)
            row.update(east_m=f'{x:.9f}', north_m=f'{y:.9f}',
                       latitude=f'{lat:.12f}', longitude=f'{lon:.12f}')
            # Single-point stop-line markers deliberately have no tangent.
            if row['path_yaw_rad']:
                row['path_yaw_rad'] = f"{placement.yaw(float(row['path_yaw_rad'])):.12f}"
        with target.open('w', encoding='utf-8', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=fields, lineterminator='\n')
            writer.writeheader()
            writer.writerows(rows)
    for name in ('yongin_route_project.json', 'yongin_mission_landmarks.json'):
        data = json.loads((source / name).read_text(encoding='utf-8'))
        (output / name).write_text(json.dumps(placement.metadata(data),
            ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    package = Path(__file__).resolve().parents[1]
    # Bundled placements stay relocatable when this repository is cloned on
    # another PC. External one-off exports retain an absolute directory.
    output_name = str(output.relative_to(package)) if package in output.parents else str(output)
    source_name = str(source.relative_to(package)) if package in source.parents else str(source)
    for name in ('initialization.yaml', 'viewer.yaml'):
        data = yaml.safe_load((source / name).read_text(encoding='utf-8'))
        if name == 'initialization.yaml':
            data['initialization']['rddf_directory'] = output_name
            data['reference']['source'] = output_name + '/yongin_route_project.json#origin'
        else:
            data['rddf_directory'] = output_name
        (output / name).write_text(yaml.safe_dump(data, allow_unicode=True, sort_keys=False),
                                  encoding='utf-8')
    report = dict(source=source_name, route=route, antenna_latitude=latitude,
                  antenna_longitude=longitude, vehicle_yaw_deg=math.degrees(yaw),
                  lever_arm_m=[lever_x, lever_y], rear_axle_map_m=list(placement.target),
                  rotation_deg=math.degrees(placement.rotation))
    (output / 'placement.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--route', default='3_s-static-obstacle')
    parser.add_argument('--latitude', type=float, required=True, help='GPS antenna latitude')
    parser.add_argument('--longitude', type=float, required=True, help='GPS antenna longitude')
    parser.add_argument('--yaw-deg', type=float, required=True, help='Measured ENU vehicle heading')
    parser.add_argument('--gps-config', type=Path,
                        default=Path(__file__).resolve().parents[1] / 'config/gps_reference.yaml')
    args = parser.parse_args()
    lever = yaml.safe_load(args.gps_config.read_text())['lever_arm']
    report = prepare(args.source, args.output, args.route, args.latitude,
                     args.longitude, math.radians(args.yaw_deg), lever['x_m'], lever['y_m'])
    print(json.dumps(report, indent=2))
    print('Launch overrides (append to your existing school launch; does not start nodes):')
    for key, value in [('rddf_directory', args.output.resolve()),
                       ('rddf_initialization_config', args.output.resolve() / 'initialization.yaml'),
                       ('localization_viewer_config', args.output.resolve() / 'viewer.yaml')]:
        print(f'  {key}:={value}')


if __name__ == '__main__':
    main()
