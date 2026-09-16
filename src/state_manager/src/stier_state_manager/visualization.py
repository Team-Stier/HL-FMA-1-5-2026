"""Inspectable RViz marker descriptions; geometry generation needs no ROS."""
import math

from .mission import hill_target


COLORS = (
    (0.13, 0.73, 0.95), (1.0, 0.60, 0.14), (0.45, 0.86, 0.31),
    (0.97, 0.35, 0.47), (0.68, 0.50, 0.96), (0.19, 0.86, 0.75),
    (0.98, 0.79, 0.22), (0.98, 0.42, 0.18), (0.39, 0.61, 0.98),
    (0.93, 0.40, 0.80), (0.66, 0.81, 0.41), (0.91, 0.71, 0.48),
    (0.46, 0.86, 0.94),
)


def marker_specs(routes, decision, odom=None, config=None):
    """Return primitive dictionaries for route, progress, landmarks and state."""
    config, odom = config or {}, odom or {}
    active = decision.get("route", decision.get("route_name", ""))
    markers = []
    for route in sorted(routes.values(), key=lambda item: (item.section, item.name)):
        color = COLORS[(route.section - 1) % len(COLORS)]
        markers.append({"kind": "LINE_STRIP", "ns": "rddf_routes", "key": route.name,
                        "points": [(x, y, .04) for x, y, _ in route.points],
                        "scale": (.10, 0, 0), "color": color + (.70,)})
        middle = route.pose_at(route.length / 2)
        offset = .65 if route.branch == "left" else -.65 if route.branch == "right" else 0
        markers.append({"kind": "TEXT_VIEW_FACING", "ns": "rddf_labels", "key": route.name,
                        "position": (middle[0], middle[1] + offset, .8), "scale": (0, 0, .42),
                        "color": color + (1,), "text": "S%02d %s" % (route.section, route.name)})
        for boundary, point in (("START", route.start), ("END", route.end)):
            markers.append({"kind": "SPHERE", "ns": "rddf_boundaries", "key": route.name + boundary,
                            "position": (point[0], point[1], .07), "scale": (.20, .20, .20),
                            "color": color + (.8,)})
        if route.name == active:
            markers.append({"kind": "LINE_STRIP", "ns": "active_route", "key": "active",
                            "points": [(x, y, .14) for x, y, _ in route.points],
                            "scale": (.24, 0, 0), "color": (1, 1, 1, 1)})
            distance = decision.get("distance_m", 0)
            if isinstance(distance, (float, int)) and math.isfinite(distance):
                projected = route.pose_at(distance)
                markers.append({"kind": "SPHERE", "ns": "route_progress", "key": "projection",
                                "position": (projected[0], projected[1], .3), "scale": (.5, .5, .5),
                                "color": (1, 1, 0, 1)})
    for route_name, landmarks in config.get("landmarks", {}).items():
        if route_name not in routes or not isinstance(landmarks, dict):
            continue
        landmarks = dict(landmarks)
        if routes[route_name].section == 1:
            try:
                landmarks['hill_stop_s'] = hill_target(landmarks)[1]
            except ValueError:
                pass
        for name, distance in sorted(landmarks.items()):
            if not name.endswith("_s") or not isinstance(distance, (int, float)) or not math.isfinite(distance):
                continue
            route = routes[route_name]
            if not 0 <= distance <= route.length:
                continue
            x, y, _ = route.pose_at(distance)
            yaw = route.pose_at(distance)[2]
            line_width = config.get("vehicle", {}).get("width_m")
            line_width = line_width + 1.0 if isinstance(line_width, (int, float)) and math.isfinite(line_width) and line_width > 0 else 3.0
            if name == "stop_line_s":
                kind, position, scale, color = "CUBE", (x, y, .04), (.12, line_width, .08), (1, .1, .1, .95)
            elif name in ("hill_zone_start_s", "hill_zone_end_s"):
                kind, position, scale, color = "CUBE", (x, y, .04), (.12, line_width, .08), (.95, .95, .95, 1)
            elif name == "hill_stop_s":
                kind, position, scale, color = "SPHERE", (x, y, .32), (.58, .58, .58), (1, .55, .05, 1)
            else:
                kind, position, scale, color = "CYLINDER", (x, y, .25), (.36, .36, .5), (1, .9, .25, 1)
            markers.append({"kind": kind, "ns": "landmarks", "key": route_name + name,
                            "position": position, "yaw": yaw, "scale": scale, "color": color})
            markers.append({"kind": "TEXT_VIEW_FACING", "ns": "landmark_labels", "key": route_name + name,
                            "position": (x, y, 1.05), "scale": (0, 0, .28), "color": (1, .9, .25, 1),
                            "text": "%s / %s\n%.2f m" % (route_name, name, distance)})
    wall = decision.get("virtual_stop")
    if wall and wall.get("valid") and "pose" in wall:
        x, y, yaw = wall["pose"]
        blocked = wall.get("active") or not decision.get("valid")
        width = config.get("vehicle", {}).get("width_m")
        width = width if isinstance(width, (int, float)) and math.isfinite(width) and width > 0 else 2.0
        if blocked:
            markers.append({"kind": "CUBE", "ns": "traffic_wall", "key": "stop_line",
                            "position": (x, y, .75), "yaw": yaw,
                            "scale": (.12, width + 1.0, 1.5), "color": (1, .05, .05, .65)})
        markers.append({"kind": "TEXT_VIEW_FACING", "ns": "traffic_wall_label", "key": "signal",
                        "position": (x, y, 2.0), "scale": (0, 0, .35),
                        "color": (1, .2, .2, 1) if blocked else (.2, 1, .2, 1),
                        "text": ("STOP WALL / " if blocked else "RDDF OPEN / ") + wall.get("required_signal", "")})
    has_pose = all(isinstance(odom.get(k), (float, int)) and math.isfinite(odom[k]) for k in ("x", "y", "yaw"))
    anchor = (odom["x"], odom["y"] + 3.0, 2.5) if has_pose else (0, 0, 2.5)
    if has_pose:
        markers.append({"kind": "ARROW", "ns": "localization", "key": "body_pose",
                        "position": (odom["x"], odom["y"], .4), "yaw": odom["yaw"],
                        "scale": (1.4, .40, .40), "color": (0.3, 1, .4, 1) if decision.get("valid") else (1, .5, .1, 1)})
    progress = decision.get("progress", 0.0)
    progress = progress if isinstance(progress, (int, float)) and math.isfinite(progress) else 0
    distance = decision.get("distance_m", 0.0)
    distance = distance if isinstance(distance, (int, float)) and math.isfinite(distance) else 0
    safety = decision.get("safety", {})
    stop = decision.get("stop_requested", True) or safety.get("stop", True)
    text = ("S%02d  %s\n%s / %s\nProgress %.1f%% | s %.2f m | %s\n"
            "Branch %s | Path %s | Gear %s\n%s\nSafety: %s" % (
                decision.get("section", 0), active or "WAIT_ROUTE_MAP",
                decision.get("mission", "INITIALIZATION"), decision.get("phase", "WAIT_INPUT"),
                100 * progress, distance, "STOP" if stop else "DRIVE",
                decision.get("selected_branch") or "NONE", decision.get("path_mode", "NONE"),
                "REVERSE" if decision.get("direction") == -1 else "FORWARD",
                decision.get("reason", "WAIT_INPUT"), safety.get("reason", "WAIT_INPUT")))
    elapsed = decision.get('elapsed_time_s')
    if isinstance(elapsed, (int, float)) and math.isfinite(elapsed):
        excluded = decision.get('excluded_signal_wait_s', 0.0)
        excluded = excluded if isinstance(excluded, (int, float)) and math.isfinite(excluded) else 0
        text += '\nElapsed %.1f s | excluded signal wait %.1f s' % (elapsed, excluded)
    if decision.get('diagnostics'):
        text += '\n' + ', '.join(str(item) for item in decision['diagnostics'])[:140]
    if decision.get("parking_leg_index", -1) >= 0:
        text += "\nParking leg %d: %s (target %.2f m)" % (decision["parking_leg_index"], decision.get("parking_leg_phase", ""), decision.get("parking_leg_target_s", -1.0))
    if decision.get("hill_target_s") is not None:
        text += "\nHill target %.2f m / hold %.1f s" % (decision["hill_target_s"], decision.get("hill_hold_elapsed_s", 0.0))
    markers.append({"kind": "TEXT_VIEW_FACING", "ns": "mission_status", "key": "status",
                    "position": anchor, "scale": (0, 0, .43), "text": text,
                    "color": (1, .35, .25, 1) if stop else (.3, 1, .45, 1)})
    return markers


def to_marker_array(specs, stamp, frame="map"):
    """Import ROS only at the rendering boundary, keeping specs testable."""
    from rospy import Duration
    from geometry_msgs.msg import Point
    from visualization_msgs.msg import Marker, MarkerArray

    output = MarkerArray()
    clear = Marker()
    clear.header.stamp, clear.header.frame_id = stamp, frame
    clear.action = Marker.DELETEALL
    output.markers.append(clear)
    counts = {}
    for spec in specs:
        marker = Marker()
        marker.header.stamp, marker.header.frame_id = stamp, frame
        marker.ns = spec["ns"]
        marker.id = counts.get(marker.ns, 0)
        counts[marker.ns] = marker.id + 1
        marker.type, marker.action = getattr(Marker, spec["kind"]), Marker.ADD
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = spec.get("position", (0, 0, 0))
        yaw = spec.get("yaw", 0)
        marker.pose.orientation.z, marker.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        marker.scale.x, marker.scale.y, marker.scale.z = spec["scale"]
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = spec["color"]
        marker.lifetime = Duration.from_sec(0.5)
        marker.text = spec.get("text", "")
        marker.points = [Point(x=x, y=y, z=z) for x, y, z in spec.get("points", [])]
        output.markers.append(marker)
    return output
