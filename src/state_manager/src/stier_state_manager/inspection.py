"""Read-only inspection markers. Observed RDDF never authorizes a mission."""
import math


def fresh(item, now, timeout=.5):
    return bool(item and all(isinstance(item.get(k), (int, float))
                            and math.isfinite(item[k]) and item[k] > 0
                            and 0 <= now - item[k] <= timeout
                            for k in ('stamp', 'received')))


def inspection_specs(routes, samples, now, timeout=.5):
    markers = []
    pose = samples.get('pose', {})
    positioned = (fresh(pose, now, timeout) and pose.get('frame') == 'map'
                  and all(math.isfinite(pose.get(k, math.nan)) for k in ('x', 'y')))
    anchor = (pose['x'], pose['y'] + 9, 3) if positioned else (0, 8, 3)
    lines = ['INSPECTION ONLY | no motion authorization']
    observed = samples.get('observed', {})
    valid = samples.get('localization_valid', {})
    ready = (fresh(observed, now, timeout) and observed.get('frame') == 'map'
             and fresh(valid, now, timeout) and valid.get('value') is True
             and observed.get('matched') and observed.get('has_nearest'))
    route = routes.get(observed.get('source')) if ready else None
    if route:
        lines.append('Observed S%02d: %s' % (route.section, route.name))
        markers.append({'kind': 'LINE_STRIP', 'ns': 'observed_rddf', 'key': 'route',
                        'points': [(x, y, .22) for x, y, _ in route.points],
                        'scale': (.17, 0, 0), 'color': (0, 1, 1, 1)})
    else:
        reason = observed.get('reason', 'NO_DATA') if fresh(observed, now, timeout) else 'NO_DATA / STALE'
        if not fresh(valid, now, timeout) or valid.get('value') is not True:
            reason = 'LOCALIZATION_INVALID / STALE'
        elif fresh(observed, now, timeout) and observed.get('frame') != 'map':
            reason = 'FRAME_MISMATCH'
        lines.append('Observed RDDF: UNKNOWN (%s)' % reason)
    for key, label in (('mission', 'Manager'), ('scan', 'LiDAR'), ('roi', 'ROI points'),
                       ('clusters', 'Clusters'), ('signal', 'Traffic signal'),
                       ('lane', 'Lane signals'), ('dynamic', 'Dynamic observation'),
                       ('safety', 'Safety'), ('selection', 'Selector')):
        item = samples.get(key)
        if not item:
            value = 'NO_DATA'
        elif not fresh(item, now, timeout):
            value = 'STALE / INVALID TIME'
        else:
            value = item.get('text', 'RECEIVED')
        lines.append('%s: %s' % (label, value))
    for source, color in (('rddf', (.9, .6, .2, .8)), ('local', (.5, .6, 1, .9)),
                          ('park', (1, .3, .8, .9))):
        item = samples.get(source)
        if not item:
            status = 'NO_DATA'
        elif not fresh(item, now, timeout):
            status = 'STALE'
        elif item.get('frame') != 'map':
            status = 'FRAME_MISMATCH'
        else:
            points = item.get('points', [])
            status = '%s | %d points | decision %s' % (
                item.get('route', ''), len(points), item.get('decision_id', '?'))
            if len(points) >= 2 and all(all(math.isfinite(v) for v in p) for p in points):
                markers.append({'kind': 'LINE_STRIP', 'ns': 'candidate_' + source, 'key': source,
                                'points': points, 'scale': (.10, 0, 0), 'color': color})
        lines.append('%s candidate: %s' % (source.upper(), status))
    lines.append('Candidate paths are previews; /path/final is the selected output.')
    markers.append({'kind': 'TEXT_VIEW_FACING', 'ns': 'inspection_status', 'key': 'status',
                    'position': anchor, 'scale': (0, 0, .38), 'color': (.7, .95, 1, 1),
                    'text': '\n'.join(lines)})
    return markers
