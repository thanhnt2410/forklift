#!/usr/bin/env python3
"""Measure physical path error on approved Nav2 legs (offline, no control input)."""
import argparse
import csv
import json
import math
import re
from pathlib import Path

import numpy as np


def project(x, y, path):
    """Nearest projection on a polyline; heading interpolates across +/-pi."""
    best = None
    for a, b in zip(path, path[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        length2 = dx * dx + dy * dy
        if length2 < 1e-12:
            continue
        u = max(0., min(1., ((x-a[0])*dx + (y-a[1])*dy)/length2))
        distance = math.hypot(x-a[0]-u*dx, y-a[1]-u*dy)
        heading = a[2] + u * math.atan2(math.sin(b[2]-a[2]), math.cos(b[2]-a[2]))
        if best is None or distance < best[0]:
            best = distance, heading
    if best is None:
        raise ValueError('Path has no nonzero segment')
    return best


def stats(values):
    return dict(rmse=float(np.sqrt(np.mean(np.square(values)))),
                p95=float(np.percentile(np.abs(values), 95)),
                maximum=float(np.max(np.abs(values))))


def analyze(folder):
    route = json.loads((folder/'mission_route.json').read_text())
    paths = {s['stage']: s['poses'] for s in route['segments'] if s['kind'] == 'navigation'}
    rows = list(csv.DictReader((folder/'mission.csv').open()))
    report = dict(approved_sha256=route['approved_sha256'], stages={})
    samples = []
    for stage, path in paths.items():
        selected = [r for r in rows if r['state'] == stage and r['source'] == 'NAV2']
        errors = []
        localized_errors = []
        localization_offsets = []
        for r in selected:
            distance, heading = project(float(r['robot_x']), float(r['robot_y']), path)
            angle = math.degrees(math.atan2(math.sin(float(r['robot_yaw'])-heading),
                                           math.cos(float(r['robot_yaw'])-heading)))
            errors.append((distance, angle))
            if r.get('map_x') and r.get('map_y'):
                localized_errors.append(project(float(r['map_x']), float(r['map_y']), path)[0])
                localization_offsets.append(math.hypot(float(r['map_x'])-float(r['robot_x']),
                                                       float(r['map_y'])-float(r['robot_y'])))
            samples.append(dict(stage=stage, sim_t=r['sim_t'], distance_m=distance,
                                heading_deg=angle, cmd_x=r['cmd_x']))
        if not errors:
            continue
        moving = [e for e, r in zip(errors, selected) if abs(float(r['cmd_x'])) > .03]
        report['stages'][stage] = dict(samples=len(errors), distance_m=stats([e[0] for e in errors]),
            heading_deg=stats([e[1] for e in errors]),
            moving_distance_m=stats([e[0] for e in moving]) if moving else None,
            sim_seconds=float(selected[-1]['sim_t'])-float(selected[0]['sim_t']))
        report['stages'][stage]['physical_endpoint_distance_m'] = math.hypot(
            float(selected[-1]['robot_x'])-path[-1][0], float(selected[-1]['robot_y'])-path[-1][1])
        if localized_errors:
            report['stages'][stage]['localized_path_distance_m'] = stats(localized_errors)
            report['stages'][stage]['localization_offset_m'] = stats(localization_offsets)
            report['stages'][stage]['localized_samples'] = len(localized_errors)
        for direction, sign in [('forward', 1), ('reverse', -1)]:
            q = [e[0] for e, r in zip(errors, selected) if sign*float(r['cmd_x']) > .03]
            report['stages'][stage][direction+'_distance_m'] = stats(q) if q else None
    report['overall_distance_m'] = stats([s['distance_m'] for s in samples])
    log = folder/'launch.log'
    if log.exists():
        report['heading_recovery_starts'] = len(re.findall('Heading recovery: stop translation', log.read_text()))
    report['method'] = ('Gazebo ground truth to nearest point on approved polyline; map/world assumed aligned. '
        'Includes stationary heading recovery; overlapping forward/reverse portions are geometrically ambiguous. '
        'This is physical path distance, not localization-frame controller error. Samples approximately 10 Hz wall time.')
    (folder/'tracking.json').write_text(json.dumps(report, indent=2)+'\n')
    with (folder/'tracking.csv').open('w') as f:
        writer = csv.DictWriter(f, fieldnames=samples[0])
        writer.writeheader()
        writer.writerows(samples)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directories', nargs='+', type=Path)
    args = parser.parse_args()
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 3, figsize=(14, 4))
    for folder in args.directories:
        report = analyze(folder)
        print(str(folder), json.dumps(report, indent=2))
        with (folder/'tracking.csv').open() as f:
            samples = list(csv.DictReader(f))
        for ax, stage in zip(axes, report['stages']):
            q = [r for r in samples if r['stage'] == stage]
            ax.plot([float(r['sim_t'])-float(q[0]['sim_t']) for r in q],
                    [100*float(r['distance_m']) for r in q], label=folder.name)
            ax.set(title=stage, xlabel='Simulation time (s)', ylabel='Path distance (cm)')
            ax.grid(alpha=.3)
            ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(args.directories[-1]/'tracking_comparison.png', dpi=160)
    folder = args.directories[-1]
    route = json.loads((folder/'mission_route.json').read_text())
    rows = list(csv.DictReader((folder/'mission.csv').open()))
    legs = [s for s in route['segments'] if s['kind'] == 'navigation']
    fig, axes = plt.subplots(1, len(legs), figsize=(14, 5), squeeze=False)
    for ax, leg in zip(axes[0], legs):
        path = leg['poses']
        q = [r for r in rows if r['state'] == leg['stage'] and r['source'] == 'NAV2']
        ax.plot([p[0] for p in path], [p[1] for p in path], label='Approved path', lw=2)
        ax.plot([float(r['robot_x']) for r in q], [float(r['robot_y']) for r in q],
                label='Gazebo robot', lw=1.2, color='crimson')
        ax.set(title=leg['stage'], xlabel='x (m)', ylabel='y (m)')
        ax.set_aspect('equal', adjustable='box')
        ax.grid(alpha=.3)
        ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(folder/'tracking_trajectory.png', dpi=180)


if __name__ == '__main__':
    main()
