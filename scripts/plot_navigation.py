#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Export the P4 recorded trajectory and independently measured goal errors."""
import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import yaml


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('directory', type=Path)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    rows = list(csv.DictReader((args.directory / 'navigation.csv').open()))
    checks = json.loads((args.directory / 'navigation_checks.json').read_text())
    meta = yaml.safe_load((root / 'src/forklift_navigation/maps/warehouse.yaml').read_text())
    image = plt.imread(root / 'src/forklift_navigation/maps' / meta['image'])
    x0, y0, _ = meta['origin']; resolution = meta['resolution']
    height, width = image.shape[:2]
    fig, axes = plt.subplots(1, 2, figsize=(12, 5), constrained_layout=True)
    axes[0].imshow(image, cmap='gray', origin='upper',
                   extent=(x0, x0 + width * resolution, y0, y0 + height * resolution))
    for x, y, label in [('x', 'y', 'Gazebo physical pose'),
                         ('amcl_x', 'amcl_y', 'AMCL estimate')]:
        axes[0].plot([float(r[x]) for r in rows], [float(r[y]) for r in rows],
                     linewidth=1, label=label)
    axes[0].set(xlabel='x (m)', ylabel='y (m)', title='P4 navigation on saved SLAM map',
                xlim=(-2.2, 2.4), ylim=(-2.0, 2.0))
    axes[0].legend(fontsize=8); axes[0].set_aspect('equal')
    goals = [c for c in checks if c['test'].startswith('goal_') and 'position_error_m' in c]
    axes[1].bar(np.arange(len(goals)) + 1, [c['position_error_m'] for c in goals])
    axes[1].axhline(.15, color='red', linestyle='--', label='Acceptance limit: 0.15 m')
    axes[1].set(xlabel='Goal number', ylabel='Position error (m)',
                title='Final error against Gazebo physical pose', ylim=(0, .18))
    axes[1].legend(fontsize=8)
    fig.savefig(args.directory / 'navigation_summary.png', dpi=160)


if __name__ == '__main__':
    main()
