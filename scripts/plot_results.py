#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Plot saved SIM observations; requires matplotlib, not a running ROS graph."""
import argparse
import csv
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def main():
    p=argparse.ArgumentParser()
    p.add_argument('directory', nargs='?', default='results/p1_final')
    args=p.parse_args()
    path=Path(args.directory)
    with (path/'telemetry.csv').open() as f:
        rows=list(csv.DictReader(f))
    def col(name):
        return np.array([float(r[name]) for r in rows])
    t=col('sim_s')-col('sim_s')[0]
    fig,axes=plt.subplots(2,2,figsize=(12,8),layout='constrained')
    ax=axes[0,0]
    ax.plot(col('gt_x'),col('gt_y'),label='Gazebo physics')
    ax.plot(col('odom_x'),col('odom_y'),'--',label='Wheel odometry')
    ax.set(xlabel='x (m)',ylabel='y (m)',title='Motion and odometry')
    ax.axis('equal');ax.legend();ax.grid(True)
    ax=axes[0,1]
    ax.plot(t,col('gt_z'),label='Base z (m)')
    ax.plot(t,col('gt_roll'),label='Roll (rad)')
    ax.plot(t,col('gt_pitch'),label='Pitch (rad)')
    ax.set(xlabel='Simulation time (s)',title='Physical stability');ax.legend();ax.grid(True)
    ax=axes[1,0]
    for name in ('front_left','front_right','rear_left','rear_right'):
        ax.plot(t,col(name),label=name.replace('_',' '),linewidth=.8)
    ax.set(xlabel='Simulation time (s)',ylabel='rad/s',title='Measured wheel velocities');ax.legend();ax.grid(True)
    ax=axes[1,1]
    ax.plot(t,col('lift'),label='Measured lift position')
    ax.axhline(0,color='gray',linestyle=':');ax.axhline(.25,color='gray',linestyle=':')
    ax.set(xlabel='Simulation time (s)',ylabel='Height (m)',title='Lift, limits and cancellation');ax.grid(True)
    fig.suptitle('P1 simulation measurements — not real robot validation')
    fig.savefig(path/'simulation_results.png',dpi=160)
    plt.close(fig)
    print(path/'simulation_results.png')


if __name__=='__main__':
    main()
