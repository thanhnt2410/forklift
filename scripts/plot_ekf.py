#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Export controlled sensor replay evidence; no physical-slip claims."""
import csv
import math
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    directory=Path(__file__).resolve().parents[1]/'results/ekf/replay'
    rows=list(csv.DictReader((directory/'fusion.csv').open()))
    fig,ax=plt.subplots(figsize=(8,4),constrained_layout=True)
    for key,label in [('true_yaw','Defined reference'),('wheel_yaw','Wheel yaw rate +50%'),
                      ('ekf_yaw','EKF: wheel + gyro')]:
        ax.plot([float(r['t']) for r in rows],
                [math.degrees(float(r[key])) for r in rows],label=label)
    ax.set(xlabel='Replay time (s)',ylabel='Yaw (degrees)',
           title='Controlled measurement error: wheel yaw rate +50%')
    ax.grid(alpha=.25);ax.legend();fig.savefig(directory/'fusion_comparison.png',dpi=160)


if __name__=='__main__':main()
