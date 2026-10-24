#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Export P5 repeatability data from independent Gazebo link pose checks."""
import csv
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def main():
    root=Path(__file__).resolve().parents[1]/'results/p5'
    fig,axes=plt.subplots(1,2,figsize=(12,4),constrained_layout=True)
    metrics={}
    for directory,label,color in [('unloaded','No proof mass','tab:blue'),('test_mass_0p1kg','Rigid proof mass 0.1 kg','tab:orange')]:
        checks=json.loads((root/directory/'checks.json').read_text())
        levels=[c for c in checks if c['test'].startswith('height_')]
        metrics[directory]=dict(repeats_per_height={str(h):sum(c['target_m']==h for c in levels) for h in [.05,.12,.20]},
            maximum_error_m=max(c['error_m'] for c in levels),passed=sum(c['passed'] for c in levels),total=len(levels))
        rows=list(csv.DictReader((root/directory/'lift.csv').open()))
        reference=next(float(r['physical_carriage_z']) for r in rows if r['homed']=='True' and r['stage']=='home')
        sample=[r for r in rows if r['stage'].startswith('height_')]
        start=float(sample[0]['t'])
        axes[0].plot([float(r['t'])-start for r in sample],
                     [100*(float(r['physical_carriage_z'])-reference) for r in sample],color=color,label=label,linewidth=1)
        offset=-.15 if directory=='unloaded' else .15
        errors=[1000*max(c['error_m'] for c in levels if c['target_m']==h) for h in [.05,.12,.20]]
        axes[1].bar(np.arange(3)+offset,errors,width=.3,color=color,label=label)
    axes[0].set(xlabel='Time in repeat test (s)',ylabel='Physical carriage travel (cm)',title='10 visits to each height, per mass condition')
    axes[0].legend(fontsize=8);axes[0].grid(alpha=.2)
    axes[1].set(xticks=[0,1,2],xticklabels=['5 cm','12 cm','20 cm'],ylabel='Maximum absolute error (mm)',
                title='SIM ideal position actuator; acceptance limit 5 mm')
    axes[1].legend(fontsize=8);axes[1].set_ylim(bottom=0)
    fig.savefig(root/'lift_results.png',dpi=160)
    (root/'repeatability.json').write_text(json.dumps(metrics,indent=2)+'\n')


if __name__=='__main__':main()
