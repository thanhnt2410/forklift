#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Plot independent physical trajectories and lift clearance from P6 CSV."""
import argparse
import csv
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=argparse.ArgumentParser();p.add_argument('directory');a=p.parse_args();out=Path(a.directory)
rows=list(csv.DictReader((out/'mission.csv').open()))
# Show the first complete cycle; fixture resets between repetitions are not transport.
end=next((i+1 for i,r in enumerate(rows) if r['state']=='DONE'),len(rows))
rows=rows[:end]
t0=float(rows[0]['t']);t=[float(r['t'])-t0 for r in rows]
fig,axes=plt.subplots(1,2,figsize=(12,5))
axes[0].plot([float(r['robot_x']) for r in rows],[float(r['robot_y']) for r in rows],label='Robot (physics)')
axes[0].plot([float(r['pallet_x']) for r in rows],[float(r['pallet_y']) for r in rows],label='Pallet (physics)',color='darkorange')
for label,x,y in [('Home',0,0),('Pick A',.95,.65),('Drop B',.95,-.6)]:
 axes[0].scatter([x],[y],s=35);axes[0].annotate(label,(x,y),xytext=(5,5),textcoords='offset points')
axes[0].set(xlabel='x (m)',ylabel='y (m)',title='P6: physical robot and pallet trajectories');axes[0].axis('equal');axes[0].grid();axes[0].legend()
axes[1].plot(t,[1000*float(r['pallet_z']) for r in rows],label='Pallet clearance',color='darkorange')
if 'raw_lift' in rows[0]:axes[1].plot(t,[1000*float(r['raw_lift']) for r in rows],label='Lift joint')
axes[1].axhline(20,color='red',linestyle='--',label='Transport joint limit')
axes[1].set(xlabel='Wall time (s)',ylabel='Height (mm)',title='Actual load lift and release');axes[1].grid();axes[1].legend()
fig.tight_layout();fig.savefig(out/'mission_results.png',dpi=160)
