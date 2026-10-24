#!/usr/bin/env python3
"""Plot the full precomputed mission, including docking and loaded withdrawal."""
import argparse,csv,json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=argparse.ArgumentParser();p.add_argument('directory');args=p.parse_args();out=Path(args.directory)
route=json.loads((out/'mission_route.json').read_text())
fig,ax=plt.subplots(figsize=(9,7));palette={'NAV_PICK':'royalblue','NAV_DROP':'forestgreen','RETURN_HOME':'purple'}
used=set()
for segment in route['segments']:
    points=segment['poses'];name=segment['stage'];kind=segment['kind']
    label=name if kind=='navigation' else 'Straight reverse after pickup/release' if name.startswith('RETREAT') else 'LiDAR alignment + insertion'
    color=palette.get(name,'darkorange' if name.startswith('RETREAT') else 'darkcyan')
    ax.plot([v[0] for v in points],[v[1] for v in points],color=color,lw=2.5,ls='--' if name.startswith('RETREAT') else '-',label=label if label not in used else None)
    used.add(label)
    mid=len(points)//2
    if len(points)>2:
        u,v=points[max(0,mid-2)],points[min(len(points)-1,mid+2)]
        ax.annotate('',xy=v[:2],xytext=u[:2],arrowprops=dict(arrowstyle='->',color=color,lw=2))
for key,text in [('lift_at','Pickup + lift'),('release_at','Release')]:
    point=route[key];ax.scatter(*point[:2],color='black',zorder=5);ax.annotate(text,point[:2],xytext=(5,12),textcoords='offset points',fontsize=10)
    ax.plot([1.,1.],[point[1]-.12,point[1]+.12],color='goldenrod',lw=6)
ax.scatter([0],[0],color='black');ax.annotate('Home',(0,0),xytext=(5,5),textcoords='offset points')
rows=list(csv.DictReader((out/'mission.csv').open()))
ax.plot([float(r['robot_x']) for r in rows],[float(r['robot_y']) for r in rows],color='black',alpha=.35,lw=1,label='Actual robot trajectory')
ax.set_aspect('equal');ax.grid(alpha=.25);ax.set_xlabel('x (m)');ax.set_ylabel('y (m)')
ax.set_title(f"Full mission planned before departure: 9 segments\nReverse clearance {route.get('retreat_minimum',1.3):.2f} m; final straight {route.get('terminal_distance',.5):.2f} m")
ax.legend(fontsize=9,loc='best');fig.tight_layout();fig.savefig(out/'mission_route.png',dpi=170)
