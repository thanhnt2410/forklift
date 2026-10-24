#!/usr/bin/env python3
"""Compare recorded plans and physical transit samples; no synthetic trajectories."""
import argparse
import csv
import json
import math
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=argparse.ArgumentParser();p.add_argument('baseline');p.add_argument('current');a=p.parse_args()
fig,axes=plt.subplots(1,2,figsize=(11,5));report={}
for ax,label,directory in zip(axes,('forward_only','forward_reverse'),(a.baseline,a.current)):
    folder=Path(directory)
    if not json.loads((folder/'summary.json').read_text())['passed']:raise SystemExit(f'Incomplete/failed acceptance: {folder}')
    plans=json.loads((folder/'plans.json').read_text())
    rows=list(csv.DictReader((folder/'mission.csv').open()))
    path=next(q for q in plans if len(q)>2 and math.hypot(q[-1][0]-.35,q[-1][1]-.65)<.03)
    length=reverse=0.
    for u,v in zip(path,path[1:]):
        dx,dy=v[0]-u[0],v[1]-u[1];d=math.hypot(dx,dy);length+=d
        backwards=dx*math.cos(u[2])+dy*math.sin(u[2])<0
        if backwards:reverse+=d
        ax.plot([u[0],v[0]],[u[1],v[1]],color='darkorange' if backwards else 'royalblue',lw=2)
    nav=[r for r in rows if r['state'] in ('NAV_PICK','NAV_DROP','RETURN_HOME') and r['source']=='NAV2']
    nav_seconds=sum(float(v['t'])-float(u['t']) for u,v in zip(rows,rows[1:]) if u['state'] in ('NAV_PICK','NAV_DROP','RETURN_HOME'))
    distance=sum(math.hypot(float(v['robot_x'])-float(u['robot_x']),float(v['robot_y'])-float(u['robot_y'])) for u,v in zip(rows,rows[1:]) if u['state'] in ('NAV_PICK','NAV_DROP','RETURN_HOME'))
    report[label]=dict(nav_physical_m=distance,first_pick_plan_m=length,first_pick_reverse_m=reverse,nav_wall_seconds=nav_seconds,
        mission_wall_seconds=float(rows[-1]['t'])-float(rows[0]['t']),peak_command_mps=max(abs(float(r['cmd_x'])) for r in nav),
        reverse_samples=sum(float(r['cmd_x'])<-.03 for r in nav))
    ax.scatter([0,.35,1.],[0,.65,.65],s=25,color=['black','red','green'])
    ax.plot([],[],color='royalblue',label='Forward');ax.plot([],[],color='darkorange',label='Reverse');ax.legend(fontsize=8)
    ax.set_xlim(-.6,1.1);ax.set_ylim(-.7,.85)
    ax.set_title(f'{label}: {length:.2f} m to pickup');ax.set_aspect('equal');ax.grid(alpha=.3);ax.set_xlabel('x (m)');ax.set_ylabel('y (m)')
base,now=report.values();report['pick_path_reduction_percent']=100*(1-now['first_pick_plan_m']/base['first_pick_plan_m'])
report['nav_distance_reduction_percent']=100*(1-now['nav_physical_m']/base['nav_physical_m'])
report['timing_note']='Wall times depend on Gazebo real-time factor and host load; not a hardware speed benchmark.'
report['legend']='Blue: forward; orange: reverse. Paths recorded from /plan.'
out=Path(a.current);(out/'comparison.json').write_text(json.dumps(report,indent=2)+'\n')
fig.tight_layout();fig.savefig(out/'path_comparison.png',dpi=160)
print(json.dumps(report,indent=2))
