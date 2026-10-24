#!/usr/bin/env python3
"""Plot the recorded Nav2 path and its straight, pallet-aligned final leg."""
import argparse
import csv
import json
import math
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
p=argparse.ArgumentParser();p.add_argument('directory');a=p.parse_args();out=Path(a.directory)
plans=json.loads((out/'plans.json').read_text())
path=next(path for path in plans if len(path)>2 and math.hypot(path[-1][0]-.35,path[-1][1]-.65)<.03)
length=json.loads((out/'mission_route.json').read_text()).get('terminal_distance',.5) if (out/'mission_route.json').exists() else .5
if (out/'mission_route.json').exists():
    route=json.loads((out/'mission_route.json').read_text())
    length=next((s.get('terminal_distance',length) for s in route['segments'] if s['stage']=='NAV_PICK'),length)
gx,gy,angle=path[-1];c,s=math.cos(angle),math.sin(angle);entry=(gx-length*c,gy-length*s)
# Highlight the last half metre of recorded points, not an invented overlay.
tail=[path[-1][:2]];remaining=length
for j in range(len(path)-2,-1,-1):
    end=path[j+1];start=path[j];distance=math.hypot(end[0]-start[0],end[1]-start[1])
    if distance>=remaining and distance>0:
        ratio=remaining/distance;tail.append([end[0]+ratio*(start[0]-end[0]),end[1]+ratio*(start[1]-end[1])]);break
    tail.append(start[:2]);remaining-=distance
tail.reverse()
assert max(abs(-(x-gx)*s+(y-gy)*c) for x,y in tail)<.005
fig,axes=plt.subplots(1,2,figsize=(12,5))
for ax in axes:
 ax.plot([p[0] for p in path],[p[1] for p in path],color='royalblue',lw=2,label='Recorded Nav2 path')
 ax.plot([p[0] for p in tail],[p[1] for p in tail],color='crimson',lw=3,label=f'Final straight leg: {length:.2f} m')
 ax.plot([gx,1.0],[gy,.65],'--',color='dimgray',lw=1,label='Fork / pallet alignment axis')
 ax.scatter([gx],[gy],color='crimson',s=35)
 ax.add_patch(Rectangle((1.0,.53),.06,.24,color='goldenrod',alpha=.8))
 ax.annotate('Pallet',xy=(1.03,.65),xytext=(7,7),textcoords='offset points')
 ax.set_aspect('equal',adjustable='box');ax.grid(alpha=.3);ax.set_xlabel('x (m)');ax.set_ylabel('y (m)')
rows=list(csv.DictReader((out/'mission.csv').open()))
r=[r for r in rows if r['state']=='NAV_PICK']
axes[0].plot([float(r['robot_x']) for r in r],[float(r['robot_y']) for r in r],':',color='black',lw=1,label='Robot (simulation)')
axes[0].set_title('Curve in free space, straight approach to pallet')
axes[0].legend(fontsize=8,loc='best')
ax=axes[1];ax.add_patch(Rectangle((gx-.1551,gy-.13472),.3102,.26944,fill=False,edgecolor='black',lw=1.5))
for y in (gy-.0725,gy+.0725):ax.plot([gx+.2313,gx+.4313],[y,y],color='black',lw=3)
ax.annotate('Nav2 stops here',xy=(gx,gy),xytext=(gx-.15,gy-.25),arrowprops={'arrowstyle':'->'},fontsize=9)
ax.set_xlim(gx-length-.15,1.16);ax.set_ylim(gy-.4,gy+.4);ax.set_title('Final path already aligns the forks with the pallet')
fig.tight_layout();fig.savefig(out/'terminal_path.png',dpi=160)
