#!/usr/bin/env python3
"""Report command reversals and actual body attitude from recorded samples."""
import argparse,csv,json,math
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('directory');a=p.parse_args();out=Path(a.directory)
rows=list(csv.DictReader((out/'mission.csv').open()));report={}
for state in dict.fromkeys(r['state'] for r in rows):
    q=[r for r in rows if r['state']==state];signs=[]
    for r in q:
        v=float(r['cmd_x']);deadband=.003 if state in ('ALIGN','INSERT') else .03
        s=1 if v>deadband else -1 if v<-deadband else 0
        if s and (not signs or signs[-1]['direction']!=s):signs.append(dict(t=float(r['t'])-float(q[0]['t']),direction=s))
    report[state]=dict(seconds=sum(float(v['t'])-float(u['t']) for u,v in zip(rows,rows[1:]) if u['state']==state and v['state']==state),direction_changes=max(0,len(signs)-1),transitions=signs,minimum_cmd_x=min(float(r['cmd_x']) for r in q),maximum_cmd_x=max(float(r['cmd_x']) for r in q),maximum_cmd_w=max(abs(float(r['cmd_w'])) for r in q),
        maximum_roll_deg=max(abs(float(r.get('robot_roll',0)))*180/math.pi for r in q),maximum_pitch_deg=max(abs(float(r.get('robot_pitch',0)))*180/math.pi for r in q))
(out/'motion_stability.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
