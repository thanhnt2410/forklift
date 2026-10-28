#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Collect selected SIM trials without dropping failed checks; plot physical/map error."""
import argparse
import csv
import json
from pathlib import Path
import shutil


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--runs',nargs='+',required=True,help='name=directory for each selected trial')
    parser.add_argument('--output',required=True)
    args=parser.parse_args();out=Path(args.output);out.mkdir(parents=True,exist_ok=True)
    summaries={};trajectories=[]
    for item in args.runs:
        name,path=item.split('=',1);source=Path(path);target=out/name;target.mkdir(exist_ok=True)
        summary=json.loads((source/'summary.json').read_text())
        cleanup=json.loads((source/'cleanup.json').read_text())
        summary['cleanup_empty']=not cleanup['remaining_pids']
        summaries[name]=summary
        for filename in ('summary.json','checks.json','cleanup.json','run.json','final_status.json','mission.csv','route_versions.json'):
            if (source/filename).is_file():shutil.copy2(source/filename,target/filename)
        if (source/'config/manifest.json').is_file():shutil.copy2(source/'config/manifest.json',target/'source_manifest.json')
        if summary.get('scenario') in ('nominal','rotated') and (source/'final_status.json').is_file() and not summary.get('missions_expected'):
            with (source/'mission.csv').open() as stream:rows=list(csv.DictReader(stream))
            trajectories.append((name,rows,json.loads((source/'final_status.json').read_text())))
    result=dict(passed=all(s['passed'] and s['cleanup_empty'] for s in summaries.values()),
                checks_passed=sum(s['checks_passed'] for s in summaries.values()),
                checks_total=sum(s['checks_total'] for s in summaries.values()),runs=summaries,physical_hardware_used=False)
    (out/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    if trajectories:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from matplotlib.patches import Circle
        fig,axes=plt.subplots(1,2,figsize=(11,5),layout='constrained')
        for name,rows,status in trajectories:
            axes[0].plot([float(r['robot_x']) for r in rows],[float(r['robot_y']) for r in rows],label=name+' robot')
            axes[0].plot([float(r['pallet_x']) for r in rows],[float(r['pallet_y']) for r in rows],'--',label=name+' pallet')
            valid=[r for r in rows if r.get('map_x') not in ('','None',None)]
            axes[1].plot([float(r['map_x']) for r in valid],[float(r['map_y']) for r in valid],label=name)
            early=status.get('initial_detection') or {}
            correction=early if early.get('outcome')=='DETECTED' else status['refinement']
            if correction:
                for key,marker in [('initial','x'),('corrected','+')]:
                    p=correction[key];axes[1].scatter(p[0],p[1],marker=marker,s=100,label=name+' A '+key)
        axes[0].add_patch(Circle((.95,-.60),.15,fill=False,color='green',linestyle=':',label='B physical delivery region (15 cm)'))
        axes[0].scatter([.95],[-.60],marker='x',color='green')
        axes[1].scatter([.72],[-.60],marker='x',color='green',s=100,label='B requested robot pose')
        for axis,title in zip(axes,['Gazebo physical positions','Localized robot positions in map']):
            axis.set(title=title,xlabel='x (m)',ylabel='y (m)');axis.axis('equal');axis.grid(alpha=.25);axis.legend(fontsize=7)
        fig.savefig(out/'trajectories.png',dpi=160);plt.close(fig)
    print(json.dumps(result,indent=2))
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
