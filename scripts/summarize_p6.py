#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Consolidate only completed canonical P6 runs, never count development trials."""
import hashlib
import json
import math
from pathlib import Path
import re
import subprocess

root=Path(__file__).resolve().parents[1];out=root/'results/p6'
def read(name):return json.loads((out/name).read_text())
nominal=read('nominal/summary.json');faults=read('faults/summary.json');demo=read('demo/summary.json')
checks=read('nominal/checks.json');errors=[];home=[];lateral=[];angle=[];clearance=[];durations=[]
for check in checks:
    name=check['test']
    if name.endswith('_pallet_at_b'):errors.append(math.hypot(check['x']-.95,check['y']+.60))
    if name.endswith('_home'):home.append(math.hypot(check['x'],check['y']))
    if name.endswith('_pickup_alignment'):
        lateral.append(abs(check['lateral_m']));angle.append(abs(check['yaw_rad'])*180/math.pi)
    if name.endswith('_physical_carry'):clearance.append(check['minimum_z'])
for line in (out/'nominal/test.log').read_text().splitlines():
    try:
        d=json.loads(line)
        if d.get('state')=='DONE' and d.get('events'):durations.append(d['events'][-1]['wall_time']-d['events'][0]['wall_time'])
    except ValueError:pass
unit=re.search(r'(\d+) passed',(out/'unit_tests.log').read_text());units=int(unit.group(1)) if unit else 0
manifest=read('nominal/config/manifest.json');mismatch=[]
for name,digest in manifest.items():
    if hashlib.sha256((root/name).read_bytes()).hexdigest()!=digest:mismatch.append(name)
video=out/'demo/mission_demo.mp4'
build_text=(out/'build_final_verified.log').read_text()
build_ok='Summary: 9 packages finished' in build_text and 'Failed <<<' not in build_text
video_info=json.loads(subprocess.check_output(['ffprobe','-v','error','-show_entries','format=duration,size','-of','json',str(video)],text=True))
video_duration=float(video_info['format']['duration'])
summary=dict(gate='P6_SIM_ONLY',passed=bool(nominal['passed'] and nominal['missions_completed']==10
    and faults['passed'] and faults['scenarios_passed']==11 and demo['passed'] and units>0
    and build_ok and video.is_file() and video.stat().st_size>10000 and video_duration>=90 and not mismatch),
    physical_hardware_used=False,firmware_flashed=False,
    canonical_nominal='results/p6/nominal/summary.json',canonical_faults='results/p6/faults/summary.json',
    nominal=nominal,faults=faults,demo=demo,unit_tests_passed=units,build_passed=build_ok,video_duration_s=video_duration,
    metrics=dict(maximum_drop_position_error_m=max(errors),maximum_home_position_error_m=max(home),
        maximum_pickup_lateral_error_m=max(lateral),maximum_pickup_yaw_error_degrees=max(angle),
        minimum_physical_pallet_clearance_m=min(clearance),mission_wall_durations_s=durations),
    configuration_unchanged_during_acceptance=not mismatch,configuration_mismatches=mismatch,
    payload_model='Independent dynamic 0.1 kg pallet/parcel with gravity and fork/deck contacts; no attachment',
    development_runs_excluded=True,video='results/p6/demo/mission_demo.mp4')
(out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
source_files=sorted(p for p in (root/'src').glob('forklift_*/**/*') if p.is_file() and (p.name=='CMakeLists.txt' or p.suffix in ('.py','.cpp','.hpp','.yaml','.sdf','.xacro','.xml','.action','.msg','.config')) and '__pycache__' not in str(p))
(out/'source_manifest.json').write_text(json.dumps({str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in source_files},indent=2)+'\n')
print(json.dumps({k:summary[k] for k in ['gate','passed','unit_tests_passed','metrics']},indent=2))
raise SystemExit(0 if summary['passed'] else 1)
