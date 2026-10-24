#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Run isolated P6 fault cases sequentially to avoid contention between simulators."""
import json
from pathlib import Path
import subprocess
import sys

root=Path(__file__).resolve().parents[1];out=root/'results/p6/faults';out.mkdir(parents=True,exist_ok=True)
scenarios=['cancel','scan_loss','imu_loss','tf_loss','estop','missing_pallet','misaligned','blocked','lift_fault','load_loss','cancel_loaded']
results=[]
for index,scenario in enumerate(scenarios):
    print('P6 fault case:',scenario,flush=True)
    directory=out/scenario
    result=subprocess.run([sys.executable,str(root/'scripts/run_p6_checks.py'),'--domain',str(140+index),
                           '--scenario',scenario,'--output',str(directory)],cwd=root)
    summary=json.loads((directory/'summary.json').read_text()) if (directory/'summary.json').exists() else {}
    results.append(dict(scenario=scenario,exit_code=result.returncode,passed=bool(summary.get('passed')),summary=summary))
    (out/'summary.json').write_text(json.dumps(dict(passed=all(x['passed'] for x in results) and len(results)==len(scenarios),
        scenarios_passed=sum(x['passed'] for x in results),scenarios_total=len(scenarios),cases=results,physical_hardware_used=False),indent=2)+'\n')
sys.exit(0 if all(x['passed'] for x in results) else 1)
