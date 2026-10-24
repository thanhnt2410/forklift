# SPDX-License-Identifier: Apache-2.0
"""Nominal station geometry for planning, never simulator ground truth."""
import math
import json
import hashlib
from pathlib import Path

def station_poses(parameters, station):
    angle=station['approach'][2];c,s=math.cos(angle),math.sin(angle)
    def at(distance):
        return [station['face'][0]-distance*c,station['face'][1]-distance*s,angle]
    inserted=at(parameters['inserted_distance'])
    along=(inserted[0]-station['approach'][0])*c+(inserted[1]-station['approach'][1])*s
    distance=parameters['retreat_distance']
    if parameters.get('withdrawal_policy','turn_setup')=='turn_setup':
        distance=max(distance,along+parameters.get('terminal_distance',0.)+parameters.get('turn_setup_distance',0.))
    retreat=[inserted[0]-distance*c,inserted[1]-distance*s,angle]
    return at(parameters['preinsert_distance']),inserted,retreat

def straight_samples(start, end, step=.02):
    n=max(1,math.ceil(math.hypot(end[0]-start[0],end[1]-start[1])/step))
    return [[start[0]+(end[0]-start[0])*i/n,start[1]+(end[1]-start[1])*i/n,end[2]] for i in range(n+1)]


def load_approved_route(filename, parameters):
    raw=Path(filename).read_bytes();data=json.loads(raw)
    names=['NAV_PICK','ALIGN_PICK','INSERT_PICK','RETREAT_PICK','NAV_DROP','ALIGN_DROP','INSERT_DROP','RETREAT_DROP','RETURN_HOME']
    if [s['stage'] for s in data['segments']]!=names:raise ValueError('Approved route stages do not match mission')
    for segment in data['segments']:
        if len(segment['poses'])<2 or any(len(p)!=3 or not all(math.isfinite(v) for v in p) for p in segment['poses']):
            raise ValueError('Approved route contains invalid poses')
    if parameters.get('withdrawal_policy')!='clearance' or abs(data['withdrawal_clearance']-parameters['retreat_distance'])>.001:
        raise ValueError('Approved withdrawal and mission configuration differ')
    goals={'NAV_PICK':parameters['a']['approach'],'NAV_DROP':parameters['b']['approach'],'RETURN_HOME':parameters['home']}
    for segment in data['segments']:
        if segment['stage'] not in goals:continue
        end=segment['poses'][-1];goal=goals[segment['stage']]
        angle=math.atan2(math.sin(end[2]-goal[2]),math.cos(end[2]-goal[2]))
        if math.hypot(end[0]-goal[0],end[1]-goal[1])>.06 or abs(angle)>.06:
            raise ValueError('Approved endpoint differs from mission goal')
    data['sha256']=hashlib.sha256(raw).hexdigest()
    return data
