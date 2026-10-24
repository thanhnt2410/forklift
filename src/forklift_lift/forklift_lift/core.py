# SPDX-License-Identifier: Apache-2.0
"""Deterministic lift supervisor; I/O and actions are adapters around this core."""
import math


class LiftCore:
    def __init__(self, config):
        self.p=config;self.sensor=None;self.received=0.;self.homed=False;self.offset=0.
        self.fault='';self.estopped=False;self.active=False;self.kind='';self.state='UNHOMED'
        self.target=0.;self.outcome=None;self.sequence=0;self.enabled=False
        self.command_position=0.;self.duration=.2;self.deadline=0.;self.phase_deadline=0.
        self.settled_since=None;self.progress_position=0.;self.progress_time=0.

    @property
    def height(self):
        return self.sensor.position-self.offset if self.sensor else 0.

    def fresh(self, now):
        return self.sensor is not None and now-self.received<self.p['state_timeout'] and self.sensor.feedback_valid

    def healthy(self, now):
        return (self.fresh(now) and self.sensor.lower_valid and self.sensor.upper_valid
                and self.sensor.controller_ok and not self.estopped)

    def inhibit(self, now):
        return not self.healthy(now) or not self.homed or self.active or bool(self.fault)

    def receive(self, sensor, now):
        if all(math.isfinite(v) for v in (sensor.position,sensor.velocity)):
            self.sensor=sensor;self.received=now

    def acceptable(self, kind, height, timeout, now):
        return (not self.active and not self.fault and self.healthy(now)
                and self.sensor.base_stationary and math.isfinite(timeout) and 0<=timeout<=180
                and (kind=='home' or (self.homed and math.isfinite(height) and 0<=height<=self.p['max_height'])))

    def command(self, position=None, velocity=None):
        self.sequence+=1
        self.enabled=position is not None
        if self.enabled:
            self.command_position=float(position)
            self.duration=max(.25,1.5*abs(position-self.sensor.position)/velocity)
        else:self.duration=.15

    def segment(self, state, position, velocity, now):
        self.state=state;self.settled_since=None
        self.command(position,velocity);self.phase_deadline=now+3*self.duration+1.
        self.progress_position=self.sensor.position;self.progress_time=now

    def start(self, kind, height, timeout, now):
        if not self.acceptable(kind,height,timeout,now):return False
        self.active=True;self.kind=kind;self.outcome=None;self.target=height
        self.deadline=now+(timeout or (self.p['home_timeout'] if kind=='home' else
                         max(8.,4.5*abs(self.height-height)/self.p['move_velocity']+4.)))
        if kind=='home':
            self.homed=False
            if self.sensor.lower_active:self.backoff(now)
            else:self.segment('HOME_SEEK',0.,self.p['home_fast_velocity'],now)
        else:self.segment('MOVING',self.offset+height,self.p['move_velocity'],now)
        return True

    def backoff(self, now):
        target=self.sensor.position+self.p['home_backoff']
        if target>self.p['max_height']:self.fail('HOME_BACKOFF_RANGE')
        else:self.segment('HOME_BACKOFF',target,self.p['home_fast_velocity']/2,now)

    def finish(self, success, code, canceled=False):
        self.command();self.active=False
        if not success and self.kind=='home':self.homed=False
        self.state='IDLE' if self.homed else 'UNHOMED'
        self.outcome=dict(success=success,code=code,canceled=canceled,height=self.height,homed=self.homed)

    def fail(self, code):
        if not self.fault:
            self.fault=code;self.homed=False;self.command()
            if self.active:
                self.active=False
                self.outcome=dict(success=False,code=code,canceled=False,height=self.height,homed=False)
        self.state='FAULT'

    def reset(self, now):
        if (self.active or not self.healthy(now) or not self.sensor.base_stationary
                or self.sensor.upper_active or abs(self.sensor.velocity)>.002):return False
        if self.fault=='HOME_SWITCH_STUCK' and self.sensor.lower_active:return False
        if self.fault=='HOME_SWITCH_NOT_FOUND' and not self.sensor.lower_active:return False
        self.fault='';self.homed=False;self.state='UNHOMED';self.command();return True

    def settled(self, condition, now):
        if not condition:self.settled_since=None;return False
        if self.settled_since is None:self.settled_since=now
        return now-self.settled_since>=.15

    def tick(self, now):
        if self.estopped:self.fail('ESTOP');return
        if not self.active and not self.homed:return
        if not self.fresh(now):self.fail('FEEDBACK_STALE');return
        s=self.sensor
        if not s.lower_valid or not s.upper_valid:self.fail('SWITCH_CIRCUIT');return
        if not s.controller_ok:self.fail('ACTUATOR_NOT_READY');return
        if s.position<-.002 or s.position>.252:self.fail('PHYSICAL_RANGE');return
        if not self.active:return
        if not s.base_stationary:self.fail('BASE_MOVING');return
        if now>self.deadline:self.fail('TIMEOUT');return
        if s.upper_active and self.enabled and self.command_position>s.position+.0001:
            self.fail('UPPER_LIMIT');return
        stopped=abs(s.velocity)<.002
        if self.state=='HOME_SEEK':
            if s.lower_active:self.backoff(now)
            elif now>self.phase_deadline:self.fail('HOME_SWITCH_NOT_FOUND')
        elif self.state=='HOME_BACKOFF':
            reached=abs(s.position-self.command_position)<.001 and stopped
            if self.settled(reached and not s.lower_active,now):
                self.segment('HOME_APPROACH',0.,self.p['home_slow_velocity'],now)
            elif now>self.phase_deadline:self.fail('HOME_SWITCH_STUCK' if s.lower_active else 'HOME_BACKOFF_TIMEOUT')
        elif self.state=='HOME_APPROACH':
            if s.lower_active:
                self.command();self.state='HOME_SETTLE';self.settled_since=None
                self.phase_deadline=now+2.
            elif now>self.phase_deadline:self.fail('HOME_SWITCH_NOT_FOUND')
        elif self.state=='HOME_SETTLE':
            if self.settled(s.lower_active and stopped,now):
                self.offset=s.position;self.homed=True;self.finish(True,'HOMED')
            elif now>self.phase_deadline:self.fail('HOME_SWITCH_UNSTABLE')
        elif self.state=='MOVING':
            if s.lower_active and self.command_position<s.position-.0001 and self.target>.004:
                self.fail('LOWER_LIMIT');return
            if self.settled(abs(self.height-self.target)<self.p['position_tolerance'] and stopped,now):
                self.finish(True,'SUCCEEDED')
            elif abs(s.position-self.progress_position)>.0005:
                self.progress_position=s.position;self.progress_time=now
            elif now-self.progress_time>2.5 and abs(self.height-self.target)>.003:
                self.fail('NO_PROGRESS')
