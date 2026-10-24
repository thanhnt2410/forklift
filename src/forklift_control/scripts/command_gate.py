#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Select/limit motion commands. Wall-clock expiry remains valid if SIM pauses."""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


class CommandGate(Node):
    def __init__(self):
        super().__init__('command_gate')
        defaults = dict(command_timeout=.3, state_timeout=.5, max_vx=.25,
                        max_vy=.2, max_wz=.8, max_ax=.5, max_ay=.5, max_aw=1.5,
                        max_wheel_speed=12., wheel_radius=.05,
                        kinematic_sum=.2275, transport_height=.02)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = {key: float(self.get_parameter(key).value) for key in defaults}
        if not all(math.isfinite(v) and v > 0 for v in self.p.values()):
            raise ValueError('All gate limits must be finite and positive')
        self.sources = {}
        self.declare_parameter('autonomous_enabled', False)
        self.autonomous = bool(self.get_parameter('autonomous_enabled').value)
        self.declare_parameter('require_navigation_ready', False)
        self.require_navigation_ready = bool(self.get_parameter('require_navigation_ready').value)
        self.navigation_ready = False
        self.navigation_stamp = 0.
        self.estop = False
        self.state = None
        self.lift_busy = True
        self.lift_stamp = 0.
        self.output = [0., 0., 0.]
        self.last_tick = time.monotonic()
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.stop_pub = self.create_publisher(Bool, '/safety/estop', latched)
        self.stop_pub.publish(Bool(data=False))
        for name in ('teleop', 'nav'):
            self.create_subscription(Twist, f'/cmd_vel_{name}',
                                     lambda m, n=name: self.receive(n, m), 10)
        self.create_subscription(JointState, '/joint_states', self.joints, 10)
        self.create_subscription(Bool, '/lift/moving', self.lift, latched)
        self.create_subscription(Bool, '/navigation/ready', self.navigation, 10)
        self.create_service(SetBool, '~/estop', self.stop)
        self.create_service(SetBool, '~/set_autonomous', self.mode)
        self.create_timer(.02, self.tick, clock=rclpy.clock.Clock())

    def receive(self, source, msg):
        values = (msg.linear.x, msg.linear.y, msg.angular.z)
        unsupported = (msg.linear.z, msg.angular.x, msg.angular.y)
        if not all(math.isfinite(v) for v in values + unsupported) or any(unsupported):
            # Invalid input from the selected publisher is an explicit stop.
            self.sources[source] = ([0., 0., 0.], time.monotonic())
            return
        if not self.estop:
            self.sources[source] = (values, time.monotonic())

    def joints(self, msg):
        if 'lift_joint' in msg.name:
            i = msg.name.index('lift_joint')
            if i < len(msg.position) and i < len(msg.velocity):
                pos, vel = msg.position[i], msg.velocity[i]
                if math.isfinite(pos) and math.isfinite(vel):
                    self.state = (pos, vel, time.monotonic())

    def lift(self, msg):
        self.lift_busy = msg.data
        self.lift_stamp = time.monotonic()

    def navigation(self, msg):
        self.navigation_ready = msg.data
        self.navigation_stamp = time.monotonic()

    def stop(self, req, res):
        self.estop = req.data
        self.sources.clear()
        self.output = [0., 0., 0.]
        self.stop_pub.publish(Bool(data=self.estop))
        self.pub.publish(Twist())
        res.success = True
        res.message = 'Stopped' if self.estop else 'Released; send a fresh command'
        return res

    def mode(self, req, res):
        self.autonomous = req.data
        self.sources.clear()
        self.output = [0., 0., 0.]
        self.pub.publish(Twist())
        res.success = True
        res.message = 'Autonomous with teleop priority' if req.data else 'Manual only'
        return res

    def tick(self):
        now = time.monotonic()
        dt = min(now-self.last_tick, .1)
        self.last_tick = now
        selected = None
        selected_source = None
        for source in (('teleop', 'nav') if self.autonomous else ('teleop',)):
            if source in self.sources and now-self.sources[source][1] <= self.p['command_timeout']:
                selected = self.sources[source][0]
                selected_source = source
                break
        allowed = (not self.estop and not self.lift_busy and self.state is not None
                   and now-self.lift_stamp <= self.p['state_timeout']
                   and now-self.state[2] <= self.p['state_timeout']
                   and self.state[0] <= self.p['transport_height']
                   and abs(self.state[1]) < .002)
        if selected_source == 'nav' and self.require_navigation_ready:
            # Sensor/localization interlocks bypass the normal command ramp.
            allowed = allowed and self.navigation_ready and now-self.navigation_stamp < .3
        if not allowed or selected is None:
            # Timeout/interlock bypasses the normal acceleration ramp.
            self.output = [0., 0., 0.]
        else:
            target = [max(-self.p[k], min(self.p[k], v))
                      for k, v in zip(('max_vx', 'max_vy', 'max_wz'), selected)]
            norm = (abs(target[0])+abs(target[1])+self.p['kinematic_sum']*abs(target[2]))/self.p['wheel_radius']
            scale = min(1., self.p['max_wheel_speed']/max(norm, 1.e-9))
            target = [v*scale for v in target]
            self.output = [old + max(-self.p[a]*dt, min(self.p[a]*dt, new-old))
                           for old, new, a in zip(self.output, target,
                                                 ('max_ax', 'max_ay', 'max_aw'))]
            # Independent axis ramps can otherwise briefly leave the wheel
            # envelope while changing between two valid combined commands.
            norm = (abs(self.output[0])+abs(self.output[1])+
                    self.p['kinematic_sum']*abs(self.output[2]))/self.p['wheel_radius']
            scale = min(1., self.p['max_wheel_speed']/max(norm, 1.e-9))
            self.output = [v*scale for v in self.output]
        out = Twist()
        out.linear.x, out.linear.y, out.angular.z = self.output
        self.pub.publish(out)


def main():
    rclpy.init()
    node = CommandGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.pub.publish(Twist())
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
