#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""P1 simulation helper, NOT a hardware homing or load holding controller."""
import math
import time

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class LiftSim(Node):
    def __init__(self):
        super().__init__('lift_sim')
        self.declare_parameter('travel', .25)
        self.declare_parameter('velocity', .04)
        self.travel = float(self.get_parameter('travel').value)
        self.velocity = float(self.get_parameter('velocity').value)
        if not all(math.isfinite(v) and v > 0 for v in (self.travel, self.velocity)):
            raise ValueError('Invalid lift limits')
        self.state = None
        self.target = None
        self.deadline = 0.
        self.stopped = False
        self.result = 'SIM_READY_NOT_HOMED_HARDWARE'
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.busy_pub = self.create_publisher(Bool, '/lift/moving', latched)
        self.status_pub = self.create_publisher(String, '/lift/status', 10)
        self.trajectory_pub = self.create_publisher(JointTrajectory, '/lift_controller/joint_trajectory', 10)
        self.create_subscription(Float64, '/lift/target', self.command, 10)
        self.create_subscription(JointState, '/joint_states', self.joints, 10)
        self.create_subscription(Bool, '/safety/estop', self.estop, latched)
        self.create_service(Trigger, '/lift/stop', self.cancel)
        self.create_timer(.05, self.tick, clock=rclpy.clock.Clock())

    def joints(self, msg):
        if 'lift_joint' in msg.name:
            i = msg.name.index('lift_joint')
            if i < len(msg.position) and i < len(msg.velocity):
                p, v = msg.position[i], msg.velocity[i]
                if math.isfinite(p) and math.isfinite(v):
                    self.state = (p, v, time.monotonic())

    def send(self, target, seconds):
        msg = JointTrajectory(joint_names=['lift_joint'])
        point = JointTrajectoryPoint(positions=[target], velocities=[0.])
        point.time_from_start = Duration(sec=int(seconds), nanosec=int(seconds % 1 * 1.e9))
        msg.points = [point]
        self.trajectory_pub.publish(msg)

    def command(self, msg):
        if (not math.isfinite(msg.data) or not 0 <= msg.data <= self.travel
                or self.stopped or self.state is None or time.monotonic()-self.state[2] > .5):
            self.get_logger().warning('Rejected lift goal: invalid/range/estop/stale state')
            return
        self.target = msg.data
        seconds = max(.5, abs(self.target-self.state[0])/self.velocity*1.5)
        self.deadline = time.monotonic()+seconds+3.
        self.result = 'MOVING'
        self.busy_pub.publish(Bool(data=True))
        self.send(self.target, seconds)

    def halt(self, result):
        self.target = None
        self.result = result
        if self.state is not None:
            self.send(max(0., min(self.travel, self.state[0])), .15)

    def estop(self, msg):
        self.stopped = msg.data
        if self.stopped:
            self.halt('ESTOP')

    def cancel(self, req, res):
        self.halt('CANCELLED')
        res.success = True
        res.message = 'Holding current simulated position'
        return res

    def tick(self):
        now = time.monotonic()
        stale = self.state is None or now-self.state[2] > .5
        if self.target is not None:
            if stale or now > self.deadline:
                self.halt('FAULT_STALE_OR_TIMEOUT')
            elif abs(self.state[0]-self.target) < .003 and abs(self.state[1]) < .002:
                self.target = None
                self.result = 'SUCCEEDED'
        self.busy_pub.publish(Bool(data=stale or self.stopped or self.target is not None))
        self.status_pub.publish(String(data=self.result))


def main():
    rclpy.init()
    node = LiftSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
