#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""PS4 / SDL teleop for the existing SIM command gate and lift actions."""
import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.clock import Clock
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from std_srvs.srv import SetBool, Trigger
from forklift_interfaces.action import HomeLift, MoveLift
from forklift_interfaces.msg import LiftState


class PadState:
    """Wall-clock expiry and a neutral/release handshake across reconnects."""
    def __init__(self, axes, buttons, timeout, neutral):
        self.axes, self.buttons = axes, buttons
        self.timeout, self.neutral = timeout, neutral
        self.reset()

    def reset(self):
        self.stamp = None
        self.values = [0., 0., 0.]
        self.previous = []
        self.released = False
        self.armed = False

    def fresh(self, now):
        return self.stamp is not None and 0. <= now - self.stamp < self.timeout

    def enabled(self, now):
        return self.fresh(now) and self.armed

    def centered(self):
        return all(abs(v) <= self.neutral for v in self.values)

    def receive(self, axes, buttons, now):
        if not self.fresh(now):
            self.reset()
        if (len(axes) <= max(self.axes) or len(buttons) <= max(self.buttons.values()) or
                not all(math.isfinite(v) and abs(v) <= 1.001 for v in axes) or
                any(v not in (0, 1) for v in buttons)):
            self.reset()
            return set()
        rising = {name for name, i in self.buttons.items()
                  if buttons[i] and (not self.previous or not self.previous[i])}
        held = bool(buttons[self.buttons['enable']])
        self.values = [max(-1., min(1., axes[i])) for i in self.axes]
        if not held:
            self.armed = False
            self.released = self.centered()
        elif not self.armed and 'enable' in rising and self.released and self.centered():
            self.armed = True
        self.stamp, self.previous = now, list(buttons)
        return rising


class Ps4Teleop(Node):
    def __init__(self):
        super().__init__('ps4_teleop')
        defaults = dict(axis_forward=1, axis_lateral=0, axis_yaw=2,
                        button_enable=9, button_home=3, button_up=11,
                        button_down=12, button_stop=1, speed_forward=.15,
                        speed_lateral=.12, speed_yaw=.4, joy_timeout=.25,
                        state_timeout=.5, neutral_tolerance=.05,
                        lift_step=.01, lift_max_height=.115)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = {key: self.get_parameter(key).value for key in defaults}
        for key, value in self.p.items():
            if key.startswith(('axis_', 'button_')):
                if not isinstance(value, int) or value < 0:
                    raise ValueError(f'{key} must be a nonnegative index')
            elif not math.isfinite(value) or value <= 0:
                raise ValueError(f'{key} must be finite and positive')
        if self.p['neutral_tolerance'] >= 1. or self.p['joy_timeout'] > .5:
            raise ValueError('Invalid neutral tolerance or Joy timeout')
        self.pad = PadState([self.p['axis_' + k] for k in ('forward', 'lateral', 'yaw')],
                            {k: self.p['button_' + k] for k in ('enable', 'home', 'up', 'down', 'stop')},
                            self.p['joy_timeout'], self.p['neutral_tolerance'])
        self.pub = self.create_publisher(Twist, '/cmd_vel_teleop', 10)
        self.mode = self.create_client(SetBool, '/command_gate/set_autonomous')
        self.stop = self.create_client(Trigger, '/lift/stop')
        self.home = ActionClient(self, HomeLift, '/lift/home')
        self.move = ActionClient(self, MoveLift, '/lift/move')
        self.manual = False
        self.mode_future = None
        self.last_mode_request = -math.inf
        self.lift = self.odom = None
        self.lift_stamp = self.odom_stamp = -math.inf
        self.pending = self.cancel_requested = False
        self.closing = False
        self.lift_handle = None
        self.create_subscription(Joy, '/joy', self.joy,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(LiftState, '/lift/state', self.lift_state,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Odometry, '/odom', self.odometry, 10)
        self.create_timer(.02, self.tick, clock=Clock())
        self.get_logger().info('PS4 SIM teleop: release L1 and center sticks, then hold L1 to enable. '
                               'Circle stops; D-pad up/down moves lift; Triangle homes.')

    def lift_state(self, msg):
        self.lift, self.lift_stamp = msg, time.monotonic()

    def odometry(self, msg):
        self.odom, self.odom_stamp = msg, time.monotonic()

    def lift_fresh(self, now):
        return (self.lift is not None and now - self.lift_stamp < self.p['state_timeout'] and
                math.isfinite(self.lift.height) and not self.lift.estopped and not self.lift.fault_code)

    def stationary(self, now):
        if self.odom is None or now - self.odom_stamp >= self.p['state_timeout']:
            return False
        v = self.odom.twist.twist
        return all(math.isfinite(x) and abs(x) < .01 for x in (v.linear.x, v.linear.y, v.angular.z))

    def joy(self, msg):
        if self.closing:
            return
        now = time.monotonic()
        rising = self.pad.receive(msg.axes, msg.buttons, now)
        if self.pad.previous and self.pad.previous[self.p['button_stop']]:
            self.pad.armed = self.pad.released = False
            self.pub.publish(Twist())
            if 'stop' in rising:
                self.stop_lift()
            return
        if not self.pad.enabled(now):
            self.pub.publish(Twist())
            if self.pending and not self.cancel_requested:
                self.stop_lift()
            return
        actions = rising & {'home', 'up', 'down'}
        if not actions:
            return
        if (len(actions) != 1 or not self.manual or self.pending or not self.pad.centered() or
                not self.stationary(now) or not self.lift_fresh(now) or self.lift.moving):
            self.get_logger().warning('Lift button ignored: center sticks and wait for fresh, stationary lift/base.')
            return
        kind = actions.pop()
        if kind == 'home':
            client, goal = self.home, HomeLift.Goal(timeout=60.)
        else:
            if not self.lift.homed:
                self.get_logger().warning('Home the lift before using D-pad up/down.')
                return
            target = max(0., min(self.p['lift_max_height'], self.lift.height +
                                (1. if kind == 'up' else -1.) * self.p['lift_step']))
            client, goal = self.move, MoveLift.Goal(height=target, timeout=15.)
        if not client.server_is_ready():
            self.get_logger().warning('Lift action server is not available.')
            return
        self.pub.publish(Twist())
        self.pending, self.cancel_requested = True, False
        client.send_goal_async(goal).add_done_callback(self.goal_response)

    def goal_response(self, future):
        try:
            self.lift_handle = future.result()
            if not self.lift_handle.accepted:
                self.pending = False
                self.lift_handle = None
                self.get_logger().warning('Lift goal rejected by supervisor.')
                return
            self.lift_handle.get_result_async().add_done_callback(self.lift_result)
            if self.cancel_requested or not self.pad.enabled(time.monotonic()):
                self.stop_lift()
        except Exception as error:
            self.pending = False
            self.get_logger().error(f'Lift request failed: {error}')

    def lift_result(self, future):
        self.pending = False
        self.lift_handle = None
        try:
            result = future.result().result
            self.get_logger().info(f'Lift result: {result.code}')
        except Exception as error:
            self.get_logger().error(f'Lift result unavailable: {error}')

    def stop_lift(self):
        self.cancel_requested = True
        if self.lift_handle is not None:
            self.lift_handle.cancel_goal_async()
        if self.stop.service_is_ready():
            self.stop.call_async(Trigger.Request())

    def mode_response(self, future):
        self.mode_future = None
        try:
            self.manual = bool(future.result().success)
        except Exception as error:
            self.get_logger().warning(f'Manual-mode request failed: {error}')

    def tick(self):
        if self.closing:
            self.pub.publish(Twist())
            return
        now = time.monotonic()
        if not self.manual and self.mode_future is None and now - self.last_mode_request >= 1.:
            self.last_mode_request = now
            if self.mode.service_is_ready():
                self.mode_future = self.mode.call_async(SetBool.Request(data=False))
                self.mode_future.add_done_callback(self.mode_response)
        if not self.pad.enabled(now) and self.pending and not self.cancel_requested:
            self.stop_lift()
        out = Twist()
        if (self.manual and self.pad.enabled(now) and not self.pending and self.lift_fresh(now) and
                self.lift.homed and not self.lift.moving and not self.lift.inhibited):
            out.linear.x = self.pad.values[0] * self.p['speed_forward']
            out.linear.y = self.pad.values[1] * self.p['speed_lateral']
            out.angular.z = self.pad.values[2] * self.p['speed_yaw']
        self.pub.publish(out)


def main():
    # Keep the context alive during Ctrl+C cleanup so pending action requests
    # can be cancelled, including an acceptance arriving during shutdown.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Ps4Teleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.closing = True
            node.pad.reset()
            node.pub.publish(Twist())
            node.stop_lift()
            until = time.monotonic() + .2
            while time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=.02)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
