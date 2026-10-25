#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Terminal input for the command gate and lift actions (Gazebo or ESP32 backend)."""
import math
import os
import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.action import ActionClient
from rclpy.clock import Clock
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool, Trigger
from std_msgs.msg import Float64
from forklift_interfaces.action import HomeLift, MoveLift
from forklift_interfaces.msg import LiftState


KEYS = {
    '\x1b[A': 'forward', '\x1b[B': 'reverse',
    '\x1b[D': 'left', '\x1b[C': 'right',
    '\x1bOA': 'forward', '\x1bOB': 'reverse',
    '\x1bOD': 'left', '\x1bOC': 'right',
    '\x1b[1;2D': 'strafe_left', '\x1b[1;2C': 'strafe_right',
    'a': 'strafe_left', 'd': 'strafe_right',
    'r': 'lift_up', 'f': 'lift_down', 'h': 'home',
    '\x1b[5~': 'lift_up', '\x1b[6~': 'lift_down',
    '\x1b[H': 'home', '\x1bOH': 'home', '\x1b[1~': 'home',
    ' ': 'stop', 's': 'stop', 'q': 'quit', '\x03': 'quit',
}


def read_key(fd, timeout=.02):
    """Drain one complete ANSI key sequence; unknown sequences are stops."""
    if not select.select([fd], [], [], timeout)[0]:
        return None
    data = os.read(fd, 1)
    if not data:
        return 'quit'
    key = data.decode('latin1')
    if key == '\x1b':
        until = time.monotonic() + .05
        while len(key) < 16:
            remaining = until - time.monotonic()
            if remaining <= 0 or not select.select([fd], [], [], remaining)[0]:
                return 'stop'
            data = os.read(fd, 1)
            if not data:
                return 'quit'
            key += data.decode('latin1')
            if len(key) == 2 and key[1] not in ('[', 'O'):
                return 'stop'
            if len(key) >= 3 and '@' <= key[-1] <= '~':
                break
    # Normalize letters only: lowercasing an ANSI sequence breaks arrow keys.
    if len(key) == 1 and 'A' <= key <= 'Z':
        key = key.lower()
    return KEYS.get(key, 'stop')


class ArrowTeleop(Node):
    def __init__(self):
        super().__init__('arrow_teleop')
        defaults = dict(speed=.15, reverse_speed=.12, strafe_speed=.12,
                        turn=.4, key_timeout=.35, state_timeout=.5,
                        lift_step=.01, lift_max_height=.115)
        limits = dict(speed=.25, reverse_speed=.16, strafe_speed=.2,
                      turn=.6, key_timeout=1., state_timeout=1.,
                      lift_step=.115, lift_max_height=.115)
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in defaults}
        for name, value in self.p.items():
            if not math.isfinite(value) or not 0. < value <= limits[name]:
                raise ValueError(f'{name} must be positive and <= {limits[name]}')
        self.pub = self.create_publisher(Twist, '/cmd_vel_teleop', 10)
        self.mode = self.create_client(SetBool, '/command_gate/set_autonomous')
        self.lift_stop = self.create_client(Trigger, '/lift/stop')
        self.lift_home = ActionClient(self, HomeLift, '/lift/home')
        self.lift_move = ActionClient(self, MoveLift, '/lift/move')
        self.lift = self.odom = None
        self.preset_target = None
        self.lift_stamp = self.odom_stamp = -math.inf
        self.pending = self.cancel_requested = self.closing = False
        self.lift_handle = None
        self.create_subscription(LiftState, '/lift/state', self.lift_state,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Odometry, '/odom', self.odometry, 10)
        self.create_subscription(Float64, '/lift/preset_target', self.pulse_preset,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.manual = False
        self.mode_future = None
        self.last_request = self.last_key = -math.inf
        self.last_lift_key = None
        self.last_lift_key_at = -math.inf
        self.command = (0., 0., 0.)
        self.create_timer(.02, self.tick, clock=Clock())

    def press(self, key):
        if self.closing:
            return
        commands = dict(forward=(self.p['speed'], 0., 0.),
                        reverse=(-self.p['reverse_speed'], 0., 0.),
                        left=(0., 0., self.p['turn']),
                        right=(0., 0., -self.p['turn']),
                        strafe_left=(0., self.p['strafe_speed'], 0.),
                        strafe_right=(0., -self.p['strafe_speed'], 0.))
        now = time.monotonic()
        blocked = self.pending or (self.lift is not None and self.lift.moving)
        self.command = commands.get(key, (0., 0., 0.)) if not blocked else (0., 0., 0.)
        self.last_key = now if not blocked else -math.inf
        if key not in commands:
            self.pub.publish(Twist())
            if key in ('lift_up', 'lift_down', 'home'):
                # Terminal key repeat must not queue more relative raises.
                repeated = key == self.last_lift_key and now-self.last_lift_key_at < self.p['key_timeout']
                self.last_lift_key, self.last_lift_key_at = key, now
                if not repeated:
                    self.start_lift(key, now)
            else:
                self.stop_lift()

    def lift_state(self, msg):
        self.lift, self.lift_stamp = msg, time.monotonic()

    def odometry(self, msg):
        self.odom, self.odom_stamp = msg, time.monotonic()

    def pulse_preset(self, msg):
        if math.isfinite(msg.data) and 0. < msg.data <= self.p['lift_max_height']:
            self.preset_target = msg.data
            self.get_logger().info('Manual lift: R adds one configured pulse increment; F/H lowers for up to 3 seconds or until the bottom switch.')

    def start_lift(self, key, now):
        if self.pending:
            return
        reason = None
        if not self.manual:
            reason = 'waiting for manual mode from command gate'
        elif self.lift is None or now - self.lift_stamp >= self.p['state_timeout']:
            reason = 'missing or stale /lift/state; check lift supervisor'
        elif self.odom is None or now - self.odom_stamp >= self.p['state_timeout']:
            reason = 'missing or stale /odom'
        elif self.lift.estopped:
            reason = 'lift E-stop is active'
        elif self.lift.fault_code:
            reason = f'lift fault: {self.lift.fault_code}'
        elif not math.isfinite(self.lift.height):
            reason = 'invalid lift height feedback'
        elif self.lift.moving:
            reason = 'lift is already moving'
        else:
            v = self.odom.twist.twist
            if not all(math.isfinite(x) and abs(x) < .01
                       for x in (v.linear.x, v.linear.y, v.angular.z)):
                reason = 'vehicle is not stationary; release drive keys and wait'
        if reason is not None:
            self.get_logger().warning(f'Lift key ignored: {reason}.')
            return
        if key == 'home' or (key == 'lift_down' and self.preset_target is not None):
            client, goal = self.lift_home, HomeLift.Goal(timeout=5. if self.preset_target is not None else 60.)
        else:
            if self.preset_target is None and not self.lift.homed:
                self.get_logger().warning('Press H to home the lift first.')
                return
            target = (self.preset_target if self.preset_target is not None else
                      max(0., min(self.p['lift_max_height'], self.lift.height +
                                  (1. if key == 'lift_up' else -1.) * self.p['lift_step'])))
            client, goal = self.lift_move, MoveLift.Goal(height=target, timeout=15.)
        if not client.server_is_ready():
            self.get_logger().warning('Lift action server is not available.')
            return
        self.last_key = -math.inf
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
            if self.cancel_requested or self.closing:
                self.stop_lift()
        except Exception as error:
            self.pending = False
            self.get_logger().error(f'Lift request failed: {error}')

    def lift_result(self, future):
        self.pending = False
        self.lift_handle = None
        self.command = (0., 0., 0.)
        self.last_key = -math.inf
        try:
            self.get_logger().info(f'Lift result: {future.result().result.code}')
        except Exception as error:
            self.get_logger().error(f'Lift result unavailable: {error}')

    def stop_lift(self):
        self.cancel_requested = True
        if self.lift_handle is not None:
            self.lift_handle.cancel_goal_async()
        if self.lift_stop.service_is_ready():
            self.lift_stop.call_async(Trigger.Request())

    def mode_response(self, future):
        self.mode_future = None
        try:
            self.manual = bool(future.result().success)
            # Require a new key after claiming manual mode; don't replay input
            # typed while waiting for the command gate to start.
            self.command = (0., 0., 0.)
            self.last_key = -math.inf
            if self.manual:
                self.get_logger().info('Manual mode ready; keyboard input enabled.')
        except Exception as error:
            self.get_logger().warning(f'Manual-mode request failed: {error}')

    def tick(self):
        now = time.monotonic()
        if not self.manual and self.mode_future is None and now - self.last_request >= 1.:
            self.last_request = now
            if self.mode.service_is_ready():
                self.mode_future = self.mode.call_async(SetBool.Request(data=False))
                self.mode_future.add_done_callback(self.mode_response)
        out = Twist()
        if self.pending or self.closing or (self.lift is not None and self.lift.moving):
            self.command = (0., 0., 0.)
            self.last_key = -math.inf
        elif self.manual and now - self.last_key < self.p['key_timeout']:
            out.linear.x, out.linear.y, out.angular.z = self.command
        self.pub.publish(out)


def main():
    if not sys.stdin.isatty():
        print('Open an interactive terminal to use arrow_teleop (stdin must be a TTY).', file=sys.stderr)
        return 1
    fd = sys.stdin.fileno()
    settings = termios.tcgetattr(fd)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = ArrowTeleop()
        print('Forklift keyboard: Up/Down = forward/reverse; Left/Right = turn; '
              'A/D or Shift+Left/Right = strafe.\n'
              'R/PageUp = raise; F/PageDown = lower; H/Home = homing.\n'
              'Position mode: 1 cm steps. Manual pulse mode: R raises one increment; F/H lowers up to 3 s or bottom switch.\n'
              'Letter keys work with or without Shift/Caps Lock.\n'
              'Space = stop wheels and lift; Q/Ctrl+C = stop and quit.\n'
              'Focus this terminal. Drive keys expire after 0.35 s by default; lift goals run to their target.',
              flush=True)
        tty.setcbreak(fd)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.)
            key = read_key(fd)
            if key is not None:
                node.press(key)
            if key == 'quit':
                break
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSANOW, settings)
        if node is not None and rclpy.ok():
            node.press('stop')
            node.closing = True
            until = time.monotonic() + .2
            while time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=.02)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
