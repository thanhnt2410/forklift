#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""ROS manual-control bridge for ESP32 plain CSV (optional legacy UART v2)."""
import binascii
from dataclasses import dataclass
import errno
import fcntl
import math
import os
import re
import struct
import termios
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.clock import Clock
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from rclpy.task import Future
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, Header, String
from std_srvs.srv import SetBool, Trigger
from tf2_msgs.msg import TFMessage
from forklift_interfaces.action import HomeLift, MoveLift
from forklift_interfaces.msg import LiftState

NAMES = ['front_left_wheel_joint', 'front_right_wheel_joint',
         'rear_left_wheel_joint', 'rear_right_wheel_joint', 'lift_joint']
MODES = ('LOCKED', 'DISARMED', 'ACTIVE', 'HOMING', 'FAULTED')


def frame(payload):
    data = payload.encode('ascii')
    result = data + f'*{binascii.crc32(data):08x}\n'.encode('ascii')
    if len(result) > 256:
        raise ValueError('UART frame exceeds 256 bytes')
    return result


@dataclass(frozen=True)
class State:
    boot: int
    seq: int
    us: int
    ack: int
    ticks: tuple
    steps: int
    fault: int
    mode: int
    homed: bool
    low: bool
    high: bool
    estop: bool
    capabilities: int


def parse_state(line):
    """Validate the firmware's exact 18-field telemetry, including integer ranges."""
    if not 10 <= len(line) <= 256 or not line.endswith(b'\n'):
        raise ValueError('invalid frame length/terminator')
    payload, checksum = line[:-1].rsplit(b'*', 1)
    if not re.fullmatch(rb'[0-9a-fA-F]{8}', checksum) or binascii.crc32(payload) != int(checksum, 16):
        raise ValueError('CRC mismatch')
    fields = payload.decode('ascii').split(',')
    if len(fields) != 18 or fields[:2] != ['S', '2']:
        raise ValueError('not UART v2 telemetry')
    nums = []
    for i, value in enumerate(fields[2:], 2):
        signed = 6 <= i <= 10
        if not re.fullmatch(r'-?[0-9]+' if signed else r'[0-9]+', value):
            raise ValueError('invalid integer')
        n = int(value)
        if not (-(2**63) <= n < 2**63 if signed else 0 <= n < 2**64):
            raise ValueError('integer overflow')
        nums.append(n)
    boot, seq, us, ack, *rest = nums
    ticks, steps, fault, mode, homed, low, high, estop, caps = tuple(rest[:4]), *rest[4:]
    if not seq or fault > 255 or mode > 4 or caps not in (0, 1, 3) or any(v not in (0, 1) for v in (homed, low, high, estop)):
        raise ValueError('invalid state enumeration')
    return State(boot, seq, us, ack, ticks, steps, fault, mode,
                 bool(homed), bool(low), bool(high), bool(estop), caps)


def parse_simple_state(line):
    """S,ms,tFL,tFR,tRL,tRR,lift_steps,mode,homed,lower,fault,capabilities."""
    if not 1 <= len(line) <= 256 or not line.endswith(b'\n'):
        raise ValueError('invalid CSV line')
    fields = line.rstrip(b'\r\n').decode('ascii').split(',')
    if len(fields) != 12 or fields[0] != 'S':
        raise ValueError('expected plain S CSV')
    values = []
    for index, field in enumerate(fields[1:], 1):
        signed = 2 <= index <= 6
        if not re.fullmatch(r'-?[0-9]+' if signed else r'[0-9]+', field):
            raise ValueError('invalid CSV integer')
        value = int(field)
        if not (-(2**63) <= value < 2**63 if signed else 0 <= value < 2**64):
            raise ValueError('CSV integer overflow')
        values.append(value)
    ms, fl, fr, rl, rr, steps, mode, homed, low, fault, caps = values
    if ms >= 2**64//1000 or mode > 4 or homed not in (0, 1) or low not in (0, 1) or fault > 255 or caps not in (0, 1, 3):
        raise ValueError('invalid CSV state values')
    return State(0, 0, ms*1000, 0, (fl, fr, rl, rr), steps, fault, mode,
                 bool(homed), bool(low), False, bool(fault & 2), caps)


def inverse(vx, vy, wz, radius, k):
    return ((vx-vy-k*wz)/radius, (vx+vy+k*wz)/radius,
            (vx+vy-k*wz)/radius, (vx-vy+k*wz)/radius)


def forward(wheels, radius, k):
    fl, fr, rl, rr = wheels
    return (radius*(fl+fr+rl+rr)/4, radius*(-fl+fr+rl-rr)/4,
            radius*(-fl+fr-rl+rr)/(4*k))


class SerialPort:
    """Nonblocking raw 8N1 serial, with exclusive ownership and bounded framing."""
    def __init__(self, path, baud):
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK | os.O_CLOEXEC)
        self.buffer = bytearray()
        self.discard = False
        self.rx_bytes = self.rx_lines = self.discarded_frames = 0
        self.modem_lines = 'unsupported'
        try:
            fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            fcntl.ioctl(self.fd, termios.TIOCEXCL)
            attrs = termios.tcgetattr(self.fd)
            attrs[0] = attrs[1] = attrs[3] = 0
            attrs[2] = termios.CS8 | termios.CLOCAL | termios.CREAD
            attrs[4] = attrs[5] = getattr(termios, f'B{baud}')
            attrs[6][termios.VMIN] = attrs[6][termios.VTIME] = 0
            termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
            # CP210x DTR/RTS can drive GPIO0/EN on ESP32 DevKits. Release
            # GPIO0 before EN, so an open port cannot hold reset/download mode.
            # HUPCL and RTS/CTS flow control are already clear in attrs[2].
            try:
                for bit in (termios.TIOCM_DTR, termios.TIOCM_RTS):
                    fcntl.ioctl(self.fd, termios.TIOCMBIC, struct.pack('I', bit))
                self.modem_lines = 'released'
            except OSError as error:
                # PTYs have no modem-control pins; retain virtual-MCU support.
                if error.errno not in (errno.ENOTTY, errno.EINVAL, errno.EOPNOTSUPP):
                    raise
            termios.tcflush(self.fd, termios.TCIOFLUSH)
        except Exception:
            self.close()
            raise

    def read(self):
        lines = []
        # A continuous noise source cannot monopolize the ROS executor.
        for _ in range(8):
            try:
                data = os.read(self.fd, 1024)
            except BlockingIOError:
                break
            if not data:
                break
            self.rx_bytes += len(data)
            for byte in data:
                if byte == 10:
                    self.rx_lines += 1
                    if not self.discard:
                        lines.append(bytes(self.buffer) + b'\n')
                    self.buffer.clear()
                    self.discard = False
                elif not self.discard:
                    self.buffer.append(byte)
                    if len(self.buffer) > 255:
                        self.discarded_frames += 1
                        self.buffer.clear()
                        self.discard = True
        return lines

    def write(self, data):
        # A short write must never be followed by another command frame.
        if os.write(self.fd, data) != len(data):
            raise OSError(errno.EIO, 'partial UART frame write; restart bridge')

    def close(self):
        if self.fd >= 0:
            try:
                fcntl.ioctl(self.fd, termios.TIOCNXCL)
            except OSError:
                pass  # USB may already be unplugged.
            finally:
                os.close(self.fd)
                self.fd = -1


class Esp32Bridge(Node):
    def __init__(self):
        super().__init__('esp32_bridge')
        defaults = dict(port='/dev/serial/by-id/SET_ESP32_PORT', baud=230400, serial_protocol='simple',
                        wheel_radius=.05, kinematic_sum=.260145, encoder_cpr=937.2,
                        max_wheel_speed=2., state_timeout=.25, command_timeout=.3,
                        ack_timeout=.25, lift_steps_per_m=0., lift_max_height=0.,
                        lift_tolerance=.002, publish_odom_tf=True,
                        lift_preset_mode=False, lift_raise_steps=2000, lift_max_steps=2000)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = {key: self.get_parameter(key).value for key in defaults}
        if self.p['serial_protocol'] not in ('simple', 'v2'):
            raise ValueError('serial_protocol must be simple or v2')
        self.simple = self.p['serial_protocol'] == 'simple'
        self.sent_after = {}
        self.simple_rx_seq = self.simple_boot = 0
        positive = ('wheel_radius', 'kinematic_sum', 'encoder_cpr', 'max_wheel_speed',
                    'state_timeout', 'command_timeout', 'ack_timeout', 'lift_tolerance')
        if any(not math.isfinite(self.p[k]) or self.p[k] <= 0 for k in positive):
            raise ValueError('Geometry, timing and speed parameters must be finite and positive')
        if self.p['max_wheel_speed'] > 12 or self.p['state_timeout'] > .3 or self.p['ack_timeout'] > .3:
            raise ValueError('Wheel limit <=12 rad/s; feedback/ACK limits <=0.3 s')
        if self.p['baud'] not in (115200, 230400):
            raise ValueError('Unsupported baud; firmware default is 230400')
        if any(not math.isfinite(self.p[k]) or self.p[k] < 0 for k in ('lift_steps_per_m', 'lift_max_height')):
            raise ValueError('Lift calibration must be nonnegative and finite')
        if self.p['lift_preset_mode']:
            if (not 0 < self.p['lift_raise_steps'] <= self.p['lift_max_steps'] or
                    self.p['lift_steps_per_m'] <= 0 or
                    not math.isclose(self.p['lift_max_height'],
                                     self.p['lift_max_steps']/self.p['lift_steps_per_m'])):
                raise ValueError('Pulse preset needs positive counts and matching UART coordinate scale/limit')
        self.state = None
        self.received_at = self.cmd_at = -math.inf
        self.seq = 0
        self.handshake_seq = None
        self.handshake = False
        self.armed = self.arming = False
        self.arm_seq = self.reset_seq = None
        self.stop_seq = None
        self.hold_after_stop_seq = None
        self.arm_at = 0.
        self.host_fault = ''
        self.transport_broken = False
        self.invalid_frames = 0
        self.command = (0., 0., 0.)
        self.await_command_zero = True
        self.wheel_velocity = (0.,)*4
        self.height = self.lift_velocity = 0.
        self.hold_height = 0.
        self.pose = [0., 0., 0.]
        self.op = None
        self.reserved = False
        self.stop_generation = 0
        self.goal_generation = 0
        self.software_estop = False
        self.last_status = ''
        self.last_diag = 0.
        self.started_at = time.monotonic()
        self.last_rx_warning = -math.inf
        self.pending_tx = []
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_pub = self.create_publisher(TFMessage, '/tf', 10)
        self.lift_pub = self.create_publisher(LiftState, '/lift/state', latched)
        self.busy_pub = self.create_publisher(Bool, '/lift/moving', latched)
        self.status_pub = self.create_publisher(String, '/esp32/status', latched)
        self.preset_pub = self.create_publisher(Float64, '/lift/preset_target', latched)
        if self.p['lift_preset_mode']:
            self.preset_pub.publish(Float64(data=self.p['lift_raise_steps']/self.p['lift_steps_per_m']))
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.create_subscription(Twist, '/cmd_vel', self.receive_command, 10)
        self.create_subscription(Bool, '/safety/estop', self.estop, latched)
        self.create_service(SetBool, '/esp32/arm', self.arm)
        self.create_service(Trigger, '/esp32/reset', self.reset)
        self.create_service(Trigger, '/lift/reset', self.reset)
        self.create_service(Trigger, '/lift/stop', self.stop)
        self.home_server = ActionServer(self, HomeLift, '/lift/home',
            execute_callback=self.execute_home, goal_callback=lambda g: self.accept('home', g),
            cancel_callback=lambda h: CancelResponse.ACCEPT)
        self.move_server = ActionServer(self, MoveLift, '/lift/move',
            execute_callback=self.execute_move, goal_callback=lambda g: self.accept('move', g),
            cancel_callback=lambda h: CancelResponse.ACCEPT)
        # Open last, after ROS construction and parameter validation.
        self.serial = SerialPort(self.p['port'], self.p['baud'])
        self.create_timer(.02, self.tick, clock=Clock())
        self.get_logger().info(f'UART {self.p["serial_protocol"]}: {self.p["port"]}, baud={self.p["baud"]}, '
                               f'DTR/RTS={self.serial.modem_lines}; DISARMED, waiting for telemetry')

    def fresh(self):
        return self.state is not None and time.monotonic()-self.received_at < self.p['state_timeout']

    def lift_configured(self):
        return (self.state is not None and self.state.capabilities == 3 and
                self.p['lift_steps_per_m'] > 0 and self.p['lift_max_height'] > 0)

    def stationary(self):
        return self.fresh() and all(abs(v) < .15 for v in self.wheel_velocity)

    def ready(self):
        return (self.fresh() and not self.host_fault and not self.transport_broken and
                not self.software_estop and not self.state.fault and not self.state.estop and
                bool(self.state.capabilities & 1))

    def fail(self, reason):
        if not self.host_fault:
            self.get_logger().error(reason + '; no automatic wheel stop (manual-cutoff profile)')
            self.host_fault = reason
        self.armed = self.arming = False
        self.command = (0., 0., 0.)
        # Do not send D/C-disabled on a fault: user selected hold-PWM firmware.
        # Stop publishing fresh odometry if telemetry stops. Re-arm is explicit.
        if self.op is not None:
            self.finish(False, reason, send_stop=False)

    def send(self, kind, values=None):
        if self.state is None or self.transport_broken:
            return None
        self.seq += 1
        payload = f'{kind},2,{self.state.boot},{self.seq}'
        if values is not None:
            wheels, height, enabled = values
            payload += ''.join(f',{v:.9g}' for v in wheels) + f',{height:.9g},{int(enabled)}'
        try:
            if self.simple:
                payload = kind
                if values is not None:
                    wheels, height, enabled = values
                    steps = round(height*self.p['lift_steps_per_m'])
                    payload += ''.join(f',{v:.4f}' for v in wheels) + f',{steps},{int(enabled)}'
                self.serial.write((payload+'\n').encode('ascii'))
                self.sent_after[self.seq] = self.state.seq
                keep = {self.arm_seq, self.stop_seq, self.reset_seq, self.handshake_seq,
                        self.hold_after_stop_seq, self.op['seq'] if self.op else None}
                self.sent_after = {k: v for k, v in self.sent_after.items() if k in keep or k > self.seq-64}
            else:
                self.serial.write(frame(payload))
                self.pending_tx.append((self.seq, time.monotonic()))
            # Bound bookkeeping even with an unresponsive MCU.
            self.pending_tx = self.pending_tx[-64:]
            return self.seq
        except OSError as error:
            self.transport_broken = True
            self.fail(f'UART_WRITE: {error}')
            return None

    def confirmed(self, token, state=None):
        """CSV has no ACK: require later telemetry AND caller's mode/position check."""
        state = state or self.state
        if token is None or state is None:
            return False
        if self.simple:
            return state.seq >= self.sent_after.get(token, math.inf)+2
        return state.ack >= token

    def receive_command(self, msg):
        values = (msg.linear.x, msg.linear.y, msg.angular.z)
        extra = (msg.linear.z, msg.angular.x, msg.angular.y)
        if not all(math.isfinite(v) for v in values+extra) or any(extra):
            values = (0., 0., 0.)
        # Commands arriving during lift/arming cannot be replayed afterward.
        if not any(values):
            self.await_command_zero = False
        self.command = values if self.armed and not self.reserved and not self.await_command_zero else (0., 0., 0.)
        self.cmd_at = time.monotonic()

    def arm(self, req, res):
        if not req.data:
            self.explicit_stop(disarm=True)
            res.success = not self.transport_broken and self.state is not None
            res.message = 'Stop sent; verify fresh DISARMED/FAULTED telemetry'
        elif self.armed:
            res.success, res.message = True, 'Already ARMED'
        elif (not self.ready() or not self.handshake or not self.stationary() or
              self.state.mode != 1 or self.reserved or self.arming or
              (self.state.capabilities == 3 and (not self.lift_configured() or
               (self.state.homed and not 0 <= self.height <= self.p['lift_max_height'])))):
            res.success, res.message = False, 'Need fresh fault-free DISARMED telemetry and stationary wheels; inspect /esp32/status'
        else:
            self.command = (0., 0., 0.)
            self.hold_height = self.height if self.state.homed else 0.
            self.arm_seq = self.send('C', ((0.,)*4, self.hold_height, True))
            self.arming = self.arm_seq is not None
            self.arm_at = time.monotonic()
            res.success = self.arming
            res.message = 'Arm requested; wait for /esp32/status ARMED before keyboard input'
        return res

    def reset(self, req, res):
        if (not self.fresh() or self.transport_broken or self.armed or self.arming or
                self.reserved or not self.stationary() or self.state.estop or self.software_estop or
                self.state.mode not in (1, 4) or self.stop_seq is None or
                not self.confirmed(self.stop_seq)):
            res.success, res.message = False, 'Disarm first; wait for fresh stopped telemetry and healthy inputs'
        else:
            self.reset_seq = self.send('R')
            res.success = self.reset_seq is not None
            res.message = 'Reset sent; wait for fault-free DISARMED telemetry, then arm explicitly'
        return res

    def estop(self, msg):
        self.software_estop = msg.data
        if msg.data:
            self.explicit_stop(disarm=True)

    def explicit_stop(self, disarm=False):
        self.stop_generation += 1
        self.command = (0., 0., 0.)
        self.await_command_zero = True
        self.cmd_at = time.monotonic()
        self.hold_height = self.height if self.state is not None and self.state.homed else 0.
        self.stop_seq = self.send('D')
        self.hold_after_stop_seq = self.stop_seq
        if disarm:
            self.armed = self.arming = False
        if self.op is not None:
            self.finish(False, 'STOPPED', send_stop=False)
        # A host-only fault can clear after an explicit stop, never after a timeout alone.
        if self.fresh() and not self.state.fault and not self.transport_broken:
            self.host_fault = ''
            if disarm or not self.armed:
                self.handshake = False
                self.handshake_seq = self.stop_seq

    def stop(self, req, res):
        self.explicit_stop()
        res.success = self.state is not None and not self.transport_broken
        res.message = 'Stop sent; check fresh stopped telemetry in /esp32/status'
        return res

    def accept(self, kind, goal):
        pulse_move = self.p['lift_preset_mode'] and kind == 'move'
        timeout = goal.timeout if goal.timeout else (60. if kind == 'home' else 15.)
        valid = math.isfinite(timeout) and 0 < timeout <= 120.
        if kind == 'move':
            valid = valid and math.isfinite(goal.height) and 0 <= goal.height <= self.p['lift_max_height']
            if pulse_move:
                target = self.p['lift_raise_steps']/self.p['lift_steps_per_m']
                valid = valid and (goal.height == 0. or math.isclose(goal.height, target, abs_tol=1e-9))
                # A zero target means seek the physical bottom switch, not an
                # unverified step-coordinate zero. It can establish first home.
                if goal.height == 0.:
                    kind = 'home'
        if (not valid or not self.ready() or not self.armed or self.reserved or
                not self.lift_configured() or not self.stationary() or
                (kind == 'move' and not self.state.homed)):
            self.get_logger().warning(f'Rejected {kind}: arm, stop wheels and check lift calibration/capabilities')
            return GoalResponse.REJECT
        self.reserved = True
        self.goal_generation = self.stop_generation
        self.command = (0., 0., 0.)
        self.busy_pub.publish(Bool(data=True))
        return GoalResponse.ACCEPT

    async def execute(self, handle, kind, cls):
        if self.goal_generation != self.stop_generation or not self.ready() or not self.armed:
            self.reserved = False
            handle.abort()
            return cls.Result(success=False, code='STOPPED_BEFORE_START', height=self.height,
                              homed=self.state.homed if self.state else False)
        future = Future()
        timeout = handle.request.timeout or (60. if kind == 'home' else 15.)
        self.op = dict(handle=handle, kind=kind, cls=cls, future=future,
                       target=handle.request.height if kind == 'move' else 0.,
                       deadline=time.monotonic()+timeout, seq=None, phase='start', settled=0)
        return await future

    async def execute_home(self, handle):
        return await self.execute(handle, 'home', HomeLift)

    async def execute_move(self, handle):
        kind = 'home' if self.p['lift_preset_mode'] and handle.request.height == 0. else 'move'
        return await self.execute(handle, kind, MoveLift)

    def finish(self, success, code, send_stop=True):
        op, self.op = self.op, None
        if op is None:
            return
        if send_stop:
            self.hold_after_stop_seq = self.send('D')
        self.hold_height = self.height if self.state and self.state.homed else 0.
        self.command = (0., 0., 0.)
        self.await_command_zero = True
        self.cmd_at = time.monotonic()
        self.reserved = False
        handle = op['handle']
        if handle.is_cancel_requested:
            success, code = False, 'CANCELED'
            handle.canceled()
        elif success:
            handle.succeed()
        else:
            handle.abort()
        op['future'].set_result(op['cls'].Result(success=success, code=code, height=self.height,
                                                homed=self.state.homed if self.state else False))

    def feedback(self, state):
        old = self.state
        if old is not None and state.boot == old.boot:
            if state.seq <= old.seq or state.us <= old.us or (not self.simple and (state.ack < old.ack or state.ack > self.seq)):
                self.invalid_frames += 1
                return
        elif old is not None:
            self.fail('MCU_REBOOT; explicit stop and arm required')
            self.handshake = False
            self.handshake_seq = None
            self.pending_tx.clear()
            self.sent_after.clear()
        first = old is None or old.boot != state.boot
        if first:
            self.seq = state.ack
            self.wheel_velocity = (0.,)*4
        self.state, self.received_at = state, time.monotonic()
        self.pending_tx = [(seq, at) for seq, at in self.pending_tx if seq > state.ack]
        if state.fault or state.estop:
            self.fail(f'MCU_FAULT_{state.fault}' if state.fault else 'MCU_ESTOP')
        if self.reset_seq is not None and self.confirmed(self.reset_seq, state) and not state.fault and state.mode == 1:
            self.reset_seq = None
            self.host_fault = ''
            self.handshake = True
        if self.handshake_seq is not None and self.confirmed(self.handshake_seq, state) and state.mode == 1:
            self.handshake = True
        if self.arming and self.confirmed(self.arm_seq, state) and state.mode == 2 and self.ready():
            self.arming, self.armed = False, True
            self.command = (0., 0., 0.)
            self.cmd_at = time.monotonic()
        self.height = state.steps/self.p['lift_steps_per_m'] if self.lift_configured() else 0.
        if (self.hold_after_stop_seq is not None and self.confirmed(self.hold_after_stop_seq, state)
                and state.mode in (1, 4)):
            # Capture position after fresh stopped telemetry, not from lagging
            # pre-stop telemetry. Never resume toward the old cached position.
            self.hold_height = self.height if state.homed else 0.
            self.hold_after_stop_seq = None
        self.lift_velocity = 0.
        dt = 0. if first else (state.us-old.us)/1e6
        delta = tuple(0. if first else (n-p)*math.tau/self.p['encoder_cpr'] for n, p in zip(state.ticks, state.ticks if first else old.ticks))
        if dt:
            self.wheel_velocity = tuple(d/dt for d in delta)
            # A home latch resets step position; it must not appear as physical velocity.
            if self.lift_configured() and old.homed and state.homed:
                self.lift_velocity = (state.steps-old.steps)/self.p['lift_steps_per_m']/dt
        dx, dy, da = forward(delta, self.p['wheel_radius'], self.p['kinematic_sum'])
        theta = self.pose[2] + da/2
        self.pose[0] += math.cos(theta)*dx-math.sin(theta)*dy
        self.pose[1] += math.sin(theta)*dx+math.cos(theta)*dy
        self.pose[2] = math.remainder(self.pose[2]+da, math.tau)
        stamp = self.get_clock().now().to_msg()
        positions = [t*math.tau/self.p['encoder_cpr'] for t in state.ticks] + [self.height]
        self.joint_pub.publish(JointState(header=Header(stamp=stamp), name=NAMES,
                                         position=positions, velocity=list(self.wheel_velocity)+[self.lift_velocity]))
        odom = Odometry(header=Header(stamp=stamp, frame_id='odom'), child_frame_id='base_link')
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.pose[:2]
        odom.pose.pose.orientation.z = math.sin(self.pose[2]/2)
        odom.pose.pose.orientation.w = math.cos(self.pose[2]/2)
        vx, vy, wz = forward(self.wheel_velocity, self.p['wheel_radius'], self.p['kinematic_sum'])
        odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.angular.z = vx, vy, wz
        for i, v in enumerate((.02, .02, 1000., 1000., 1000., .04)):
            odom.pose.covariance[7*i] = odom.twist.covariance[7*i] = v
        self.odom_pub.publish(odom)
        if self.p['publish_odom_tf']:
            tf = TransformStamped(header=odom.header, child_frame_id='base_link')
            tf.transform.translation.x, tf.transform.translation.y = self.pose[:2]
            tf.transform.rotation = odom.pose.pose.orientation
            self.tf_pub.publish(TFMessage(transforms=[tf]))
        if self.op is not None:
            self.advance_operation()

    def advance_operation(self):
        op, s = self.op, self.state
        if op['kind'] == 'home':
            if op['phase'] == 'disabled' and self.confirmed(op['seq'], s) and s.mode == 1 and self.stationary():
                op['seq'] = self.send('H')
                op['phase'] = 'homing'
            elif op['phase'] == 'homing' and op['seq'] is not None and self.confirmed(op['seq'], s) and s.homed and s.low and s.mode == 1:
                self.finish(True, 'HOMED')
        elif op['seq'] is not None and self.confirmed(op['seq'], s):
            if self.p['lift_preset_mode']:
                settled = s.steps == self.p['lift_raise_steps'] and self.lift_velocity == 0.
            else:
                settled = abs(self.height-op['target']) <= self.p['lift_tolerance'] and abs(self.lift_velocity) < .002
            op['settled'] = op['settled']+1 if settled else 0
            if op['settled'] >= 3:
                self.finish(True, 'REACHED')

    def publish_status(self, now):
        s = self.state
        stopping = self.hold_after_stop_seq is not None
        blocked = not self.ready() or not self.armed or self.reserved or stopping
        if self.lift_configured():
            blocked = blocked or not s.homed
        elif s is not None and s.capabilities == 3:
            # Never treat a fitted but uncalibrated lift as base-only.
            blocked = True
        fault = self.host_fault or ('NO_TELEMETRY' if not self.fresh() else '')
        if s is not None and s.capabilities == 3 and not self.lift_configured():
            fault = fault or 'HOST_LIFT_CALIBRATION_REQUIRED'
        header = Header(stamp=self.get_clock().now().to_msg(), frame_id='base_link')
        self.busy_pub.publish(Bool(data=bool(blocked)))
        self.lift_pub.publish(LiftState(header=header, state=MODES[s.mode] if s else 'DISCONNECTED',
            fault_code=fault, homed=s.homed if s else False, moving=self.reserved or stopping,
            inhibited=bool(blocked), estopped=self.software_estop or bool(s and s.estop),
            height=self.height, target=self.op['target'] if self.op else self.hold_height,
            lower_active=bool(s and s.low), upper_active=bool(s and s.high)))
        label = 'ARMED' if self.armed else ('ARMING' if self.arming else 'DISARMED')
        status = (f'{label}; host={fault or "OK"}; mcu={MODES[s.mode] if s else "UNKNOWN"}; '
                  f'fault={s.fault if s else "?"}; capabilities={s.capabilities if s else "?"}; '
                  f'handshake={self.handshake}; ack={s.ack if s else "?"}/{self.seq}; '
                  f'lift_mode={"PULSE_PRESET" if self.p["lift_preset_mode"] else "POSITION"}; '
                  f'lift_calibrated={self.lift_configured() and not self.p["lift_preset_mode"]}; '
                  f'boot={s.boot if s else "?"}')
        if self.simple:
            status = (f'{label}; host={fault or "OK"}; mcu={MODES[s.mode] if s else "UNKNOWN"}; '
                      f'fault={s.fault if s else "?"}; homed={int(s.homed) if s else "?"}; '
                      f'bottom={int(s.low) if s else "?"}; steps={s.steps if s else "?"}; '
                      f'rx_bytes={self.serial.rx_bytes}')
        self.status_pub.publish(String(data=status))
        # Log state transitions rather than every telemetry packet.
        signature = (label, fault, s.mode if s else None, s.capabilities if s else None, self.handshake)
        if signature != getattr(self, 'last_signature', None):
            self.get_logger().info(status)
        self.last_status, self.last_signature = status, signature
        if now-self.last_diag >= 1.:
            self.last_diag = now
            self.diag_pub.publish(DiagnosticArray(header=header, status=[DiagnosticStatus(
                name='forklift_hardware/esp32',
                hardware_id=self.p['port'] if self.simple else (str(s.boot) if s else 'UNKNOWN'),
                level=DiagnosticStatus.ERROR if fault else (DiagnosticStatus.OK if self.armed else DiagnosticStatus.WARN), message=status,
                values=[KeyValue(key='serial_protocol', value=self.p['serial_protocol']),
                        KeyValue(key='port', value=self.p['port']),
                        KeyValue(key='baud', value=str(self.p['baud'])),
                        KeyValue(key='rx_bytes', value=str(self.serial.rx_bytes)),
                        KeyValue(key='rx_lines', value=str(self.serial.rx_lines)),
                        KeyValue(key='discarded_frames', value=str(self.serial.discarded_frames)),
                        KeyValue(key='modem_lines', value=self.serial.modem_lines),
                        KeyValue(key='invalid_frames', value=str(self.invalid_frames)),
                        KeyValue(key='wheel_fault_policy', value='manual_power_cutoff')])]))

    def tick(self):
        now = time.monotonic()
        if not self.transport_broken:
            try:
                lines = self.serial.read()
            except OSError as error:
                self.transport_broken = True
                self.fail(f'UART_READ: {error}')
                lines = []
            for line in lines:
                try:
                    state = parse_simple_state(line) if self.simple else parse_state(line)
                except (ValueError, UnicodeError):
                    self.invalid_frames += 1
                    continue
                if self.simple:
                    self.simple_rx_seq += 1
                    if self.state is not None and state.us < self.state.us:
                        self.simple_boot += 1
                    state = State(self.simple_boot, self.simple_rx_seq, state.us, 0,
                                  state.ticks, state.steps, state.fault, state.mode,
                                  state.homed, state.low, state.high, state.estop, state.capabilities)
                self.feedback(state)
        if (self.armed or self.arming or self.reserved) and not self.fresh():
            self.fail('TELEMETRY_TIMEOUT')
        if self.pending_tx and now-self.pending_tx[0][1] > self.p['ack_timeout']:
            self.fail('ACK_TIMEOUT')
        if self.armed and not self.reserved and now-self.cmd_at > self.p['command_timeout']:
            self.fail('ROS_COMMAND_TIMEOUT')
        if self.arming and now-self.arm_at > 1.:
            self.fail('ARM_TIMEOUT')
        if self.state is None and now-self.started_at >= 5. and now-self.last_rx_warning >= 10.:
            self.last_rx_warning = now
            self.get_logger().warning(
                f'No valid ESP32 telemetry on {self.p["port"]} at {self.p["baud"]}: '
                f'rx_bytes={self.serial.rx_bytes}, rx_lines={self.serial.rx_lines}, '
                f'invalid_frames={self.invalid_frames}, discarded_frames={self.serial.discarded_frames}. '
                'Check /diagnostics; zero RX means port/boot/USB, nonzero RX needs frame validation.')
        if self.op is not None:
            op = self.op
            if op['handle'].is_cancel_requested or now >= op['deadline']:
                self.finish(False, 'CANCELED' if op['handle'].is_cancel_requested else 'LIFT_TIMEOUT')
            else:
                cls = op['cls']
                values = dict(state=MODES[self.state.mode], height=self.height,
                              lower_active=self.state.low, upper_active=self.state.high)
                if cls == MoveLift:
                    values['target'] = op['target']
                op['handle'].publish_feedback(cls.Feedback(**values))
        if self.ready() and self.hold_after_stop_seq is None:
            if self.op is not None and self.op['kind'] == 'home':
                seq = self.send('C', ((0.,)*4, 0., False))
                if self.op['phase'] == 'start' and seq is not None:
                    self.op.update(phase='disabled', seq=seq)
            elif self.armed or self.arming:
                wheels = inverse(*self.command, self.p['wheel_radius'], self.p['kinematic_sum'])
                if not all(math.isfinite(w) for w in wheels):
                    wheels = (0.,)*4
                if self.reserved or self.arming:
                    wheels = (0.,)*4
                scale = min(1., self.p['max_wheel_speed']/max(max(abs(w) for w in wheels), 1e-9))
                wheels = tuple(w*scale for w in wheels)
                target = self.op['target'] if self.op else self.hold_height
                seq = self.send('C', (wheels, target, True))
                if self.op is not None and self.op['seq'] is None:
                    self.op['seq'] = seq
            elif self.state.mode == 1:
                seq = self.send('C', ((0.,)*4, 0., False))
                if self.handshake_seq is None:
                    self.handshake_seq = seq
        self.publish_status(now)

    def close(self, stop=True):
        # Normal shutdown is an explicit operator stop, unlike communication loss.
        if hasattr(self, 'serial'):
            if stop:
                self.explicit_stop(disarm=True)
            self.serial.close()


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = None
    operator_stop = False
    try:
        node = Esp32Bridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        operator_stop = True
    finally:
        if node is not None:
            node.close(stop=operator_stop)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
