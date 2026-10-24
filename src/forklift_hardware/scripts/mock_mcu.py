#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Virtual MCU on a new PTY. No firmware, GPIO, USB or physical UART access."""
import json
import math
import os
from pathlib import Path
import pty
import time
import tty
import zlib

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger


def frame(payload):
    return f'{payload}*{zlib.crc32(payload.encode()):08x}\n'.encode()


class MockMCU(Node):
    def __init__(self):
        super().__init__('mock_mcu')
        self.declare_parameter('port_file', '')
        self.declare_parameter('encoder_cpr', 4096.)
        self.declare_parameter('lift_steps_per_m', 10000.)
        self.declare_parameter('wheel_radius', .05)
        self.declare_parameter('kinematic_sum', .2275)
        self.cpr = float(self.get_parameter('encoder_cpr').value)
        self.steps = float(self.get_parameter('lift_steps_per_m').value)
        self.radius = float(self.get_parameter('wheel_radius').value)
        self.kinematic_sum = float(self.get_parameter('kinematic_sum').value)
        self.port_file = Path(self.get_parameter('port_file').value)
        self.master, self.slave = pty.openpty()
        tty.setraw(self.slave)
        os.set_blocking(self.master, False)
        self.port_file.write_text(os.ttyname(self.slave)+'\n')
        self.connected = True
        self.drop = self.corrupt = self.freeze = False
        self.buffer = b''
        self.boot = 1
        self.seq = self.ack = self.bad = 0
        self.origin = self.last_tick = time.monotonic()
        self.last_cmd = self.origin
        self.next_state = self.origin
        self.next_status = self.origin
        self.speed = [0.]*4
        self.target = [0.]*4
        self.ticks = [0.]*4
        self.height = self.lift_target = 0.
        self.pose = [0., 0., 0.]
        self.enabled = False
        self.status = self.create_publisher(String, '/mock_mcu/status', 10)
        for name, attr in [('drop_telemetry', 'drop'), ('corrupt_telemetry', 'corrupt'),
                           ('freeze_encoders', 'freeze')]:
            self.create_service(SetBool, f'/mock_mcu/{name}',
                                lambda q, r, a=attr: self.set_fault(a, q, r))
        self.create_service(Trigger, '/mock_mcu/reset', self.reset)
        self.create_service(Trigger, '/mock_mcu/disconnect', self.disconnect)
        self.create_timer(.005, self.tick)
        self.get_logger().info(f'VIRTUAL MCU READY: {os.ttyname(self.slave)}')

    def set_fault(self, attr, request, response):
        setattr(self, attr, request.data)
        response.success = True
        response.message = f'{attr}={request.data}'
        return response

    def reset(self, request, response):
        self.boot += 1
        self.seq = self.ack = 0
        self.origin = time.monotonic()
        self.enabled = False
        self.target = [0.]*4
        self.speed = [0.]*4
        self.ticks = [0.]*4
        self.height = self.lift_target = 0.
        self.pose = [0., 0., 0.]
        response.success = True
        response.message = 'Virtual MCU reset; host must explicitly reactivate'
        return response

    def disconnect(self, request, response):
        self.connected = False
        os.close(self.master)
        os.close(self.slave)
        self.enabled = False
        self.target = [0.]*4
        response.success = True
        response.message = 'PTY closed; restart mock bringup to reconnect'
        return response

    def parse(self, line):
        try:
            if len(line) > 256:
                raise ValueError('length')
            payload, checksum = line.rsplit(b'*', 1)
            if len(checksum) != 8 or zlib.crc32(payload) != int(checksum, 16):
                raise ValueError('checksum')
            fields = payload.decode('ascii').split(',')
            if len(fields) != 9 or fields[:2] != ['C', '1']:
                raise ValueError('shape')
            seq = int(fields[2])
            if seq <= self.ack:
                raise ValueError('sequence')
            values = list(map(float, fields[3:8]))
            if not all(math.isfinite(v) for v in values) or any(abs(v) > 12.001 for v in values[:4]):
                raise ValueError('speed')
            if not 0 <= values[4] <= .25 or fields[8] not in ['0', '1']:
                raise ValueError('lift/enable')
            self.ack = seq
            self.last_cmd = time.monotonic()
            self.enabled = fields[8] == '1'
            self.target = values[:4] if self.enabled else [0.]*4
            self.lift_target = values[4] if self.enabled else self.height
        except (ValueError, UnicodeError):
            self.bad += 1

    def tick(self):
        now = time.monotonic()
        dt = min(now-self.last_tick, .05)
        self.last_tick = now
        if self.connected:
            try:
                self.buffer += os.read(self.master, 4096)
            except BlockingIOError:
                pass
            while b'\n' in self.buffer:
                line, self.buffer = self.buffer.split(b'\n', 1)
                self.parse(line)
            if len(self.buffer) > 1024:
                self.buffer = b''
                self.bad += 1
        watchdog = now-self.last_cmd > .3
        if watchdog or not self.connected:
            self.enabled = False
            self.target = [0.]*4
            self.lift_target = self.height
        # First-order motor plant, NOT a calibrated GA25 motor/PID model.
        gain = 1-math.exp(-dt/.08)
        for i in range(4):
            self.speed[i] += gain*(self.target[i]-self.speed[i])
            if not self.freeze:
                self.ticks[i] += self.speed[i]*dt/(2*math.pi)*self.cpr
        fl, fr, rl, rr = self.speed
        vx = self.radius*(fl+fr+rl+rr)/4
        vy = self.radius*(-fl+fr+rl-rr)/4
        wz = self.radius*(-fl+fr-rl+rr)/(4*self.kinematic_sum)
        theta = self.pose[2]
        self.pose[0] += (math.cos(theta)*vx-math.sin(theta)*vy)*dt
        self.pose[1] += (math.sin(theta)*vx+math.cos(theta)*vy)*dt
        self.pose[2] += wz*dt
        delta = max(-.04*dt, min(.04*dt, self.lift_target-self.height))
        self.height = max(0., min(.25, self.height+delta))
        if self.connected and now >= self.next_state:
            self.seq += 1
            payload = ','.join(map(str, ['S', 1, self.boot, self.seq,
                int((now-self.origin)*1.e6), self.ack,
                *[round(x) for x in self.ticks], round(self.height*self.steps), 0]))
            packet = frame(payload)
            if self.corrupt:
                packet = packet[:-9]+b'00000000\n'
            if not self.drop:
                try:
                    os.write(self.master, packet)
                except BlockingIOError:
                    pass
            self.next_state = now+.02
        if now >= self.next_status:
            self.status.publish(String(data=json.dumps(dict(boot=self.boot, ack=self.ack,
                enabled=self.enabled, watchdog=watchdog, actual_speed=self.speed,
                ticks=self.ticks, pose=self.pose, height=self.height, invalid_commands=self.bad,
                dropped=self.drop, corrupted=self.corrupt, connected=self.connected))))
            self.next_status = now+.05

    def close(self):
        if self.connected:
            os.close(self.master)
            os.close(self.slave)
        self.port_file.unlink(missing_ok=True)


def main():
    rclpy.init()
    node = MockMCU()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
