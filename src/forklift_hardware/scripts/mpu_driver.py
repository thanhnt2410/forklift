#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""MPU6050/MPU6500 qua I2C trên Raspberry Pi -> /imu/data_raw (khung imu_link).

Cùng giao diện với imu_adapter.py của SIM: chỉ gyro + gia tốc, KHÔNG có
orientation (orientation_covariance[0] = -1). EKF xe thật chỉ dùng vyaw.

Chiều lắp: tham số `axes` ánh xạ trục chip sang trục imu_link (thẳng hàng
base_link, URDF imu_rpy = 0), ví dụ "-y x z" nghĩa là imu_link.x = -chip.y.
Dùng imu_mount_check.py để tìm chuỗi này trên xe. Bias gyro hiệu chuẩn lúc
khởi động khi xe đứng yên; chưa bù trôi bias theo nhiệt độ.

yaw_from_gravity (mặc định true, 08/11/2026): chip lắp nghiêng thì gyro z chỉ đo
cos(góc nghiêng) tốc độ quay thật. Đo trên xe: trục z lệch 55,8° so với phương thẳng
đứng, gyro z chỉ bằng ~0,51 tốc độ quay thật (0,269 rad so với 0,507 rad), nên EKF
đo thiếu góc quay và AMCL phải kéo lại ~0,2 rad sau mỗi lần quay. Giải pháp: lấy
hướng "lên" từ gia tốc lúc hiệu chuẩn (xe đứng yên) rồi publish angular_velocity.z =
ω · hướng_lên, là tốc độ quay quanh phương THẲNG ĐỨNG, đúng với mọi kiểu lắp chip.
Hai thành phần x, y đặt 0 vì EKF chỉ dùng vyaw. Chưa kiểm chứng trên xe sau khi sửa.
"""
import math
import statistics
import struct
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Header

try:
    from smbus2 import SMBus
except ImportError:  # python3-smbus của apt có cùng API cho các lệnh dùng ở đây
    from smbus import SMBus

from mpu_registers import (ACCEL_CONFIG, ACCEL_LSB, ACCEL_XOUT_H, CONFIG, G, GYRO_CONFIG, GYRO_LSB,
                           KNOWN_IDS, PWR_MGMT_1, SMPLRT_DIV, WHO_AM_I, parse_axes, remap)


class MpuDriver(Node):
    def __init__(self):
        super().__init__('mpu_driver')
        defaults = dict(bus=1, address=0x68, rate_hz=50., dlpf=3, axes='x y z',
                        calib_seconds=2., max_still_gyro_std=.02, min_gyro_variance=.003**2,
                        accel_variance=.05**2, max_read_errors=10, yaw_from_gravity=True)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = {key: self.get_parameter(key).value for key in defaults}
        self.axes = parse_axes(str(self.p['axes']))
        if not 10. <= float(self.p['rate_hz']) <= 200. or not 0 <= int(self.p['dlpf']) <= 6:
            raise ValueError('rate_hz must be 10..200 Hz and dlpf 0..6')
        self.bus = SMBus(int(self.p['bus']))
        self.address = int(self.p['address'])
        self.configure()
        self.pub = self.create_publisher(Imu, '/imu/data_raw', qos_profile_sensor_data)
        self.bias = None
        self.up = None  # hướng "lên" (vector đơn vị trong imu_link), đặt sau hiệu chuẩn
        self.calib = []
        self.calib_start = time.monotonic()
        self.errors = 0
        self.timer = self.create_timer(1./float(self.p['rate_hz']), self.tick)

    def configure(self):
        who = self.bus.read_byte_data(self.address, WHO_AM_I)
        if who not in KNOWN_IDS:
            raise RuntimeError(f'Unknown IMU WHO_AM_I=0x{who:02x} at 0x{self.address:02x}')
        # Clock PLL theo trục X gyro (ổn định hơn dao động nội), thoát sleep.
        self.bus.write_byte_data(self.address, PWR_MGMT_1, 0x01)
        time.sleep(.1)
        # DLPF 3 ~ 44 Hz (MPU6050) / 41 Hz (MPU6500): đủ cho EKF 30 Hz, lọc rung motor.
        self.bus.write_byte_data(self.address, CONFIG, int(self.p['dlpf']))
        # Có DLPF thì tốc độ lấy mẫu gốc 1 kHz; chia xuống đúng rate_hz.
        self.bus.write_byte_data(self.address, SMPLRT_DIV, max(0, min(255, round(1000./float(self.p['rate_hz']))-1)))
        self.bus.write_byte_data(self.address, GYRO_CONFIG, 0x00)   # ±250 °/s
        self.bus.write_byte_data(self.address, ACCEL_CONFIG, 0x00)  # ±2 g
        self.get_logger().info(f'{KNOWN_IDS[who]} (WHO_AM_I=0x{who:02x}) at bus {self.p["bus"]} '
                               f'addr 0x{self.address:02x}; axes="{self.p["axes"]}"; calibrating gyro bias, keep robot still')

    def read(self):
        d = bytes(self.bus.read_i2c_block_data(self.address, ACCEL_XOUT_H, 14))
        ax, ay, az, _temp, gx, gy, gz = struct.unpack('>hhhhhhh', d)
        accel = [v/ACCEL_LSB*G for v in (ax, ay, az)]
        gyro = [math.radians(v/GYRO_LSB) for v in (gx, gy, gz)]
        return accel, gyro

    def tick(self):
        try:
            accel, gyro = self.read()
            self.errors = 0
        except OSError as error:
            # Lỗi I2C lẻ (nhiễu dây dài) bỏ qua; liên tiếp thì ngừng phát để EKF
            # hết hạn sensor_timeout thay vì nhận số cũ.
            self.errors += 1
            if self.errors == int(self.p['max_read_errors']):
                self.get_logger().error(f'IMU I2C read failing ({error}); /imu/data_raw stopped')
            return
        if self.bias is None:
            self.calibrate(accel, gyro)
            return
        msg = Imu(header=Header(stamp=self.get_clock().now().to_msg(), frame_id='imu_link'))
        w = remap([g-b for g, b in zip(gyro, self.bias)], self.axes)
        a = remap(accel, self.axes)
        if self.up is not None:
            # Tốc độ quay quanh phương thẳng đứng (dương = ngược chiều kim đồng hồ nhìn từ trên).
            w = [0., 0., sum(wi*ui for wi, ui in zip(w, self.up))]
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = w
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = a
        msg.orientation.w = 1.
        msg.orientation_covariance[0] = -1.
        gv, av = self.gyro_variance, float(self.p['accel_variance'])
        msg.angular_velocity_covariance = [gv, 0., 0., 0., gv, 0., 0., 0., gv]
        msg.linear_acceleration_covariance = [av, 0., 0., 0., av, 0., 0., 0., av]
        self.pub.publish(msg)

    def calibrate(self, accel, gyro):
        self.calib.append((accel, gyro))
        if time.monotonic()-self.calib_start < float(self.p['calib_seconds']):
            return
        gyros = list(zip(*[g for _, g in self.calib]))
        std = max(statistics.pstdev(axis) for axis in gyros)
        if std > float(self.p['max_still_gyro_std']):
            # Xe đang bị đẩy/rung: bias sai sẽ thành góc trôi liên tục. Đo lại.
            self.get_logger().warning(f'Robot moving during gyro calibration (std {std:.3f} rad/s); retrying')
            self.calib = []
            self.calib_start = time.monotonic()
            return
        self.bias = [statistics.fmean(axis) for axis in gyros]
        # Phương sai = nhiễu đo thật, có sàn tối thiểu để EKF không quá tin gyro.
        self.gyro_variance = max(std**2, float(self.p['min_gyro_variance']))
        a = remap([statistics.fmean(v) for v in zip(*[a for a, _ in self.calib])], self.axes)
        self.get_logger().info('Gyro bias [%.4f %.4f %.4f] rad/s, noise std %.4f; gravity in imu_link [%.2f %.2f %.2f] m/s^2'
                               % (*self.bias, std, *a))
        magnitude = math.sqrt(sum(v*v for v in a))
        # |a| lệch xa g nghĩa là xe đang bị đẩy/rung lúc hiệu chuẩn: không tin hướng "lên" đo được.
        gravity_ok = abs(magnitude-G) <= 1.0
        if bool(self.p['yaw_from_gravity']) and gravity_ok:
            # Gia tốc đo lúc đứng yên chính là phản lực hướng LÊN (+g dọc phương thẳng đứng).
            self.up = [v/magnitude for v in a]
            tilt = math.degrees(math.acos(max(-1., min(1., self.up[2]))))
            if tilt > 5.:
                self.get_logger().warning(
                    'IMU z axis tilted %.1f deg from vertical; publishing angular_velocity.z = gyro . up '
                    'with up = [%.3f %.3f %.3f] (yaw_from_gravity)' % (tilt, *self.up))
        elif abs(a[2]-G) > 1.5:
            self.get_logger().error('Gravity is not on +z of imu_link: wrong "axes" mapping or tilted mount; '
                                    'yaw rate will be wrong. Run imu_mount_check.py')
        self.calib = []


def main():
    rclpy.init()
    node = MpuDriver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.bus.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
