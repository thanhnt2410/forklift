#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Gazebo six-axis interface for MPU6050 integration; never expose ideal yaw.

Noise/covariances are simulation assumptions, not measured sensor calibration.
Real MPU6050 transport and calibration remain separate future work.
"""
import copy
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_srvs.srv import SetBool


def raw_measurement(msg, gyro_variance, accel_variance):
    values = [getattr(v, a) for v in [msg.angular_velocity, msg.linear_acceleration]
              for a in ['x', 'y', 'z']]
    if msg.header.frame_id != 'imu_link' or not all(math.isfinite(v) for v in values):
        return None
    raw = copy.deepcopy(msg)
    raw.orientation.x = raw.orientation.y = raw.orientation.z = 0.
    raw.orientation.w = 1.
    raw.orientation_covariance = [-1., 0., 0., 0., 0., 0., 0., 0., 0.]
    raw.angular_velocity_covariance = [gyro_variance, 0., 0., 0., gyro_variance, 0., 0., 0., gyro_variance]
    raw.linear_acceleration_covariance = [accel_variance, 0., 0., 0., accel_variance, 0., 0., 0., accel_variance]
    return raw


class ImuAdapter(Node):
    def __init__(self):
        super().__init__('imu_adapter')
        self.declare_parameter('gyro_variance', .005 ** 2)
        self.declare_parameter('accel_variance', .03 ** 2)
        self.gyro = float(self.get_parameter('gyro_variance').value)
        self.accel = float(self.get_parameter('accel_variance').value)
        if not all(math.isfinite(v) and v > 0 for v in [self.gyro, self.accel]):
            raise ValueError('IMU variances must be positive and finite')
        self.drop = False
        self.pub = self.create_publisher(Imu, '/imu/data_raw', qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu/sim', self.receive, qos_profile_sensor_data)
        self.create_service(SetBool, '~/drop', self.set_drop)

    def set_drop(self, req, res):
        self.drop = req.data
        res.success = True
        res.message = 'SIM IMU suppressed' if self.drop else 'SIM IMU restored'
        return res

    def receive(self, msg):
        raw = raw_measurement(msg, self.gyro, self.accel)
        if not self.drop and raw is not None:
            self.pub.publish(raw)


def main():
    rclpy.init(); node = ImuAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__': main()
