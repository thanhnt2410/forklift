#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Gate autonomous velocity with live scan/odom/map TF and valid localization.

Feedback loss while a Nav2 motion action runs (or a non-zero command was just
issued) latches until explicitly reset. While idle it only reports WAITING, so
re-setting the AMCL pose does not latch. Runs on wall time so a paused
simulator cannot keep an old velocity enabled indefinitely.
"""
import math
import time
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from action_msgs.msg import GoalStatus, GoalStatusArray

# Action Nav2 có thể làm xe chạy; goal của chúng còn sống = đang chuyển động tự động.
MOTION_ACTIONS = ('follow_path', 'navigate_to_pose', 'spin', 'backup', 'drive_on_heading')
LIVE_GOAL = (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING, GoalStatus.STATUS_CANCELING)


class NavigationGuard(Node):
    def __init__(self):
        super().__init__('navigation_guard')
        self.stamps = {}
        self.good_pose = False
        self.command = Twist()
        self.command_stamp = 0.
        self.ever_ready = False
        self.fault = False
        self.declare_parameter('require_imu', False)
        self.require_imu = bool(self.get_parameter('require_imu').value)
        # TF map->odom chỉ được AMCL phát lại mỗi scan xử lý được. Xe thật: LiDAR
        # 6 Hz + odom TF qua Wi-Fi trễ tới 0,5 s làm AMCL bỏ scan ("earlier than all
        # the data in the transform cache"); 29/10/2026 đo khoảng trống tới 1,46 s,
        # ready tụt mỗi 7-12 s -> FEEDBACK_OR_NAVIGATION_FAULT. scan/odom giữ 0,7 s.
        self.declare_parameter('map_tf_timeout', .7)
        self.map_tf_timeout = float(self.get_parameter('map_tf_timeout').value)
        if not .7 <= self.map_tf_timeout <= 3.:
            raise ValueError('map_tf_timeout must be within [0.7, 3.0] s')
        self.pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.ready_pub = self.create_publisher(Bool, '/navigation/ready', 10)
        self.status_pub = self.create_publisher(String, '/navigation/status', 10)
        self.create_subscription(Twist, '/nav/cmd_vel_smoothed', self.receive, 10)
        self.create_subscription(LaserScan, '/scan', self.scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.odom, 10)
        if self.require_imu:
            self.create_subscription(Imu, '/imu/data_raw', self.imu, qos_profile_sensor_data)
            self.create_subscription(Odometry, '/wheel/odometry', self.wheel, 10)
        # AMCL publishes only on filter updates and retains its latest pose.
        # A volatile subscriber started at rest can miss the initial pose forever.
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(TFMessage, '/tf', self.tf, 100)
        self.motion_goals = {}
        self.moving_stamp = 0.
        status_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for name in MOTION_ACTIONS:
            self.create_subscription(GoalStatusArray, '/'+name+'/_action/status',
                                     lambda msg, n=name: self.motion_goals.__setitem__(
                                         n, any(s.status in LIVE_GOAL for s in msg.status_list)), status_qos)
        self.create_service(Trigger, '~/reset', self.reset)
        # 20 Hz = nhịp smoother/controller; 50 Hz trước đây chỉ nhân bản lệnh cũ và
        # đẩy 100 tin/s /navigation/ready + /cmd_vel_nav qua Wi-Fi sang gate trên Pi.
        # Gate vẫn coi lệnh cũ sau 0,3 s và ready cũ sau 0,3 s.
        self.create_timer(.05, self.tick, clock=rclpy.clock.Clock())

    def receive(self, msg):
        values=(msg.linear.x,msg.linear.y,msg.angular.z)
        if all(math.isfinite(v) for v in values):
            self.command=msg;self.command_stamp=time.monotonic()
            if any(abs(v)>1e-3 for v in values):self.moving_stamp=self.command_stamp
        else:self.fault=True

    def motion_active(self):
        # Velocity smoother phát số 0 liên tục khi rảnh, nên không dùng "có tin
        # nhắn". Dùng goal action còn sống, hoặc lệnh khác 0 trong 1 s gần nhất.
        return any(self.motion_goals.values()) or time.monotonic()-self.moving_stamp<1.

    def scan(self, msg):
        if msg.header.frame_id=='laser_frame' and len(msg.ranges)>10 and any(math.isfinite(v) and v>msg.range_min for v in msg.ranges):
            self.stamps['scan']=time.monotonic()

    def odom(self, msg):
        if msg.header.frame_id=='odom' and msg.child_frame_id=='base_link':
            self.stamps['odom']=time.monotonic()

    def pose(self, msg):
        p=msg.pose
        values=[p.pose.position.x,p.pose.position.y,p.covariance[0],p.covariance[7],p.covariance[35]]
        # ROS covariance arrays may expose NumPy scalars. Keep a native bool
        # for std_msgs/Bool and service response fields.
        # AMCL can produce tiny negative variances from floating-point roundoff.
        self.good_pose=bool(all(math.isfinite(v) for v in values) and -1e-9<=p.covariance[0]<.25
                           and -1e-9<=p.covariance[7]<.25 and -1e-9<=p.covariance[35]<.5)

    def imu(self, msg):
        if (msg.header.frame_id=='imu_link' and math.isfinite(msg.angular_velocity.z)
                and math.isfinite(msg.angular_velocity_covariance[8])
                and msg.angular_velocity_covariance[8]>0):
            self.stamps['imu']=time.monotonic()

    def wheel(self, msg):
        if msg.header.frame_id=='odom' and msg.child_frame_id=='base_link':
            self.stamps['wheel']=time.monotonic()

    def tf(self, msg):
        for t in msg.transforms:
            if t.header.frame_id=='map' and t.child_frame_id=='odom':
                self.stamps['tf']=time.monotonic()

    def timeout(self, key):
        return self.map_tf_timeout if key=='tf' else .7

    def feedback_ready(self):
        now=time.monotonic()
        keys=['scan','odom','tf']+(['imu','wheel'] if self.require_imu else [])
        return self.good_pose and all(now-self.stamps.get(k,0.)<self.timeout(k) for k in keys)

    def reset(self, req, res):
        res.success=self.feedback_ready()
        if res.success:
            self.fault=False;self.command=Twist();self.command_stamp=0.
            self.get_logger().info('Navigation interlock reset; send a fresh goal')
        res.message='Reset; send a fresh goal/command' if res.success else 'Feedback not ready'
        return res

    def tick(self):
        ready=self.feedback_ready()
        # Chỉ khóa khi mất phản hồi trong lúc đang chạy tự động; lúc đứng yên
        # (vd. đặt lại 2D Pose Estimate) chỉ chờ, không cho lệnh nào đi qua.
        if self.ever_ready and not ready and self.motion_active():
            if not self.fault:
                stale=[k for k in ['scan','odom','tf']+(['imu','wheel'] if self.require_imu else [])
                       if time.monotonic()-self.stamps.get(k,0.)>=self.timeout(k)]
                self.get_logger().error(f'FAULT_LATCHED: stale={stale}, localization_valid={self.good_pose}; cancel goal and reset after feedback recovers')
            self.fault=True
        if ready:self.ever_ready=True
        allowed=ready and not self.fault
        self.pub.publish(self.command if allowed and time.monotonic()-self.command_stamp<.3 else Twist())
        self.ready_pub.publish(Bool(data=allowed))
        self.status_pub.publish(String(data='FAULT_LATCHED' if self.fault else ('READY' if ready else 'WAITING_FOR_LOCALIZATION')))


def main():
    rclpy.init();node=NavigationGuard()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if rclpy.ok():node.pub.publish(Twist());node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
