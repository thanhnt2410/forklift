# SPDX-License-Identifier: Apache-2.0
"""Manual ESP32 USB/UART bringup with optional automatic startup arm."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def setup(context):
    config = LaunchConfiguration('config').perform(context)
    port = LaunchConfiguration('port').perform(context)
    lidar_config = LaunchConfiguration('lidar_params_file').perform(context)
    lidar_port = yaml.safe_load(Path(lidar_config).read_text())['ydlidar_ros2_driver_node']['ros__parameters']['port']
    if not isinstance(lidar_port, str) or not Path(lidar_port).is_absolute():
        raise ValueError('LiDAR config must specify an absolute serial path so ESP32 auto can exclude it')
    # One file supplies geometry/limits to the bridge and the command gate.
    params = yaml.safe_load(Path(config).read_text())['esp32_bridge']['ros__parameters']
    # An explicit launch argument overrides the fixed physical port in YAML.
    if not port:
        port = params['port']
    # Hồ sơ bánh (wheels_real.yaml) đè hình học/giới hạn của esp32.yaml cho cả hai node.
    wheel_type = LaunchConfiguration('wheel_type').perform(context)
    wheel = yaml.safe_load((Path(share('forklift_hardware'))/'config/wheels_real.yaml').read_text())[wheel_type]
    wheel = dict(wheel)
    # Khóa riêng của gate, không đưa vào esp32_bridge.
    dock_limit = wheel.pop('dock_max_wheel_speed', 0.)
    params.update(wheel)
    gate = {key: params[key] for key in ('wheel_radius', 'kinematic_sum', 'max_wheel_speed')}
    gate['dock_max_wheel_speed'] = float(dock_limit)
    gate['holonomic'] = wheel['drive_type'] == 'mecanum'
    # Lớp an toàn LiDAR trong gate: footprint và vị trí LiDAR lấy từ robot.yaml như URDF.
    # LiDAR thật lắp xoay 90° (laser_yaw π/2, khớp navigation_real/mapping_real).
    geo = yaml.safe_load((Path(share('forklift_description'))/'config/robot.yaml').read_text())
    gate.update(scan_safety=ParameterValue(LaunchConfiguration('scan_safety'), value_type=bool),
                laser_x=float(geo['laser_xyz'][0]), laser_y=float(geo['laser_xyz'][1]),
                laser_yaw=1.5707963267948966,
                footprint_xmin=float(geo['footprint_xmin']), footprint_xmax=float(geo['footprint_xmax']),
                footprint_halfwidth=float(geo['footprint_halfwidth']),
                loaded_xmax=float(geo['loaded_xmax']), loaded_halfwidth=float(geo['loaded_halfwidth']))
    # Mặc định tự động (teleop vẫn ưu tiên): sau mỗi lần khởi động lại launch
    # không phải gọi set_autonomous thủ công thì mission mới chạy được bánh.
    gate.update(use_sim_time=False, require_navigation_ready=False,
                autonomous_enabled=ParameterValue(LaunchConfiguration('autonomous'), value_type=bool),
                enforce_transport_height=ParameterValue(
                    LaunchConfiguration('enforce_transport_height'), value_type=bool))
    def shutdown(reason):
        return EmitEvent(event=Shutdown(reason=reason))
    imu = LaunchConfiguration('imu').perform(context) == 'true'
    # Có gyro thì gate giữ hướng khi lệnh tịnh tiến không kèm quay (bánh phải hụt
    # tốc độ làm xe quay trái 30/10/2026). Không IMU: vòng hở như trước.
    gate['heading_hold'] = imu
    bridge = {'port': port, 'lidar_port': lidar_port, 'use_sim_time': False,
              'auto_arm': ParameterValue(LaunchConfiguration('auto_arm'), value_type=bool),
              # Hạ càng ngay khi khởi động: phiên trước lỗi giữa lúc đang chở thì càng vẫn
              # cao, task_manager recoverable() đòi < 0,02 m nên /mission/reset và Run bị từ chối.
              'auto_home': ParameterValue(LaunchConfiguration('auto_home'), value_type=bool),
              'auto_home_timeout': ParameterValue(LaunchConfiguration('auto_home_timeout'), value_type=float),
              **wheel}
    # Có IMU: EKF là nguồn DUY NHẤT phát odom -> base_link. Bridge chỉ phát odom
    # bánh sang /wheel/odometry, không phát TF (hai nguồn TF sẽ giật qua lại).
    if imu:
        bridge['publish_odom_tf'] = False
    nodes = [
        Node(package='forklift_hardware', executable='esp32_bridge.py',
             parameters=[config, bridge], remappings=[('/odom', '/wheel/odometry')] if imu else [],
             output='screen',
             on_exit=shutdown('ESP32 bridge exited')),
        Node(package='forklift_control', executable='command_gate.py',
             parameters=[str(Path(share('forklift_control'))/'config/command_gate.yaml'), gate],
             output='screen', on_exit=shutdown('Command gate exited')),
    ]
    if imu:
        hw = Path(share('forklift_hardware'))/'config'
        geometry = yaml.safe_load((Path(share('forklift_description'))/'config/robot.yaml').read_text())
        xyz, rpy = geometry['imu_xyz'], geometry['imu_rpy']
        nodes += [
            Node(package='forklift_hardware', executable='mpu_driver.py',
                 parameters=[str(hw/'mpu.yaml')], output='screen', on_exit=shutdown('MPU driver exited')),
            # EKF trên Pi cần base_link -> imu_link ngay cả khi laptop (robot_state_publisher)
            # chưa chạy. Cùng giá trị URDF nên trùng với RSP cũng không xung đột.
            Node(package='tf2_ros', executable='static_transform_publisher', name='imu_static_tf',
                 arguments=['--x', str(xyz[0]), '--y', str(xyz[1]), '--z', str(xyz[2]),
                            '--roll', str(rpy[0]), '--pitch', str(rpy[1]), '--yaw', str(rpy[2]),
                            '--frame-id', 'base_link', '--child-frame-id', 'imu_link']),
            Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
                 parameters=[str(hw/'ekf_real.yaml')], remappings=[('odometry/filtered', '/odom')],
                 output='screen', on_exit=shutdown('EKF exited')),
        ]
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='',
                              description='Default: port from ESP32 YAML; override with a serial path or auto'),
        DeclareLaunchArgument('config', default_value=str(Path(share('forklift_hardware'))/'config/esp32.yaml')),
        DeclareLaunchArgument('lidar_params_file',
                              default_value=str(Path(share('forklift_hardware'))/'config/ydlidar.yaml'),
                              description='Reserve this LiDAR serial port during ESP32 detection'),
        DeclareLaunchArgument('wheel_type', default_value='rubber', choices=['rubber', 'mecanum'],
                              description='rubber: V3 85 mm skid-steer (no strafe); mecanum: 100 mm omni'),
        DeclareLaunchArgument('auto_arm', default_value='true', choices=['true', 'false'],
                              description='Automatically arm once at startup after healthy telemetry and zero input'),
        DeclareLaunchArgument('auto_home', default_value='true', choices=['true', 'false'],
                              description='Lower the forks once at startup (before auto arm); false: keep them as found'),
        DeclareLaunchArgument('auto_home_timeout', default_value='45.0',
                              description='Deadline for the startup lowering, 30-120 s (firmware home timeout is 30 s)'),
        DeclareLaunchArgument('imu', default_value='true', choices=['true', 'false'],
                              description='MPU6050/6500 + EKF own odom->base_link; false: bridge wheel odom TF'),
        DeclareLaunchArgument('autonomous', default_value='true', choices=['true', 'false'],
                              description='Gate accepts nav/mission commands at startup (teleop keeps priority)'),
        # Dừng/giảm tốc theo LiDAR trong command gate; false = như trước 01/11/2026.
        DeclareLaunchArgument('scan_safety', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('enforce_transport_height', default_value='false', choices=['true', 'false'],
                              description='False: raised stationary forks warn without blocking driving'),
        OpaqueFunction(function=setup),
    ])
