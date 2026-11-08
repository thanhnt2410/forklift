# SPDX-License-Identifier: Apache-2.0
"""Mapping OR localization/navigation. Exactly one owner of map -> odom."""
from pathlib import Path
import tempfile
import os
import yaml
import copy
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    mode=LaunchConfiguration('mode').perform(context)
    common={'use_sim_time':LaunchConfiguration('use_sim_time').perform(context).lower()=='true'}
    autostart=LaunchConfiguration('autostart').perform(context).lower()=='true'
    use_ekf=LaunchConfiguration('use_ekf').perform(context).lower()=='true'
    p6=LaunchConfiguration('p6').perform(context)=='true'
    relay=Node(package='forklift_navigation',executable='scan_relay.py',parameters=[common,{'p6_load_filter':p6, 'geometry_file':str(Path(share('forklift_description'))/'config/robot.yaml')}],output='screen',
               on_exit=EmitEvent(event=Shutdown(reason='Scan relay exited')))
    if mode=='mapping':
        return [relay,Node(package='forklift_navigation',executable='probability_slam',name='slam_toolbox',
                     parameters=[str(Path(share('forklift_navigation'))/'config/slam.yaml'),common],output='screen',
                     on_exit=EmitEvent(event=Shutdown(reason='SLAM exited'))),
                IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
                    Path(share('forklift_navigation'))/'launch/mapping_services.launch.py')),
                    launch_arguments={'use_sim_time':str(common['use_sim_time']).lower(),
                                      'autostart':str(autostart).lower()}.items())]
    map_file=Path(LaunchConfiguration('map').perform(context))
    if not map_file.is_file():raise FileNotFoundError(f'Save a SLAM map before navigation: {map_file}')
    config=yaml.safe_load((Path(share('forklift_navigation'))/'config/nav2.yaml').read_text())
    # Xe thật: odom/TF từ Pi qua Wi-Fi có lúc trễ ~0,45 s; Spin với 0,1 s mất
    # footprint và coi là va chạm. Rỗng = giữ nav2.yaml (SIM).
    tolerance=LaunchConfiguration('behavior_transform_tolerance').perform(context)
    if tolerance:config['behavior_server']['ros__parameters']['transform_tolerance']=float(tolerance)
    # Xe thật: docking đi ngang tối thiểu 0,04 m/s; smoother kẹp 0,03 và
    # scale_velocities sẽ thu nhỏ cả vx/wz xuống dưới ngưỡng bánh lăn.
    behavior=config['behavior_server']['ros__parameters']
    for argument,key in (('spin_min_rotational_vel','min_rotational_vel'),('spin_max_rotational_vel','max_rotational_vel')):
        value=LaunchConfiguration(argument).perform(context)
        if value:behavior[key]=float(value)
    # Xe thật: gia tốc/giảm tốc smoother "ax,ay,aw". Firmware wheel_accel 6 rad/s²
    # (~1,15 rad/s² quay, 0,3 m/s² ngang) thì giới hạn SIM 0,5 rad/s² / 0,15 m/s² thành
    # chỗ nghẽn: xung chỉnh docking trôi lâu sau khi lệnh về 0. Rỗng = như SIM.
    accel=LaunchConfiguration('smoother_accel').perform(context)
    if accel:
        values=[float(v) for v in accel.split(',')]
        if len(values)!=3 or not all(0.<v<=3. for v in values):raise ValueError('smoother_accel needs "ax,ay,aw" in (0, 3]')
        smoother=config['velocity_smoother']['ros__parameters']
        smoother['max_accel']=values;smoother['max_decel']=[-v for v in values]
    lateral=LaunchConfiguration('smoother_max_lateral').perform(context)
    if lateral:
        smoother=config['velocity_smoother']['ros__parameters']
        smoother['max_velocity'][1]=float(lateral);smoother['min_velocity'][1]=-float(lateral)
    # Xe thật: AMCL của SIM (alpha 0,05, 60 tia, z_rand 0,5, sigma_hit 0,2) tin odom
    # gần như tuyệt đối và coi nhẹ scan. Bánh mecanum trượt: 29/10/2026 odom báo quay
    # 19° khi LiDAR thấy ~4°, AMCL bám theo odom với phương sai 0,002 (tự tin sai).
    # Hồ sơ này cho particle trải rộng theo sai số odom thật và để scan kéo về tường.
    # Chưa hiệu chuẩn bằng phép đo lặp; chỉnh tiếp theo T4 (relocalization).
    if LaunchConfiguration('amcl_profile').perform(context)=='real':
        config['amcl']['ros__parameters'].update(
            alpha1=.4,   # quay -> sai số quay: trượt lớn nhất khi quay tại chỗ
            alpha2=.2,   # tịnh tiến -> sai số quay
            alpha3=.2,   # tịnh tiến -> sai số tịnh tiến (đo: odom 6 cm, LiDAR 1,5 cm)
            alpha4=.2,   # quay -> sai số tịnh tiến
            alpha5=.3,   # đi ngang (Omni): bánh dính rồi giật
            max_beams=180,  # ~1/4 trong 680 tia X4; 60 tia quá ít cho phòng nhỏ
            # Scan được tin nhiều hơn (z_hit 0,9, sigma 0,08) nên cần ít particle hơn
            # SIM (1500) để bám; giảm tải CPU khi chạy AMCL trên Pi.
            max_particles=800, min_particles=300,
            z_hit=.9, z_rand=.1,  # tổng z_* phải ~1 với likelihood_field (z_short/z_max không dùng)
            sigma_hit=.08,  # bản đồ 4 cm + nhiễu X4 1-2 cm; 0,2 m chấp nhận lệch quá lớn
            # Không tự đặt pose (0,0,0) lúc khởi động: guard sẽ báo READY với pose sai.
            # Bắt buộc 2D Pose Estimate trên RViz.
            set_initial_pose=False)
    # Bánh cao su skid-steer: AMCL dùng mô hình vi sai (không lan particle theo
    # phương ngang) và smoother khóa vy = 0, đè cả smoother_max_lateral của xe thật.
    # MPPI vốn đã DiffDrive (vy_max 0) nên không đổi.
    if LaunchConfiguration('wheel_type').perform(context)=='rubber':
        amcl=config['amcl']['ros__parameters']
        amcl['robot_model_type']='nav2_amcl::DifferentialMotionModel'
        amcl.pop('alpha5',None)
        smoother=config['velocity_smoother']['ros__parameters']
        smoother['max_velocity'][1]=smoother['min_velocity'][1]=0.
    controller=LaunchConfiguration('controller').perform(context)
    if controller=='mppi':
        overlay=yaml.safe_load((Path(share('forklift_navigation'))/'config/nav2_mppi.yaml').read_text())
        config['controller_server']['ros__parameters'].update(overlay['controller_server']['ros__parameters'])
        # Xe thật: ít quỹ đạo lấy mẫu hơn SIM (1000) để giảm tải CPU (chuẩn bị chạy
        # Nav2 trên Pi). Xe chậm (<=0,25 m/s) nên 600 mẫu vẫn đủ phủ. Rỗng = như SIM.
        batch=LaunchConfiguration('mppi_batch_size').perform(context)
        if batch:config['controller_server']['ros__parameters']['FollowPath']['batch_size']=int(batch)
    if p6:
        control=config['controller_server']['ros__parameters']
        control['goal_checker_plugins'].append('pickup_goal_checker')
        # A is a standoff for mandatory LiDAR verification/docking. Keep the
        # general checker unchanged for delivery, refinement and legacy routes.
        control['pickup_goal_checker']={'plugin':'nav2_controller::SimpleGoalChecker',
            'stateful':True,'xy_goal_tolerance':.06,'yaw_goal_tolerance':.10}
        # B (NAV_DROP động) là chỗ hạ pallet xuống sàn, không cần khớp như đưa càng.
        # Xe thật 31/10/2026: dừng cách B 5,5 cm, lệch 25° -> general (3,5 cm) không đạt,
        # xe skid-steer không sửa ngang được nên "Failed to make progress", không hạ.
        # stateful: vào 8 cm là chốt xy, phần còn lại chỉ quay tại chỗ sửa góc.
        control['goal_checker_plugins'].append('drop_goal_checker')
        control['drop_goal_checker']={'plugin':'nav2_controller::SimpleGoalChecker',
            'stateful':True,'xy_goal_tolerance':.08,'yaw_goal_tolerance':.12}
        planners=config['planner_server']['ros__parameters']
        profile=copy.deepcopy(planners['GridBased'])
        profile.update(reverse_penalty=1.0,change_penalty=.15,non_straight_penalty=1.5,reuse_cached_path=False)
        # Pallet05/07: đoạn thẳng cuối ngắn cho phòng nhỏ (xe thật 30/10/2026: mọi cấu hình
        # 0,9-1,5 m đều bị tường chắn tới B -> phải quay tại chỗ). Chỉ được gọi khi các
        # cấu hình dài thất bại và drop_short_terminal bật.
        for name,length in [('Pallet',.9),('Pallet11',1.1),('Pallet13',1.3),('Pallet15',1.5),('Pallet07',.7),('Pallet05',.5)]:
            planners['planner_plugins'].append(name)
            planners[name]={**copy.deepcopy(profile),'terminal_distance':length}
        # PalletX (02/11/2026, pick_via_x): đường tới điểm ảo X trước pallet, vòng quanh vật
        # cản. Smac Hybrid thuần, không đoạn thẳng cuối; phạt đi cong/đổi hướng lái ở mức
        # thấp nhất (non_straight 1,0 = không phạt) và bán kính quay nhỏ hơn để vòng gắt.
        # Lùi vẫn phạt (2,0) để ưu tiên đi tiến vòng qua vật. Phạt chi phí giữ nguyên
        # để đường vẫn giữ khoảng cách với vật.
        free={k:v for k,v in copy.deepcopy(planners['GridBased']).items() if k!='terminal_distance'}
        free.update(plugin='nav2_smac_planner/SmacPlannerHybrid',non_straight_penalty=1.0,change_penalty=0.,
                    reverse_penalty=2.0,minimum_turning_radius=.35)
        planners['planner_plugins'].append('PalletX');planners['PalletX']=free
    g=yaml.safe_load((Path(share('forklift_description'))/'config/robot.yaml').read_text())
    # Include wheel contact spheres, chassis and fork tips in collision footprint.
    xmin=g['footprint_xmin']
    xmax=g['footprint_xmax']
    halfwidth=g['footprint_halfwidth']
    if p6:
        xmax=max(xmax,g['loaded_xmax'])
        halfwidth=max(halfwidth,g['loaded_halfwidth'])
    footprint=str([[xmin,-halfwidth],[xmax,-halfwidth],[xmax,halfwidth],[xmin,halfwidth]])
    for key in ['local_costmap','global_costmap']:
        config[key][key]['ros__parameters']['footprint']=footprint
    # Xe thật 02/11/2026: padding 1 cm cộng lượng tử ô 4 cm và lệch bản đồ/AMCL 3-5 cm làm
    # khe 2-3 cm sát tường thành "Starting point in lethal space". Rỗng = giữ nav2.yaml (SIM).
    padding=LaunchConfiguration('footprint_padding').perform(context)
    if padding:
        for key in ['local_costmap','global_costmap']:
            config[key][key]['ros__parameters']['footprint_padding']=float(padding)
    # StaticLayer ép global costmap theo độ phân giải BẢN ĐỒ (resolution trong nav2.yaml bị
    # bỏ qua). Bản đồ mịn hơn 4 cm -> Smac/TerminalApproach duyệt nhiều ô hơn ~(0,04/res)^2:
    # nới max_planning_time theo tỉ lệ cạnh ô để không đổi thành "hết giờ lập đường".
    # Hệ số chưa đo trên xe thật.
    map_resolution=float(yaml.safe_load(map_file.read_text()).get('resolution',.04))
    if map_resolution<.04:
        planners=config['planner_server']['ros__parameters']
        for name in planners['planner_plugins']:
            if 'max_planning_time' in planners[name]:
                planners[name]['max_planning_time']*=.04/map_resolution
    config['map_server']['ros__parameters']['yaml_filename']=str(map_file.resolve())
    fd,path=tempfile.mkstemp(prefix='forklift_nav2_',suffix='.yaml')
    with os.fdopen(fd,'w') as stream:yaml.safe_dump(config,stream)
    nodes=[relay]
    # Bước 2 né vật (01/11/2026): nhận biết vật tĩnh/động, CHỈ hiển thị/công bố, không lái xe.
    if LaunchConfiguration('obstacle_tracker').perform(context)=='true':
        nodes.append(Node(package='forklift_navigation',executable='obstacle_tracker.py',
            parameters=[common,{k:float(g[k]) for k in ('footprint_xmin','footprint_xmax','footprint_halfwidth',
                                                       'loaded_xmax','loaded_halfwidth')}],output='screen'))
    def node(package,executable,name=None,remappings=None):
        return Node(package=package,executable=executable,name=name or executable,parameters=[path,common],
                    remappings=remappings or [],output='screen')
    nodes.extend([
        node('nav2_map_server','map_server'),node('nav2_amcl','amcl'),
        Node(package='nav2_controller',executable='controller_server',name='controller_server',
             parameters=[path,common],remappings=[('cmd_vel','/nav/cmd_vel_raw')],
             additional_env={'OMP_NUM_THREADS':'1'},output='screen'),
        node('nav2_planner','planner_server'),node('nav2_smoother','smoother_server'),
        node('nav2_behaviors','behavior_server',remappings=[('cmd_vel','/nav/cmd_vel_raw')]),
        node('nav2_bt_navigator','bt_navigator'),node('nav2_waypoint_follower','waypoint_follower'),
        node('nav2_velocity_smoother','velocity_smoother',remappings=[('cmd_vel','/nav/cmd_vel_selected' if p6 else '/nav/cmd_vel_raw'),('cmd_vel_smoothed','/nav/cmd_vel_smoothed')]),
        Node(package='forklift_navigation',executable='navigation_guard.py',parameters=[common,{'require_imu':use_ekf},
             # Rỗng = 0,7 s như SIM; xe thật nới vì AMCL qua Wi-Fi có lúc bỏ vài scan.
             {'map_tf_timeout':float(LaunchConfiguration('guard_map_tf_timeout').perform(context) or .7)}],output='screen',
             on_exit=EmitEvent(event=Shutdown(reason='Navigation guard exited'))),
    ])
    for label,names in [('localization',['map_server','amcl']),('navigation',['controller_server','planner_server','smoother_server','behavior_server','bt_navigator','waypoint_follower','velocity_smoother'])]:
        nodes.append(Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name=f'lifecycle_manager_{label}',
            parameters=[common,{'autostart':autostart,'node_names':names,'bond_timeout':4.}],output='screen'))
    def cleanup(event,context):Path(path).unlink(missing_ok=True);return []
    return [RegisterEventHandler(OnShutdown(on_shutdown=cleanup)),*nodes]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('p6',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('autostart',default_value='true',choices=['true','false']),
        DeclareLaunchArgument('use_sim_time',default_value='true',choices=['true','false']),
        DeclareLaunchArgument('controller',default_value='mppi',choices=['rpp','mppi']),
        DeclareLaunchArgument('mode',default_value='mapping',choices=['mapping','localization']),
        DeclareLaunchArgument('obstacle_tracker',default_value='true',choices=['true','false']),
        DeclareLaunchArgument('use_ekf',default_value='true',choices=['true','false']),
        DeclareLaunchArgument('map',default_value=str(Path(share('forklift_navigation'))/'maps/warehouse.yaml')),
        DeclareLaunchArgument('behavior_transform_tolerance',default_value=''),
        DeclareLaunchArgument('smoother_max_lateral',default_value=''),
        DeclareLaunchArgument('spin_min_rotational_vel',default_value=''),
        DeclareLaunchArgument('spin_max_rotational_vel',default_value=''),
        DeclareLaunchArgument('guard_map_tf_timeout',default_value=''),
        DeclareLaunchArgument('amcl_profile',default_value='',choices=['','real']),
        DeclareLaunchArgument('mppi_batch_size',default_value=''),
        DeclareLaunchArgument('smoother_accel',default_value=''),
        DeclareLaunchArgument('footprint_padding',default_value=''),
        DeclareLaunchArgument('wheel_type',default_value='rubber',choices=['rubber','mecanum']),
        OpaqueFunction(function=setup)])
