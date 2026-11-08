--------------------SSH PI4 ---------------------- 
ssh thainguyen1233@192.168.100.205
--------------------Chạy để vẽ map ---------------------- 
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=1
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
 
ngoài ra . install/setup.bash 

ros2 launch forklift_bringup real.launch.py
ros2 launch forklift_bringup lidar.launch.py
ros2 launch forklift_bringup mapping_real.launch.py   #laptop

Lưu map: Nhập đường dẫn vào ô Save Map và bấm Save Map
--------------------Chạy khi có map ---------------------- 
terminal trên PC và pi:
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=1
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
cd ros_ws/forklift
. install/setup.bash 


ros2 launch forklift_bringup real.launch.py
ros2 launch forklift_bringup lidar.launch.py scan_topic:=/scan_raw
ros2 launch forklift_bringup navigation_real.launch.py \
  map:=/home/thainguyen1233/forklift/src/mecanum_navigation/maps/home1.yaml
  
Lệnh để xe căn chỉnh đúng ở map:
ros2 service call /request_nomotion_update std_srvs/srv/Empty