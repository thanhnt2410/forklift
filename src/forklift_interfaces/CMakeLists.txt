cmake_minimum_required(VERSION 3.8)
project(forklift_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/RunMission.action" "action/HomeLift.action" "action/MoveLift.action"
  "msg/LiftCommand.msg" "msg/LiftSensors.msg" "msg/LiftState.msg"
  DEPENDENCIES std_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
