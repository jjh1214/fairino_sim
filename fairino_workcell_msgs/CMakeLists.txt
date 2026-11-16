cmake_minimum_required(VERSION 3.8)
project(fairino_workcell_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/IOState.msg"
  "msg/StepReport.msg"
  "srv/Gripper.srv"
  "srv/SetIO.srv"
  "srv/Wait.srv"
  "action/Move.action"
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
