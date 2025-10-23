cmake_minimum_required(VERSION 3.5)
project(rmf_manager_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotState.msg"
  "msg/RobotCommand.msg"
  "msg/RobotEvent.msg"
  "msg/Edge2D.msg"
  "msg/Vertex2D.msg"
  "msg/MapMetadata.msg"
  "srv/LoadMap.srv"
  "srv/SchedulePatrol.srv"
  "srv/ScheduleDelivery.srv"
  "srv/ScheduleGoTo.srv"
  "srv/CancelTask.srv"
  "srv/CancelAll.srv"
  "srv/ListTasks.srv"
  "srv/SetResumeAfterCharge.srv"
  "srv/TriggerDelivery.srv"
  "srv/MoveTask.srv"
  DEPENDENCIES std_msgs nav_msgs geometry_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
