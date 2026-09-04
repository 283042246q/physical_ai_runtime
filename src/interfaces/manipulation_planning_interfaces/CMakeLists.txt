cmake_minimum_required(VERSION 3.16)
project(manipulation_planning_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/PlanBimanualTrajectory.action"
  "msg/DynamicObject.msg"
  "msg/DynamicWorld.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs trajectory_msgs
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
