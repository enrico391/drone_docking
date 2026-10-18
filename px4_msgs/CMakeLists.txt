cmake_minimum_required(VERSION 3.8)
project(px4_msgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/OffboardControlMode.msg"
  "msg/TrajectorySetpoint.msg"
  "msg/VehicleCommand.msg"
  "msg/VehicleCommandAck.msg"
  "msg/VehicleLandDetected.msg"
  "msg/VehicleLocalPosition.msg"
  "msg/VehicleStatus.msg"
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
