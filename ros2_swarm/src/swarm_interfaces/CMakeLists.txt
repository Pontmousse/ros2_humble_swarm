cmake_minimum_required(VERSION 3.8)
project(swarm_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/StateFull.msg"
  "msg/StatePos.msg"
  "msg/HedgeImuAddressed.msg"
  "msg/CoordXY.msg"
  "msg/Markers.msg"
  "msg/Neighbours.msg"
  "msg/NeighbourState.msg"
  "msg/NeighbourStates.msg"
  "msg/ApBids.msg"
  "msg/Readiness.msg"
  "msg/TargetEstimate.msg"
  # Copied byte-identical from the driver stack (jasonli03/swarm_rover_v2); DDS matches them by name+layout
  "msg/ArucoMarker.msg"
  "msg/ArucoMarkerArray.msg"
  DEPENDENCIES geometry_msgs sensor_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)


ament_package()
