cmake_minimum_required(VERSION 3.8)
project(swarm_dwb_critics)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(dwb_core REQUIRED)
find_package(dwb_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_2d_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/virtual_velocity_critic.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  dwb_core
  dwb_msgs
  geometry_msgs
  nav2_costmap_2d
  nav2_util
  nav_2d_msgs
  nav_msgs
  pluginlib
  rclcpp
  tf2
  tf2_geometry_msgs
)

pluginlib_export_plugin_description_file(dwb_core plugins.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_velocity_target test/test_velocity_target.cpp)
  target_include_directories(test_velocity_target PUBLIC include)
  target_link_libraries(test_velocity_target ${PROJECT_NAME})
  ament_target_dependencies(test_velocity_target dwb_core nav_2d_msgs)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  dwb_core
  dwb_msgs
  geometry_msgs
  nav2_costmap_2d
  nav2_util
  nav_2d_msgs
  nav_msgs
  pluginlib
  rclcpp
  tf2
  tf2_geometry_msgs
)
ament_package()
