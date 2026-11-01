cmake_minimum_required(VERSION 3.16)
project(dock_apriltag_localizer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d)
find_package(yaml-cpp REQUIRED)
find_package(apriltag REQUIRED)

add_library(dock_pose_processing SHARED src/pose_processing.cpp src/detection.cpp)
target_include_directories(dock_pose_processing PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include> ${OpenCV_INCLUDE_DIRS})
target_link_libraries(dock_pose_processing PUBLIC Eigen3::Eigen ${OpenCV_LIBS}
  PRIVATE yaml-cpp::yaml-cpp apriltag::apriltag)

add_library(dock_localizer_component SHARED src/dock_localizer.cpp)
target_include_directories(dock_localizer_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(dock_localizer_component PUBLIC dock_pose_processing
  rclcpp::rclcpp rclcpp_components::component ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS} ${std_msgs_TARGETS} ${std_srvs_TARGETS}
  tf2_ros::tf2_ros cv_bridge::cv_bridge)
rclcpp_components_register_node(dock_localizer_component
  PLUGIN "dock_apriltag_localizer::DockLocalizer" EXECUTABLE dock_localizer)

install(TARGETS dock_pose_processing dock_localizer_component
  EXPORT export_${PROJECT_NAME} ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pose_processing test/test_pose_processing.cpp)
  target_link_libraries(test_pose_processing dock_pose_processing)
  ament_add_gtest(test_detection test/test_detection.cpp)
  target_link_libraries(test_detection dock_pose_processing apriltag::apriltag)
  target_compile_definitions(test_detection PRIVATE TEST_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/dock_tags.yaml")
  ament_add_gtest(test_component test/test_component.cpp
    APPEND_ENV "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_logs")
  target_link_libraries(test_component dock_localizer_component apriltag::apriltag)
  target_compile_definitions(test_component PRIVATE TEST_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/dock_tags.yaml")
endif()
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 OpenCV rclcpp rclcpp_components sensor_msgs geometry_msgs std_msgs std_srvs tf2_ros cv_bridge)
ament_package()
