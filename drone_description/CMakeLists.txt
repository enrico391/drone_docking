cmake_minimum_required(VERSION 3.8)
project(drone_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY
    models/drone_docking_base_structure
    models/mono_cam
    models/x500
    models/x500_triangular_base
    models/x500_mono_cam_down
    models/x500_with_ardupilot
  DESTINATION share/${PROJECT_NAME}/models
)

ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/hooks/drone_description.dsv.in"
  "${CMAKE_CURRENT_SOURCE_DIR}/hooks/drone_description.sh.in"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
