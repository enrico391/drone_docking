cmake_minimum_required(VERSION 3.8)
project(drone_description)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# Install the verified firmware explicitly; PATH may still contain 4.8 SITL.
get_filename_component(_workspace "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
set(ARDUPILOT47_ROOT "${_workspace}/.worktrees/ardupilot-4.7.0" CACHE PATH
    "Exact ArduPilot 1511f271 source tree with a DDS-enabled SITL build")
execute_process(COMMAND git -C "${ARDUPILOT47_ROOT}" rev-parse HEAD
  OUTPUT_VARIABLE _firmware_commit OUTPUT_STRIP_TRAILING_WHITESPACE
  RESULT_VARIABLE _firmware_git_result)
if(NOT _firmware_git_result EQUAL 0 OR
  NOT _firmware_commit STREQUAL "1511f27194f1dcc3728270883047bdf022b3fd53")
  message(FATAL_ERROR "ARDUPILOT47_ROOT must point to verified commit 1511f271")
endif()
if(NOT EXISTS "${ARDUPILOT47_ROOT}/build/sitl/bin/arducopter")
  message(FATAL_ERROR "Build ArduCopter 4.7.0 SITL in ${ARDUPILOT47_ROOT} first")
endif()
file(SHA256 "${ARDUPILOT47_ROOT}/build/sitl/bin/arducopter" _firmware_sha256)
if(NOT _firmware_sha256 STREQUAL
  "f40655c42537293d800ddc9d85b566468352a6844557cbe35379d6b6caa0badb")
  message(FATAL_ERROR "ArduCopter binary does not match the verified 4.7.0 DDS build")
endif()
install(PROGRAMS "${ARDUPILOT47_ROOT}/build/sitl/bin/arducopter"
  DESTINATION lib/${PROJECT_NAME} RENAME arducopter-4.7.0)
install(FILES "${ARDUPILOT47_ROOT}/Tools/autotest/default_params/copter.parm"
  DESTINATION share/${PROJECT_NAME}/config RENAME copter-4.7.parm)

install(
  DIRECTORY config launch worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/disturbance_injector
  DESTINATION lib/${PROJECT_NAME})

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
  find_package(ament_cmake_pytest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(disturbance_model_test
    test/test_disturbance_model.py TIMEOUT 30)
  ament_add_pytest_test(disturbance_injector_test
    test/test_disturbance_injector.py TIMEOUT 30
    ENV "ROS_HOME=/tmp/ground_disturbances_ros_home")
  ament_add_pytest_test(disturbance_wiring_test
    test/test_disturbance_wiring.py TIMEOUT 30
    ENV "ROS_HOME=/tmp/ground_disturbances_ros_home")
endif()

ament_package()
