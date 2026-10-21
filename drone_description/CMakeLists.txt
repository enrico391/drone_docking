cmake_minimum_required(VERSION 3.8)
project(drone_description)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)

# Keep GUI state flowing when WSL or NTP steps CLOCK_REALTIME backwards.
# Use a distinct library and plugin alias; never replace the system plugin.
add_library(drone_scene_broadcaster SHARED
  vendor/gz_sim8_8_15_0/SceneBroadcaster.cc)
target_compile_features(drone_scene_broadcaster PRIVATE cxx_std_17)
target_link_libraries(drone_scene_broadcaster PRIVATE
  gz-sim8::gz-sim8 gz-plugin2::gz-plugin2-register)
install(TARGETS drone_scene_broadcaster
  LIBRARY DESTINATION lib/${PROJECT_NAME})
install(FILES vendor/gz_sim8_8_15_0/LICENSE
  vendor/gz_sim8_8_15_0/README.md
  DESTINATION share/${PROJECT_NAME}/vendor/gz_sim8_8_15_0)

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
set(_firmware_binary "${ARDUPILOT47_ROOT}/build/sitl/bin/arducopter")
if(NOT EXISTS "${_firmware_binary}")
  message(FATAL_ERROR "Build ArduCopter 4.7.0 SITL in ${ARDUPILOT47_ROOT} first")
endif()

# SITL embeds absolute source paths, so its whole-file checksum changes across
# otherwise equivalent workspaces. Verify the firmware identity and that DDS
# was actually linked instead.
file(STRINGS "${_firmware_binary}" _firmware_version
  REGEX "ArduCopter V4\\.7\\.0 \\(1511f271\\)")
if(NOT _firmware_version)
  message(FATAL_ERROR "ArduCopter binary is not version 4.7.0 at commit 1511f271")
endif()

execute_process(COMMAND "${CMAKE_NM}" -C "${_firmware_binary}"
  OUTPUT_VARIABLE _firmware_symbols RESULT_VARIABLE _firmware_nm_result)
string(FIND "${_firmware_symbols}" "AP_DDS_Client::start()" _firmware_dds_symbol)
if(NOT _firmware_nm_result EQUAL 0 OR _firmware_dds_symbol EQUAL -1)
  message(FATAL_ERROR
    "ArduCopter 4.7.0 is missing DDS; configure with --enable-DDS and rebuild")
endif()

install(PROGRAMS "${_firmware_binary}"
  DESTINATION lib/${PROJECT_NAME} RENAME arducopter-4.7.0)
install(FILES "${ARDUPILOT47_ROOT}/Tools/autotest/default_params/copter.parm"
  DESTINATION share/${PROJECT_NAME}/config RENAME copter-4.7.parm)

install(
  DIRECTORY config launch worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/disturbance_injector scripts/disturbance_tuner
  scripts/camera_latency
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY
    models/drone_docking_base_structure
    models/mono_cam
    models/x500
    models/x500_px4_base
    models/x500_px4_mono_cam_down
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
  ament_add_pytest_test(disturbance_tuner_test
    test/test_disturbance_tuner.py TIMEOUT 30)
  ament_add_pytest_test(camera_latency_test
    test/test_camera_latency.py TIMEOUT 30)
  ament_add_pytest_test(camera_latency_launch_test
    test/test_camera_latency_launch.py TIMEOUT 30)
  ament_add_pytest_test(px4_launch_test
    test/test_px4_launch.py TIMEOUT 30)
  ament_add_pytest_test(scene_broadcaster_test
    test/test_scene_broadcaster.py TIMEOUT 90)
endif()

ament_package()
