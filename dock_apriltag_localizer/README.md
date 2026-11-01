# dock_apriltag_localizer

C++ ROS 2 component and standalone node for two-stage precision docking with
multiple AprilTags. The combined camera launch loads this component and
`camera::CameraNode` into one process with intra-process communication enabled.

## Concept
- Outer tags: long-range / coarse centering.
- Central tag: short-range / final precision landing.
- Every detected tag is converted to the same `dock_target` frame using the fixed geometry in `config/dock_tags.yaml`.
- Multiple outer tag estimates are fused.
- Central switching uses distance threshold + consecutive-frame hysteresis.
- The selected pose is temporally smoothed and implausible position jumps are
  rejected before `/dock/pose` is published.

## Frame timing diagnostics

While the camera and localizer run, inspect timing for each processed image:

```bash
ros2 topic echo /dock/timing
```

All durations are seconds. `image_age_at_callback_s` is ROS time at image
callback entry minus the original image stamp. `pose_age_at_publish_s` is
the same age immediately before publishing an accepted pose; it is `null`
when no pose is published. `source_stamp_s` identifies the source image.
`conversion_s` includes callback setup and image conversion; `detection_s`
measures AprilTag detection; `pose_processing_s` includes candidate estimation,
filtering and pose/mode publication; `debug_image_s` measures the optional debug
image work. `callback_processing_s` covers the callback up to timing-message
serialization. Processing durations use a monotonic clock.

Timing is also published for no-target and filter-rejected frames, with `mode`
and `filter_status`. Images received before camera calibration is available
are skipped. Zero image timestamps produce `null` ages; negative ages are
preserved to expose future timestamps or clock mismatch. Capture-age accuracy
requires acquisition timestamps and synchronized ROS clocks. These ages do
not include the extra signal lag introduced by pose smoothing.

## Dependencies and build

The localizer uses native AprilTag, OpenCV, Eigen and yaml-cpp, with no Python
AprilTag dependency at runtime. Install dependencies through rosdep:

```bash
cd /home/ubuntu/drone_ws
source /opt/ros/lyrical/setup.bash
rosdep install --from-paths src/drone_docking/dock_apriltag_localizer --ignore-src -r -y
```

No virtual environment is needed to build or run the C++ localizer. This package changed from
`ament_python` to `ament_cmake`; use fresh build/install bases to avoid mixing
old Python entry points with the new C++ executable:

```bash
cd /home/ubuntu/drone_ws
source /opt/ros/lyrical/setup.bash
python3 -m colcon build --symlink-install \
  --build-base build_cpp --install-base install_cpp \
  --packages-select dock_apriltag_localizer \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install_cpp/setup.bash
```

Source the existing workspace first if you need its other packages, then source
`install_cpp/setup.bash` last so this C++ package takes precedence. For a clean
workspace, the usual default `build`/`install` bases also work.

The retained Python localizer/filter source and `requirements.txt` serve only
as optional regression references and are not installed. The runtime package
has no Python detector or MAVLink dependencies. The installed `dock_localizer`
executable is C++.

Run the native tests with:

```bash
python3 -m colcon test --build-base build_cpp --install-base install_cpp \
  --packages-select dock_apriltag_localizer --event-handlers console_direct+
python3 -m colcon test-result --test-result-base build_cpp/dock_apriltag_localizer
```

## Run
```bash
ros2 launch dock_apriltag_localizer dock_localizer.launch.py \
  image_topic:=/camera/image_raw \
  camera_info_topic:=/camera/camera_info
```

The standalone launch runs the native `dock_localizer` executable directly.
`python_executable` is accepted as a deprecated compatibility argument for
existing parent launch files and has no effect on the C++ localizer.
`use_sim_time` defaults to `false`; set it to `true` when using Gazebo `/clock`.
When included by the PX4 manager, tag geometry is passed explicitly through
`localizer_config`, separately from the manager's `config_file`.

## Raspberry Pi camera and ROS libcamera workaround

On the Raspberry Pi target, sourcing ROS 2 Lyrical puts the ROS-vendored
libcamera libraries from `/opt/ros/lyrical` before the Ubuntu libraries in
`/usr`. The ROS libcamera stack detects and configures the OV5647, but its IPA
worker can terminate with:

```text
FATAL Serializer control_serializer.cpp:626
A list of V4L2 controls requires a ControlInfoMap
```

The native Raspberry Pi stack is known to work on this target. It can be
checked independently of the ROS environment with:

```bash
env -u LD_LIBRARY_PATH rpicam-hello --timeout 5000 --nopreview
```

If this command cannot stream from the camera, fix the native camera, kernel,
or device permissions first. The ROS workaround will not fix a native camera
failure.

Use the combined launch file to compose the camera and dock localizer:

```bash
cd /home/ubuntu/drone_ws
source /opt/ros/lyrical/setup.bash
source install_cpp/setup.bash

ros2 launch dock_apriltag_localizer \
  dock_localizer_with_camera.launch.py
```

The PX4 manager can select this combined launch with
`ros2 launch flight_cycle_manager px4_flight_cycle.launch.py use_real_camera:=true`.
It starts exactly one localizer plus the driver, defaults to wall-clock time,
and forwards the camera settings and tag configuration. This is only a camera
selection; the manager's default flight profile remains SITL-only.

The combined launch starts one `rclcpp_components/component_container_mt`
process containing `camera::CameraNode` and
`dock_apriltag_localizer::DockLocalizer`. Both components explicitly enable
`use_intra_process_comms`. To compare against DDS transport while retaining
composition, pass `use_intra_process_comms:=false`.

The image subscription uses best-effort, volatile, keep-last depth 1. Incoming
images are shared as const messages. Native mono8 frames share the grayscale
buffer directly; RGB/BGR frames need a grayscale conversion for detection.
Debug annotation always uses a private copy. Intra-process transport avoids
serialization between these two components; it does not eliminate image-format
conversion or copies required by additional subscribers.

The libcamera workaround now applies to the shared container process. It:

- prepends `/usr/lib/<multiarch>` to the camera process's
  `LD_LIBRARY_PATH`;
- selects the system IPA module and proxy under `/usr/lib` and
  `/usr/libexec`;
- selects tuning data from `/usr/share/libcamera/ipa`;
- applies the same library environment to the localizer, since both components
  share one process. The C++ localizer does not load NumPy or pupil-apriltags.

Both nodes use wall-clock time by default. `use_sim_time:=true` explicitly
selects ROS simulation time for both.

The camera log should therefore report the working system version and paths,
for example:

```text
libcamera v0.7.0
Using tuning file /usr/share/libcamera/ipa/rpi/vc4/ov5647.json
```

The default camera configuration is `800x600` with `RGB888`. Its OV5647
calibration is installed from `config/ov5647_800x600.yaml`. Common overrides
can be passed as launch arguments:

```bash
ros2 launch dock_apriltag_localizer \
  dock_localizer_with_camera.launch.py \
  camera:=0 \
  camera_width:=800 \
  camera_height:=600 \
  camera_format:=RGB888
```

## Main outputs
- `/dock/pose` (`geometry_msgs/PoseStamped`): filtered dock center in camera
  optical frame. Its timestamp remains the source image acquisition time.
- TF: `camera_frame -> dock_target`
- `/dock/mode`: `OUTER`, `CENTRAL_ACQUIRING`, `CENTRAL`, or `NO_TARGET`.
- `/dock/visible_tag_ids`
- `/dock/debug_image`
- `/dock/timing` (`std_msgs/String`): per-frame JSON timing diagnostics.

## Pose filter

The temporal filter is enabled by default. It combines exponential position
and rotation smoothing with a speed-aware position innovation gate. An
isolated pose outside the gate is not published. Three mutually consistent
rejected poses reacquire the target, avoiding permanent lockout after a real
large displacement.

The main controls are exposed as launch arguments:

```bash
ros2 launch dock_apriltag_localizer dock_localizer.launch.py \
  pose_filter_enabled:=true \
  pose_filter_time_constant_s:=0.12 \
  pose_filter_max_translation_speed_m_s:=2.0 \
  pose_filter_max_position_jump_m:=0.20 \
  pose_filter_reacquire_frames:=3
```

Set `pose_filter_enabled:=false` for a raw-pose comparison. A smaller time
constant follows motion more closely; a larger value smooths more but adds
latency. The jump gate allows the configured fixed jump plus the configured
translation speed multiplied by the sample interval.

## Pose convention

`/dock/pose` is `T_camera_optical_dock`: the pose of the dock origin expressed
in `camera_optical_frame`. The optical axes are X right, Y down, Z forward. The
dock axes are X through the open front, Y left, Z up. Consequently, for a level
downward camera, the dock's Z axis points approximately along optical -Z.

The planar PnP result is corrected by 180 degrees about tag Y before tag
offsets are applied. This is important for the outer tags: using the uncorrected
normal rotates their `dock_to_tag` offsets to the wrong side of the image.

## Change tag sizes / positions quickly
Edit `config/dock_tags.yaml`, for example:
```yaml
- id: 4
  role: central
  size_m: 0.15
  dock_to_tag:
    xyz_m: [0.0, 0.0, 0.0]
    rpy_deg: [0.0, 0.0, 0.0]
```

Then reload without restarting the node. Reload replaces the detector and tag
configuration atomically, resets switching and filtering, and retains the old
configuration if the new file is invalid:
```bash
ros2 service call /dock_apriltag_localizer/reload_config std_srvs/srv/Trigger '{}'
```

**Important:** `size_m` is the physical AprilTag square side used by pose estimation. Measure the exact convention matching the generated tag (normally the outer black square used by your tag generator/detector workflow) and stay consistent.

## Geometry
`dock_to_tag.xyz_m` is the tag center position relative to dock center. If the tag is coplanar and aligned with the dock, keep `rpy_deg: [0,0,0]`.
