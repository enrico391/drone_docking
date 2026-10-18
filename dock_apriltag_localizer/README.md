# dock_apriltag_localizer

ROS 2 node for two-stage precision docking with multiple AprilTags.

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

## Dependencies

ROS packages:
```bash
sudo apt install ros-$ROS_DISTRO-cv-bridge python3-opencv
```

Create a workspace-local Python environment that can also import the ROS 2
packages installed by apt:
```bash
cd /home/ubuntu/drone_ws
python3 -m venv --system-site-packages apriltag_venv
touch apriltag_venv/COLCON_IGNORE
source apriltag_venv/bin/activate
python -m pip install -r \
  src/drone_docking/dock_apriltag_localizer/requirements.txt
```

`COLCON_IGNORE` prevents colcon from interpreting package metadata inside the
virtual environment as workspace packages.

## Build
```bash
cd /home/ubuntu/drone_ws
source /opt/ros/lyrical/setup.bash
source apriltag_venv/bin/activate
python -m colcon build --symlink-install \
  --packages-select dock_apriltag_localizer
source install/setup.bash
```

Use `python -m colcon`, not the `/usr/bin/colcon` executable. The latter has a
system-Python shebang and can generate a `dock_localizer` entry point that
cannot import packages installed only in `apriltag_venv`.

## Run
```bash
ros2 launch dock_apriltag_localizer dock_localizer.launch.py \
  image_topic:=/camera/image_raw \
  camera_info_topic:=/camera/camera_info
```

The launch excludes Python user-site packages (`PYTHONNOUSERSITE=1`) so an
unrelated `~/.local` NumPy installation cannot shadow the ROS/OpenCV runtime.
Install localizer dependencies into `apriltag_venv`, not the user site.
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

Use the combined launch file to start both `camera_ros/camera_node` and the
dock localizer:

```bash
cd /home/ubuntu/drone_ws
source /opt/ros/lyrical/setup.bash
source apriltag_venv/bin/activate
source install/setup.bash

ros2 launch dock_apriltag_localizer \
  dock_localizer_with_camera.launch.py
```

The PX4 manager can select this combined launch with
`ros2 launch flight_cycle_manager px4_flight_cycle.launch.py use_real_camera:=true`.
It starts exactly one localizer plus the driver, defaults to wall-clock time,
and forwards the camera settings and tag configuration. This is only a camera
selection; the manager's default flight profile remains SITL-only.

The combined launch modifies the environment of `camera_node` only. It:

- prepends `/usr/lib/<multiarch>` to the camera process's
  `LD_LIBRARY_PATH`;
- selects the system IPA module and proxy under `/usr/lib` and
  `/usr/libexec`;
- selects tuning data from `/usr/share/libcamera/ipa`;
- leaves the localizer in the ROS and Python virtual-environment setup, with
  user-site Python packages excluded to avoid NumPy/OpenCV ABI conflicts.

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

## MAVLink dock-pose bridge

For standalone precision-landing debugging, launch the bridge with direct
access to ArduPilot's MAVLink serial endpoint:

```bash
ros2 launch dock_apriltag_localizer dock_pose_mavlink.launch.py
```

The bridge subscribes to `/dock/pose`, converts the downward camera optical
position to MAVLink Body-FRD, and sends a position-valid `LANDING_TARGET` for
every received pose. The MAVLink measurement timestamp comes from the source
image rather than the bridge transmission time. It uses MAVLink 2 with the
`ardupilotmega` dialect and connects through `pymavlink` directly; MAVProxy is
not required. The default endpoint is `/dev/ttyAMA0` at `921600` baud.

The camera origin offset from the vehicle body origin is configurable in
Body-FRD. For example:

```bash
ros2 launch dock_apriltag_localizer dock_pose_mavlink.launch.py \
  mavlink_connection:=/dev/ttyAMA0 mavlink_baud:=921600 \
  camera_down_m:=0.206
```

For SITL, select its direct TCP endpoint instead:

```bash
ros2 launch dock_apriltag_localizer dock_pose_mavlink.launch.py \
  mavlink_connection:=tcp:127.0.0.1:5760
```

Exactly one process should own the selected serial or TCP endpoint. Do not run
this bridge at the same time as `flight_cycle_manager`, which maintains its own
direct pymavlink connection unless its launch starts the localizer itself.

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

Then reload without restarting the node:
```bash
ros2 service call /dock_apriltag_localizer/reload_config std_srvs/srv/Trigger '{}'
```

**Important:** `size_m` is the physical AprilTag square side used by pose estimation. Measure the exact convention matching the generated tag (normally the outer black square used by your tag generator/detector workflow) and stay consistent.

## Geometry
`dock_to_tag.xyz_m` is the tag center position relative to dock center. If the tag is coplanar and aligned with the dock, keep `rpy_deg: [0,0,0]`.
