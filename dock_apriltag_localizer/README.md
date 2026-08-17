# dock_apriltag_localizer

ROS 2 node for two-stage precision docking with multiple AprilTags.

## Concept
- Outer tags: long-range / coarse centering.
- Central tag: short-range / final precision landing.
- Every detected tag is converted to the same `dock_target` frame using the fixed geometry in `config/dock_tags.yaml`.
- Multiple outer tag estimates are fused.
- Central switching uses distance threshold + consecutive-frame hysteresis.

## Dependencies

ROS packages:
```bash
sudo apt install ros-$ROS_DISTRO-cv-bridge python3-opencv
```

Python AprilTag detector (install in the Python environment used by ROS):
```bash
python3 -m pip install pupil-apriltags pyyaml
```

## Build
```bash
cd ~/ros2_ws/src
cp -r /path/to/dock_apriltag_localizer .
cd ~/ros2_ws
colcon build --symlink-install --packages-select dock_apriltag_localizer
source install/setup.bash
```

## Run
```bash
ros2 launch dock_apriltag_localizer dock_localizer.launch.py \
  image_topic:=/camera/image_raw \
  camera_info_topic:=/camera/camera_info
```

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

The combined launch modifies the environment of `camera_node` only. It:

- prepends `/usr/lib/<multiarch>` to the camera process's
  `LD_LIBRARY_PATH`;
- selects the system IPA module and proxy under `/usr/lib` and
  `/usr/libexec`;
- selects tuning data from `/usr/share/libcamera/ipa`;
- leaves the localizer in the normal ROS and Python virtual-environment setup.

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
- `/dock/pose` (`geometry_msgs/PoseStamped`): dock center in camera optical frame.
- TF: `camera_frame -> dock_target`
- `/dock/mode`: `OUTER`, `CENTRAL_ACQUIRING`, `CENTRAL`, or `NO_TARGET`.
- `/dock/visible_tag_ids`
- `/dock/debug_image`

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
