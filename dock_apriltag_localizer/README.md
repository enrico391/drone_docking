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
