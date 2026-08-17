# Drone docking base with central 660x660 mm structure — Gazebo Harmonic

This model matches the **box / open-front structure** you described more closely than the previous flat plate.

## Geometry

### Central structure
- outer size: **660 x 660 x 350 mm**
- open front cavity faces **+X**
- wall thickness: **30 mm**
- top landing recess opening: **320 x 320 mm**
- recess floor: **70 mm below the top surface**
- central AprilTag: **120 x 120 mm**, ID **4**, placed on the recess floor

### Outer AprilTags
- size: **300 x 300 mm** each
- family: **tag36h11**
- IDs: **0, 1, 2, 3**
- positions (centers, meters):
  - ID0 NE: `( +0.520, +0.520 )`
  - ID1 NW: `( -0.520, +0.520 )`
  - ID2 SW: `( -0.520, -0.520 )`
  - ID3 SE: `( +0.520, -0.520 )`

The tags are on the ground and are **visual only**, which is usually what you want for AprilTag-based docking simulation.

## Files
- `model.sdf` — main model
- `model.config`
- `world_example.sdf` — minimal Gazebo world including the model
- `materials/textures/*.png` — AprilTag textures
- `meshes/tag_plane.obj` — thin plane used for tag visuals

## Install

Add the parent directory to Gazebo's resource path:

```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/path/to/parent/of/drone_docking_base_structure
```

Launch the example world:

```bash
gz sim /path/to/drone_docking_base_structure/world_example.sdf
```

Or include it in your own world:

```xml
<include>
  <uri>model://drone_docking_base_structure</uri>
  <pose>0 0 0 0 0 0</pose>
</include>
```

## Notes / easy edits

If you want to tune the model, the easiest values to change in `model.sdf` are:
- central structure height `0.350`
- outer tag center offset `0.520`
- top recess size `0.320`
- wall thickness `0.030`

## Suggested next step

If you want, the next refinement can be one of these:
1. make the **top recess dimensions match your real base** exactly,
2. add **charging contacts / rails** with more realistic geometry,
3. add a dedicated **dock reference frame** and extra collision tuning,
4. make the whole thing as a **mesh imported from Fusion 360 / STL**.
