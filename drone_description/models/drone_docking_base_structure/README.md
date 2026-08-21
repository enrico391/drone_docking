# Drone docking base — Gazebo Harmonic

This model uses the physical docking-base mockup exported from Fusion 360 and
adds the AprilTags as separate Gazebo visuals.

## Geometry

- Base mesh: `meshes/base.stl`
- Mesh size: **1.600 x 1.600 x 1.060 m**
- Mesh pose: translated **+0.000235591 m** in Z so its bottom is at `z=0`
- Large upper surface: `z=1.000 m`
- Central tag recess floor: `z=0.880 m`

The base STL is used for rendering. Collision uses primitive boxes for the
lower body, center recess floor, and upper deck outside an `0.8 x 0.8 m`
vehicle-clearance envelope. The matching triangular drone insert uses four
spherical point contacts at its bottom face. Keeping the close-fitting CAD
meshes out of the physics contact pair prevents startup impulses, tilting, and
blocked takeoff while preserving the exact visual geometry.

## AprilTags

- Family: **tag36h11**
- Center tag: ID 4, **120 x 120 mm**, pose `(0, 0, 0.881)`
- Outer tags: IDs 0-3, **300 x 300 mm**, at `z=1.001 m`
- Outer centers: `(+-0.520, +-0.520) m`

The 1 mm tag offset above each physical surface prevents rendering z-fighting.
Tag textures and poses remain independent of the Fusion mesh so their IDs,
sizes, and layout can be changed without exporting the CAD again.

## Include

```xml
<include>
  <uri>model://drone_docking_base_structure</uri>
  <pose>0 0 0 0 0 0</pose>
</include>
```
