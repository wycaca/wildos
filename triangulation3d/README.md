# Triangulation3D Target Particle Filter

This ROS 2 Python package contains the pure recursive particle filter used by WildOS target fusion

## Current Scope

The package intentionally keeps one implementation:

```text
triangulation3d/target_particle_filter.py
```

`TargetParticleFilter`:

1. Initializes a fixed particle set from one confirmed camera mask
2. Reweights the same particles with distinct later views
3. Rejects repeated or inconsistent observations
4. Produces a tracking estimate after sufficient visual support
5. Produces a stable visual estimate after convergence
6. Accepts consistent LiDAR support as optional near-range refinement

Multi-view vision is the primary method. LiDAR support is not required for a visual target estimate

## ROS Integration

The ROS adapter is:

```text
visual_navigation/visual_navigation/object_target_fusion.py
```

It consumes `ObjectMaskWithTf`, updates the particle filter, and publishes `TargetEstimate` plus visualization data

The package itself does not install a console script. Start target fusion through the current integrated launch or the visual component launch:

```bash
./scripts/start_wildos_elevation.sh do_object_search:=true
ros2 launch visual_navigation wildos_component.launch.py do_object_search:=true
```

## Tests

```bash
python3 -m pytest triangulation3d/test/test_target_particle_filter.py
```
