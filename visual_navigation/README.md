# Visual Navigation

ROS 2 package for the current WildOS visual pipeline, object-search control, research baselines, and diagnostic tools

## Current Pipeline

| File | Responsibility |
|---|---|
| `visual_navigation/wildos/nav.py` | ExploRFM inference, graph scoring, confirmed object masks, and reached evidence |
| `visual_navigation/object_target_fusion.py` | ROS adapter for recursive multi-view target fusion |
| `visual_navigation/object_search_goal_mux.py` | Exploration goal, stable target goal, and completion ownership |
| `visual_navigation/object_detection_filter.py` | Connected-component and temporal detection filtering |
| `visual_navigation/object_reached_evidence.py` | Near-range visual completion evidence |
| `visual_navigation/utils/object_search_utils.py` | `ObjectMaskWithTf` construction and query localization |
| `visual_navigation/utils/paths.py` | Repository resource path resolution |

The pure target filter is implemented in `triangulation3d/target_particle_filter.py`

## Configuration

| Config | Used by |
|---|---|
| `configs/wildos_nav_conf.yaml` | Robot-oriented WildOS defaults |
| `configs/wildos_nav_sim_conf.yaml` | Simulation WildOS defaults |
| `configs/object_search_goal_mux.yaml` | Goal Mux strategy |
| `configs/imgfrontier_nav_conf.yaml` | ImgFrontier baseline |
| `configs/lrn_nav_conf.yaml` | LRN baseline |
| `configs/geofrontier_nav_conf.yaml` | GeoFrontier baseline |

Platform topic and frame overrides live in `graph_construction/configs/topic_profiles.yaml`

## Launch

The full current system is launched from the repository root:

```bash
./scripts/start_wildos_elevation.sh do_object_search:=true
```

Use the component launch only for isolated visual debugging:

```bash
ros2 launch visual_navigation wildos_component.launch.py do_object_search:=true
```

## Research Baselines

The following modules are retained for research comparisons and are not launched by the default elevation pipeline:

- `visual_navigation/lrn/`
- `visual_navigation/imgfrontier_nav/`
- `visual_navigation/geofrontier_nav/`
- `visual_navigation/gps/`
- `visual_navigation/imgfrontier_nav/viz_net.py`

All navigation modules resolve model resources from the repository root. Set `WILDOS_REPO_ROOT` when the source location cannot be inferred from an install tree

## Removed Legacy Paths

The package no longer provides:

- visual ray-based coarse target pose publication
- `explorfm_triangulate`
- standalone ExploRFM triangulation launch and config
- the `explorfm_triangulation` Python package
