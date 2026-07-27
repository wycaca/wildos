<div align="center">

# WildOS: Open-Vocabulary Object Search in the Wild

[Hardik Shah](https://hardik01shah.github.io/)<sup>1,2</sup>,
[Erica Tevere](https://www-robotics.jpl.nasa.gov/who-we-are/people/erica-tevere/)<sup>1</sup>,
[Deegan Atha](https://www.linkedin.com/in/deeganatha/)<sup>1</sup>,
[Marcel Kaufmann](http://www.kaufmann.space/)<sup>1</sup>,
<br>
[Shehryar Khattak](https://www.linkedin.com/in/shehryar-khattak)<sup>1</sup>,
[Manthan Patel](https://manthan99.github.io/)<sup>2</sup>,
[Marco Hutter](https://rsl.ethz.ch/the-lab/people/person-detail.MTIxOTEx.TGlzdC8yNDQxLC0xNDI1MTk1NzM1.html)<sup>2</sup>,
[Jonas Frey](https://jonasfrey96.github.io/)<sup>2,3,4</sup>,
[Patrick Spieler](https://www-robotics.jpl.nasa.gov/who-we-are/people/patrick-spieler/)<sup>1</sup>

<sup>1</sup>Jet Propulsion Laboratory (JPL), NASA &nbsp;&nbsp;
<sup>2</sup>Robotics Systems Lab, ETH Zurich &nbsp;&nbsp;

<sup>3</sup>Stanford University &nbsp;&nbsp;
<sup>4</sup>University of California, Berkeley

[![arXiv](https://img.shields.io/badge/arXiv-2602.19308-b31b1b.svg)](https://arxiv.org/abs/2602.19308)
[![Project Page](https://img.shields.io/badge/Project-Page-blue)](https://leggedrobotics.github.io/wildos/)
[![Videos](https://img.shields.io/badge/Experiment-Videos-red?logo=youtube)](https://www.youtube.com/playlist?list=PLE-BQwvVGf8HjidqjQSML1E4tP20daDNS)
[![Dataset](https://img.shields.io/badge/%F0%9F%A4%97-Dataset-yellow)](https://huggingface.co/datasets/leggedrobotics/wildos)
[![License](https://img.shields.io/badge/License-Apache%202.0-green.svg)](LICENSE)

</div>

<div align="center">
  <img src="assets/teaser.svg" alt="WildOS Teaser" width="90%"/>
</div>

## 📄 Abstract

Autonomous navigation in complex, unstructured outdoor environments requires robots to operate over long ranges without prior maps and limited depth sensing. In such settings, relying solely on geometric frontiers for exploration is often insufficient; the ability to reason semantically about *where to go* and *what is safe to traverse* is crucial for robust, efficient exploration.

This work presents **WildOS**, a unified system for long-range, open-vocabulary object search that combines safe geometric exploration with semantic visual reasoning. WildOS builds a sparse navigation graph to maintain spatial memory, while utilizing a foundation-model-based vision module, **ExploRFM**, to score frontier nodes of the graph. ExploRFM simultaneously predicts traversability, visual frontiers, and object similarity in image space, enabling real-time, onboard semantic navigation tasks. The resulting vision-scored graph enables the robot to explore semantically meaningful directions while ensuring geometric safety.

Furthermore, we introduce a **particle-filter-based method for coarse localization** of the open-vocabulary target query, that estimates candidate goal positions beyond the robot's immediate depth horizon, enabling effective planning toward distant goals. Extensive closed-loop field experiments across diverse off-road and urban terrains demonstrate that WildOS enables robust navigation, significantly outperforming purely geometric and purely vision-based baselines in both efficiency and autonomy.

<br>

## 📁 Repository Structure

```
wildos/
├── nvidia_radio/          # Modified RADIO backbone with NACLIP + SigLIP2 alignment
├── explorfm/              # ExploRFM model (inference): frontiers, traversability, object similarity
├── explorfm_trainer/      # Training pipeline for ExploRFM heads (Lightning + Hydra)
├── visual_navigation/     # ROS 2 navigation: WildOS, baselines (LRN, ImgFrontierNav)
├── triangulation3d/       # Particle-filter-based 3D object triangulation
├── graphnav_planner/      # Graph-based path planner (C++)
├── graphnav_msgs/         # ROS 2 message definitions for navigation graph
├── object_search_msgs/    # ROS 2 message definitions for object search
├── gps_visualization/     # GPS path visualization (ROS 2 C++)
└── ckpts/                 # Model checkpoints
```

Each package has its own README with additional details. See the [Component Overview](#-component-overview) section below.

<br>

## 🧭 Development Documentation Rules

When modifying this repository, keep implementation notes synchronized with the code:

- Add a dated changelog under the relevant package docs for same-day changes
- Put all changes from the same date into the same dated changelog when possible
- Keep changelog entries concise, like release notes, without excessive implementation detail
- When formulas, algorithms, message contracts, or core principles change, update the corresponding principles and implementation documents in the same change
- For `graph_construction`, use `graph_construction/docs/YYYY-MM-DD/` for dated plans, implementation notes, tests, and changelogs

<br>

## 🧪 Current Simulation Notes

The current simulation uses the elevation/2.5D GridMap pipeline as its only geometric backend:

```text
/livox/lidar
  -> pointcloud_axis_adapter
  -> elevation_mapping_cupy
  -> GridMap
  -> graph_construction

/camera/front, /camera/left, /camera/right
  -> visual_navigation / WildOS
  -> /spot1/scored_nav_graph
```

Use the consolidated startup script:

```bash
./scripts/start_wildos_elevation.sh
```

The script launches the mapping backend, graph construction, odometry adapter, camera TF fallback, WildOS visual scoring, target fusion, goal mux, and graph planner. The optional `path_follower_node` is not part of this default chain.

Topic and frame names are selected by profile. Built-in profiles live in `graph_construction/configs/topic_profiles.yaml`:

The default profile is `unity`, so the plain startup command uses `/livox/lidar`, ROS domain 89, and `rmw_zenoh_cpp`.

See `graph_construction/docs/details/topics.md` for the full topic contract and `graph_construction/docs/details/environment.md` for the environment migration checklist.

```bash
WILDOS_TOPIC_PROFILE=isaac ./scripts/start_wildos_elevation.sh
WILDOS_TOPIC_PROFILE=unity ./scripts/start_wildos_elevation.sh
WILDOS_TOPIC_PROFILE=robot ./scripts/start_wildos_elevation.sh
```

Individual launch arguments still override profile values, for example `pointcloud_input_topic:=/custom/lidar`.

<br>

## ⚙️ Installation

### Prerequisites

- **ROS 2 Humble** (current integration environment)
- **Python >= 3.10**
- **[uv](https://docs.astral.sh/uv/getting-started/installation/)** — Python package manager
- **CUDA-capable GPU** (ExploRFM trained on *NVIDIA GeForce RTX 4090*, deployed on *NVIDIA Jetson AGX Orin* GPU)

Install uv:
```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

### 1. Create a Virtual Environment

```bash
uv venv wildos_venv
source wildos_venv/bin/activate
```

### 2. Install Python Dependencies

```bash
uv pip install -r requirements.txt
```

### 3. Install Local Packages

```bash
uv pip install -e ./nvidia_radio
uv pip install -e ./explorfm
```

### 4. Install HuggingFace CLI (for downloading checkpoints)

```bash
uv tool install huggingface_hub[cli]
```

### 5. Build ROS 2 Packages

```bash
# From your colcon workspace (with this repo cloned/symlinked into src/)
colcon build --packages-up-to object_search_msgs triangulation3d visual_navigation graph_construction graphnav_planner
source install/setup.bash
```

> **Note**: WildOS was deployed inside a Docker container during field experiments. The dependencies above can be replicated in a virtual environment for development.

### Docker Compose Deployment

The current repository provides separate GPU images for x86_64 and NVIDIA Jetson AGX Orin:

```bash
docker compose -f compose.x86_64.yaml build
docker compose -f compose.x86_64.yaml up -d

docker compose -f compose.orin.yaml build
docker compose -f compose.orin.yaml up -d
```

See [WildOS dual-architecture Docker deployment](graph_construction/docs/2026-07-16/2026-07-16-wildos-docker-deployment.md) for model prerequisites, JetPack compatibility, topic profiles, GPU validation, and RViz usage.

<br>

## 💾 Checkpoints

Pre-trained head checkpoints are included in `ckpts/`:

| Checkpoint | Description |
|---|---|
| `ckpts/frontier_head.ckpt` | Visual frontier prediction head |
| `ckpts/trav_head.ckpt` | Traversability prediction head |

### Download Backbone & Adaptor Weights

1. **C-RADIOv3-B backbone** — download to `ckpts/`:
   ```bash
   # Download from: https://huggingface.co/nvidia/C-RADIOv3-B/blob/main/c-radio_v3-b_half.pth.tar
   wget -P ckpts/ https://huggingface.co/nvidia/C-RADIOv3-B/resolve/main/c-radio_v3-b_half.pth.tar
   ```

2. **SigLIP2 adaptor** — download to `ckpts/siglip2/`:
   ```bash
   huggingface-cli download google/siglip2-so400m-patch16-naflex --cache-dir ckpts/siglip2
   ```

> **Path configuration**: Navigation nodes resolve `ckpts/` from the repository root. Set `WILDOS_REPO_ROOT` when running from a copied install tree.

### Verify Installation

```bash
python explorfm/explorfm_model.py
```

Expected output:
```
[INFO] Loading SigLIP2 model and processor for version: google/siglip2-so400m-patch16-naflex
[INFO] Using checkpoint path: ckpts/siglip2
Loaded traversability head from ckpts/trav_head.ckpt
Loaded frontier head from ckpts/frontier_head.ckpt
Traversability shape: torch.Size([1, 1, 720, 1280])
Frontiers shape: torch.Size([1, 1, 720, 1280])
Adaptor features shape: torch.Size([1, 1152, 22, 40])
```

<br>

## 🚀 Quick Start: Deployment

### Launch WildOS (Full Pipeline)

```bash
# Launch the current elevation pipeline with open-vocabulary object search
./scripts/start_wildos_elevation.sh do_object_search:=true
```

### Launch Baselines

```bash
# Image Frontier Navigation baseline
ros2 launch visual_navigation imgfrontier_nav_launch.py ns:=spot1 do_object_search:=true

# LRN baseline
ros2 launch visual_navigation lrn_launch.py ns:=spot1 do_object_search:=false
```

### Standalone Tools

```bash
# Launch only WildOS and the current target fusion adapter
ros2 launch visual_navigation wildos_component.launch.py do_object_search:=true

# Launch the optional path-to-goal adapter
ros2 launch graphnav_planner path_follower.launch.py

# Visualize ExploRFM outputs (debugging)
ros2 run visual_navigation viz_net
```

> All experiment videos are available on [YouTube](https://www.youtube.com/playlist?list=PLE-BQwvVGf8HjidqjQSML1E4tP20daDNS).

### Required External Components

The following packages must be running alongside WildOS:

- [**Elevation Mapping CuPy**](https://github.com/leggedrobotics/elevation_mapping_cupy) — GPU based local 2.5D mapping
- [**DLIO**](https://github.com/vectr-ucla/direct_lidar_inertial_odometry) — LiDAR-inertial odometry
- [**Nav2**](https://github.com/ros-navigation/navigation2) — local planning and control
- **Graph Construction** - code will be released in a future update.

<br>

## 🧩 Component Overview

| Package | Description | Details |
|---|---|---|
| [`nvidia_radio/`](nvidia_radio/) | Modified [RADIO](https://github.com/NVlabs/RADIO) backbone with NACLIP + SigLIP2 language alignment | [README](nvidia_radio/README.md) |
| [`explorfm/`](explorfm/) | ExploRFM model — predicts traversability, visual frontiers, and object similarity | [README](explorfm/README.md) |
| [`explorfm_trainer/`](explorfm_trainer/) | Lightning + Hydra training pipeline for ExploRFM heads | [README](explorfm_trainer/README.md) |
| [`visual_navigation/`](visual_navigation/) | ROS 2 navigation: WildOS, target fusion adapter, goal mux, and research baselines | [README](visual_navigation/README.md) |
| [`triangulation3d/`](triangulation3d/) | Recursive target particle filter | [README](triangulation3d/README.md) |
| [`graphnav_planner/`](graphnav_planner/) | C++ graph-based path planner | — |
| [`graphnav_msgs/`](graphnav_msgs/) | ROS 2 message definitions for navigation graph | — |
| [`object_search_msgs/`](object_search_msgs/) | ROS 2 message definitions for object search | — |
| [`gps_visualization/`](gps_visualization/) | GPS path visualization (ROS 2 C++) | — |

<br>

## 📝 Citation

If you find this work useful, please cite:

```bibtex
@misc{shah2026wildosopenvocabularyobjectsearch,
      title={WildOS: Open-Vocabulary Object Search in the Wild}, 
      author={Hardik Shah and Erica Tevere and Deegan Atha and Marcel Kaufmann and Shehryar Khattak and Manthan Patel and Marco Hutter and Jonas Frey and Patrick Spieler},
      year={2026},
      eprint={2602.19308},
      archivePrefix={arXiv},
      primaryClass={cs.RO},
      url={https://arxiv.org/abs/2602.19308}, 
}
```

<br>

## 🙏 Acknowledgements

We thank the authors of the following works for open-sourcing their code:

- [NVIDIA RADIO](https://github.com/NVlabs/RADIO)
- [RayFronts](https://github.com/RayFronts/RayFronts)
- [NACLIP](https://github.com/sinahmr/NACLIP)
- [Lightning-Hydra-Template](https://github.com/ashleve/lightning-hydra-template)

We also thank the authors of [LRN](https://arxiv.org/abs/2504.13149) for sharing their code, which was helpful in setting up the baseline.

<br>

## 📜 License

This project is released under the [Apache 2.0 License](LICENSE).
