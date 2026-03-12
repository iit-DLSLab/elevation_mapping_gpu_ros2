# Elevation Mapping CuPy (ROS2)

![ros2 ci](https://github.com/leggedrobotics/elevation_mapping_cupy/actions/workflows/jazzy-docker-tests.yml/badge.svg)

[Documentation](https://leggedrobotics.github.io/elevation_mapping_cupy/)

## Branch Selection

- For ROS2 Jazzy, use the [`ros2`](https://github.com/leggedrobotics/elevation_mapping_cupy/tree/ros2) branch.
- The `main` branch is the legacy ROS1 line.
- Current ROS2/Jazzy release: `v2.1.0`.

## Overview

GPU-accelerated elevation mapping for robotic navigation and locomotion. This package provides real-time terrain mapping using CuPy for GPU acceleration, integrating with ROS2 for point cloud registration, ray casting, and multi-modal sensor fusion.

![screenshot](docs/media/main_repo.png)
![screenshot](docs/media/main_mem.png)

## Branch Information

| Branch | Status | Description |
|--------|--------|-------------|
| `ros2` (this branch) | **Actively maintained** | ROS2 Jazzy branch, Python bindings only, maintained by Lorenzo Terenzi |
| `ros2_cpp` | Work in progress | C++ bindings (external contribution, not currently running) |
| `main` | Legacy | ROS1 version |

## Key Features

- **Height Drift Compensation**: Tackles state estimation drifts that can create mapping artifacts, ensuring more accurate terrain representation.

- **Visibility Cleanup and Artifact Removal**: Raycasting methods and an exclusion zone feature remove virtual artifacts and correctly interpret overhanging obstacles.

- **Learning-based Traversability Filter**: Assesses terrain traversability using local geometry, improving path planning and navigation.

- **Multi-Modal Elevation Map (MEM) Framework**: Seamless integration of geometry, semantics, and RGB information for multi-modal robotic perception.

- **Semantic Layer Support**: Fuse semantic segmentation data from external sources (point clouds or images) into the elevation map using various fusion algorithms.

- **GPU-Enhanced Efficiency**: Rapid processing of large data structures using CuPy, crucial for real-time applications.

## Requirements

- **ROS2**: Jazzy (recommended)
- **CUDA**: 12.x
- **Python**: 3.10+
- **GPU**: NVIDIA GPU with CUDA support

## Installation

### Clone the Repository

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone -b ros2 https://github.com/leggedrobotics/elevation_mapping_cupy.git
```

### Install Dependencies

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### Build

```bash
cd ~/ros2_ws
colcon build --packages-up-to semantic_sensor elevation_mapping_cupy --symlink-install
source install/setup.bash
```

### Docker (Alternative)

A Docker setup is available for easy deployment:

```bash
cd ~/ros2_ws/src/elevation_mapping_cupy/docker
./run.sh
```

## Usage

### TurtleBot3 Simulation Example

![Elevation map examples](docs/media/turtlebot.png)

```bash
# Terminal 1: Launch elevation mapping with TurtleBot3 simulation
export TURTLEBOT3_MODEL=waffle
ros2 launch elevation_mapping_cupy elevation_mapping_turtle.launch.py
```

```bash
# Terminal 2: Control robot with keyboard
export TURTLEBOT3_MODEL=waffle
ros2 run turtlebot3_teleop teleop_keyboard
```

Use `w`, `a`, `s`, `d`, `x` keys to control the robot.

### Custom Robot Configuration

```bash
ros2 launch elevation_mapping_cupy elevation_mapping.launch.py robot_config:=<your_config.yaml>
```

Available example configurations in `config/setups/`:
- `turtle_bot/` - TurtleBot3 configurations
- `anymal/` - ANYmal robot configurations
- `menzi/` - Menzi Muck configurations

## Testing

```bash
cd ~/ros2_ws
colcon test --packages-select elevation_mapping_cupy --event-handlers console_direct+
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest \
  src/elevation_mapping_cupy/sensor_processing/semantic_sensor/test -q
```

## Services

The node provides three services for map management:

| Service | Type | Description |
|---------|------|-------------|
| `/elevation_mapping_cupy/save_map` | `grid_map_msgs/srv/ProcessFile` | Save map to rosbag2 file |
| `/elevation_mapping_cupy/load_map` | `grid_map_msgs/srv/ProcessFile` | Load map from rosbag2 file |
| `/elevation_mapping_cupy/masked_replace` | `grid_map_msgs/srv/SetGridMap` | Replace map regions using a mask |

### Service Examples

```bash
# Save current map
ros2 service call /elevation_mapping_cupy/save_map grid_map_msgs/srv/ProcessFile \
  "{file_path: '/tmp/elevation_map', topic_name: ''}"

# Load saved map
ros2 service call /elevation_mapping_cupy/load_map grid_map_msgs/srv/ProcessFile \
  "{file_path: '/tmp/elevation_map', topic_name: ''}"
```

## Configuration

Configuration is done through YAML files. Key configuration areas:

- **Core parameters**: `config/core/core_param.yaml` - Map resolution, sensor noise, variance settings
- **Plugin configuration**: `config/core/plugin_config.yaml` - Post-processing plugins
- **Robot setups**: `config/setups/<robot>/` - Robot-specific configurations

### Subscribers

Configure point cloud and image inputs:

```yaml
subscribers:
  front_cam:
    topic_name: '/camera/depth/points'
    data_type: 'pointcloud'
    channels: ['rgb']
```

### Publishers

Configure output map topics:

```yaml
publishers:
  elevation_map:
    layers: ['elevation', 'traversability', 'variance']
    basic_layers: ['elevation']
    fps: 5.0
```

## Plugins

Available post-processing plugins:

- `min_filter` / `max_filter` - Morphological operations
- `smooth_filter` - Smoothing filter
- `inpainting` - Fill missing values
- `erosion` - Morphological erosion
- `robot_centric_elevation` - Robot-centric perspective
- `semantic_filter` - Semantic class visualization
- `semantic_traversability` - Semantic-aware traversability
- `features_pca` - PCA feature visualization

## Citing

If you use Elevation Mapping CuPy, please cite:

**[Elevation Mapping for Locomotion and Navigation using GPU](https://arxiv.org/abs/2204.12876)**

Takahiro Miki, Lorenz Wellhausen, Ruben Grandia, Fabian Jenelten, Timon Homberger, Marco Hutter

```bibtex
@inproceedings{miki2022elevation,
  title={Elevation mapping for locomotion and navigation using gpu},
  author={Miki, Takahiro and Wellhausen, Lorenz and Grandia, Ruben and Jenelten, Fabian and Homberger, Timon and Hutter, Marco},
  booktitle={2022 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={2273--2280},
  year={2022},
  organization={IEEE}
}
```

If you use multi-modal features (color or semantic layers), please also cite:

**[MEM: Multi-Modal Elevation Mapping for Robotics and Learning](https://arxiv.org/abs/2309.16818v1)**

Gian Erni, Jonas Frey, Takahiro Miki, Matias Mattamala, Marco Hutter

```bibtex
@inproceedings{erni2023mem,
  title={MEM: Multi-Modal Elevation Mapping for Robotics and Learning},
  author={Erni, Gian and Frey, Jonas and Miki, Takahiro and Mattamala, Matias and Hutter, Marco},
  booktitle={2023 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={11011--11018},
  year={2023},
  organization={IEEE}
}
```

## Contributing

Contributions are welcome! The semantic fusion infrastructure is available and working - contributions for specific research use cases are appreciated.

## License

MIT License - see [LICENSE](LICENSE) for details.
