# Lyra Robot

Lyra Robot is a ROS 2 Humble workspace for simulated indoor mobile robot navigation and semantic navigation research. It combines an iRobot Create 3 simulation stack, a custom Gazebo home world, SLAM, Nav2 navigation, RGB-D sensing, and an experimental LyraGraph semantic navigation layer.

The project is simulation-first and targets Ubuntu 22.04 with ROS 2 Humble.

## Highlights

- Gazebo home environment for indoor navigation experiments.
- iRobot Create 3 robot simulation with differential drive control.
- Nav2 planning, control, recovery behavior, costmaps, and RViz visualization.
- Cartographer-based 2D SLAM and saved-map navigation support.
- Simulated RGB-D camera with ROS-GZ bridges for color image, depth image, camera info, and point cloud topics.
- RGB-D probe node for validating camera streams.
- LyraGraph MVP packages for semantic keyframe logging, offline object grounding, graph fusion, graph storage, and language-to-Nav2 goal resolution.

## Repository Layout

```text
.
├── config/lyragraph/              # Ontology, perception, and manual region config
├── src/
│   ├── create3_sim/               # Vendored Create 3 simulation packages
│   ├── lyra_home_gz/              # Gazebo home world and world generation helpers
│   ├── lyra_navigation/           # Nav2 launch files, parameters, and RViz config
│   ├── lyra_perception/           # RGB-D camera sanity/probe node
│   ├── lyra_robot/                # Workspace-level robot package
│   ├── lyra_slam/                 # Cartographer, slam_toolbox config, and saved maps
│   ├── lyragraph_bringup/         # Semantic runtime launch wrapper
│   ├── lyragraph_fusion/          # Offline detection fusion into semantic graph nodes
│   ├── lyragraph_keyframe_logger/ # RGB-D keyframe and pose logger
│   ├── lyragraph_msgs/            # Custom graph messages and services
│   ├── lyragraph_nav_bridge/      # Language query to Nav2 goal bridge
│   ├── lyragraph_perception/      # Offline VLM/object detection grounding pipeline
│   └── lyragraph_store/           # Semantic graph store, query service, and markers
├── research.md                    # Research direction and roadmap notes
└── lyra_robot_guide.md            # Learning-oriented ROS 2/Nav2 guide
```

Runtime folders such as `build/`, `install/`, `log/`, and generated LyraGraph data are ignored by git.

## System Architecture

Lyra has two layers:

1. Classical navigation layer
   - Gazebo simulates the home and robot.
   - ROS-GZ bridges expose simulated sensors and control topics.
   - SLAM or a saved occupancy map provides geometry.
   - Nav2 handles global planning, local control, costmaps, recovery, and `/cmd_vel` execution.

2. Semantic navigation layer
   - RGB-D keyframes are sampled during mapping/navigation.
   - Offline perception extracts object labels and image-space detections.
   - Depth and stored camera poses ground detections into the ROS `map` frame.
   - Detections are fused into persistent object nodes.
   - A semantic graph stores objects, regions, relations, confidence, and reachable poses.
   - A language bridge resolves supported text instructions into validated Nav2 goals.

Nav2 remains the execution authority. LyraGraph provides semantic goal selection and graph memory.

## Current Capabilities

### Navigation

- Load a custom furnished home world in Gazebo.
- Spawn and control a Create 3 robot.
- Publish laser scan, odometry, TF, robot state, and simulated Create 3 sensor topics.
- Run Cartographer SLAM.
- Run Nav2 with global and local costmaps.
- Send direct velocity commands through `/cmd_vel`.

### RGB-D Perception

The simulated RGB-D camera publishes:

- `/camera/color/image_raw`
- `/camera/color/camera_info`
- `/camera/depth/image_raw`
- `/camera/depth/points`

The `lyra_perception` probe subscribes to RGB, depth, and camera info, then publishes a compact health summary on:

- `/lyra/perception/rgbd_probe`

### LyraGraph MVP

The semantic navigation MVP includes:

- RGB-D keyframe logging with map-frame robot and camera poses.
- Offline VLM/object-detection entry point using `Qwen2.5-VL-3B-Instruct` as the intended model backend.
- Mock detection mode for testing the graph pipeline without running the model.
- Depth-based 3D grounding into the ROS `map` frame.
- Object fusion by label and distance.
- Manual region assignment using polygons.
- Query service for semantic graph lookup.
- Navigation bridge for a small supported instruction set:
  - `go to the sofa`
  - `go to the chair in the kitchen`
  - `go to the table near the sofa`

## Requirements

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo / Ignition Gazebo 6 stack used by the Create 3 simulation packages
- Nav2
- Cartographer ROS
- slam_toolbox
- ros_gz bridge packages
- OpenCV Python bindings for RGB-D keyframe image writing
- Python packages used by the offline pipeline, including `numpy` and `PyYAML`

Optional for VLM-backed detection:

- PyTorch
- Transformers with Qwen2.5-VL support
- Qwen2.5-VL-3B-Instruct model weights

## Build

From the workspace root:

```bash
cd ~/projects/lyra_robot
conda deactivate  # recommended when running ROS 2 commands
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Core Bringup

Use separate terminals unless noted. In each terminal:

```bash
cd ~/projects/lyra_robot
conda deactivate
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### 1. Launch the home world

```bash
ros2 launch lyra_home_gz _home.launch.xml
```

For headless simulation:

```bash
ros2 launch lyra_home_gz _home.launch.xml headless:=true
```

### 2. Spawn the Create 3 robot

```bash
ros2 launch irobot_create_gz_bringup create3_gz.launch.py
```

### 3. Run SLAM

```bash
ros2 launch lyra_slam carto_slam.launch.py
```

### 4. Run Nav2

```bash
ros2 launch lyra_navigation navigation.launch.py
```

## Quick Validation

Check core ROS graph topics:

```bash
ros2 topic list | grep -E "^/scan$|^/odom$|^/map$|^/tf$|^/cmd_vel$"
```

Check RGB-D camera topics:

```bash
ros2 topic list | grep camera
ros2 topic echo /camera/color/camera_info --once
ros2 topic echo /camera/color/image_raw --field header --once
ros2 topic echo /camera/depth/image_raw --field header --once
```

Run the RGB-D probe:

```bash
ros2 launch lyra_perception rgbd_probe.launch.py
ros2 topic echo /lyra/perception/rgbd_probe --once
```

Send a short manual velocity command:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped \
"{header: {frame_id: 'base_link'}, twist: {linear: {x: 0.2}, angular: {z: 0.0}}}" -r 10
```

Stop it with `Ctrl+C`.

## LyraGraph Workflow

### 1. Log RGB-D keyframes

Run this while the world, robot, TF, and camera topics are active:

```bash
ros2 launch lyragraph_keyframe_logger keyframe_logger.launch.py run_id:=home_test
```

Output is written to:

```text
data/lyragraph_runs/home_test/
```

Each sampled frame stores:

- RGB image
- depth array
- camera intrinsics
- camera pose in `map`
- robot pose in `map`
- timestamp

### 2. Build detections offline

For pipeline testing without the VLM:

```bash
ros2 run lyragraph_perception build_detections \
  --run-dir data/lyragraph_runs/home_test \
  --mock-detections /path/to/mock_detections.jsonl \
  --output data/lyragraph_runs/home_test/detections.jsonl
```

For Qwen-backed detection, run the same command without `--mock-detections` after the model environment is configured. The mock file is optional and is only meant for pipeline tests.

### 3. Fuse detections into a graph

```bash
mkdir -p data/lyragraph_graphs/home_test

ros2 run lyragraph_fusion build_graph \
  --detections data/lyragraph_runs/home_test/detections.jsonl \
  --output data/lyragraph_graphs/home_test/graph.json \
  --run-id home_test
```

### 4. Run the semantic graph runtime

```bash
ros2 launch lyragraph_bringup semantic_runtime.launch.py \
  graph_file:=data/lyragraph_graphs/home_test/graph.json
```

This starts:

- graph store
- graph query service
- semantic navigation bridge
- visualization markers

### 5. Query a semantic goal

```bash
ros2 service call /lyragraph/resolve_semantic_goal lyragraph_msgs/srv/ResolveSemanticGoal \
"{instruction: 'go to the sofa', execute: false}"
```

Set `execute: true` only when Nav2 is active and the goal should be sent to the robot.

## Data Outputs

Generated runtime data is intentionally not committed:

```text
data/lyragraph_runs/
data/lyragraph_graphs/
```

Semantic graph outputs are JSON files designed for inspection, debugging, and repeatable offline testing.

## Development Notes

- Keep ROS commands outside Conda unless a specific ML environment is needed.
- Use the Qwen/ML environment only for offline perception if ROS Python dependencies are not available there.
- Keep generated data, build outputs, local tool files, and model weights out of git.
- The current semantic graph pipeline is an MVP and is intentionally conservative: offline build first, manual regions first, Nav2 execution preserved.

## Roadmap

- Improve object detection reliability and ontology coverage.
- Add better reachable-pose sampling around semantic objects.
- Add richer object-region and object-object relations.
- Add semantic graph visualization in RViz.
- Add semantic costmap layers for constraints such as restricted or avoided regions.
- Evaluate semantic navigation success rate across repeated simulation runs.

## License

See package-level license files where available.
