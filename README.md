# Lyra Robot

Lyra is a ROS 2 mobile robot workspace centered on a simulated home-assistant robot built on the iRobot Create 3 platform. The repository combines a custom home environment, SLAM, Nav2-based navigation, and the Create 3 Gazebo simulation stack into one workspace.

The original work in this repository was put together by Chirag Makwana. In its current form, Lyra is a classical ROS 2 navigation stack for a homemaker robot.

## What Chirag Built

The current workspace already covers the main pieces needed for indoor robot navigation:

- A custom home simulation environment in Gazebo.
- A Create 3 robot simulation stack with robot description, control, sensors, and docking support.
- 2D SLAM support using Cartographer and a separate lifelong mapping path using `slam_toolbox`.
- A Nav2 bringup with planner, controller, behavior server, waypoint follower, collision monitor, map server, and docking server.
- A saved occupancy map for localization and navigation in the home environment.

In short, this repo is not just a single demo launch file. It is a full ROS 2 workspace for simulated indoor mobile navigation.

## Workspace Layout

This workspace contains both Lyra-specific packages and the Create 3 simulation packages it depends on.

### Lyra Packages

- `lyra_home_gz`
  Builds the custom home world from a building description and generates Gazebo assets, navigation graphs, and crowd simulation artifacts.

- `lyra_slam`
  Contains SLAM launch files, mapping configs, RViz configs, and the saved map used by navigation.

- `lyra_navigation`
  Contains the Nav2 launch files, RViz config, and navigation parameters for planning and control.

- `lyra_robot`
  Package shell for the overall robot workspace.

- `lyra_behavior`
  Placeholder package for higher-level task or behavior logic.

### Create 3 Simulation Packages

The `create3_sim/` subtree vendors the iRobot Create 3 simulation stack and provides:

- robot description and URDF/Xacro
- Gazebo bridging
- differential drive control
- robot state publishers
- Create 3 sensor publishers and internal support nodes
- docking-related simulation support

## Current Architecture

At the moment, Lyra uses a standard geometric navigation pipeline:

1. The custom home world is loaded in Gazebo.
2. The Create 3 robot is spawned into the environment.
3. Laser-based SLAM or saved-map navigation is used for map-aware motion.
4. Nav2 handles path planning, local control, recovery behavior, and command velocity output.

The current stack is classical ROS 2 navigation, not yet a vision-language navigation system.

## Key Capabilities Today

- Simulated indoor home environment.
- Differential-drive mobile robot simulation.
- Laser scan based mapping and navigation.
- Nav2 global planning and local control.
- Collision monitoring.
- Docking server configuration.
- RViz visualization for SLAM and navigation.

## What Is In The Home Environment

The home environment is defined from a building description in `lyra_home_gz/floorplans/home.building.yaml`. It includes a furnished apartment-style layout with objects such as:

- sofa
- tables and chairs
- beds
- sinks
- toilets
- fridge
- desks
- TV stand
- storage cabinets

## Navigation Stack

The navigation setup in `lyra_navigation` is based on Nav2 and currently includes:

- `planner_server`
- `controller_server`
- `smoother_server`
- `behavior_server`
- `bt_navigator`
- `waypoint_follower`
- `velocity_smoother`
- `collision_monitor`
- `map_server`
- `map_saver`
- `opennav_docking`

The current controller is MPPI-based, and the planner is NavFn-based. The local and global costmaps are built around a laser scan input on `/scan`.

## SLAM Stack

The repository contains two mapping paths:

- `cartographer_ros` for 2D SLAM
- `slam_toolbox` lifelong mapping configuration

There is also a saved map in `lyra_slam/maps/` that can be used directly with navigation.

## Sensors Exposed In Simulation

The Create 3 simulation stack currently exposes or uses:

- lidar / laser scan
- IMU
- bumper contact
- cliff sensors
- IR intensity sensors
- wheel and robot state information
- docking-related interfaces

There is no RGB camera pipeline integrated yet.

## Prerequisites

This repository assumes a ROS 2 environment with Gazebo and Nav2-related packages available. From the code and launch files, the workspace depends on components in the following areas:

- ROS 2
- Gazebo Harmonic / `ros_gz`
- Nav2
- Cartographer
- `slam_toolbox`
- RMF building map tools
- OpenNav docking
- iRobot Create 3 ROS interfaces

The exact package installation may vary by ROS 2 distribution and host machine setup. This repository is best treated as a source workspace to be built inside an already-prepared ROS 2 simulation environment.

## Build

From the workspace root:

```bash
colcon build --symlink-install
source install/setup.bash
```

If dependencies are missing, install them first with your normal ROS 2 workflow, for example `rosdep` or distribution-specific package installation.

## Typical Bringup Flow

### 1. Launch the home simulation

```bash
ros2 launch lyra_home_gz _home.launch.xml
```

### 2. Spawn the Create 3 robot stack

```bash
ros2 launch irobot_create_gz_bringup create3_gz.launch.py
```

### 3. Run navigation

```bash
ros2 launch lyra_navigation navigation.launch.py
```

### 4. Run SLAM if mapping is needed

```bash
ros2 launch lyra_slam carto_slam.launch.py
```

### 5. Optional lifelong mapping path

```bash
ros2 launch lyra_slam lifelong_mapping.launch.py
```

## Notes On Current State

- `lyra_robot` and `lyra_behavior` are mostly scaffolding packages at this stage.
- The real implemented work is in `lyra_home_gz`, `lyra_slam`, `lyra_navigation`, and `create3_sim`.
- The current perception pipeline is laser-first. Camera-based understanding is not integrated yet.

## Attribution

Original Lyra workspace and package structure by Chirag Makwana.
