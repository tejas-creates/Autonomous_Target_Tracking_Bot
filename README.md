# Autonomous Target Tracking Bot

An Ackermann AGV follows a target AGV in Gazebo. It sees a checkerboard on the target's back with its
camera, keeps a 0.5 m gap, and avoids static and moving obstacles that it detects with its LiDAR. An
NMPC controller (CasADi + IPOPT) does the steering and speed control.

- **Target_AGV** drives a fixed route (`docs/trajectory.csv`). It ignores the obstacles.
- **AGV** follows it using the camera (checkerboard pose) and the LiDAR (obstacle circles).
- **20 moving obstacles** (small garden robots) cross the route or wander at random.
- **7 static obstacles** (pots, planters, a stump) stand inside a 12 × 8 m area enclosed by a hedge.

## Requirements

| | Version |
|---|---|
| OS | Ubuntu 24.04 |
| ROS 2 | Jazzy (`ros-jazzy-desktop`, which includes RViz) |
| Gazebo | Harmonic (installed with `ros-jazzy-ros-gz`) |
| Python | 3.12 (system) + a virtual environment for CasADi |

Install ROS 2 Jazzy by following the [official guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html),
then install the rest:

```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-cv-bridge \
    python3-opencv python3-numpy python3-venv python3-colcon-common-extensions \
    python3-gz-transport13 python3-gz-msgs10
```

## Setup

### 1. Clone

```bash
git clone <repo-url>
cd <repo-folder>
```

### 2. Create the Python virtual environment (for the NMPC)

The NMPC needs **CasADi**, which is not available from apt. Install it in a virtual environment at
**`ros_ws/.venv_nmpc`**. Use this exact path: the launch file runs the NMPC node with
`ros_ws/.venv_nmpc/bin/python`.

```bash
cd ros_ws
python3 -m venv --system-site-packages .venv_nmpc
.venv_nmpc/bin/pip install -r requirements.txt
```

`--system-site-packages` is required. Without it the venv cannot see ROS (`rclpy`), OpenCV,
NumPy or colcon.

### 3. Build

Build from inside `ros_ws`, with ROS and the venv sourced:

```bash
source /opt/ros/jazzy/setup.bash
source .venv_nmpc/bin/activate
python -m colcon build
deactivate
```

You only need to build again after changing code in `ros_ws/src`. Files in `ros_ws/config` and
`ros_ws/launch` are read from the source folder, so changes there take effect on the next launch.

## Run

The script starts everything (Gazebo world, robots, and all ROS nodes) with the default options:

```bash
bash scripts/complete_system.sh
```

To choose options, start the two launch files yourself, each in its own terminal, from the
repository root:

```bash
# Terminal 1: Gazebo world, AGV and Target_AGV
source /opt/ros/jazzy/setup.bash && source ros_ws/install/setup.bash
ros2 launch simulation/launch/world.launch.py world:=garden

# Terminal 2: bridges, target driver, camera, LiDAR, NMPC, moving obstacles
source /opt/ros/jazzy/setup.bash && source ros_ws/install/setup.bash
ros2 launch ros_ws/launch/simulation.launch.py obstacle_motion:=crossing obstacle_seed:=7
```

Start terminal 1 first and wait until Gazebo is open.

## Launch options

### `simulation/launch/world.launch.py`: world and robots

| Argument | Values | Default | Effect |
|---|---|---|---|
| `world` | `garden`, `plain` | `garden` | `garden`: realistic back yard (`simulation/garden_world.sdf`). `plain`: simple oval arena (`simulation/simulation_world.sdf`). Both have the same obstacles at the same places. |

### `ros_ws/launch/simulation.launch.py`: ROS nodes

| Argument | Values | Default | Effect |
|---|---|---|---|
| `random_obstacles` | `true`, `false` | `true` | Start the controller that moves the 20 moving obstacles. With `false` they stand still. |
| `obstacle_motion` | `random`, `crossing` | `random` | How the moving obstacles move (see below). |
| `obstacle_seed` | any integer | `-1` | Random seed for the obstacle motion. `-1` gives different motion every run; any other number repeats the same run. |

**`obstacle_motion`**
- `random`: each obstacle drives a straight segment, then picks the next one within ±45° of its
  heading. If the way is blocked (wall or pot) or it is about to hit another obstacle, it drives to a
  random free point in the garden instead.
- `crossing`: each obstacle shuttles back and forth on a segment that crosses the target's route.
  The crossings are spread evenly along the route. The target drives through this traffic, and the
  AGV has to find its own way through. Speeds are redrawn on every pass (0.10–0.30 m/s).

Examples:

```bash
ros2 launch simulation/launch/world.launch.py world:=plain
ros2 launch ros_ws/launch/simulation.launch.py obstacle_motion:=crossing
ros2 launch ros_ws/launch/simulation.launch.py obstacle_seed:=42           # repeatable run
ros2 launch ros_ws/launch/simulation.launch.py random_obstacles:=false     # obstacles stand still
```

To change a default permanently, edit `default_value` of that argument in the launch file.
`bash scripts/complete_system.sh` always uses the defaults.

## Tuning

| File | What |
|---|---|
| `ros_ws/config/nmpc.yaml` | NMPC settings: speeds, following distance (`desired_distance`), obstacle weights, escape mode. All other defaults are in `ros_ws/src/nmpc/nmpc/nmpc_node.py` (`DEFAULTS`). |
| `ros_ws/config/communication_bridges.yaml` | Gazebo ↔ ROS topic bridges |
| `docs/trajectory.csv` | Route driven by the Target_AGV (also used to place the crossing segments) |

The settings of each package are described in [`CLAUDE.md`](CLAUDE.md), the change log.

## Checking that it works

```bash
ros2 topic echo /obstacles            # detected obstacles (moving / static / wall)
ros2 topic echo /target/relative_pose # checkerboard pose seen by the camera
```

In RViz (`rviz2`), set **Fixed Frame** to `odom` and add:
- `/obstacles/markers` (MarkerArray): red = moving, blue = static, grey = wall
- `/nmpc/path` (Path): the planned path of the AGV

## Packages (`ros_ws/src`)

| Package | Node | What it does |
|---|---|---|
| `camera_module` | `camera_module` | Finds the checkerboard and publishes the target's pose (`/target/relative_pose`) |
| `lidar` | `obstacle_detector` | Turns LiDAR scans into obstacle circles and estimates their velocity (`/obstacles`) |
| `obstacle_msgs` | – | `Obstacle` and `ObstacleArray` messages |
| `nmpc` | `nmpc_controller` | Following + obstacle avoidance controller (`/AGV/cmd_vel`) |
| `waypoint_controller` | `waypoint_controller` | Drives the Target_AGV along `docs/trajectory.csv` |
| `random_obstacles` | `random_obstacle_controller` | Moves the 20 moving obstacles (`random` / `crossing`) |
| `dynamic_obstacle_controller` | `obstacle_controller` | Older obstacle mover. Not started by the launch files. |

## Repository layout

```
ros_ws/
  src/                 ROS 2 packages (above)
  config/              nmpc.yaml, communication_bridges.yaml
  launch/              simulation.launch.py (ROS nodes)
  requirements.txt     Python packages for .venv_nmpc
simulation/
  launch/world.launch.py   Gazebo world + AGV + Target_AGV
  garden_world.sdf         garden world (generated, see below)
  simulation_world.sdf     plain world
  Target_AGV.sdf           target robot
  URDF_with_meshes/        AGV model (URDF + STL meshes)
  garden/                  garden world generator and its assets
docs/                  trajectory.csv, garden world notes
scripts/complete_system.sh   starts everything
```

## Garden world

`simulation/garden_world.sdf` is generated. To change it, edit `simulation/garden/build_garden_world.py`
rather than the `.sdf`, then run:

```bash
python3 simulation/garden/build_garden_world.py
```

The models and textures from outside sources are in `simulation/garden/assets/external`. They are all
CC0 (public domain); sources are listed in `simulation/garden/assets/external/CREDITS.md`. To download
them again, run `python3 simulation/garden/fetch_assets.py`.
