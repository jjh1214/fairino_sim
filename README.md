# Fairino_sim
[![license - MIT](https://img.shields.io/:license-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![ROS2 Jazzy](https://img.shields.io/badge/ROS2-Jazzy-purple.svg)](https://index.ros.org/doc/ros2/Releases/)

Simulation of FAIRINO cobots with ROS 2 Jazzy, Gazebo Harmonic and MoveIt 2.

# Packages
| Package | Contents |
|---|---|
| `fairino_sim` | Gazebo and MoveIt 2 launch files, robot and tool description |
| `fairino_workcell` | Work cell tools: CAD import, cell scene, recipes, path validation (in progress) |
| `fairino_workcell_msgs` | ROS 2 interfaces for `fairino_workcell` (in progress) |

Robot models come from [FAIR-INNOVATION/frcobot_ros2](https://github.com/FAIR-INNOVATION/frcobot_ros2).
Supported models: `fairino3_v6`, `fairino5_v6`, `fairino10_v6`, `fairino16_v6`, `fairino20_v6`, `fairino30_v6`.
*NOTE : fairino3_mt_v6 is not supported.*

# Build
### *This Package is implemented at ROS2-Jazzy (Ubuntu 24.04).*
Install ROS 2 Jazzy (`ros-jazzy-desktop`) first, then Gazebo Harmonic, ros2_control and MoveIt 2:
```
sudo apt install ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-ros2controlcli ros-jazzy-moveit ros-jazzy-xacro
```

Build the workspace (I recommend `~/fairino_ws/src`):
```
mkdir -p ~/fairino_ws/src
cd ~/fairino_ws/src
git clone https://github.com/FAIR-INNOVATION/frcobot_ros2.git
git clone https://github.com/jjh1214/fairino_sim.git

cd ~/fairino_ws
colcon build --packages-up-to fairino_workcell
source install/setup.bash
```
`--packages-up-to fairino_workcell` builds the three packages of this repository and the parts of
frcobot_ros2 they need, and skips the real-robot drivers (`fairino_hardware*`).
A plain `colcon build` also works and builds everything in `src`.

If you want: `echo 'source ~/fairino_ws/install/setup.bash' >> ~/.bashrc`

# GZ Launch
## Default Model : fairino5_v6
```
ros2 launch fairino_sim fairino_gz.launch.py
```
## Other Models
```
ros2 launch fairino_sim fairino_gz.launch.py robot_name:=fairino3_v6
ros2 launch fairino_sim fairino_gz.launch.py robot_name:=fairino10_v6
ros2 launch fairino_sim fairino_gz.launch.py robot_name:=fairino16_v6
ros2 launch fairino_sim fairino_gz.launch.py robot_name:=fairino20_v6
ros2 launch fairino_sim fairino_gz.launch.py robot_name:=fairino30_v6
```

# Move Model
```
ros2 topic pub -1 /frcobot_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [2.0, -1.571, -1.571, -1.571, -1.571, -1.571]}"

ros2 topic pub -1 /frcobot_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```
# Move example

[Screencast from 2025-07-31 09-28-38.webm](https://github.com/user-attachments/assets/c4b7654f-bd6c-4130-88a2-4ee26a23a3ae)

# GZ with Moveit Launch
## Default Model : fairino5_v6
```
ros2 launch fairino_sim fairino_gz_moveit2.launch.py
```
## Other Models
```
ros2 launch fairino_sim fairino_gz_moveit2.launch.py robot_name:=fairino3_v6
ros2 launch fairino_sim fairino_gz_moveit2.launch.py robot_name:=fairino10_v6
ros2 launch fairino_sim fairino_gz_moveit2.launch.py robot_name:=fairino16_v6
ros2 launch fairino_sim fairino_gz_moveit2.launch.py robot_name:=fairino20_v6
ros2 launch fairino_sim fairino_gz_moveit2.launch.py robot_name:=fairino30_v6
```

# Move example

[Screencast from 2025-07-31 09-30-59.webm](https://github.com/user-attachments/assets/f1d53846-4953-4821-be4b-f542406576af)

# Launch arguments
Both launch files accept these arguments.

| Argument | Default | Description |
|---|---|---|
| `robot_name` | `fairino5_v6` | Robot model |
| `tool` | `none` | `none`, a tool name under `fairino_sim/tools` (e.g. `example_gripper`), or the path of a tool YAML file |
| `base_xyz` | `"0 0 0"` | Position of `base_link` in the `world` frame, in metres |
| `base_rpy` | `"0 0 0"` | Roll, pitch and yaw of `base_link` in the `world` frame, in radians |
| `initial_positions` | `"0 -1.571 0 0 0 0"` | Initial angles of j1 to j6, in radians |
| `use_sim_time` | `true` | Use the Gazebo `/clock` |
| `use_rviz` | `true` | Start RViz (`fairino_gz_moveit2.launch.py` only) |

Example: a robot on a 0.5 m pedestal with the example gripper
```
ros2 launch fairino_sim fairino_gz_moveit2.launch.py tool:=example_gripper base_xyz:="0 0 0.5"
```

# Tools
Gazebo, MoveIt and RViz all use the same robot description, which adds these frames:

| Frame | Meaning |
|---|---|
| `flange` | Tool mounting face at the end of `wrist3_link` |
| `tool_link` | The tool body (only when a tool is given) |
| `tcp` | Tool centre point (on the flange when there is no tool) |

A tool is described by a YAML file, so changing the tool needs no code or URDF changes.
Mesh paths are relative to the YAML file; STL, OBJ, DAE and GLB meshes are supported.
```yaml
name: my_gripper
mesh:
  visual: my_gripper.stl       # STL, OBJ, DAE or GLB
  collision: my_gripper.stl    # optional, defaults to visual
  units: mm                    # required: m or mm (STL and OBJ files carry no units)
mount:                         # tool origin relative to the flange
  xyz: [0.0, 0.0, 0.0]         # metres
  rpy: [0.0, 0.0, 0.0]         # radians (or rpy_deg: [...] in degrees)
tcp:                           # TCP relative to the tool origin
  xyz: [0.0, 0.0, 0.120]
  rpy: [0.0, 0.0, 0.0]
mass: 0.8                      # kg, used by Gazebo
touch_links: [wrist3_link]     # robot links the tool may touch without a collision
```
```
ros2 launch fairino_sim fairino_gz_moveit2.launch.py tool:=/path/to/my_gripper.yaml
```
