# Fairino_sim
[![license - MIT](https://img.shields.io/:license-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![ROS2 Jazzy](https://img.shields.io/badge/ROS2-Jazzy-purple.svg)](https://index.ros.org/doc/ros2/Releases/)

Simulation of FAIRINO cobots with ROS 2 Jazzy, Gazebo Harmonic and MoveIt 2.

# Packages
| Package | Contents |
|---|---|
| `fairino_sim` | Gazebo and MoveIt 2 launch files, robot and tool description |
| `fairino_workcell` | Work cell around the robot: CAD import, cell file, scene in Gazebo / MoveIt / TF, Python recipes with path validation |
| `fairino_workcell_msgs` | ROS 2 interfaces of the work cell: move action, gripper and IO services, step reports |

Robot models come from [FAIR-INNOVATION/frcobot_ros2](https://github.com/FAIR-INNOVATION/frcobot_ros2).
Supported models: `fairino3_v6`, `fairino5_v6`, `fairino10_v6`, `fairino16_v6`, `fairino20_v6`, `fairino30_v6`.
*NOTE : fairino3_mt_v6 is not supported.*

# Build
### *This Package is implemented at ROS2-Jazzy (Ubuntu 24.04).*
Install ROS 2 Jazzy (`ros-jazzy-desktop`) first, then Gazebo Harmonic, ros2_control and MoveIt 2:
```
sudo apt install ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-ros2controlcli ros-jazzy-moveit ros-jazzy-moveit-py ros-jazzy-xacro
```
`ros-jazzy-moveit-py` is needed by the work cell executor (path checks); `ros-jazzy-moveit` does
not install it.

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

# Work cell
`fairino_workcell` builds a work cell around the robot from one cell file. Equipment imported from
CAD (STEP, STL, OBJ) appears in Gazebo, in the MoveIt planning scene (collision checking) and as
TF frames. Recipes written in Python move the robot through the cell; every step is checked for
reachability, joint limits and collisions before it runs (see [Recipes](#recipes)).

## Example cell
```
ros2 launch fairino_workcell fairino_workcell.launch.py
```
A fairino5_v6 with the example gripper on a pedestal, next to a belt conveyor (with a part at its
stop position), a table and a safety fence.

| Argument | Default | Description |
|---|---|---|
| `cell` | example conveyor cell | Path of the cell file |
| `use_moveit` | `true` | Plan with MoveIt 2; `false` uses the joint position controller |
| `use_rviz` | `true` | Start RViz (with `use_moveit`) |
| `use_sim_time` | `true` | Use the Gazebo `/clock` |

After editing the cell file, update the running simulation without restarting it:
```
ros2 service call /workcell/reload std_srvs/srv/Trigger
```

## Cell file
```yaml
cell: my_cell
robot:
  model: fairino5_v6
  base: {xyz: [0.0, 0.0, 0.75]}                 # pose of base_link in the world frame
  initial_positions_deg: [0, -90, 0, 0, 0, 0]   # or initial_positions in radians
  tool: example_gripper                         # tool name, or tool YAML path relative to this file
objects:
  - name: conveyor                              # letters, digits and _
    visual: meshes/conveyor/visual.glb          # GLB, DAE, OBJ or STL, relative to this file
    collision: meshes/conveyor/collision.stl    # STL (may be omitted when visual is an STL)
    units: m                                    # required for meshes: m or mm
    pose: {xyz: [0.0, -0.6, 0.0], rpy_deg: [0, 0, 90]}   # in the world frame; or rpy in radians
    color: [0.36, 0.40, 0.45, 1.0]              # optional
    frames:                                     # optional named frames on the object
      station: {xyz: [0.0, 0.0, 0.80]}
  - name: part
    visual: meshes/part/visual.glb
    units: m
    pose: {xyz: [0.0, -0.6, 0.80]}
    movable: true                               # recipes can grip and move it
    frames:
      grasp: {xyz: [0.0, 0.0, 0.03]}            # where the TCP must be to grip it
  - name: pedestal
    box: [0.30, 0.30, 0.75]                     # or cylinder: {radius: r, length: l}, in metres,
    pose: {xyz: [0.0, 0.0, 0.375]}              # centred on the pose
    touch_links: [base_link]                    # robot links allowed to touch this object
```
Each object is a TF frame named after it, and each named frame is `<object>/<frame>`
(for example `conveyor/station`), also available as a MoveIt subframe. Movable objects follow the
gripper when a recipe grips them; the other objects stay where the cell file puts them.
In Gazebo the objects are visual only: the executor checks collisions itself, so Gazebo physics
never stops the robot at a contact that a step allows (fingers around a part, a part set down).

## Importing CAD files
`import_cad` converts STEP, STL and OBJ files into a visual mesh (GLB) and a simplified collision
mesh (STL), both in metres. STEP files carry their units; give `--units` for STL and OBJ files.
It needs a few Python packages; install them once in a virtual environment:
```
python3 -m venv ~/fairino_cad_venv --system-site-packages
~/fairino_cad_venv/bin/pip install -r ~/fairino_ws/src/fairino_sim/fairino_workcell/requirements-cad.txt
```
Then, from the directory of the cell file, with the workspace sourced:
```
~/fairino_cad_venv/bin/python -m fairino_workcell.cad_import cad/conveyor.step --out meshes
~/fairino_cad_venv/bin/python -m fairino_workcell.cad_import cad/table.stl cad/fence.obj --units mm --out meshes
```
It prints the entries to paste into the cell file. Options: `--origin keep|center|bottom`
(default: keep the CAD origin), `--collision mesh|convex|box` and `--max-faces N`
(triangle budget of the collision mesh, default 3000).

# Recipes
A recipe is a Python function that receives the cell. Steps take joint angles (degrees, as on the
teach pendant) or TCP poses in a cell frame:
```python
HOME = [-45, -90, -90, -90, 90, 0]

def recipe(cell):
    cell.move_joints(HOME, name='home')
    cell.move_to('part/grasp', xyz=(0, 0, 0.15), rpy_deg=(180, 0, 180), name='above part')
    cell.move_to('part/grasp', rpy_deg=(180, 0, 180), speed=0.1, allow_contact=['part'], name='pick')
    cell.grip(name='grip')
    ...
```
The complete example is [`examples/conveyor_cell/pick_and_place.py`](fairino_workcell/examples/conveyor_cell/pick_and_place.py):
it picks the part at the conveyor station and places it on the table. Start the cell:
```
ros2 launch fairino_workcell fairino_workcell.launch.py
```
Then, in another terminal:
```
cd ~/fairino_ws/src/fairino_sim/fairino_workcell/examples/conveyor_cell
ros2 run fairino_workcell run_recipe pick_and_place.py --check    # check only: nothing moves
ros2 run fairino_workcell run_recipe pick_and_place.py            # run in the simulation
```
A recipe can also be checked without ROS or the simulation, for example in automated tests:
```
ros2 run fairino_workcell run_recipe pick_and_place.py --offline cell.yaml
```
`--report report.json` saves the report and `--keep-going` continues after a failed step.
Every run prints a report:
```
  #  step            kind     result      time  clearance  closest                   message
  1  home            move     OK         2.71s      94 mm  upperarm_link / pedestal  path is free
  2  above part      move     OK         1.65s      94 mm  upperarm_link / pedestal  path is free
  3  pick            move     OK         2.20s      25 mm  tool_link / conveyor      path is free
  4  grip            grip     OK         0.00s          -                            gripped part, TCP 0.0 mm from part/grasp, lifting it from conveyor
...
Cycle time 17.03 s | 10 steps: 10 OK, 0 warnings, 0 failed
```

| Step | Description |
|---|---|
| `move_joints(joints, degrees=True, speed=None, name=None, allow_contact=())` | PTP move to joint angles j1..j6 |
| `move_to(frame, xyz=(0, 0, 0), rpy_deg=(180, 0, 0), ...)` | PTP move of the TCP to a pose in `world`, an object or `object/frame`; the default orientation points the tool down onto the frame |
| `move_pose(pose, ...)` | PTP move of the TCP to a `geometry_msgs/PoseStamped` |
| `grip(name=None)` | Grip the movable object whose `grasp` frame is at the TCP (within 10 mm) |
| `release(name=None)` | Leave the held object where it is |
| `set_io(name, value)`, `wait_io(name, value=True, timeout=None)` | Virtual digital IO (skipped in check runs) |
| `wait(seconds)` | Wait; counted in the cycle time |

`speed` is a fraction of the joint velocity and acceleration limits (default 0.3). A failed step
raises `RecipeError` and stops the recipe.

## What is checked
- **Reachability and joint limits.** A pose target is solved like on the robot controller: the
  joint solution nearest to the current joints, so the arm configuration (shoulder, elbow, wrist)
  follows from the previous step and a check gives the same joints as the run.
- **Collisions along the whole path** of the robot, the tool and the held object, with the cell
  objects and with themselves (every 0.5 degrees of joint motion). The report names the colliding
  pair (for example `tool_link <-> table`) and where on the path it happens; the move is not done.
- **Clearance**: the smallest distance to the cell along the path and the closest pair. Below
  10 mm the step is a warning.
- **Contacts**: a step can allow contact of the tool and the held object with named objects
  (`allow_contact=['part']` while closing the fingers around the part). After `grip`, the next move
  may lift the part off its support; after `release`, the next move may take the tool away from it.
- **Cycle time** from the PTP profiles (trapezoidal joint velocities).

RViz shows the TCP path of every step (green: OK, orange: warning, red: collision, with a red
sphere where it collides) on the `/workcell/check_markers` topic.

## Interfaces
The recipe API only uses these ROS interfaces (`fairino_workcell_msgs`), so other programs
(vision, IO, upper systems) can drive the same cell.

| Interface | Type | Description |
|---|---|---|
| `/workcell/move` | action `Move` | Joint or pose target, speed, allowed contacts, `check_only`; result and feedback |
| `/workcell/gripper` | service `Gripper` | Grip (`close: true`) or release |
| `/workcell/set_io` | service `SetIO` | Set a virtual digital IO |
| `/workcell/check/begin` | service `std_srvs/Trigger` | Start a check session from the current state |
| `/workcell/run/begin` | service `std_srvs/Trigger` | Start a recipe run (clears the step markers) |
| `/workcell/io` | topic `IOState` | Virtual IO values (latched) |
| `/workcell/step_report` | topic `StepReport` | Report of every step |
| `/workcell/check_markers` | topic `visualization_msgs/MarkerArray` | TCP paths of the steps |

Check-only steps run in a check session: a copy of the cell state that the steps update (the
robot stays where it is), so a whole recipe can be checked before it runs.
