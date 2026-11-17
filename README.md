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
| `finger_left`, `finger_right` | The fingers of a parallel gripper (only when the tool has `fingers`), on the +y and the -y side of the tool |

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
  xyz: [0.0, 0.0, 0.134]
  rpy: [0.0, 0.0, 0.0]
mass: 0.9                      # kg, used by Gazebo
touch_links: [wrist3_link]     # robot links the tool may touch without a collision
fingers:                       # optional: a parallel gripper whose two fingers close along y
  mesh:                        # the finger on the +y side, in its own frame (see below)
    visual: finger.stl
    collision: finger.stl      # optional, defaults to visual
    units: mm
  origin: {xyz: [0.0, 0.0, 0.062]}   # finger frame on the tool origin when the gripper is closed
  stroke: 0.055                # m each finger travels from closed to open
  speed: 0.05                  # m/s of each finger
  mass: 0.08                   # kg per finger, used by Gazebo
```
```
ros2 launch fairino_sim fairino_gz_moveit2.launch.py tool:=/path/to/my_gripper.yaml
```
The finger mesh is the finger on the +y side: its gripping face lies in the plane y = 0 of its
own frame and it extends to +y, away from the other finger; the other finger is the same mesh
turned 180 degrees about z. Each finger is a prismatic joint (`finger_left_joint`,
`finger_right_joint`) whose value is the distance of its gripping face from the middle, from 0
(closed) to `stroke` (open, the start position). In Gazebo the `gripper_controller` (a joint
trajectory controller, `/gripper_controller/follow_joint_trajectory`) moves both; MoveIt and the
recipe checks see the fingers where they are. With fingers, Gazebo's joints follow their
commands more closely (gz_ros2_control `position_proportional_gain` 0.5 instead of 0.1), so the
fingers reach the part when the recipe expects them to. The example gripper
([`tools/example_gripper`](fairino_sim/tools/example_gripper)) opens to 110 mm; its meshes are
made by `generate_meshes.py`. A tool without `fingers` (a vacuum cup, a fixed gripper) grips
without moving parts.

# Work cell
`fairino_workcell` builds a work cell around the robot from one cell file. Equipment imported from
CAD (STEP, STL, OBJ) appears in Gazebo, in the MoveIt planning scene (collision checking) and as
TF frames. Recipes written in Python move the robot through the cell; every step is checked for
reachability, joint limits and collisions before it runs (see [Recipes](#recipes)).

## Example cell
```
ros2 launch fairino_workcell fairino_workcell.launch.py
```
A fairino5_v6 with the example gripper on a pedestal, next to a belt conveyor that brings four
parts to its station, a table and a safety fence. The parts do not lie exactly on the station:
each one lies up to 3 cm across the belt and is turned up to 25 degrees (`placement_error`), and
the recipes pick them where they are.

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
parts:                                          # part types that conveyors bring (optional)
  - name: part                                  # the parts are named part_1, part_2, ...
    visual: meshes/part/visual.glb
    collision: meshes/part/collision.stl
    units: m
    frames:
      grasp: {xyz: [0.0, 0.0, 0.03]}            # where the TCP must be to grip it
objects:
  - name: conveyor                              # letters, digits and _
    visual: meshes/conveyor/visual.glb          # GLB, DAE, OBJ or STL, relative to this file
    collision: meshes/conveyor/collision.stl    # STL (may be omitted when visual is an STL)
    units: m                                    # required for meshes: m or mm
    pose: {xyz: [0.0, -0.6, 0.0], rpy_deg: [0, 0, 90]}   # in the world frame; or rpy in radians
    color: [0.36, 0.40, 0.45, 1.0]              # optional
    frames:                                     # optional named frames on the object
      entry: {xyz: [-0.75, 0.0, 0.80]}
      station: {xyz: [0.0, 0.0, 0.80]}
    conveyor:                                   # optional: an index conveyor (see below)
      part: part                                # part type it brings
      entry: entry                              # frame where parts are put on the belt
      station: station                          # frame where parts stop
      speed: 0.15                               # m/s
      spacing: 0.30                             # m between parts on the belt
      count: 4                                  # parts to bring; 0 or omitted: no limit
      placement_error:                          # optional: parts lie off their nominal place
        across: 0.03                            # +/- m across the belt
        along: 0.0                              # +/- m along the belt
        yaw_deg: 25                             # +/- degrees about the vertical
        seed: 7                                 # the same parts in every run
  - name: pedestal
    box: [0.30, 0.30, 0.75]                     # or cylinder: {radius: r, length: l}, in metres,
    pose: {xyz: [0.0, 0.0, 0.375]}              # centred on the pose
    touch_links: [base_link]                    # robot links allowed to touch this object
  - name: fixture_part                          # an object can also be a part itself:
    box: [0.1, 0.1, 0.05]
    pose: {xyz: [0.5, 0.3, 0.775]}
    movable: true                               # recipes can grip and move it
    frames:
      grasp: {xyz: [0.0, 0.0, 0.0]}
```
Each object is a TF frame named after it, and each named frame is `<object>/<frame>`
(for example `conveyor/station`), also available as a MoveIt subframe. The parts that conveyors
bring are frames too (`part_1`, `part_1/grasp`). Movable objects and parts follow the gripper
when a recipe grips them; the other objects stay where the cell file puts them.
In Gazebo the objects are visual only: the executor checks collisions itself, so Gazebo physics
never stops the robot at a contact that a step allows (fingers around a part, a part set down).

### Conveyors
A conveyor puts a part on its belt at the `entry` frame, then one every `spacing` metres of belt
travel. The parts travel in a straight line. The conveyors start with the simulation; `--reset`
(see below) puts the parts back.

- **Index conveyor** (`mode: index`, the default): the parts stop at the `station` frame. The belt
  stands still while a part waits at the station, and the IO `<conveyor>/part_ready` (for example
  `conveyor/part_ready`) is true. When the robot grips that part, the belt runs again and brings
  the next one; the parts behind keep their spacing.
- **Continuous conveyor** (`mode: continuous`): the belt keeps running. The `station` frame is a
  sensor: `<conveyor>/part_ready` is true while a part that has passed it is on the belt, to be
  picked on the move with `pick_moving` (conveyor tracking). Parts that reach the `exit` frame
  leave the cell; the reports list them as missed.
  ```yaml
      conveyor:
        mode: continuous
        part: part
        entry: entry
        station: sensor        # a frame of the object: the sensor (photo-eye)
        exit: exit             # parts that reach it leave the belt
        speed: 0.06
        spacing: 0.90
  ```

**Placement errors.** With `placement_error`, each part lies a random distance across the belt
(`across`) and along it (`along`) from its nominal place, and is turned about the vertical
(`yaw_deg`), each drawn evenly within +/- the given range. The draws depend only on `seed` and
the part number, so a check, the run after it and every run after `--reset` see the same parts in
the same places; another seed gives other places. An index conveyor still stops each part at the
station along the belt (its sensor stops the belt), so `along` only makes parts arrive earlier or
later there; on a continuous conveyor they pass the sensor earlier or later. A recipe finds the
part where it lies through the frame `<conveyor>/part` (see [Recipes](#recipes)), as a vision
system would report it; a conveyor object cannot have its own frame named `part`.

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
(triangle budget of the collision mesh, default 3000). With `--part` it prints part type
entries for the `parts` list (no pose, a `grasp` frame to set) instead of objects.

## Tests
```
cd ~/fairino_ws
colcon test --packages-select fairino_sim fairino_workcell && colcon test-result --verbose
```
The `import_cad` tests need the packages of `requirements-cad.txt` and are skipped without
them; run them with the virtual environment of [Importing CAD files](#importing-cad-files):
```
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ~/fairino_cad_venv/bin/python -m pytest \
  ~/fairino_ws/src/fairino_sim/fairino_workcell/test/test_cad_import.py
```

# Recipes
A recipe is a Python function that receives the cell. Steps take joint angles (degrees, as on the
teach pendant) or TCP poses in a cell frame, and can wait for IO such as conveyor sensors:
```python
HOME = [-45, -90, -90, -90, 90, 0]

def recipe(cell):
    cell.move_joints(HOME, name='home')
    cell.move_to('conveyor/station', xyz=(0, 0, 0.18), rpy_deg=(180, 0, 180), name='above station')
    cell.wait_io('conveyor/part_ready', name='wait for part')
    # conveyor/part: the part waiting at the station, wherever it lies; the gripper turns with it
    cell.move_line('conveyor/part/grasp', xyz=(0, 0, 0.15), rpy_deg=(180, 0, 180), name='above part')
    cell.move_line('conveyor/part/grasp', rpy_deg=(180, 0, 180), speed=0.1, name='pick')
    cell.grip(name='grip')                                   # the fingers close on the part
    cell.move_line('tcp', xyz=(0, 0, -0.15), name='lift')   # 15 cm back along the tool axis
    ...
```
The examples in [`examples/conveyor_cell`](fairino_workcell/examples/conveyor_cell):
- [`pick_and_place.py`](fairino_workcell/examples/conveyor_cell/pick_and_place.py): wait for the
  first part at the conveyor station and place it on the table;
- [`index_cycle.py`](fairino_workcell/examples/conveyor_cell/index_cycle.py): take the four parts
  the conveyor brings, one after the other, and place them on the table in a 2 x 2 pattern;
- [`lines_and_arcs.py`](fairino_workcell/examples/conveyor_cell/lines_and_arcs.py): trace a
  rectangle with rounded corners above the table with straight lines and arcs (MoveL, MoveC);
- [`pick_on_the_move.py`](fairino_workcell/examples/conveyor_cell/pick_on_the_move.py) with
  [`tracking_cell.yaml`](fairino_workcell/examples/conveyor_cell/tracking_cell.yaml): the belt
  keeps running and the robot picks the parts on the move.

Start the cell:
```
ros2 launch fairino_workcell fairino_workcell.launch.py
```
Then, in another terminal:
```
cd ~/fairino_ws/src/fairino_sim/fairino_workcell/examples/conveyor_cell
ros2 run fairino_workcell run_recipe index_cycle.py --reset --check   # check only: nothing moves
ros2 run fairino_workcell run_recipe index_cycle.py                   # run in the simulation
```
`--reset` first puts the cell back to its start: the robot stays where it is, objects and parts
go back, and the conveyors wait for the next run. So a check from a reset and the run after it
start from the same state, and the run follows the timeline of the check.
A recipe can also be checked without ROS or the simulation, for example in automated tests:
```
ros2 run fairino_workcell run_recipe index_cycle.py --offline cell.yaml
```
`--report report.json` saves the report and `--keep-going` continues after a failed step.
Every run prints a report, a line for each step as it finishes:
```
  #  step               kind     result      time  clearance  closest                   message
  1  home               move     OK         2.71s      94 mm  upperarm_link / pedestal  path is free
  2  1: above station   move     OK         1.65s      94 mm  upperarm_link / pedestal  path is free
  3  1: wait for part   wait     OK         0.64s      94 mm  upperarm_link / pedestal  conveyor/part_ready = true after 0.64 s
  4  1: above part      move     OK         0.76s      94 mm  upperarm_link / pedestal  line is free
  5  1: pick            move     OK         2.00s      12 mm  finger_right / conveyor   line is free
  6  1: grip            grip     OK         0.30s      12 mm  finger_right / conveyor   gripped part_1, TCP 0.0 mm from part_1/grasp, fingers closed to 80.0 mm in 0.30 s, lifting it from conveyor
  7  1: lift            move     OK         1.00s      12 mm  finger_left / conveyor    line is free
...
Cycle time 59.28 s | 42 steps: 42 OK, 0 warnings, 0 failed | waiting 0.64 s | 4 parts gripped, one every 13.69 s
```

| Step | Description |
|---|---|
| `move_joints(joints, degrees=True, speed=None, name=None, allow_contact=())` | PTP move to joint angles j1..j6 |
| `move_to(frame, xyz=(0, 0, 0), rpy_deg=None, ...)` | PTP move of the TCP to a pose in a frame (see below); by default the tool points down onto the frame |
| `move_pose(pose, ...)` | PTP move of the TCP to a `geometry_msgs/PoseStamped` |
| `move_line(frame, xyz=(0, 0, 0), rpy_deg=None, ...)` | Straight TCP line to a pose (like MoveL) |
| `move_arc(frame, via, xyz, rpy_deg=None, ...)` | TCP arc through the point `via` to `xyz`, both in `frame` (like MoveC) |
| `pick_moving(conveyor, approach=0.1, lift=0.1, rpy_deg=(180, 0, 0), speed=None, grip_time=None, part='')` | Pick a part on the move from a continuous conveyor (see below) |
| `grip(name=None)` | Close the gripper on the part at the TCP (see below) |
| `release(name=None)` | Leave the held object where it is and open the fingers |
| `wait_io(io, value=True, timeout=None, name=None)` | Wait until a virtual IO has the value; a timeout (s) fails the step |
| `wait(seconds, name=None)` | Wait for a time |
| `set_io(io, value, name=None)` | Set a virtual IO, for example an output to another machine |

`speed` is a fraction (default 0.3) of the joint velocity and acceleration limits for PTP moves,
and of the TCP limits for lines, arcs and picks on the move: 1.0 m/s and 2.0 m/s² by default
(executor parameters `max_tcp_speed`, `max_tcp_acceleration`; the FAIRINO values may differ).
Along a line or an arc the TCP orientation turns uniformly from the start to the target.
`allow_contact` takes object names, part names (`part_3`) or part types (`part`: all its parts).
A failed step raises `RecipeError` and stops the recipe.

Frames of a target: `world`; an object or a part (`table`, `part_3`) and its named frames
(`table/place`, `part_3/grasp`); `<conveyor>/part`, the part that conveyor has ready (at its
station, or the first one past its sensor) wherever it lies, with its frames
(`conveyor/part/grasp`); and `tcp`, the TCP where the move starts, for moves in the tool frame
(its z axis points out of the tool, so `move_line('tcp', xyz=(0, 0, -0.1))` backs the tool off
10 cm). By default the tool points down onto the frame (180 degrees about x), or keeps its
orientation on `tcp`.

**Gripping.** With a gripper that has fingers, `grip` closes them from where they are until they
touch the part between them: the part whose `grasp` frame is nearest to the TCP. The finger that
touches first pushes the part until the other one touches it too, so the part ends up centred
between the fingers (the report says how far it was pushed; more than 10 mm is a warning). If
the fingers close without touching it on both sides, the grip fails (`nothing to grip: the
fingers close without touching part_1: it is not between them`). The closing takes the time the
fingers need (0.30 s for the example gripper on the example part) and is checked against the
cell; `release` opens them fully, also checked. A part on a running conveyor is picked with
`pick_moving`. Without fingers, `grip` takes the object whose `grasp` frame is within 10 mm of the
TCP, at once.

### Picking on the move
`pick_moving` is one checked motion: the TCP comes in step with the part `approach` metres above
its grasp pose (arriving at the belt's speed, turned with the part: `rpy_deg` is relative to
the part's grasp frame), goes down to it in a straight line while following it, follows it while
the fingers close (`grip_time`, by default the time they need), lifts it `lift` metres while
still following the belt, and stops. Without `part`, it takes the first part past the sensor that
can still be caught, as a tracking system skips parts that are already too far; those leave the
cell as missed parts.
```python
cell.move_to('conveyor/sensor', xyz=(0.3, 0, 0.2), rpy_deg=(180, 0, 180), name='wait above belt')
cell.wait_io('conveyor/part_ready', name='wait for part')
cell.pick_moving('conveyor', approach=0.10, lift=0.12, rpy_deg=(180, 0, 180), name='pick')
```

## What is checked
- **Reachability and joint limits.** A pose target is solved like on the robot controller: the
  joint solution nearest to the current joints, so the arm configuration (shoulder, elbow, wrist)
  follows from the previous step and a check gives the same joints as the run.
- **Lines, arcs and picks on the move** are followed point by point from the current joints. They
  fail where the TCP leaves the reach, where a joint leaves its limits, or where the arm would
  have to change its configuration (a singularity); the message names the singularity, for
  example `j6 leaves its limits at 85% of the path (175.3 deg), after passing close to the wrist
  singularity at 50% (j5 3.4 deg)`. Coming within 5 degrees (wrist j5, elbow j3) or 2 cm
  (shoulder) of a singularity is a warning. Where a joint would turn faster than its limit, a line
  or arc slows down (a warning); a pick on the move cannot, so it fails.
- **Collisions along the whole path** of the robot, the tool and the held object, with the cell
  objects, the parts and themselves (every 0.5 degrees of joint motion). Parts on a running
  conveyor are checked where they are at each moment of the move. The report names the colliding
  pair (for example `tool_link <-> table`) and where on the path it happens; the move is not done.
- **Waits**: parts that move while the robot waits are checked against it too, for example a part
  that arrives at the station while the gripper is already down there.
- **Clearance**: the smallest distance to the cell and the parts along the path, and the closest
  pair. Below 10 mm the step is a warning.
- **Fingers**: the open fingers must come down beside the part without touching it, also on
  the move; closing and opening them is checked against the rest of the cell, with the gripped
  part allowed to touch them. A recipe that picks at a fixed place while the parts lie elsewhere
  fails here (`collision at 68% of the path: finger_right <-> part_1`).
- **Contacts**: a step can allow contact of the tool and the held object with named objects
  (`allow_contact=['table']` while setting a part down on it). After `grip`, the next move
  may lift the part off its support; after `release`, the next move may take the tool away from it.
- **Contact** closer than 0.1 mm counts as a collision unless it is allowed, so setting a part
  down exactly on a surface needs `allow_contact`.
- **Times**: the duration of every move (trapezoidal joint velocity profiles), every wait and
  every grip and release (the fingers' travel at their speed), the cycle time, the time spent
  waiting, how often a part is gripped and the parts missed.

## Cell time
In a check, the conveyors run in virtual time: a move lets them run for its duration, a wait
until the IO has the value. `wait_io` on a conveyor sensor therefore knows how long the robot
waits, and fails if the part never comes (`conveyor/part_ready will not become true: conveyor
conveyor has no more parts`). IO that other programs set (vision, PLC) is assumed in a check.

In the simulation, the conveyors run in simulation time. During a recipe run they only run while
a step takes time, like on a robot controller where checking a step and the gaps between steps
take no time; so the run follows the timeline of its check. Outside runs they run freely.

RViz shows the TCP path of every step (green: OK, orange: warning, red: collision, with a red
sphere where it collides) on the `/workcell/check_markers` topic, and the parts on
`/workcell/part_markers`.

## Interfaces
The recipe API only uses these ROS interfaces (`fairino_workcell_msgs`), so other programs
(vision, IO, upper systems) can drive the same cell.

| Interface | Type | Description |
|---|---|---|
| `/workcell/move` | action `Move` | PTP, LIN or CIRC to a joint or pose target, speed, allowed contacts, `check_only` |
| `/workcell/pick_moving` | action `PickMoving` | Pick a part on the move from a continuous conveyor |
| `/workcell/gripper` | service `Gripper` | Grip (`close: true`) or release; fingers move in the simulation |
| `/workcell/wait` | service `Wait` | Wait for a time or for an IO value |
| `/workcell/set_io` | service `SetIO` | Set a virtual IO (conveyor sensors cannot be set) |
| `/workcell/check/begin` | service `std_srvs/Trigger` | Start a check session from the current state |
| `/workcell/run/begin` | service `std_srvs/Trigger` | Start a recipe run (clears the step markers) |
| `/workcell/run/end` | service `std_srvs/Trigger` | End the run: the conveyors run freely again |
| `/workcell/reset` | service `std_srvs/Trigger` | Put objects, parts and conveyors back to the start |
| `/workcell/io` | topic `IOState` | Virtual IO values, conveyor sensors included (latched) |
| `/workcell/step_report` | topic `StepReport` | Report of every step |
| `/workcell/check_markers` | topic `visualization_msgs/MarkerArray` | TCP paths of the steps |
| `/workcell/part_markers` | topic `visualization_msgs/MarkerArray` | Parts brought by conveyors |

Check-only steps run in a check session: a copy of the cell state that the steps update (the
robot stays where it is), so a whole recipe can be checked before it runs.
