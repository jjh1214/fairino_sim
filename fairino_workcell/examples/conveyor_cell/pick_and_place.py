"""
Example recipe: wait for the first part at the conveyor station and place it on the table.

  ros2 run fairino_workcell run_recipe pick_and_place.py --reset          # run in the simulation
  ros2 run fairino_workcell run_recipe pick_and_place.py --reset --check  # check only
  ros2 run fairino_workcell run_recipe pick_and_place.py --offline cell.yaml   # without ROS

Pose targets of PTP moves are solved like on the robot controller: the joint solution nearest
to the current joints. HOME (elbow up, tool down) therefore sets the arm configuration of the
moves. The vertical moves to and from the part are straight lines (move_line, like MoveL).
"""

HOME = [-45, -90, -90, -90, 90, 0]   # joint angles in degrees, as on the teach pendant
GRASP_HEIGHT = 0.03                  # height of the part's grasp frame above its bottom (m)
APPROACH = 0.15                      # approach distance above the grasp and place positions (m)
# tool pointing down, turned about the vertical so that the wrist turns little between targets
AT_CONVEYOR = (180, 0, 180)          # roll, pitch, yaw in degrees, relative to the target frame
AT_TABLE = (180, 0, -90)


def pick(cell, n=''):
    """Pick the part at the conveyor station and lift it; the conveyor brings the next one."""
    cell.move_to('conveyor/station', xyz=(0, 0, GRASP_HEIGHT + APPROACH), rpy_deg=AT_CONVEYOR,
                 name=f'{n}above station')
    cell.wait_io('conveyor/part_ready', name=f'{n}wait for part')
    # the gripper fingers close around the part, so contact with parts is allowed
    cell.move_line('conveyor/station', xyz=(0, 0, GRASP_HEIGHT), rpy_deg=AT_CONVEYOR, speed=0.1,
                   allow_contact=['part'], name=f'{n}pick')
    cell.grip(name=f'{n}grip')
    cell.move_line('conveyor/station', xyz=(0, 0, GRASP_HEIGHT + APPROACH), rpy_deg=AT_CONVEYOR,
                   name=f'{n}lift')


def place(cell, x=0.0, y=0.0, n=''):
    """Place the held part on the table, at (x, y) from the table/place frame."""
    cell.move_to('table/place', xyz=(x, y, GRASP_HEIGHT + APPROACH), rpy_deg=AT_TABLE,
                 name=f'{n}above table')
    # the part touches the table at the end of the move
    cell.move_line('table/place', xyz=(x, y, GRASP_HEIGHT), rpy_deg=AT_TABLE, speed=0.1,
                   allow_contact=['table'], name=f'{n}place')
    cell.release(name=f'{n}release')
    cell.move_line('table/place', xyz=(x, y, GRASP_HEIGHT + APPROACH), rpy_deg=AT_TABLE,
                   name=f'{n}retreat')


def recipe(cell):
    cell.move_joints(HOME, name='home')
    pick(cell)
    place(cell)
    cell.move_joints(HOME, name='home')


if __name__ == '__main__':
    import sys

    from fairino_workcell.api import run
    sys.exit(run(recipe))
