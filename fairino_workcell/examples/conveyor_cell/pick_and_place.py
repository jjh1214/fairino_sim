"""
Example recipe: pick the part at the conveyor station and place it on the table.

  ros2 run fairino_workcell run_recipe pick_and_place.py            # run in the simulation
  ros2 run fairino_workcell run_recipe pick_and_place.py --check    # check only, nothing moves
  ros2 run fairino_workcell run_recipe pick_and_place.py --offline cell.yaml   # without ROS

Pose targets are solved like on the robot controller: the joint solution nearest to the
current joints. HOME (elbow up, tool down) therefore sets the arm configuration of the moves.
"""

HOME = [-45, -90, -90, -90, 90, 0]   # joint angles in degrees, as on the teach pendant
GRASP_HEIGHT = 0.03                  # height of part/grasp above the bottom of the part (m)
APPROACH = 0.15                      # approach distance above the grasp and place positions (m)
# tool pointing down, turned about the vertical so that the wrist turns little between targets
AT_CONVEYOR = (180, 0, 180)          # roll, pitch, yaw in degrees, relative to the target frame
AT_TABLE = (180, 0, -90)


def recipe(cell):
    cell.move_joints(HOME, name='home')

    # pick: the gripper fingers close around the part, so contact with it is allowed
    cell.move_to('part/grasp', xyz=(0, 0, APPROACH), rpy_deg=AT_CONVEYOR, name='above part')
    cell.move_to('part/grasp', rpy_deg=AT_CONVEYOR, speed=0.1, allow_contact=['part'],
                 name='pick')
    cell.grip(name='grip')
    cell.move_to('conveyor/station', xyz=(0, 0, GRASP_HEIGHT + APPROACH), rpy_deg=AT_CONVEYOR,
                 name='lift')

    # place: the part touches the table at the end of the move
    cell.move_to('table/place', xyz=(0, 0, GRASP_HEIGHT + APPROACH), rpy_deg=AT_TABLE,
                 name='above table')
    cell.move_to('table/place', xyz=(0, 0, GRASP_HEIGHT), rpy_deg=AT_TABLE, speed=0.1,
                 allow_contact=['table'], name='place')
    cell.release(name='release')
    cell.move_to('table/place', xyz=(0, 0, GRASP_HEIGHT + APPROACH), rpy_deg=AT_TABLE,
                 name='retreat')
    cell.move_joints(HOME, name='home')


if __name__ == '__main__':
    import sys

    from fairino_workcell.api import run
    sys.exit(run(recipe))
