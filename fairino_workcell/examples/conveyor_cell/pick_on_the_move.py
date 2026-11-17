"""
Example recipe for tracking_cell.yaml: pick parts on the move from the running conveyor.

The robot places them on the table in a 2 x 2 pattern.

  ros2 launch fairino_workcell fairino_workcell.launch.py cell:=$PWD/tracking_cell.yaml
  ros2 run fairino_workcell run_recipe pick_on_the_move.py --reset --check   # check only
  ros2 run fairino_workcell run_recipe pick_on_the_move.py                   # run
  ros2 run fairino_workcell run_recipe pick_on_the_move.py --offline tracking_cell.yaml

The report shows how long the robot waits for parts, how often it picks one, and the parts the
conveyor carries off unpicked.
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from index_cycle import PATTERN  # noqa: E402
from pick_and_place import AT_CONVEYOR, HOME, place  # noqa: E402

WAIT_ABOVE = (0.30, 0.0, 0.20)     # m from the sensor frame: above the belt, downstream


def recipe(cell):
    cell.move_joints(HOME, name='home')
    for number, (x, y) in enumerate(PATTERN, 1):
        n = f'{number}: '
        cell.move_to('conveyor/sensor', xyz=WAIT_ABOVE, rpy_deg=AT_CONVEYOR,
                     name=f'{n}wait above belt')
        cell.wait_io('conveyor/part_ready', name=f'{n}wait for part')
        cell.pick_moving('conveyor', approach=0.10, lift=0.12, rpy_deg=AT_CONVEYOR,
                         name=f'{n}pick on the move')
        place(cell, x, y, n=n)
    cell.move_joints(HOME, name='home')


if __name__ == '__main__':
    from fairino_workcell.api import run
    sys.exit(run(recipe))
