"""
Example recipe: an index cycle of four parts from the conveyor to the table.

The conveyor brings four parts, one after the other, to its station; the robot places them on
the table in a 2 x 2 pattern.

  ros2 run fairino_workcell run_recipe index_cycle.py --reset --check   # check only
  ros2 run fairino_workcell run_recipe index_cycle.py --reset           # run in the simulation
  ros2 run fairino_workcell run_recipe index_cycle.py --offline cell.yaml

The report shows how long the robot waits for parts and how often it grips one.
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pick_and_place import HOME, pick, place  # noqa: E402  (the steps of the single cycle)

PATTERN = [(-0.08, -0.08), (0.08, -0.08), (-0.08, 0.08), (0.08, 0.08)]   # m from table/place


def recipe(cell):
    cell.move_joints(HOME, name='home')
    for number, (x, y) in enumerate(PATTERN, 1):
        pick(cell, n=f'{number}: ')
        place(cell, x, y, n=f'{number}: ')
    cell.move_joints(HOME, name='home')


if __name__ == '__main__':
    from fairino_workcell.api import run
    sys.exit(run(recipe))
