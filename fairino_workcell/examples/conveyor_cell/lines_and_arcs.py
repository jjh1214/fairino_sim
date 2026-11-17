"""
Example recipe: a TCP path of straight lines and arcs (MoveL and MoveC).

As for dispensing or deburring, the TCP traces a rectangle with rounded corners 5 cm above the
table.

  ros2 run fairino_workcell run_recipe lines_and_arcs.py --check
  ros2 run fairino_workcell run_recipe lines_and_arcs.py --offline cell.yaml
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pick_and_place import AT_TABLE, HOME  # noqa: E402

HEIGHT = 0.05                   # m above the table top
RADIUS = 0.05                   # m, corner radius
X0, X1, Y0, Y1 = -0.20, 0.15, -0.10, 0.10   # m from table/place: the rectangle
SPEED = 0.1                     # of the TCP speed limit: 0.1 m/s


def corner(cx, cy, start_angle):
    """Return the via and end points of a quarter arc around (cx, cy), counterclockwise."""
    middle, end = math.radians(start_angle + 45), math.radians(start_angle + 90)
    return ((cx + RADIUS * math.cos(middle), cy + RADIUS * math.sin(middle), HEIGHT),
            (cx + RADIUS * math.cos(end), cy + RADIUS * math.sin(end), HEIGHT))


def recipe(cell):
    cell.move_joints(HOME, name='home')
    start = (X0 + RADIUS, Y0, HEIGHT)
    cell.move_to('table/place', xyz=(start[0], start[1], HEIGHT + 0.1), rpy_deg=AT_TABLE,
                 name='above start')
    cell.move_line('table/place', xyz=start, rpy_deg=AT_TABLE, speed=SPEED, name='down')
    sides = [((X1 - RADIUS, Y0, HEIGHT), (X1 - RADIUS, Y0 + RADIUS, -90)),
             ((X1, Y1 - RADIUS, HEIGHT), (X1 - RADIUS, Y1 - RADIUS, 0)),
             ((X0 + RADIUS, Y1, HEIGHT), (X0 + RADIUS, Y1 - RADIUS, 90)),
             ((X0, Y0 + RADIUS, HEIGHT), (X0 + RADIUS, Y0 + RADIUS, 180))]
    for number, (line_end, (cx, cy, angle)) in enumerate(sides, 1):
        cell.move_line('table/place', xyz=line_end, rpy_deg=AT_TABLE, speed=SPEED,
                       name=f'side {number}')
        via, end = corner(cx, cy, angle)
        cell.move_arc('table/place', via=via, xyz=end, rpy_deg=AT_TABLE, speed=SPEED,
                      name=f'corner {number}')
    cell.move_line('table/place', xyz=(start[0], start[1], HEIGHT + 0.1), rpy_deg=AT_TABLE,
                   name='up')
    cell.move_joints(HOME, name='home')


if __name__ == '__main__':
    from fairino_workcell.api import run
    sys.exit(run(recipe))
