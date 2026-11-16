"""
Generate the sample CAD files of the example cell (all in millimetres).

  conveyor.step   belt conveyor, 2 m long, belt surface 800 mm above the floor
  part.step       120 x 80 x 60 mm part with rounded vertical edges
  table.stl       600 x 600 mm table, 750 mm high
  fence.obj       1.5 m wide, 1.8 m high safety fence panel

Each file has its origin on the floor (or the part's bottom face), centred.
Needs gmsh and trimesh:  pip install gmsh trimesh
"""

import os

import gmsh
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))


def conveyor_and_part():
    gmsh.initialize()
    gmsh.option.setNumber('General.Terminal', 0)
    occ = gmsh.model.occ

    gmsh.model.add('conveyor')
    occ.addBox(-1000, -200, 700, 2000, 400, 100)                 # belt body
    for y in (200, -220):                                       # side guides
        occ.addBox(-1000, y, 760, 2000, 20, 80)
    for x in (-1000, 1000):                                     # end rollers
        occ.addCylinder(x, -200, 750, 0, 400, 0, 50)
    for x in (-930, 870):                                       # legs
        for y in (-180, 120):
            occ.addBox(x, y, 0, 60, 60, 700)
    occ.synchronize()
    gmsh.write(os.path.join(HERE, 'conveyor.step'))

    gmsh.model.add('part')
    box = occ.addBox(-60, -40, 0, 120, 80, 60)
    occ.synchronize()
    vertical_edges = [tag for _, tag in occ.getEntities(1)
                      if abs(occ.getMass(1, tag) - 60) < 1e-6]
    occ.fillet([box], vertical_edges, [8])
    occ.synchronize()
    gmsh.write(os.path.join(HERE, 'part.step'))
    gmsh.finalize()


def box(extents, centre):
    return trimesh.creation.box(extents=extents).apply_translation(centre)


def table():
    parts = [box([600, 600, 40], [0, 0, 730])]
    for x in (-265, 265):
        for y in (-265, 265):
            parts.append(box([50, 50, 710], [x, y, 355]))
    trimesh.util.concatenate(parts).export(os.path.join(HERE, 'table.stl'))


def fence():
    parts = [box([40, 40, 1800], [0, y, 900]) for y in (-750, 750)]          # posts
    parts += [box([40, 1460, 40], [0, 0, z]) for z in (250, 1780)]           # rails
    parts.append(box([6, 1460, 1490], [0, 0, 1015]))                         # mesh panel
    trimesh.util.concatenate(parts).export(os.path.join(HERE, 'fence.obj'))


if __name__ == '__main__':
    conveyor_and_part()
    table()
    fence()
