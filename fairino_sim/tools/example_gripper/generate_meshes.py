"""
Generate the meshes of the example parallel gripper (millimetres).

  body.stl     flange adapter and housing; origin on the flange face, +z away from the robot
  finger.stl   one finger in its own frame: the gripping face lies in the plane y = 0 and the
               finger extends to +y (away from the other finger) and down to +z

tool.yaml mounts the finger frame 62 mm below the flange face (the bottom of the housing).
Needs trimesh:  pip install trimesh
"""

import os

import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))


def box(extents, centre):
    return trimesh.creation.box(extents=extents).apply_translation(centre)


def cylinder(radius, z0, z1):
    return trimesh.creation.cylinder(radius=radius, height=z1 - z0, sections=48).apply_translation(
        [0.0, 0.0, 0.5 * (z0 + z1)])


def body():
    parts = [cylinder(31.5, 0.0, 10.0),                 # adapter flange
             cylinder(24.0, 10.0, 18.0),                # neck
             box([70.0, 156.0, 44.0], [0.0, 0.0, 40.0])]  # housing with the finger guides
    return trimesh.util.concatenate(parts)


def finger():
    parts = [box([46.0, 26.0, 14.0], [0.0, 13.0, 7.0]),   # carriage in the guide
             box([28.0, 10.0, 76.0], [0.0, 5.0, 52.0]),   # finger plate, gripping face y = 0
             box([8.0, 14.0, 16.0], [0.0, 17.0, 22.0])]   # rib between carriage and plate
    return trimesh.util.concatenate(parts)


if __name__ == '__main__':
    for name, mesh in (('body', body()), ('finger', finger())):
        path = os.path.join(HERE, f'{name}.stl')
        mesh.export(path)
        print(f'{path}: {len(mesh.faces)} faces, bounds {mesh.bounds.tolist()}')
