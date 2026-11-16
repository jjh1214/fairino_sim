"""Read STL files into vertex and triangle arrays without third-party mesh libraries."""

import struct

import numpy as np

_BINARY_TRIANGLE = np.dtype([('normal', '<f4', (3,)), ('corners', '<f4', (9,)),
                             ('attribute', '<u2')])


class MeshError(RuntimeError):
    """A mesh file cannot be read."""


def read_stl(path, scale=1.0):
    """Return (vertices, triangles) of a binary or ASCII STL file, vertices scaled to metres."""
    try:
        with open(path, 'rb') as stream:
            data = stream.read()
    except OSError as exc:
        raise MeshError(f'Cannot read {path}: {exc}') from None
    corners = _binary_corners(data)
    if corners is None:
        corners = _ascii_corners(data, path)
    if len(corners) == 0:
        raise MeshError(f'{path} contains no triangles')
    vertices, inverse = np.unique(corners, axis=0, return_inverse=True)
    triangles = np.asarray(inverse).reshape(-1, 3)
    return vertices * scale, triangles


def _binary_corners(data):
    if len(data) < 84:
        return None
    count = struct.unpack_from('<I', data, 80)[0]
    if len(data) != 84 + 50 * count:
        return None
    records = np.frombuffer(data, dtype=_BINARY_TRIANGLE, count=count, offset=84)
    return records['corners'].reshape(-1, 3).astype(np.float64)


def _ascii_corners(data, path):
    text = data.decode('ascii', errors='ignore')
    if not text.lstrip().lower().startswith('solid'):
        raise MeshError(f'{path} is not an STL file')
    values = [line.split()[1:4] for line in text.splitlines()
              if line.strip().lower().startswith('vertex')]
    try:
        corners = np.array(values, dtype=np.float64).reshape(-1, 3)
    except ValueError:
        raise MeshError(f'{path} is not a valid STL file') from None
    if len(corners) % 3:
        raise MeshError(f'{path} is not a valid STL file')
    return corners
