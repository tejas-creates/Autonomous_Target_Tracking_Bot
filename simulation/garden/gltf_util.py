"""Minimal glTF reader: world-space vertex positions (Y-up as stored)."""
import json
from pathlib import Path
import numpy as np

COMP = {5126: np.float32, 5123: np.uint16, 5125: np.uint32, 5121: np.uint8}
NCOMP = {'SCALAR': 1, 'VEC2': 2, 'VEC3': 3, 'VEC4': 4, 'MAT4': 16}


def _mat(node):
    if 'matrix' in node:
        return np.array(node['matrix'], float).reshape(4, 4).T
    t = np.array(node.get('translation', [0, 0, 0]), float)
    x, y, z, w = node.get('rotation', [0, 0, 0, 1])
    r = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    s = np.array(node.get('scale', [1, 1, 1]), float)
    m = np.eye(4)
    m[:3, :3] = r * s
    m[:3, 3] = t
    return m


def positions(path):
    """All vertex positions in model space, as {mesh_name: (N,3)} (glTF Y-up)."""
    path = Path(path)
    g = json.loads(path.read_text())
    bins = [(path.parent / b['uri']).read_bytes() for b in g['buffers']]

    def accessor(i):
        a = g['accessors'][i]
        bv = g['bufferViews'][a['bufferView']]
        n = NCOMP[a['type']]
        dt = np.dtype(COMP[a['componentType']])
        off = bv.get('byteOffset', 0) + a.get('byteOffset', 0)
        stride = bv.get('byteStride', 0) or dt.itemsize * n
        raw = np.frombuffer(bins[bv['buffer']], np.uint8, count=stride * (a['count'] - 1) + dt.itemsize * n,
                            offset=off)
        out = np.lib.stride_tricks.as_strided(raw, (a['count'], dt.itemsize * n), (stride, 1)).copy()
        return out.view(dt).reshape(a['count'], n)

    out = {}

    def walk(idx, parent):
        node = g['nodes'][idx]
        m = parent @ _mat(node)
        if 'mesh' in node:
            mesh = g['meshes'][node['mesh']]
            pts = [accessor(p['attributes']['POSITION']) for p in mesh['primitives']]
            p = np.concatenate(pts).astype(float)
            p = p @ m[:3, :3].T + m[:3, 3]
            out[mesh.get('name', node.get('name', str(idx)))] = np.vstack([out.get(mesh.get('name'), np.zeros((0, 3))), p])
        for c in node.get('children', []):
            walk(c, m)
    for root in g['scenes'][g.get('scene', 0)]['nodes']:
        walk(root, np.eye(4))
    return out
