#!/usr/bin/env python3
"""Build the garden world: procedural textures, OBJ meshes and ../garden_world.sdf.

Everything is generated from fixed seeds, so the output is reproducible and the
world needs no internet / Fuel downloads.  Run from anywhere:

    python3 simulation/garden/build_garden_world.py

Design rules (see docs/garden.md):
  * Operating area 12 x 8 m centred at (1.9, 2.1) - same as simulation_world.sdf.
  * LiDAR scan plane ~0.22 m.  Inside the area every object is either flat on the
    ground (<= 3 mm, visual only) or a real obstacle (r 0.10 m, h 0.30 m cylinder
    collision, visual cross-section ~0.10 m at scan height).
  * Hedge 0.55 m tall, outside the area, with box collisions.
  * All decoration (trees, bench, lamp, pergola, ...) outside the hedge.
"""

import math
import sys
from pathlib import Path

import numpy as np
from PIL import Image

HERE = Path(__file__).resolve().parent
SIM_DIR = HERE.parent
TEX_DIR = HERE / 'assets' / 'textures'
MESH_DIR = HERE / 'assets' / 'meshes'
WORLD_FILE = SIM_DIR / 'garden_world.sdf'
REL = 'garden/assets'               # asset path relative to the world file
EXT = f'{REL}/external'             # downloaded CC0 assets (fetch_assets.py)

WORLD_NAME = 'Autonomous_Target_Tracking_Bot'

# Real-world size (m) covered by one repeat of the photo textures.
GRASS_TILE = 1.4

# Operating area (identical to the plain world's ground box).
AREA_CX, AREA_CY = 1.9, 2.1
AREA_X0, AREA_X1 = AREA_CX - 6.0, AREA_CX + 6.0
AREA_Y0, AREA_Y1 = AREA_CY - 4.0, AREA_CY + 4.0

# Hedge: inner face this far outside the area edge, thickness, height.
HEDGE_GAP, HEDGE_T, HEDGE_H = 0.07, 0.55, 0.55

# Back yard (wooden fence) outside the hedge.
YARD_X0, YARD_X1 = -8.8, 12.6
YARD_Y0, YARD_Y1 = -5.4, 11.4

STATIC_OBSTACLES = [
    ('Static_Cylinder_01', 3.5, 0.0),
    ('Static_Cylinder_02', 3.91646629383458, 3.95615843583788),
    ('Static_Cylinder_03', 5.29579574182807, 2.88040766331977),
    ('Static_Cylinder_04', 5.21739651040941, 0.97503696357694),
    ('Static_Cylinder_05', 1.567105558238, 0.0),
    ('Static_Cylinder_06', -0.2984938219409, 0.02240008134529),
    ('Static_Cylinder_07', -1.7432329733961, 2.98037686654909),
]

MOVING_OBSTACLES = [
    ('Moving_Cylinder_01', 1.0, 4.06746940175069),
    ('Moving_Cylinder_02', 2.0, 4.0),
    ('Moving_Cylinder_03', 2.7641495217722, 4.14340191291121),
    ('Moving_Cylinder_04', 4.58368182869024, 3.68096213347195),
    ('Moving_Cylinder_05', 5.67757135101035, 1.75373810816494),
    ('Moving_Cylinder_06', 4.39444506236884, 0.42654272926868),
    ('Moving_Cylinder_07', 2.4, 0.0),
    ('Moving_Cylinder_08', 0.84925437257673, 0.03754459685878),
    ('Moving_Cylinder_09', -1.2428006979732, 0.21601543891485),
    ('Moving_Cylinder_10', -1.77305438444233, 2.0),
    ('Moving_Cylinder_11', -0.56251075657829, 2.65627689144573),
    # Added later: start spots only, the controller moves them onto their crossing segments.
    ('Moving_Cylinder_12', 3.0, 2.2),
    ('Moving_Cylinder_13', 0.8, 1.6),
    ('Moving_Cylinder_14', 6.8, 4.6),
    ('Moving_Cylinder_15', 6.9, -0.6),
    ('Moving_Cylinder_16', -3.0, 5.0),
    # 17-20 added 2026-11-23 (20 moving obstacles).
    ('Moving_Cylinder_17', -3.0, -1.0),
    ('Moving_Cylinder_18', 6.5, 2.6),
    ('Moving_Cylinder_19', 1.0, 5.5),
    ('Moving_Cylinder_20', 4.2, -1.3),
]

TRAJECTORY_CSV = SIM_DIR.parent / 'docs' / 'trajectory.csv'


# =============================================================================
# Texture helpers
# =============================================================================

def fbm(n, seed, beta=2.0, lo=1.0, hi=None, aspect=(1.0, 1.0)):
    """Tileable fractal noise (zero mean, unit std) by spectral synthesis.

    aspect=(ax, ay) > 1 damps frequencies along that axis -> features stretched
    along it (rows are v / y, columns are u / x).
    """
    rng = np.random.default_rng(seed)
    white = rng.standard_normal((n, n))
    f = np.fft.fftfreq(n) * n
    fy, fx = np.meshgrid(f, f, indexing='ij')
    r = np.sqrt((fx * aspect[0]) ** 2 + (fy * aspect[1]) ** 2)
    mask = r >= lo
    if hi is not None:
        mask &= r <= hi
    amp = np.zeros_like(r)
    amp[mask] = r[mask] ** (-beta / 2.0)
    out = np.real(np.fft.ifft2(np.fft.fft2(white) * amp))
    out -= out.mean()
    return out / (out.std() + 1e-9)


def voronoi(n, cells, seed, jitter=0.85):
    """Tileable cell noise: returns F1, F2 (in cell units) and cell id."""
    rng = np.random.default_rng(seed)
    off = 0.5 + (rng.random((cells, cells, 2)) - 0.5) * jitter
    ys, xs = np.mgrid[0:n, 0:n].astype(np.float64) + 0.5
    px, py = xs * cells / n, ys * cells / n
    cx, cy = np.floor(px).astype(int), np.floor(py).astype(int)
    f1 = np.full((n, n), 9.0)
    f2 = np.full((n, n), 9.0)
    cid = np.zeros((n, n), dtype=np.int64)
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            gx, gy = (cx + dx) % cells, (cy + dy) % cells
            o = off[gy, gx]
            d = np.hypot(px - (cx + dx + o[..., 0]), py - (cy + dy + o[..., 1]))
            closer = d < f1
            f2 = np.where(closer, f1, np.minimum(f2, d))
            cid = np.where(closer, gy * cells + gx, cid)
            f1 = np.where(closer, d, f1)
    return f1, f2, cid


def sat(x):
    return np.clip(x, 0.0, 1.0)


def smooth(x, lo, hi):
    t = sat((x - lo) / (hi - lo))
    return t * t * (3 - 2 * t)


def mix(a, b, t):
    a, b = np.asarray(a, float), np.asarray(b, float)
    t = np.asarray(t, float)[..., None]
    return a * (1 - t) + b * t


def normal_from_height(h, strength):
    dx = (np.roll(h, -1, 1) - np.roll(h, 1, 1)) * 0.5 * strength
    dy = (np.roll(h, -1, 0) - np.roll(h, 1, 0)) * 0.5 * strength
    nrm = np.stack([-dx, dy, np.ones_like(h)], -1)
    nrm /= np.linalg.norm(nrm, axis=-1, keepdims=True)
    return nrm * 0.5 + 0.5


def downsample(img, factor):
    if factor == 1:
        return img
    n = img.shape[0] // factor
    return img[:n * factor, :n * factor].reshape(n, factor, n, factor, -1).mean((1, 3))


def save_tex(name, rgb, quality=88):
    TEX_DIR.mkdir(parents=True, exist_ok=True)
    arr = (sat(np.asarray(rgb)) * 255 + 0.5).astype(np.uint8)
    Image.fromarray(arr, 'RGB').save(TEX_DIR / f'{name}.jpg', quality=quality, optimize=True)
    return f'{REL}/textures/{name}.jpg'


def stamp_leaves(img, hmap, count, length, width, palette, rng, z0, bright=(0.8, 1.15)):
    """Paint `count` pointed leaves in place (tileable, wraps around)."""
    n = img.shape[0]
    palette = np.asarray(palette, float)
    for _ in range(count):
        ln = length * rng.uniform(0.7, 1.3)
        wd = width * rng.uniform(0.75, 1.25)
        a = rng.uniform(0, 2 * np.pi)
        cx, cy = rng.integers(0, n, 2)
        r = int(ln / 2) + 2
        yy, xx = np.mgrid[-r:r + 1, -r:r + 1]
        u = (xx * math.cos(a) + yy * math.sin(a)) / (ln / 2)
        v = (-xx * math.sin(a) + yy * math.cos(a)) / (wd / 2)
        prof = np.clip(1 - u ** 2, 0, None) ** 0.7
        m = np.abs(v) < prof
        if not m.any():
            continue
        col = palette[rng.integers(len(palette))] * rng.uniform(*bright)
        rel = np.abs(v[m]) / (prof[m] + 1e-6)
        shade = (0.78 + 0.22 * (1 - rel)) * (0.88 + 0.12 * u[m])
        shade = np.where(rel < 0.09, shade * 0.86, shade)
        rows, cols = (cy + yy[m]) % n, (cx + xx[m]) % n
        img[rows, cols] = col * shade[:, None]
        hmap[rows, cols] = z0 + 0.6 * (1 - rel)


def stamp_flowers(img, hmap, clusters, per, radius, petal, centre, rng, spread):
    n = img.shape[0]
    for _ in range(clusters):
        ccx, ccy = rng.uniform(0, n, 2)
        col = np.asarray(petal, float) * rng.uniform(0.85, 1.1)
        for _ in range(per):
            fx, fy = ccx + rng.normal(0, spread), ccy + rng.normal(0, spread)
            rr = radius * rng.uniform(0.7, 1.3)
            r = int(rr) + 2
            yy, xx = np.mgrid[-r:r + 1, -r:r + 1]
            ang = np.arctan2(yy, xx)
            dist = np.hypot(xx, yy) / rr
            m = dist < (0.75 + 0.25 * np.cos(5 * ang + rng.uniform(0, 6.28)))
            if not m.any():
                continue
            rows = (int(fy) + yy[m]) % n
            cols = (int(fx) + xx[m]) % n
            shade = (1.0 - 0.25 * dist[m])[:, None]
            c = np.where((dist[m] < 0.3)[:, None], np.asarray(centre, float), col)
            img[rows, cols] = c * shade
            hmap[rows, cols] = 1.6 - 0.4 * dist[m]


def strokes(img, hmap, n_strokes, lmin, lmax, colours, rng, base_h=0.5, angles=None, width=1):
    """Vectorised short line strokes (grass blades, pine needles)."""
    n = img.shape[0]
    x0 = rng.uniform(0, n, n_strokes)
    y0 = rng.uniform(0, n, n_strokes)
    ln = rng.uniform(lmin, lmax, n_strokes)
    ang = rng.uniform(0, 2 * np.pi, n_strokes) if angles is None else angles
    k = int(math.ceil(lmax)) + 1
    t = np.linspace(0, 1, k)[None, :] * (lmax / max(lmax, 1e-6))
    keep = (t * lmax) <= ln[:, None]
    xs = x0[:, None] + np.cos(ang)[:, None] * t * lmax
    ys = y0[:, None] + np.sin(ang)[:, None] * t * lmax
    tt = np.clip(t * lmax / ln[:, None], 0, 1)
    shade = 0.72 + 0.4 * tt
    col = colours[:, None, :] * shade[..., None]
    h = base_h + 0.5 * tt
    for w in range(width):
        r = (ys.astype(int) % n)[keep]
        c = ((xs.astype(int) + w) % n)[keep]
        img[r, c] = col[keep]
        hmap[r, c] = h[keep]


# =============================================================================
# Textures
# =============================================================================

def make_textures():
    tex = {}

    # ---------------- Photo-scanned CC0 materials (see assets/external/CREDITS.md) --
    for key, kind, name in (('grass', 'ambientcg', 'Grass004'), ('gravel', 'ambientcg', 'Gravel022'),
                            ('stone', 'polyhaven', 'rock_pitted_mossy'), ('soil', 'polyhaven', 'farm_soil'),
                            ('mulch', 'polyhaven', 'wood_chips'), ('patio', 'polyhaven', 'grey_stone_path'),
                            ('concrete', 'polyhaven', 'rough_concrete'), ('bark', 'polyhaven', 'bark_brown_02')):
        folder = f'{EXT}/{kind}/{name}'
        if not (HERE.parent / folder / f'{name}_albedo.jpg').exists():
            raise SystemExit(f'missing {folder}: run garden/fetch_assets.py first')
        # Unique file names matter: Gazebo caches textures by file name.
        tex[key], tex[key + '_n'], tex[key + '_r'] = (f'{folder}/{name}_albedo.jpg', f'{folder}/{name}_normal.jpg',
                                                      f'{folder}/{name}_roughness.jpg')

    # ---------------- Foliage family ---------------------------------------------
    def leaves_tex(name, seed, n, layers, bg, palettes, length, width, flowers=None, needles=None):
        rng = np.random.default_rng(seed)
        img = np.zeros((n, n, 3)) + np.asarray(bg)
        img *= (0.7 + 0.3 * smooth(fbm(n, seed + 1, beta=2.0), -2, 2))[..., None]
        hm = np.zeros((n, n))
        for i, (count, pal) in enumerate(zip(layers, palettes)):
            if needles:
                cols = np.asarray(pal, float)[rng.integers(0, len(pal), count)] * rng.uniform(0.8, 1.2, (count, 1))
                strokes(img, hm, count, needles[0], needles[1], cols, rng, base_h=i * 0.5, width=2)
            else:
                stamp_leaves(img, hm, count, length, width, pal, rng, z0=i * 0.5)
        if flowers:
            for petal, centre, clusters in flowers:
                stamp_flowers(img, hm, clusters, 9, n / 110, petal, centre, rng, n / 60)
        save_tex(name + '_albedo', img)
        save_tex(name + '_normal', normal_from_height(hm, 1.2))
        return f'{REL}/textures/{name}_albedo.jpg', f'{REL}/textures/{name}_normal.jpg'

    hedge_pal = [[(0.10, 0.24, 0.07), (0.13, 0.28, 0.08)],
                 [(0.15, 0.33, 0.09), (0.18, 0.36, 0.10), (0.13, 0.30, 0.09)],
                 [(0.22, 0.42, 0.12), (0.26, 0.45, 0.14), (0.20, 0.40, 0.13)]]
    tex['hedge'], tex['hedge_n'] = leaves_tex('hedge', 51, 1024, [7000, 7000, 6000], (0.03, 0.07, 0.02),
                                              hedge_pal, 30, 14)
    plant_pal = [[(0.11, 0.26, 0.08)], [(0.17, 0.36, 0.10), (0.20, 0.40, 0.12)],
                 [(0.27, 0.48, 0.15), (0.31, 0.52, 0.17)]]
    tex['foliage'], tex['foliage_n'] = leaves_tex('foliage', 81, 512, [500, 500, 420], (0.04, 0.08, 0.03),
                                                  plant_pal, 58, 26)
    shrub_pal = [[(0.10, 0.22, 0.07)], [(0.15, 0.31, 0.09), (0.17, 0.33, 0.10)], [(0.22, 0.40, 0.13)]]
    tex['pink'], tex['pink_n'] = leaves_tex('shrub_pink', 91, 512, [1700, 1700, 1400], (0.04, 0.07, 0.03),
                                            shrub_pal, 26, 12,
                                            flowers=[((0.86, 0.42, 0.58), (0.95, 0.85, 0.40), 60),
                                                     ((0.93, 0.62, 0.72), (0.95, 0.85, 0.40), 35)])
    tex['lavender'], tex['lavender_n'] = leaves_tex('shrub_lavender', 92, 512, [1700, 1700, 1400],
                                                    (0.10, 0.08, 0.14),
                                                    [[(0.20, 0.28, 0.16)], [(0.28, 0.38, 0.24)], [(0.36, 0.46, 0.30)]],
                                                    24, 9,
                                                    flowers=[((0.58, 0.44, 0.84), (0.45, 0.32, 0.70), 120), ((0.70, 0.58, 0.90), (0.5, 0.4, 0.75), 50)])
    tex['white'], tex['white_n'] = leaves_tex('shrub_white', 93, 512, [1700, 1700, 1400], (0.04, 0.07, 0.03),
                                              shrub_pal, 26, 12,
                                              flowers=[((0.93, 0.92, 0.86), (0.92, 0.78, 0.30), 70)])
    tex['yellow'], tex['yellow_n'] = leaves_tex('shrub_yellow', 94, 512, [1700, 1700, 1400], (0.04, 0.07, 0.03),
                                                shrub_pal, 26, 12,
                                                flowers=[((0.93, 0.70, 0.18), (0.70, 0.36, 0.10), 70),
                                                         ((0.90, 0.48, 0.12), (0.55, 0.25, 0.08), 18)])

    # ---------------- Stump top rings (disc, not tiled) ---------------------------
    n = 512
    ys, xs = (np.mgrid[0:n, 0:n] + 0.5) / n * 2 - 1
    rad = np.hypot(xs, ys)
    ang = np.arctan2(ys, xs)
    warp = fbm(n, 111, beta=3.0) * 0.03
    rings = 0.5 + 0.5 * np.sin((rad + warp) * 2 * np.pi * 17)
    img = mix((0.62, 0.48, 0.33), (0.50, 0.36, 0.23), rings ** 3)
    img = mix(img, (0.56, 0.54, 0.50), smooth(rad, 0.3, 0.95) * 0.45)                    # weathering
    crack_n = np.abs(np.sin(ang * 3.0 + fbm(n, 112, beta=2.5) * 0.8))
    crack = (crack_n < 0.035) & (rad > 0.15) & (rad < 0.85)
    img[crack] *= 0.35
    img = mix(img, (0.20, 0.15, 0.11), smooth(rad, 0.88, 0.93))
    tex['rings'] = save_tex('stump_rings', img)

    # ---------------- Wood planks (grain along u) ---------------------------------
    def wood(name, seed, c0, c1, grey):
        n = 512
        grain = fbm(n, seed, beta=2.2, lo=2, aspect=(9.0, 1.0))
        fine = fbm(n, seed + 1, beta=0.8, lo=40, aspect=(6.0, 1.0))
        img = mix(c0, c1, smooth(grain * 0.7 + fine * 0.3, -1.6, 1.6))
        img = mix(img, (0.50, 0.48, 0.44), grey * smooth(fbm(n, seed + 2, beta=2.5), -1.0, 1.5))
        knot = smooth(fbm(n, seed + 3, beta=1.6, lo=6), 2.3, 3.0)
        img = img * (1 - 0.45 * knot)[..., None]
        save_tex(name + '_albedo', img)
        save_tex(name + '_normal', normal_from_height(grain * 0.4 + fine * 0.2 - knot, 1.5))
        return f'{REL}/textures/{name}_albedo.jpg', f'{REL}/textures/{name}_normal.jpg'

    tex['teak'], tex['teak_n'] = wood('wood_teak', 121, (0.42, 0.28, 0.16), (0.62, 0.45, 0.28), 0.35)
    tex['cedar'], tex['cedar_n'] = wood('wood_fence', 131, (0.30, 0.21, 0.14), (0.47, 0.34, 0.23), 0.45)
    tex['oakwood'], tex['oakwood_n'] = wood('wood_pergola', 141, (0.36, 0.26, 0.17), (0.54, 0.41, 0.28), 0.25)

    # Barrel staves (grain vertical, dark gaps between 14 staves).
    n = 512
    grain = fbm(n, 151, beta=2.2, lo=2, aspect=(1.0, 9.0))
    img = mix((0.33, 0.22, 0.13), (0.50, 0.35, 0.21), smooth(grain, -1.6, 1.6))
    u = (np.arange(n) / n)[None, :] * 14
    gapm = np.abs(u - np.round(u)) < 0.03
    img[np.broadcast_to(gapm, (n, n))] *= 0.35
    img = mix(img, (0.48, 0.46, 0.42), smooth(fbm(n, 152, beta=2.6), 0.0, 2.0) * 0.4)
    tex['barrel'] = save_tex('barrel_albedo', img)

    return tex


# =============================================================================
# Mesh helpers
# =============================================================================

def normals_of(v, f):
    n = np.zeros_like(v)
    fn = np.cross(v[f[:, 1]] - v[f[:, 0]], v[f[:, 2]] - v[f[:, 0]])
    for k in range(3):
        np.add.at(n, f[:, k], fn)
    ln = np.linalg.norm(n, axis=1, keepdims=True)
    return n / np.where(ln < 1e-12, 1, ln)


class Mesh:
    def __init__(self, v=None, t=None, f=None, n=None):
        self.v = np.zeros((0, 3)) if v is None else np.asarray(v, float)
        self.t = np.zeros((len(self.v), 2)) if t is None else np.asarray(t, float)
        self.f = np.zeros((0, 3), int) if f is None else np.asarray(f, int)
        self.n = normals_of(self.v, self.f) if n is None else np.asarray(n, float)

    def flipped(self):
        return Mesh(self.v, self.t, self.f[:, ::-1], -self.n)

    def ensure(self, idx, direction):
        """Flip the whole mesh if the normal at vertex idx points against `direction`."""
        return self.flipped() if np.dot(self.n[idx], direction) < 0 else self

    def transformed(self, rot=None, trans=(0, 0, 0), scale=1.0):
        rot = np.eye(3) if rot is None else rot
        s = np.broadcast_to(np.asarray(scale, float), (3,))
        v = (self.v * s) @ rot.T + np.asarray(trans, float)
        n = (self.n / s) @ rot.T
        n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
        f = self.f if np.prod(s) > 0 else self.f[:, ::-1]
        return Mesh(v, self.t, f, n)

    @staticmethod
    def merge(meshes):
        v, t, f, n, off = [], [], [], [], 0
        for m in meshes:
            v.append(m.v); t.append(m.t); n.append(m.n); f.append(m.f + off)
            off += len(m.v)
        return Mesh(np.concatenate(v), np.concatenate(t), np.concatenate(f), np.concatenate(n))

    def save(self, name):
        MESH_DIR.mkdir(parents=True, exist_ok=True)
        lines = [f'# {name} - generated by build_garden_world.py', 'mtllib default.mtl', 'o ' + name, 'usemtl default']
        lines += [f'v {a:.5f} {b:.5f} {c:.5f}' for a, b, c in self.v]
        lines += [f'vt {a:.5f} {b:.5f}' for a, b in self.t]
        lines += [f'vn {a:.4f} {b:.4f} {c:.4f}' for a, b, c in self.n]
        lines += ['f {0}/{0}/{0} {1}/{1}/{1} {2}/{2}/{2}'.format(*(tri + 1)) for tri in self.f]
        (MESH_DIR / f'{name}.obj').write_text('\n'.join(lines) + '\n')
        (MESH_DIR / 'default.mtl').write_text('# Placeholder, the SDF <material> overrides it.\n'
                                              'newmtl default\nKa 1 1 1\nKd 1 1 1\nKs 0 0 0\n')
        return f'{REL}/meshes/{name}.obj'


def grid_mesh(p, uv):
    rows, cols = p.shape[:2]
    i, j = np.meshgrid(np.arange(rows - 1), np.arange(cols - 1), indexing='ij')
    a = (i * cols + j).ravel()
    b, c, d = a + 1, a + cols + 1, a + cols
    f = np.concatenate([np.stack([a, b, c], 1), np.stack([a, c, d], 1)])
    return Mesh(p.reshape(-1, 3), uv.reshape(-1, 2), f)


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def rot_axis(axis, a):
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    x, y, z = axis
    c, s, C = math.cos(a), math.sin(a), 1 - math.cos(a)
    return np.array([[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
                     [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
                     [z * x * C - y * s, z * y * C + x * s, c + z * z * C]])


def rot_from_z(d):
    d = np.asarray(d, float) / np.linalg.norm(d)
    z = np.array([0, 0, 1.0])
    ax = np.cross(z, d)
    if np.linalg.norm(ax) < 1e-9:
        return np.eye(3) if d[2] > 0 else rot_axis((1, 0, 0), math.pi)
    return rot_axis(ax, math.acos(np.clip(np.dot(z, d), -1, 1)))


def lathe(profile, seg=48, tile_u=0.5, tile_v=0.5, radial=None):
    """Surface of revolution; profile = [(r, z), ...] ordered so the surface
    normal points outward when the profile goes upward on the outside."""
    prof = np.asarray(profile, float)
    th = np.linspace(0, 2 * np.pi, seg + 1)
    r = prof[:, 0][:, None] * np.ones_like(th)[None, :]
    z = prof[:, 1][:, None] * np.ones_like(th)[None, :]
    if radial is not None:
        r = r * (1 + radial(th[None, :], z))
    p = np.stack([r * np.cos(th), r * np.sin(th), z], -1)
    arc = np.concatenate([[0], np.cumsum(np.hypot(np.diff(prof[:, 0]), np.diff(prof[:, 1])))])
    rref = max(prof[:, 0].max(), 1e-3)
    uv = np.stack([np.broadcast_to(th[None, :] * rref / tile_u, r.shape),
                   np.broadcast_to(arc[:, None] / tile_v, r.shape)], -1)
    m = grid_mesh(p, uv)
    n = m.n.reshape(len(prof), seg + 1, 3)
    seam = n[:, 0] + n[:, -1]
    seam /= np.linalg.norm(seam, axis=1, keepdims=True) + 1e-12
    n[:, 0] = n[:, -1] = seam
    m.n = n.reshape(-1, 3)
    return m


def disc(radius, z, seg=40, centre=(0, 0), uv_mode='disc', tile=0.5, radial=None, up=True):
    th = np.linspace(0, 2 * np.pi, seg, endpoint=False)
    rr = radius * (1 + (radial(th) if radial else 0))
    ring = np.stack([centre[0] + rr * np.cos(th), centre[1] + rr * np.sin(th), np.full(seg, z)], 1)
    v = np.vstack([[centre[0], centre[1], z], ring])
    if uv_mode == 'disc':
        t = np.vstack([[0.5, 0.5], np.stack([0.5 + 0.5 * rr / radius * np.cos(th),
                                             0.5 + 0.5 * rr / radius * np.sin(th)], 1)])
    else:
        t = v[:, :2] / tile
    f = np.array([[0, 1 + k, 1 + (k + 1) % seg] for k in range(seg)])
    m = Mesh(v, t, f, np.tile([0, 0, 1.0], (seg + 1, 1)))
    return m if up else m.flipped()


def icosphere(sub):
    t = (1 + 5 ** 0.5) / 2
    v = [(-1, t, 0), (1, t, 0), (-1, -t, 0), (1, -t, 0), (0, -1, t), (0, 1, t), (0, -1, -t), (0, 1, -t),
         (t, 0, -1), (t, 0, 1), (-t, 0, -1), (-t, 0, 1)]
    v = [np.array(p) / np.linalg.norm(p) for p in v]
    f = [(0, 11, 5), (0, 5, 1), (0, 1, 7), (0, 7, 10), (0, 10, 11), (1, 5, 9), (5, 11, 4), (11, 10, 2),
         (10, 7, 6), (7, 1, 8), (3, 9, 4), (3, 4, 2), (3, 2, 6), (3, 6, 8), (3, 8, 9), (4, 9, 5),
         (2, 4, 11), (6, 2, 10), (8, 6, 7), (9, 8, 1)]
    for _ in range(sub):
        cache, nf = {}, []

        def mid(a, b):
            key = (min(a, b), max(a, b))
            if key not in cache:
                p = v[a] + v[b]
                v.append(p / np.linalg.norm(p))
                cache[key] = len(v) - 1
            return cache[key]
        for a, b, c in f:
            ab, bc, ca = mid(a, b), mid(b, c), mid(c, a)
            nf += [(a, ab, ca), (b, bc, ab), (c, ca, bc), (ab, bc, ca)]
        f = nf
    v = np.array(v)
    f = np.array(f)
    cen = v[f].mean(1)
    fn = np.cross(v[f[:, 1]] - v[f[:, 0]], v[f[:, 2]] - v[f[:, 0]])
    bad = (fn * cen).sum(1) < 0
    f[bad] = f[bad][:, ::-1]
    return v, f


class Noise3:
    """Smooth 3-D noise as a sum of random plane waves (std ~1)."""

    def __init__(self, seed, waves=12, fmin=2.0, fmax=9.0):
        rng = np.random.default_rng(seed)
        d = rng.normal(size=(waves, 3))
        d /= np.linalg.norm(d, axis=1, keepdims=True)
        self.k = d * rng.uniform(fmin, fmax, (waves, 1))
        self.p = rng.uniform(0, 2 * np.pi, waves)
        self.a = math.sqrt(2.0 / waves)

    def __call__(self, pts):
        return (np.sin(pts @ self.k.T + self.p) * self.a).sum(-1)


def blob(radius, centre=(0, 0, 0), stretch=(1, 1, 1), seed=0, amp=0.18, freq=2.5, sub=3, tile=0.5,
         z_min=None):
    """Lumpy foliage blob with triplanar per-face UVs (smooth normals)."""
    v, f = icosphere(sub)
    noise = Noise3(seed, fmin=freq * 0.6, fmax=freq * 1.6)
    disp = 1 + amp * np.clip(noise(v * 1.0) / 2.0, -1, 1)
    v = v * disp[:, None] * radius * np.asarray(stretch, float) + np.asarray(centre, float)
    if z_min is not None:
        v[:, 2] = np.maximum(v[:, 2], z_min)
    n = normals_of(v, f)
    vv = v[f].reshape(-1, 3)
    nn = n[f].reshape(-1, 3)
    fnorm = np.cross(v[f[:, 1]] - v[f[:, 0]], v[f[:, 2]] - v[f[:, 0]])
    axis = np.argmax(np.abs(fnorm), 1).repeat(3)
    uv = np.where(axis[:, None] == 0, vv[:, [1, 2]], np.where(axis[:, None] == 1, vv[:, [0, 2]], vv[:, [0, 1]]))
    return Mesh(vv, uv / tile, np.arange(len(vv)).reshape(-1, 3), nn)


def box_mesh(size, centre=(0, 0, 0), rot=None, tile=0.5):
    half = np.asarray(size, float) / 2
    vs, ts, ns, fs = [], [], [], []
    for axis in range(3):
        for sign in (-1, 1):
            nrm = np.zeros(3); nrm[axis] = sign
            p, q = [a for a in range(3) if a != axis]
            t1 = np.zeros(3); t1[p] = 1
            t2 = np.zeros(3); t2[q] = 1
            if np.dot(np.cross(t1, t2), nrm) < 0:
                t1, t2, p, q = t2, t1, q, p
            c = nrm * half
            corners = [c - t1 * half[p] - t2 * half[q], c + t1 * half[p] - t2 * half[q],
                       c + t1 * half[p] + t2 * half[q], c - t1 * half[p] + t2 * half[q]]
            uvs = []
            for k in corners:
                a, b = k[p], k[q]
                uvs.append((a, b) if half[p] >= half[q] else (b, a))
            base = len(vs)
            vs += corners; ts += uvs; ns += [nrm] * 4
            fs += [(base, base + 1, base + 2), (base, base + 2, base + 3)]
    m = Mesh(np.array(vs), np.array(ts) / tile, np.array(fs), np.array(ns))
    return m.transformed(rot, centre)


def polygon_ccw(pts):
    pts = np.asarray(pts, float)
    area = 0.5 * np.sum(pts[:, 0] * np.roll(pts[:, 1], -1) - np.roll(pts[:, 0], -1) * pts[:, 1])
    return pts if area > 0 else pts[::-1]


def extrude(poly, z0, z1, tile=0.5, uv_offset=(0, 0)):
    """Flat slab from a star-shaped polygon: top face + side walls (no bottom)."""
    poly = polygon_ccw(poly)
    k = len(poly)
    c = poly.mean(0)
    top_v = np.vstack([[c[0], c[1], z1], np.column_stack([poly, np.full(k, z1)])])
    top_t = (top_v[:, :2] + np.asarray(uv_offset)) / tile
    top_f = np.array([[0, 1 + i, 1 + (i + 1) % k] for i in range(k)])
    top = Mesh(top_v, top_t, top_f, np.tile([0, 0, 1.0], (k + 1, 1)))
    ring = np.vstack([poly, poly[:1]])
    arc = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(ring, axis=0), axis=1))])
    p = np.stack([np.column_stack([ring, np.full(k + 1, z0)]), np.column_stack([ring, np.full(k + 1, z1)])])
    uv = np.stack([np.column_stack([arc, np.full(k + 1, 0.0)]), np.column_stack([arc, np.full(k + 1, z1 - z0)])])
    side = grid_mesh(p, uv / tile)
    side = side.ensure(0, np.array([*(poly[0] - c), 0.0]))
    return Mesh.merge([top, side])


def blob_polygon(cx, cy, rx, ry, seed, k=40, amp=0.12, rot=0.0, power=2.0):
    rng = np.random.default_rng(seed)
    th = np.linspace(0, 2 * np.pi, k, endpoint=False)
    w = sum(rng.uniform(-1, 1) * np.sin(m * th + rng.uniform(0, 6.3)) / m for m in range(2, 6))
    rr = 1 + amp * w
    c, s = np.cos(th), np.sin(th)
    sx = np.sign(c) * np.abs(c) ** (2 / power)
    sy = np.sign(s) * np.abs(s) ** (2 / power)
    x, y = sx * rx * rr, sy * ry * rr
    cr, sr = math.cos(rot), math.sin(rot)
    return np.column_stack([cx + x * cr - y * sr, cy + x * sr + y * cr])


# =============================================================================
# Meshes
# =============================================================================

def radial_noise(seed, amp, kmin=3, kmax=11, zfreq=6.0):
    rng = np.random.default_rng(seed)
    ks = rng.integers(kmin, kmax + 1, 6)
    ph = rng.uniform(0, 6.3, 6)
    zf = rng.uniform(-zfreq, zfreq, 6)
    w = rng.uniform(0.5, 1.0, 6)

    def fn(th, z):
        out = sum(w[i] * np.sin(ks[i] * th + zf[i] * z + ph[i]) for i in range(6))
        return amp * out / w.sum()
    return fn


# Photo-scanned pots used as static obstacles: asset -> (mesh parts forming the pot, fit mode).
SCANNED_POTS = {
    'potted_plant_01': (('Circle.008', 'Circle.002'), 'uniform'),
    'potted_plant_02': (('Circle',), 'uniform'),
    'potted_plant_04': (('Mesh',), 'uniform'),
    'planter_pot_clay': (('Planter_Pot_01',), 'stretch'),
}
SCAN_BAND = (0.19, 0.25)       # around the LiDAR plane (~0.22 m)
LOW_LIMIT = 0.27               # below this everything must stay inside...
LOW_RADIUS = 0.112             # ...this radius (collision cylinder is 0.10)


def fit_scanned_pots():
    """Scale each scanned pot (and lift it on a plinth if needed) so that it matches the
    0.10 m x 0.30 m obstacle: ~0.10 m radius at LiDAR height and no geometry wider than
    LOW_RADIUS below LOW_LIMIT.  Plants may only spread out above that."""
    sys.path.insert(0, str(HERE))
    from gltf_util import positions
    fits = {}
    rng = np.random.default_rng(5)
    for asset, (pot_parts, mode) in SCANNED_POTS.items():
        parts = positions(HERE / 'assets' / 'external' / 'polyhaven' / asset / f'{asset}.gltf')
        parts = {k: np.c_[v[:, 0], -v[:, 2], v[:, 1]] for k, v in parts.items()}     # glTF Y-up -> Z-up
        pot = np.vstack([parts[k] for k in pot_parts])
        pts = np.vstack(list(parts.values()))
        lo, hi = pot.min(0), pot.max(0)
        centre = np.array([(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, lo[2]])
        pts = pts - centre
        if len(pts) > 40000:
            pts = pts[rng.choice(len(pts), 40000, replace=False)]
        r0, z0 = np.hypot(pts[:, 0], pts[:, 1]), pts[:, 2]
        top0 = hi[2] - lo[2]

        def metrics(sxy, sz, h):
            r, z = r0 * sxy, z0 * sz + h
            band = (z > SCAN_BAND[0]) & (z < SCAN_BAND[1])
            r_scan = np.percentile(r[band], 99) if band.any() else 0.0
            if h > SCAN_BAND[0]:
                r_scan = max(r_scan, 0.1)                    # plinth
            low = z < LOW_LIMIT
            r_low = np.percentile(r[low], 99.5) if low.any() else 0.0
            return r_scan, r_low, top0 * sz + h

        best = None
        if mode == 'stretch':                                # empty pot: stretch to 0.30 m
            sz = 0.30 / top0
            band = (z0 * sz > SCAN_BAND[0]) & (z0 * sz < SCAN_BAND[1])
            sxy = 0.1 / np.percentile(r0[band], 99)
            best = (sxy, sz, 0.0)
        else:
            for sc in np.arange(2.0, 0.2, -0.005):
                for h in np.arange(0.0, 0.2201, 0.005):
                    if not 0.26 <= top0 * sc + h <= 0.36:
                        continue
                    r_scan, r_low, _ = metrics(sc, sc, h)
                    if 0.088 <= r_scan <= 0.106 and r_low <= LOW_RADIUS:
                        best = (sc, sc, round(h, 3))
                        break
                if best:
                    break
        if best is None:
            raise SystemExit(f'no valid fit for {asset}')
        sxy, sz, h = best
        r_scan, r_low, top = metrics(sxy, sz, h)
        fits[asset] = dict(sxy=sxy, sz=sz, plinth=h, centre=centre)
        print(f'  fit {asset:18s} scale {sxy:.3f}/{sz:.3f} plinth {h:.3f} m -> radius at scan height '
              f'{r_scan:.3f}, max below {LOW_LIMIT} m {r_low:.3f}, pot top {top:.3f}')
    return fits


def make_meshes():
    M = {}

    # ---------------- Ground ------------------------------------------------------
    def plane(x0, x1, y0, y1, z, tile):
        p = np.array([[[x0, y0, z], [x1, y0, z]], [[x0, y1, z], [x1, y1, z]]])
        return grid_mesh(p, p[..., :2] / tile).ensure(0, (0, 0, 1))
    M['lawn'] = plane(YARD_X0 - 0.3, YARD_X1 + 0.3, YARD_Y0 - 0.3, YARD_Y1 + 0.3, 0.0, GRASS_TILE).save('lawn')
    big = 60.0
    meadow = Mesh.merge([
        plane(-big, big, -big, YARD_Y0 - 0.3, -0.004, GRASS_TILE * 1.6), plane(-big, big, YARD_Y1 + 0.3, big, -0.004, GRASS_TILE * 1.6),
        plane(-big, YARD_X0 - 0.3, YARD_Y0 - 0.3, YARD_Y1 + 0.3, -0.004, GRASS_TILE * 1.6),
        plane(YARD_X1 + 0.3, big, YARD_Y0 - 0.3, YARD_Y1 + 0.3, -0.004, GRASS_TILE * 1.6)])
    M['meadow'] = meadow.save('meadow')

    # ---------------- Garden path along the target trajectory ---------------------
    M.update(make_path())

    # ---------------- Hedge -------------------------------------------------------
    M['hedge'] = make_hedge().save('hedge')

    # ---------------- Static obstacle visuals ------------------------------------
    rn = radial_noise
    M['fits'] = fit_scanned_pots()
    for name, fit in M['fits'].items():
        if fit['plinth'] > 0.0:
            h = fit['plinth']
            M[f'plinth_{name}'] = lathe([(0.0, 0.0), (0.1, 0.0), (0.1, h - 0.008), (0.096, h), (0.0, h)],
                                        seg=48, tile_u=0.6, tile_v=0.6).save(f'plinth_{name}')
    # Weathered concrete planter (for the box ball)
    M['pot_concrete'] = lathe([(0.0, 0.0), (0.096, 0.0), (0.1, 0.006), (0.1, 0.288), (0.094, 0.3),
                               (0.08, 0.3), (0.078, 0.28)], seg=56, tile_u=0.6, tile_v=0.6).save('pot_concrete')
    # Tree stump (photo bark) with a ring-textured top.  The scanned stumps flare to ~3x
    # their top radius, which would hide a wide base under the LiDAR plane, so this one is modelled.
    M['stump'] = lathe([(0.0, 0.0), (0.118, 0.0), (0.108, 0.035), (0.1, 0.08), (0.098, 0.22),
                        (0.096, 0.28), (0.094, 0.292)], seg=40, tile_u=0.5, tile_v=0.6,
                       radial=rn(401, 0.06, 5, 13, 3.0)).save('stump')
    M['stump_top'] = disc(0.094, 0.292, seg=40, radial=lambda th: rn(401, 0.06, 5, 13, 3.0)(th, 0.292)
                          ).save('stump_top')
    # Half wine-barrel planter + iron hoops
    M['barrel'] = lathe([(0.0, 0.0), (0.094, 0.0), (0.1, 0.06), (0.103, 0.15), (0.1, 0.24), (0.097, 0.295),
                         (0.088, 0.3), (0.086, 0.28)], seg=56, tile_u=2 * math.pi * 0.103, tile_v=0.3
                        ).save('barrel')
    hoops = []
    for z in (0.035, 0.255):
        r = 0.1035 if z > 0.1 else 0.1005
        hoops.append(lathe([(r, z - 0.012), (r + 0.003, z), (r, z + 0.012)], seg=56))
    M['hoops'] = Mesh.merge(hoops).save('barrel_hoops')

    # Soil discs and plants (all plant geometry below 0.30 m stays inside r 0.09).
    M['soil_disc'] = disc(0.093, 0.275, seg=32, uv_mode='planar', tile=0.6).save('soil_disc')

    def plant(seed, lumps, r0, r1, spread, zc, zspread, stretch=(1, 1, 0.85), tile=0.4):
        rng = np.random.default_rng(seed)
        parts = []
        for i in range(lumps):
            rad = rng.uniform(r0, r1)
            a = rng.uniform(0, 2 * np.pi)
            d = rng.uniform(0, spread) if i else 0.0
            z = zc + rng.uniform(-zspread, zspread)
            z = max(z, 0.30 + rad * stretch[2] * 0.75)       # never dips below the rim
            parts.append(blob(rad, (d * math.cos(a), d * math.sin(a), z), stretch, seed * 13 + i,
                              amp=0.25, freq=3.0 / rad * 0.08, sub=2, tile=tile, z_min=0.285))
        return Mesh.merge(parts)

    M['plant_barrel'] = plant(504, 5, 0.08, 0.11, 0.07, 0.39, 0.03, tile=0.3).save('plant_barrel')
    M['boxwood'] = blob(0.155, (0, 0, 0.455), (1, 1, 0.97), 505, amp=0.07, freq=4.0, sub=3, tile=0.5,
                        z_min=0.295).save('boxwood')
    M['grass_tuft'] = make_grass_tuft(506).save('grass_tuft')

    # ---------------- Garden robots (z from 0 to 0.30) ----------------------------
    M['robot_chassis'] = lathe([(0.0, 0.012), (0.075, 0.012), (0.083, 0.02), (0.083, 0.07)], seg=40
                               ).save('robot_chassis')
    M['robot_bumper'] = lathe([(0.083, 0.048), (0.096, 0.05), (0.1, 0.058), (0.1, 0.086), (0.096, 0.094),
                               (0.09, 0.096)], seg=48).save('robot_bumper')
    M['robot_body_a'] = lathe([(0.088, 0.094), (0.097, 0.10), (0.098, 0.24), (0.094, 0.258), (0.082, 0.272),
                               (0.05, 0.284), (0.0, 0.288)], seg=48).save('robot_body_a')
    M['robot_body_b'] = lathe([(0.088, 0.094), (0.097, 0.10), (0.098, 0.245), (0.09, 0.262),
                               (0.05, 0.27), (0.0, 0.272)], seg=48).save('robot_body_b')
    M['robot_body_c'] = lathe([(0.088, 0.094), (0.097, 0.10), (0.098, 0.20), (0.098, 0.236)], seg=48
                              ).save('robot_body_c')
    M['robot_dome_c'] = lathe([(0.098, 0.236), (0.096, 0.252), (0.085, 0.272), (0.055, 0.29), (0.0, 0.296)],
                              seg=48).save('robot_dome_c')
    M['robot_band'] = lathe([(0.0985, 0.15), (0.0985, 0.168)], seg=48).save('robot_band')

    # ---------------- Shrubs (procedural: no suitable CC0 garden shrub scans) -------
    for i in range(4):
        M[f'shrub_{i}'] = blob(1.0, (0, 0, 0), (1, 1, 0.8), 800 + i, amp=0.32, freq=3.2, sub=3, tile=0.35
                               ).save(f'shrub_{i}')

    # ---------------- Garden furniture --------------------------------------------
    M['bench'] = make_bench().save('bench')
    M['pergola'] = make_pergola().save('pergola')
    M['fence'] = make_fence().save('fence')
    M['compost'] = make_compost().save('compost')
    M['birdbath'] = lathe([(0.0, 0.0), (0.2, 0.0), (0.2, 0.06), (0.12, 0.09), (0.07, 0.16), (0.06, 0.5),
                           (0.08, 0.6), (0.14, 0.64), (0.3, 0.7), (0.33, 0.74), (0.31, 0.76),
                           (0.27, 0.73), (0.0, 0.69)], seg=40, tile_u=0.5, tile_v=0.5).save('birdbath')
    M['water'] = disc(0.27, 0.725, seg=32).save('water')
    M['lamp_cap'] = lathe([(0.15, 2.52), (0.0, 2.64)], seg=4).transformed(rot_z(math.pi / 4)).save('lamp_cap')
    return M


def make_grass_tuft(seed):
    """Ornamental grass: thin double-sided blades rising from the pot centre."""
    rng = np.random.default_rng(seed)
    blades = []
    for _ in range(46):
        a = rng.uniform(0, 2 * np.pi)
        lean = rng.uniform(0.15, 0.7)
        length = rng.uniform(0.28, 0.5)
        w = rng.uniform(0.008, 0.013)
        k = 6
        s = np.linspace(0, 1, k)
        d = np.array([math.cos(a), math.sin(a), 0])
        side = np.array([-math.sin(a), math.cos(a), 0])
        root = d * rng.uniform(0, 0.03) + np.array([0, 0, 0.29])
        centre = root[None, :] + (d[None, :] * (lean * s ** 1.6 * length)[:, None]
                                  + np.array([0, 0, 1.0])[None, :] * (length * (s - 0.35 * lean * s ** 2))[:, None])
        hw = (w * (1 - s) + 0.001)[:, None]
        p = np.stack([centre - side * hw, centre + side * hw], 1)
        uv = np.stack([np.column_stack([np.zeros(k), s]), np.column_stack([np.ones(k), s])], 1)
        blades.append(grid_mesh(p, uv))
    return Mesh.merge(blades)


def make_bench():
    parts = []
    L, D, H = 1.5, 0.56, 0.44
    for x in (-L / 2 + 0.06, L / 2 - 0.06):
        parts.append(box_mesh((0.06, 0.06, H), (x, D / 2 - 0.05, H / 2)))                   # front legs
        parts.append(box_mesh((0.06, 0.06, 0.88), (x, -D / 2 + 0.05, 0.44),
                              rot_axis((1, 0, 0), -0.12)))                                    # back legs
        parts.append(box_mesh((0.06, D + 0.02, 0.05), (x, 0.0, H - 0.04)))                  # seat rail
        parts.append(box_mesh((0.06, D, 0.04), (x, 0.03, 0.66)))                            # armrest
        parts.append(box_mesh((0.05, 0.05, 0.24), (x, D / 2 - 0.05, 0.54)))                 # arm post
        parts.append(box_mesh((0.05, D - 0.1, 0.05), (x, 0.0, 0.12)))                       # low rail
    for k in range(5):
        y = -D / 2 + 0.08 + k * (D - 0.14) / 4
        parts.append(box_mesh((L - 0.04, 0.085, 0.028), (0, y, H)))                         # seat slats
    for k in range(4):
        z = 0.56 + k * 0.085
        y = -D / 2 + 0.04 - (z - 0.44) * 0.12
        parts.append(box_mesh((L - 0.16, 0.024, 0.06), (0, y, z), rot_axis((1, 0, 0), -0.12)))  # back slats
    parts.append(box_mesh((L, 0.05, 0.08), (0, -D / 2 - 0.0, 0.92), rot_axis((1, 0, 0), -0.12)))
    return Mesh.merge(parts)


def make_pergola():
    parts = []
    W, D, H = 2.8, 2.2, 2.3
    for x in (-W / 2, W / 2):
        for y in (-D / 2, D / 2):
            parts.append(box_mesh((0.12, 0.12, H), (x, y, H / 2), tile=0.6))
            parts.append(box_mesh((0.06, 0.4, 0.06), (x, y + (0.18 if y < 0 else -0.18), H - 0.25),
                                  rot_axis((1, 0, 0), 0.7 if y < 0 else -0.7), tile=0.6))
    for y in (-D / 2, D / 2):
        parts.append(box_mesh((W + 0.5, 0.07, 0.18), (0, y, H + 0.09), tile=0.6))
    for k in range(9):
        x = -W / 2 - 0.1 + k * (W + 0.2) / 8
        parts.append(box_mesh((0.05, D + 0.5, 0.13), (x, 0, H + 0.245), tile=0.6))
    return Mesh.merge(parts)


def make_fence():
    parts = []
    H, board_w, gap = 1.6, 0.1, 0.012

    def run(p0, p1, inward):
        p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
        d = p1 - p0
        L = np.linalg.norm(d)
        u = d / L
        yaw = math.atan2(u[1], u[0])
        R = rot_z(yaw)
        inn = np.asarray(inward, float)
        nb = int(L / (board_w + gap))
        rng = np.random.default_rng(int(abs(p0[0] * 100 + p0[1] * 7)))
        for k in range(nb):
            c = p0 + u * (k + 0.5) * (board_w + gap)
            h = H + rng.uniform(-0.01, 0.01)
            parts.append(box_mesh((board_w, 0.018, h), (c[0], c[1], h / 2), R, tile=0.5)
                         .transformed(None, (0, 0, 0)))
        for z in (0.3, 1.3):
            c = p0 + d / 2 + inn * 0.03
            parts.append(box_mesh((L, 0.04, 0.09), (c[0], c[1], z), R, tile=0.5))
        for k in range(int(L / 2.4) + 1):
            c = p0 + u * min(k * 2.4, L) + inn * 0.06
            parts.append(box_mesh((0.09, 0.09, H + 0.08), (c[0], c[1], (H + 0.08) / 2), R, tile=0.5))

    run((YARD_X0, YARD_Y0), (YARD_X1, YARD_Y0), (0, 1))
    run((YARD_X0, YARD_Y1), (YARD_X1, YARD_Y1), (0, -1))
    run((YARD_X0, YARD_Y0), (YARD_X0, YARD_Y1), (1, 0))
    run((YARD_X1, YARD_Y0), (YARD_X1, YARD_Y1), (-1, 0))
    return Mesh.merge(parts)


def make_compost():
    parts = []
    S, H = 1.0, 0.8
    for k in range(6):
        z = 0.08 + k * 0.13
        for (cx, cy, sx, sy) in ((0, -S / 2, S, 0.025), (0, S / 2, S, 0.025),
                                 (-S / 2, 0, 0.025, S), (S / 2, 0, 0.025, S)):
            parts.append(box_mesh((sx, sy, 0.11), (cx, cy, z)))
    for x in (-S / 2, S / 2):
        for y in (-S / 2, S / 2):
            parts.append(box_mesh((0.06, 0.06, H + 0.05), (x, y, (H + 0.05) / 2)))
    return Mesh.merge(parts)


def make_hedge():
    """Four lumpy hedge walls.  Inner face never comes closer than 0.035 m to the area."""
    noise = Noise3(9001, waves=14, fmin=3.0, fmax=11.0)
    noise2 = Noise3(9002, waves=10, fmin=12.0, fmax=24.0)
    T, Hh, d0 = HEDGE_T, HEDGE_H - 0.04, HEDGE_GAP
    rc = 0.12
    prof = [(d0, -0.02)]
    prof += [(d0, z) for z in np.linspace(0.05, Hh - rc, 6)]
    for a in np.linspace(math.pi, math.pi / 2, 5)[1:]:
        prof.append((d0 + rc + rc * math.cos(a), Hh - rc + rc * math.sin(a)))
    prof += [(d, Hh) for d in np.linspace(d0 + rc + 0.06, d0 + T - rc - 0.06, 4)]
    for a in np.linspace(math.pi / 2, 0, 5)[1:]:
        prof.append((d0 + T - rc + rc * math.cos(a), Hh - rc + rc * math.sin(a)))
    prof += [(d0 + T, z) for z in np.linspace(Hh - rc - 0.05, 0.05, 6)]
    prof += [(d0 + T, -0.02)]
    prof = np.array(prof)
    seg = np.diff(prof, axis=0)
    tang = np.vstack([seg[:1], (seg[1:] + seg[:-1]) / 2, seg[-1:]])
    tang /= np.linalg.norm(tang, axis=1, keepdims=True)
    pn = np.column_stack([tang[:, 1], -tang[:, 0]])      # outward of the cross-section (d, z)
    arc = np.concatenate([[0], np.cumsum(np.linalg.norm(seg, axis=1))])
    cx_sec = np.array([d0 + T / 2, Hh * 0.45])

    def side(origin, along, out, length):
        along, out = np.asarray(along, float), np.asarray(out, float)
        s = np.linspace(0, length, int(length / 0.07) + 1)
        base = np.asarray(origin, float)[None, :] + s[:, None] * along[None, :]
        P = (base[None, :, :] + prof[:, 0][:, None, None] * out[None, None, :]
             + prof[:, 1][:, None, None] * np.array([0, 0, 1.0])[None, None, :])
        N3 = pn[:, 0][:, None, None] * out + pn[:, 1][:, None, None] * np.array([0, 0, 1.0])
        amp = 0.035 * np.clip(noise(P) / 1.8, -1, 1) + 0.012 * np.clip(noise2(P) / 1.8, -1, 1)
        bottom = (prof[:, 1] < 0.0)[:, None]
        amp = np.where(bottom, amp * 0.6, amp)
        P = P + N3 * amp[..., None]
        uv = np.stack([np.broadcast_to(s[None, :] / 0.8, P.shape[:2]),
                       np.broadcast_to(arc[:, None] / 0.8, P.shape[:2])], -1)
        m = grid_mesh(P, uv)
        top_idx = int(np.argmax(prof[:, 1])) * P.shape[1] + P.shape[1] // 2
        m = m.ensure(top_idx, (0, 0, 1))
        caps = []
        for j, sign in ((0, -1), (P.shape[1] - 1, 1)):
            ring = P[:, j]
            centre = (np.asarray(origin, float) + along * s[j] + out * cx_sec[0] + np.array([0, 0, cx_sec[1]]))
            v = np.vstack([centre, ring])
            k = len(ring)
            f = np.array([[0, 1 + i, 1 + (i + 1) % k] for i in range(k)])
            uvc = np.vstack([[0.5, 0.5], np.column_stack([prof[:, 0] / 0.8, prof[:, 1] / 0.8])])
            cap = Mesh(v, uvc, f, np.tile(along * sign, (k + 1, 1)))
            fn = np.cross(v[f[0, 1]] - v[f[0, 0]], v[f[0, 2]] - v[f[0, 0]])
            if np.dot(fn, along * sign) < 0:
                cap = Mesh(v, uvc, f[:, ::-1], cap.n)
            caps.append(cap)
        return Mesh.merge([m] + caps)

    ext = d0 + T
    walls = [
        side((AREA_X0 - ext, AREA_Y0, 0), (1, 0, 0), (0, -1, 0), AREA_X1 - AREA_X0 + 2 * ext),
        side((AREA_X0 - ext, AREA_Y1, 0), (1, 0, 0), (0, 1, 0), AREA_X1 - AREA_X0 + 2 * ext),
        side((AREA_X0, AREA_Y0 - d0, 0), (0, 1, 0), (-1, 0, 0), AREA_Y1 - AREA_Y0 + 2 * d0),
        side((AREA_X1, AREA_Y0 - d0, 0), (0, 1, 0), (1, 0, 0), AREA_Y1 - AREA_Y0 + 2 * d0),
    ]
    return Mesh.merge(walls)


def path_centreline():
    pts = np.loadtxt(TRAJECTORY_CSV, delimiter=',', skiprows=1)[:, 1:3]
    n = len(pts)
    dense = []
    for i in range(n):
        p0, p1, p2, p3 = pts[(i - 1) % n], pts[i], pts[(i + 1) % n], pts[(i + 2) % n]
        for t in np.linspace(0, 1, 24, endpoint=False):
            t2, t3 = t * t, t * t * t
            dense.append(0.5 * ((2 * p1) + (-p0 + p2) * t + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t2
                                + (-p0 + 3 * p1 - 3 * p2 + p3) * t3))
    dense = np.array(dense)

    def resample(c, step=0.05):
        closed = np.vstack([c, c[:1]])
        seg = np.linalg.norm(np.diff(closed, axis=0), axis=1)
        s = np.concatenate([[0], np.cumsum(seg)])
        ss = np.arange(0, s[-1], step)
        return np.column_stack([np.interp(ss, s, closed[:, 0]), np.interp(ss, s, closed[:, 1])])

    def smooth_closed(c, window, iters):
        k = np.ones(window) / window
        for _ in range(iters):
            pad = np.vstack([c[-window:], c, c[:window]])
            c = np.column_stack([np.convolve(pad[:, 0], k, 'same'), np.convolve(pad[:, 1], k, 'same')])
            c = c[window:-window]
        return c

    c = smooth_closed(resample(dense), 9, 3)
    pots = np.array([(x, y) for _, x, y in STATIC_OBSTACLES])
    need = 0.30 + 0.10 + 0.10
    for _ in range(12):                      # push the path so the pots stand beside it
        tang = np.roll(c, -1, 0) - np.roll(c, 1, 0)
        tang /= np.linalg.norm(tang, axis=1, keepdims=True)
        nor = np.column_stack([-tang[:, 1], tang[:, 0]])
        for p in pots:
            rel = c - p
            d = np.linalg.norm(rel, axis=1)
            lat = (rel * nor).sum(1)
            sign = np.where(lat >= 0, 1.0, -1.0)
            push = np.clip(need - d, 0, None) * 2.0
            c = c + nor * (sign * push)[:, None]
        c = smooth_closed(resample(c), 11, 2)
    clear = min(np.min(np.linalg.norm(c - p, axis=1)) for p in pots)
    print(f'  path: {len(c) * 0.05:.1f} m, closest pot centre {clear:.2f} m from centre line')
    return c


def make_path():
    c = path_centreline()
    n = len(c)
    tang = np.roll(c, -1, 0) - np.roll(c, 1, 0)
    tang /= np.linalg.norm(tang, axis=1, keepdims=True)
    nor = np.column_stack([-tang[:, 1], tang[:, 0]])
    s = np.arange(n) * 0.05
    hw = 0.30 + 0.035 * np.sin(s * 0.9) + 0.02 * np.sin(s * 2.3 + 1.0)
    wob_l = 0.012 * np.sin(s * 7.1) + 0.008 * np.sin(s * 13.0 + 2)
    wob_r = 0.012 * np.sin(s * 6.3 + 1) + 0.008 * np.sin(s * 11.0)
    idx = np.r_[np.arange(n), 0]

    def strip(off_a, off_b, z, tile):
        a = c + nor * off_a[:, None]
        b = c + nor * off_b[:, None]
        P = np.stack([np.column_stack([a[idx], np.full(n + 1, z)]),
                      np.column_stack([b[idx], np.full(n + 1, z)])], 1)
        return grid_mesh(P, P[..., :2] / tile).ensure(0, (0, 0, 1))

    # The robots drive on the physics ground at z = 0 along this path, so every layer is
    # at most 3 mm high (it was up to 12 mm and the wheels looked sunk into it).
    out = {}
    out['path_gravel'] = strip(hw + wob_l, -(hw + wob_r), 0.0012, 1.0).save('path_gravel')
    edge_w = 0.035
    out['path_edge'] = Mesh.merge([strip(hw + wob_l + edge_w, hw + wob_l - 0.005, 0.0018, 0.5),
                                   strip(-(hw + wob_r) + 0.005, -(hw + wob_r) - edge_w, 0.0018, 0.5)]
                                  ).save('path_edge')
    rng = np.random.default_rng(3301)
    pots = np.array([(x, y) for _, x, y in STATIC_OBSTACLES])
    stones = []
    pos = 0.3
    while pos < s[-1] - 0.3:
        i = int(pos / 0.05)
        centre = c[i] + nor[i] * rng.uniform(-0.06, 0.06)
        if np.min(np.linalg.norm(pots - centre, axis=1)) > 0.36:
            rad = rng.uniform(0.15, 0.2) * min(1.0, hw[i] / 0.3)
            poly = blob_polygon(centre[0], centre[1], rad * rng.uniform(1.0, 1.2), rad * rng.uniform(0.8, 1.0),
                                int(rng.integers(1e6)), k=24, amp=0.14,
                                rot=math.atan2(tang[i, 1], tang[i, 0]) + rng.uniform(-0.6, 0.6), power=2.4)
            stones.append(extrude(poly, 0.0008, 0.0024 + rng.uniform(0, 0.0006), tile=0.9,
                                  uv_offset=rng.uniform(0, 5, 2)))
        pos += rng.uniform(0.52, 0.68)
    out['path_stones'] = Mesh.merge(stones).save('path_stones')
    return out


# =============================================================================
# SDF writing
# =============================================================================

def fmt(*vals):
    return ' '.join(f'{v:.6g}' for v in vals)


def material(albedo=None, normal=None, color=(1, 1, 1), rough=0.9, metal=0.0, emissive=None,
             specular=0.06, double_sided=False, rough_map=None):
    c = fmt(*color)
    sp = fmt(specular, specular, specular)
    xml = [f'<material><ambient>{c} 1</ambient><diffuse>{c} 1</diffuse><specular>{sp} 1</specular>']
    if emissive:
        xml.append(f'<emissive>{fmt(*emissive)} 1</emissive>')
    if double_sided:
        xml.append('<double_sided>true</double_sided>')
    xml.append(f'<pbr><metal><roughness>{rough}</roughness><metalness>{metal}</metalness>')
    if albedo:
        xml.append(f'<albedo_map>{albedo}</albedo_map>')
    if normal:
        xml.append(f'<normal_map>{normal}</normal_map>')
    if rough_map:
        xml.append(f'<roughness_map>{rough_map}</roughness_map>')
    xml.append('</metal></pbr></material>')
    return ''.join(xml)


def visual(name, geometry, mat, pose=(0, 0, 0, 0, 0, 0), shadows=True):
    sh = '' if shadows else '<cast_shadows>false</cast_shadows>'
    return (f'<visual name="{name}"><pose>{fmt(*pose)}</pose>{sh}'
            f'<geometry>{geometry}</geometry>{mat}</visual>')


def mesh_geom(uri, scale=None):
    sc = f'<scale>{fmt(*scale)}</scale>' if scale is not None else ''
    return f'<mesh><uri>{uri}</uri>{sc}</mesh>'


def model(name, pose, body, static=True, comment=None):
    head = f'    <!-- {comment} -->\n' if comment else ''
    return (f'{head}    <model name="{name}">\n      <static>{str(static).lower()}</static>\n'
            f'      <pose>{fmt(*pose)}</pose>\n      <link name="link">\n        '
            + '\n        '.join(body) + '\n      </link>\n    </model>\n')


def photo(T, key, color=(1, 1, 1)):
    """PBR material from a downloaded photo-scanned texture set."""
    return material(T[key], T[key + '_n'], color=color, rough=1.0, rough_map=T[key + '_r'])


def scanned(name, asset, fit, mat=''):
    """Visual of a Poly Haven glTF pot, fitted by fit_scanned_pots (model frame at z = 0.15)."""
    sxy, sz, h, c = fit['sxy'], fit['sz'], fit['plinth'], fit['centre']
    uri = f'{EXT}/polyhaven/{asset}/{asset}.gltf'
    pose = (-c[0] * sxy, -c[1] * sxy, h - c[2] * sz - 0.15, math.pi / 2, 0, 0)    # glTF is Y-up
    return visual(name, mesh_geom(uri, (sxy, sz, sxy)), mat, pose)


def build_world(T, M):
    mats = {
        'grass': photo(T, 'grass', color=(0.86, 1.0, 0.80)),
        'meadow': photo(T, 'grass', color=(0.78, 0.92, 0.74)),
        'gravel': photo(T, 'gravel'),
        'stone': photo(T, 'stone', color=(0.72, 0.72, 0.74)),
        'soil': photo(T, 'soil'),
        'mulch': photo(T, 'mulch'),
        'patio': photo(T, 'patio'),
        'hedge': material(T['hedge'], T['hedge_n'], rough=0.85),
        'bark': photo(T, 'bark'),
        'rings': material(T['rings'], None, rough=0.9),
        'foliage': material(T['foliage'], T['foliage_n'], rough=0.75),
        'pink': material(T['pink'], T['pink_n'], rough=0.8),
        'lavender': material(T['lavender'], T['lavender_n'], rough=0.8),
        'white': material(T['white'], T['white_n'], rough=0.8),
        'yellow': material(T['yellow'], T['yellow_n'], rough=0.8),
        'concrete': photo(T, 'concrete'),
        'barrel': material(T['barrel'], None, rough=0.9),
        'iron': material(None, None, color=(0.12, 0.11, 0.10), rough=0.7, metal=0.3),
        'teak': material(T['teak'], T['teak_n'], rough=0.8),
        'cedar': material(T['cedar'], T['cedar_n'], rough=0.9),
        'pergola_wood': material(T['oakwood'], T['oakwood_n'], rough=0.85),
        'grass_blade': material(None, None, color=(0.42, 0.50, 0.22), rough=0.8, double_sided=True),
        'charcoal_metal': material(None, None, color=(0.09, 0.09, 0.10), rough=0.55, metal=0.35),
        'lamp_glass': material(None, None, color=(0.95, 0.85, 0.6), rough=0.3, emissive=(0.55, 0.42, 0.2)),
        'water': material(None, None, color=(0.22, 0.30, 0.33), rough=0.12, specular=0.3),
        # Garden robots: matte painted housings.
        'r_green': material(None, None, color=(0.17, 0.40, 0.19), rough=0.62),
        'r_charcoal': material(None, None, color=(0.16, 0.17, 0.18), rough=0.6),
        'r_yellow': material(None, None, color=(0.86, 0.66, 0.12), rough=0.6),
        'r_black': material(None, None, color=(0.05, 0.05, 0.055), rough=0.75),
        'r_rubber': material(None, None, color=(0.09, 0.09, 0.09), rough=0.9),
        'r_hub': material(None, None, color=(0.45, 0.46, 0.47), rough=0.5, metal=0.2),
        'r_led_amber': material(None, None, color=(1.0, 0.7, 0.2), rough=0.3, emissive=(1.0, 0.62, 0.1)),
        'r_led_green': material(None, None, color=(0.3, 1.0, 0.45), rough=0.3, emissive=(0.2, 0.9, 0.35)),
        'r_led_blue': material(None, None, color=(0.5, 0.8, 1.0), rough=0.3, emissive=(0.35, 0.65, 1.0)),
        'r_led_red': material(None, None, color=(0.9, 0.15, 0.1), rough=0.4, emissive=(0.6, 0.06, 0.04)),
    }
    mg = mesh_geom
    out = []

    # ---------------- Ground --------------------------------------------------------
    ground_coll = ('<collision name="ground_collision"><pose>{} 0 0 0</pose>'
                   '<geometry><box><size>{} {} 0.02</size></box></geometry>'
                   '<surface><contact><collide_bitmask>0xFFFF</collide_bitmask></contact></surface>'
                   '</collision>').format(fmt((YARD_X0 + YARD_X1) / 2, (YARD_Y0 + YARD_Y1) / 2, -0.01),
                                         fmt(YARD_X1 - YARD_X0), fmt(YARD_Y1 - YARD_Y0))
    out.append(model('Garden_Ground', (0, 0, 0, 0, 0, 0), [
        ground_coll,
        visual('lawn', mg(M['lawn']), mats['grass'], shadows=False),
        visual('meadow', mg(M['meadow']), mats['meadow'], shadows=False),
    ], comment='Flat lawn. The 12 x 8 m operating area is the same as in simulation_world.sdf.'))

    out.append(model('Garden_Path', (0, 0, 0, 0, 0, 0), [
        visual('gravel', mg(M['path_gravel']), mats['gravel'], shadows=False),
        visual('edging', mg(M['path_edge']), mats['soil'], shadows=False),
        visual('stepping_stones', mg(M['path_stones']), mats['stone'], shadows=False),
    ], comment='Visual only, max 3 mm high: no collision, below the LiDAR scan plane.'))

    # ---------------- Hedge (collision boxes, inner face 7 cm outside the area) ------
    coll = []
    ext = HEDGE_GAP + HEDGE_T
    hz = HEDGE_H / 2
    boxes = [
        ((AREA_X0 + AREA_X1) / 2, AREA_Y0 - HEDGE_GAP - HEDGE_T / 2, AREA_X1 - AREA_X0 + 2 * ext, HEDGE_T),
        ((AREA_X0 + AREA_X1) / 2, AREA_Y1 + HEDGE_GAP + HEDGE_T / 2, AREA_X1 - AREA_X0 + 2 * ext, HEDGE_T),
        (AREA_X0 - HEDGE_GAP - HEDGE_T / 2, (AREA_Y0 + AREA_Y1) / 2, HEDGE_T, AREA_Y1 - AREA_Y0),
        (AREA_X1 + HEDGE_GAP + HEDGE_T / 2, (AREA_Y0 + AREA_Y1) / 2, HEDGE_T, AREA_Y1 - AREA_Y0),
    ]
    for i, (x, y, sx, sy) in enumerate(boxes):
        coll.append(f'<collision name="hedge_collision_{i}"><pose>{fmt(x, y, hz)} 0 0 0</pose>'
                    f'<geometry><box><size>{fmt(sx, sy, HEDGE_H)}</size></box></geometry>'
                    '<surface><contact><collide_bitmask>0xFFFF</collide_bitmask></contact></surface></collision>')
    out.append(model('Garden_Hedge', (0, 0, 0, 0, 0, 0),
                     coll + [visual('hedge', mg(M['hedge']), mats['hedge'])],
                     comment='Continuous 0.55 m hedge just outside the operating area (LiDAR boundary).'))

    # ---------------- Static obstacles ----------------------------------------------
    cyl_coll = ('<collision name="{0}_collision"><pose>0 0 0 0 0 0</pose><geometry><cylinder><radius>0.1</radius>'
                '<length>0.3</length></cylinder></geometry><surface><contact><collide_bitmask>0x01'
                '</collide_bitmask></contact></surface></collision>')
    off = (0, 0, -0.15, 0, 0, 0)
    soil = visual('soil', mg(M['soil_disc']), mats['soil'], off, shadows=False)
    fits = M['fits']

    def plinth(asset):
        if fits[asset]['plinth'] <= 0.0:
            return []
        return [visual('plinth', mg(M[f'plinth_{asset}']), mats['concrete'], off)]

    clay = fits['planter_pot_clay']
    designs = [
        plinth('potted_plant_01') + [scanned('potted_plant', 'potted_plant_01', fits['potted_plant_01'])],
        plinth('potted_plant_04') + [scanned('potted_plant', 'potted_plant_04', fits['potted_plant_04'])],
        [visual('pot', mg(M['pot_concrete']), mats['concrete'], off), soil,
         visual('topiary', mg(M['boxwood']), mats['hedge'], off)],
        [visual('stump', mg(M['stump']), mats['bark'], off),
         visual('rings', mg(M['stump_top']), mats['rings'], off, shadows=False)],
        plinth('potted_plant_02') + [scanned('potted_plant', 'potted_plant_02', fits['potted_plant_02'])],
        [scanned('pot', 'planter_pot_clay', clay),
         visual('soil', mg(M['soil_disc'], (1.05, 1.05, 1.0)), mats['soil'], off, shadows=False),
         visual('grass', mg(M['grass_tuft']), mats['grass_blade'], off)],
        [visual('barrel', mg(M['barrel']), mats['barrel'], off),
         visual('hoops', mg(M['hoops']), mats['iron'], off), soil,
         visual('plant', mg(M['plant_barrel']), mats['yellow'], off)],
    ]
    labels = ['scanned terracotta bowl with leafy plant on a plinth', 'scanned white pot with succulent',
              'concrete planter with box ball', 'tree stump', 'scanned terracotta pot with plant on a plinth',
              'scanned clay long-tom pot with ornamental grass', 'half-barrel planter with marigolds']
    rng = np.random.default_rng(77)
    for (name, x, y), parts, label in zip(STATIC_OBSTACLES, designs, labels):
        out.append(model(name, (x, y, 0.15, 0, 0, rng.uniform(-math.pi, math.pi)),
                         [cyl_coll.format(name)] + parts,
                         comment=f'Static obstacle: {label}. Collision r 0.10 h 0.30.'))

    # ---------------- Moving obstacles: garden robots ---------------------------------
    def robot(variant):
        o = (0, 0, -0.15, 0, 0, 0)
        v = [visual('chassis', mg(M['robot_chassis']), mats['r_black'], o),
             visual('bumper', mg(M['robot_bumper']), mats['r_rubber'] if variant != 1 else mats['r_yellow'], o)]
        for i, (wx, wy) in enumerate(((0.052, 0.072), (0.052, -0.072), (-0.052, 0.072), (-0.052, -0.072))):
            v.append(visual(f'wheel_{i}', '<cylinder><radius>0.032</radius><length>0.022</length></cylinder>',
                            mats['r_rubber'], (wx, wy, 0.032 - 0.15, math.pi / 2, 0, 0)))
            v.append(visual(f'hub_{i}', '<cylinder><radius>0.014</radius><length>0.004</length></cylinder>',
                            mats['r_hub'], (wx, wy + (0.012 if wy > 0 else -0.012), 0.032 - 0.15, math.pi / 2, 0, 0),
                            shadows=False))
        if variant == 0:      # green mower, charcoal bumper, red stop button, amber LED
            v += [visual('body', mg(M['robot_body_a']), mats['r_green'], o),
                  visual('stop_button', '<cylinder><radius>0.022</radius><length>0.012</length></cylinder>',
                         mats['r_led_red'], (0, 0, 0.292 - 0.15, 0, 0, 0)),
                  visual('led_front', '<box><size>0.006 0.05 0.012</size></box>', mats['r_led_amber'],
                         (0.0955, 0, 0.215 - 0.15, 0, 0, 0), shadows=False),
                  visual('band', mg(M['robot_band']), mats['r_charcoal'], o, shadows=False)]
        elif variant == 1:    # charcoal rover, yellow bumper/stripe, sensor puck, green LEDs
            v += [visual('body', mg(M['robot_body_b']), mats['r_charcoal'], o),
                  visual('band', mg(M['robot_band']), mats['r_yellow'], o, shadows=False),
                  visual('sensor_puck', '<cylinder><radius>0.034</radius><length>0.03</length></cylinder>',
                         mats['r_black'], (0, 0, 0.284 - 0.15, 0, 0, 0)),
                  visual('led_left', '<box><size>0.006 0.016 0.01</size></box>', mats['r_led_green'],
                         (0.095, 0.03, 0.215 - 0.15, 0, 0, 0.3), shadows=False),
                  visual('led_right', '<box><size>0.006 0.016 0.01</size></box>', mats['r_led_green'],
                         (0.095, -0.03, 0.215 - 0.15, 0, 0, -0.3), shadows=False)]
        else:                 # yellow scout, black dome, blue "eyes", amber status light
            v += [visual('body', mg(M['robot_body_c']), mats['r_yellow'], o),
                  visual('dome', mg(M['robot_dome_c']), mats['r_black'], o),
                  visual('eye_left', '<box><size>0.006 0.018 0.012</size></box>', mats['r_led_blue'],
                         (0.0955, 0.028, 0.2 - 0.15, 0, 0, 0.28), shadows=False),
                  visual('eye_right', '<box><size>0.006 0.018 0.012</size></box>', mats['r_led_blue'],
                         (0.0955, -0.028, 0.2 - 0.15, 0, 0, -0.28), shadows=False),
                  visual('status', '<cylinder><radius>0.012</radius><length>0.008</length></cylinder>',
                         mats['r_led_amber'], (-0.03, 0, 0.297 - 0.15, 0, 0, 0), shadows=False)]
        return v

    rng = np.random.default_rng(88)
    for i, (name, x, y) in enumerate(MOVING_OBSTACLES):
        coll = cyl_coll.format(name)
        inertial = ('<gravity>false</gravity><inertial><mass>1.0</mass><inertia><ixx>0.01</ixx>'
                    '<iyy>0.01</iyy><izz>0.005</izz></inertia></inertial>')
        body = [inertial, coll] + robot(i % 3)
        out.append(model(name, (x, y, 0.15, 0, 0, rng.uniform(-math.pi, math.pi)), body,
                         comment=f'Moving obstacle: garden robot variant {"ABC"[i % 3]}. '
                                 'Collision r 0.10 h 0.30, moved by the random_obstacles package.'))

    # ---------------- Background: flower beds, shrubs ---------------------------------
    out.append(background(M, mats))
    return out


def background(M, mats):
    mg = mesh_geom
    v = []
    rng = np.random.default_rng(1234)
    hedge_out = HEDGE_GAP + HEDGE_T

    # Flower beds (mulch slabs 3 cm, outside the hedge).
    beds = [
        ('bed_south', blob_polygon(1.9, AREA_Y0 - hedge_out - 0.55, 5.4, 0.42, 11, amp=0.08, power=4.0)),
        ('bed_north_w', blob_polygon(-1.2, AREA_Y1 + hedge_out + 0.6, 2.4, 0.45, 12, amp=0.1, power=4.0)),
        ('bed_north_e', blob_polygon(5.3, AREA_Y1 + hedge_out + 0.6, 2.2, 0.45, 13, amp=0.1, power=4.0)),
        ('bed_east', blob_polygon(AREA_X1 + hedge_out + 0.6, 3.6, 0.45, 2.0, 14, amp=0.1, power=4.0)),
        ('bed_west', blob_polygon(AREA_X0 - hedge_out - 0.6, 1.0, 0.45, 2.2, 15, amp=0.1, power=4.0)),
        ('ring_oak_nw', blob_polygon(-2.8, 9.3, 1.25, 1.15, 16, amp=0.08)),
        ('ring_oak_ne', blob_polygon(7.4, 9.5, 1.2, 1.1, 17, amp=0.08)),
        ('ring_oak_e', blob_polygon(11.4, 6.6, 1.1, 1.0, 18, amp=0.08)),
        ('ring_oak_sw', blob_polygon(-7.0, -3.2, 1.0, 1.0, 19, amp=0.08)),
    ]
    bed_meshes = [extrude(p, -0.01, 0.03, tile=1.0) for _, p in beds]
    v.append(visual('flower_beds', mg(Mesh.merge(bed_meshes).save('flower_beds')), mats['mulch'], shadows=False))

    # Patio under the pergola and pad around the bird bath (flat stone/gravel).
    patio = extrude(np.array([(-7.6, 6.5), (-4.6, 6.5), (-4.6, 8.9), (-7.6, 8.9)]), -0.01, 0.02, tile=1.6)
    v.append(visual('patio', mg(patio.save('patio')), mats['patio'], shadows=False))
    pad = extrude(blob_polygon(10.0, 0.9, 0.75, 0.75, 21, amp=0.05), -0.01, 0.012, tile=1.0)
    pad2 = extrude(blob_polygon(2.0, 7.55, 1.3, 0.7, 22, amp=0.05, power=3.0), -0.01, 0.012, tile=1.0)
    v.append(visual('gravel_pads', mg(Mesh.merge([pad, pad2]).save('gravel_pads')), mats['gravel'], shadows=False))

    # Shrubs on the beds (all outside the hedge).
    def shrubs(cx0, cx1, cy0, cy1, count, mats_cycle, size, seed):
        r = np.random.default_rng(seed)
        res = []
        for k in range(count):
            t = (k + r.uniform(0.2, 0.8)) / count
            x = cx0 + (cx1 - cx0) * t
            y = cy0 + (cy1 - cy0) * t + r.uniform(-0.1, 0.1)
            s = r.uniform(*size)
            res.append((x, y, s, mats_cycle[k % len(mats_cycle)], int(r.integers(0, 4))))
        return res

    sy = AREA_Y0 - hedge_out - 0.55
    ny = AREA_Y1 + hedge_out + 0.6
    ex = AREA_X1 + hedge_out + 0.6
    wx = AREA_X0 - hedge_out - 0.6
    plants = []
    plants += shrubs(-3.0, 6.8, sy, sy, 13, ['pink', 'foliage', 'yellow', 'hedge', 'white', 'foliage'],
                     (0.25, 0.38), 31)
    plants += shrubs(-3.3, 0.9, ny, ny, 6, ['pink', 'foliage', 'white', 'hedge'], (0.3, 0.5), 32)
    plants += shrubs(3.3, 7.3, ny, ny, 6, ['white', 'foliage', 'pink', 'yellow'], (0.3, 0.5), 33)
    plants += shrubs(ex, ex, 1.9, 5.4, 5, ['foliage', 'pink', 'hedge', 'yellow'], (0.3, 0.45), 34)
    plants += shrubs(wx, wx, -1.0, 3.0, 5, ['white', 'foliage', 'yellow', 'pink'], (0.3, 0.45), 35)
    for k, (x, y, s, mname, mesh_i) in enumerate(plants):
        v.append(visual(f'shrub_{k}', mg(M[f'shrub_{mesh_i}'], (s, s, s)), mats[mname],
                        (x, y, s * 0.45, 0, 0, rng.uniform(0, 6.28)), shadows=False))
    # Larger evergreen shrubs at the yard corners / next to the fence.
    big = [(-8.0, -4.5, 0.75), (11.8, -4.6, 0.8), (-8.1, 2.8, 0.7), (12.0, 6.4, 0.75), (-4.4, 10.6, 0.8),
           (4.2, 10.7, 0.7), (9.3, 10.6, 0.8), (-8.0, 5.6, 0.65)]
    for k, (x, y, s) in enumerate(big):
        v.append(visual(f'evergreen_{k}', mg(M[f'shrub_{k % 4}'], (s, s, s * 1.1)),
                        mats['hedge' if k % 2 else 'foliage'], (x, y, s * 0.5, 0, 0, rng.uniform(0, 6.28))))

    # Trees (sun shadows fall north-east, away from the lawn).
    trees = [('oak', 1, -2.8, 9.3, 1.0), ('oak', 2, 7.4, 9.5, 0.9), ('oak', 1, 11.4, 6.6, 0.75),
             ('oak', 2, -7.0, -3.2, 0.78), ('pine', 1, 1.6, 10.2, 0.95), ('pine', 2, -7.4, 4.4, 0.85),
             ('pine', 1, 11.5, 8.4, 0.8), ('pine', 2, 12.0, 1.6, 0.85), ('pine', 1, -7.7, 9.8, 1.05),
             # neighbours' trees beyond the fence
             ('pine', 2, -12.5, 14.0, 1.2), ('oak', 1, 4.5, 16.0, 1.3), ('pine', 1, 17.0, 6.0, 1.15),
             ('oak', 2, 19.0, 9.0, 1.2), ('pine', 2, -14.0, 1.0, 1.25), ('oak', 2, -6.0, 16.5, 1.1),
             ('pine', 1, 12.0, 15.5, 1.3), ('oak', 1, -13.0, -9.0, 1.1), ('pine', 2, 6.0, -11.0, 1.2)]
    fuel = {'oak': ('Oak_tree', 'oak_tree.dae', 'branch_diffuse.png'),
            'pine': ('Pine_Tree', 'pine_tree.dae', 'branch_2_diffuse.png')}
    for k, (kind, _var, x, y, s) in enumerate(trees):
        folder, dae, leaves = fuel[kind]
        uri = f'{EXT}/fuel/{folder}/meshes/{dae}'
        pose = (x, y, 0, 0, 0, rng.uniform(0, 6.28))
        s = s * 0.9
        for sub, tex, double in (('Branch', leaves, True), ('Bark', 'bark_diffuse.png', False)):
            geom = (f'<mesh><uri>{uri}</uri><submesh><name>{sub}</name></submesh>'
                    f'<scale>{fmt(s, s, s)}</scale></mesh>')
            mat = material(f'{EXT}/fuel/{folder}/materials/textures/{tex}', None, rough=0.9,
                           double_sided=double)
            v.append(visual(f'tree_{k}_{sub.lower()}', geom, mat, pose))

    # Pergola corner with climbing roses and a bench, lamp post, bird bath, compost bins, fence.
    v.append(visual('pergola', mg(M['pergola']), mats['pergola_wood'], (-6.1, 7.7, 0, 0, 0, 0)))
    for j, x in enumerate(np.linspace(-7.2, -5.0, 4)):
        v.append(visual(f'climber_top_{j}', mg(M[f'shrub_{j % 4}'], (0.45, 0.35, 0.22)), mats['pink'],
                        (x, 6.6 + (j % 2) * 2.2, 2.45, 0, 0, j), shadows=False))
    v.append(visual('lamp_base', '<cylinder><radius>0.11</radius><length>0.3</length></cylinder>',
                    mats['charcoal_metal'], (-4.95, 6.95, 0.15, 0, 0, 0)))
    v.append(visual('lamp_pole', '<cylinder><radius>0.038</radius><length>2.2</length></cylinder>',
                    mats['charcoal_metal'], (-4.95, 6.95, 1.3, 0, 0, 0)))
    v.append(visual('lamp_glass', '<box><size>0.19 0.19 0.28</size></box>', mats['lamp_glass'],
                    (-4.95, 6.95, 2.38, 0, 0, 0.785), shadows=False))
    v.append(visual('lamp_frame', '<box><size>0.24 0.24 0.03</size></box>', mats['charcoal_metal'],
                    (-4.95, 6.95, 2.235, 0, 0, 0.785)))
    v.append(visual('lamp_cap', mg(M['lamp_cap']), mats['charcoal_metal'], (-4.95, 6.95, 0, 0, 0, 0)))
    v.append(visual('lamp_finial', '<sphere><radius>0.03</radius></sphere>', mats['charcoal_metal'],
                    (-4.95, 6.95, 2.66, 0, 0, 0)))
    v.append(visual('birdbath', mg(M['birdbath']), mats['concrete'], (10.0, 0.9, 0.012, 0, 0, 0)))
    v.append(visual('birdbath_water', mg(M['water']), mats['water'], (10.0, 0.9, 0.012, 0, 0, 0), shadows=False))
    v.append(visual('bench_north', mg(M['bench']), mats['teak'], (2.0, 7.75, 0.012, 0, 0, 0)))
    v.append(visual('compost', mg(M['compost']), mats['cedar'], (11.3, 10.3, 0, 0, 0, 0.1)))
    v.append(visual('compost_heap', mg(M['shrub_1'], (0.42, 0.42, 0.35)), mats['mulch'], (11.3, 10.3, 0.35, 0, 0, 0),
                    shadows=False))
    v.append(visual('compost_2', mg(M['compost']), mats['cedar'], (10.2, 10.35, 0, 0, 0, -0.05)))
    v.append(visual('compost_heap_2', mg(M['shrub_2'], (0.42, 0.42, 0.3)), mats['mulch'],
                    (10.2, 10.35, 0.3, 0, 0, 0), shadows=False))
    v.append(visual('fence', mg(M['fence']), mats['cedar']))

    # Photo-scanned props (Poly Haven, CC0); glTF is Y-up, hence the 90 deg roll.
    def prop(name, asset, x, y, yaw, z=0.0, s=1.0):
        uri = f'{EXT}/polyhaven/{asset}/{asset}.gltf'
        return visual(name, mesh_geom(uri, (s, s, s)), '', (x, y, z, math.pi / 2, 0, yaw))
    v += [prop('patio_set', 'outdoor_table_chair_set_01', -6.4, 7.5, 0.3, z=0.02),
          prop('lantern', 'wooden_lantern_01', -5.0, 8.6, 0.4, z=0.02, s=0.8),
          prop('planter_box_a', 'planter_box_01', -7.1, 6.25, 0.0),
          prop('planter_box_b', 'planter_box_03', -7.95, 8.4, 1.5708),
          prop('gnome', 'garden_gnome', -2.3, AREA_Y0 - HEDGE_GAP - HEDGE_T - 0.6, 2.6, z=0.03, s=0.7),
          prop('rain_barrel', 'Barrel_02', 9.3, 10.55, 0.0),
          prop('watering_can', 'watering_can_metal_01', 9.75, 10.0, 2.2),
          prop('watering_can_2', 'watering_can_metal_01', 10.4, 0.55, -0.8)]
    for k, (x, y) in enumerate(((-7.1, 6.25), (-7.95, 8.4))):
        v.append(visual(f'planter_box_plants_{k}', mg(M[f'shrub_{k + 1}'], (0.38, 0.38, 0.3) if k == 0 else
                                                   (0.3, 0.38, 0.32)), mats['pink' if k else 'white'],
                        (x, y, 0.45 if k == 0 else 0.86, 0, 0, k), shadows=False))
    return model('Garden_Background', (0, 0, 0, 0, 0, 0), v,
                 comment='Everything outside the hedge: beds, trees, pergola, bench, lamp, bird bath, compost, fence.')


GUI = '''    <gui fullscreen="0">
      <plugin filename="MinimalScene" name="3D View">
        <gz-gui><title>3D View</title><property type="bool" key="showTitleBar">false</property>
          <property type="string" key="state">docked</property></gz-gui>
        <engine>ogre2</engine>
        <scene>scene</scene>
        <ambient_light>0.45 0.47 0.5</ambient_light>
        <background_color>0.62 0.76 0.92</background_color>
        <camera_pose>{cam}</camera_pose>
        <camera_clip><near>0.05</near><far>400</far></camera_clip>
      </plugin>
{floating}
      <plugin filename="WorldControl" name="World control">
        <gz-gui><title>World control</title><property type="bool" key="showTitleBar">false</property>
          <property type="bool" key="resizable">false</property><property type="double" key="height">72</property>
          <property type="double" key="z">1</property><property type="string" key="state">floating</property>
          <anchors target="3D View"><line own="left" target="left"/><line own="bottom" target="bottom"/></anchors>
        </gz-gui>
        <play_pause>true</play_pause><step>true</step><start_paused>true</start_paused><use_event>true</use_event>
      </plugin>
      <plugin filename="WorldStats" name="World stats">
        <gz-gui><title>World stats</title><property type="bool" key="showTitleBar">false</property>
          <property type="bool" key="resizable">false</property><property type="double" key="height">110</property>
          <property type="double" key="width">290</property><property type="double" key="z">1</property>
          <property type="string" key="state">floating</property>
          <anchors target="3D View"><line own="right" target="right"/><line own="bottom" target="bottom"/></anchors>
        </gz-gui>
        <sim_time>true</sim_time><real_time>true</real_time><real_time_factor>true</real_time_factor>
        <iterations>true</iterations>
      </plugin>
      <plugin filename="ComponentInspector" name="Component inspector">
        <gz-gui><property type="bool" key="showTitleBar">false</property>
          <property type="string" key="state">docked</property></gz-gui>
      </plugin>
      <plugin filename="EntityTree" name="Entity tree">
        <gz-gui><property type="bool" key="showTitleBar">false</property>
          <property type="string" key="state">docked</property></gz-gui>
      </plugin>
    </gui>
'''

FLOATING_PLUGINS = ['EntityContextMenuPlugin', 'GzSceneManager', 'InteractiveViewControl', 'CameraTracking',
                    'MarkerManager', 'SelectEntities', 'Spawn', 'VisualizationCapabilities']


def gui_block():
    eye = np.array([14.0, -9.0, 8.0])
    target = np.array([1.9, 2.6, 0.0])
    d = target - eye
    yaw = math.atan2(d[1], d[0])
    pitch = math.atan2(-d[2], math.hypot(d[0], d[1]))
    floating = '\n'.join(
        f'      <plugin filename="{p}" name="{p}"><gz-gui><property key="resizable" type="bool">false</property>'
        '<property key="width" type="double">5</property><property key="height" type="double">5</property>'
        '<property key="state" type="string">floating</property>'
        '<property key="showTitleBar" type="bool">false</property></gz-gui></plugin>' for p in FLOATING_PLUGINS)
    return GUI.format(cam=fmt(*eye, 0, pitch, yaw), floating=floating)


def write_world(models):
    sun_dir = np.array([0.32, 0.55, -0.78])
    sun_dir /= np.linalg.norm(sun_dir)
    header = f'''<?xml version="1.0"?>
<!--
  The Autonomous Backyard - garden version of simulation_world.sdf.
  GENERATED by simulation/garden/build_garden_world.py (do not edit by hand).

  Same world name, operating area (12 x 8 m centred at 1.9, 2.1), obstacle names,
  sizes and start positions as simulation_world.sdf.  Inside the area, every visual is
  either flat on the ground (<= 3 mm) or one of the {len(STATIC_OBSTACLES) + len(MOVING_OBSTACLES)} cylinder obstacles
  (collision r 0.10 m, h 0.30 m).  The hedge (0.55 m) is the LiDAR boundary.
-->
<sdf version="1.9">
  <world name="{WORLD_NAME}">

    <physics name="physics" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <gravity>0 0 -9.81</gravity>

    <scene>
      <ambient>0.45 0.47 0.5 1</ambient>
      <background>0.62 0.76 0.92 1</background>
      <shadows>true</shadows>
      <grid>false</grid>
      <sky>
        <clouds><speed>0.6</speed><humidity>0.35</humidity><mean_size>0.4</mean_size></clouds>
      </sky>
    </scene>

    <!-- Late-morning sun: ~51 deg elevation, shadows fall north-east (away from the lawn). -->
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 30 0 0 0</pose>
      <diffuse>0.98 0.94 0.86 1</diffuse>
      <specular>0.18 0.18 0.17 1</specular>
      <intensity>1.0</intensity>
      <direction>{fmt(*sun_dir)}</direction>
    </light>

    <!-- Soft sky fill so shaded sides stay visible (no shadows, no specular). -->
    <light name="sky_fill" type="directional">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 30 0 0 0</pose>
      <diffuse>0.24 0.27 0.32 1</diffuse>
      <specular>0 0 0 1</specular>
      <direction>-0.35 -0.45 -0.82</direction>
    </light>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

{gui_block()}
'''
    WORLD_FILE.write_text(header + '\n'.join(models) + '\n  </world>\n</sdf>\n')


def main():
    print('textures ...')
    tex = make_textures()
    print('meshes ...')
    meshes = make_meshes()
    print('world ...')
    write_world(build_world(tex, meshes))
    size = sum(p.stat().st_size for p in (HERE / 'assets').rglob('*') if p.is_file())
    print(f'wrote {WORLD_FILE} (assets {size / 1e6:.1f} MB)')


if __name__ == '__main__':
    main()
