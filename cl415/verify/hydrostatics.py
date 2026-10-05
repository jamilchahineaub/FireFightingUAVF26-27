"""expected floating attitude of the cl415 from its collision boxes (what gazebo's graded buoyancy sees)

solves for base_link height z, roll and pitch with buoyancy = weight and no moment about the cg,
water surface at z = 0, density 1000. also reports the draft of the real CAD hull for comparison.
writes verify/out/hydrostatics.json

run from the project root:  .venv/Scripts/python verify/hydrostatics.py
"""
import json
import math
import os

import numpy as np
from scipy.optimize import least_squares

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GEO = json.load(open(os.path.join(ROOT, 'cad', 'geometry.json')))
MASS = json.load(open(os.path.join(ROOT, 'mass', 'mass_props.json')))
RHO, G = 1000.0, 9.81


def rot(roll, pitch):
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    return Ry @ Rx


def rpy_m(r, p, y):
    return rot(r, p) if abs(y) < 1e-12 else None


# every collision box as its 8 corners in the base frame. only boxes that can reach the water matter;
# the moving links sit high, so their link frames are taken at zero deflection
from itertools import product

from scipy.spatial import ConvexHull

BOXES = []
for link, cs in GEO['collision'].items():
    L = GEO['links'][link]
    p0 = np.array(L['xyz'], float)
    RL = rot(L['rpy'][0], L['rpy'][1])
    for c in cs:
        if c['type'] != 'box':
            continue
        if abs(c['rpy'][2]) > 1e-12:
            raise ValueError('yawed box')
        h = np.array(c['size']) / 2
        corners = np.array([[sx * h[0], sy * h[1], sz * h[2]] for sx, sy, sz in product((-1, 1), repeat=3)])
        BOXES.append(p0 + (corners @ rot(c['rpy'][0], c['rpy'][1]).T + np.array(c['xyz'])) @ RL.T)
EDGES = [(i, j) for i in range(8) for j in range(i + 1, 8) if bin(i ^ j).count('1') == 1]


def clip_below(C):
    """volume and centroid of a convex box (corners C, world frame) below z = 0, exactly"""
    below = C[:, 2] < 0
    if not below.any():
        return 0.0, np.zeros(3)
    if below.all():
        return float(ConvexHull(C).volume), C.mean(0)
    pts = list(C[below])
    for i, j in EDGES:
        a, b = C[i], C[j]
        if (a[2] < 0) != (b[2] < 0):
            t = a[2] / (a[2] - b[2])
            pts.append(a + t * (b - a))
    pts = np.array(pts)
    hull = ConvexHull(pts)
    c0 = pts.mean(0)
    vol, mom = 0.0, np.zeros(3)
    for s in hull.simplices:
        a, b, c = pts[s]
        v = abs(np.dot(a - c0, np.cross(b - c0, c - c0))) / 6
        vol += v
        mom += v * (a + b + c + c0) / 4
    return vol, mom / vol


def buoyancy(z, roll, pitch):
    R = rot(roll, pitch)
    V, M = 0.0, np.zeros(3)
    for C in BOXES:
        v, c = clip_below(C @ R.T + np.array([0, 0, z]))
        V += v
        M += v * c
    return V, (M / V if V > 0 else np.zeros(3)), R


def residual(s, m, cg):
    z, roll, pitch = s
    V, cb, R = buoyancy(z, roll, pitch)
    cgw = R @ cg + np.array([0, 0, z])
    F = RHO * V * G
    M = np.cross(cb - cgw, [0, 0, F])
    return [(F - m * G) / (m * G), M[0] / (m * G * 0.1), M[1] / (m * G * 0.1)]


def solve(case):
    m = MASS[case]['mass']
    cg = np.array(MASS[case]['cg'])
    best = None
    for r0 in (0.0, math.radians(4), math.radians(-4)):
        sol = least_squares(residual, [0.27, r0, 0.0], args=(m, cg), bounds=([0, -0.6, -0.4], [0.6, 0.6, 0.4]),
                            xtol=1e-12, ftol=1e-12)
        z, roll, pitch = sol.x
        # roll stiffness: does a small extra heel come back?
        e = math.radians(0.5)
        k = (residual([z, roll + e, pitch], m, cg)[1] - residual([z, roll - e, pitch], m, cg)[1]) / (2 * e)
        V, cb, R = buoyancy(z, roll, pitch)
        r = {'z': float(z), 'roll_deg': math.degrees(roll), 'pitch_deg': math.degrees(pitch), 'stable_in_roll': bool(k < 0),
             'residual': float(np.abs(sol.fun).max()), 'displaced_L': float(V * 1e3),
             'keel_depth': float(-(z + GEO['misc']['keel_z']))}
        if best is None or (r['stable_in_roll'] and not best['stable_in_roll']):
            best = r
        if r0 == 0.0:
            best['symmetric_solution_stable'] = bool(k < 0)
    return best


def cad_hull_draft(case):
    """level draft of the real hull (cad outer solid) carrying the whole weight"""
    import cadquery as cq
    import trimesh
    h = cq.importers.importStep(os.path.join(ROOT, 'cad', 'build', 'hull_outer.step')).val().scale(0.001)
    vs, tr = h.tessellate(0.0005, 0.2)
    mesh = trimesh.Trimesh(np.array([v.toTuple() for v in vs]), np.array(tr), process=True)
    m = MASS[case]['mass']
    lo, hi = mesh.bounds[0][2], mesh.bounds[1][2]
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        sub = trimesh.intersections.slice_mesh_plane(mesh, [0, 0, -1], [0, 0, mid], cap=True)
        if sub.volume * RHO > m:
            hi = mid
        else:
            lo = mid
    return mid - mesh.bounds[0][2]


out = {}
for case in ('full', 'empty'):
    r = solve(case)
    r['cad_hull_level_draft'] = float(cad_hull_draft(case))
    out[case] = r
    print(case, {k: (round(v, 4) if isinstance(v, float) else v) for k, v in r.items()})
os.makedirs(os.path.join(ROOT, 'verify', 'out'), exist_ok=True)
json.dump(out, open(os.path.join(ROOT, 'verify', 'out', 'hydrostatics.json'), 'w'), indent=1)
