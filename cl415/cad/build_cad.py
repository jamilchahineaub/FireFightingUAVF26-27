"""step 2: parametric cad of the scaled cl-415 uav (cadquery)

every component is its own solid. outputs:
  cad/parts/<part>.step, .stl      one file per part, assembly frame, metres
  cad/cl415_assembly.step, .stl    whole aircraft (step keeps part names and colours)
  cad/geometry.json                volumes, areas, inertia per unit density, hinge lines, link frames, collision boxes
  cl415_description/meshes/*.stl   coarse visual meshes, each in its urdf link frame
  cad/renders/*.png                cad views and cad-vs-obj outline overlays

frame: REP-103 (x forward, y left, z up), metres, origin at the wing leading edge root on the chord line.
the spec numbers (span, chord, airfoil, tank volume) are set here. the other proportions were measured
from cl415.obj by obj_inspect/inspect_obj.py; scalars are copied below (rounded), the hull/nacelle/float
offset tables are read from obj_inspect/measurements.json.

run from the project root:  .venv/Scripts/python cad/build_cad.py
"""
import json
import math
import os
import sys
import time

import numpy as np
import cadquery as cq
import trimesh
from OCP.BRepGProp import BRepGProp
from OCP.BRepTools import BRepTools
from OCP.GProp import GProp_GProps

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tools'))
import render  # noqa: E402

CAD = os.path.join(ROOT, 'cad')
PARTS = os.path.join(CAD, 'parts')
RENDERS = os.path.join(CAD, 'renders')
MESHES = os.path.join(ROOT, 'cl415_description', 'meshes')
for d in (PARTS, RENDERS, MESHES):
    os.makedirs(d, exist_ok=True)

MEAS = json.load(open(os.path.join(ROOT, 'obj_inspect', 'measurements.json')))

# spec
SPAN = 2.51                 # m
CHORD = 0.278               # m, constant
WING_FOIL = '4417'
TANK_VOLUME = 3.0e-3        # m^3 (3 l of water)

# wing, from the obj
WING_INC = 2.65             # deg, incidence to the hull datum (le up)
AIL_Y = (0.793, 1.195)      # aileron inboard/outboard station
AIL_CF = 0.25               # aileron chord ratio

# horizontal tail, from the obj
HT_FOIL = '0016'            # obj t/c = 0.164
HT_X_LE, HT_X_HINGE, HT_X_TE = -0.813, -0.957, -1.052
HT_Z = 0.127                # chord plane height
HT_HALF_SPAN = 0.468
# finlets on the h-stab (obj group derive2)
FL_FOIL = '0012'
FL_Y = 0.183
FL_Z = (0.010, HT_Z, 0.236)            # bottom, stab level, top
FL_X_LE = (-0.889, -0.7775, -0.889)

# vertical tail, from the obj (straight le, hinge and te lines)
VT_FOIL = '0012'            # obj t/c = 0.117
VT_Z_ROOT, VT_Z_TIP = -0.0173, 0.417
VT_ROOT = {'le': -0.668, 'hinge': -0.956, 'te': -1.087}
VT_TIP = {'le': -0.970, 'hinge': -1.060, 'te': -1.133}

# propulsion, from the obj
NAC_Y = 0.2973
PROP_X, PROP_Z = 0.1895, 0.0606
PROP_D = 0.3554
N_BLADES = 4
PROP_PITCH = 0.18           # m geometric pitch (14x7 class), assumed, the obj blades are flat paddles
SPIN_X0, SPIN_X1, SPIN_R = 0.1775, 0.2364, 0.0296

# floats
FLOAT_Y = 1.1517

# hull
DECK_Z = -0.0173

# water tank (sized for exactly 3 l inside)
TANK_L, TANK_W, TANK_R = 0.200, 0.150, 0.015   # inner length, width, corner radius
TANK_WALL = 0.002
TANK_Z_BOTTOM = -0.240     # outside bottom, above the hull chine line

# skins of the hollow parts (the rest are solid: foam-core surfaces, props, tank walls)
SKIN_HULL = 0.002
SKIN_NACELLE = 0.0015
SKIN_FLOAT = 0.0015

# clearances
GAP = 0.0015                # radial gap in hinge coves
END_GAP = 0.002             # spanwise gap at control surface ends
R_TUBE = 0.004              # elevator torque tube

TOL_FINE = (0.0003, 0.15)   # stl for cad (linear m, angular rad)
TOL_SIM = (0.0020, 0.30)    # stl for gazebo visuals
FINE_CAP = 40000            # max triangles per part in cad/parts/*.stl (fuselage gets 2x)
SIM_CAP = {'fuselage': 16000, 'left_propeller': 2500, 'right_propeller': 2500}   # gazebo meshes, default below
SIM_CAP_DEFAULT = 5000

V = cq.Vector


def t0():
    return time.time()


# naca 4-digit, cosine spacing
def naca4(code, n=70):
    m, p, t = int(code[0]) / 100.0, int(code[1]) / 10.0, int(code[2:]) / 100.0
    beta = np.linspace(0.0, math.pi, n)
    x = 0.5 * (1.0 - np.cos(beta))
    yt = 5 * t * (0.2969 * np.sqrt(x) - 0.1260 * x - 0.3516 * x ** 2 + 0.2843 * x ** 3 - 0.1015 * x ** 4)
    if m > 0:
        yc = np.where(x < p, m / p ** 2 * (2 * p * x - x ** 2), m / (1 - p) ** 2 * (1 - 2 * p + 2 * p * x - x ** 2))
        dyc = np.where(x < p, 2 * m / p ** 2 * (p - x), 2 * m / (1 - p) ** 2 * (p - x))
    else:
        yc = np.zeros_like(x)
        dyc = np.zeros_like(x)
    th = np.arctan(dyc)
    up = np.c_[x - yt * np.sin(th), yc + yt * np.cos(th)]
    lo = np.c_[x + yt * np.sin(th), yc - yt * np.cos(th)]
    return up, lo


def naca4_at(code, xc):
    """camber, half thickness at chord fraction xc"""
    m, p, t = int(code[0]) / 100.0, int(code[1]) / 10.0, int(code[2:]) / 100.0
    yt = 5 * t * (0.2969 * math.sqrt(xc) - 0.1260 * xc - 0.3516 * xc ** 2 + 0.2843 * xc ** 3 - 0.1015 * xc ** 4)
    if m == 0:
        return 0.0, yt
    yc = m / p ** 2 * (2 * p * xc - xc ** 2) if xc < p else m / (1 - p) ** 2 * (1 - 2 * p + 2 * p * xc - xc ** 2)
    return yc, yt


def foil_loop(code, n=70):
    up, lo = naca4(code, n)
    return np.vstack([up[::-1], lo[1:]])          # te upper -> le -> te lower (blunt te)


def foil_wire(code, to3d, n=70):
    pts = [V(*to3d(x, y)) for x, y in foil_loop(code, n)]
    return cq.Wire.assembleEdges([cq.Edge.makeSpline(pts), cq.Edge.makeLine(pts[-1], pts[0])])


def pitch_up(x, z, deg):
    a = math.radians(deg)
    return x * math.cos(a) - z * math.sin(a), x * math.sin(a) + z * math.cos(a)


def wing_pt(xc, yc, y):
    x, z = pitch_up(-xc * CHORD, yc * CHORD, WING_INC)
    return x, y, z


def frame(origin, e_aft, e_span):
    """location whose local x = aft chord dir, local z = hinge (span) dir, local y = thickness dir"""
    return cq.Location(cq.Plane(origin=V(*origin), xDir=V(*e_aft), normal=V(*e_span)))


def aft_box(loc, w0, w1, u0=0.0):
    return cq.Solid.makeBox(2.0, 2.0, w1 - w0, pnt=V(u0, -1.0, w0)).moved(loc)


def cove(loc, w0, w1, r0, r1):
    if abs(r0 - r1) < 1e-9:
        s = cq.Solid.makeCylinder(r0, w1 - w0, pnt=V(0, 0, w0), dir=V(0, 0, 1))
    else:
        s = cq.Solid.makeCone(r0, r1, w1 - w0, pnt=V(0, 0, w0), dir=V(0, 0, 1))
    return s.moved(loc)


def split_moving(S, loc, w0, w1, r0, r1):
    """moving part: everything aft of the hinge plane plus a round nose of radius r centred on the hinge"""
    a = S.intersect(aft_box(loc, w0 + END_GAP, w1 - END_GAP))
    b = S.intersect(cove(loc, w0 + END_GAP, w1 - END_GAP, r0, r1))
    return a.fuse(b).clean()


def remove_moving(S, loc, w0, w1, r0, r1):
    """fixed part: hinge plane cut plus a cove with radial gap"""
    return S.cut(aft_box(loc, w0, w1)).cut(cove(loc, w0, w1, r0 + GAP, r1 + GAP)).clean()


def box(x0, x1, y0, y1, z0, z1):
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, pnt=V(x0, y0, z0))


def mvol(shape):
    """volume from a fine tessellation. occ's gprop integration is off by 5-10 % on these
    spline-bounded solids (checked against solidworks and the naca area formula), the mesh is not"""
    if shape is None or len(shape.Solids()) == 0:
        return 0.0
    BRepTools.Clean_s(shape.wrapped)
    vs, tr = shape.tessellate(0.0002, 0.1)
    if not tr:
        return 0.0
    return float(trimesh.Trimesh(np.array([v.toTuple() for v in vs]), np.array(tr), process=True).volume)


def largest_solid(s):
    sols = s.Solids()
    if len(sols) <= 1:
        return s
    return max(sols, key=mvol)


T = t0()
print('building parts ...')

# wing: naca 4417 straight extrusion, split into halves at the centreline
wing_root_wire = foil_wire(WING_FOIL, lambda x, y: wing_pt(x, y, 0.0))
wingL_full = cq.Solid.extrudeLinear(wing_root_wire, [], V(0, SPAN / 2, 0))
wingR_full = cq.Solid.extrudeLinear(wing_root_wire, [], V(0, -SPAN / 2, 0))

xc_h = 1.0 - AIL_CF
yc_h, yt_h = naca4_at(WING_FOIL, xc_h)
ail_hinge = wing_pt(xc_h, yc_h, 0.0)
ail_r = yt_h * CHORD
ia = math.radians(WING_INC)
e_aft_wing = (-math.cos(ia), 0.0, -math.sin(ia))
loc_ail = frame(ail_hinge, e_aft_wing, (0, 1, 0))
aileronL = split_moving(wingL_full, loc_ail, AIL_Y[0], AIL_Y[1], ail_r, ail_r)
aileronR = split_moving(wingR_full, loc_ail, -AIL_Y[1], -AIL_Y[0], ail_r, ail_r)
wingL = remove_moving(wingL_full, loc_ail, AIL_Y[0], AIL_Y[1], ail_r, ail_r)
wingR = remove_moving(wingR_full, loc_ail, -AIL_Y[1], -AIL_Y[0], ail_r, ail_r)
print(f'  wing + ailerons {time.time() - T:.1f}s')


# envelopes of the wing for trimming other parts against it
def wing_envelope(kind, y0, y1):
    up, lo = naca4(WING_FOIL, 70)
    big = 1.0 / CHORD
    if kind == 'above_lower':
        curve = lo
        poly = [(curve[-1][0], big), (curve[0][0], big)]
    else:
        curve = up
        poly = [(curve[-1][0], -big), (curve[0][0], -big)]
    pts = [V(*wing_pt(x, y, y0)) for x, y in curve]
    ext = [V(*wing_pt(x, y, y0)) for x, y in poly]
    edges = [cq.Edge.makeSpline(pts), cq.Edge.makeLine(pts[-1], ext[0]), cq.Edge.makeLine(ext[0], ext[1]),
             cq.Edge.makeLine(ext[1], pts[0])]
    w = cq.Wire.assembleEdges(edges)
    return cq.Solid.extrudeLinear(w, [], V(0, y1 - y0, 0))


# hull: offset tables from the obj, two lofts (forebody, afterbody) joined at the planing step
H = MEAS['hull']
FR = H['fracs']


def clean_offsets(st):
    w = np.array(st['half_breadth'], float)
    f = np.array(FR)
    ok = w > 1e-6
    ok[0] = True
    w[0] = 0.0
    if not ok[-1]:
        last = np.where(ok)[0].max()
        w[-1] = 0.8 * w[last]
        ok[-1] = True
    w = np.interp(f, f[ok], w[ok])
    return w


def hull_points(st, shrink=0.0, z_top=None):
    w = np.maximum(clean_offsets(st) - shrink, 0.0005)
    zk = st['z_keel']
    zt = (st['z_top'] if z_top is None else z_top) - shrink
    zs = zk + np.array(FR) * (zt - zk)
    x = st['x']
    side = [(x, -w[k], zs[k]) for k in range(1, len(FR))]
    return [(x, 0.0, zk)] + side + [(x, 0.0, zt)] + [(x, -p[1], p[2]) for p in side[::-1]]


def spline_wire(pts):
    return cq.Wire.assembleEdges([cq.Edge.makeSpline([V(*p) for p in pts], periodic=True)])


def hull_section(st, shrink=0.0, z_top=None):
    return spline_wire(hull_points(st, shrink, z_top))


def inner_section(st, t, shrink=0.0, x=None):
    """true 2d offset of a hull section by the skin thickness: sample the outer spline, buffer the
    polygon inwards, project each spline point onto the offset ring, keep it symmetric. stations too
    small for that (nose, stern) get a 20 % copy about their centre, which keeps the inner loft on the
    same stations as the outer one, so both lofts interpolate alike between stations"""
    from shapely.geometry import Point, Polygon
    pts = hull_points(st, shrink)
    n = len(pts)
    x = pts[0][0] if x is None else x
    edge = cq.Edge.makeSpline([V(*p) for p in pts], periodic=True)
    dense = [edge.positionAt(u) for u in np.linspace(0, 1, 400, endpoint=False)]
    poly = Polygon([(p.y, p.z) for p in dense])
    inner = poly.buffer(-t, join_style=1, quad_segs=16)
    b = inner.bounds if not inner.is_empty else None
    if b is None or inner.geom_type != 'Polygon' or b[2] - b[0] < 3 * t or b[3] - b[1] < 3 * t:
        cy, cz = poly.centroid.x, poly.centroid.y
        return spline_wire([(x, cy + 0.2 * (p[1] - cy), cz + 0.2 * (p[2] - cz)) for p in pts])
    ring = inner.exterior
    proj = [ring.interpolate(ring.project(Point(p[1], p[2]))) for p in pts]
    out = []
    for k in range(n):
        q, m = proj[k], proj[(n - k) % n]          # point k and its mirror image
        y = 0.5 * (abs(q.x) + abs(m.x)) * (1 if pts[k][1] >= 0 else -1)
        if abs(pts[k][1]) < 1e-9:
            y = 0.0
        out.append((x, y, 0.5 * (q.y + m.y)))
    return spline_wire(out)


fore = H['forebody']
aft = H['afterbody']
step_x = MEAS['hull_misc']['step_x']
# nose cap: a small copy of the first station a bit ahead
nose = dict(fore[0])
nose_c = 0.5 * (nose['z_keel'] + nose['z_top'])
nose = {'x': MEAS['hull_misc']['nose_x'], 'z_keel': nose_c - 0.15 * (nose_c - nose['z_keel']),
        'z_top': nose_c + 0.15 * (nose['z_top'] - nose_c), 'half_breadth': [0.15 * w for w in fore[0]['half_breadth']]}
fore_secs = [nose] + fore[:-1] + [dict(fore[-1], x=step_x)]
# afterbody starts at the step, a hair inside the forebody outline so the union is clean. the obj hull is
# open right behind the step (the water doors are separate objects), so this section reuses the shape of
# the next good station, scaled to the forebody beam, at the keel height measured just behind the step
ref = aft[1]
k_beam = max(fore[-1]['half_breadth']) / max(ref['half_breadth'])
aft_first = {'x': step_x, 'z_keel': aft[0]['z_keel'], 'z_top': fore[-1]['z_top'],
             'half_breadth': [k_beam * w for w in ref['half_breadth']]}
fore_secs_w = [hull_section(s) for s in fore_secs]
aft_secs_w = [hull_section(aft_first, shrink=0.0006)] + [hull_section(s) for s in aft[1:]]
hull_fore = cq.Solid.makeLoft(fore_secs_w)
hull_aft = cq.Solid.makeLoft(aft_secs_w)
hull_outer = hull_fore.fuse(hull_aft).clean()      # solid outline, used to trim the fin and check fits
# hollow it: every station offset inwards by the skin thickness. the nose block and the thin stern
# stay solid. both cavities start one skin ahead of the step, so the step face keeps its wall and
# the forebody and afterbody cavities overlap (no bulkhead across the tank)
x_cav = step_x + SKIN_HULL
inner_fore = [inner_section(s, SKIN_HULL) for s in fore_secs[:-1]] + [inner_section(fore[-1], SKIN_HULL, x=x_cav)]
inner_aft = ([inner_section(aft_first, SKIN_HULL, shrink=0.0006, x=x_cav)] + [inner_section(s, SKIN_HULL) for s in aft[1:]])
hull_inner = cq.Solid.makeLoft(inner_fore).fuse(cq.Solid.makeLoft(inner_aft)).clean()
inner_poke = mvol(hull_inner.cut(hull_outer))
POKE = {'hull': inner_poke}
hull = hull_outer.cut(hull_inner).clean()
os.makedirs(os.path.join(CAD, 'build'), exist_ok=True)       # helper solids for fit checks, not deliverables
hull_inner.exportStep(os.path.join(CAD, 'build', 'hull_cavity.step'), unit='M')
hull_outer.exportStep(os.path.join(CAD, 'build', 'hull_outer.step'), unit='M')
print(f'  hull lofts {time.time() - T:.1f}s, valid={hull.isValid()}, inner outside outer {inner_poke * 1e6:.3f} cm^3')

# saddle between the deck and the wing lower surface
saddle = box(-CHORD * math.cos(ia) + 0.006, -0.004, -0.085, 0.085, DECK_Z - 0.0015, 0.03)
saddle = saddle.cut(wing_envelope('above_lower', -0.1, 0.1))
hull = hull.fuse(saddle).clean()

# water tank: rounded box, inner volume exactly 3 l
area_in = TANK_L * TANK_W - (4 - math.pi) * TANK_R ** 2
TANK_H = TANK_VOLUME / area_in
x_qc, z_qc = pitch_up(-0.25 * CHORD, 0.0, WING_INC)
tank_c = (x_qc, 0.0, TANK_Z_BOTTOM + TANK_WALL + TANK_H / 2)


def rbox(L, W, Hh, r, c):
    return cq.Workplane('XY').box(L, W, Hh).edges('|Z').fillet(r).val().translate(V(*c))


water = rbox(TANK_L, TANK_W, TANK_H, TANK_R, tank_c)
tank_out = rbox(TANK_L + 2 * TANK_WALL, TANK_W + 2 * TANK_WALL, TANK_H + 2 * TANK_WALL, TANK_R + TANK_WALL, tank_c)
tank = tank_out.cut(water).clean()
tank_fit = mvol(tank_out.cut(hull_inner))       # 0 = the tank sits inside the hull cavity
fuselage = hull.cut(wingL).cut(wingR).clean()
print(f'  tank (inner {water.Volume() * 1e3:.4f} l, outside the cavity {tank_fit * 1e6:.2f} cm^3) {time.time() - T:.1f}s')

# horizontal tail + finlets
ht_c = HT_X_LE - HT_X_TE


def ht_pt(xc, yc, y):
    return HT_X_LE - xc * ht_c, y, HT_Z + yc * ht_c


ht_full = cq.Solid.extrudeLinear(foil_wire(HT_FOIL, lambda x, y: ht_pt(x, y, -HT_HALF_SPAN)), [],
                                 V(0, 2 * HT_HALF_SPAN, 0))
el_xc = (HT_X_LE - HT_X_HINGE) / ht_c
el_r = naca4_at(HT_FOIL, el_xc)[1] * ht_c
loc_el = frame((HT_X_HINGE, 0.0, HT_Z), (-1, 0, 0), (0, 1, 0))

# vertical tail loft (root pushed 6 mm into the hull, trimmed by it later)


def vt_line(key, z):
    f = (z - VT_Z_ROOT) / (VT_Z_TIP - VT_Z_ROOT)
    return VT_ROOT[key] + f * (VT_TIP[key] - VT_ROOT[key])


def vt_wire(z):
    xl, xt = vt_line('le', z), vt_line('te', z)
    c = xl - xt
    return foil_wire(VT_FOIL, lambda x, y: (xl - x * c, y * c, z))


def vt_half_t(x, z):
    xl, xt = vt_line('le', z), vt_line('te', z)
    c = xl - xt
    xc = min(max((xl - x) / c, 0.0), 1.0)
    return naca4_at(VT_FOIL, xc)[1] * c


vt_full = cq.Solid.makeLoft([vt_wire(VT_Z_ROOT - 0.006), vt_wire(VT_Z_TIP)], True)
h0 = np.array([vt_line('hinge', VT_Z_ROOT), 0.0, VT_Z_ROOT])
h1 = np.array([vt_line('hinge', VT_Z_TIP), 0.0, VT_Z_TIP])
e_rud = (h1 - h0) / np.linalg.norm(h1 - h0)
a = np.array([-1.0, 0, 0])
e_rud_aft = a - a.dot(e_rud) * e_rud
e_rud_aft /= np.linalg.norm(e_rud_aft)
loc_rud = frame(tuple(h0), tuple(e_rud_aft), tuple(e_rud))
rud_len = float(np.linalg.norm(h1 - h0))
r_rud0 = vt_half_t(h0[0], VT_Z_ROOT)
r_rud1 = vt_half_t(h1[0], VT_Z_TIP)

# elevator inner edge: clear of the fin between the two hinge lines, and of the rudder swing aft of it
x_rh_stab = vt_line('hinge', HT_Z)
fin_t_max = max(vt_half_t(x, HT_Z) for x in np.linspace(x_rh_stab, HT_X_HINGE + el_r + GAP, 25))
EL_Y_IN = fin_t_max + 2 * GAP
rud_c_stab = x_rh_stab - vt_line('te', HT_Z)
r_rud_stab = vt_half_t(x_rh_stab, HT_Z)
RUD_SWING = math.radians(30.0 + 5.0)
y_clear = rud_c_stab * math.sin(RUD_SWING) + r_rud_stab + 2 * GAP

# finlets: diamond planform, trailing edge kept ahead of the elevator cove
fl_te = HT_X_HINGE + el_r + 2 * GAP
finlets = []
for sgn in (1, -1):
    wires = []
    for z, xl in zip(FL_Z, FL_X_LE):
        c = xl - fl_te
        wires.append(foil_wire(FL_FOIL, lambda x, y, xl=xl, c=c, z=z: (xl - x * c, sgn * FL_Y + y * c, z)))
    finlets.append(cq.Solid.makeLoft(wires, True))

elevL = split_moving(ht_full, loc_el, EL_Y_IN, HT_HALF_SPAN + END_GAP, el_r, el_r)
elevR = split_moving(ht_full, loc_el, -HT_HALF_SPAN - END_GAP, -EL_Y_IN, el_r, el_r)
rud_clear = box(-2.0, x_rh_stab + r_rud_stab + 2 * GAP, -y_clear, y_clear, -1, 1)
tube = cq.Solid.makeCylinder(R_TUBE, 2 * (EL_Y_IN + 0.012), pnt=V(HT_X_HINGE, -(EL_Y_IN + 0.012), HT_Z),
                             dir=V(0, 1, 0))
elevator = elevL.cut(rud_clear).fuse(elevR.cut(rud_clear)).fuse(tube).clean()
h_stab = remove_moving(ht_full, loc_el, -HT_HALF_SPAN - 0.01, HT_HALF_SPAN + 0.01, el_r, el_r)
h_stab = h_stab.fuse(*finlets).clean()
print(f'  h-stab + elevator (inner y {EL_Y_IN * 1000:.1f} mm, rudder clearance {y_clear * 1000:.1f} mm) '
      f'{time.time() - T:.1f}s')

# rudder: everything aft of the swept hinge, kept 5 mm above the deck so it can swing
rudder = split_moving(vt_full, loc_rud, -0.05, rud_len + END_GAP, r_rud0, r_rud1)
rudder = rudder.cut(box(-2, 2, -1, 1, -1, DECK_Z + 0.005)).clean()
v_stab = remove_moving(vt_full, loc_rud, -0.05, rud_len + 0.01, r_rud0, r_rud1)
v_stab = v_stab.cut(hull_outer).cut(h_stab).clean()
v_stab = v_stab.cut(cq.Solid.makeCylinder(R_TUBE + GAP, 2 * (EL_Y_IN + 0.02),
                                          pnt=V(HT_X_HINGE, -(EL_Y_IN + 0.02), HT_Z), dir=V(0, 1, 0))).clean()
v_stab = largest_solid(v_stab)
print(f'  fin + rudder {time.time() - T:.1f}s')

# nacelles: ellipse stations, trimmed to sit on the wing upper surface
NAC = MEAS['nacelle_left']


def ellipse_wire(a, b, c, normal, xdir):
    # an ellipse with equal axes goes to step as a degenerate ellipse and solidworks then can't
    # compute mass properties on the body, so write those as circles
    if abs(a - b) < 1e-4:
        return cq.Wire.makeCircle(0.5 * (a + b), c, normal)
    return cq.Wire.makeEllipse(a, b, c, normal, xdir)


def ellipse_loft(stations, sgn, shrink=0.0):
    wires = []
    for s in stations:
        x = min(s['x'], SPIN_X0 - 0.004)    # the loft bulges a few mm past its first section
        wires.append(ellipse_wire(s['half_w'] - shrink, s['half_h'] - shrink, V(x, sgn * NAC_Y, s['z_c']),
                                  V(1, 0, 0), V(0, 1, 0)))
    return cq.Solid.makeLoft(wires)


def nacelle(sgn):
    # hollow fairing (front ring and tail cone stay solid), trimmed to sit on the wing upper surface
    o, i = ellipse_loft(NAC, sgn), ellipse_loft(NAC[1:-1], sgn, SKIN_NACELLE)
    POKE[f'nacelle_{sgn:+d}'] = mvol(i.cut(o))
    n = o.cut(i)
    y0 = sgn * NAC_Y - 0.07
    return largest_solid(n.cut(wing_envelope('below_upper', y0, y0 + 0.14)).clean())


nacL = nacelle(1)
nacR = nacelle(-1)
print(f'  nacelles {time.time() - T:.1f}s')


# props: spinner + 4 blades, left one turns +x (clockwise from behind), right one is its mirror
def propeller():
    yc, zc = NAC_Y, PROP_Z
    L = SPIN_X1 - SPIN_X0
    circles = []
    for s in [0.0, 0.25, 0.5, 0.7, 0.85, 0.94, 0.985]:
        r = max(SPIN_R * math.sqrt(max(1 - s * s, 0)), 0.0012)
        circles.append(cq.Wire.makeCircle(r, V(SPIN_X0 + s * L, yc, zc), V(1, 0, 0)))
    spinner = cq.Solid.makeLoft(circles)
    R = PROP_D / 2
    stations = [(0.010, 0.020, 0.30), (0.03, 0.024, 0.16), (0.06, 0.027, 0.12), (0.10, 0.026, 0.10),
                (0.14, 0.022, 0.09), (0.170, 0.016, 0.08), (R, 0.010, 0.08)]
    ex = np.array([1.0, 0, 0])
    blades = []
    for k in range(N_BLADES):
        ph = 2 * math.pi * k / N_BLADES + math.radians(20)
        er = np.array([0.0, math.cos(ph), math.sin(ph)])
        et = np.cross(ex, er)
        wires = []
        for r, c, tc in stations:
            beta = math.atan(PROP_PITCH / (2 * math.pi * r))
            chord_dir = math.cos(beta) * et + math.sin(beta) * ex
            ctr = np.array([PROP_X, yc, zc]) + r * er
            wires.append(cq.Wire.makeEllipse(c / 2, tc * c / 2, V(*ctr), V(*er), V(*chord_dir)))
        blades.append(cq.Solid.makeLoft(wires))
    return spinner.fuse(*blades).clean()


propL = propeller()
propR = propL.mirror('XZ')
print(f'  props {time.time() - T:.1f}s')

# floats: ellipse stations + a naca 0010 pylon trimmed to the wing lower surface
FB = MEAS['float_left']['body']
PY = MEAS['float_left']['pylon']


def float_loft(stations, yc, shrink=0.0):
    return cq.Solid.makeLoft([ellipse_wire(s['half_w'] - shrink, 0.5 * (s['z_hi'] - s['z_lo']) - shrink,
                                                  V(s['x'], yc, 0.5 * (s['z_lo'] + s['z_hi'])), V(1, 0, 0),
                                                  V(0, 1, 0)) for s in stations])


def float_part(sgn):
    yc = sgn * FLOAT_Y
    outer = float_loft(FB, yc)
    inner = float_loft(FB[2:-2], yc, SKIN_FLOAT)
    POKE[f'float_{sgn:+d}'] = mvol(inner.cut(outer))
    body = outer.cut(inner)      # hollow, bow and tail blocks solid
    zb, zt = PY[0]['z'] - 0.008, 0.012
    fz = np.array([p['z'] for p in PY])
    pw = []
    for z in (zb, zt):
        xl = float(np.polyval(np.polyfit(fz, [p['x_le'] for p in PY], 1), z))
        xt = float(np.polyval(np.polyfit(fz, [p['x_te'] for p in PY], 1), z))
        c = xl - xt
        pw.append(foil_wire('0010', lambda x, y, xl=xl, c=c, z=z: (xl - x * c, yc + y * c, z)))
    pylon = cq.Solid.makeLoft(pw, True)
    y0 = yc - 0.05
    pylon = pylon.cut(wing_envelope('above_lower', y0, y0 + 0.1)).cut(float_loft(FB[2:-2], yc, SKIN_FLOAT))
    return body.fuse(pylon).clean(), outer


floatL, floatL_body = float_part(1)
floatR, floatR_body = float_part(-1)
print(f'  floats {time.time() - T:.1f}s')

# equipment envelopes (the masses live in mass/mass_props.py; the battery box is placed there too,
# because its position comes out of the cg check)
SERVO = (0.040, 0.020, 0.038)          # standard 35 g servo
WING_SERVO = (0.030, 0.030, 0.012)     # slim wing servo, lying flat
x_ws = -0.16
z_ws = pitch_up(x_ws, naca4_at(WING_FOIL, -x_ws / CHORD)[0] * CHORD, WING_INC)[1]


def cyl_x(r, L, c):
    return cq.Solid.makeCylinder(r, L, pnt=V(c[0] - L / 2, c[1], c[2]), dir=V(1, 0, 0))


def cbox(d, c):
    return box(c[0] - d[0] / 2, c[0] + d[0] / 2, c[1] - d[1] / 2, c[1] + d[1] / 2, c[2] - d[2] / 2, c[2] + d[2] / 2)


EQUIP = [
    # name, link, shape, centre
    ('eq_left_motor', 'left_nacelle', ('cyl', 0.025, 0.045), (PROP_X - 0.052, NAC_Y, PROP_Z)),
    ('eq_right_motor', 'right_nacelle', ('cyl', 0.025, 0.045), (PROP_X - 0.052, -NAC_Y, PROP_Z)),
    ('eq_left_esc', 'left_nacelle', ('box', (0.070, 0.030, 0.015)), (0.06, NAC_Y, 0.045)),
    ('eq_right_esc', 'right_nacelle', ('box', (0.070, 0.030, 0.015)), (0.06, -NAC_Y, 0.045)),
    ('eq_left_aileron_servo', 'left_wing', ('box', WING_SERVO), (x_ws, 0.90, z_ws)),
    ('eq_right_aileron_servo', 'right_wing', ('box', WING_SERVO), (x_ws, -0.90, z_ws)),
    ('eq_avionics', 'fuselage', ('box', (0.100, 0.070, 0.040)), (0.35, 0.0, -0.10)),
    ('eq_elevator_servo', 'fuselage', ('box', SERVO), (-0.70, 0.0105, -0.09)),
    ('eq_rudder_servo', 'fuselage', ('box', SERVO), (-0.70, -0.0105, -0.09)),
    ('eq_door_servo', 'fuselage', ('box', SERVO), (x_qc, 0.0, -0.268)),
]
EQUIP_DEF = []
for name, link, shp, c in EQUIP:
    s = cyl_x(shp[1], shp[2], c) if shp[0] == 'cyl' else cbox(shp[1], c)
    EQUIP_DEF.append((name, s, link, (0.25, 0.25, 0.28)))
# servo bays in the wings, open to the lower skin like a hatch (a closed void trips occ's volume calc)
for sgn in (1, -1):
    bay = box(x_ws - 0.016, x_ws + 0.016, sgn * 0.90 - 0.016, sgn * 0.90 + 0.016, z_ws - 0.10, z_ws + 0.007)
    if sgn > 0:
        wingL = wingL.cut(bay).clean()
    else:
        wingR = wingR.cut(bay).clean()

# part table
PARTS_DEF = [
    # name, shape, urdf link, rgb
    ('fuselage', fuselage, 'fuselage', (0.95, 0.80, 0.10)),
    ('left_wing', wingL, 'left_wing', (0.92, 0.92, 0.92)),
    ('right_wing', wingR, 'right_wing', (0.92, 0.92, 0.92)),
    ('left_aileron', aileronL, 'left_aileron', (0.85, 0.20, 0.15)),
    ('right_aileron', aileronR, 'right_aileron', (0.85, 0.20, 0.15)),
    ('h_stab', h_stab, 'h_stab', (0.92, 0.92, 0.92)),
    ('elevator', elevator, 'elevator', (0.85, 0.20, 0.15)),
    ('v_stab', v_stab, 'v_stab', (0.95, 0.80, 0.10)),
    ('rudder', rudder, 'rudder', (0.85, 0.20, 0.15)),
    ('left_nacelle', nacL, 'left_nacelle', (0.55, 0.57, 0.62)),
    ('right_nacelle', nacR, 'right_nacelle', (0.55, 0.57, 0.62)),
    ('left_propeller', propL, 'left_propeller', (0.12, 0.12, 0.12)),
    ('right_propeller', propR, 'right_propeller', (0.12, 0.12, 0.12)),
    ('left_float', floatL, 'left_float', (0.20, 0.35, 0.75)),
    ('right_float', floatR, 'right_float', (0.20, 0.35, 0.75)),
    ('water_tank', tank, 'water_tank', (0.70, 0.85, 0.95)),
]
REFERENCE = [('water_3L', water, None, (0.15, 0.45, 0.95))]   # the water itself, for mass props in solidworks


def mesh_props(shape):
    """volume, centroid, inertia at unit density about the centroid (base axes) and area, from a fine
    tessellation (see mvol for why not occ gprop)"""
    BRepTools.Clean_s(shape.wrapped)
    vs, tr = shape.tessellate(0.0002, 0.1)
    m = trimesh.Trimesh(np.array([v.toTuple() for v in vs]), np.array(tr), process=True)
    m.density = 1.0
    return float(m.volume), m.center_mass.tolist(), m.moment_inertia.tolist(), float(m.area), bool(m.is_watertight)


def tess(shape, tol, cap=None):
    BRepTools.Clean_s(shape.wrapped)     # drop any earlier triangulation, otherwise occ reuses it
    vs, tr = shape.tessellate(*tol)
    m = trimesh.Trimesh(np.array([v.toTuple() for v in vs]), np.array(tr), process=True)
    if cap is not None and len(m.faces) > cap:
        m = m.simplify_quadric_decimation(face_count=cap)
        m.fix_normals()
    return m


# link frames (urdf). fixed links share the base frame. moving links sit on their hinge/axis.
y_mid_ail = 0.5 * (AIL_Y[0] + AIL_Y[1])
LINKS = {
    'left_aileron': {'xyz': [ail_hinge[0], y_mid_ail, ail_hinge[2]], 'rpy': [0, 0, 0], 'parent': 'left_wing',
                     'type': 'revolute', 'axis': [0, -1, 0], 'limit_deg': 25.0, 'positive': 'trailing edge down'},
    'right_aileron': {'xyz': [ail_hinge[0], -y_mid_ail, ail_hinge[2]], 'rpy': [0, 0, 0], 'parent': 'right_wing',
                      'type': 'revolute', 'axis': [0, -1, 0], 'limit_deg': 25.0, 'positive': 'trailing edge down'},
    'elevator': {'xyz': [HT_X_HINGE, 0.0, HT_Z], 'rpy': [0, 0, 0], 'parent': 'h_stab', 'type': 'revolute',
                 'axis': [0, -1, 0], 'limit_deg': 25.0, 'positive': 'trailing edge down'},
    'rudder': {'xyz': h0.tolist(), 'rpy': [0, 0, 0], 'parent': 'v_stab', 'type': 'revolute',
               'axis': (-e_rud).tolist(), 'limit_deg': 30.0, 'positive': 'trailing edge left'},
    'left_propeller': {'xyz': [PROP_X, NAC_Y, PROP_Z], 'rpy': [0, math.pi / 2, 0], 'parent': 'left_nacelle',
                       'type': 'continuous', 'axis': [0, 0, 1], 'positive': 'thrust along +x (link z)',
                       'spin': 'clockwise seen from behind'},
    'right_propeller': {'xyz': [PROP_X, -NAC_Y, PROP_Z], 'rpy': [0, math.pi / 2, 0], 'parent': 'right_nacelle',
                        'type': 'continuous', 'axis': [0, 0, 1], 'positive': 'thrust along +x (link z)',
                        'spin': 'counter-clockwise seen from behind'},
}
for name in ['fuselage', 'left_wing', 'right_wing', 'h_stab', 'v_stab', 'left_nacelle', 'right_nacelle',
             'left_float', 'right_float', 'water_tank']:
    LINKS[name] = {'xyz': [0, 0, 0], 'rpy': [0, 0, 0], 'parent': 'base_link', 'type': 'fixed'}


def rpy_matrix(rpy):
    r, p, y = rpy
    Rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    Ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    Rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def link_pose_in_base(link):
    """pose of a link frame in base_link (only one level of nesting under fixed links at the base origin)"""
    L = LINKS[link]
    return np.array(L['xyz'], float), rpy_matrix(L['rpy'])


# export
print('exporting ...')
geo = {'frame': 'REP-103 x fwd, y left, z up; metres; origin wing LE root on chord line',
       'scale_from_obj': MEAS['scale'], 'parts': {}, 'links': LINKS}
assy = cq.Assembly(name='cl415')
all_fine = []
render_parts = []
KIND = {n: 'structure' for n, *_ in PARTS_DEF}
KIND.update({n: 'equipment' for n, *_ in EQUIP_DEF})
KIND.update({n: 'reference' for n, *_ in REFERENCE})
for name, shp, link, rgb in PARTS_DEF + EQUIP_DEF + REFERENCE:
    if not shp.isValid():
        print(f'  WARNING {name} not valid, fixing')
        shp = shp.fix()
    vol, com, I, area, wt = mesh_props(shp)
    bb = shp.BoundingBox()
    shp.exportStep(os.path.join(PARTS, f'{name}.step'), unit='M')
    cap = FINE_CAP * (2 if name == 'fuselage' else 1)
    fine = tess(shp, TOL_FINE, cap)
    fine.export(os.path.join(PARTS, f'{name}.stl'))
    entry = {'kind': KIND[name], 'link': link, 'rgb': rgb, 'volume': vol, 'centroid': com, 'inertia_unit_density': I,
             'area': area, 'mesh_watertight': wt,
             'n_solids': len(shp.Solids()), 'valid': shp.isValid(),
             'bbox': [[bb.xmin, bb.ymin, bb.zmin], [bb.xmax, bb.ymax, bb.zmax]]}
    if KIND[name] != 'reference':
        assy.add(shp, name=name, color=cq.Color(*rgb))
    if KIND[name] == 'structure':
        all_fine.append(fine)
        coarse = tess(shp, TOL_SIM, SIM_CAP.get(name, SIM_CAP_DEFAULT))
        render_parts.append((coarse, rgb))
        p, Rm = link_pose_in_base(link)
        local = trimesh.Trimesh((coarse.vertices - p) @ Rm, coarse.faces, process=False)
        local.export(os.path.join(MESHES, f'{name}.stl'))
        entry['mesh'] = f'meshes/{name}.stl'
        entry['mesh_triangles'] = int(len(local.faces))
    geo['parts'][name] = entry
    print(f'  {name:16s} vol {vol * 1e6:9.1f} cm3  area {area * 1e4:8.1f} cm2  solids {entry["n_solids"]}  '
          f'valid {entry["valid"]}  tris {entry.get("mesh_triangles", "-")}')
assy.export(os.path.join(CAD, 'cl415_assembly.step'), unit='M')
trimesh.util.concatenate(all_fine).export(os.path.join(CAD, 'cl415_assembly.stl'))

geo['hull_outer'] = {'volume': mvol(hull_outer), 'cavity_volume': mvol(hull_inner),
                     'cavity_outside_outer': inner_poke, 'skin': SKIN_HULL}
geo['skins'] = {'hull': SKIN_HULL, 'nacelle': SKIN_NACELLE, 'float': SKIN_FLOAT}
geo['skin_poke_through_m3'] = POKE
print('  cavity poke-through (m^3, want 0):', {k: f'{v:.2e}' for k, v in POKE.items()})
geo['float_body'] = {'volume': mvol(floatL_body)}
geo['tank'] = {'inner_volume': water.Volume(),   # planar + cylindrical faces, occ is exact here 'inner_L': TANK_L, 'inner_W': TANK_W, 'inner_H': TANK_H,
               'corner_r': TANK_R, 'wall': TANK_WALL, 'center': list(tank_c),
               'outside_hull_cavity_volume': tank_fit,
               'outer_bbox': geo['parts']['water_tank']['bbox']}


# projected areas for the aero model, straight from the cad tessellation
def proj_area(shape, normal):
    from shapely.geometry import Polygon
    from shapely.ops import unary_union
    m = tess(shape, (0.0005, 0.2))
    n = np.asarray(normal, float)
    n /= np.linalg.norm(n)
    u = np.cross(n, [0, 0, 1.0]) if abs(n[2]) < 0.9 else np.cross(n, [1.0, 0, 0])
    u /= np.linalg.norm(u)
    v = np.cross(n, u)
    tri = m.vertices[m.faces]
    p2 = np.stack([tri @ u, tri @ v], -1)
    d1, d2 = p2[:, 1] - p2[:, 0], p2[:, 2] - p2[:, 0]
    keep = np.abs(d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0]) > 1e-14
    polys = [Polygon(t) for t in p2[keep]]
    g = unary_union(polys)
    return float(g.area), g


n_wing = (-math.sin(ia), 0.0, math.cos(ia))
S_wL = proj_area(wingL_full, n_wing)[0]
S_wR = proj_area(wingR_full, n_wing)[0]
S_ailL = proj_area(aileronL, n_wing)[0]
S_ht = proj_area(ht_full, (0, 0, 1))[0]
S_el = proj_area(elevator, (0, 0, 1))[0]
vt_above = vt_full.cut(box(-2, 2, -1, 1, -1, DECK_Z))
S_vt = proj_area(vt_above, (0, 1, 0))[0]
S_rud = proj_area(rudder, (0, 1, 0))[0]
S_fl = proj_area(finlets[0], (0, 1, 0))[0]
fl_cent = mesh_props(finlets[0])[1]
vt_mac_z = VT_Z_ROOT + (VT_Z_TIP - VT_Z_ROOT) / 3 * (1 + 2 * (vt_line('le', VT_Z_TIP) - vt_line('te', VT_Z_TIP)) /
                                                     (vt_line('le', VT_Z_ROOT) - vt_line('te', VT_Z_ROOT))) / \
    (1 + (vt_line('le', VT_Z_TIP) - vt_line('te', VT_Z_TIP)) / (vt_line('le', VT_Z_ROOT) - vt_line('te', VT_Z_ROOT)))
vt_root_c = vt_line('le', VT_Z_ROOT) - vt_line('te', VT_Z_ROOT)
vt_tip_c = vt_line('le', VT_Z_TIP) - vt_line('te', VT_Z_TIP)
lam = vt_tip_c / vt_root_c
vt_mac = 2.0 / 3.0 * vt_root_c * (1 + lam + lam ** 2) / (1 + lam)

geo['aero'] = {
    'wing': {'airfoil': WING_FOIL, 'span': SPAN, 'chord': CHORD, 'incidence_deg': WING_INC,
             'area_left': S_wL, 'area_right': S_wR, 'area_total': S_wL + S_wR, 'mac': CHORD,
             'x_le_mac': 0.0, 'ac_left': [x_qc, SPAN / 4, z_qc], 'ac_right': [x_qc, -SPAN / 4, z_qc],
             'aspect_ratio': SPAN ** 2 / (S_wL + S_wR), 'normal': list(n_wing)},
    'aileron': {'y': list(AIL_Y), 'cf_c': AIL_CF, 'area_each': S_ailL, 'hinge': list(ail_hinge),
                'cove_r': ail_r},
    'htail': {'airfoil': HT_FOIL, 'area': S_ht, 'span': 2 * HT_HALF_SPAN, 'chord': ht_c,
              'ac': [HT_X_LE - 0.25 * ht_c, 0.0, HT_Z], 'x_hinge': HT_X_HINGE, 'cf_c': 1 - el_xc,
              'elevator_area': S_el, 'elevator_inner_y': EL_Y_IN, 'rudder_clearance_y': y_clear},
    'vtail': {'airfoil': VT_FOIL, 'area': S_vt, 'height': VT_Z_TIP - VT_Z_ROOT, 'root_chord': vt_root_c,
              'tip_chord': vt_tip_c, 'mac': vt_mac, 'z_mac': vt_mac_z,
              'ac': [vt_line('le', vt_mac_z) - 0.25 * vt_mac, 0.0, vt_mac_z],
              'rudder_area': S_rud, 'hinge_root': h0.tolist(), 'hinge_tip': h1.tolist(),
              'hinge_axis': e_rud.tolist(), 'cf_c_root': (VT_ROOT['hinge'] - VT_ROOT['te']) / vt_root_c,
              'cf_c_tip': (VT_TIP['hinge'] - VT_TIP['te']) / vt_tip_c},
    'finlets': {'area_each': S_fl, 'centroid_left': fl_cent, 'height': FL_Z[2] - FL_Z[0],
                'airfoil': FL_FOIL},
    'props': {'diameter': PROP_D, 'center_left': [PROP_X, NAC_Y, PROP_Z], 'center_right': [PROP_X, -NAC_Y, PROP_Z],
              'blades': N_BLADES, 'pitch_m': PROP_PITCH},
}


# collision primitives (boxes / cylinders) in each link frame
def hull_boxes():
    from shapely.geometry import Polygon
    secs = [s for s in fore[1:]] + [dict(aft_first)] + aft[1:]
    xs = [s['x'] for s in secs]
    data = []
    for s in secs:
        w = clean_offsets(s)
        zs = s['z_keel'] + np.array(FR) * (s['z_top'] - s['z_keel'])
        poly = Polygon([(-w[k], zs[k]) for k in range(len(FR))] + [(w[k], zs[k]) for k in range(len(FR))][::-1])
        data.append((s['x'], poly.area, s['z_keel'], s['z_top']))
    cuts = [MEAS['hull_misc']['nose_x'], 0.47, 0.30, step_x, -0.45, -0.75, MEAS['hull_misc']['stern_x']]
    boxes = []
    for xa, xb in zip(cuts[:-1], cuts[1:]):
        sel = [d for d in data if xb <= d[0] <= xa] or [min(data, key=lambda d: abs(d[0] - 0.5 * (xa + xb)))]
        A = np.mean([d[1] for d in sel])
        zk = np.mean([d[2] for d in sel])
        zt = np.mean([d[3] for d in sel])
        h = zt - zk
        boxes.append({'type': 'box', 'size': [xa - xb, A / h, h], 'xyz': [0.5 * (xa + xb), 0.0, zk + h / 2],
                      'rpy': [0, 0, 0]})
    return boxes


def bbox_box(shape, shrink=1.0):
    b = shape.BoundingBox()
    return {'type': 'box', 'size': [b.xlen * shrink, b.ylen * shrink, b.zlen * shrink],
            'xyz': [b.center.x, b.center.y, b.center.z], 'rpy': [0, 0, 0]}


t_w = 0.17 * CHORD
cx, cz = pitch_up(-0.5 * CHORD, naca4_at(WING_FOIL, 0.5)[0] * CHORD, WING_INC)
fb = floatL_body.BoundingBox()
fvol = geo['float_body']['volume']
f_h = fb.zlen
f_l = fb.xlen
COLL = {
    'fuselage': hull_boxes(),
    'left_wing': [{'type': 'box', 'size': [CHORD, SPAN / 2, t_w], 'xyz': [cx, SPAN / 4, cz], 'rpy': [0, -ia, 0]}],
    'right_wing': [{'type': 'box', 'size': [CHORD, SPAN / 2, t_w], 'xyz': [cx, -SPAN / 4, cz], 'rpy': [0, -ia, 0]}],
    'h_stab': [{'type': 'box', 'size': [HT_X_LE - HT_X_HINGE, 2 * HT_HALF_SPAN, 0.16 * ht_c],
                'xyz': [0.5 * (HT_X_LE + HT_X_HINGE), 0, HT_Z], 'rpy': [0, 0, 0]}],
    'v_stab': [bbox_box(v_stab, 0.9)],
    'left_nacelle': [bbox_box(nacL, 0.9)],
    'right_nacelle': [bbox_box(nacR, 0.9)],
    'water_tank': [],   # inside the hull, a collision here would count twice for buoyancy
}
for sgn, nm in ((1, 'left_float'), (-1, 'right_float')):
    COLL[nm] = [{'type': 'box', 'size': [f_l, fvol / (f_l * f_h), f_h],
                 'xyz': [fb.center.x, sgn * FLOAT_Y, fb.center.z], 'rpy': [0, 0, 0]},
                {'type': 'box', 'size': [0.08, 0.009, 0.10],
                 'xyz': [-0.10, sgn * FLOAT_Y, fb.zmax + 0.05], 'rpy': [0, 0, 0]}]


def local_box(shape, link):
    p, Rm = link_pose_in_base(link)
    m = tess(shape, (0.001, 0.4))
    v = (m.vertices - p) @ Rm
    lo, hi = v.min(0), v.max(0)
    return [{'type': 'box', 'size': (hi - lo).tolist(), 'xyz': (0.5 * (lo + hi)).tolist(), 'rpy': [0, 0, 0]}]


COLL['left_aileron'] = local_box(aileronL, 'left_aileron')
COLL['right_aileron'] = local_box(aileronR, 'right_aileron')
COLL['elevator'] = local_box(elevator, 'elevator')
COLL['rudder'] = local_box(rudder, 'rudder')
for nm in ('left_propeller', 'right_propeller'):
    COLL[nm] = [{'type': 'cylinder', 'radius': SPIN_R, 'length': SPIN_X1 - SPIN_X0,
                 'xyz': [0, 0, 0.5 * (SPIN_X0 + SPIN_X1) - PROP_X], 'rpy': [0, 0, 0]}]
geo['collision'] = COLL
geo['hull_box_volume'] = float(sum(np.prod(b['size']) for b in COLL['fuselage']))
geo['misc'] = {'deck_z': DECK_Z, 'keel_z': MEAS['hull_misc']['keel_z'], 'step_x': step_x,
               'nose_x': MEAS['hull_misc']['nose_x'], 'stern_x': MEAS['hull_misc']['stern_x'],
               'float_bottom_z': fb.zmin, 'float_y': FLOAT_Y, 'quarter_chord': [x_qc, 0.0, z_qc]}

with open(os.path.join(CAD, 'geometry.json'), 'w') as f:
    json.dump(geo, f, indent=1)

# renders
print('rendering ...')
render.save_views(render_parts, os.path.join(RENDERS, 'cad_views.png'),
                  title='CAD model (REP-103: X fwd, Y left, Z up; origin + at wing LE root)')
sys.path.insert(0, os.path.join(ROOT, 'tools'))
from objparse import read_obj_groups  # noqa: E402

_, G = read_obj_groups(os.path.join(ROOT, 'cl415.obj'))
keep = ('fuselage', 'ailes', 'volet', 'aileron', 'derive', 'profondeur', 'direction', 'moteurs', 'bol', 'helice',
        'flotteur')
org = np.array(MEAS['origin_obj'])
Rm = np.array(MEAS['rot_obj_to_rep'])
objm = trimesh.util.concatenate([trimesh.Trimesh(MEAS['scale'] * (g.vertices - org) @ Rm.T, g.faces, process=False)
                                 for n, g in G.items() if n.startswith(keep)])
for v in ('top', 'side', 'front'):
    render.save_single(render_parts, os.path.join(RENDERS, f'cad_vs_obj_{v}.png'), v,
                       title=f'CAD (shaded) vs scaled OBJ outline (red dashed), {v} view',
                       outline=render.silhouette(objm, v))
print(f'done in {time.time() - T:.1f}s')
