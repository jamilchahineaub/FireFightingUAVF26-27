"""step 1: inspect cl415.obj, work out scale/orientation, measure the proportions the cad needs.

run from the project root:  .venv/Scripts/python obj_inspect/inspect_obj.py
writes obj_inspect/report.md, obj_inspect/measurements.json, obj_inspect/renders/*.png
"""
import json
import os
import sys

import numpy as np
import trimesh

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tools'))
from objparse import read_obj_groups  # noqa: E402
import render  # noqa: E402

OUT = os.path.join(ROOT, 'obj_inspect')
os.makedirs(os.path.join(OUT, 'renders'), exist_ok=True)

SPAN_SPEC = 2.51     # m
CHORD_SPEC = 0.278   # m
REAL_CL415 = {'span': 28.63, 'length': 19.82, 'height': 8.98}  # published dims, m

V, G = read_obj_groups(os.path.join(ROOT, 'cl415.obj'))
with open(os.path.join(ROOT, 'cl415.obj'), errors='ignore') as f:
    n_poly = sum(1 for line in f if line.startswith('f '))


def cat(*names):
    return trimesh.util.concatenate([G[n] for n in names])


def segs_at(mesh, normal, origin):
    return trimesh.intersections.mesh_plane(mesh, normal, origin)


def bounds_at(mesh, normal, origin):
    s = segs_at(mesh, normal, origin)
    if len(s) == 0:
        return None
    p = s.reshape(-1, 3)
    return p.min(0), p.max(0)


# raw stats
tris = sum(len(g.faces) for g in G.values())
merged = trimesh.util.concatenate(list(G.values()))
merged.merge_vertices()
lo, hi = V.min(0), V.max(0)
ext = hi - lo
wt_groups = [n for n, g in G.items() if g.is_watertight]

# orientation
wing = cat('ailes_ailes', 'voletG_voletG', 'voletD_voletD', 'aileronG_aileronG', 'aileronD_aileronD')
span_obj = np.ptp(G['ailes_ailes'].vertices[:, 2])
nose_x = G['fuselage_fuselage'].vertices[:, 0].min()
cockpit_x = G['vitres_vitres'].bounds.mean(0)[0]
fin_top_y = G['direction_direction'].vertices[:, 1].max()
keel_y = G['fuselage_fuselage'].vertices[:, 1].min()
left_float_z = G['flotteurG_flotteurG'].bounds.mean(0)[2]   # G = gauche = left

scale = SPAN_SPEC / span_obj

# wing chord, taper, incidence
chords = []
for z in [1.5, 3, 5, 7, 9, 11, 12.5, 13.5]:
    b = bounds_at(wing, [0, 0, 1], [0, 0, z])
    chords.append((z, b[0][0], b[1][0]))
P = segs_at(wing, [0, 0, 1], [0, 0, 5.0]).reshape(-1, 3)
le = P[P[:, 0].argmin()]
te = P[P[:, 0].argmax()]
chord_obj = float(np.linalg.norm(te[:2] - le[:2]))
incidence = float(np.degrees(np.arctan2(le[1] - te[1], te[0] - le[0])))
u_chord = (te - le) / np.linalg.norm(te - le)
u_chord[2] = 0
u_chord /= np.linalg.norm(u_chord)
# max thickness of the obj section, measured normal to the chord line
nrm = np.array([u_chord[1], -u_chord[0], 0]) * -1
tc_obj = float(np.ptp((P - le) @ nrm) / chord_obj)

# cad origin: wing le root on the chord line. keep the obj quarter chord where it is
# (so tail arm / ac position stay true to the obj) and fit the 0.278 m chord around it
q_obj = le + 0.25 * chord_obj * u_chord
origin_obj = q_obj - 0.25 * (CHORD_SPEC / scale) * u_chord
origin_obj[2] = 0.0
R = np.array([[-1, 0, 0], [0, 0, 1], [0, 1, 0]], float)   # obj (x aft, y up, z left) -> rep103 (x fwd, y left, z up)


def to_rep(p):
    return scale * (np.asarray(p, float) - origin_obj) @ R.T


def xr(x_obj):
    return -scale * (x_obj - origin_obj[0])


def zr(y_obj):
    return scale * (y_obj - origin_obj[1])


def yr(z_obj):
    return scale * z_obj


# hull keel line + step
fus = G['fuselage_fuselage']
keel = []
for x in np.arange(-9.85, 9.41, 0.05):
    s = segs_at(fus, [1, 0, 0], [x, 0, 0])
    if len(s):
        keel.append((x, s.reshape(-1, 3)[:, 1].min()))
keel = np.array(keel)
dk = np.diff(keel[:, 1])
i_step = int(np.argmax(dk))
x_step = float(0.5 * (keel[i_step, 0] + keel[i_step + 1, 0]))
HULL_TOP = -0.632   # flat deck height in the obj; fin, wing fairing and cockpit glazing sit above it

FRACS = [0.0, 0.03, 0.08, 0.15, 0.25, 0.40, 0.55, 0.70, 0.82, 0.91, 0.97, 1.0]


def half_breadth(segs, h):
    a, b = segs[:, 0, 1], segs[:, 1, 1]
    m = (np.minimum(a, b) <= h) & (np.maximum(a, b) >= h)
    if not m.any():
        return 0.0
    sa, sb = segs[m, 0], segs[m, 1]
    d = sb[:, 1] - sa[:, 1]
    t = np.where(np.abs(d) > 1e-12, (h - sa[:, 1]) / np.where(np.abs(d) > 1e-12, d, 1), 0.0)
    zz = sa[:, 2] + t * (sb[:, 2] - sa[:, 2])
    return float(np.abs(zz).max())


def hull_station(x):
    s = segs_at(fus, [1, 0, 0], [x, 0, 0])
    p = s.reshape(-1, 3)
    zk = p[:, 1].min()
    zt = min(p[:, 1].max(), HULL_TOP)
    ws = []
    for f in FRACS:
        h = zk + f * (zt - zk)
        if f == 0.0:
            ws.append(0.0)
        elif f == 1.0:
            ws.append(0.75 * half_breadth(s, zt - 0.02 * (zt - zk)))
        else:
            ws.append(half_breadth(s, h))
    return {'x': xr(x), 'z_keel': zr(zk), 'z_top': zr(zt), 'half_breadth': [scale * w for w in ws]}


fore_x = [-9.86, -9.7, -9.4, -9.0, -8.5, -7.8, -7.0, -6.0, -5.0, -4.0, -3.0, -2.0, x_step - 0.08]
aft_x = [x_step + 0.12, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 8.8, 9.25, 9.42]


def good_station(x, step):
    # the obj hull has door/window openings; slide off a slice that falls into one
    for k in range(12):
        st = hull_station(x + k * step)
        if sum(w == 0.0 for w in st['half_breadth'][1:-1]) <= 1:
            return st
    return hull_station(x)


hull = {'fracs': FRACS,
        'forebody': [good_station(x, -0.05) for x in fore_x],
        'afterbody': [good_station(x, 0.05) for x in aft_x]}

# nacelle (left side) as ellipse stations
mot = G['moteurs_moteurs']
mL = trimesh.Trimesh(mot.vertices, mot.faces[mot.vertices[mot.faces][:, :, 2].mean(1) > 0], process=False)
nac = []
for x in [-5.10, -4.95, -4.75, -4.4, -3.8, -3.0, -2.3, -1.7, -1.1, -0.6, -0.25, -0.12]:
    b = bounds_at(mL, [1, 0, 0], [x, 0, 0])
    (x0, y0, z0), (x1, y1, z1) = b
    nac.append({'x': xr(x), 'z_c': zr(0.5 * (y0 + y1)), 'y_c': yr(0.5 * (z0 + z1)),
                'half_w': scale * 0.5 * (z1 - z0), 'half_h': scale * 0.5 * (y1 - y0)})

# spinner + prop
spin = G['bolG_bolG']
prop = G['heliceG_heliceG']
blades = prop.split(only_watertight=False)
pc = prop.bounds.mean(0)
widths = []
for bl in blades:
    v = bl.vertices[:, 1:] - pc[1:]
    r = np.linalg.norm(v, axis=1)
    rad = v[r.argmax()] / r.max()
    perp = np.array([-rad[1], rad[0]])
    for rr in [0.3, 0.6, 0.9]:
        m = np.abs(r - rr * r.max()) < 0.08 * r.max()
        if m.sum() > 1:
            widths.append((rr, float(np.ptp(v[m] @ perp))))
        else:
            widths.append((rr, float('nan')))
blade_w = {rr: np.mean([w for q, w in widths if q == rr]) for rr in [0.3, 0.6, 0.9]}
propd = {'center': to_rep(pc).tolist(), 'diameter': scale * float(np.ptp(prop.vertices[:, 1])),
         'n_blades': len(blades), 'blade_chord_at_r': {str(k): scale * v for k, v in blade_w.items()},
         'blade_axial_extent': scale * float(np.ptp(prop.vertices[:, 0])),
         'spinner_x': [xr(spin.bounds[1][0]), xr(spin.bounds[0][0])],
         'spinner_diameter': scale * float(np.ptp(spin.vertices[:, 1]))}

# float (left): body stations below the pylon, then the pylon
flt = G['flotteurG_flotteurG']
fz = flt.bounds.mean(0)[2]
fbody = []
for x in [-3.41, -3.3, -3.1, -2.85, -2.5, -2.1, -1.7, -1.3, -0.95, -0.7, -0.55, -0.46]:
    s = segs_at(flt, [1, 0, 0], [x, 0, 0]).reshape(-1, 3)
    s = s[s[:, 1] < -1.60]
    if len(s) == 0:
        continue
    fbody.append({'x': xr(x), 'z_lo': zr(s[:, 1].min()), 'z_hi': zr(s[:, 1].max()),
                  'half_w': scale * 0.5 * np.ptp(s[:, 2])})
pyl = []
for y in [-1.55, -1.1, -0.72]:
    b = bounds_at(flt, [0, 1, 0], [0, y, 0])
    pyl.append({'z': zr(y), 'x_le': xr(b[0][0]), 'x_te': xr(b[1][0]), 't': scale * (b[1][2] - b[0][2])})
floatd = {'y_c': yr(fz), 'body': fbody, 'pylon': pyl}

# tail
stab = G['derive_derive']
elev = G['profondeur_profondeur']
hs = cat('derive_derive', 'profondeur_profondeur')
b2 = bounds_at(hs, [0, 0, 1], [0, 0, 2.0])
stab_te = bounds_at(stab, [0, 0, 1], [0, 0, 2.0])[1][0]
elev_inner = float(np.abs(elev.vertices[:, 2]).min())
b_in = bounds_at(elev, [0, 0, 1], [0, 0, elev_inner + 0.1])
htail = {'x_le': xr(b2[0][0]), 'x_hinge': xr(stab_te), 'x_te': xr(b2[1][0]),
         'z_chord': zr(0.5 * (b2[0][1] + b2[1][1])), 'half_span': scale * float(stab.vertices[:, 2].max()),
         't_over_c': float((b2[1][1] - b2[0][1]) / (b2[1][0] - b2[0][0])),
         'elevator_inner_y': scale * elev_inner, 'elevator_inner_te_x': xr(b_in[1][0]),
         'tip_round_start_y': scale * 4.9}
fl2 = G['derive2_derive2']
fl2L = trimesh.Trimesh(fl2.vertices, fl2.faces[fl2.vertices[fl2.faces][:, :, 2].mean(1) > 0], process=False)
fb = fl2L.bounds
finlet = {'x_le': xr(fb[0][0]), 'x_te': xr(fb[1][0]), 'z_lo': zr(fb[0][1]), 'z_hi': zr(fb[1][1]),
          'y_c': yr(0.5 * (fb[0][2] + fb[1][2])), 't': scale * (fb[1][2] - fb[0][2])}
# chord of the finlet at the stab and at its ends
fl_sec = {}
for y in [fb[0][1] + 0.05, 1.02, fb[1][1] - 0.05]:
    b = bounds_at(fl2L, [0, 1, 0], [0, y, 0])
    fl_sec[round(zr(y), 4)] = [xr(b[0][0]), xr(b[1][0])]
finlet['sections'] = fl_sec

rud = G['direction_direction']
fin_rows = []
for y in [-0.6, 0.0, 0.5, 1.5, 2.0, 2.5, 3.0]:
    s = segs_at(fus, [0, 1, 0], [0, y, 0]).reshape(-1, 3)
    s = s[s[:, 0] > 4.5]
    r = bounds_at(rud, [0, 1, 0], [0, y, 0])
    fin_rows.append((y, s[:, 0].min(), r[0][0], r[1][0], np.ptp(s[:, 2])))
fin_rows = np.array(fin_rows)
# straight-line fits (y = 1.0 skipped, the stab root fairing sits there)
le_fit = np.polyfit(fin_rows[:, 0], fin_rows[:, 1], 1)
hinge_fit = np.polyfit(fin_rows[:, 0], fin_rows[:, 2], 1)
te_fit = np.polyfit(fin_rows[:, 0], fin_rows[:, 3], 1)
fin_tip_y = float(rud.vertices[:, 1].max())
fin_root_y = HULL_TOP
fin_chord_mid = np.polyval(te_fit, 2.0) - np.polyval(le_fit, 2.0)
fin_t_mid = float(fin_rows[fin_rows[:, 0] == 2.0, 4][0])
vtail = {'z_root': zr(fin_root_y), 'z_tip': zr(fin_tip_y),
         'root': {'x_le': xr(np.polyval(le_fit, fin_root_y)), 'x_hinge': xr(np.polyval(hinge_fit, fin_root_y)),
                  'x_te': xr(np.polyval(te_fit, fin_root_y))},
         'tip': {'x_le': xr(np.polyval(le_fit, fin_tip_y)), 'x_hinge': xr(np.polyval(hinge_fit, fin_tip_y)),
                 'x_te': xr(np.polyval(te_fit, fin_tip_y))},
         't_over_c': float(fin_t_mid / fin_chord_mid),
         'le_sweep_deg': float(np.degrees(np.arctan(le_fit[0]))),
         'hinge_sweep_deg': float(np.degrees(np.arctan(hinge_fit[0])))}

# ailerons and flaps
ail = G['aileronG_aileronG']
wing_te_at_ail = bounds_at(G['ailes_ailes'], [0, 0, 1], [0, 0, 11.0])[1][0]
wing_te_at_flap = bounds_at(G['ailes_ailes'], [0, 0, 1], [0, 0, 5.0])[1][0]
fl = G['voletG_voletG']
xle, xte = le[0], te[0]
cs = {'aileron_y': [yr(ail.vertices[:, 2].min()), yr(ail.vertices[:, 2].max())],
      'aileron_cf_c': float((xte - wing_te_at_ail) / (xte - xle)),
      'flap_y': [yr(fl.vertices[:, 2].min()), yr(fl.vertices[:, 2].max())],
      'flap_cf_c': float((xte - wing_te_at_flap) / (xte - xle))}

# hull top / keel / misc in rep frame
misc = {'hull_top_z': zr(HULL_TOP), 'keel_z': zr(keel_y), 'nose_x': xr(nose_x),
        'stern_x': xr(fus.vertices[:, 0].max()), 'rudder_te_max_x': xr(rud.vertices[:, 0].max()),
        'step_x': xr(x_step), 'hull_half_beam_max': scale * float(np.abs(fus.vertices[:, 2]).max()),
        'wing_lower_surface_z_obj': -0.75}

meas = {
    'source': 'cl415.obj (Blender 2.69 export)',
    'obj_units': 'm (full scale 1:1)',
    'obj_axes': {'nose': '-X', 'up': '+Y', 'left': '+Z'},
    'scale': scale,
    'origin_obj': origin_obj.tolist(),
    'rot_obj_to_rep': R.tolist(),
    'wing': {'span_obj': float(span_obj), 'chord_obj': chord_obj, 'chord_scaled': chord_obj * scale,
             'chord_spec': CHORD_SPEC, 'incidence_deg': incidence, 't_over_c_obj': tc_obj,
             'le_obj': le.tolist(), 'te_obj': te.tolist(),
             'chord_vs_station_obj': [(float(z), float(b - a)) for z, a, b in chords],
             'z_chord_le_rep': 0.0},
    'control_surfaces': cs,
    'hull': hull,
    'hull_misc': misc,
    'keel_line_rep': [(xr(x), zr(y)) for x, y in keel[::4]],
    'nacelle_left': nac,
    'nacelle_y_c': nac[3]['y_c'],
    'prop_left': propd,
    'float_left': floatd,
    'htail': htail,
    'finlet_left': finlet,
    'vtail': vtail,
}


def jsonable(o):
    if isinstance(o, dict):
        return {str(k): jsonable(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [jsonable(v) for v in o]
    if isinstance(o, (np.floating, np.integer)):
        return o.item()
    if isinstance(o, np.ndarray):
        return o.tolist()
    return o


with open(os.path.join(OUT, 'measurements.json'), 'w') as f:
    json.dump(jsonable(meas), f, indent=1)

# renders in the scaled rep103 frame
COL = {'hull': (0.85, 0.82, 0.30), 'wing': (0.92, 0.92, 0.92), 'tail': (0.80, 0.25, 0.20),
       'nac': (0.55, 0.55, 0.60), 'prop': (0.15, 0.15, 0.15), 'float': (0.30, 0.45, 0.80),
       'misc': (0.65, 0.65, 0.65)}


def colour(n):
    if n.startswith(('fuselage', 'vitres', 'porte', 'trappe', 'roof')):
        return COL['hull']
    if n.startswith(('ailes', 'volet', 'aileron', 'surailes')):
        return COL['wing']
    if n.startswith(('derive', 'profondeur', 'direction')):
        return COL['tail']
    if n.startswith(('moteurs', 'bol')):
        return COL['nac']
    if n.startswith('helice'):
        return COL['prop']
    if n.startswith('flotteur'):
        return COL['float']
    return COL['misc']


skip = ('instr', 'panel', 'centrale', 'manettes', 'throttle', 'mixture', 'cotes', 'cloison', 'interieur',
        'planchet', 'inttour', 'trous')
parts = []
for n, g in G.items():
    if n.startswith(skip) or len(g.faces) < 20 and not n.startswith(('aileron', 'porte', 'trappe')):
        continue
    if np.ptp(g.vertices, 0).max() < 0.2:
        continue
    m = trimesh.Trimesh(to_rep(g.vertices), g.faces, process=False)
    parts.append((m, colour(n)))
render.save_views(parts, os.path.join(OUT, 'renders', 'obj_views.png'),
                  title=f'cl415.obj scaled x{scale:.5f} into REP-103 (X fwd, Y left, Z up), origin = wing LE root (+)')
for v in ['top', 'side', 'front']:
    render.save_single(parts, os.path.join(OUT, 'renders', f'obj_{v}.png'), v,
                       title=f'cl415.obj, {v} view (scaled, REP-103)')
# raw obj axes, unscaled, so the original orientation can be checked too
raw = [(trimesh.Trimesh(g.vertices, g.faces, process=False), colour(n)) for n, g in G.items()
       if not n.startswith(skip) and np.ptp(g.vertices, 0).max() > 0.2]
import matplotlib.pyplot as plt  # noqa: E402
fig, axs = plt.subplots(1, 3, figsize=(18, 5.5))
for ax, (a, b, lab) in zip(axs, [(0, 2, 'raw X vs Z (top)'), (0, 1, 'raw X vs Y (side)'), (2, 1, 'raw Z vs Y (front)')]):
    for m, c in raw:
        ax.scatter(m.vertices[:, a], m.vertices[:, b], s=0.2, color=c)
    ax.set_xlabel('XYZ'[a] + ' [obj units]')
    ax.set_ylabel('XYZ'[b] + ' [obj units]')
    ax.set_aspect('equal')
    ax.grid(True, lw=0.3)
    ax.set_title(lab)
fig.suptitle('cl415.obj in its own axes: nose at -X, fin up +Y, left float (flotteurG) at +Z')
fig.tight_layout()
fig.savefig(os.path.join(OUT, 'renders', 'obj_raw_axes.png'), dpi=150)
plt.close(fig)

# report
L = []
L.append('# Step 1: OBJ inspection\n')
L.append('Generated by `obj_inspect/inspect_obj.py`. Numbers in the scaled frame are in metres, REP-103 '
         '(X forward, Y left, Z up), origin at the wing leading edge root on the centreline.\n')
L.append('## File\n')
L.append(f'- vertices: {len(V)}, polygon records: {n_poly}, triangles after fan split: {tris}')
L.append(f'- object groups: {len(G)} (hull, wing, flaps, ailerons, tail, nacelles, props, floats, gear, doors, '
         'plus about 100 small cockpit/instrument objects)')
L.append(f'- bounding box (obj units): min {np.round(lo, 3).tolist()}, max {np.round(hi, 3).tolist()}, '
         f'size {np.round(ext, 3).tolist()}')
L.append(f'- units: metres at full scale. Size {ext[0]:.2f} x {ext[2]:.2f} x {ext[1]:.2f} (length x span x height) '
         f'vs the real CL-415 {REAL_CL415["length"]} x {REAL_CL415["span"]} x {REAL_CL415["height"]} m')
L.append(f'- watertight: no. The merged mesh is open ({len(wt_groups)} of {len(G)} groups are closed on their own; '
         'the hull, wing and nacelles are open shells). Fine for proportions, useless for volumes, '
         'which is another reason to rebuild in CAD.')
L.append('- no .mtl file next to the obj (only referenced), materials ignored\n')
L.append('## Orientation\n')
L.append(f'- nose is -X (hull nose at x = {nose_x:.2f}, cockpit glazing at x = {cockpit_x:.2f})')
L.append(f'- up is +Y (fin tip at y = {fin_top_y:.2f}, keel at y = {keel_y:.2f})')
L.append(f'- left is +Z (flotteurG, G = gauche, sits at z = {left_float_z:+.2f}). Check: forward x up = right, '
         '(-X) x (+Y) = -Z, so left is +Z. Conversion to REP-103: x = -X, y = +Z, z = +Y (det = +1)\n')
L.append('## Scale\n')
L.append(f'- wingspan in the file: {span_obj:.3f} (tip to tip of `ailes`, winglet caps included)')
L.append(f'- scale factor to 2.51 m: **{scale:.6f}** (about 1:{1 / scale:.2f})')
L.append(f'- obj chord {chord_obj:.3f} m, constant from root to the tip cap (stations: '
         + ', '.join(f'{b - a:.3f}' for _, a, b in chords) + ')')
L.append(f'- scaled chord: **{chord_obj * scale:.4f} m** vs spec 0.278 m, so the OBJ wing is '
         f'{100 * (chord_obj * scale / CHORD_SPEC - 1):.1f} % wider than the spec. Aspect ratio: OBJ '
         f'{span_obj / chord_obj:.2f}, spec {SPAN_SPEC / CHORD_SPEC:.2f}.')
L.append('- no taper in the OBJ, so the CAD wing uses a constant 0.278 m chord as the spec says')
L.append(f'- wing incidence relative to the hull datum (X axis): {incidence:.2f} deg; OBJ section t/c about {tc_obj:.3f} '
         '(NACA 4417 is 0.17, so the OBJ is close to it)')
L.append('- the CAD wing keeps the OBJ quarter-chord position and the narrower chord is centred on it, '
         f'so the CAD leading edge sits {1000 * 0.25 * (chord_obj * scale - CHORD_SPEC):.1f} mm aft of the scaled OBJ leading edge\n')
L.append('## What the OBJ contains\n')
L.append('| obj group | what it is | used for |')
L.append('|---|---|---|')
for a, b, c in [('fuselage', 'boat hull + fin + wing root fairing', 'hull offsets, fin planform'),
                ('ailes', 'wing (fixed part) incl. winglet caps', 'span, chord, incidence'),
                ('voletG/D, aileronG/D', 'flaps, ailerons', 'aileron span and chord ratio'),
                ('derive, profondeur', 'h-stab, split elevator', 'tail planform, hinge line'),
                ('derive2', 'tailplane finlets (two)', 'finlets on the h-stab'),
                ('direction', 'rudder', 'hinge and trailing edge lines'),
                ('moteurs, bolG/D, heliceG/D', 'nacelles, spinners, 4-blade props', 'nacelle stations, prop disc'),
                ('flotteurG/D', 'wingtip floats + pylons', 'float stations, pylon'),
                ('roue*, axe*, trappe*, porte*', 'landing gear, doors', 'not modelled'),
                ('~100 cockpit objects', 'instruments, seats', 'not modelled')]:
    L.append(f'| {a} | {b} | {c} |')
L.append('')
L.append('Nothing the spec asks for is missing from the OBJ. The finlets (derive2) are not in the requested '
         'part list, so they are modelled as part of the h-stab body.\n')
L.append('## Key proportions after scaling (REP-103, m)\n')
L.append(f'- hull: nose x = {misc["nose_x"]:.3f}, stern x = {misc["stern_x"]:.3f}, keel z = {misc["keel_z"]:.3f}, '
         f'deck z = {misc["hull_top_z"]:.3f}, max half beam {misc["hull_half_beam_max"]:.3f}, planing step at x = {misc["step_x"]:.3f}')
L.append(f'- nacelles at y = +/-{nac[3]["y_c"]:.3f}, prop disc diameter {propd["diameter"]:.3f} m, '
         f'{propd["n_blades"]} blades, prop plane x = {propd["center"][0]:.3f}, axis z = {propd["center"][2]:.3f}')
L.append(f'- floats at y = +/-{floatd["y_c"]:.3f}, float bottom z = {min(b["z_lo"] for b in fbody):.3f}')
L.append(f'- h-stab: LE x = {htail["x_le"]:.3f}, hinge x = {htail["x_hinge"]:.3f}, TE x = {htail["x_te"]:.3f}, '
         f'half span {htail["half_span"]:.3f}, chord line z = {htail["z_chord"]:.3f}, t/c {htail["t_over_c"]:.3f}')
L.append(f'- fin: root z = {vtail["z_root"]:.3f}, tip z = {vtail["z_tip"]:.3f}, LE sweep {vtail["le_sweep_deg"]:.1f} deg, '
         f'hinge sweep {vtail["hinge_sweep_deg"]:.1f} deg, t/c {vtail["t_over_c"]:.3f}')
L.append(f'- ailerons from y = {cs["aileron_y"][0]:.3f} to {cs["aileron_y"][1]:.3f}, chord ratio {cs["aileron_cf_c"]:.3f}\n')
L.append('## Renders\n')
L.append('- `renders/obj_raw_axes.png`: the file in its own axes')
L.append('- `renders/obj_views.png`: top/side/front/iso after scaling and conversion to REP-103 (origin marked +)')
L.append('- `renders/obj_top.png`, `obj_side.png`, `obj_front.png`: the same views, larger')
with open(os.path.join(OUT, 'report.md'), 'w', encoding='utf-8') as f:
    f.write('\n'.join(L) + '\n')
print('\n'.join(L))
