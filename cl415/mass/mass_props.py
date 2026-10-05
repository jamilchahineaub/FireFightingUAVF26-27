"""step 3: mass budget, cg check and link inertials (tank full and empty)

two passes around the solidworks build:
  .venv/Scripts/python mass/mass_props.py plan
      budget + cg check on the cad numbers (cad/geometry.json). if the cg misses 25-30 % mac the
      battery moves. writes mass/plan.json and cad/parts/eq_battery.step
  .venv/Scripts/python cad/solidworks/build_sw.py
      solidworks parts + assembly, density per part from the plan, mass properties measured there
  .venv/Scripts/python mass/mass_props.py final
      reads cad/solidworks/sw_mass_props.json, builds the urdf link inertials, re-checks the cg,
      writes mass/mass_props.json and mass/report.md

every structure part is a uniform solid at its budget mass. the hull, nacelles and floats are real
hollow skins in the cad, so that is a shell model for them. battery, avionics, motors, escs and
servos are small envelope parts at their mounting points.
"""
import json
import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GEO = json.load(open(os.path.join(ROOT, 'cad', 'geometry.json')))
OUT = os.path.join(ROOT, 'mass')
PARTS_DIR = os.path.join(ROOT, 'cad', 'parts')
SW_JSON = os.path.join(ROOT, 'cad', 'solidworks', 'sw_mass_props.json')

M_TOTAL = 10.5
M_WATER = 3.0
MAC = GEO['aero']['wing']['chord']
X_LE_MAC = GEO['aero']['wing']['x_le_mac']
CG_BAND = (25.0, 30.0)          # % mac
CG_TARGET_EMPTY = 28.0          # % mac. the water sits at 25 %, so the full case lands near 27 %

# part, urdf link, kg, what it is
BUDGET = [
    ('fuselage', 'fuselage', 2.08, 'Hull: composite skin, frames, step, wing saddle (1.70) plus wiring, connectors, '
                                   'plumbing and fasteners (0.38), spread over the hull skin'),
    ('left_wing', 'left_wing', 0.50, 'Left wing half, foam core + glass skin + spar'),
    ('right_wing', 'right_wing', 0.50, 'Right wing half'),
    ('left_aileron', 'left_aileron', 0.035, 'Left aileron'),
    ('right_aileron', 'right_aileron', 0.035, 'Right aileron'),
    ('h_stab', 'h_stab', 0.14, 'H-stab incl. finlets'),
    ('elevator', 'elevator', 0.06, 'Elevator (both halves + torque tube)'),
    ('v_stab', 'v_stab', 0.09, 'V-stab'),
    ('rudder', 'rudder', 0.035, 'Rudder'),
    ('left_nacelle', 'left_nacelle', 0.10, 'Left nacelle fairing'),
    ('right_nacelle', 'right_nacelle', 0.10, 'Right nacelle fairing'),
    ('left_propeller', 'left_propeller', 0.05, 'Left 14x7 prop + spinner'),
    ('right_propeller', 'right_propeller', 0.05, 'Right 14x7 prop + spinner'),
    ('left_float', 'left_float', 0.13, 'Left float + pylon'),
    ('right_float', 'right_float', 0.13, 'Right float + pylon'),
    ('water_tank', 'water_tank', 0.25, 'Water tank, empty, with outlet valve'),
    ('eq_left_motor', 'left_nacelle', 0.30, 'Left motor, 600-800 W outrunner'),
    ('eq_right_motor', 'right_nacelle', 0.30, 'Right motor'),
    ('eq_left_esc', 'left_nacelle', 0.07, 'Left ESC'),
    ('eq_right_esc', 'right_nacelle', 0.07, 'Right ESC'),
    ('eq_left_aileron_servo', 'left_wing', 0.035, 'Left aileron servo'),
    ('eq_right_aileron_servo', 'right_wing', 0.035, 'Right aileron servo'),
    ('eq_avionics', 'fuselage', 0.30, 'Avionics: FC, GPS, telemetry, RC receiver, power module, airspeed'),
    ('eq_elevator_servo', 'fuselage', 0.035, 'Elevator servo'),
    ('eq_rudder_servo', 'fuselage', 0.035, 'Rudder servo'),
    ('eq_door_servo', 'fuselage', 0.035, 'Tank door servo'),
    ('eq_battery', 'fuselage', 2.00, 'Battery, 6S 16 Ah LiPo (355 Wh)'),
]
WATER = ('water_3L', 'water_tank', M_WATER, 'Water, 3 L')
BATT_DIM = (0.197, 0.077, 0.064)    # m

dry = sum(r[2] for r in BUDGET)
assert abs(dry - (M_TOTAL - M_WATER)) < 1e-9, dry

tank_bb = GEO['tank']['outer_bbox']
BATT_START = (tank_bb[1][0] + 0.010 + BATT_DIM[0] / 2, 0.0, -0.250 + BATT_DIM[2] / 2)   # floor, ahead of the tank
BATT_TRAY_Z = tank_bb[1][2] + 0.006 + BATT_DIM[2] / 2                                 # tray on top of the tank


def pct_mac(x):
    return 100.0 * (X_LE_MAC - x) / MAC


def shift(I, m, d):
    d = np.asarray(d, float)
    return I + m * (d.dot(d) * np.eye(3) - np.outer(d, d))


def combine(items):
    """items: (mass, com, inertia about own com). -> mass, com, inertia about com"""
    m = sum(i[0] for i in items)
    c = sum(i[0] * np.asarray(i[1]) for i in items) / m
    I = sum(shift(np.asarray(i[2]), i[0], np.asarray(i[1]) - c) for i in items)
    return m, c, I


def box_inertia(m, d):
    a, b, c = d
    return np.diag([m * (b * b + c * c), m * (a * a + c * c), m * (a * a + b * b)]) / 12.0


def cad_body(part, m, batt_pos=None):
    """mass props of a part at mass m from the cad mesh numbers (uniform density)"""
    if part == 'eq_battery':
        return m, np.array(batt_pos, float), box_inertia(m, BATT_DIM)
    p = GEO['parts'][part]
    return m, np.array(p['centroid']), np.array(p['inertia_unit_density']) * m / p['volume']


def links_from(bodies, water):
    """bodies: dict part -> (m, com, I). returns dict link -> combined (m, com, I)"""
    groups = {}
    for part, link, m, _ in BUDGET + ([WATER] if water else []):
        groups.setdefault(link, []).append(bodies[part])
    return {k: combine(v) for k, v in groups.items()}


def aircraft(links):
    return combine(list(links.values()))


def to_link_frame(link, m, c, I):
    L = GEO['links'][link]
    p = np.array(L['xyz'], float)
    r, pp, y = L['rpy']
    Rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    Ry = np.array([[math.cos(pp), 0, math.sin(pp)], [0, 1, 0], [-math.sin(pp), 0, math.cos(pp)]])
    Rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    R = Rz @ Ry @ Rx
    Il = R.T @ I @ R
    Il[np.abs(Il) < 1e-12] = 0.0
    return R.T @ (c - p), Il


def battery_box(pos):
    import cadquery as cq
    L, W, H = BATT_DIM
    return cq.Solid.makeBox(L, W, H, pnt=cq.Vector(pos[0] - L / 2, pos[1] - W / 2, pos[2] - H / 2))


def mesh_vol(shape):
    import trimesh
    if shape is None or not shape.Solids():
        return 0.0
    vs, tr = shape.tessellate(0.0002, 0.1)
    if not tr:
        return 0.0
    return abs(float(trimesh.Trimesh(np.array([v.toTuple() for v in vs]), np.array(tr)).volume))


def battery_fit(pos):
    """outside-the-cavity volume and overlaps with the tank and other equipment (m^3)"""
    import cadquery as cq

    def load(p):
        return cq.importers.importStep(p).val().scale(0.001)
    b = battery_box(pos)
    cav = load(os.path.join(ROOT, 'cad', 'build', 'hull_cavity.step'))
    res = {'outside_cavity': mesh_vol(b.cut(cav))}
    for part in ['water_tank', 'eq_avionics', 'eq_door_servo', 'eq_elevator_servo', 'eq_rudder_servo']:
        res[part] = mesh_vol(b.intersect(load(os.path.join(PARTS_DIR, f'{part}.step'))))
    return res, b


def plan():
    def totals(pos):
        bodies = {part: cad_body(part, m, pos) for part, link, m, _ in BUDGET + [WATER]}
        return [aircraft(links_from(bodies, w)) for w in (False, True)]

    log = []
    pos = np.array(BATT_START)
    e, f = totals(pos)
    log.append({'where': 'hull floor, right ahead of the tank', 'pos': pos.tolist(),
                'cg_empty': pct_mac(e[1][0]), 'cg_full': pct_mac(f[1][0])})
    if not all(CG_BAND[0] <= pct_mac(t[1][0]) <= CG_BAND[1] for t in (e, f)):
        pos = np.array([0.0, 0.0, BATT_TRAY_Z])
        m0, c0, _ = totals(pos)[0]
        x_target = X_LE_MAC - CG_TARGET_EMPTY / 100.0 * MAC
        pos[0] = (x_target - c0[0]) * m0 / 2.00     # battery is 2 kg, moving it dx moves the cg by 2 dx / m
        e, f = totals(pos)
        log.append({'where': 'tray on top of the water tank', 'pos': pos.tolist(),
                    'cg_empty': pct_mac(e[1][0]), 'cg_full': pct_mac(f[1][0])})
    fit, b = battery_fit(pos)
    b.exportStep(os.path.join(PARTS_DIR, 'eq_battery.step'), unit='M')
    parts = []
    for part, link, m, desc in BUDGET + [WATER]:
        parts.append({'name': part, 'link': link, 'mass': m, 'desc': desc,
                      'step': os.path.relpath(os.path.join(PARTS_DIR, f'{part}.step'), ROOT).replace('\\', '/'),
                      'kind': 'water' if part == 'water_3L' else ('equipment' if part.startswith('eq_') else 'structure'),
                      'rgb': GEO['parts'].get(part, {}).get('rgb', [0.85, 0.35, 0.1])})
    out = {'parts': parts, 'battery': {'pos': pos.tolist(), 'dims': BATT_DIM, 'log': log, 'fit_m3': fit},
           'cad_estimate': {'empty': {'mass': e[0], 'cg': e[1].tolist(), 'cg_pct_mac': pct_mac(e[1][0])},
                            'full': {'mass': f[0], 'cg': f[1].tolist(), 'cg_pct_mac': pct_mac(f[1][0])}}}
    json.dump(out, open(os.path.join(OUT, 'plan.json'), 'w'), indent=1)
    for s in log:
        print(f"battery at {np.round(s['pos'], 4).tolist()} ({s['where']}): cg {s['cg_empty']:.1f} % mac empty, "
              f"{s['cg_full']:.1f} % mac full")
    print('battery fit (m^3, want all 0):', {k: f'{v:.1e}' for k, v in fit.items()})


def final():
    sw = json.load(open(SW_JSON))
    plan_ = json.load(open(os.path.join(OUT, 'plan.json')))
    pos = np.array(plan_['battery']['pos'])
    bodies_sw = {}
    bodies_cad = {}
    for part, link, m, _ in BUDGET + [WATER]:
        s = sw['parts'][part]
        assert abs(s['mass'] - m) < 1e-6 * max(m, 1), (part, s['mass'], m)
        bodies_sw[part] = (s['mass'], np.array(s['cg']), np.array(s['I_cg']))
        bodies_cad[part] = cad_body(part, m, pos)
    res = {'source': 'solidworks ' + sw.get('solidworks_revision', ''), 'mac': MAC, 'x_le_mac': X_LE_MAC,
           'battery': plan_['battery']}
    for case, water in (('full', True), ('empty', False)):
        links = links_from(bodies_sw, water)
        m, c, I = aircraft(links)
        mc, cc, Ic = aircraft(links_from(bodies_cad, water))
        asm = sw['assembly'][case]
        res[case] = {'mass': m, 'cg': c.tolist(), 'cg_pct_mac': pct_mac(c[0]), 'inertia_about_cg': I.tolist(),
                     'check_vs_sw_assembly': {'dmass': m - asm['mass'],
                                              'dcg_mm': ((c - np.array(asm['cg'])) * 1000).tolist(),
                                              'dI_max': float(np.abs(I - np.array(asm['I_cg'])).max())},
                     'cad_mesh_estimate': {'cg': cc.tolist(), 'cg_pct_mac': pct_mac(cc[0]), 'inertia_about_cg': Ic.tolist()},
                     'links': {}}
        for name, (lm, lc, lI) in links.items():
            com_l, I_l = to_link_frame(name, lm, lc, lI)
            res[case]['links'][name] = {'mass': lm, 'com': com_l.tolist(), 'com_base': lc.tolist(),
                                        'inertia': {'ixx': I_l[0, 0], 'ixy': I_l[0, 1], 'ixz': I_l[0, 2],
                                                    'iyy': I_l[1, 1], 'iyz': I_l[1, 2], 'izz': I_l[2, 2]},
                                        'principal': np.linalg.eigvalsh(I_l).tolist()}
    res['parts_sw_vs_cad'] = {}
    for part in bodies_sw:
        ms, cs, Is = bodies_sw[part]
        _, cc, Ic = bodies_cad[part]
        res['parts_sw_vs_cad'][part] = {'dcg_mm': float(np.linalg.norm(cs - cc) * 1000),
                                        'dI_rel': float(np.abs(Is - Ic).max() / max(np.abs(Is).max(), 1e-12))}
    in_band = all(CG_BAND[0] <= res[c]['cg_pct_mac'] <= CG_BAND[1] for c in ('full', 'empty'))
    res['cg_in_band'] = in_band
    json.dump(res, open(os.path.join(OUT, 'mass_props.json'), 'w'), indent=1)
    write_report(res, sw, plan_)
    if not in_band:
        print('cg outside the band with the solidworks numbers: rerun "plan" with CG_TARGET_EMPTY adjusted')


def write_report(res, sw, plan_):
    F, E = res['full'], res['empty']
    dx = (np.array(F['cg']) - np.array(E['cg'])) * 1000
    L = ['# Step 3: mass properties', '',
         f'Numbers measured by SolidWorks ({sw.get("solidworks_revision", "")}) on the parts in '
         '`cad/solidworks/`, each part at the density that gives its budget mass. Frame: REP-103, origin '
         'at the wing leading edge root, which is also the leading edge of the mean chord because the '
         'wing is rectangular. CG % MAC is taken along the body X axis: (x_LE - x_cg) / MAC.', '',
         '## Mass breakdown', '',
         '| item | SolidWorks part | URDF link | mass [kg] | density [kg/m^3] |', '|---|---|---|---:|---:|']
    for part, link, m, desc in BUDGET:
        L.append(f'| {desc} | {part} | {link} | {m:.3f} | {sw["parts"][part]["density"]:.0f} |')
    L.append(f'| **Dry total** | | | **{sum(r[2] for r in BUDGET):.3f}** | |')
    L.append(f'| {WATER[3]} | {WATER[0]} | {WATER[1]} | {WATER[2]:.3f} | {sw["parts"][WATER[0]]["density"]:.0f} |')
    L.append(f'| **Take-off total** | | | **{sum(r[2] for r in BUDGET) + M_WATER:.3f}** | |')
    L += ['', 'The density column is just budget mass over CAD volume, so it is an equivalent density. '
          'The hull, nacelles and floats are hollow skins (2.0, 1.5 and 1.5 mm), so their mass sits at the '
          'skin, as it does on the real thing. Wings and tail surfaces are solid (foam core). '
          'Equipment items are boxes or cylinders of their real size at their mounting point.', '',
          'Where the numbers come from: composite hull at about 1.3 kg/m^2 of skin plus frames, foam/glass '
          'wing at about 1.4 kg/m^2 of planform, 600-800 W outrunners with 14x7 props, a 355 Wh battery '
          '(roughly 40 min at a 500 W cruise), 35 g servos. The battery takes whatever is left of the '
          '7.5 kg dry budget.', '',
          '## CG check and battery position', '']
    for s in plan_['battery']['log']:
        L.append(f"- {s['where']}: battery centre at ({s['pos'][0]:.3f}, {s['pos'][1]:.3f}, {s['pos'][2]:.3f}) m "
                 f"-> CG {s['cg_empty']:.1f} % MAC empty, {s['cg_full']:.1f} % MAC full (CAD-mesh estimate)")
    if len(plan_['battery']['log']) > 1:
        a, b = plan_['battery']['log'][0]['pos'], plan_['battery']['log'][-1]['pos']
        L.append(f'- The first spot puts the CG ahead of the 25-30 % band, so the battery moved '
                 f'{(a[0] - b[0]) * 1000:.0f} mm aft and {(b[2] - a[2]) * 1000:.0f} mm up, onto a tray on top '
                 'of the water tank.')
    fit = plan_['battery']['fit_m3']
    L.append(f"- Fit check of the battery box: {fit['outside_cavity'] * 1e6:.2f} cm^3 outside the hull cavity, "
             f"{max(v for k, v in fit.items() if k != 'outside_cavity') * 1e6:.2f} cm^3 overlap with the tank "
             'and the other equipment.')
    L.append(f"- With the SolidWorks numbers: **{E['cg_pct_mac']:.1f} % MAC empty, {F['cg_pct_mac']:.1f} % MAC "
             f"full**, {'inside' if res['cg_in_band'] else 'OUTSIDE'} the 25-30 % band.")
    L += ['', '## Aircraft totals (SolidWorks)', '', '| | tank full | tank empty |', '|---|---:|---:|',
          f'| mass [kg] | {F["mass"]:.3f} | {E["mass"]:.3f} |',
          f'| CG x [m] | {F["cg"][0]:.4f} | {E["cg"][0]:.4f} |',
          f'| CG y [m] | {F["cg"][1]:.4f} | {E["cg"][1]:.4f} |',
          f'| CG z [m] | {F["cg"][2]:.4f} | {E["cg"][2]:.4f} |',
          f'| CG [% MAC] | {F["cg_pct_mac"]:.1f} | {E["cg_pct_mac"]:.1f} |']
    for i, j, n in ((0, 0, 'Ixx'), (1, 1, 'Iyy'), (2, 2, 'Izz'), (0, 2, 'Ixz'), (0, 1, 'Ixy'), (1, 2, 'Iyz')):
        L.append(f'| {n} about CG [kg m^2] | {F["inertia_about_cg"][i][j]:.4f} | {E["inertia_about_cg"][i][j]:.4f} |')
    L += ['', 'Products of inertia are in tensor form (Ixz = -integral of xz dm), which is what URDF and SDF '
          'expect. SolidWorks prints +integral of xz dm in its Mass Properties window, so the signs there are '
          'flipped.', '',
          f'Filling the tank moves the CG by {dx[0]:.1f} mm in x and {dx[2]:.1f} mm in z '
          f'({abs(F["cg_pct_mac"] - E["cg_pct_mac"]):.1f} % MAC): the water centroid sits on the wing quarter '
          'chord, next to the dry CG, and low in the hull.', '',
          '## Cross-checks', '',
          f'- Sum of the parts vs the SolidWorks assembly (Full / Empty configurations): mass differs by '
          f'{F["check_vs_sw_assembly"]["dmass"]:.1e} / {E["check_vs_sw_assembly"]["dmass"]:.1e} kg, CG by '
          f'{max(abs(v) for v in F["check_vs_sw_assembly"]["dcg_mm"]):.4f} / '
          f'{max(abs(v) for v in E["check_vs_sw_assembly"]["dcg_mm"]):.4f} mm, inertia by '
          f'{F["check_vs_sw_assembly"]["dI_max"]:.1e} / {E["check_vs_sw_assembly"]["dI_max"]:.1e} kg m^2.',
          f'- SolidWorks vs the CadQuery mesh estimate, whole aircraft: CG {F["cad_mesh_estimate"]["cg_pct_mac"]:.2f} '
          f'vs {F["cg_pct_mac"]:.2f} % MAC full.']
    worst = max(res['parts_sw_vs_cad'].items(), key=lambda kv: kv[1]['dI_rel'])
    worst_c = max(res['parts_sw_vs_cad'].items(), key=lambda kv: kv[1]['dcg_mm'])
    L.append(f'- Per part, the largest CoM difference is {worst_c[1]["dcg_mm"]:.2f} mm ({worst_c[0]}) and the '
             f'largest inertia difference {100 * worst[1]["dI_rel"]:.2f} % ({worst[0]}); the mesh is a little '
             'smaller than the exact surfaces.')
    L += ['', '## Per-link inertials (as written to the URDF)', '',
          'Inertia about each link CoM, in the link frame. Fixed links share the base_link frame; control '
          'surfaces sit on their hinge lines; the propeller frames are turned so their Z axis points forward.', '',
          '| link | parts | mass [kg] | CoM in link frame [m] | ixx | iyy | izz | ixy | ixz | iyz |',
          '|---|---|---:|---|---:|---:|---:|---:|---:|---:|']
    members = {}
    for part, link, m, _ in BUDGET + [WATER]:
        members.setdefault(link, []).append(part)
    for case in ('full', 'empty'):
        for name, d in res[case]['links'].items():
            if case == 'empty' and name != 'water_tank':
                continue
            I = d['inertia']
            lab = f'{name} ({case})' if name == 'water_tank' else name
            parts = ', '.join(p for p in members[name] if case == 'full' or p != 'water_3L')
            L.append(f'| {lab} | {parts} | {d["mass"]:.4f} | ({d["com"][0]:.4f}, {d["com"][1]:.4f}, {d["com"][2]:.4f}) | '
                     f'{I["ixx"]:.3e} | {I["iyy"]:.3e} | {I["izz"]:.3e} | {I["ixy"]:.2e} | {I["ixz"]:.2e} | {I["iyz"]:.2e} |')
    with open(os.path.join(OUT, 'report.md'), 'w', encoding='utf-8') as f:
        f.write('\n'.join(L) + '\n')
    print('\n'.join(L))


if __name__ == '__main__':
    stage = sys.argv[1] if len(sys.argv) > 1 else 'plan'
    {'plan': plan, 'final': final}[stage]()
