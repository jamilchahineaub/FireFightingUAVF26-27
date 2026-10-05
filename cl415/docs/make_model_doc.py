"""docs/aircraft_model_and_control.md from the project's data files, so every number is the one in use

reads  cad/geometry.json, mass/mass_props.json, flight_params/cl415_flight_data.json, flight_params/cl415_params.json,
       flight_params/matlab_check/check_results.json, px4_sitl/px4_model_summary.json, px4_sitl/airframes/*,
       px4_sitl/models/cl415_px4_water/model.sdf (+ maps), px4_sitl/flight_logs/*_summary.json
run from the project root:  .venv/Scripts/python docs/make_model_doc.py
"""
import glob
import json
import math
import os
import re
import sys
import xml.etree.ElementTree as ET

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'flight_params', 'propulsion'))
import prop_tools as PT  # noqa: E402

J = lambda *p: json.load(open(os.path.join(ROOT, *p)))
G, M, D, P, S = J('cad', 'geometry.json'), J('mass', 'mass_props.json'), J('flight_params', 'cl415_flight_data.json'), \
    J('flight_params', 'cl415_params.json'), J('px4_sitl', 'px4_model_summary.json')
CHK = J('flight_params', 'matlab_check', 'check_results.json')
RUN = J('px4_sitl', 'flight_logs', '2026-10-03_21_53_10_summary.json')
WAT = J('px4_sitl', 'flight_logs', '2026-10-03_22_59_02_summary.json')
geo, A = D['geometry'], D['aero']
F, E = A['about_full_cg'], A['about_empty_cg']
b, Sw, c = geo['b'], geo['S'], geo['c_bar']
mf, me = M['full']['mass'], M['empty']['mass']
cgf, cge = np.array(M['full']['cg']), np.array(M['empty']['cg'])
If, Ie = np.array(M['full']['inertia_about_cg']), np.array(M['empty']['inertia_about_cg'])
W = mf * 9.81
R2D = 180 / math.pi
V = 20.0
q = 0.5 * 1.225 * V * V

# ---------------------------------------------------------------- derived positions
x_ac = G['aero']['wing']['ac_left'][0]
pct = lambda x: -x / c * 100
xnp_f, xnp_e = -F['Xnp_over_c'] * c, -E['Xnp_over_c'] * c
sm_f, sm_e = -F['Cm_alpha'] / F['CL_alpha'] * 100, -E['Cm_alpha'] / E['CL_alpha'] * 100
ht, vt = G['aero']['htail'], G['aero']['vtail']
l_t = cgf[0] - ht['ac'][0]
l_v = cgf[0] - vt['ac'][0]
V_H = ht['area'] * l_t / (Sw * c)
V_V = vt['area'] * l_v / (Sw * b)
hub = np.array(G['aero']['props']['center_left'])
h_T_f, h_T_e = hub[2] - cgf[2], hub[2] - cge[2]
keel, step = G['misc']['keel_z'], G['misc']['step_x']
step_angle = math.degrees(math.atan2(cgf[0] - step, cgf[2] - keel))
T_static = S['propulsion']['static_thrust_total']
T_cruise = S['propulsion']['cruise_thrust_total']
qSc = q * Sw * c
Cm_T_cruise = -T_cruise * h_T_f / qSc
de_T = Cm_T_cruise / F['Cm_de'] * R2D

# composite CG of the gazebo model from its links (a check that the physics model carries the SolidWorks numbers)
sdf = ET.parse(os.path.join(ROOT, 'px4_sitl', 'models', 'cl415_px4_water', 'model.sdf')).getroot().find('model')
joint_pose = {}
for j in sdf.findall('joint'):
    pe = j.find('pose')
    if pe is not None and pe.get('relative_to') == 'base_link':
        joint_pose[j.get('name')] = np.array([float(v) for v in pe.text.split()[:3]])
msum, mx = 0.0, np.zeros(3)
link_rows = []
for ln in sdf.findall('link'):
    inert = ln.find('inertial')
    if inert is None:
        continue
    m = float(inert.find('mass').text)
    ip = inert.find('pose')
    ipos = np.array([float(v) for v in ip.text.split()[:3]]) if ip is not None else np.zeros(3)
    lp = ln.find('pose')
    base = np.zeros(3)
    if lp is not None and lp.get('relative_to') in joint_pose:
        base = joint_pose[lp.get('relative_to')] + np.array([float(v) for v in lp.text.split()[:3]])
    elif lp is not None and lp.get('relative_to') is None:
        base = np.array([float(v) for v in lp.text.split()[:3]])
    pos = base + ipos
    msum += m
    mx += m * pos
    if m > 0.02:
        link_rows.append((ln.get('name'), m, pos))
cg_sdf = mx / msum

# propulsion numbers from the maps actually used by the sim
maps_dir = os.path.join(ROOT, 'px4_sitl', 'models', 'cl415_px4_water', 'maps')
thr_map = np.loadtxt(os.path.join(maps_dir, 'thrust_N.csv'), delimiter=',', skiprows=1)
thr_cols = [float(x) for x in open(os.path.join(maps_dir, 'thrust_N.csv')).readline().strip().split(',')[1:]]
speeds = thr_map[:, 0]
Tm = thr_map[:, 1:] * 2
a_fit, b_fit, rms_fit = PT.fit_momentum(speeds, thr_cols, Tm)
jfull, j56 = thr_cols.index(1.0), min(range(len(thr_cols)), key=lambda i: abs(thr_cols[i] - 0.55))


def T_at(v, j):
    return float(np.interp(v, speeds, Tm[:, j]))


# airframe parameters
def params(name):
    out = {}
    for line in open(os.path.join(ROOT, 'px4_sitl', 'airframes', name)):
        m = re.match(r'param set-default (\S+) (\S+)', line)
        if m:
            out[m.group(1)] = m.group(2)
    return out


AF, AFW = params('4050_gz_cl415'), params('4051_gz_cl415_water')
servo = {}
for p in sdf.findall('plugin'):
    if p.get('name') == 'gz::sim::systems::JointPositionController':
        servo[p.find('joint_name').text] = {k: p.find(k).text for k in ('p_gain', 'i_gain', 'd_gain', 'cmd_max')}
hull = S['hull']
modes = {r['id']: {m['name']: m for m in r['modes']} for r in CHK}
cl_modes = modes['CL']


def f(x, n=3):
    return f'{x:.{n}g}' if isinstance(x, float) else str(x)


def row(*cells):
    return '| ' + ' | '.join(str(x) for x in cells) + ' |'


L = []
add = L.append
add('# CL-415 UAV: aircraft model and flight controller')
add('')
add('What the simulations and the autopilot use for this aircraft, with the values and where each one comes from. '
    'Generated by `docs/make_model_doc.py` from the project data files, so the numbers here are the ones in the '
    'models. Axes: the CAD, Gazebo and this document use x forward, y left, z up (REP-103) with the origin at the '
    'wing leading edge on the centreline; the team MATLAB file (`flight_params/cl415_params.yaml`) and PX4 use x '
    'forward, y right, z down (FRD), which flips the sign of y, z and of Ixz\'s definition as noted there. '
    'Chord positions are quoted in percent of the mean aerodynamic chord (MAC) from the leading edge.')
add('')
add('## 1. Geometry')
add('')
add(row('item', 'value', 'source'))
add(row('---', '---:', '---'))
for item, val, src in (
        ('wing span b', f'{b} m', 'specification'), ('wing area S', f'{Sw:.4f} m^2', 'CAD planform, incl. the part over the hull'),
        ('chord c = MAC', f'{c} m', 'specification, constant chord'), ('aspect ratio', f'{geo["AR"]:.2f}', ''),
        ('wing section / incidence', f'NACA 4417 / {G["aero"]["wing"]["incidence_deg"]} deg to the hull datum', 'specification / CAD'),
        ('wing aerodynamic centre', f'x = {x_ac:.4f} m ({pct(x_ac):.1f} % MAC)', 'quarter chord'),
        ('horizontal tail', f'S_h {ht["area"]:.4f} m^2, span {ht["span"]:.3f} m, chord {ht["chord"]:.3f} m, NACA 0016, AC at x = {ht["ac"][0]:.3f}, z = {ht["ac"][2]:.3f} m', 'CAD'),
        ('tail arm, tail volume', f'l_t = {l_t:.3f} m, V_H = {V_H:.3f}', 'from the full CG'),
        ('elevator', f'{ht["cf_c"]:.2f} chord ratio, {ht["elevator_area"]:.4f} m^2', 'CAD'),
        ('vertical tail', f'S_v {vt["area"]:.4f} m^2 (+ {geo["S_finlets"]:.4f} m^2 finlets), height {vt["height"]:.3f} m, MAC {vt["mac"]:.3f} m, NACA 0012', 'CAD'),
        ('fin arm, fin volume', f'l_v = {l_v:.3f} m, V_V = {V_V:.4f}', ''),
        ('rudder', f'{vt["cf_c_root"]:.2f} to {vt["cf_c_tip"]:.2f} chord ratio, {vt["rudder_area"]:.4f} m^2', 'CAD'),
        ('ailerons', f'y = {geo["aileron_y"][0]} to {geo["aileron_y"][1]} m, {geo["aileron_cf_c"]:.2f} chord ratio', 'CAD'),
        ('hull length, keel, step', f'{geo["L_fus"]:.3f} m, keel z = {keel:.3f} m, step at x = {step:.3f} m', 'CAD'),
        ('floats', f'y = +-{G["misc"]["float_y"]} m, bottom z = {G["misc"]["float_bottom_z"]:.3f} m', 'CAD'),
        ('propeller hubs', f'x = {hub[0]}, y = +-{hub[1]}, z = {hub[2]} m; {G["aero"]["props"]["diameter"]} m disc in the CAD, 13.5 in (0.343 m) in the sim', 'CAD / propulsion choice'),
):
    add(row(item, val, src))
add('')
add('## 2. Mass and inertia (SolidWorks)')
add('')
add('Measured on the SolidWorks parts with the real material densities and the equipment at its planned positions '
    '(`mass/report.md`). Inertia about each configuration\'s own CG. The tensor is in REP-103 axes; in the team\'s FRD '
    'convention Ixx, Iyy, Izz are unchanged and the Beard Ixz equals the tensor element I_xz listed here.')
add('')
add(row('', 'full tank (3 L)', 'empty'))
add(row('---', '---:', '---:'))
add(row('mass [kg]', f'{mf:.2f}', f'{me:.2f}'))
add(row('CG x [m], % MAC', f'{cgf[0]:.4f}, {M["full"]["cg_pct_mac"]:.1f} %', f'{cge[0]:.4f}, {M["empty"]["cg_pct_mac"]:.1f} %'))
add(row('CG z [m] (below the wing root chord)', f'{cgf[2]:.4f}', f'{cge[2]:.4f}'))
for k, (i, j) in (('Ixx', (0, 0)), ('Iyy', (1, 1)), ('Izz', (2, 2)), ('I_xz', (0, 2))):
    add(row(f'{k} [kg m^2]', f'{If[i, j]:.4f}', f'{Ie[i, j]:.4f}'))
add(row('Ixy, Iyz', 'below 1e-5', 'below 1e-5'))
add('')
add(f'The 3.0 kg of water sits almost on the CG: filling the tank moves the CG {abs(cgf[0]-cge[0])*1000:.1f} mm '
    f'{"forward" if cgf[0] > cge[0] else "aft"} and {abs(cgf[2]-cge[2])*1000:.0f} mm down, so the drop changes the trim '
    'very little; it changes the stall speed from {:.1f} to {:.1f} m/s.'.format(A['stall']['full'][2], A['stall']['empty'][2]))
add('')
add('## 3. Is the aircraft balanced? CG, centre of lift, neutral point, thrust line')
add('')
add('Longitudinal positions along the chord (full tank unless stated):')
add('')
add(row('point', 'x [m]', '% MAC', 'note'))
add(row('---', '---:', '---:', '---'))
add(row('wing aerodynamic centre (centre of lift of the wing alone)', f'{x_ac:.4f}', f'{pct(x_ac):.1f}', 'quarter chord; the wing\'s pitching moment about it is constant (Cm_ac = -0.105 from XFOIL)'))
add(row('centre of gravity, full', f'{cgf[0]:.4f}', f'{pct(cgf[0]):.1f}', 'SolidWorks'))
add(row('centre of gravity, empty', f'{cge[0]:.4f}', f'{pct(cge[0]):.1f}', 'SolidWorks'))
add(row('neutral point (centre of lift of the whole aircraft), full', f'{xnp_f:.4f}', f'{pct(xnp_f):.1f}', 'AVL, wing + tail + hull'))
add(row('neutral point, empty', f'{xnp_e:.4f}', f'{pct(xnp_e):.1f}', 'AVL'))
add(row('static margin, full / empty', '', f'{sm_f:.1f} / {sm_e:.1f}', '-Cm_alpha / CL_alpha, the same as NP minus CG'))
add('')
add(f'The CG is {(x_ac - cgf[0])*1000:.0f} mm behind the wing\'s own centre of lift, so the wing alone would be slightly '
    f'unstable, which is normal for a tailed aircraft; the tail (tail volume {V_H:.2f}) moves the aircraft\'s centre of '
    f'lift back to {pct(xnp_f):.0f} % MAC, {sm_f:.0f} % of the chord behind the CG. Positive static margin means a '
    'nose-up disturbance produces a nose-down moment: the aircraft is statically stable in both loading cases, and '
    'the team MATLAB check and the PX4 flights confirm it dynamically (short period damping ratio '
    f'{cl_modes["short period"]["zeta"]:.2f}, every mode Level 1 except the slow spiral). The margin is on the large '
    'side (10 to 20 % is the usual design range for a UAV of this kind): it costs a little elevator and trim drag '
    f'(the trim elevator runs from +{P["trim_reference"]["cruise_loaded"]["de_deg"]:.1f} deg at cruise to about -4 deg at '
    '1.1 Vs) and makes the aircraft steady rather than agile, which suits a water bomber. The CG could move aft to '
    f'about {pct(xnp_f) - 15:.0f} % MAC (x = {-(pct(xnp_f) - 15) / 100 * c:.3f} m) before the margin drops to 15 %; moving '
    'the battery is the lever for that, as in `mass/report.md`.')
add('')
add('Vertical arrangement and the moments it creates:')
add('')
add(row('item', 'value', 'effect'))
add(row('---', '---:', '---'))
add(row('wing chord plane above the CG', f'{-cgf[2]*1000:.0f} mm (full), {-cge[2]*1000:.0f} mm (empty)', 'lift and drag act above the CG: a small pendulum stability and a nose-up moment from drag'))
add(row('thrust line above the CG', f'{h_T_f*1000:.0f} mm (full), {h_T_e*1000:.0f} mm (empty)', f'power pitches the nose down: {T_static*h_T_f:.1f} Nm at full static thrust, {T_cruise*h_T_f:.2f} Nm (Cm {Cm_T_cruise:+.3f}, {abs(de_T):.1f} deg of elevator) at cruise'))
add(row('horizontal tail above the CG', f'{(ht["ac"][2]-cgf[2])*1000:.0f} mm', 'keeps the tail above the spray and the prop wash'))
add(row('keel below the CG', f'{(cgf[2]-keel)*1000:.0f} mm', 'water resistance acts low: full-power hull drag at the hump adds another nose-down moment on the water'))
add(row('main step aft of the CG', f'{(cgf[0]-step)*1000:.0f} mm, {step_angle:.0f} deg behind the vertical through the CG', 'flying-boat practice puts the step 10 to 15 deg aft of the CG so the hull trims bow-up on the step; this hull is at the aft end of that range and planed cleanly in the sim'))
add(row('floating attitude at rest', f'draft {WAT["water_takeoff"]["draft_at_rest_m"]*1000:.0f} mm, 3.4 deg bow up, 2.7 deg heel onto one float', 'the centre of buoyancy sits under the CG; the hull alone is unstable in roll, as on the real aircraft, and leans on a float'))
add('')
add('How these positions are carried into the simulators:')
add('')
add(f'- Gazebo: the aerodynamic forces act at `cp` = the full-tank CG ({cgf[0]:.4f}, 0, {cgf[2]:.4f}) with the AVL moment '
    'coefficients taken about that same point, which is equivalent to lift acting at the neutral point with the '
    'wing\'s own moment; the inertial frame of `base_link` plus the servo, propeller and camera links reproduce the '
    f'SolidWorks totals (composite mass {msum:.3f} kg, CG x {cg_sdf[0]:.4f} m, z {cg_sdf[2]:.4f} m against {mf:.3f} kg, '
    f'{cgf[0]:.4f}, {cgf[2]:.4f}); the IMU sits at the CG; the thrust is applied at each hub with its reaction torque.')
add('- Team MATLAB model: moments about the loaded CG with `r_thrust_from_loaded_cg_frd` for the thrust line, and '
    'the CG offset of the unloaded case through `mass_properties.cad`.')
add(f'- PX4 does not need the positions; it sees their effect as trim. In the sim the cruise elevator is '
    f'{abs(RUN["cruise"]["elevator_deg_TE_down"]):.1f} deg trailing edge up at {RUN["cruise"]["pitch_deg"]:.1f} deg pitch, '
    f'where the static balance without the thrust line predicts {P["trim_reference"]["cruise_loaded"]["de_deg"]:.1f} deg '
    f'trailing edge down; {abs(de_T):.1f} deg of the difference is the thrust-line moment, the remaining degree is the '
    'elevator\'s own lift and the fitted stall curve in the plugin. Both are within the first turn of a trim tab.')
add('')
add('## 4. Aerodynamic model')
add('')
add('AVL 3.52 vortex lattice on the CAD geometry (wing with NACA 4417 camber and the XFOIL lift slope, tail, hull and '
    'nacelles as slender bodies), XFOIL 6.99 for the section data, drag build-up by component. All derivatives per '
    'radian, body angle of attack measured from the hull datum (the wing sits at +2.65 deg), rates as p b/2V, q c/2V, '
    'r b/2V. Moments about the full-tank CG; the empty-CG set is in `flight_params/README.md`. The control derivatives '
    f'are AVL\'s times the XFOIL viscous flap factor ({A["control_viscous_factors"]["a"]:.2f} aileron, '
    f'{A["control_viscous_factors"]["e"]:.2f} elevator, {A["control_viscous_factors"]["r"]:.2f} rudder).')
add('')
add(row('longitudinal', 'value', 'lateral-directional', 'value', 'controls', 'value'))
add(row('---', '---:', '---', '---:', '---', '---:'))
lon = [('CL0', F['CL0']), ('CL_alpha', F['CL_alpha']), ('CL_q', F['CL_q']), ('Cm0', F['Cm0']), ('Cm_alpha', F['Cm_alpha']),
       ('Cm_q', F['Cm_q']), ('CD0', A['CD0']), ('K (CD = CD0 + K CL^2)', A['K']), ('Oswald e', A['oswald_e']), ('CL_max', A['CL_max'])]
lat = [('CY_beta', F['CY_beta']), ('Cl_beta', F['Cl_beta']), ('Cn_beta', F['Cn_beta']), ('CY_p', F['CY_p']), ('Cl_p', F['Cl_p']),
       ('Cn_p', F['Cn_p']), ('CY_r', F['CY_r']), ('Cl_r', F['Cl_r']), ('Cn_r', F['Cn_r']), ('', '')]
ctl = [('CL_de', F['CL_de']), ('Cm_de', F['Cm_de']), ('CD_de', A['CD_de']), ('Cl_da', F['Cl_da']), ('Cn_da', F['Cn_da']),
       ('CY_da', F['CY_da']), ('CY_dr', F['CY_dr']), ('Cn_dr', F['Cn_dr']), ('Cl_dr', F['Cl_dr']), ('', '')]
for (a1, v1), (a2, v2), (a3, v3) in zip(lon, lat, ctl):
    add(row(a1, f(v1, 4) if v1 != '' else '', a2, f(v2, 4) if v2 != '' else '', a3, f(v3, 4) if v3 != '' else ''))
add('')
add(f'Stall: CL_max {A["CL_max"]:.3f} at {A["stall"]["full"][1]:.1f} deg body alpha (critical section method: the first '
    f'wing strip reaches the XFOIL cl_max at the stall Reynolds number), stall speed {A["stall"]["full"][2]:.1f} m/s full, '
    f'{A["stall"]["empty"][2]:.1f} m/s empty, best L/D {D["propulsion"]["requirements"]["full"]["LD_max"]:.1f} at '
    f'{D["propulsion"]["requirements"]["full"]["V_md"]:.1f} m/s. Signs follow the team convention: elevator and ailerons '
    'positive trailing edge down (Cm_de < 0, Cl_da > 0 for left down / right up), rudder positive trailing edge left '
    '(Cn_dr < 0), sideslip positive from the right.')
add('')
add('Two implementations use these numbers. The team MATLAB model applies them directly with a sigmoid stall blend. '
    'Gazebo\'s `AdvancedLiftDrag` has a fixed-width blend, so for it the lift curve is refitted: CL0 '
    f'{S["ald"]["CL0"]:.3f}, CL_alpha {S["ald"]["CLa"]:.3f}, blend centre {S["ald"]["alpha_stall_deg"]:.1f} deg, which '
    f'matches AVL within {S["ald"]["fit_rms_-5_to_6deg"]:.3f} in CL from -5 to 6 deg and peaks at CL {S["ald"]["CL_peak"]:.3f} '
    f'at {S["ald"]["alpha_peak_deg"]:.1f} deg. Each control surface is a separate entry (per degree of joint angle), '
    'the ailerons split antisymmetrically with their own small pitching-moment contribution.')
add('')
add('Open-loop dynamics the controller sees (team MATLAB linearisation, cruise, full tank):')
add('')
add(row('mode', 'wn [rad/s]', 'zeta', 'time constant / time to double', 'MIL-F-8785C'))
add(row('---', '---:', '---:', '---:', '---'))
for nm in ('short period', 'phugoid', 'dutch roll', 'roll subsidence', 'spiral'):
    m = cl_modes[nm]
    tc = f'{m["tau"]:.2f} s' if nm == 'roll subsidence' else (f'doubles in {m["t_x"]:.0f} s' if nm == 'spiral' else '')
    z = m.get('zeta')
    add(row(nm, f'{m["wn"]:.2f}', f'{z:.2f}' if isinstance(z, (int, float)) and not math.isnan(z) else '', tc, m['verdict']))
add('')
add('## 5. Propulsion model')
add('')
add(f'Baseline hardware from the builders\' thread for this airframe: {S["propulsion"]["motor"]} (365 kV, 21 mOhm, 1.8 A '
    f'no-load) on 6S 16 Ah, {S["propulsion"]["prop"]} four-blade scale propeller, counter-rotating (left CW, right CCW '
    'seen from behind). The model is a DC-motor equation (Kv, Rm, I0, pack sag, ESC 0.97) in balance with APC\'s '
    'published Ct and Cp tables for the prop, scaled to the 13.5 in diameter the hull clearance allows '
    '(`flight_params/propulsion/prop_tools.py`). Both simulators read the same maps.')
add('')
add(row('', 'value'))
add(row('---', '---:'))
add(row('static thrust, both motors', f'{T_static:.1f} N (T/W {T_static/W:.2f})'))
add(row('full-throttle thrust at 5 / 10 / 15 / 20 m/s', ' / '.join(f'{T_at(v, jfull):.0f}' for v in (5, 10, 15, 20)) + ' N'))
add(row('thrust at 0.55 throttle, 10 / 15 / 20 m/s', ' / '.join(f'{T_at(v, j56):.1f}' for v in (10, 15, 20)) + ' N'))
add(row('cruise 20 m/s, full tank', f'{T_cruise:.2f} N at throttle {S["propulsion"]["cruise_throttle_full"]:.3f}'))
add(row('team momentum model T = a dT^2 - b dT V', f'a = {a_fit:.1f} N, b = {b_fit:.2f} N s/m (fit rms {rms_fit:.1f} N)'))
add(row('motor time constant', f'{S["propulsion"]["motor_time_constant_s"]:.3f} s (0.04 s up / 0.06 s down in Gazebo)'))
add(row('thrust line', f'x = +{hub[0]-cgf[0]:.3f} m ahead of and {h_T_f:.3f} m above the full CG, hubs {hub[1]} m from the centreline'))
add('')
add('In Gazebo `libCl415PropellerMap.so` turns PX4\'s 0 to 1000 motor command into throttle, looks up thrust, shaft '
    'torque and rpm against the axial airspeed, applies the thrust at the hub and the reaction torque to the airframe. '
    'The team model uses the momentum fit above (`propulsion.momentum`). When different hardware is chosen, '
    '`flight_params/propulsion/evaluate.py --prop <apc file> --motor|--kv ...` regenerates every number in this section.')
add('')
add('## 6. Actuators')
add('')
add(row('surface', 'travel', 'sign in the model', 'PX4 output', 'team model'))
add(row('---', '---:', '---', '---', '---'))
add(row('ailerons', f'+-{AF["SIM_GZ_SV_MINA1"]} deg', '+ trailing edge down, left/right separate joints', 'servo 1 left, servo 2 right, + = trailing edge up, mapped from +25 to -25 deg', 'delta_a + = left down right up'))
add(row('elevator', f'+-{AF["SIM_GZ_SV_MINA3"]} deg', '+ trailing edge down', 'servo 3, + = trailing edge up', 'delta_e + = trailing edge down'))
add(row('rudder', f'+-{AF["SIM_GZ_SV_MINA4"]} deg', '+ trailing edge left', 'servo 4, + = trailing edge right', 'delta_r + = trailing edge left'))
add(row('motors', '0 to 1', 'throttle', 'motor 1 left, motor 2 right, 1 to 1000', 'dT 0 to 1'))
add('')
add('In Gazebo each surface is a revolute joint with a position PID (the gains come from the surface inertia and a '
    '0.03 s target: elevator p {} i {} d {}, ailerons p {} i {} d {}), with the hinge torque limited to {} Nm. The '
    'team model uses first-order servos with tau 0.03 s and 500 deg/s rate limits, the same as `actuators` in the yaml.'
    .format(servo['elevator_joint']['p_gain'], servo['elevator_joint']['i_gain'], servo['elevator_joint']['d_gain'],
            servo['left_aileron_joint']['p_gain'], servo['left_aileron_joint']['i_gain'], servo['left_aileron_joint']['d_gain'],
            servo['elevator_joint']['cmd_max']))
add('')
add('## 7. Hull model (water)')
add('')
add('`libCl415FlyingBoatHull.so` on the water variant, acting on the collision boxes of the hull, the floats and the '
    'wings (see `px4_sitl/README.md` for why Gazebo\'s own buoyancy and hydrodynamics plugins were not usable). Per box: '
    'exact clipping against the water plane for buoyancy, planing lift and bluff drag on the wetted bottom and sides, '
    'skin friction, form drag per group on the largest wetted cross-section, and heave damping. The bow boxes are '
    'tilted to represent the keel rocker, the afterbody boxes to the sternpost angle, and the afterbody runs in the '
    'wake hollow of the step once the hull planes.')
add('')
add(row('element', 'size x y z [m]', 'bottom tilt', 'planing k', 'wake'))
add(row('---', '---', '---:', '---:', '---:'))
for el in hull['elements']:
    sz = ' x '.join(f'{float(v):.3f}' for v in el['size'].split())
    tilt = -math.degrees(float(el['pose'].split()[4]))
    add(row(el['name'], sz, f'{tilt:.0f} deg' if abs(tilt) > 0.1 else '0', f'{el["k_bottom"]:.2f}', f'{el["wake_depth"]:.2f} m' if el['wake_depth'] else ''))
add('')
add(f'Constants: water 998 kg/m^3, friction coefficient 0.004, side-force factor 0.6 fading out as the chines run dry '
    f'between 6 and 9 m/s, bluff coefficient 1.0, heave damping 1.0 of critical, form drag coefficients hull '
    f'{hull["form_cd"]["hull"]}, floats {hull["form_cd"]["left_float"]}, wings {hull["form_cd"]["left_wing"]}. Checks: it floats '
    f'at {WAT["water_takeoff"]["draft_at_rest_m"]*1000:.0f} mm draft with buoyancy equal to the weight (hydrostatics '
    'predicted 85 mm for the CAD hull); the open-loop resistance curve peaks at 0.36 of the weight at the 6 m/s hump '
    'and settles to 0.07 to 0.09 when planing; the autopilot run peaked at 0.25 at 5 m/s with 0.19 to 0.21 on the step.')
add('')
add('## 8. Flight controller (PX4 v1.16.2, fixed-wing stack)')
add('')
add('PX4\'s standard fixed-wing cascade is used unchanged; the airframe files '
    '`px4_sitl/airframes/4050_gz_cl415` (wheels) and `4051_gz_cl415_water` set its parameters from the model above.')
add('')
add('**Rate loops** (`fw_rate_control`, 250 Hz): for each axis the normalised torque command is '
    '`tau = FF * rate_sp + P * (rate_sp - rate) + I * integral(rate_sp - rate)`, with P and I scaled by (V_trim/V)^2 '
    'and FF by V_trim/V so the surfaces do the same work at every airspeed. The feed-forwards are the steady-state '
    'surface deflection per unit body rate from the AVL derivatives at the 20 m/s trim speed, normalised by the '
    'full travel:')
add('')
add('- roll: a steady roll rate p needs Cl_da * da = -Cl_p * p b/2V, so FF_roll = (-Cl_p b/2V) / (Cl_da * 25 deg) = '
    f'{S["px4"]["FW_RR_FF"]:.3f}')
add('- pitch: a steady pull-up at rate q needs elevator for the pitch damping and for the extra angle of attack of the '
    'load factor, FF_pitch = ((-Cm_q c/2V) - Cm_alpha m V/(q S CL_alpha)) / (-Cm_de * 25 deg) = '
    f'{S["px4"]["FW_PR_FF"]:.3f}')
add(f'- yaw: FF_yaw = (-Cn_r b/2V) / (-Cn_dr * 30 deg) = {S["px4"]["FW_YR_FF"]:.3f}')
add('')
add(row('parameter', 'roll', 'pitch', 'yaw', 'meaning'))
add(row('---', '---:', '---:', '---:', '---'))
add(row('FF', AF['FW_RR_FF'], AF['FW_PR_FF'], AF['FW_YR_FF'], 'feed-forward, from AVL'))
add(row('P', AF['FW_RR_P'], AF['FW_PR_P'], AF['FW_YR_P'], 'rate error gain, moderate starting values'))
add(row('I', AF['FW_RR_I'], AF['FW_PR_I'], AF['FW_YR_I'], 'integral, trims the surfaces (limit FW_PR_IMAX 0.4)'))
add('')
add('**Attitude loop** (`fw_att_control`): `rate_sp = (angle_sp - angle) / TC` with the time constant `FW_P_TC` '
    f'{AF.get("FW_P_TC", "0.4")} s on the runway variant and {AFW["FW_P_TC"]} s on the water variant, roll TC 0.4 s, plus '
    'the coordinated-turn yaw rate g tan(phi) cos(theta) / V. Limits: bank {} deg, pitch {} to +{} deg, pitch rate 60 deg/s.'
    .format(AF['FW_R_LIM'], AF['FW_P_LIM_MIN'], AF['FW_P_LIM_MAX']))
add('')
add('**Energy and path loops** (`fw_pos_control`): TECS trades altitude and airspeed with the throttle and the pitch '
    f'setpoint (trim speed {AF["FW_AIRSPD_TRIM"]} m/s, minimum {AF["FW_AIRSPD_MIN"]}, stall {AF["FW_AIRSPD_STALL"]}, '
    f'maximum {AF["FW_AIRSPD_MAX"]}, throttle trim {AF["FW_THR_TRIM"]}, climb up to {AF["FW_T_CLMB_MAX"]} m/s, sink '
    f'{AF["FW_T_SINK_MIN"]} to {AF["FW_T_SINK_MAX"]} m/s, pitch offset {AF["FW_PSP_OFF"]} deg); NPFG lateral guidance '
    f'(default 10 s period, 0.7 damping) turns the track error into a roll setpoint; waypoints are accepted at '
    f'{AF["NAV_ACC_RAD"]} m and loiters have {AF["NAV_LOITER_RAD"]} m radius.')
add('')
add('**Control allocation** (`CA_AIRFRAME 1`, standard plane): the torque and thrust setpoints are distributed by '
    'the effectiveness matrix: left and right ailerons -0.5 / +0.5 of the roll torque, elevator 1.0 pitch, rudder 1.0 '
    'yaw, both motors 0.5 of the thrust. On the water variant the real lateral motor positions (+-0.297 m) are given to '
    'the allocator so differential thrust helps the rudder in yaw, which is what steers the hull before the rudder '
    'bites. The vertical offset of the thrust line is deliberately left out of the allocator: with it the allocator '
    'commanded full nose-down elevator against its own estimate of the thrust moment at take-off power; the moment '
    'is in the physics and the pitch integrator trims it.')
add('')
add('**Take-off and landing logic**:')
add('')
add(row('', 'runway', 'water', 'why'))
add(row('---', '---', '---', '---'))
add(row('method', 'runway take-off', 'runway take-off', 'heading hold with rudder (and differential thrust on the water), pitch held, rotation at FW_TKO_AIRSPD'))
add(row('taxi pitch RWTO_PSP', f'{AF["RWTO_PSP"]} deg', f'{AFW["RWTO_PSP"]} deg', 'the hull needs the nose held up to climb onto the step'))
add(row('pitch trims', 'none', f'TRIM_PITCH {AFW["TRIM_PITCH"]}, FW_DTRIM_P_VMIN {AFW["FW_DTRIM_P_VMIN"]}', 'the runway logic resets the integrators until lift-off, so on the water the loop needs the back pressure a pilot applies'))
add(row('rotation / climb-out', f'{AF["FW_TKO_AIRSPD"]} m/s, min pitch {AF["FW_TKO_PITCH_MIN"]} deg', f'{AFW["FW_TKO_AIRSPD"]} m/s, min pitch {AFW["FW_TKO_PITCH_MIN"]} deg', 'wheels: stern clearance 10 deg'))
add(row('approach', f'{AF["FW_LND_ANG"]} deg at {AF["FW_LND_AIRSPD"]} m/s', f'{AFW["FW_LND_ANG"]} deg at {AFW["FW_LND_AIRSPD"]} m/s', 'glassy-water approach is shallow'))
add(row('flare', f'from {AF["FW_LND_FLALT"]} m, {AF["FW_LND_FL_PMIN"]} to {AF["FW_LND_FL_PMAX"]} deg, sink {AF["FW_LND_FL_SINK"]} m/s', f'from {AFW["FW_LND_FLALT"]} m, {AFW["FW_LND_FL_PMIN"]} to {AFW["FW_LND_FL_PMAX"]} deg, sink {AFW["FW_LND_FL_SINK"]} m/s', 'touch on the step, nose up'))
add(row('terrain', 'FW_LND_USETER 0', 'FW_LND_USETER 0', 'no rangefinder yet, flare on GNSS/baro height'))
add('')
add('**Estimation**: EKF2 with the Gazebo IMU at the CG, barometer, GNSS, the pitot and the magnetometer plugin; '
    f'over a full circuit the attitude error against the simulator truth is {RUN["ekf_attitude_error_deg"]["roll"]["rms"]:.1f} '
    f'deg roll, {RUN["ekf_attitude_error_deg"]["pitch"]["rms"]:.1f} deg pitch, {RUN["ekf_attitude_error_deg"]["yaw"]["rms"]:.1f} deg yaw rms.')
add('')
add('## 9. What the closed loops did')
add('')
add(row('', 'runway flight', 'water flight'))
add(row('---', '---', '---'))
add(row('take-off run', f'{RUN["takeoff"]["ground_roll_m"]:.0f} m, {RUN["takeoff"]["ground_roll_time_s"]:.1f} s, lift-off {RUN["takeoff"]["liftoff_airspeed"]:.1f} m/s',
        f'{WAT["takeoff"]["ground_roll_m"]:.0f} m, {WAT["takeoff"]["ground_roll_time_s"]:.1f} s, lift-off {WAT["takeoff"]["liftoff_airspeed"]:.1f} m/s'))
add(row('cruise throttle at 20 m/s (predicted 0.556)', f'{RUN["cruise"]["throttle"]:.3f}', f'{WAT["cruise"]["throttle"]:.3f}'))
add(row('cruise pitch / elevator', f'{RUN["cruise"]["pitch_deg"]:.1f} deg / {abs(RUN["cruise"]["elevator_deg_TE_down"]):.1f} deg TE up', f'{WAT["cruise"]["pitch_deg"]:.1f} deg / {abs(WAT["cruise"]["elevator_deg_TE_down"]):.1f} deg TE up'))
add(row('altitude tracking rms', f'{RUN["tracking"]["alt_err_ref_rms_m"]:.1f} m', f'{WAT["tracking"]["alt_err_ref_rms_m"]:.1f} m'))
add(row('airspeed tracking rms', f'{RUN["tracking"]["airspeed_err_rms"]:.2f} m/s', f'{WAT["tracking"]["airspeed_err_rms"]:.2f} m/s'))
add(row('touchdown', f'{RUN["touchdown"]["vz_mps"]:.2f} m/s sink at {RUN["touchdown"]["airspeed"]:.1f} m/s, {RUN["touchdown"]["pitch_deg"]:.1f} deg nose up',
        f'{WAT["water_landing"]["touch_vz"]*-1:.2f} m/s sink at {WAT["water_landing"]["touch_speed"]:.1f} m/s, stopped in {WAT["water_landing"]["stop_time_s"]:.1f} s'))
add(row('max bank (limit 40 deg)', f'{RUN["max_roll_deg"]:.0f} deg', f'{WAT["max_roll_deg"]:.0f} deg'))
add('')
add('Signs and authority of all four surfaces are confirmed by these loops: a wrong sign diverges, and the '
    'feed-forwards alone put the rate loops close to their setpoints. Open items are the roll overshoot in turns, '
    'the abrupt rotation off the water (22 deg pitch with the 12 deg taxi target and the trims carrying into the first '
    'second of flight) and the flare sink rate on the water, which comes out above the 0.4 m/s asked for.')
add('')
add('## 10. Files')
add('')
add('- `cad/geometry.json`, `mass/mass_props.json`, `mass/report.md`: geometry and SolidWorks mass properties')
add('- `flight_params/`: XFOIL and AVL inputs and outputs, the drag build-up, the team-schema parameter file, the MATLAB check')
add('- `px4_sitl/make_px4_model.py`: builds the Gazebo models, worlds and PX4 airframes from the above')
add('- `px4_sitl/px4_model_summary.json`: the fitted stall curve, the control entries, the propulsion maps, the hull elements')
add('- `px4_sitl/flight_logs/`: the flights quoted here (ulog, summary json, plots, hull state)')
out = os.path.join(ROOT, 'docs', 'aircraft_model_and_control.md')
open(out, 'w', encoding='utf-8').write('\n'.join(L) + '\n')
print('wrote', out)
print(f'composite sdf mass {msum:.3f} kg cg {cg_sdf.round(4)}  vs solidworks {mf:.3f} {cgf.round(4)}')
print(f'NP full {pct(xnp_f):.1f} % empty {pct(xnp_e):.1f} %  SM {sm_f:.1f}/{sm_e:.1f}  step angle {step_angle:.1f} deg  V_H {V_H:.3f} V_V {V_V:.4f}')
print(f'momentum fit a {a_fit:.1f} b {b_fit:.3f} rms {rms_fit:.2f}; thrust full throttle 5/10/15/20: '
      + '/'.join(f'{T_at(v, jfull):.0f}' for v in (5, 10, 15, 20)))
