"""every number the flight sims need for the cl415, in one place

inputs:  cad/geometry.json            geometry from the cad
         mass/mass_props.json         solidworks mass properties (full / empty)
         flight_params/airfoils/summary.json   XFOIL section data   (run_xfoil.py)
         flight_params/avl/results.json        AVL derivatives       (avl/build_avl.py)
         flight_params/propulsion/apc/*.dat    APC prop performance files
outputs: flight_params/cl415_params.yaml (+ .json)   drop-in for the team's params.yaml schema v1 (FRD axes)
         flight_params/cl415_flight_data.json         everything, incl. both CG references and all candidates
         flight_params/propulsion/maps/*.csv          thrust / current / power maps per motor-prop candidate
         flight_params/report.md, flight_params/plots/*.png

run from the project root:  .venv/Scripts/python flight_params/build_flight_params.py
"""
import copy
import hashlib
import json
import math
import os
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(HERE, 'propulsion'))
import prop_tools as PT  # noqa: E402

GEO = json.load(open(os.path.join(ROOT, 'cad', 'geometry.json')))
MASS = json.load(open(os.path.join(ROOT, 'mass', 'mass_props.json')))
XF = json.load(open(os.path.join(HERE, 'airfoils', 'summary.json')))
AV = json.load(open(os.path.join(HERE, 'avl', 'results.json')))
A = GEO['aero']
PLOTS = os.path.join(HERE, 'plots')
MAPS = os.path.join(HERE, 'propulsion', 'maps')
for d in (PLOTS, MAPS):
    os.makedirs(d, exist_ok=True)

G, RHO0, NU = 9.81, 1.225, 1.46e-5
D2R = math.pi / 180
S, B, C = A['wing']['area_total'], A['wing']['span'], A['wing']['chord']
AR = B * B / S
L_FUS = GEO['misc']['nose_x'] - GEO['misc']['stern_x']
V_CRUISE = 20.0
H_TRIM = 100.0
XFOIL_CLMAX_KNOCKDOWN = 0.92     # XFOIL is optimistic on cl_max at these Reynolds numbers
ROUGHNESS = 1.10                 # seams, hinge gaps, servo horns on the XFOIL section drag
HUMP_R_OVER_W = 0.20             # water resistance at the hump, typical stepped flying-boat hull
CLIMB_RATE = 2.5                 # m/s design climb at 1.3 Vs


def isa_rho(h):
    T0, Lr, g0, R = 288.15, 0.0065, 9.80665, 287.058
    return RHO0 * ((T0 - Lr * h) / T0) ** (g0 / (R * Lr) - 1)


def flu_to_frd(v):
    return [v[0], -v[1], -v[2]]


# ---------------------------------------------------------------- airfoils at the cruise Reynolds number
def sec(name, re):
    keys = sorted(XF['sections'][name]['reynolds'], key=float)
    vals = [float(k) for k in keys]
    k = min(keys, key=lambda kk: abs(float(kk) - re))
    out = dict(XF['sections'][name]['reynolds'][k])
    # linear interpolation of the scalar entries between the two nearest Re
    i = int(np.searchsorted(vals, re))
    if 0 < i < len(vals):
        w = (re - vals[i - 1]) / (vals[i] - vals[i - 1])
        lo, hi = XF['sections'][name]['reynolds'][keys[i - 1]], XF['sections'][name]['reynolds'][keys[i]]
        for kk, v in lo.items():
            if isinstance(v, (int, float)):
                out[kk] = (1 - w) * v + w * hi[kk]
    return out


def wing_profile_cd(cl, re):
    """XFOIL 4417 profile drag at section cl and Re (from the stored polar csv)"""
    keys = sorted(XF['sections']['wing_4417']['reynolds'], key=float)
    k = min(keys, key=lambda kk: abs(float(kk) - re))
    a = np.loadtxt(os.path.join(HERE, 'airfoils', 'polars', f'wing_4417_Re{float(k) / 1e3:.0f}k.csv'),
                   delimiter=',', skiprows=1)
    a = a[a[:, 0] < XF['sections']['wing_4417']['reynolds'][k]['alpha_cl_max_deg']]
    o = np.argsort(a[:, 1])
    return float(np.interp(cl, a[o, 1], a[o, 2]))


# ---------------------------------------------------------------- aero
Re_cr = V_CRUISE * C / NU
a0f, a0e = AV['runs']['full_a0'], AV['runs']['empty_a0']
nob_f = AV['runs']['full_a0_nobody']


def derivs(r):
    return {
        'CL0': r['CLtot'], 'CL_alpha': r['CLa'], 'CL_q': r['CLq'], 'CL_de': r['CLd02'] / D2R,
        'Cm0': r['Cmtot'], 'Cm_alpha': r['Cma'], 'Cm_q': r['Cmq'], 'Cm_de': r['Cmd02'] / D2R,
        'CY_beta': r['CYb'], 'Cl_beta': r['Clb'], 'Cn_beta': r['Cnb'],
        'CY_p': r['CYp'], 'Cl_p': r['Clp'], 'Cn_p': r['Cnp'],
        'CY_r': r['CYr'], 'Cl_r': r['Clr'], 'Cn_r': r['Cnr'],
        'CY_da': r['CYd01'] / D2R, 'Cl_da': r['Cld01'] / D2R, 'Cn_da': r['Cnd01'] / D2R,
        'CY_dr': r['CYd03'] / D2R, 'Cl_dr': r['Cld03'] / D2R, 'Cn_dr': r['Cnd03'] / D2R,
        'CL_beta': r['CLb'], 'Cm_beta': r['Cmb'], 'CD_q': r['CDq'], 'Cm_p': r['Cmp'], 'Cm_r': r['Cmr'],
        'Xnp_over_c': r['Neutral point  Xnp'] / C if 'Neutral point  Xnp' in r else None,
        'spiral_param': r.get('Clb Cnr / Clr Cnb'), 'e_trefftz': r['e'],
    }


D_full, D_empty, D_nobody = derivs(a0f), derivs(a0e), derivs(nob_f)

# AVL control derivatives are inviscid (thin-airfoil flap effectiveness). scale by XFOIL's viscous / thin ratio,
# and the elevator also by the share of the elevator strip the cad really covers (avl has no rudder cut-outs)
def visc_ratio(section):
    f = XF['sections'][section]['flap']
    return f['tau_xfoil'] / f['tau_thin_airfoil']


H = A['htail']
r_e = 5 * 0.16 * (0.2969 * math.sqrt(1 - H['cf_c']) - 0.126 * (1 - H['cf_c']) - 0.3516 * (1 - H['cf_c']) ** 2
                  + 0.2843 * (1 - H['cf_c']) ** 3 - 0.1015 * (1 - H['cf_c']) ** 4) * H['chord']
cov_cad = (H['elevator_area'] - r_e * 2 * (H['span'] / 2 - H['elevator_inner_y'])) / (H['cf_c'] * H['chord'] * H['span'])
cov_avl = (H['span'] / 2 - H['elevator_inner_y']) / (H['span'] / 2)
CTRL_FACTOR = {'a': visc_ratio('wing_4417'), 'e': visc_ratio('htail_0016') * cov_cad / cov_avl, 'r': visc_ratio('fin_0012')}
for D_ in (D_full, D_empty, D_nobody):
    for k in list(D_):
        if k.endswith('_da'):
            D_[k] *= CTRL_FACTOR['a']
        elif k.endswith('_de'):
            D_[k] *= CTRL_FACTOR['e']
        elif k.endswith('_dr'):
            D_[k] *= CTRL_FACTOR['r']

# drag build-up: AVL Trefftz induced drag (incl. tail) + XFOIL profile drag on the surfaces + bodies
fl = XF['sections']
S_h, S_v, S_fl = A['htail']['area'], A['vtail']['area'], 2 * A['finlets']['area_each']
cd_tail = (sec('htail_0016', V_CRUISE * A['htail']['chord'] / NU)['cd_min'] * S_h
           + sec('fin_0012', V_CRUISE * A['vtail']['mac'] / NU)['cd_min'] * (S_v + S_fl)) * ROUGHNESS / S


def cf_turb(re):
    return 0.455 / math.log10(re) ** 2.58


d_hull = 2 * GEO['misc'].get('half_beam', 0.1155)
f_hull = L_FUS / d_hull
bodies = [
    ('hull (with step and spray rails, Q = 1.25)', cf_turb(V_CRUISE * L_FUS / NU), 1 + 60 / f_hull ** 3 + f_hull / 400,
     1.25, 0.5 * GEO['parts']['fuselage']['area']),
    ('nacelles (Q = 1.3)', cf_turb(V_CRUISE * 0.43 / NU), 1 + 0.35 / (0.43 / 0.085), 1.3, GEO['parts']['left_nacelle']['area']),
    ('floats and pylons (Q = 1.2)', cf_turb(V_CRUISE * 0.26 / NU), 1.2, 1.2, GEO['parts']['left_float']['area']),
]
cd_bodies = sum(cf * ff * q * sw for _, cf, ff, q, sw in bodies) / S
MISC = 0.05
polar = []
for s in AV['sweep']:
    CL, CDi = s['CLtot'], s['CDff']
    if CL < 0.1 or CL > 1.25:
        continue
    cdp_w = wing_profile_cd(CL, Re_cr) * ROUGHNESS
    CD = (CDi + cdp_w + cd_tail + cd_bodies) * (1 + MISC)
    polar.append((s['alpha_deg'], CL, CD, CDi, cdp_w))
polar = np.array(polar)
# the team model is CD = CD0 + K CL^2 (no linear term): fit that form directly
Kp, CD0p = np.linalg.lstsq(np.c_[polar[:, 1] ** 2, np.ones(len(polar))], polar[:, 2], rcond=None)[0]
e_osw = 1 / (math.pi * AR * Kp)

# stall: critical section, the first wing strip to reach the section cl_max (XFOIL at the stall Re)
CLa_sweep = np.array([(s['alpha_deg'], s['CLtot'], s['strip_cl_max']) for s in AV['sweep']])


def clmax_3d(mass):
    V = 13.0
    for _ in range(20):
        re = V * C / NU
        clmax_sec = sec('wing_4417', re)['cl_max'] * XFOIL_CLMAX_KNOCKDOWN
        al = float(np.interp(clmax_sec, CLa_sweep[:, 2], CLa_sweep[:, 0]))
        CLm = float(np.interp(al, CLa_sweep[:, 0], CLa_sweep[:, 1]))
        V = math.sqrt(2 * mass * G / (RHO0 * S * CLm))
    return CLm, al, V, clmax_sec, re


CLmax_f, al_st_f, Vs_f, clsec_f, re_st_f = clmax_3d(MASS['full']['mass'])
CLmax_e, al_st_e, Vs_e, clsec_e, re_st_e = clmax_3d(MASS['empty']['mass'])
CL_MAX = min(CLmax_f, CLmax_e)

# ---------------------------------------------------------------- mass in the team's FRD form
m_f, m_e = MASS['full']['mass'], MASS['empty']['mass']
cg_f, cg_e = np.array(MASS['full']['cg']), np.array(MASS['empty']['cg'])


def jfrd(case):
    """team convention: J = [Ixx 0 -Ixz; 0 Iyy 0; -Ixz 0 Izz] in FRD, Ixz = +int(x z_frd dm).
    from the FLU tensor element I_xz = -int(x z_flu dm) = +int(x z_frd dm), so Ixz is the same number"""
    I = np.array(MASS[case]['inertia_about_cg'])
    return {'Ixx': I[0, 0], 'Iyy': I[1, 1], 'Izz': I[2, 2], 'Ixz': I[0, 2], 'Ixy_flu_tensor': I[0, 1],
            'Iyz_flu_tensor': I[1, 2]}


J_f, J_e = jfrd('full'), jfrd('empty')
cg_full_rel_frd = flu_to_frd(list(cg_f - cg_e))
water_c = np.array(GEO['parts']['water_3L']['centroid'])
water_rel_frd = flu_to_frd(list(water_c - cg_e))
prop_c = np.array(A['props']['center_left'])
r_thrust_frd = flu_to_frd([prop_c[0] - cg_f[0], 0.0, prop_c[2] - cg_f[2]])

# ---------------------------------------------------------------- propulsion requirements
rho_t = isa_rho(H_TRIM)


def drag(V, m, rho=RHO0):
    q = 0.5 * rho * V * V
    CL = m * G / (q * S)
    return q * S * (CD0p + Kp * CL * CL), CL


req = {}
for case, m in (('full', m_f), ('empty', m_e)):
    Vs = Vs_f if case == 'full' else Vs_e
    Vg = np.linspace(1.1 * Vs, 30, 200)
    Dg = np.array([drag(v, m)[0] for v in Vg])
    i_minD, i_minP = int(np.argmin(Dg)), int(np.argmin(Dg * Vg))
    V13 = 1.3 * Vs
    D13 = drag(V13, m)[0]
    req[case] = {'V_stall': Vs, 'V_md': float(Vg[i_minD]), 'D_min': float(Dg[i_minD]), 'V_mp': float(Vg[i_minP]),
                 'P_min_prop_W': float(Dg[i_minP] * Vg[i_minP]), 'LD_max': m * G / float(Dg[i_minD]),
                 'T_cruise20': drag(V_CRUISE, m)[0], 'CL_cruise20': drag(V_CRUISE, m)[1],
                 'T_climb': D13 + m * G * CLIMB_RATE / V13, 'V_climb': V13,
                 'T_hump': HUMP_R_OVER_W * m * G + drag(0.5 * 1.15 * Vs, m)[0] * 0.3,
                 'grid_V': Vg.tolist(), 'grid_D': Dg.tolist()}
T_req_static = max(req['full']['T_hump'], req['full']['T_climb']) * 1.15     # 15 % margin

# ---------------------------------------------------------------- motor / prop candidates
props = sorted(f for f in os.listdir(os.path.join(HERE, 'propulsion', 'apc')) if f.endswith('.dat'))
KVS = [380, 450, 530, 620, 700]
bat = PT.Battery(cells=6, capacity_ah=16.0)
speeds = np.arange(0.0, 31.0, 1.0)
throttles = np.round(np.arange(0.0, 1.0001, 0.05), 2)
# prop tip clearance to the hull side: hull corner sits 0.198 m from the prop axis (cad)
HULL_CLEAR_R = 0.1978
cands = []


def evaluate(pr, mot, preferred=False):
    """one motor + prop on the 6S pack against the take-off, climb, cruise, current, rpm and clearance checks"""
    if True:
        kv = mot.kv
        st = PT.operating_point(pr, mot, bat, 1.0, 0.0)
        if st is None:
            return None
        hump = PT.operating_point(pr, mot, bat, 1.0, 0.5 * 1.15 * Vs_f)
        # cruise throttle at 20 m/s, full tank
        Tneed = req['full']['T_cruise20']
        lo, hi = 0.0, 1.0
        cr = None
        if PT.operating_point(pr, mot, bat, 1.0, V_CRUISE)['thrust_total'] >= Tneed:
            for _ in range(40):
                mid = 0.5 * (lo + hi)
                o = PT.operating_point(pr, mot, bat, mid, V_CRUISE)
                if o is None or o['thrust_total'] < Tneed:
                    lo = mid
                else:
                    hi, cr = mid, o
            cr = dict(cr)
            cr['throttle'] = hi
        # max level speed, full tank
        vmax = None
        for v in np.arange(15.0, 45.0, 0.25):
            o = PT.operating_point(pr, mot, bat, 1.0, v)
            if o is None or o['thrust_total'] < drag(v, m_f)[0]:
                vmax = float(v)
                break
        c = {'prop': pr.name, 'D_m': pr.D, 'pitch_in': pr.pitch_in, 'kv': kv, 'rm': mot.rm, 'i0': mot.i0,
             'motor': mot.name, 'preferred': preferred, '_prop': pr, '_mot': mot,
             'static_thrust_total': st['thrust_total'], 'static_rpm': st['rpm'], 'static_i_motor': st['i_motor'],
             'static_p_batt': st['p_batt'], 'hump_thrust_total': hump['thrust_total'] if hump else 0.0,
             'cruise': cr, 'v_max': vmax, 'rpm_limit': pr.rpm_limit,
             'tip_clearance_m': HULL_CLEAR_R - pr.D / 2}
        c['ok_takeoff'] = c['hump_thrust_total'] >= req['full']['T_hump'] * 1.15
        c['ok_current'] = st['i_motor'] <= 55.0 and st['p_batt'] / 2 <= mot.p_max * 1.15
        c['ok_rpm'] = st['rpm'] <= pr.rpm_limit
        c['ok_clearance'] = c['tip_clearance_m'] >= 0.015
        c['ok_cruise'] = cr is not None and cr['throttle'] <= 0.85
        c['feasible'] = all(c[k] for k in ('ok_takeoff', 'ok_current', 'ok_rpm', 'ok_clearance', 'ok_cruise'))
        if cr is not None:
            usable_wh = bat.energy_wh * bat.usable
            c['endurance_min'] = usable_wh / cr['p_batt'] * 60
            c['cruise_p_batt'] = cr['p_batt']
        return c


# the generic sweep: every APC file on hand at five motor kv values
for pf in props:
    pr = PT.Prop(os.path.join(HERE, 'propulsion', 'apc', pf))
    for kv in KVS:
        c = evaluate(pr, PT.generic_motor(kv))
        if c:
            cands.append(c)
# the set the builders of this airframe fly (rcgroups CL-415 thread): hacker A50-16S on 6S with a four-blade
# scale prop, the APC 15.5x12-4 cut down to the 13.5 in the hull clearance allows. the gazebo model uses it
pref = evaluate(PT.Prop(os.path.join(HERE, 'propulsion', 'apc', 'PER3_155x12-4.dat')).scaled(13.5),
                copy.copy(PT.MOTORS['hacker_a50_16s']), preferred=True)
if pref:
    cands.append(pref)
feas = [c for c in cands if c['feasible']]
feas.sort(key=lambda c: c['cruise_p_batt'])
base = pref if (pref and pref['feasible']) else (feas[0] if feas else min(cands, key=lambda c: c.get('cruise_p_batt', 1e9)))
base_prop, base_mot = base['_prop'], base['_mot']

# maps and model fits for every feasible candidate (and the baseline)
fits = {}
for c in feas + ([base] if base not in feas else []):
    pr, mot = c['_prop'], c['_mot']
    T, I, P, R = PT.thrust_map(pr, mot, bat, speeds, throttles)
    tag = (f"{pr.d_in:g}x{pr.pitch_in:.3g}-{pr.blades}_hacker_a50_16s_6S" if c['preferred']
           else f"{c['prop']}_{c['kv']}kv_6S")
    hdr = 'V_mps\\throttle,' + ','.join(f'{d:.2f}' for d in throttles)
    for arr, nm in ((T, 'thrust_N'), (I, 'battery_current_A'), (P, 'battery_power_W'), (R, 'rpm')):
        np.savetxt(os.path.join(MAPS, f'{tag}_{nm}.csv'), np.c_[speeds, arr], delimiter=',', fmt='%.3f',
                   header=hdr, comments='')
    a, b, rms = PT.fit_momentum(speeds, throttles, T)
    st = PT.operating_point(pr, mot, bat, 1.0, 0.0)
    ct0, cp0 = pr.coeffs(0.0, st['rpm'])
    w_max = st['rpm'] * math.pi / 30
    # mechanical time constant of motor + prop: J R / Kt^2 (prop inertia from the cad, rotor estimate)
    J_rot = MASS['full']['links']['left_propeller']['inertia']['izz'] * (pr.D / A['props']['diameter']) ** 2 + 1.5e-4
    tau = J_rot * mot.rm / mot.kt ** 2
    fits[tag] = {'momentum_a_T_static_total': a, 'momentum_b': b, 'momentum_fit_rms_N': rms,
                 'gz_motorConstant': st['thrust'] / w_max ** 2, 'gz_momentConstant': cp0 / (2 * math.pi) * pr.D / ct0,
                 'gz_maxRotVelocity': w_max, 'motor_time_constant_s': tau}
    c['tag'] = tag
base_tag = base['tag']
bf = fits[base_tag]

# ---------------------------------------------------------------- trim reference (same balance as static_trim.m)
def static_trim(mass_case, V, gamma=0.0):
    m = m_f if mass_case == 'loaded' else m_e
    D = D_full
    qS = 0.5 * rho_t * V * V * S
    CL = m * G * math.cos(gamma) / qS
    Am = np.array([[D['CL_alpha'], D['CL_de']], [D['Cm_alpha'], D['Cm_de']]])
    al, de = np.linalg.solve(Am, [CL - D['CL0'], -D['Cm0']])
    CD = CD0p + Kp * CL * CL + CD_DE * abs(de)
    T = qS * CD + m * G * math.sin(gamma)
    a, b = bf['momentum_a_T_static_total'], bf['momentum_b']
    # a dT^2 - b V dT - T = 0
    dT = (b * V + math.sqrt((b * V) ** 2 + 4 * a * T)) / (2 * a)
    return {'V': V, 'mass': mass_case, 'gamma_deg': math.degrees(gamma), 'CL': CL, 'alpha_deg': math.degrees(al),
            'de_deg': math.degrees(de), 'thrust_N': T, 'dT': dT}


cd10 = [r['dcd'] for r in XF['sections']['htail_0016']['flap']['table'] if r['delta_deg'] == 10.0][0]
CD_DE = cd10 * (S_h * 0.854 / S) / (10 * D2R)
tr_l = static_trim('loaded', V_CRUISE)
tr_u = static_trim('unloaded', V_CRUISE)

# ---------------------------------------------------------------- yaml in the team's schema
src_avl = 'avl'
dp = []
for case, mc, Vs in (('loaded', 'L', Vs_f), ('unloaded', 'U', Vs_e)):
    dp.append({'id': f'C{mc}', 'mass': case, 'V': V_CRUISE, 'gamma_deg': 0.0, 'basis': 'cruise'})
    dp.append({'id': f'A{mc}', 'mass': case, 'V': round(1.3 * Vs, 1), 'gamma_deg': -4.0, 'basis': '1.3 Vs'})
    dp.append({'id': f'S{mc}', 'mass': case, 'V': round(1.1 * Vs, 1), 'gamma_deg': 0.0, 'basis': '1.1 Vs'})


def v(x, n=5):
    return f'{x:.{n}g}'


def coef(name, val, source, note=None):
    s = f'    {name + ":":<10}{{value: {v(val)}, source: {source}'
    if note:
        s += f', note: "{note}"'
    return s + '}'


hull_boxes = []
for b_ in GEO['collision']['fuselage']:
    c_rel = flu_to_frd(list(np.array(b_['xyz']) - cg_e))
    hull_boxes.append(f'    - {{center_frd: [{v(c_rel[0], 4)}, {v(c_rel[1], 4)}, {v(c_rel[2], 4)}], '
                      f'size: [{v(b_["size"][0], 4)}, {v(b_["size"][1], 4)}, {v(b_["size"][2], 4)}]}}')

Df = D_full
Y = f'''# CL-415 UAV parameters in the params.yaml schema v1 of FireFightingUAVF26-27 (see params/schema.md there).
# Generated by cl415/flight_params/build_flight_params.py. SI units, radians unless the key ends in _deg.
# Body FRD, inertial NED. Aero moment coefficients are about the FULL-TANK CG (the team model applies one
# set to both mass cases); flight_params/report.md lists the empty-CG values as well.
# Sources: avl = AVL 3.52 vortex lattice on the CAD geometry (flight_params/avl), xfoil = XFOIL 6.99 section data,
# solidworks = mass properties measured on the SolidWorks parts, cad = CadQuery geometry, handbook = textbook
# method, apc = APC published prop data with a generic motor model (flight_params/propulsion).

meta:
  schema_version: 1
  aircraft: cl415
  description: "2.51 m span scaled CL-415 water bomber UAV (about 1:11.5), NACA 4417 wing, 10.5 kg with 3 L of water"
  units: SI
  angles: "rad unless key ends _deg"
  body_frame: FRD
  inertial_frame: NED
  reference_report: "cl415/flight_params/report.md"

sign_conventions:
  delta_e: "+ trailing edge down -> nose-down pitching moment (Cm_de < 0, CL_de > 0)"
  delta_a: "+ produces positive (right-wing-down) rolling moment (Cl_da > 0): left aileron TE down, right TE up"
  delta_r: "+ trailing edge left -> nose-left yawing moment (Cn_dr < 0)"
  delta_L: "not used on the CL-415 (no direct-lift surface)"
  throttle: "0..1, fraction of full throttle"
  alpha: "+ nose above relative wind"
  beta: "+ relative wind from the right"
  note_px4: "PX4 torque +nose-up/+right-wing-down/+nose-right; handle with servo reversal"
  note_gazebo: "the Gazebo model (cl415_description) uses ailerons + = TE down on each side, rudder + = TE left"

environment:
  g: 9.81
  rho_sl: 1.225
  rho_water: 998.0
  isa: true

geometry:
  b: {v(B)}                    # wingspan [m] (spec)
  S: {v(S)}                  # reference area [m^2], wing planform incl. the part over the hull (cad)
  c_bar: {v(C)}                 # mean aerodynamic chord [m], rectangular wing (spec)
  AR: {v(AR)}
  oswald_e: {v(e_osw, 4)}               # from the CD0 + K CL^2 fit, e = 1 / (pi AR K)
  L_fus: {v(L_FUS, 4)}                # hull length [m] (cad)

mass_properties:
  use_cad: true              # SolidWorks numbers in the cad block below
  airframe:
    m: {v(m_e)}                   # dry aircraft [kg]
    m_rounded: {v(m_e)}
    kx_over_b: {v(math.sqrt(J_e['Ixx'] / m_e) / B, 4)}           # back-computed from SolidWorks, for reference only
    ky_over_Lfus: {v(math.sqrt(J_e['Iyy'] / m_e) / L_FUS, 4)}
    kz_rule: "SolidWorks tensor, not a rule"
  payload:
    m: {v(MASS['full']['mass'] - m_e)}                   # water [kg]
    r_from_airframe_cg_frd: [{v(water_rel_frd[0], 4)}, {v(water_rel_frd[1], 4)}, {v(water_rel_frd[2], 4)}]   # water centroid
  loaded:
    m: {v(m_f)}
    m_rounded: {v(m_f)}
  unloaded:
    m: {v(m_e)}
    m_rounded: {v(m_e)}
  use_rounded_mass: false
  Ixz_airframe: {v(J_e['Ixz'], 4)}
  cad:
    # SolidWorks, about each configuration's own CG, FRD axes, Ixz = +integral(x z dm) (team convention)
    # cg_frd is relative to the airframe (empty) CG; the full CG is 2.5 mm fwd and 35 mm lower
    loaded:   {{m: {v(m_f)}, cg_frd: [{v(cg_full_rel_frd[0], 4)}, {v(cg_full_rel_frd[1], 4)}, {v(cg_full_rel_frd[2], 4)}], Ixx: {v(J_f['Ixx'], 5)}, Iyy: {v(J_f['Iyy'], 5)}, Izz: {v(J_f['Izz'], 5)}, Ixz: {v(J_f['Ixz'], 4)}}}
    unloaded: {{m: {v(m_e)}, cg_frd: [0.0, 0.0, 0.0], Ixx: {v(J_e['Ixx'], 5)}, Iyy: {v(J_e['Iyy'], 5)}, Izz: {v(J_e['Izz'], 5)}, Ixz: {v(J_e['Ixz'], 4)}}}

aero:
  stall:
    CL_max: {v(CL_MAX, 4)}             # avl critical section with XFOIL 4417 cl_max x {XFOIL_CLMAX_KNOCKDOWN} at the stall Re
    blend_M: 50.0            # sigmoid sharpness (Beard & McLain), as in the team file
  drag_polar:
    CD0: {v(CD0p, 4)}              # avl Trefftz + XFOIL profile (x{ROUGHNESS}) + hull/nacelle/float build-up, +{MISC*100:.0f} % misc
    K: {v(Kp, 4)}                # fit of CD = CD0 + K CL^2 over CL 0.1-1.25
  longitudinal:
{coef('CL0', Df['CL0'], src_avl, 'body alpha = 0, includes 2.65 deg wing incidence and camber')}
{coef('CL_alpha', Df['CL_alpha'], src_avl)}
{coef('CL_q', Df['CL_q'], src_avl)}
{coef('CL_de', Df['CL_de'], 'avl+xfoil', 'avl x ' + f"{CTRL_FACTOR['e']:.3f}" + ' viscous flap factor')}
{coef('CL_dL', 0.0, 'n/a', 'no direct-lift surface')}
{coef('CD_de', CD_DE, 'xfoil', 'elevator section drag at 10 deg scaled by elevator area')}
{coef('Cm0', Df['Cm0'], src_avl, 'about the full-tank CG')}
{coef('Cm_alpha', Df['Cm_alpha'], src_avl, 'static margin ' + f"{-Df['Cm_alpha']/Df['CL_alpha']*100:.0f}" + ' % MAC')}
{coef('Cm_q', Df['Cm_q'], src_avl)}
{coef('Cm_de', Df['Cm_de'], 'avl+xfoil', 'avl x ' + f"{CTRL_FACTOR['e']:.3f}" + ' viscous flap factor')}
{coef('Cm_dL', 0.0, 'n/a', 'no direct-lift surface')}
  lateral:
{coef('CY0', 0.0, 'symmetry')}
{coef('Cl0', 0.0, 'symmetry')}
{coef('Cn0', 0.0, 'symmetry', 'prop torque and swirl not included')}
{coef('CY_beta', Df['CY_beta'], src_avl)}
{coef('Cl_beta', Df['Cl_beta'], src_avl)}
{coef('Cn_beta', Df['Cn_beta'], src_avl, 'hull as an AVL slender body; without it ' + f"{D_nobody['Cn_beta']:.3f}")}
{coef('CY_p', Df['CY_p'], src_avl)}
{coef('Cl_p', Df['Cl_p'], src_avl)}
{coef('Cn_p', Df['Cn_p'], src_avl)}
{coef('CY_r', Df['CY_r'], src_avl)}
{coef('Cl_r', Df['Cl_r'], src_avl)}
{coef('Cn_r', Df['Cn_r'], src_avl)}
{coef('CY_da', Df['CY_da'], 'avl+xfoil', 'avl x ' + f"{CTRL_FACTOR['a']:.3f}" + ' viscous flap factor')}
{coef('Cl_da', Df['Cl_da'], 'avl+xfoil', 'avl x ' + f"{CTRL_FACTOR['a']:.3f}" + ' viscous flap factor')}
{coef('Cn_da', Df['Cn_da'], 'avl+xfoil', 'avl x ' + f"{CTRL_FACTOR['a']:.3f}" + ' viscous flap factor')}
{coef('CY_dr', Df['CY_dr'], 'avl+xfoil', 'avl x ' + f"{CTRL_FACTOR['r']:.3f}" + ' viscous flap factor')}
{coef('Cl_dr', Df['Cl_dr'], 'avl+xfoil', 'avl x ' + f"{CTRL_FACTOR['r']:.3f}" + ' viscous flap factor')}
{coef('Cn_dr', Df['Cn_dr'], 'avl+xfoil', 'avl x ' + f"{CTRL_FACTOR['r']:.3f}" + ' viscous flap factor')}
  mixing:
    # conventional tail: only the rear surface (elevator) is used, no direct lift
    Mix: [[0.0, 0.0],
          [1.0, 0.0]]

propulsion:
  # baseline: {base['prop']} props on {base['motor']} ({base['kv']:.0f} kV) motors, 6S 16 Ah
  # (flight_params/propulsion lists every candidate, with thrust maps and model fits)
  n_motors: 2
  P_elec_max_each: {v(base['static_p_batt'] / 2, 4)}     # W, full throttle static
  P_elec_max_total: {v(base['static_p_batt'], 4)}
  eta_p: {v(base['cruise']['eta_prop'], 3)}               # prop efficiency at 20 m/s cruise (apc data)
  eta_total: {v(req['full']['T_cruise20'] * V_CRUISE / base['cruise']['p_batt'], 3)}           # thrust power / battery power at cruise
  model: momentum
  legacy_constant_power:
    P_prop_max: {v(req['full']['T_cruise20'] * V_CRUISE / tr_l['dT'], 4)}         # not physical, kept for the schema (D-02 in the team repo)
    V_min: 5.0
    source: "n/a for the CL-415, use momentum"
  momentum:
    # T = a dT^2 - b dT V (both motors), least-squares fit to the motor+prop map, rms {bf['momentum_fit_rms_N']:.2f} N
    T_static_total: {v(bf['momentum_a_T_static_total'], 4)}
    V_anchor: {V_CRUISE}
    T_anchor: {v(req['full']['T_cruise20'], 4)}       # cruise drag at 20 m/s, full tank
    dT_anchor: {v(tr_l['dT'], 4)}
    b: {v(bf['momentum_b'], 4)}
    P_check_W: {v(base['static_p_batt'], 4)}
    source: "apc {base['prop']} data + {base['motor']} motor model on 6S, flight_params/propulsion"
  thrust_line:
    through: prop_hubs       # both hubs, 0.16 m above the full-tank CG
    r_thrust_from_loaded_cg_frd: [{v(r_thrust_frd[0], 4)}, 0.0, {v(r_thrust_frd[2], 4)}]

actuators:
  elevator: {{tau: 0.03, delta_max_deg: 25.0, rate_max_deg_s: 500.0}}
  aileron:  {{tau: 0.03, delta_max_deg: 25.0, rate_max_deg_s: 500.0}}
  rudder:   {{tau: 0.03, delta_max_deg: 30.0, rate_max_deg_s: 500.0}}
  throttle: {{tau: {v(max(0.05, bf['motor_time_constant_s']), 3)}, min: 0.0, max: 1.0}}
  direct_lift: {{tau: 0.05, delta_max_deg: 0.0, rate_max_deg_s: 0.0}}    # not fitted

speeds:
  V_cruise: {V_CRUISE}
  V_stall_loaded: {v(Vs_f, 4)}
  V_stall_unloaded: {v(Vs_e, 4)}
  approach_factor: 1.3
  slow_factor: 1.1

trim_reference:
  # this file's own static balance (L = W, Cm = 0, T = D), same equations as static_trim.m
  altitude_m: {H_TRIM}
  cruise_loaded:   {{V: {V_CRUISE}, mass: loaded,   gamma_deg: 0.0, CL: {v(tr_l['CL'], 4)}, alpha_deg: {v(tr_l['alpha_deg'], 4)}, de_deg: {v(tr_l['de_deg'], 4)}, thrust_N: {v(tr_l['thrust_N'], 4)}, dT: {v(tr_l['dT'], 4)}}}
  cruise_unloaded: {{V: {V_CRUISE}, mass: unloaded, gamma_deg: 0.0, CL: {v(tr_u['CL'], 4)}, alpha_deg: {v(tr_u['alpha_deg'], 4)}, de_deg: {v(tr_u['de_deg'], 4)}, thrust_N: {v(tr_u['thrust_N'], 4)}, dT: {v(tr_u['dT'], 4)}}}
  tolerance:
    alpha_deg: 0.02
    de_deg: 0.02
    dT: 0.001
  full_trim_tolerance:
    alpha_deg: 0.3           # thrust tilt
    de_deg: 2.0              # the thrust line is 0.16 m above the CG, so the full trim needs more up elevator
    dT: 0.01

design_points:
''' + '\n'.join(f"  - {{id: {d['id']}, mass: {d['mass']}, V: {d['V']}, gamma_deg: {d['gamma_deg']}, basis: \"{d['basis']}\"}}" for d in dp) + f'''

hydro:
  enabled: false
  water_z_ned: 0.0
  # collision boxes of the Gazebo model, relative to the airframe CG (FRD), sized to keep the hull volume
  hull_boxes:
''' + '\n'.join(hull_boxes) + '''

control_specs:
  phase_margin_deg_min: 45.0
  gain_margin_dB_min: 6.0
  loop_separation_min: 5.0
  rate_step_deg_s: 30.0
  attitude_step_deg: 10.0
  robustness:
    Cm_alpha_scale: [0.7, 1.0, 1.3]
    Cm_q_scale:     [0.7, 1.0, 1.3]
    mass_scale:     [0.9, 1.0, 1.1]
    pm_min_offnominal_deg: 40.0
    gm_min_offnominal_dB: 5.0
'''
open(os.path.join(HERE, 'cl415_params.yaml'), 'w', encoding='utf-8').write(Y)

import yaml  # noqa: E402
raw = Y.encode('utf-8')
data = yaml.safe_load(raw)
data['meta']['sha256'] = hashlib.sha256(raw).hexdigest()
data['meta']['source_file'] = 'cl415_params.yaml'
json.dump(data, open(os.path.join(HERE, 'cl415_params.json'), 'w'), indent=1)

# ---------------------------------------------------------------- everything else, machine readable
full = {
    'geometry': {'b': B, 'S': S, 'c_bar': C, 'AR': AR, 'L_fus': L_FUS, 'wing_incidence_deg': A['wing']['incidence_deg'],
                 'S_htail': S_h, 'S_vtail': S_v, 'S_finlets': S_fl, 'aileron_y': A['aileron']['y'],
                 'aileron_cf_c': A['aileron']['cf_c'], 'elevator_cf_c': A['htail']['cf_c'],
                 'rudder_cf_c_root_tip': [A['vtail']['cf_c_root'], A['vtail']['cf_c_tip']],
                 'prop_hub_frd_from_full_cg': r_thrust_frd, 'prop_diameter': A['props']['diameter']},
    'mass': {'full': {'m': m_f, 'cg_flu': cg_f.tolist(), 'J_frd': J_f, 'cg_pct_mac': MASS['full']['cg_pct_mac']},
             'empty': {'m': m_e, 'cg_flu': cg_e.tolist(), 'J_frd': J_e, 'cg_pct_mac': MASS['empty']['cg_pct_mac']}},
    'aero': {'about_full_cg': D_full, 'about_empty_cg': D_empty, 'no_bodies_full_cg': D_nobody,
             'control_viscous_factors': CTRL_FACTOR,
             'CD0': CD0p, 'K': Kp, 'oswald_e': e_osw, 'CD_de': CD_DE, 'polar_points': polar.tolist(),
             'CD_breakdown_at_cruise': {'tail_profile': cd_tail, 'bodies': cd_bodies, 'misc_fraction': MISC},
             'CL_max': CL_MAX, 'stall': {'full': [CLmax_f, al_st_f, Vs_f, clsec_f, re_st_f],
                                         'empty': [CLmax_e, al_st_e, Vs_e, clsec_e, re_st_e]},
             'avl_trim20': {'full': AV['runs']['full_trim20'], 'empty': AV['runs']['empty_trim20']}},
    'airfoils': XF,
    'propulsion': {'requirements': {k: {kk: vv for kk, vv in r.items() if not kk.startswith('grid')} for k, r in req.items()},
                   'T_required_static_recommended': T_req_static, 'hump_R_over_W': HUMP_R_OVER_W,
                   'battery': {'cells': bat.cells, 'capacity_ah': bat.capacity_ah, 'energy_wh': bat.energy_wh},
                   'candidates': [{k: v for k, v in c.items() if not k.startswith('_')} for c in cands],
                   'fits': fits, 'baseline': base_tag},
    'trim_reference': {'loaded': tr_l, 'unloaded': tr_u},
}
json.dump(full, open(os.path.join(HERE, 'cl415_flight_data.json'), 'w'), indent=1, default=float)

# ---------------------------------------------------------------- plots
fig, axs = plt.subplots(1, 3, figsize=(17, 5))
ax = axs[0]
ax.plot(CLa_sweep[:, 0], CLa_sweep[:, 1], 'o-', label='CL (AVL, linear)')
ax.plot(CLa_sweep[:, 0], CLa_sweep[:, 2], 's--', label='highest wing strip cl')
ax.axhline(clsec_f, color='r', ls=':', label=f'section cl_max x{XFOIL_CLMAX_KNOCKDOWN} (Re {re_st_f/1e3:.0f}k)')
ax.axvline(al_st_f, color='r', lw=0.8)
ax.set_xlabel('body alpha [deg]')
ax.set_ylabel('CL, cl')
ax.legend(fontsize=8)
ax.grid(True, lw=0.3)
ax.set_title(f'stall: CL_max = {CL_MAX:.3f} at {al_st_f:.1f} deg')
ax = axs[1]
ax.plot(polar[:, 2], polar[:, 1], 'o', label='build-up')
cl_ = np.linspace(0, 1.3, 50)
ax.plot(CD0p + Kp * cl_ ** 2, cl_, '-', label=f'CD = {CD0p:.4f} + {Kp:.4f} CL^2')
ax.set_xlabel('CD')
ax.set_ylabel('CL')
ax.legend(fontsize=8)
ax.grid(True, lw=0.3)
ax.set_title(f'drag polar, e = {e_osw:.2f}')
ax = axs[2]
for case, ls in (('full', '-'), ('empty', '--')):
    ax.plot(req[case]['grid_V'], req[case]['grid_D'], ls, label=f'thrust required, {case}')
Vb = speeds
Tb, _, _, _ = PT.thrust_map(base_prop, base_mot, bat, Vb, [0.25, 0.5, 0.75, 1.0])
for j, d in enumerate([0.25, 0.5, 0.75, 1.0]):
    ax.plot(Vb, Tb[:, j], color='k', alpha=0.25 + 0.6 * d, lw=1, label=f'available {d:.2f} throttle' if d in (0.25, 1.0) else None)
ax.axhline(req['full']['T_hump'], color='r', ls=':', label='water take-off hump (full)')
ax.set_xlabel('V [m/s]')
ax.set_ylabel('thrust [N]')
ax.set_ylim(0, max(Tb.max(), 40) * 1.05)
ax.legend(fontsize=7)
ax.grid(True, lw=0.3)
ax.set_title(f'baseline {base_tag}')
fig.tight_layout()
fig.savefig(os.path.join(PLOTS, 'aero_and_thrust.png'), dpi=130)
plt.close(fig)

print('CL0 %.4f CLa %.3f Cm0 %.4f Cma %.3f CD0 %.4f K %.4f e %.3f CLmax %.3f Vs %.2f/%.2f' %
      (Df['CL0'], Df['CL_alpha'], Df['Cm0'], Df['Cm_alpha'], CD0p, Kp, e_osw, CL_MAX, Vs_f, Vs_e))
print('trim loaded', {k: round(vv, 3) if isinstance(vv, float) else vv for k, vv in tr_l.items()})
print('trim unloaded', {k: round(vv, 3) if isinstance(vv, float) else vv for k, vv in tr_u.items()})
print('requirements full', {k: round(vv, 2) for k, vv in full['propulsion']['requirements']['full'].items()})
print('T static recommended', round(T_req_static, 1))
print('feasible', len(feas), 'of', len(cands), 'baseline', base_tag, bf)
for c in sorted(cands, key=lambda c: (not c['feasible'], c.get('cruise_p_batt', 1e9)))[:14]:
    print(f"{c['prop']:28s} {c['kv']:4.0f}kv static {c['static_thrust_total']:5.1f} N {c['static_i_motor']:5.1f} A/mot "
          f"hump {c['hump_thrust_total']:5.1f} cruise thr {c['cruise']['throttle'] if c['cruise'] else float('nan'):.2f} "
          f"P {c.get('cruise_p_batt', float('nan')):6.1f} W end {c.get('endurance_min', float('nan')):5.1f} min "
          f"vmax {c['v_max']} rpm {c['static_rpm']:.0f}/{c['rpm_limit']:.0f} clr {c['tip_clearance_m']*1000:.0f}mm "
          f"{'OK' if c['feasible'] else [k for k in ('ok_takeoff','ok_current','ok_rpm','ok_clearance','ok_cruise') if not c[k]]}")
