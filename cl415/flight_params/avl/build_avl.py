"""AVL 3.52 model of the cl415 from the cad geometry, and the runs the flight sims need

geometry: wing (NACA 4417 camber, 2.65 deg incidence, ailerons), h-stab (0016, elevator), fin (0012, rudder,
swept hinge), two finlets, hull and nacelles as slender bodies. CLAF per surface from the XFOIL lift slope.
AVL axes are x aft, y right, z up; the cad is x fwd, y left, z up, so x and y flip sign here.

controls (signs chosen to match the team's params.yaml): d1 aileron (+ = right wing down roll, Cl_da > 0),
d2 elevator (+ = trailing edge down), d3 rudder (+ = trailing edge left, Cn_dr < 0)

runs, about the full-tank CG and about the empty CG:
  a0      alpha = 0, controls 0              -> linear model (CL0, Cm0 and all derivatives)
  trim    CL = W/(qS) at 20 m/s, Cm = 0 by elevator
  sweep   alpha -4..16 deg, strip forces      -> CL_max by the critical-section method
writes flight_params/avl/results.json (+ the .avl/.dat/.st files)
run from the project root:  .venv/Scripts/python flight_params/avl/build_avl.py
"""
import json
import math
import os
import re
import shutil
import subprocess
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
AVL = os.path.join(ROOT, 'flight_params', 'bin', 'avl.exe')
GEO = json.load(open(os.path.join(ROOT, 'cad', 'geometry.json')))
MEAS = json.load(open(os.path.join(ROOT, 'obj_inspect', 'measurements.json')))
MASS = json.load(open(os.path.join(ROOT, 'mass', 'mass_props.json')))
XF = json.load(open(os.path.join(ROOT, 'flight_params', 'airfoils', 'summary.json')))
A = GEO['aero']
RHO, G, NU = 1.225, 9.81, 1.46e-5
V_TRIM = 20.0


def ax(x, y, z):
    """cad (x fwd, y left, z up) -> avl (x aft, y right, z up)"""
    return -x, -y, z


def claf(section, re=4e5):
    k = XF['sections'][section]['reynolds'][f'{re:.0f}']['cl_alpha_per_rad'] / (2 * math.pi)
    return min(k, 1.0)


def sec(x, y, z, c, ainc=0.0, foil=None, claf_=None, controls=()):
    L = [f'SECTION', f'{x:.5f} {y:.5f} {z:.5f} {c:.5f} {ainc:.4f}']
    if foil:
        L += ['NACA', foil]
    if claf_:
        L += ['CLAF', f'{claf_:.4f}']
    for name, gain, xh, sdup in controls:
        L += ['CONTROL', f'{name} {gain:.3f} {xh:.5f} 0. 0. 0. {sdup:.1f}']
    return L


def wing():
    c = A['wing']['chord']
    inc = A['wing']['incidence_deg']
    y0, y1 = A['aileron']['y']
    xh = 1 - A['aileron']['cf_c']
    k = claf('wing_4417')
    ail = [('aileron', -1.0, xh, -1.0)]          # defined on the right wing; + = right TE up / left TE down
    # three abutting surfaces (inboard, aileron, tip) in one component, so the aileron starts and stops
    # sharply; defined on the right side (y_avl > 0) and mirrored by YDUPLICATE
    L = []
    for name, ya, yb, ctl, ns in (('Wing', 0.0, y0, (), 30), ('Wing_ail', y0, y1, ail, 16), ('Wing_tip', y1, b2, (), 3)):
        L += ['SURFACE', name, f'12 1.0 {ns} 1.0', 'COMPONENT', '1', 'YDUPLICATE', '0.0', 'ANGLE', '0.0']
        L += sec(0.0, ya, 0.0, c, inc, '4417', k, ctl)
        L += sec(0.0, yb, 0.0, c, inc, '4417', k, ctl)
        L += ['#']
    return L


def htail():
    H = A['htail']
    c = H['chord']
    xle = -(H['ac'][0] - (-0.25 * c))        # cad le x = ac x + c/4
    x_le_cad = H['ac'][0] + 0.25 * c
    z = H['ac'][2]
    yin = H['elevator_inner_y']
    xh = 1 - H['cf_c']
    k = claf('htail_0016')
    el = [('elevator', 1.0, xh, 1.0)]
    L = []
    for name, ya, yb, ctl, ns in (('Hstab_root', 0.0, yin, (), 2), ('Hstab', yin, H['span'] / 2, el, 18)):
        L += ['SURFACE', name, f'10 1.0 {ns} 1.0', 'COMPONENT', '2', 'YDUPLICATE', '0.0', 'ANGLE', '0.0']
        L += sec(-x_le_cad, ya, z, c, 0.0, '0016', k, ctl)
        L += sec(-x_le_cad, yb, z, c, 0.0, '0016', k, ctl)
        L += ['#']
    return L


def fin():
    V = A['vtail']
    h0, h1 = V['hinge_root'], V['hinge_tip']
    rc, tc = V['root_chord'], V['tip_chord']
    te_r = h0[0] - V['cf_c_root'] * rc
    te_t = h1[0] - V['cf_c_tip'] * tc
    le_r, le_t = te_r + rc, te_t + tc
    k = claf('fin_0012')
    L = ['SURFACE', 'Fin', '10 1.0 16 1.0', 'COMPONENT', '3', 'ANGLE', '0.0']
    # hinge vector points up along the hinge line; + avl rotation would swing the te right, gain -1 makes + = te left
    L += sec(-le_r, 0.0, h0[2], rc, 0.0, '0012', k, [('rudder', -1.0, 1 - V['cf_c_root'], 1.0)])
    L += sec(-le_t, 0.0, h1[2], tc, 0.0, '0012', k, [('rudder', -1.0, 1 - V['cf_c_tip'], 1.0)])
    return L


def finlets():
    F = A['finlets']
    yc = F['centroid_left'][1]
    te = A['htail']['x_hinge'] + 0.01446 + 0.003          # same trailing edge rule as build_cad.py
    rows = [(0.010, -0.889), (0.127, -0.7775), (0.236, -0.889)]
    L = ['SURFACE', 'Finlet', '6 1.0 10 1.0', 'COMPONENT', '4', 'YDUPLICATE', '0.0', 'ANGLE', '0.0']
    for z, xle in rows:
        L += sec(-xle, yc, z + (1e-3 if abs(z - 0.127) < 1e-6 else 0.0), xle - te, 0.0, '0012', 1.0)
    return L


def body_file(path, stations, name):
    """stations: list of (x_cad, z_centre, r_eq). writes an airfoil-format side view in avl axes"""
    st = sorted(stations, key=lambda s: -s[0])            # nose first in avl x (smallest x_avl)
    xs = [-s[0] for s in st]
    top = [(x, s[1] + s[2]) for x, s in zip(xs, st)]
    bot = [(x, s[1] - s[2]) for x, s in zip(xs, st)]
    pts = top[::-1] + bot[1:]
    with open(path, 'w') as f:
        f.write(name + '\n')
        for x, z in pts:
            f.write(f'{x:.5f} {z:.5f}\n')


def hull_stations():
    from shapely.geometry import Polygon
    H = MEAS['hull']
    FR = np.array(H['fracs'])
    out = []
    for s in H['forebody'] + H['afterbody'][1:]:
        w = np.array(s['half_breadth'], float)
        ok = w > 1e-6
        ok[0] = True
        w[0] = 0.0
        if not ok[-1]:
            w[-1] = 0.8 * w[np.where(ok)[0].max()]
            ok[-1] = True
        w = np.interp(FR, FR[ok], w[ok])
        zs = s['z_keel'] + FR * (s['z_top'] - s['z_keel'])
        poly = Polygon([(-w[k], zs[k]) for k in range(len(FR))] + [(w[k], zs[k]) for k in range(len(FR))][::-1])
        out.append((s['x'], poly.centroid.y, math.sqrt(poly.area / math.pi)))
    out.append((GEO['misc']['nose_x'], out[0][1], 0.002))
    return out


def nacelle_stations():
    return [(s['x'], s['z_c'], math.sqrt(s['half_w'] * s['half_h'])) for s in MEAS['nacelle_left']]


def write_avl(path, cg, bodies=True):
    xr, yr, zr = ax(*cg)
    L = ['CL-415 UAV 2.51 m span (cl415_description)', '0.0', '0 0 0.0',
         f'{A["wing"]["area_total"]:.5f} {A["wing"]["chord"]:.4f} {A["wing"]["span"]:.4f}',
         f'{xr:.5f} 0.0 {zr:.5f}', '0.0', '#']
    L += wing() + ['#'] + htail() + ['#'] + fin() + ['#'] + finlets() + ['#']
    if bodies:
        L += ['BODY', 'Hull', '40 1.0', 'TRANSLATE', '0.0 0.0 0.0', 'BFILE', 'hull.dat', '#']
        L += ['BODY', 'Nacelle', '20 1.0', 'YDUPLICATE', '0.0', 'TRANSLATE', f'0.0 {GEO["aero"]["props"]["center_left"][1]:.5f} 0.0',
              'BFILE', 'nacelle.dat', '#']
    open(path, 'w').write('\n'.join(L) + '\n')


def run_avl(avl_file, commands, files_out):
    """run avl in a scratch copy of HERE; return dict name -> text of each output file"""
    tmp = tempfile.mkdtemp(prefix='avl')
    try:
        for f in (avl_file, 'hull.dat', 'nacelle.dat'):
            shutil.copy(os.path.join(HERE, f), tmp)
        shutil.copy(AVL, tmp)
        script = f'LOAD {avl_file}\nOPER\n' + commands + '\n\nQUIT\n'
        r = subprocess.run([os.path.join(tmp, 'avl.exe')], input=script, text=True, cwd=tmp,
                           capture_output=True, timeout=600)
        out = {}
        for f in files_out:
            p = os.path.join(tmp, f)
            out[f] = open(p).read() if os.path.exists(p) else ''
        out['_stdout'] = r.stdout
        return out
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


NUM = r'[-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[EeDd][-+]?\d+)?'


def parse_st(txt):
    vals = {}
    for k, v in re.findall(r'([A-Za-z][\w /]*?)\s*=\s*(' + NUM + ')', txt):
        vals[k.strip()] = float(v.replace('D', 'E'))
    return vals


def parse_strips(txt, surface='Wing'):
    """rows of (yle, chord, cl) for the strips of the named surface and its mirror"""
    out = []
    blocks = re.split(r'\n\s*Surface #', txt)
    for b in blocks:
        head = b.split('\n', 1)[0]
        if surface not in head:
            continue
        for line in b.splitlines():
            v = line.split()
            if len(v) >= 10 and v[0].isdigit():
                try:
                    out.append((float(v[2]), float(v[4]), float(v[9])))
                except ValueError:
                    pass
    return out


def main():
    global b2
    b2 = A['wing']['span'] / 2
    cg = {c: MASS[c]['cg'] for c in ('full', 'empty')}
    body_file(os.path.join(HERE, 'hull.dat'), hull_stations(), 'CL415 hull equivalent body')
    body_file(os.path.join(HERE, 'nacelle.dat'), nacelle_stations(), 'CL415 nacelle equivalent body')
    res = {'reference': {'Sref': A['wing']['area_total'], 'Cref': A['wing']['chord'], 'Bref': A['wing']['span']},
           'cg': cg, 'runs': {}}
    for case in ('full', 'empty'):
        f = f'cl415_{case}.avl'
        write_avl(os.path.join(HERE, f), cg[case])
        m = MASS[case]['mass']
        CL_tr = m * G / (0.5 * RHO * V_TRIM ** 2 * A['wing']['area_total'])
        # linear model at alpha = 0
        o = run_avl(f, 'A A 0\nD1 D1 0\nD2 D2 0\nD3 D3 0\nX\nST\na0.st\n', ['a0.st'])
        res['runs'][f'{case}_a0'] = parse_st(o['a0.st'])
        open(os.path.join(HERE, f'{case}_a0.st'), 'w').write(o['a0.st'])
        # trimmed level flight at 20 m/s
        o = run_avl(f, f'A C {CL_tr:.5f}\nD2 PM 0\nX\nST\ntrim.st\n', ['trim.st'])
        res['runs'][f'{case}_trim20'] = parse_st(o['trim.st'])
        res['runs'][f'{case}_trim20']['CL_target'] = CL_tr
        open(os.path.join(HERE, f'{case}_trim20.st'), 'w').write(o['trim.st'])
        # same as a0 without the hull and nacelle bodies, to show what the bodies contribute
        fn = f'cl415_{case}_nobody.avl'
        write_avl(os.path.join(HERE, fn), cg[case], bodies=False)
        o = run_avl(fn, 'A A 0\nX\nST\na0.st\n', ['a0.st'])
        res['runs'][f'{case}_a0_nobody'] = parse_st(o['a0.st'])
    # alpha sweep with strip forces, full-tank cg
    sweep = []
    for a in np.arange(-4, 17, 1.0):
        o = run_avl('cl415_full.avl', f'A A {a:.2f}\nX\nFT\nft.txt\nFS\nfs.txt\n', ['ft.txt', 'fs.txt'])
        tot = parse_st(o['ft.txt'])
        strips = parse_strips(o['fs.txt'])
        sweep.append({'alpha_deg': float(a), 'CLtot': tot.get('CLtot'), 'CDind': tot.get('CDind'), 'CDff': tot.get('CDff'),
                      'Cmtot': tot.get('Cmtot'), 'e': tot.get('e'),
                      'strip_cl_max': max((s[2] for s in strips), default=None),
                      'strips': strips})
    res['sweep'] = sweep
    json.dump(res, open(os.path.join(HERE, 'results.json'), 'w'), indent=1)
    for k, v in res['runs'].items():
        keys = ['Alpha', 'CLtot', 'CDind', 'Cmtot', 'e', 'CLa', 'Cma', 'CYb', 'Clb', 'Cnb', 'Clp', 'Cnr', 'CLq', 'Cmq',
                'CLd02', 'Cmd02', 'Cld01', 'Cnd01', 'CYd03', 'Cnd03', 'Cld03', 'Xnp', 'd02']
        print(k, {kk: round(v[kk], 4) for kk in keys if kk in v})
    for s in sweep:
        print(f"alpha {s['alpha_deg']:5.1f}  CL {s['CLtot']:.4f}  strip cl max {s['strip_cl_max']}")


if __name__ == '__main__':
    main()
