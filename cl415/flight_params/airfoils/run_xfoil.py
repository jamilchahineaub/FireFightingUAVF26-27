"""airfoil properties for the cl415 sections from XFOIL 6.99 (flight_params/bin/xfoil.exe)

sections: NACA 4417 (wing), NACA 0016 (h-stab), NACA 0012 (fin and finlets)
for each: polars at several Reynolds numbers (Ncrit 9, free transition), and flap runs at the CAD hinge
positions to get the real control effectiveness (XFOIL GDES FLAP, hinge at mid-thickness)

writes flight_params/airfoils/polars/*.csv, summary.json, report.md and plots/*.png
run from the project root:  .venv/Scripts/python flight_params/airfoils/run_xfoil.py
"""
import json
import math
import os
import shutil
import subprocess
import tempfile
from concurrent.futures import ThreadPoolExecutor

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
XFOIL = os.path.join(ROOT, 'flight_params', 'bin', 'xfoil.exe')
GEO = json.load(open(os.path.join(ROOT, 'cad', 'geometry.json')))
A = GEO['aero']
POL = os.path.join(HERE, 'polars')
PLOTS = os.path.join(HERE, 'plots')
for d in (POL, PLOTS):
    os.makedirs(d, exist_ok=True)

NU = 1.46e-5          # m^2/s, 15 C sea level
NCRIT = 9
SECTIONS = {
    # name: naca code, chord [m], hinge x/c of its control surface, surface name
    'wing_4417': {'naca': '4417', 'chord': A['wing']['chord'], 'hinge': 1 - A['aileron']['cf_c'], 'surface': 'aileron'},
    'htail_0016': {'naca': '0016', 'chord': A['htail']['chord'], 'hinge': 1 - A['htail']['cf_c'], 'surface': 'elevator'},
    'fin_0012': {'naca': '0012', 'chord': A['vtail']['mac'],
                 'hinge': 1 - 0.5 * (A['vtail']['cf_c_root'] + A['vtail']['cf_c_tip']), 'surface': 'rudder'},
}
RE_LIST = [1.5e5, 2.0e5, 3.0e5, 4.0e5, 5.0e5, 6.0e5]
SPEEDS = [12.0, 14.0, 17.0, 20.0, 25.0, 30.0]       # m/s, for the Re table in the report
FLAP_DEG = [-10.0, -5.0, 5.0, 10.0]
FLAP_RE = 3.0e5


def xfoil(commands, outfile):
    """run xfoil in a scratch dir (it truncates long paths), return the polar file text"""
    tmp = tempfile.mkdtemp(prefix='xf')
    try:
        shutil.copy(XFOIL, tmp)
        script = 'PLOP\nG F\n\n' + commands.replace('@POLAR@', 'p.pol') + '\nQUIT\n'
        subprocess.run([os.path.join(tmp, 'xfoil.exe')], input=script, text=True, cwd=tmp,
                       capture_output=True, timeout=600)
        p = os.path.join(tmp, 'p.pol')
        txt = open(p).read() if os.path.exists(p) else ''
        open(outfile, 'w').write(txt)
        return txt
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def parse_polar(txt):
    rows = []
    started = False
    for line in txt.splitlines():
        if line.strip().startswith('-----'):
            started = True
            continue
        if started and line.strip():
            v = line.split()
            if len(v) >= 7:
                rows.append([float(x) for x in v[:7]])
    if not rows:
        return np.zeros((0, 7))
    a = np.array(rows)
    a = a[np.argsort(a[:, 0])]
    _, idx = np.unique(np.round(a[:, 0], 3), return_index=True)
    return a[idx]


def polar_cmds(naca, re, flap=None):
    geo = f'NACA {naca}\n'
    if flap is not None:
        hinge, deg = flap
        geo += f'GDES\nFLAP\n{hinge}\n999\n0.5\n{deg}\nX\n\nPANE\n'
    oper = (f'OPER\nVISC {re:.0f}\nITER 300\nVPAR\nN {NCRIT}\n\nPACC\n@POLAR@\n\n')
    if flap is None:
        sweep = 'ASEQ 0 22 0.25\nINIT\nASEQ -0.25 -14 -0.25\n'
    else:
        sweep = 'ASEQ 0 8 0.5\nINIT\nASEQ -0.5 -6 -0.5\n'
    return geo + oper + sweep + 'PACC\n\n'


def props(a, naca):
    """2d properties from a polar array [alpha cl cd cdp cm xtr_top xtr_bot]"""
    al, cl, cd, cm = a[:, 0], a[:, 1], a[:, 2], a[:, 4]
    lin = (al >= -4) & (al <= 4) if naca[0] == '0' else (al >= -6) & (al <= 2)
    k, c0 = np.polyfit(np.radians(al[lin]), cl[lin], 1)
    a_l0 = -c0 / k
    imax = int(np.argmax(cl))
    neg = al < 0
    imin = int(np.argmin(np.where(neg, cl, np.inf)))
    dcm = np.polyfit(cl[lin], cm[lin], 1)
    cm0 = float(np.polyval(dcm, 0.0))
    ld = cl / cd
    ild = int(np.argmax(ld))
    icd = int(np.argmin(cd))
    pol = (cl > cl[icd] - 0.6) & (cl < min(cl[imax] * 0.85, cl[icd] + 0.8)) & (al < al[imax])
    q = np.polyfit(cl[pol], cd[pol], 2)
    return {'alpha_L0_deg': math.degrees(a_l0), 'cl_alpha_per_rad': k, 'cl_alpha_per_deg': k * math.pi / 180,
            'cl_max': float(cl[imax]), 'alpha_cl_max_deg': float(al[imax]),
            'cl_min': float(cl[imin]), 'alpha_cl_min_deg': float(al[imin]),
            'cd_min': float(cd[icd]), 'cl_at_cd_min': float(cl[icd]),
            'cm0': cm0, 'dcm_dcl': float(dcm[0]), 'x_ac_over_c': 0.25 - float(dcm[0]),
            'ld_max': float(ld[ild]), 'cl_at_ld_max': float(cl[ild]), 'alpha_at_ld_max_deg': float(al[ild]),
            'cd_fit_c2_c1_c0': [float(x) for x in q], 'alpha_range_deg': [float(al.min()), float(al.max())],
            'points': int(len(al))}


def run_all():
    jobs = []
    for name, s in SECTIONS.items():
        for re in RE_LIST:
            jobs.append((name, re, None, polar_cmds(s['naca'], re), os.path.join(POL, f'{name}_Re{re/1e3:.0f}k.pol')))
        for deg in FLAP_DEG:
            jobs.append((name, FLAP_RE, deg, polar_cmds(s['naca'], FLAP_RE, (s['hinge'], deg)),
                         os.path.join(POL, f'{name}_flap{deg:+.0f}_Re{FLAP_RE/1e3:.0f}k.pol')))
    with ThreadPoolExecutor(max_workers=max(2, (os.cpu_count() or 4) - 2)) as ex:
        res = list(ex.map(lambda j: (j, xfoil(j[3], j[4])), jobs))
    out = {}
    for (name, re, deg, _, path), txt in res:
        a = parse_polar(txt)
        csv = path.replace('.pol', '.csv')
        np.savetxt(csv, a, delimiter=',', fmt='%.5f', header='alpha_deg,cl,cd,cdp,cm,xtr_top,xtr_bot', comments='')
        out.setdefault(name, {}).setdefault('flap' if deg is not None else 'clean', {})[
            f'{deg:+.0f}' if deg is not None else f'{re:.0f}'] = a
    return out


def main():
    data = run_all()
    summary = {'xfoil': '6.99', 'ncrit': NCRIT, 'nu': NU, 'sections': {}}
    for name, s in SECTIONS.items():
        clean = data[name]['clean']
        S = {'naca': s['naca'], 'chord_m': s['chord'], 'reynolds': {}}
        for re in RE_LIST:
            a = clean[f'{re:.0f}']
            S['reynolds'][f'{re:.0f}'] = props(a, s['naca'])
        # flap effectiveness at FLAP_RE: delta cl and delta cm at fixed alpha, averaged over the linear range
        base = clean[f'{FLAP_RE:.0f}']
        rows = []
        for key, a in sorted(data[name]['flap'].items(), key=lambda kv: float(kv[0])):
            d = float(key)
            al_common = [x for x in np.arange(-3, 4.01, 0.5) if x in set(np.round(a[:, 0], 2)) and x in set(np.round(base[:, 0], 2))]
            if len(al_common) < 4:
                continue
            f = lambda arr, col, x: float(np.interp(x, arr[:, 0], arr[:, col]))
            dcl = np.mean([f(a, 1, x) - f(base, 1, x) for x in al_common])
            dcm = np.mean([f(a, 4, x) - f(base, 4, x) for x in al_common])
            dcd = np.mean([f(a, 2, x) - f(base, 2, x) for x in al_common])
            rows.append((d, dcl, dcm, dcd))
        rows = np.array(rows)
        k_cl = np.polyfit(np.radians(rows[:, 0]), rows[:, 1], 1)[0]
        k_cm = np.polyfit(np.radians(rows[:, 0]), rows[:, 2], 1)[0]
        cla = S['reynolds'][f'{FLAP_RE:.0f}']['cl_alpha_per_rad']
        E = 1 - s['hinge']
        th = math.acos(2 * E - 1)
        tau_thin = 1 - th / math.pi + math.sin(th) / math.pi
        S['flap'] = {'surface': s['surface'], 'hinge_x_over_c': s['hinge'], 'cf_over_c': E, 'reynolds': FLAP_RE,
                     'dcl_ddelta_per_rad': k_cl, 'dcm_ddelta_per_rad': k_cm, 'tau_xfoil': k_cl / cla,
                     'tau_thin_airfoil': tau_thin,
                     'table': [{'delta_deg': r[0], 'dcl': r[1], 'dcm': r[2], 'dcd': r[3]} for r in rows]}
        summary['sections'][name] = S
    json.dump(summary, open(os.path.join(HERE, 'summary.json'), 'w'), indent=1)

    # plots
    for name, s in SECTIONS.items():
        fig, axs = plt.subplots(2, 2, figsize=(12, 9))
        for re in RE_LIST:
            a = data[name]['clean'][f'{re:.0f}']
            lab = f'Re {re / 1e3:.0f}k'
            axs[0, 0].plot(a[:, 0], a[:, 1], label=lab)
            axs[0, 1].plot(a[:, 2], a[:, 1], label=lab)
            axs[1, 0].plot(a[:, 0], a[:, 4], label=lab)
            axs[1, 1].plot(a[:, 0], a[:, 1] / a[:, 2], label=lab)
        for ax, (xl, yl) in zip(axs.ravel(), [('alpha [deg]', 'cl'), ('cd', 'cl'), ('alpha [deg]', 'cm c/4'),
                                              ('alpha [deg]', 'cl/cd')]):
            ax.set_xlabel(xl)
            ax.set_ylabel(yl)
            ax.grid(True, lw=0.3)
        axs[0, 1].set_xlim(0, 0.06)
        axs[0, 0].legend(fontsize=8)
        fig.suptitle(f'NACA {s["naca"]} ({name.split("_")[0]}), XFOIL 6.99, Ncrit {NCRIT}')
        fig.tight_layout()
        fig.savefig(os.path.join(PLOTS, f'{name}.png'), dpi=130)
        plt.close(fig)
    write_report(summary)


def write_report(summary):
    L = ['# Airfoil properties (XFOIL)', '',
         f'XFOIL 6.99, viscous, free transition with Ncrit = {NCRIT} (clean model surface, low-turbulence air), '
         'incompressible. Polars in `polars/*.csv`, plots in `plots/`. Generated by `run_xfoil.py`.', '',
         '## Reynolds numbers in flight', '',
         '| V [m/s] | wing (c = 0.278 m) | h-stab (c = 0.239 m) | fin MAC (0.310 m) |', '|---:|---:|---:|---:|']
    for V in SPEEDS:
        L.append(f'| {V:.0f} | {V * SECTIONS["wing_4417"]["chord"] / NU / 1e3:.0f}k | '
                 f'{V * SECTIONS["htail_0016"]["chord"] / NU / 1e3:.0f}k | {V * SECTIONS["fin_0012"]["chord"] / NU / 1e3:.0f}k |')
    for name, S in summary['sections'].items():
        L += ['', f'## NACA {S["naca"]} ({name.split("_")[0]})', '',
              '| Re | alpha_L0 [deg] | cl_alpha [/rad] | cl_max | alpha at cl_max [deg] | cl_min | cd_min | cl at cd_min | '
              'cm0 (c/4) | x_ac/c | (cl/cd)max | cl at (cl/cd)max |',
              '|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|']
        for re, p in S['reynolds'].items():
            L.append(f'| {float(re) / 1e3:.0f}k | {p["alpha_L0_deg"]:.2f} | {p["cl_alpha_per_rad"]:.3f} | {p["cl_max"]:.3f} | '
                     f'{p["alpha_cl_max_deg"]:.2f} | {p["cl_min"]:.3f} | {p["cd_min"]:.4f} | {p["cl_at_cd_min"]:.3f} | '
                     f'{p["cm0"]:.4f} | {p["x_ac_over_c"]:.3f} | {p["ld_max"]:.0f} | {p["cl_at_ld_max"]:.2f} |')
        F = S['flap']
        L += ['', f'{F["surface"].capitalize()} effectiveness at Re {F["reynolds"] / 1e3:.0f}k, hinge at '
              f'{F["hinge_x_over_c"]:.3f} c (cf/c = {F["cf_over_c"]:.3f}), deflections '
              f'{", ".join(f"{r["delta_deg"]:+.0f}" for r in F["table"])} deg: '
              f'dcl/ddelta = {F["dcl_ddelta_per_rad"]:.3f} /rad, dcm/ddelta = {F["dcm_ddelta_per_rad"]:.3f} /rad, '
              f'tau = {F["tau_xfoil"]:.3f} (thin-airfoil theory says {F["tau_thin_airfoil"]:.3f}). '
              f'Extra drag at 10 deg: {[r["dcd"] for r in F["table"] if r["delta_deg"] == 10.0][0]:.4f}.']
    open(os.path.join(HERE, 'report.md'), 'w', encoding='utf-8').write('\n'.join(L) + '\n')
    print('\n'.join(L))


if __name__ == '__main__':
    main()
