"""propulsion toolkit: APC prop data, a dc motor model, battery and esc, and the maps the sims need

    prop = Prop('apc/PER3_15x8E.dat')
    mot = Motor(kv=450, rm=0.056, i0=0.96)
    bat = Battery(cells=6)
    op = operating_point(prop, mot, bat, throttle=0.6, v_air=20.0, n_motors=2)

steady state: the motor torque Kt (I - I0) with I = (dT V_bus - w/Kv) / Rm balances the prop torque
Cp rho n^2 D^5 / (2 pi) at J = V / (n D). thrust Ct rho n^2 D^4. Ct, Cp come from APC's published
performance files (blade-element results per rpm), interpolated in J and rpm.
"""
import math
import os
import re

import numpy as np

RHO = 1.225


class Prop:
    def __init__(self, path):
        self.name = os.path.basename(path).replace('PER3_', '').replace('.dat', '')
        # the first line has the real size, e.g. "15.5x12-4" for the file 155x12-4
        head = open(path, errors='ignore').readline().split()
        m = re.match(r'(\d+(?:\.\d+)?)x(\d+(?:\.\d+)?)', head[0] if head else '')
        if m is None:
            m = re.match(r'(\d+(?:\.\d+)?)x(\d+(?:\.\d+)?)', self.name.replace('85E', '8.5E'))
        self.d_in = float(m.group(1))
        self.pitch_in = float(m.group(2))
        bm = re.search(r'-(\d)$', self.name)
        self.blades = int(bm.group(1)) if bm else 2
        self.D = self.d_in * 0.0254
        # APC rpm limits: 145k/D for thin electric, 190k/D for sport and scale props
        self.rpm_limit = (145000.0 if 'E' in self.name.split('-')[0] else 190000.0) / self.d_in
        self.blocks = []
        rpm = None
        rows = []
        for line in open(path, errors='ignore'):
            mm = re.search(r'PROP RPM\s*=\s*(\d+)', line)
            if mm:
                if rpm is not None and rows:
                    self.blocks.append((rpm, np.array(rows)))
                rpm, rows = float(mm.group(1)), []
                continue
            v = line.split()
            if rpm is not None and len(v) >= 5:
                try:
                    rows.append([float(v[1]), float(v[3]), float(v[4])])     # J, Ct, Cp
                except ValueError:
                    pass
        if rpm is not None and rows:
            self.blocks.append((rpm, np.array(rows)))
        self.rpms = np.array([b[0] for b in self.blocks])

    def scaled(self, d_in):
        """geometrically similar prop of diameter d_in: same Ct, Cp vs J at the same tip speed"""
        import copy
        p = copy.deepcopy(self)
        k = d_in / self.d_in
        p.d_in, p.pitch_in, p.D = d_in, self.pitch_in * k, self.D * k
        p.rpms = self.rpms / k
        p.blocks = [(r / k, a) for r, a in self.blocks]
        p.rpm_limit = self.rpm_limit / k
        p.name = f'{d_in:g}x{p.pitch_in:.3g}-{self.blades} (scaled {self.name})'
        return p

    def coeffs(self, J, rpm):
        """Ct, Cp at advance ratio J and rpm (linear in both; past the table's J the prop windmills)"""
        rpm = float(np.clip(rpm, self.rpms[0], self.rpms[-1]))
        i = int(np.searchsorted(self.rpms, rpm))
        i = min(max(i, 1), len(self.rpms) - 1)
        out = []
        for k in (i - 1, i):
            a = self.blocks[k][1]
            if J > a[-1, 0]:
                # extrapolate the last two points (thrust and power go negative when windmilling)
                s = (a[-1, 1:] - a[-2, 1:]) / (a[-1, 0] - a[-2, 0])
                out.append(a[-1, 1:] + s * (J - a[-1, 0]))
            else:
                out.append(np.array([np.interp(J, a[:, 0], a[:, 1]), np.interp(J, a[:, 0], a[:, 2])]))
        r0, r1 = self.rpms[i - 1], self.rpms[i]
        w = 0.0 if r1 == r0 else (rpm - r0) / (r1 - r0)
        ct, cp = (1 - w) * out[0] + w * out[1]
        return float(ct), float(cp)


class Motor:
    """generic outrunner: kv [rpm/V], rm [ohm], i0 [A] at the test voltage"""
    def __init__(self, kv, rm, i0, name=None, mass=None, p_max=None):
        self.kv, self.rm, self.i0 = kv, rm, i0
        self.kv_rad = kv * 2 * math.pi / 60.0          # rad/s per V
        self.kt = 1.0 / self.kv_rad                     # N m / A
        self.name = name or f'{kv:.0f}kv'
        self.mass, self.p_max = mass, p_max


def generic_motor(kv):
    """a 41xx-50xx class outrunner rewound to kv: resistance scales with 1/kv^2, no-load current with kv.
    reference point kv 450: 0.056 ohm, 0.96 A (typical of that class). replace with the datasheet"""
    return Motor(kv, rm=0.056 * (450.0 / kv) ** 2, i0=0.96 * kv / 450.0, name=f'generic {kv:.0f} kv',
                 mass=0.30, p_max=1100.0)


# motors with datasheet numbers. i0 is the no-load current near the 6S operating speed
MOTORS = {
    # hacker A50-16S V4: 365 kv, 21 mohm, 1.5 A at 8.4 V (about 1.8 A at 6S speeds), 345 g, 1200 W for 15 s
    'hacker_a50_16s': Motor(365, rm=0.021, i0=1.8, name='Hacker A50-16S V4', mass=0.345, p_max=1200.0),
}


class Battery:
    def __init__(self, cells=6, v_cell=3.7, r_cell=0.002, capacity_ah=16.0, usable=0.8):
        self.cells, self.v_cell, self.r_cell = cells, v_cell, r_cell
        self.capacity_ah, self.usable = capacity_ah, usable

    @property
    def v_oc(self):
        return self.cells * self.v_cell

    @property
    def r(self):
        return self.cells * self.r_cell

    @property
    def energy_wh(self):
        return self.v_oc * self.capacity_ah


ETA_ESC = 0.97


def solve_motor(prop, mot, v_m, v_air):
    """steady rotor speed for motor voltage v_m and airspeed v_air -> dict (or None if it can't turn)"""
    if v_m <= 0.05:
        return None
    n_hi = v_m * mot.kv / 60.0                           # rev/s at no load
    def g(n):
        w = 2 * math.pi * n
        i = (v_m - w / mot.kv_rad) / mot.rm
        q_m = mot.kt * (i - mot.i0)
        J = v_air / (n * prop.D)
        ct, cp = prop.coeffs(J, n * 60)
        q_p = cp * RHO * n * n * prop.D ** 5 / (2 * math.pi)
        return q_m - q_p
    n_lo = max(1.0, v_air / (prop.D * 1.2) if v_air > 0 else 1.0)
    if g(n_lo) < 0:
        n_lo = 1.0
        if g(n_lo) < 0:
            return None
    if g(n_hi) > 0:
        return None
    for _ in range(80):
        nm = 0.5 * (n_lo + n_hi)
        if g(nm) > 0:
            n_lo = nm
        else:
            n_hi = nm
    n = 0.5 * (n_lo + n_hi)
    w = 2 * math.pi * n
    J = v_air / (n * prop.D)
    ct, cp = prop.coeffs(J, n * 60)
    i = (v_m - w / mot.kv_rad) / mot.rm
    return {'rpm': n * 60, 'omega': w, 'J': J, 'ct': ct, 'cp': cp, 'thrust': ct * RHO * n * n * prop.D ** 4,
            'torque': cp * RHO * n * n * prop.D ** 5 / (2 * math.pi), 'i_motor': i, 'p_motor_in': v_m * i,
            'p_shaft': cp * RHO * n ** 3 * prop.D ** 5}


def operating_point(prop, mot, bat, throttle, v_air, n_motors=2):
    """both motors at the same throttle on one pack, with pack sag solved by fixed point"""
    v_bus = bat.v_oc
    op = None
    for _ in range(6):
        op = solve_motor(prop, mot, throttle * v_bus, v_air)
        if op is None:
            return None
        i_batt = n_motors * op['p_motor_in'] / ETA_ESC / v_bus
        v_bus = bat.v_oc - i_batt * bat.r
    op = dict(op)
    op.update({'v_bus': v_bus, 'i_batt': n_motors * op['p_motor_in'] / ETA_ESC / v_bus,
               'p_batt': n_motors * op['p_motor_in'] / ETA_ESC, 'thrust_total': n_motors * op['thrust'],
               'eta_prop': op['thrust'] * v_air / op['p_shaft'] if op['p_shaft'] > 0 and v_air > 0 else 0.0})
    return op


def thrust_map(prop, mot, bat, speeds, throttles, n_motors=2):
    T = np.full((len(speeds), len(throttles)), np.nan)
    I = np.full_like(T, np.nan)
    P = np.full_like(T, np.nan)
    R = np.full_like(T, np.nan)
    for i, v in enumerate(speeds):
        for j, d in enumerate(throttles):
            op = operating_point(prop, mot, bat, d, v, n_motors)
            if op is None:
                T[i, j], I[i, j], P[i, j], R[i, j] = 0.0, 0.0, 0.0, 0.0
                continue
            T[i, j], I[i, j], P[i, j], R[i, j] = op['thrust_total'], op['i_batt'], op['p_batt'], op['rpm']
    return T, I, P, R


def fit_momentum(speeds, throttles, T, v_max=25.0, d_min=0.2):
    """least squares fit of the team's model T = a dT^2 - b dT V to a thrust map"""
    rows, rhs = [], []
    for i, v in enumerate(speeds):
        if v > v_max:
            continue
        for j, d in enumerate(throttles):
            if d < d_min or not np.isfinite(T[i, j]):
                continue
            rows.append([d * d, -d * v])
            rhs.append(T[i, j])
    A, y = np.array(rows), np.array(rhs)
    (a, b), *_ = np.linalg.lstsq(A, y, rcond=None)
    rms = float(np.sqrt(np.mean((A @ [a, b] - y) ** 2)))
    return float(a), float(b), rms
