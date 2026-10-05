"""evaluate one motor + prop + battery for the cl415 and print every number the sims need

    .venv/Scripts/python flight_params/propulsion/evaluate.py --prop 14x7E --kv 450 --rm 0.056 --i0 0.96 --cells 6
    .venv/Scripts/python flight_params/propulsion/evaluate.py --prop 13x10E --kv 380          (generic rm, i0)
    .venv/Scripts/python flight_params/propulsion/evaluate.py --prop 155x12-4 --scale-to 13.5 --motor hacker_a50_16s

--prop is an APC file name in apc/ (download more from https://www.apcprop.com/files/PER3_<name>.dat).
--rm [ohm] and --i0 [A] come from the motor datasheet; leave them out for the generic 41xx-50xx class.
writes maps/<tag>_*.csv (thrust, battery current, battery power, rpm over airspeed x throttle) and prints:
requirement check, the team's momentum-model fit (params.yaml), Gazebo MulticopterMotorModel values,
and PX4 throttle trim.
"""
import argparse
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import prop_tools as PT  # noqa: E402

DATA = json.load(open(os.path.join(os.path.dirname(HERE), 'cl415_flight_data.json')))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--prop', required=True)
    ap.add_argument('--kv', type=float)
    ap.add_argument('--motor', choices=sorted(PT.MOTORS), help='datasheet motor instead of --kv')
    ap.add_argument('--scale-to', type=float, help='geometrically scale the prop to this diameter [in]')
    ap.add_argument('--rm', type=float)
    ap.add_argument('--i0', type=float)
    ap.add_argument('--cells', type=int, default=6)
    ap.add_argument('--capacity', type=float, default=16.0, help='battery capacity [Ah]')
    ap.add_argument('--i_max', type=float, default=55.0, help='continuous current limit per motor/ESC [A]')
    a = ap.parse_args()

    pr = PT.Prop(os.path.join(HERE, 'apc', f'PER3_{a.prop}.dat'))
    if a.scale_to:
        pr = pr.scaled(a.scale_to)
    if a.motor:
        import copy
        mot = copy.copy(PT.MOTORS[a.motor])
        a.kv = mot.kv
    elif a.kv:
        mot = PT.generic_motor(a.kv)
    else:
        ap.error('give --kv or --motor')
    if a.rm is not None:
        mot.rm = a.rm
    if a.i0 is not None:
        mot.i0 = a.i0
    bat = PT.Battery(cells=a.cells, capacity_ah=a.capacity)
    R = DATA['propulsion']['requirements']
    aero = DATA['aero']
    S, m_f, m_e = DATA['geometry']['S'], DATA['mass']['full']['m'], DATA['mass']['empty']['m']

    def drag(V, m):
        q = 0.5 * 1.225 * V * V
        CL = m * 9.81 / (q * S)
        return q * S * (aero['CD0'] + aero['K'] * CL * CL)

    st = PT.operating_point(pr, mot, bat, 1.0, 0.0)
    hump_v = 0.5 * 1.15 * R['full']['V_stall']
    hump = PT.operating_point(pr, mot, bat, 1.0, hump_v)
    climb = PT.operating_point(pr, mot, bat, 1.0, R['full']['V_climb'])

    def cruise(m):
        need = drag(20.0, m)
        lo, hi, op = 0.0, 1.0, None
        for _ in range(40):
            mid = 0.5 * (lo + hi)
            o = PT.operating_point(pr, mot, bat, mid, 20.0)
            if o is None or o['thrust_total'] < need:
                lo = mid
            else:
                hi, op = mid, o
        return hi, op

    thr_f, cr_f = cruise(m_f)
    thr_e, cr_e = cruise(m_e)
    speeds = np.arange(0.0, 31.0, 1.0)
    throttles = np.round(np.arange(0.0, 1.0001, 0.05), 2)
    T, I, P, RPM = PT.thrust_map(pr, mot, bat, speeds, throttles)
    tag = f'{pr.d_in:g}x{pr.pitch_in:.3g}-{pr.blades}_{a.motor or f"{a.kv:.0f}kv"}_{a.cells}S'
    os.makedirs(os.path.join(HERE, 'maps'), exist_ok=True)
    hdr = 'V_mps\\throttle,' + ','.join(f'{d:.2f}' for d in throttles)
    for arr, nm in ((T, 'thrust_N'), (I, 'battery_current_A'), (P, 'battery_power_W'), (RPM, 'rpm')):
        np.savetxt(os.path.join(HERE, 'maps', f'{tag}_{nm}.csv'), np.c_[speeds, arr], delimiter=',', fmt='%.3f',
                   header=hdr, comments='')
    fa, fb, rms = PT.fit_momentum(speeds, throttles, T)
    ct0, cp0 = pr.coeffs(0.0, st['rpm'])
    w_max = st['rpm'] * math.pi / 30
    usable = bat.energy_wh * bat.usable

    ok = lambda c: 'ok' if c else 'FAIL'
    print(f'== {tag}: {pr.name} ({pr.D:.4f} m), {mot.name}, {a.kv:.0f} kV, Rm {mot.rm:.4f} ohm, I0 {mot.i0:.2f} A, {a.cells}S {a.capacity:.0f} Ah')
    clr = 0.1978 - pr.D / 2                 # prop axis to hull corner, current nacelle position (cad)
    print(f'tip to hull clearance {clr * 1000:.0f} mm ({"ok" if clr >= 0.015 else "FAIL, move the nacelles out " + format((0.015 - clr) * 1000, ".0f") + " mm"})')
    print(f'static, full throttle: {st["thrust_total"]:.1f} N total ({st["thrust_total"] / (m_f * 9.81):.2f} T/W full), '
          f'{st["rpm"]:.0f} rpm (APC limit {pr.rpm_limit:.0f}: {ok(st["rpm"] <= pr.rpm_limit)}), '
          f'{st["i_motor"]:.1f} A per motor ({ok(st["i_motor"] <= a.i_max)}), {st["v_bus"] * st["i_motor"]:.0f} W per motor'
          f'{" (" + ok(st["v_bus"] * st["i_motor"] <= mot.p_max) + f" vs {mot.p_max:.0f} W)" if mot.p_max else ""}, {st["p_batt"]:.0f} W from the pack')
    for thr in (0.5, 0.75):
        o = PT.operating_point(pr, mot, bat, thr, 0.0)
        print(f'  static at {thr:.2f} throttle: {o["thrust_total"]:.1f} N total, {o["i_motor"]:.1f} A per motor')
    print(f'water take-off hump at {hump_v:.1f} m/s: {hump["thrust_total"]:.1f} N available, '
          f'{R["full"]["T_hump"]:.1f} N needed (+15 %): {ok(hump["thrust_total"] >= 1.15 * R["full"]["T_hump"])}')
    print(f'climb {R["full"]["V_climb"]:.1f} m/s at 2.5 m/s: {climb["thrust_total"]:.1f} N available, '
          f'{R["full"]["T_climb"]:.1f} N needed: {ok(climb["thrust_total"] >= R["full"]["T_climb"])}')
    for nm, thr, cr in (('full', thr_f, cr_f), ('empty', thr_e, cr_e)):
        if cr is None:
            print(f'cruise 20 m/s {nm}: not reachable')
            continue
        print(f'cruise 20 m/s {nm}: throttle {thr:.3f}, {cr["thrust_total"]:.2f} N, {cr["rpm"]:.0f} rpm, '
              f'{cr["i_batt"]:.1f} A / {cr["p_batt"]:.0f} W from the pack, prop eta {cr["eta_prop"]:.2f}, '
              f'endurance {usable / cr["p_batt"] * 60:.0f} min on {usable:.0f} Wh usable')
    print('\nteam params.yaml (propulsion.momentum): T = a dT^2 - b dT V, both motors')
    print(f'  T_static_total: {fa:.3f}\n  b: {fb:.4f}          # fit rms {rms:.2f} N over V 0-25 m/s, throttle 0.2-1')
    print(f'  dT_anchor: {thr_f:.4f}     # cruise throttle, full tank\n  P_elec_max_total: {st["p_batt"]:.0f}')
    print('\nGazebo MulticopterMotorModel (per motor, static thrust model):')
    print(f'  motorConstant: {st["thrust"] / w_max ** 2:.4e}\n  momentConstant: {cp0 / (2 * math.pi) * pr.D / ct0:.4f}')
    print(f'  maxRotVelocity: {w_max:.1f}       # rad/s, full throttle static')
    print(f'\nPX4: FW_THR_TRIM {thr_f:.2f} (full tank, 20 m/s), {thr_e:.2f} empty')
    print(f'\nmaps written to propulsion/maps/{tag}_*.csv')


if __name__ == '__main__':
    main()
