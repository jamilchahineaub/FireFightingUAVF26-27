"""summarise a cl415 px4 sitl flight log against the predictions and plot it

    ~/px4_venv/bin/python /mnt/c/Users/jamil/Desktop/jim/uni/CL415-VIP/FireFightingUAVF26-27/cl415/px4_sitl/analyse_flight.py [log.ulg] [--hull hull_state.jsonl]
    (no argument: the newest log of the sitl instance). copies the log into px4_sitl/flight_logs/ and writes
    <name>_summary.json and <name>.png next to it. with --hull (default: ~/cl415_sim_logs/hull_state.jsonl if it
    exists) the hull plugin's state is added: water resistance over weight vs speed, draft, planing lift.
"""
import glob
import json
import math
import os
import shutil
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pyulog import ULog

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, 'flight_logs')
PRED = json.load(open(os.path.join(HERE, 'px4_model_summary.json')))
SERVO_DEG = {'ail': 25.0, 'ele': 25.0, 'rud': 30.0}
NAV = {0: 'MANUAL', 3: 'AUTO_MISSION', 4: 'AUTO_LOITER', 5: 'AUTO_RTL', 17: 'AUTO_TAKEOFF', 18: 'AUTO_LAND'}


def topic(u, name, multi=0):
    for d in u.data_list:
        if d.name == name and d.multi_id == multi:
            return d.data
    return None


def euler(q):
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def load_hull(path):
    """the hull plugin's Double_V stream as recorded by gz topic -e --json-output"""
    rows = []
    for line in open(path):
        line = line.strip()
        if not line.startswith('{'):
            continue
        try:
            rows.append(json.loads(line)['data'])
        except (ValueError, KeyError):
            continue
    if not rows:
        return None
    n = max(len(r) for r in rows)
    a = np.array([r for r in rows if len(r) == n])
    # several recorders may have written the same stream: sort by time and drop duplicates
    a = a[np.argsort(a[:, 0], kind='stable')]
    a = a[np.r_[True, np.diff(a[:, 0]) > 1e-6]]
    return {'t': a[:, 0], 'vol': a[:, 1], 'buoy': a[:, 2], 'resist': a[:, 3], 'plate_z': a[:, 4], 'heave': a[:, 5],
            'keel': a[:, 6], 'speed': a[:, 7], 'awp': a[:, 8], 'vz': a[:, 9]}


def main():
    args = [a for a in sys.argv[1:]]
    hull_path = None
    if '--hull' in args:
        i = args.index('--hull')
        hull_path = args[i + 1]
        del args[i:i + 2]
    elif os.path.exists(os.path.expanduser('~/cl415_sim_logs/hull_state.jsonl')):
        hull_path = os.path.expanduser('~/cl415_sim_logs/hull_state.jsonl')
    if args:
        path = args[0]
    else:
        logs = glob.glob(os.path.expanduser('~/PX4-Autopilot/build/px4_sitl_default/rootfs/log/*/*.ulg'))
        path = max(logs, key=os.path.getmtime)
    os.makedirs(OUT, exist_ok=True)
    name = os.path.basename(os.path.dirname(path)) + '_' + os.path.basename(path).replace('.ulg', '')
    if not path.startswith(OUT):
        shutil.copy(path, os.path.join(OUT, name + '.ulg'))
    u = ULog(path)
    t0 = u.start_timestamp

    def tt(d):
        return (d['timestamp'].astype(np.float64) - t0) / 1e6

    lp = topic(u, 'vehicle_local_position')
    att = topic(u, 'vehicle_attitude')
    atts = topic(u, 'vehicle_attitude_setpoint')
    asp = topic(u, 'airspeed_validated')
    tecs = topic(u, 'tecs_status')
    mot = topic(u, 'actuator_motors')
    srv = topic(u, 'actuator_servos')
    land = topic(u, 'vehicle_land_detected')
    st = topic(u, 'vehicle_status')

    t_lp = tt(lp)
    h = -lp['z']
    t_att = tt(att)
    eul = np.array([euler(q) for q in zip(att['q[0]'], att['q[1]'], att['q[2]'], att['q[3]'])])
    t_as = tt(asp)
    tas = asp['true_airspeed_m_s']
    t_m = tt(mot)
    thr = 0.5 * (mot['control[0]'] + mot['control[1]'])
    t_s = tt(srv)
    ele_deg = -srv['control[2]'] * SERVO_DEG['ele']           # + trailing edge down, as in the analysis
    ail_deg = srv['control[1]'] * SERVO_DEG['ail']            # right aileron, + trailing edge up
    rud_deg = srv['control[3]'] * SERVO_DEG['rud']

    # lift-off and touchdown from the simulator's true height (px4's land detector flips early and late)
    t_arm = tt(st)[np.argmax(st['arming_state'] == 2)] if np.any(st['arming_state'] == 2) else None
    gtp = topic(u, 'vehicle_local_position_groundtruth')
    src = gtp if gtp is not None else lp
    t_src, h_src = tt(src), -src['z']
    armed_mask = t_src >= (t_arm if t_arm is not None else t_src[0])
    h_gnd = float(np.median(h_src[armed_mask][:50])) if armed_mask.any() else 0.0
    agl_src = h_src - h_gnd
    agl = np.interp(t_lp, t_src, agl_src)
    # lift-off: last time the hull/wheels were within 10 cm of their rest height before the climb (a planing
    # hull rides 7 cm higher than at rest, so a smaller threshold would call the step the lift-off);
    # touchdown: first time back within 10 cm after the top of the flight
    CLEAR = 0.10
    up = np.where(armed_mask & (agl_src > 0.5))[0]
    t_lift = t_td = None
    if len(up):
        i_up = up[0]
        while i_up > 0 and agl_src[i_up] > CLEAR:
            i_up -= 1
        t_lift = float(t_src[i_up])
        i_top = int(np.argmax(np.where(armed_mask, agl_src, -1)))
        down = np.where((np.arange(len(agl_src)) > i_top) & (agl_src < CLEAR))[0]
        t_td = float(t_src[down[0]]) if len(down) else None

    out = {'log': name, 'duration_s': float(t_lp[-1] - t_lp[0])}
    if t_arm is not None and t_lift is not None:
        i0, i1 = np.searchsorted(t_lp, t_arm), np.searchsorted(t_lp, t_lift)
        roll_dist = math.hypot(lp['x'][i1] - lp['x'][i0], lp['y'][i1] - lp['y'][i0])
        out['takeoff'] = {'t_arm': float(t_arm), 't_liftoff': float(t_lift), 'ground_roll_m': roll_dist,
                          'liftoff_airspeed': float(np.interp(t_lift, t_as, tas)),
                          'ground_roll_time_s': float(t_lift - t_arm)}
    # cruise: level, near trim speed, wings roughly level, away from takeoff and landing
    if t_lift is not None:
        vz = np.interp(t_m, t_lp, lp['vz'])
        hh = np.interp(t_m, t_lp, h)
        aa = np.interp(t_m, t_as, tas)
        rr = np.interp(t_m, t_att, eul[:, 0])
        sel = (np.abs(vz) < 0.3) & (np.abs(aa - 20) < 1.0) & (hh - h_gnd > 30) & (np.abs(rr) < math.radians(5)) & np.isfinite(thr)
        if sel.sum() > 20:
            el = np.interp(t_m[sel], t_s, ele_deg)
            pi = np.interp(t_m[sel], t_att, eul[:, 1])
            out['cruise'] = {'samples': int(sel.sum()), 'throttle': float(np.median(thr[sel])),
                             'throttle_predicted': PRED['propulsion']['cruise_throttle_full'],
                             'elevator_deg_TE_down': float(np.median(el)), 'pitch_deg': float(np.degrees(np.median(pi))),
                             'airspeed': float(np.median(aa[sel]))}
    gpos = topic(u, 'vehicle_global_position')
    if tecs is not None and gpos is not None:
        t_t = tt(tecs)
        fl = (t_t > (t_lift or 0) + 15) & (t_t < (t_td or t_t[-1]) - 30)
        if fl.any():
            alt = np.interp(t_t[fl], tt(gpos), gpos['alt'])
            out['tracking'] = {
                'alt_err_rms_m': float(np.sqrt(np.mean((tecs['altitude_sp'][fl] - alt) ** 2))),
                'alt_err_ref_rms_m': float(np.sqrt(np.mean((tecs['altitude_reference'][fl] - alt) ** 2))),
                'airspeed_err_rms': float(np.sqrt(np.mean((tecs['true_airspeed_sp'][fl] - tecs['true_airspeed_filtered'][fl]) ** 2))),
            }
    if t_td is not None:
        i = np.searchsorted(t_lp, t_td)
        j = np.searchsorted(t_src, t_td)
        out['touchdown'] = {'t': float(t_td), 'vz_mps': float(np.max(src['vz'][max(0, j - 10):j + 1])),
                            'rollout_m': float(math.hypot(src['x'][-1] - src['x'][j], src['y'][-1] - src['y'][j])),
                            'from_liftoff_s': float(t_td - t_lift) if t_lift is not None else None,
                            'x_north': float(lp['x'][i]), 'y_east': float(lp['y'][i]),
                            'airspeed': float(np.interp(t_td, t_as, tas)),
                            'pitch_deg': float(np.degrees(np.interp(t_td, t_att, eul[:, 1])))}
    gt = topic(u, 'vehicle_attitude_groundtruth')
    if gt is not None:
        eg = np.array([euler(q) for q in zip(gt['q[0]'], gt['q[1]'], gt['q[2]'], gt['q[3]'])])
        tg = tt(gt)
        keep = tg > (t_arm if t_arm is not None else tg[0])
        tg, eg = tg[keep], eg[keep]
        err = np.degrees(np.array([np.interp(tg, t_att, eul[:, k]) - eg[:, k] for k in range(3)]))
        err[2] = (err[2] + 180) % 360 - 180
        out['ekf_attitude_error_deg'] = {ax: {'rms': float(np.sqrt(np.mean(e ** 2))), 'max': float(np.max(np.abs(e)))}
                                         for ax, e in zip(('roll', 'pitch', 'yaw'), err)}
    # the hull plugin's own numbers: gz sim time vs px4 log time are offset, line them up on lift-off
    # (first time the hull is dry) and touchdown (first time it is wet again)
    hull = load_hull(hull_path) if hull_path and os.path.exists(hull_path) else None
    if hull is not None and t_lift is not None:
        W = 10.5 * 9.81
        wet = hull['vol'] > 1e-6
        dry_i = np.where(~wet)[0]
        if len(dry_i):
            t_dry = hull['t'][dry_i[0]]
            shift = t_lift - t_dry
            hull['t_px4'] = hull['t'] + shift
            run = (hull['t_px4'] > t_arm) & (hull['t_px4'] <= t_lift) if t_arm is not None else (hull['t_px4'] <= t_lift)
            if run.any():
                rw = hull['resist'][run] / W
                k = int(np.argmax(rw))
                out['water_takeoff'] = {'hump_R_over_W': float(rw[k]), 'hump_speed': float(hull['speed'][run][k]),
                                        'hump_buoyancy_over_W': float(hull['buoy'][run][k] / W),
                                        'max_planing_lift_over_W': float(np.max(hull['plate_z'][run]) / W),
                                        'draft_at_rest_m': float(np.median(hull['keel'][:50])),
                                        'resistance_curve_V_RoverW': [[float(v), float(r)] for v, r in
                                                                      zip(np.round(hull['speed'][run][::25], 2), np.round(rw[::25], 3))]}
            if t_td is not None:
                rewet = np.where(wet & (hull['t_px4'] > t_lift + 5))[0]
                if len(rewet):
                    j = rewet[0]
                    out['water_landing'] = {'touch_speed': float(hull['speed'][j]),
                                            'touch_vz': float(hull['vz'][j]),
                                            'peak_buoyancy_over_W_first_2s': float(np.max(hull['buoy'][j:j + 100]) / W),
                                            'max_draft_after_touch_m': float(np.max(hull['keel'][j:j + 500])),
                                            'stop_time_s': float(hull['t'][j + np.argmax(hull['speed'][j:] < 1.0)] - hull['t'][j])
                                            if np.any(hull['speed'][j:] < 1.0) else None}
    out['max_roll_deg'] = float(np.degrees(np.max(np.abs(eul[:, 0]))))
    out['max_airspeed'] = float(np.max(tas))
    out['nav_states'] = sorted({NAV.get(int(n), str(int(n))) for n in st['nav_state']})
    json.dump(out, open(os.path.join(OUT, name + '_summary.json'), 'w'), indent=1)
    short = json.loads(json.dumps(out))
    if 'water_takeoff' in short:
        short['water_takeoff'].pop('resistance_curve_V_RoverW', None)
    print(json.dumps(short, indent=1))

    nrow = 4 if hull is not None and 't_px4' in hull else 3
    fig, ax = plt.subplots(nrow, 2, figsize=(15, 11 if nrow == 3 else 14))
    a = ax[0, 0]
    a.plot(lp['y'], lp['x'], lw=1)
    a.set_xlabel('east [m]')
    a.set_ylabel('north [m]')
    a.set_aspect('equal')
    a.set_title('ground track')
    a.grid(True, lw=0.3)
    a = ax[0, 1]
    a.plot(t_lp, agl, label='height above runway')
    if tecs is not None and gpos is not None:
        a.plot(tt(tecs), tecs['altitude_sp'] - (gpos['alt'][0] + lp['z'][0]) - h_gnd, '--', label='setpoint')
    a.set_ylabel('m')
    a.legend(fontsize=8)
    a.grid(True, lw=0.3)
    a.set_title('altitude')
    a = ax[1, 0]
    a.plot(t_as, tas, label='true airspeed')
    if tecs is not None:
        a.plot(tt(tecs), tecs['true_airspeed_sp'], '--', label='setpoint')
    a.axhline(PRED['px4']['stall'], color='r', ls=':', lw=0.8, label='stall (full)')
    a.set_ylabel('m/s')
    a.legend(fontsize=8)
    a.grid(True, lw=0.3)
    a.set_title('airspeed')
    a = ax[1, 1]
    a.plot(t_att, np.degrees(eul[:, 0]), label='roll')
    a.plot(t_att, np.degrees(eul[:, 1]), label='pitch')
    if atts is not None:
        es = np.array([euler(q) for q in zip(atts['q_d[0]'], atts['q_d[1]'], atts['q_d[2]'], atts['q_d[3]'])])
        a.plot(tt(atts), np.degrees(es[:, 0]), ':', lw=0.8, label='roll sp')
        a.plot(tt(atts), np.degrees(es[:, 1]), ':', lw=0.8, label='pitch sp')
    a.set_ylabel('deg')
    a.legend(fontsize=8, ncol=2)
    a.grid(True, lw=0.3)
    a.set_title('attitude')
    a = ax[2, 0]
    a.plot(t_m, thr, label='throttle')
    a.axhline(PRED['propulsion']['cruise_throttle_full'], color='k', ls=':', lw=0.8, label='predicted cruise')
    a.set_ylim(-0.05, 1.05)
    a.legend(fontsize=8)
    a.grid(True, lw=0.3)
    a.set_title('throttle')
    a = ax[2, 1]
    a.plot(t_s, ele_deg, label='elevator (+TE down)')
    a.plot(t_s, ail_deg, label='right aileron (+TE up)')
    a.plot(t_s, rud_deg, label='rudder (+TE right)')
    a.set_ylabel('deg')
    a.legend(fontsize=8)
    a.grid(True, lw=0.3)
    a.set_title('control surfaces')
    if nrow == 4:
        a = ax[3, 0]
        W = 10.5 * 9.81
        a.plot(hull['t_px4'], hull['resist'] / W, label='water resistance / W')
        a.plot(hull['t_px4'], hull['buoy'] / W, label='buoyancy / W')
        a.plot(hull['t_px4'], hull['plate_z'] / W, label='planing lift / W')
        a.set_ylim(-0.1, 1.3)
        a.legend(fontsize=8)
        a.grid(True, lw=0.3)
        a.set_title('hull forces')
        a = ax[3, 1]
        a.plot(hull['t_px4'], hull['keel'] * 1000, label='keel depth below water')
        a.axhline(0, color='k', lw=0.6)
        a.set_ylim(-150, 150)          # the flight phase is off the scale, the water phases are what matter
        a.set_ylabel('mm (+ = below the surface)')
        a.legend(fontsize=8)
        a.grid(True, lw=0.3)
        a.set_title('draft')
        for a in ax[3, :]:
            a.set_xlim(ax[1, 0].get_xlim())
    for a in ax[1:, :].ravel().tolist() + [ax[0, 1]]:
        a.set_xlabel('t [s]')
        for tm in (t_lift, t_td):
            if tm is not None:
                a.axvline(tm, color='grey', lw=0.6)
    fig.suptitle(f'CL-415 PX4 SITL {name}')
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, name + '.png'), dpi=110)
    print('wrote', os.path.join(OUT, name + '.png'))


if __name__ == '__main__':
    main()
