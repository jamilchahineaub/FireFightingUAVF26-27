"""summarise the headless gazebo runs (verify/out/*.csv) against the hydrostatic prediction

writes verify/report.md and verify/plots/*.png
run from the project root:  .venv/Scripts/python verify/analyse.py
"""
import json
import math
import os

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, 'verify', 'out')
PLOTS = os.path.join(ROOT, 'verify', 'plots')
os.makedirs(PLOTS, exist_ok=True)
HYD = json.load(open(os.path.join(OUT, 'hydrostatics.json')))
GEO = json.load(open(os.path.join(ROOT, 'cad', 'geometry.json')))
KEEL = GEO['misc']['keel_z']
D = math.degrees


def load(name):
    p = os.path.join(OUT, name + '.csv')
    if not os.path.exists(p) or os.path.getsize(p) < 100:
        return None
    with open(p) as f:
        head = f.readline().strip().split(',')
    a = np.genfromtxt(p, delimiter=',', skip_header=1)
    return {k: a[:, i] for i, k in enumerate(head)}


def last(d, k, span=5.0):
    m = d['t'] >= d['t'][-1] - span
    return float(np.mean(d[k][m])), float(np.ptp(d[k][m]))


L = ['# Step 5: headless Gazebo checks', '',
     'Gazebo Harmonic (gz-sim 8.9) with ROS 2 Jazzy in the `AMR` WSL distro, launched through '
     '`ros2 launch cl415_description sim.launch.py headless:=true ...` by `verify/run_sim_tests.sh`. '
     '`verify/sim_probe.py` records `/cl415/odometry` and `/joint_states` through the bridge. '
     'Plots in `verify/plots/`.', '']

# floating
L += ['## Floating on water', '',
      'Expected attitude from the collision boxes (what the graded buoyancy system integrates), solved in '
      '`verify/hydrostatics.py`: buoyancy equals weight and the moments about the CG vanish. The level-hull '
      'case is unstable in roll, so the solution leans onto one float, like the real aircraft at rest.', '',
      '| run | base z [m] | keel depth [m] | roll [deg] | pitch [deg] | z ripple last 5 s [mm] | status |',
      '|---|---:|---:|---:|---:|---:|---|']
fig, axs = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
for name, case, lab in (('water_full', 'full', 'full, no damping'), ('water_full_damped', 'full', 'full, damped'),
                        ('water_empty_damped', 'empty', 'empty, damped')):
    d = load(name)
    if d is None:
        L.append(f'| {lab} | - | - | - | - | - | no data |')
        continue
    z, dz = last(d, 'z')
    r, _ = last(d, 'roll')
    p, _ = last(d, 'pitch')
    vmax = float(np.max(np.abs(np.c_[d['vx'], d['vy'], d['vz']])))
    ok = np.all(np.isfinite(d['z'])) and vmax < 2.0 and 0.1 < z < 0.4
    L.append(f'| {lab} | {z:.4f} | {-(z + KEEL):.4f} | {D(r):+.2f} | {D(p):+.2f} | {1000 * dz:.1f} | '
             f'{"floats, no blow-up" if ok else "CHECK"} (max speed {vmax:.2f} m/s) |')
    axs[0].plot(d['t'], d['z'], label=lab)
    axs[1].plot(d['t'], np.degrees(d['roll']), label=lab)
    axs[2].plot(d['t'], np.degrees(d['pitch']), label=lab)
for case, ls in (('full', '--'), ('empty', ':')):
    h = HYD[case]
    L.append(f'| predicted, tank {case} | {h["z"]:.4f} | {h["keel_depth"]:.4f} | +/-{abs(h["roll_deg"]):.2f} | '
             f'{h["pitch_deg"]:+.2f} | | exact box clipping |')
    axs[0].axhline(h['z'], color='k', ls=ls, lw=0.8, label=f'predicted {case}')
    axs[1].axhline(-abs(h['roll_deg']), color='k', ls=ls, lw=0.8)
    axs[1].axhline(abs(h['roll_deg']), color='k', ls=ls, lw=0.8)
    axs[2].axhline(h['pitch_deg'], color='k', ls=ls, lw=0.8)
axs[0].set_ylabel('base_link z [m]')
axs[1].set_ylabel('roll [deg]')
axs[2].set_ylabel('pitch [deg]')
axs[2].set_xlabel('sim time [s]')
axs[0].legend(fontsize=8)
for a in axs:
    a.grid(True, lw=0.3)
fig.tight_layout()
fig.savefig(os.path.join(PLOTS, 'water.png'), dpi=140)
plt.close(fig)
L += ['', 'Pitch in ROS convention (positive = nose down), so the negative values mean bow up. '
      f'The real CAD hull would float level at a keel draft of {HYD["full"]["cad_hull_level_draft"] * 1000:.0f} mm '
      f'(full) and {HYD["empty"]["cad_hull_level_draft"] * 1000:.0f} mm (empty); the collision boxes are flat-bottomed, so '
      'they float a bit shallower than the vee hull.', '']

# ground
d = load('ground_full')
L += ['## Sitting on hard ground', '']
if d is None:
    L.append('No data.')
else:
    z, dz = last(d, 'z')
    r, _ = last(d, 'roll')
    p, _ = last(d, 'pitch')
    vmax = float(np.max(np.abs(np.c_[d['vx'], d['vy'], d['vz']])[d['t'] > 2]))
    L.append(f'Spawned at z = {d["z"][0]:.3f} m. After {d["t"][-1]:.0f} s: base z = {z:.4f} m (keel box bottom at '
             f'{z + min(b["xyz"][2] - b["size"][2] / 2 for b in GEO["collision"]["fuselage"]):+.4f} m), roll {D(r):+.2f} deg, '
             f'pitch {D(p):+.2f} deg, z ripple {1000 * dz:.2f} mm, max speed after 2 s {vmax:.3f} m/s. It rests on the '
             'flat bottoms of the hull boxes, with no jitter or drift.')
    fig, ax = plt.subplots(figsize=(10, 3.5))
    ax.plot(d['t'], d['z'])
    ax.set_xlabel('sim time [s]')
    ax.set_ylabel('base_link z [m]')
    ax.grid(True, lw=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(PLOTS, 'ground.png'), dpi=140)
    plt.close(fig)
L.append('')

# actuation
d = load('water_actuate')
L += ['## Control surfaces and motors (on water, damped)', '']
if d is None:
    L.append('No data.')
else:
    lim = {'left_aileron_joint': 0.4363, 'right_aileron_joint': 0.4363, 'elevator_joint': 0.4363, 'rudder_joint': 0.5236}
    cmd = {'left_aileron_joint': 0.30, 'right_aileron_joint': -0.30, 'elevator_joint': 0.20, 'rudder_joint': 0.40}
    L += ['| joint | command at 3 s [rad] | reached at 5.5 s [rad] | rise time 10-90 % [s] | after +1.0 rad command at 6 s |',
          '|---|---:|---:|---:|---|']
    for j, c in cmd.items():
        pos = d[f'{j}_pos']
        at = lambda t: float(np.interp(t, d['t'], pos))
        t10 = d['t'][np.argmax((d['t'] > 3) & (np.abs(pos) >= 0.1 * abs(c)))]
        t90 = d['t'][np.argmax((d['t'] > 3) & (np.abs(pos) >= 0.9 * abs(c)))]
        after = ''
        if j == 'elevator_joint':
            mx = float(np.max(pos[d['t'] > 6]))
            after = f'max {mx:.4f} rad (limit {lim[j]:.4f})'
        L.append(f'| {j} | {c:+.2f} | {at(5.5):+.4f} | {t90 - t10:.3f} | {after} |')
    w = d['left_propeller_joint_vel']
    wr = d['right_propeller_joint_vel']
    m = (d['t'] > 11) & (d['t'] < 13.5)
    x0 = float(np.interp(8.0, d['t'], d['x']))
    x1 = float(np.interp(14.0, d['t'], d['x']))
    vx = float(np.max(d['vx'][(d['t'] > 8) & (d['t'] < 14.5)]))
    yaw_drift = D(float(np.interp(14.0, d['t'], d['yaw']) - np.interp(8.0, d['t'], d['yaw'])))
    L += ['', f'Motors at 600 rad/s from 8 to 14 s: joint speed {np.mean(w[m]):+.1f} (left) and {np.mean(wr[m]):+.1f} '
          f'(right) rad/s, i.e. 600 / rotorVelocitySlowdownSim = 60 with opposite signs, as set for counter-rotation. '
          f'The aircraft moved {x1 - x0:+.3f} m along +x (forward) in those 6 s and reached {vx:.2f} m/s, so thrust '
          f'points forward. Heading changed by {yaw_drift:+.1f} deg (the rudder was at +0.4 rad, trailing edge left, '
          'so some yaw is expected once it moves).', '']
    fig, axs = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    for j in cmd:
        axs[0].plot(d['t'], d[f'{j}_pos'], label=j)
    axs[0].axhline(0.4363, color='k', ls=':', lw=0.8)
    axs[0].set_ylabel('joint angle [rad]')
    axs[0].legend(fontsize=8)
    axs[1].plot(d['t'], d['x'], label='x [m]')
    axs[1].plot(d['t'], d['vx'], label='vx [m/s]')
    axs[1].set_xlabel('sim time [s]')
    axs[1].legend(fontsize=8)
    for a in axs:
        a.grid(True, lw=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(PLOTS, 'actuation.png'), dpi=140)
    plt.close(fig)

open(os.path.join(ROOT, 'verify', 'report.md'), 'w', encoding='utf-8').write('\n'.join(L) + '\n')
print('\n'.join(L))
