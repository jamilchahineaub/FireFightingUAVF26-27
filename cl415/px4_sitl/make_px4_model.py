"""cl415 for px4 sitl: gz models, worlds and px4 airframes, all from the project numbers

reads  cl415_description/models/cl415_full/model.sdf      (links, meshes, collision boxes, servos from the cad export)
       flight_params/cl415_flight_data.json               (AVL / XFOIL aero, mass)
       flight_params/propulsion/apc + prop_tools          (motor + prop maps)
writes px4_sitl/models/cl415_px4/        runway variant: sim-only tricycle gear
       px4_sitl/models/cl415_px4_water/  water variant: flying-boat hull plugin, no gear
       px4_sitl/worlds/cl415_runway.sdf, cl415_water.sdf
       px4_sitl/airframes/4050_gz_cl415, 4051_gz_cl415_water
       px4_sitl/px4_model_summary.json

changes against cl415_full:
  - px4 sensors on base_link (imu_sensor, air_pressure_sensor, navsat_sensor) and an airspeed_link pitot, named
    the way px4's gz_bridge subscribes to them; the magnetometer comes from libCl415Magnetometer.so because
    harmonic's magnetometer sensor is wrong in banked flight
  - one AdvancedLiftDrag for the whole aircraft from the AVL derivatives (moments about the full-tank CG)
    instead of the four hand-estimated LiftDrag elements
  - props driven by libCl415PropellerMap.so (motor + prop maps vs airspeed) instead of the static
    k w^2 MulticopterMotorModel
  - servo topics /model/cl415/servo_0..3 (left aileron, right aileron, elevator, rudder)
  - water variant: libCl415FlyingBoatHull.so on the hull and float collision boxes (exact buoyancy, planing,
    water drag, heave damping)

run from the project root:  .venv/Scripts/python px4_sitl/make_px4_model.py
"""
import copy
import json
import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, 'flight_params', 'propulsion'))
import prop_tools as PT  # noqa: E402

D = json.load(open(os.path.join(ROOT, 'flight_params', 'cl415_flight_data.json')))
G = json.load(open(os.path.join(ROOT, 'cad', 'geometry.json')))
SRC = os.path.join(ROOT, 'cl415_description', 'models', 'cl415_full', 'model.sdf')
MODEL_NAME = 'cl415'
D2R = math.pi / 180

# propulsion baseline: hacker A50-16S on 6S with a 13.5 in 4-blade (APC 15.5x12-4 scaled to scale size)
PROP_FILE, PROP_SCALE_IN, MOTOR = 'PER3_155x12-4.dat', 13.5, 'hacker_a50_16s'
CMD_FULL = 1000.0                       # px4 SIM_GZ_EC_MAX, the command that means full throttle
SERVO_MAX_DEG = {'ail': 25.0, 'ele': 25.0, 'rud': 30.0}
# earth field at the world origin as px4's ekf expects it (its WMM: 0.4824 G, inclination 63.16 deg, ~3.3 deg east)
_H, _I, _D = 0.4824 * math.cos(math.radians(63.1575)), math.radians(63.1575), math.radians(3.3)
MAG_NED = (_H * math.cos(_D), _H * math.sin(_D), 0.4824 * math.sin(_I))
LAT, LON = 47.397971057728974, 8.546163739800146

geo, aero, mass = D['geometry'], D['aero'], D['mass']
F = aero['about_full_cg']
S, B, C = geo['S'], geo['b'], geo['c_bar']
AR = B * B / S
cg = np.array(mass['full']['cg_flu'])
V_TRIM = 20.0


# ---------------------------------------------------------------- AdvancedLiftDrag stall shape
# the plugin blends linear and flat-plate lift with a sigmoid of fixed sharpness M = 15, wide enough to eat
# 15 % of the lift at cruise if alpha_stall sat at the real 7.4 deg. fit CL0, CLa and alpha_stall so the
# curve matches AVL from -5 to 6 deg body alpha and peaks at our CL_max.
def ald_sigma(a, a_s, M=15.0):
    return (1 + np.exp(-M * (a - a_s)) + np.exp(M * (a + a_s))) / ((1 + np.exp(-M * (a - a_s))) * (1 + np.exp(M * (a + a_s))))


def ald_cl(a, cl0, cla, a_s):
    s = ald_sigma(a, a_s)
    return (1 - s) * (cl0 + cla * a) + s * 2 * np.sign(a) * np.sin(a) ** 2 * np.cos(a)


al_fit = np.radians(np.linspace(-5, 6, 60))
target = F['CL0'] + F['CL_alpha'] * al_fit
al_peak = np.radians(np.linspace(0, 35, 701))
best = None
for a_s in np.radians(np.arange(8.0, 30.0, 0.05)):
    sg = ald_sigma(al_fit, a_s)
    post = 2 * np.sign(al_fit) * np.sin(al_fit) ** 2 * np.cos(al_fit)
    A = np.c_[(1 - sg), (1 - sg) * al_fit]
    (cl0, cla), *_ = np.linalg.lstsq(A, target - sg * post, rcond=None)
    peak = ald_cl(al_peak, cl0, cla, a_s).max()
    err = abs(peak - aero['CL_max'])
    if best is None or err < best[0]:
        best = (err, a_s, cl0, cla, peak)
_, A_STALL, CL0_ALD, CLA_ALD, CL_PEAK = best
fit_rms = float(np.sqrt(np.mean((ald_cl(al_fit, CL0_ALD, CLA_ALD, A_STALL) - target) ** 2)))
AL_PEAK = float(al_peak[np.argmax(ald_cl(al_peak, CL0_ALD, CLA_ALD, A_STALL))])

# ---------------------------------------------------------------- control surfaces, per joint degree
# joints: ailerons and elevator + = trailing edge down, rudder + = trailing edge left (the AVL signs).
# antisymmetric aileron split: delta_a = +1 means left TE down 1, right TE up 1.
y_ail = 0.5 * sum(geo['aileron_y'])
cl_one_ail = F['Cl_da'] * B / (2 * y_ail)              # lift of one aileron per rad, from the roll power
x_ail_arm = (cg[0] - (-0.237)) / C                      # aileron lift sits behind the cg
ctrl = {
    'left_aileron_joint': dict(CD=0.0, CY=F['CY_da'] / 2, CL=cl_one_ail, Cell=F['Cl_da'] / 2,
                               Cem=-cl_one_ail * x_ail_arm, Cen=F['Cn_da'] / 2),
    'right_aileron_joint': dict(CD=0.0, CY=-F['CY_da'] / 2, CL=cl_one_ail, Cell=-F['Cl_da'] / 2,
                                Cem=-cl_one_ail * x_ail_arm, Cen=-F['Cn_da'] / 2),
    'elevator_joint': dict(CD=0.0, CY=0.0, CL=F['CL_de'], Cell=0.0, Cem=F['Cm_de'], Cen=0.0),
    'rudder_joint': dict(CD=0.0, CY=F['CY_dr'], CL=0.0, Cell=F['Cl_dr'], Cem=0.0, Cen=F['Cn_dr']),
}


def f(x, n=6):
    return f'{x:.{n}g}'


def sub(parent, tag, text=None, **attr):
    e = ET.SubElement(parent, tag, attr)
    if text is not None:
        e.text = str(text)
    return e


def ald_plugin(model):
    p = sub(model, 'plugin', filename='gz-sim-advanced-lift-drag-system', name='gz::sim::systems::AdvancedLiftDrag')
    vals = [
        ('link_name', 'base_link'), ('air_density', 1.225), ('area', f(S)), ('AR', f(AR)), ('mac', f(C)),
        ('eff', f(aero['oswald_e'])), ('cp', f'{cg[0]:.5f} {cg[1]:.5f} {cg[2]:.5f}'),
        ('forward', '1 0 0'), ('upward', '0 0 1'),
        ('CL0', f(CL0_ALD)), ('CLa', f(CLA_ALD)), ('alpha_stall', f(A_STALL)), ('CD0', f(aero['CD0'])),
        ('Cem0', f(F['Cm0'])), ('Cema', f(F['Cm_alpha'])), ('Cema_stall', f(F['Cm_alpha'])),
        ('CYa', 0), ('Cella', 0), ('Cena', 0),
        ('CLb', f(F['CL_beta'])), ('CYb', f(F['CY_beta'])), ('Cellb', f(F['Cl_beta'])),
        ('Cemb', f(F['Cm_beta'])), ('Cenb', f(F['Cn_beta'])),
        ('CDp', 0), ('CYp', f(F['CY_p'])), ('CLp', 0), ('Cellp', f(F['Cl_p'])), ('Cemp', f(F['Cm_p'])), ('Cenp', f(F['Cn_p'])),
        ('CDq', f(F['CD_q'])), ('CYq', 0), ('CLq', f(F['CL_q'])), ('Cellq', 0), ('Cemq', f(F['Cm_q'])), ('Cenq', 0),
        ('CDr', 0), ('CYr', f(F['CY_r'])), ('CLr', 0), ('Cellr', f(F['Cl_r'])), ('Cemr', f(F['Cm_r'])), ('Cenr', f(F['Cn_r'])),
        ('num_ctrl_surfaces', len(ctrl)),
    ]
    for k, v in vals:
        sub(p, k, v)
    for joint, c in ctrl.items():
        cs = sub(p, 'control_surface')
        sub(cs, 'name', joint)
        sub(cs, 'direction', 1)
        for k in ('CD', 'CY', 'CL', 'Cell', 'Cem', 'Cen'):
            sub(cs, f'{k}_ctrl', f(c[k] * D2R))          # the plugin reads joint angles in degrees


# ---------------------------------------------------------------- motor + prop maps for the plugin
prop = PT.Prop(os.path.join(ROOT, 'flight_params', 'propulsion', 'apc', PROP_FILE)).scaled(PROP_SCALE_IN)
mot = copy.copy(PT.MOTORS[MOTOR])
bat = PT.Battery(cells=6, capacity_ah=16.0)
speeds = np.arange(0.0, 41.0, 1.0)
throttles = np.round(np.arange(0.0, 1.0001, 0.05), 2)
maps = {k: np.zeros((len(speeds), len(throttles))) for k in ('thrust_N', 'torque_Nm', 'rpm')}
for i, v in enumerate(speeds):
    for j, d in enumerate(throttles):
        op = PT.operating_point(prop, mot, bat, d, v) if d > 0 else None
        if op is None:
            continue
        maps['thrust_N'][i, j] = op['thrust']          # per motor
        maps['torque_Nm'][i, j] = op['torque']
        maps['rpm'][i, j] = op['rpm']


def write_maps(out_model):
    os.makedirs(os.path.join(out_model, 'maps'), exist_ok=True)
    hdr = 'V_mps\\throttle,' + ','.join(f'{d:.2f}' for d in throttles)
    for k, arr in maps.items():
        np.savetxt(os.path.join(out_model, 'maps', f'{k}.csv'), np.c_[speeds, arr], delimiter=',', fmt='%.5g',
                   header=hdr, comments='')


def cruise_throttle(m, V=V_TRIM):
    q = 0.5 * 1.225 * V * V
    CL = m * 9.81 / (q * S)
    need = q * S * (aero['CD0'] + aero['K'] * CL * CL) / 2
    col = [np.interp(V, speeds, maps['thrust_N'][:, jj]) for jj in range(len(throttles))]
    return float(np.interp(need, col, throttles)), need * 2


thr_trim_full, T_cruise = cruise_throttle(mass['full']['m'])
thr_trim_empty, _ = cruise_throttle(mass['empty']['m'])
J_rot = 1.5e-4 + 0.05 * (prop.D / 2) ** 2 / 3 * 2      # rotor + four blades as rods, rough
tau_motor = J_rot * mot.rm / mot.kt ** 2


def prop_plugin(model, joint, idx, folder):
    # counter-rotating pair (left cw, right ccw seen from behind): no net torque roll, symmetric slipstream.
    # the real cl-415 turns both the same way; for the electric uav mirrored props cost nothing and the
    # same-rotation torque rolled the model onto its left float at the hump
    p = sub(model, 'plugin', filename='libCl415PropellerMap.so', name='cl415::PropellerMap')
    for k, v in (('joint_name', joint), ('command_topic', f'/{MODEL_NAME}/command/motor_speed'),
                 ('motor_number', idx), ('command_full_throttle', f(CMD_FULL)),
                 ('thrust_map', f'model://{folder}/maps/thrust_N.csv'), ('torque_map', f'model://{folder}/maps/torque_Nm.csv'),
                 ('rpm_map', f'model://{folder}/maps/rpm.csv'),
                 ('time_constant_up', f(max(0.04, tau_motor), 3)), ('time_constant_down', f(max(0.06, 1.5 * tau_motor), 3)),
                 ('turning_direction', 'cw' if idx == 0 else 'ccw'), ('visual_slowdown', 10)):
        sub(p, k, v)


# ---------------------------------------------------------------- sensors
def noise(parent, std, bias=None, tc=None):
    n = sub(parent, 'noise', type='gaussian')
    sub(n, 'mean', 0)
    sub(n, 'stddev', std)
    if bias:
        sub(n, 'dynamic_bias_stddev', bias)
        sub(n, 'dynamic_bias_correlation_time', tc)


def add_sensors(link):
    imu = sub(link, 'sensor', name='imu_sensor', type='imu')
    sub(imu, 'pose', f'{cg[0]:.4f} 0 {cg[2]:.4f} 0 0 0')
    sub(imu, 'always_on', 1)
    sub(imu, 'update_rate', 250)
    ii = sub(imu, 'imu')
    for grp, std, bias, tc in (('angular_velocity', 0.0003394, 3.8785e-05, 1000), ('linear_acceleration', 0.004, 0.006, 300)):
        g = sub(ii, grp)
        for ax in 'xyz':
            noise(sub(g, ax), std, bias, tc)
    baro = sub(link, 'sensor', name='air_pressure_sensor', type='air_pressure')
    sub(baro, 'always_on', 1)
    sub(baro, 'update_rate', 50)
    noise(sub(sub(baro, 'air_pressure'), 'pressure'), 0.01)
    # no gz magnetometer: harmonic's is wrong in banked flight, libCl415Magnetometer.so replaces it
    nav = sub(link, 'sensor', name='navsat_sensor', type='navsat')
    sub(nav, 'always_on', 1)
    sub(nav, 'update_rate', 30)


def inertial(link, m, ixx, iyy, izz, pose=None):
    i = sub(link, 'inertial')
    if pose:
        sub(i, 'pose', pose)
    sub(i, 'mass', m)
    it = sub(i, 'inertia')
    for k, v in (('ixx', ixx), ('ixy', 0), ('ixz', 0), ('iyy', iyy), ('iyz', 0), ('izz', izz)):
        sub(it, k, v)


def add_pitot(model):
    # pitot ahead of the nose, as on most uavs, so the px4 gz airspeed sensor has its link
    x, z = G['misc']['nose_x'] - 0.02, -0.06
    link = sub(model, 'link', name='airspeed_link')
    sub(link, 'pose', f'{x:.3f} 0 {z:.3f} 0 0 0')
    inertial(link, 0.005, 1e-6, 1e-6, 1e-6)
    vis = sub(link, 'visual', name='pitot_visual')
    sub(vis, 'pose', '0.04 0 0 0 1.5708 0')
    cyl = sub(sub(vis, 'geometry'), 'cylinder')
    sub(cyl, 'radius', 0.003)
    sub(cyl, 'length', 0.08)
    s = sub(link, 'sensor', name='air_speed', type='air_speed')
    sub(s, 'always_on', 1)
    sub(s, 'update_rate', 20)
    sub(s, 'air_speed')
    j = sub(model, 'joint', name='airspeed_joint', type='fixed')
    sub(j, 'parent', 'base_link')
    sub(j, 'child', 'airspeed_link')


def add_tail_camera(model):
    # onboard camera on the fin leading edge looking forward over the wing, recorded by CameraVideoRecorder
    # (service /cl415/record_tailcam)
    link = sub(model, 'link', name='tail_cam_link')
    sub(link, 'pose', '-0.72 0 0.43 0 0.10 0')
    inertial(link, 0.005, 1e-6, 1e-6, 1e-6)
    s = sub(link, 'sensor', name='tail_cam', type='camera')
    sub(s, 'update_rate', 25)
    sub(s, 'always_on', 1)
    sub(s, 'visualize', 'false')
    sub(s, 'topic', f'/{MODEL_NAME}/tailcam/image')
    cam = sub(s, 'camera')
    sub(cam, 'horizontal_fov', 1.4)
    img = sub(cam, 'image')
    sub(img, 'width', 960)
    sub(img, 'height', 540)
    sub(img, 'format', 'R8G8B8')
    clip = sub(cam, 'clip')
    sub(clip, 'near', 0.05)
    sub(clip, 'far', 4000)
    rec = sub(s, 'plugin', filename='gz-sim-camera-video-recorder-system', name='gz::sim::systems::CameraVideoRecorder')
    for k, v in (('service', f'/{MODEL_NAME}/record_tailcam'), ('use_sim_time', 'true'), ('fps', 25), ('bitrate', 4000000)):
        sub(rec, k, v)
    j = sub(model, 'joint', name='tail_cam_joint', type='fixed')
    sub(j, 'parent', 'base_link')
    sub(j, 'child', 'tail_cam_link')


def add_magnetometer(model):
    mp = sub(model, 'plugin', filename='libCl415Magnetometer.so', name='cl415::Magnetometer')
    for k, v in (('link_name', 'base_link'), ('sensor_name', 'magnetometer_sensor'),
                 ('field_ned_gauss', ' '.join(f'{x:.5f}' for x in MAG_NED)), ('update_rate', 100), ('noise_stddev', 0.002)):
        sub(mp, k, v)


# ---------------------------------------------------------------- landing gear (runway variant, sim only)
KEEL_Z = G['misc']['keel_z']
STEP_X = G['misc']['step_x']
R_WHEEL, W_WHEEL, M_WHEEL = 0.04, 0.02, 0.03
Z_MAIN = KEEL_Z - 0.045                               # contact plane of the mains
X_MAIN, Y_MAIN = STEP_X - 0.01, 0.16                   # at the hull step, just outside the hull sides
X_NOSE = 0.42
GROUND_PITCH = 2.0                                     # deg nose up sitting on the gear
Z_NOSE = Z_MAIN - (X_NOSE - X_MAIN) * math.tan(math.radians(GROUND_PITCH))
tipback = math.degrees(math.atan2(cg[0] - X_MAIN, cg[2] - Z_MAIN))
stern = (G['misc']['stern_x'], -0.189)
tail_strike = math.degrees(math.atan2(stern[1] - Z_MAIN, X_MAIN - stern[0]))


def add_wheel(model, name, x, y, z_contact):
    link = sub(model, 'link', name=name)
    sub(link, 'pose', f'{x:.4f} {y:.4f} {z_contact + R_WHEEL:.4f} 0 0 0')
    inertial(link, M_WHEEL, 0.25 * M_WHEEL * R_WHEEL ** 2, 0.5 * M_WHEEL * R_WHEEL ** 2, 0.25 * M_WHEEL * R_WHEEL ** 2)
    for kind in ('visual', 'collision'):
        e = sub(link, kind, name=f'{name}_{kind}')
        sub(e, 'pose', '0 0 0 1.5708 0 0')
        cyl = sub(sub(e, 'geometry'), 'cylinder')
        sub(cyl, 'radius', R_WHEEL)
        sub(cyl, 'length', W_WHEEL)
        if kind == 'collision':
            ode = sub(sub(sub(e, 'surface'), 'friction'), 'ode')
            sub(ode, 'mu', 1.0)
            sub(ode, 'mu2', 0.5)
            sub(ode, 'fdir1', '0 0 1')
        else:
            mat = sub(e, 'material')
            sub(mat, 'diffuse', '0.1 0.1 0.1 1')
            sub(mat, 'ambient', '0.1 0.1 0.1 1')
    j = sub(model, 'joint', name=f'{name}_joint', type='revolute')
    sub(j, 'parent', 'base_link')
    sub(j, 'child', name)
    ax = sub(j, 'axis')
    sub(ax, 'xyz', '0 1 0')
    lim = sub(ax, 'limit')
    sub(lim, 'lower', -1e16)
    sub(lim, 'upper', 1e16)
    sub(sub(ax, 'dynamics'), 'damping', 0.0005)


def add_struts(base):
    # visual legs from the hull to each axle
    for name, x, y, z_c, z_top in (('nose', X_NOSE, 0.0, Z_NOSE, -0.27), ('lmain', X_MAIN, Y_MAIN, Z_MAIN, -0.20),
                                   ('rmain', X_MAIN, -Y_MAIN, Z_MAIN, -0.20)):
        z_axle = z_c + R_WHEEL
        y_top = 0.0 if name == 'nose' else math.copysign(0.10, y)
        L = math.hypot(z_top - z_axle, y - y_top)
        roll = math.atan2(y - y_top, z_top - z_axle)
        v = sub(base, 'visual', name=f'{name}_strut_visual')
        sub(v, 'pose', f'{x:.4f} {(y + y_top) / 2:.4f} {(z_top + z_axle) / 2:.4f} {roll:.4f} 0 0')
        cyl = sub(sub(v, 'geometry'), 'cylinder')
        sub(cyl, 'radius', 0.006)
        sub(cyl, 'length', f'{L:.4f}')
        mat = sub(v, 'material')
        sub(mat, 'diffuse', '0.6 0.6 0.6 1')
        sub(mat, 'ambient', '0.6 0.6 0.6 1')


# ---------------------------------------------------------------- flying boat hull (water variant)
# the collision boxes of the hull and the floats become hull elements. k_bottom is the planing lift factor of
# each box's bottom (Savitsky flat plate ~0.6, less for the 20 deg deadrise forebody, less again behind the
# step where the afterbody runs in the forebody's wake)
HULL_K = {'c0': 0.30, 'c1': 0.40, 'c2': 0.40, 'c3': 0.25, 'c4': 0.20, 'c5': 0.20}
# the collision boxes are flat, the real keel line is not. bow: the two bow elements are tilted bow-up so their
# bottoms rise toward the bow (rocker); pressing the bow down then makes forebody lift ahead of the cg instead of
# ploughing (first water run: thrust line 0.16 m above the cg, nose 5 deg down, stuck at 6.5 m/s). afterbody:
# the three elements behind the step get the sternpost angle, so they only touch the water above that trim and
# the forebody is free to plane at 3 to 6 deg (second run: flat afterbody bottoms pinned the trim at 0 deg,
# stuck at 7 m/s). angles in deg, bow up
HULL_ROCKER_DEG = {'c0': 18.0, 'c1': 8.0, 'c3': 7.0, 'c4': 7.0, 'c5': 7.0}
# behind the step the afterbody runs in the wake hollow of the forebody: its water level drops by about the
# step depth plus the planing draft once the hull is on the step (blended in between 6 and 9 m/s; earlier than
# that the afterbody dries while the forebody is still deep and the hull spins, all its side area being ahead
# of the cg)
HULL_WAKE_M = {'c3': 0.10, 'c4': 0.10, 'c5': 0.10}
FORM_CD = {'hull': 0.10, 'left_float': 0.15, 'right_float': 0.15, 'left_wing': 0.3, 'right_wing': 0.3}


def hull_elements(base):
    els = []
    for c in base.findall('collision'):
        nm = c.get('name')
        box = c.find('geometry/box/size')
        if box is None:
            continue
        pose = c.find('pose').text if c.find('pose') is not None else '0 0 0 0 0 0'
        if 'fuselage_c' in nm:
            idx = nm.split('fuselage_')[1][:2]
            if idx in HULL_ROCKER_DEG:
                x, y, z = [float(v) for v in pose.split()[:3]]
                pose = f'{x} {y} {z} 0 {-math.radians(HULL_ROCKER_DEG[idx]):.5f} 0'    # gz: negative pitch = bow up
            els.append(dict(name=f'hull_{idx}', group='hull', pose=pose, size=box.text, k_bottom=HULL_K[idx],
                            wake_depth=HULL_WAKE_M.get(idx, 0.0)))
        elif 'float_c0' in nm:
            side = 'left' if 'left' in nm else 'right'
            els.append(dict(name=f'{side}_float', group=f'{side}_float', pose=pose, size=box.text, k_bottom=0.5,
                            wake_depth=0.0))
        elif 'wing_c0' in nm:
            # the wing panels, so a dropped wing tip meets the water (buoyancy, drag) instead of passing through it
            side = 'left' if 'left' in nm else 'right'
            els.append(dict(name=f'{side}_wing', group=f'{side}_wing', pose=pose, size=box.text, k_bottom=0.5,
                            wake_depth=0.0))
    return els


def add_hull(model, base):
    p = sub(model, 'plugin', filename='libCl415FlyingBoatHull.so', name='cl415::FlyingBoatHull')
    for k, v in (('link_name', 'base_link'), ('water_level', 0.0), ('water_density', 998.0), ('friction_cf', 0.004),
                 ('side_k', 0.6), ('bluff_cd', 1.0), ('heave_damping_ratio', 1.0),
                 ('wake_speed_min', 6.0), ('wake_speed_max', 9.0), ('chine_separation', 'true'),
                 ('state_topic', f'/{MODEL_NAME}/hull'), ('publish_rate', 50)):
        sub(p, k, v)
    fd = sub(p, 'form_drag')
    for g, cd in FORM_CD.items():
        sub(fd, g, cd)
    els = hull_elements(base)
    for el in els:
        e = sub(p, 'element')
        for k in ('name', 'group', 'pose', 'size', 'k_bottom', 'wake_depth'):
            sub(e, k, el[k])
    return els


# ---------------------------------------------------------------- build the models
def build_model(variant):
    folder = 'cl415_px4' if variant == 'runway' else 'cl415_px4_water'
    out = os.path.join(HERE, 'models', folder)
    write_maps(out)
    tree = ET.parse(SRC)
    model = tree.getroot().find('model')
    model.set('name', MODEL_NAME)
    base = model.find("link[@name='base_link']")
    for s in list(base.findall('sensor')):
        base.remove(s)
    add_sensors(base)
    for p in list(model.findall('plugin')):
        nm = p.get('name')
        if nm in ('gz::sim::systems::LiftDrag', 'gz::sim::systems::MulticopterMotorModel'):
            model.remove(p)
        elif nm == 'gz::sim::systems::JointPositionController':
            order = ['left_aileron_joint', 'right_aileron_joint', 'elevator_joint', 'rudder_joint']
            p.find('topic').text = f'/model/{MODEL_NAME}/servo_{order.index(p.find("joint_name").text)}'
    ald_plugin(model)
    add_magnetometer(model)
    prop_plugin(model, 'left_propeller_joint', 0, folder)
    prop_plugin(model, 'right_propeller_joint', 1, folder)
    add_pitot(model)
    add_tail_camera(model)
    els = None
    if variant == 'runway':
        add_struts(base)
        for name, x, y, z in (('nose_wheel', X_NOSE, 0.0, Z_NOSE), ('left_main_wheel', X_MAIN, Y_MAIN, Z_MAIN),
                              ('right_main_wheel', X_MAIN, -Y_MAIN, Z_MAIN)):
            add_wheel(model, name, x, y, z)
    else:
        els = add_hull(model, base)
    ET.indent(tree, space='  ')
    tree.write(os.path.join(out, 'model.sdf'), encoding='unicode', xml_declaration=True)
    what = ('sim-only tricycle gear for runway work' if variant == 'runway'
            else 'flying-boat hull plugin (exact buoyancy, planing, water drag) for water operations')
    open(os.path.join(out, 'model.config'), 'w').write(f'''<?xml version="1.0"?>
<model>
  <name>{folder}</name>
  <version>1.0</version>
  <sdf version="1.11">model.sdf</sdf>
  <author><name>CL415-VIP</name></author>
  <description>2.51 m CL-415 UAV for PX4 SITL: AVL aero (AdvancedLiftDrag), Hacker A50-16S + 13.5 in 4-blade map
  propulsion, PX4 sensor set, {what}. Generated by px4_sitl/make_px4_model.py.</description>
</model>
''')
    return els


hull_els = None
for variant in ('runway', 'water'):
    r = build_model(variant)
    if r:
        hull_els = r

# ---------------------------------------------------------------- worlds
WORLD_HEAD = '''<?xml version="1.0" encoding="UTF-8"?>
<!-- {comment}
     location, magnetic field and physics step as px4's default world so the ekf's magnetic model agrees with the
     simulated magnetometer -->
<sdf version="1.9">
  <world name="{name}">
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-air-pressure-system" name="gz::sim::systems::AirPressure"/>
    <plugin filename="gz-sim-air-speed-system" name="gz::sim::systems::AirSpeed"/>
    <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>
    <plugin filename="gz-sim-magnetometer-system" name="gz::sim::systems::Magnetometer"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>{lat}</latitude_deg>
      <longitude_deg>{lon}</longitude_deg>
      <elevation>0</elevation>
    </spherical_coordinates>
    <scene>
      <grid>false</grid>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.6 0.75 0.9 1</background>
      <shadows>true</shadows>
    </scene>
    <light name="sun" type="directional">
      <pose>0 0 500 0 0 0</pose>
      <cast_shadows>true</cast_shadows>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
      <direction>0.2 0.5 -0.8</direction>
    </light>
'''
# external chase camera: a free, weightless camera model that libCl415ChaseCamera.so keeps behind the aircraft,
# recorded server-side by CameraVideoRecorder (service /chase/record_video, start/stop with gz.msgs.VideoRecord)
WORLD_TAIL = '''    <model name="chase_cam">
      <pose>-8 -3.5 2.5 0 0 0</pose>
      <link name="link">
        <gravity>false</gravity>
        <inertial><mass>0.01</mass><inertia><ixx>1e-5</ixx><iyy>1e-5</iyy><izz>1e-5</izz></inertia></inertial>
        <sensor name="camera" type="camera">
          <update_rate>25</update_rate>
          <always_on>1</always_on>
          <visualize>false</visualize>
          <topic>/chase/image</topic>
          <camera>
            <horizontal_fov>0.85</horizontal_fov>
            <image><width>1280</width><height>720</height><format>R8G8B8</format></image>
            <clip><near>0.1</near><far>4000</far></clip>
          </camera>
          <plugin filename="gz-sim-camera-video-recorder-system" name="gz::sim::systems::CameraVideoRecorder">
            <service>/chase/record_video</service>
            <use_sim_time>true</use_sim_time>
            <fps>25</fps>
            <bitrate>6000000</bitrate>
          </plugin>
        </sensor>
      </link>
      <plugin filename="libCl415ChaseCamera.so" name="cl415::ChaseCamera">
        <target>{model}</target>
        <offset>-5.5 -2.6 1.3</offset>
        <look_height>0.1</look_height>
        <smoothing>0.25</smoothing>
      </plugin>
    </model>
    <include>
      <uri>model://{folder}</uri>
      <name>{model}</name>
      <pose>0 0 {z:.3f} 0 0 0</pose>
    </include>
  </world>
</sdf>
'''
os.makedirs(os.path.join(HERE, 'worlds'), exist_ok=True)
spawn_z = -Z_NOSE + 0.01
runway = WORLD_HEAD.format(comment='runway world for the cl415 px4 sitl.', name='cl415_runway', lat=LAT, lon=LON) + '''    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>4000 4000</size></plane></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
        <visual name="grass">
          <geometry><plane><normal>0 0 1</normal><size>4000 4000</size></plane></geometry>
          <material><ambient>0.35 0.5 0.3 1</ambient><diffuse>0.35 0.5 0.3 1</diffuse></material>
        </visual>
        <visual name="runway">
          <pose>150 0 0.005 0 0 0</pose>
          <geometry><box><size>400 20 0.01</size></box></geometry>
          <material><ambient>0.25 0.25 0.27 1</ambient><diffuse>0.25 0.25 0.27 1</diffuse></material>
        </visual>
        <visual name="centreline">
          <pose>150 0 0.011 0 0 0</pose>
          <geometry><box><size>380 0.3 0.002</size></box></geometry>
          <material><ambient>0.95 0.95 0.95 1</ambient><diffuse>0.95 0.95 0.95 1</diffuse></material>
        </visual>
      </link>
    </model>
''' + WORLD_TAIL.format(folder='cl415_px4', model=MODEL_NAME, z=spawn_z)
open(os.path.join(HERE, 'worlds', 'cl415_runway.sdf'), 'w').write(runway)

# water: the surface is z = 0 in the hull plugin, the visual plane is just under it; a seabed 30 m down catches
# anything that sinks. the aircraft is dropped from just above its floating height
water_spawn_z = -KEEL_Z - 0.09 + 0.03
water = WORLD_HEAD.format(comment='calm-water world for the cl415 px4 sitl (water take-off and landing).',
                          name='cl415_water', lat=LAT, lon=LON) + '''    <model name="sea">
      <static>true</static>
      <link name="link">
        <collision name="seabed">
          <pose>0 0 -30 0 0 0</pose>
          <geometry><plane><normal>0 0 1</normal><size>6000 6000</size></plane></geometry>
        </collision>
        <visual name="surface">
          <pose>0 0 -0.003 0 0 0</pose>
          <geometry><plane><normal>0 0 1</normal><size>6000 6000</size></plane></geometry>
          <material>
            <ambient>0.05 0.25 0.4 0.85</ambient>
            <diffuse>0.08 0.35 0.55 0.85</diffuse>
            <specular>0.6 0.6 0.6 1</specular>
          </material>
        </visual>
        <visual name="shore">
          <pose>0 -1500 0.3 0 0 0</pose>
          <geometry><box><size>6000 1000 0.6</size></box></geometry>
          <material><ambient>0.35 0.5 0.3 1</ambient><diffuse>0.35 0.5 0.3 1</diffuse></material>
        </visual>
      </link>
    </model>
''' + WORLD_TAIL.format(folder='cl415_px4_water', model=MODEL_NAME, z=water_spawn_z)
open(os.path.join(HERE, 'worlds', 'cl415_water.sdf'), 'w').write(water)

# ---------------------------------------------------------------- px4 airframes
# rate-loop feed-forwards from the AVL derivatives at the 20 m/s trim speed (actuator per rad/s):
#   roll:  steady p needs Cl_da da = -Cl_p p b/2V
#   pitch: a steady pull-up needs elevator for the rate damping and for the extra alpha of the load factor
#   yaw:   steady r needs Cn_dr dr = -Cn_r r b/2V
q_dyn = 0.5 * 1.225 * V_TRIM ** 2
m_f = mass['full']['m']
rr_ff = (-F['Cl_p'] * B / (2 * V_TRIM)) / (F['Cl_da'] * SERVO_MAX_DEG['ail'] * D2R)
pr_ff = (-(F['Cm_q'] * C / (2 * V_TRIM)) - F['Cm_alpha'] * m_f * V_TRIM / (q_dyn * S * F['CL_alpha'])) / (-F['Cm_de'] * SERVO_MAX_DEG['ele'] * D2R)
yr_ff = (-F['Cn_r'] * B / (2 * V_TRIM)) / (-F['Cn_dr'] * SERVO_MAX_DEG['rud'] * D2R)
vs = aero['stall']['full'][2]
alpha_trim = D['trim_reference']['loaded']['alpha_deg']


def airframe(variant):
    water = variant == 'water'
    name = 'CL-415 UAV water (flying boat hull)' if water else 'CL-415 UAV (2.51 m, twin 4-blade, AVL aero)'
    world = 'cl415_water' if water else 'cl415_runway'
    folder = 'cl415_px4_water' if water else 'cl415_px4'
    # take-off: on the water hold the nose a little higher so the hull climbs onto the step, and rotate later
    # (the hull keeps planing until the wing carries it); on wheels keep the stern off the ground
    tko = f'''# runway take-off logic (also right for the water run: heading hold with the rudder, pitch held, rotate at speed)
param set-default RWTO_TKOFF 1
param set-default RWTO_PSP {12.0 if water else 2.0}
param set-default RWTO_RAMP_TIME {3.0 if water else 2.0}
param set-default FW_TKO_PITCH_MIN {8.0 if water else 6.0}{'' if water else '           # the hull stern touches at ' + f'{tail_strike:.0f}' + ' deg on the gear'}
param set-default FW_TKO_AIRSPD {1.2 * vs:.1f}'''
    if water:
        tko += '''
# on the water px4's runway logic holds pitch with proportional elevator only (integrators reset until lift-off),
# which is 5 deg of elevator against the thrust line and the hull resistance below the cg. a seaplane pilot holds
# full back pressure: a high taxi pitch target, a quicker attitude loop and px4's low-airspeed pitch trim (full
# below FW_AIRSPD_MIN, gone by FW_AIRSPD_TRIM) give the loop the same authority
param set-default FW_P_TC 0.25
param set-default TRIM_PITCH 0.10
param set-default FW_DTRIM_P_VMIN 0.25'''
    land = f'''# landing. {'glassy-water technique: a shallow approach, flare early to a nose-up attitude and a low sink rate, touch on the step' if water else 'wheels: normal flare'}
param set-default FW_LND_ANG {4.0 if water else 5.0}
param set-default FW_LND_AIRSPD {1.2 * vs:.1f}
param set-default FW_LND_FLALT {2.5 if water else 1.5}
param set-default FW_LND_FL_SINK {0.4 if water else 0.5}
param set-default FW_LND_FL_PMIN {4.0 if water else 2.0}
param set-default FW_LND_FL_PMAX {9.0 if water else 8.0}
# no rangefinder on the model yet: flare from the gps/baro height, don't abort for a missing terrain estimate
param set-default FW_LND_USETER 0
param set-default NAV_LOITER_RAD 80
param set-default NAV_ACC_RAD 25'''
    return f'''#!/bin/sh
#
# @name {name}
#
# @type Standard Plane
# @class Plane
#
# @maintainer CL415-VIP
#
# generated by cl415/px4_sitl/make_px4_model.py from the AVL/XFOIL/SolidWorks numbers
# outputs: motor 1 left, motor 2 right; servo 1 left aileron, 2 right aileron, 3 elevator, 4 rudder
#

. ${{R}}etc/init.d/rc.fw_defaults

PX4_SIMULATOR=${{PX4_SIMULATOR:=gz}}
PX4_GZ_WORLD=${{PX4_GZ_WORLD:={world}}}
PX4_SIM_MODEL=${{PX4_SIM_MODEL:={folder}}}

param set-default SIM_GZ_EN 1

# geometry and actuators. rotor z stays at zero: the allocator would otherwise count the thrust-line moment
# (hubs 0.16 m above the cg) in its own units and saturate the elevator against it; the moment is in the
# physics and the pitch integrator trims it out. {'the lateral offsets are kept so the allocator can yaw with differential thrust, which is what steers the hull on the water before the rudder bites' if water else 'rotor y offsets are zero too: rudder only, no differential thrust'}
param set-default CA_AIRFRAME 1
param set-default CA_ROTOR_COUNT 2
param set-default CA_ROTOR0_PX 0
param set-default CA_ROTOR0_PY {-0.297 if water else 0}
param set-default CA_ROTOR0_PZ 0
param set-default CA_ROTOR1_PX 0
param set-default CA_ROTOR1_PY {0.297 if water else 0}
param set-default CA_ROTOR1_PZ 0
param set-default CA_SV_CS_COUNT 4
param set-default CA_SV_CS0_TYPE 1
param set-default CA_SV_CS0_TRQ_R -0.5
param set-default CA_SV_CS1_TYPE 2
param set-default CA_SV_CS1_TRQ_R 0.5
param set-default CA_SV_CS2_TYPE 3
param set-default CA_SV_CS2_TRQ_P 1.0
param set-default CA_SV_CS3_TYPE 4
param set-default CA_SV_CS3_TRQ_Y 1.0

# gz outputs: motors as throttle x {CMD_FULL:g}, read by the prop-map plugin.
# min 1 (not 0) when armed: px4's gz esc check calls a motor offline when its command is exactly zero
param set-default SIM_GZ_EC_FUNC1 101
param set-default SIM_GZ_EC_FUNC2 102
param set-default SIM_GZ_EC_MIN1 1
param set-default SIM_GZ_EC_MIN2 1
param set-default SIM_GZ_EC_MAX1 {CMD_FULL:g}
param set-default SIM_GZ_EC_MAX2 {CMD_FULL:g}
param set-default SIM_GZ_EC_DIS1 0
param set-default SIM_GZ_EC_DIS2 0
# servos: px4 + is trailing edge up (rudder: right), the model joints are + trailing edge down (rudder: left),
# so every servo runs from +max to -max
param set-default SIM_GZ_SV_FUNC1 201
param set-default SIM_GZ_SV_FUNC2 202
param set-default SIM_GZ_SV_FUNC3 203
param set-default SIM_GZ_SV_FUNC4 204
param set-default SIM_GZ_SV_MINA1 {SERVO_MAX_DEG['ail']:g}
param set-default SIM_GZ_SV_MAXA1 -{SERVO_MAX_DEG['ail']:g}
param set-default SIM_GZ_SV_MINA2 {SERVO_MAX_DEG['ail']:g}
param set-default SIM_GZ_SV_MAXA2 -{SERVO_MAX_DEG['ail']:g}
param set-default SIM_GZ_SV_MINA3 {SERVO_MAX_DEG['ele']:g}
param set-default SIM_GZ_SV_MAXA3 -{SERVO_MAX_DEG['ele']:g}
param set-default SIM_GZ_SV_MINA4 {SERVO_MAX_DEG['rud']:g}
param set-default SIM_GZ_SV_MAXA4 -{SERVO_MAX_DEG['rud']:g}

# speeds (full tank, sea level): stall {vs:.1f} m/s
param set-default FW_AIRSPD_STALL {vs:.1f}
param set-default FW_AIRSPD_MIN {1.2 * vs:.1f}
param set-default FW_AIRSPD_TRIM {V_TRIM:.1f}
param set-default FW_AIRSPD_MAX 28.0
param set-default FW_THR_TRIM {thr_trim_full:.2f}
param set-default FW_THR_MAX 1.0
param set-default FW_THR_MIN 0.0
param set-default FW_PSP_OFF {alpha_trim:.1f}
param set-default FW_P_LIM_MAX 25
param set-default FW_P_LIM_MIN -20
param set-default FW_R_LIM 40

# rate loops, feed-forward from the AVL derivatives at {V_TRIM:.0f} m/s
param set-default FW_RR_FF {rr_ff:.3f}
param set-default FW_RR_P 0.08
param set-default FW_RR_I 0.1
param set-default FW_PR_FF {pr_ff:.3f}
param set-default FW_PR_P 0.12
param set-default FW_PR_I 0.15
param set-default FW_YR_FF {yr_ff:.3f}
param set-default FW_YR_P 0.08
param set-default FW_YR_I 0.05

# tecs: plenty of thrust (static T/W ~0.9), glide sink at trim speed ~1.5 m/s
param set-default FW_T_CLMB_MAX 5.0
param set-default FW_T_CLMB_R_SP 3.0
param set-default FW_T_SINK_MAX 4.0
param set-default FW_T_SINK_MIN 1.5
param set-default FW_T_SINK_R_SP 2.0

{tko}

{land}

# sim: battery 6S 16 Ah, no rc, no datalink failsafe
param set-default BAT1_N_CELLS 6
param set-default BAT1_CAPACITY 16000
param set-default COM_RC_IN_MODE 4
param set-default NAV_RCL_ACT 0
param set-default NAV_DLL_ACT 0
'''


os.makedirs(os.path.join(HERE, 'airframes'), exist_ok=True)
open(os.path.join(HERE, 'airframes', '4050_gz_cl415'), 'w', newline='\n').write(airframe('runway'))
open(os.path.join(HERE, 'airframes', '4051_gz_cl415_water'), 'w', newline='\n').write(airframe('water'))

summary = {
    'ald': {'CL0': CL0_ALD, 'CLa': CLA_ALD, 'alpha_stall_deg': math.degrees(A_STALL), 'CL_peak': CL_PEAK,
            'alpha_peak_deg': math.degrees(AL_PEAK), 'fit_rms_-5_to_6deg': fit_rms, 'cp_cg_flu': cg.tolist(),
            'controls_per_rad': ctrl, 'aileron_one_side_CL_per_rad': cl_one_ail},
    'propulsion': {'prop': prop.name, 'motor': mot.name, 'static_thrust_total': float(2 * maps['thrust_N'][0, -1]),
                   'cruise_throttle_full': thr_trim_full, 'cruise_throttle_empty': thr_trim_empty, 'cruise_thrust_total': T_cruise,
                   'motor_time_constant_s': tau_motor},
    'gear': {'wheel_radius': R_WHEEL, 'main': [X_MAIN, Y_MAIN, Z_MAIN], 'nose': [X_NOSE, 0, Z_NOSE], 'ground_pitch_deg': GROUND_PITCH,
             'tipback_deg': tipback, 'tail_strike_pitch_deg': tail_strike, 'spawn_z': spawn_z},
    'hull': {'elements': hull_els, 'form_cd': FORM_CD, 'spawn_z': water_spawn_z},
    'px4': {'FW_RR_FF': rr_ff, 'FW_PR_FF': pr_ff, 'FW_YR_FF': yr_ff, 'FW_THR_TRIM': thr_trim_full, 'stall': vs},
}
json.dump(summary, open(os.path.join(HERE, 'px4_model_summary.json'), 'w'), indent=1)
print(json.dumps({k: v for k, v in summary.items() if k in ('hull', 'propulsion')}, indent=1))
