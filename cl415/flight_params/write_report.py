"""flight_params/README.md from cl415_flight_data.json, the airfoil summary and the MATLAB check
run from the project root:  .venv/Scripts/python flight_params/write_report.py
"""
import json
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
D = json.load(open(os.path.join(HERE, 'cl415_flight_data.json')))
XF = json.load(open(os.path.join(HERE, 'airfoils', 'summary.json')))
CHK = json.load(open(os.path.join(HERE, 'matlab_check', 'check_results.json')))
AP = json.load(open(os.path.join(ROOT, 'aero', 'aero_params.json')))
g, m, a, p = D['geometry'], D['mass'], D['aero'], D['propulsion']
F, E, NB = a['about_full_cg'], a['about_empty_cg'], a['no_bodies_full_cg']
NU = 1.46e-5


def f(x, n=4):
    return f'{x:.{n}g}'


L = ['# CL-415 UAV: flight simulation parameters', '',
     'Everything a flight sim needs for the 2.51 m CL-415 UAV, worked out from the CAD and the SolidWorks mass '
     'properties. `cl415_params.yaml` is a drop-in file for the team\'s `params.yaml` schema v1 '
     '(FireFightingUAVF26-27, FRD body axes); `cl415_flight_data.json` has every number below plus the extras '
     '(empty-CG reference, airfoil polars, all propulsion candidates).', '',
     '## Files', '',
     '| file | what |', '|---|---|',
     '| `cl415_params.yaml`, `.json` | the team schema: geometry, mass (SolidWorks), aero derivatives, propulsion, actuators, speeds, trims, design points |',
     '| `cl415_flight_data.json` | all numbers, both CG references, AVL trims, airfoil data, every propulsion candidate |',
     '| `airfoils/` | XFOIL polars (`polars/*.csv`), plots, `report.md` with the section properties per Reynolds number |',
     '| `avl/` | the AVL model (`cl415_full.avl`, `cl415_empty.avl`, body files) and its stability-derivative outputs |',
     '| `propulsion/` | APC prop data, the motor/prop/battery model, thrust maps for each feasible candidate, `evaluate.py` |',
     '| `matlab_check/` | the team\'s trim/linearize/modes run on these numbers (`check_output.txt`) |',
     '| `plots/aero_and_thrust.png` | stall, drag polar, thrust required vs available |', '',
     'Rebuild: `run_xfoil.py` (airfoils), `avl/build_avl.py`, `build_flight_params.py`, then `matlab -batch '
     '"run(\'flight_params/matlab_check/run_check.m\')"` and `write_report.py`.', '',
     '## Reference values', '',
     '| | value |', '|---|---:|',
     f'| span b | {g["b"]} m |', f'| wing area S | {f(g["S"])} m^2 |', f'| mean chord c | {g["c_bar"]} m |',
     f'| aspect ratio | {f(g["AR"])} |', f'| wing incidence to the hull datum | {g["wing_incidence_deg"]} deg |',
     f'| h-stab / fin / finlets area | {f(g["S_htail"])} / {f(g["S_vtail"])} / {f(g["S_finlets"])} m^2 |',
     f'| aileron span, chord ratio | y {g["aileron_y"][0]:.3f} to {g["aileron_y"][1]:.3f} m, {g["aileron_cf_c"]:.2f} |',
     f'| elevator / rudder chord ratio | {g["elevator_cf_c"]:.2f} / {g["rudder_cf_c_root_tip"][0]:.2f} (root) to {g["rudder_cf_c_root_tip"][1]:.2f} (tip) |',
     f'| hull length | {f(g["L_fus"])} m |', f'| prop diameter (CAD) | {f(g["prop_diameter"])} m |',
     f'| prop hubs from the full-tank CG (FRD) | x {g["prop_hub_frd_from_full_cg"][0]:+.3f}, y +/-0.297, z {g["prop_hub_frd_from_full_cg"][2]:+.3f} m (0.16 m above the CG) |',
     '', '## Mass and inertia (SolidWorks)', '',
     'FRD axes about each case\'s own CG, team convention J = [Ixx 0 -Ixz; 0 Iyy 0; -Ixz 0 Izz], Ixz = +integral(x z dm).', '',
     '| | full tank | empty |', '|---|---:|---:|',
     f'| mass [kg] | {m["full"]["m"]} | {m["empty"]["m"]} |',
     f'| CG [% MAC] | {m["full"]["cg_pct_mac"]:.1f} | {m["empty"]["cg_pct_mac"]:.1f} |',
     f'| CG (x, z) from the wing LE root, REP-103 [m] | ({m["full"]["cg_flu"][0]:.4f}, {m["full"]["cg_flu"][2]:.4f}) | ({m["empty"]["cg_flu"][0]:.4f}, {m["empty"]["cg_flu"][2]:.4f}) |']
for k in ('Ixx', 'Iyy', 'Izz', 'Ixz'):
    L.append(f'| {k} [kg m^2] | {m["full"]["J_frd"][k]:.4f} | {m["empty"]["J_frd"][k]:.4f} |')
L += ['', 'Ixy and Iyz are zero to within 1e-5 (symmetric aircraft).', '',
      '## Aerodynamic model', '',
      'Stability and control derivatives from AVL 3.52 (vortex lattice) on the CAD geometry: NACA 4417 camber on the '
      'wing, lift-slope factor from XFOIL per surface, hull and nacelles as slender bodies. All per radian, body alpha '
      '(alpha = 0 is the hull datum, the wing sits at +2.65 deg), rates as p b/2V, q c/2V, r b/2V. Control derivatives are '
      f'AVL\'s inviscid values times the XFOIL viscous flap factor: aileron {a["control_viscous_factors"]["a"]:.3f}, '
      f'elevator {a["control_viscous_factors"]["e"]:.3f} (also covers the rudder cut-outs AVL doesn\'t see), '
      f'rudder {a["control_viscous_factors"]["r"]:.3f}.', '',
      '### Longitudinal', '',
      '| | about full-tank CG | about empty CG | quick hand check |', '|---|---:|---:|---|']
hand = {
    'CL_alpha': f'wing alone {AP["derived"]["a_w"]:.2f} (Helmbold), plus tail',
    'Cm_alpha': f'static margin {-F["Cm_alpha"] / F["CL_alpha"] * 100:.0f} % (full), neutral point {F["Xnp_over_c"] * 100:.0f} % MAC',
    'Cm_q': f'tail only -2 eta V_H a_t l_t/c = {-2 * 0.9 * AP["derived"]["V_H"] * AP["derived"]["a_h"] * (0.8033 / 0.278):.1f}',
    'CL0': '2.65 deg incidence + camber, less tail download',
}
for k in ('CL0', 'CL_alpha', 'CL_q', 'CL_de', 'Cm0', 'Cm_alpha', 'Cm_q', 'Cm_de'):
    L.append(f'| {k} | {F[k]:.4f} | {E[k]:.4f} | {hand.get(k, "")} |')
L += [f'| CD_de | {a["CD_de"]:.4f} | same | XFOIL elevator drag at 10 deg |', '',
      '### Lateral-directional', '',
      '| | about full-tank CG | about empty CG | without hull/nacelle bodies |', '|---|---:|---:|---:|']
for k in ('CY_beta', 'Cl_beta', 'Cn_beta', 'CY_p', 'Cl_p', 'Cn_p', 'CY_r', 'Cl_r', 'Cn_r',
          'CY_da', 'Cl_da', 'Cn_da', 'CY_dr', 'Cl_dr', 'Cn_dr'):
    L.append(f'| {k} | {F[k]:.4f} | {E[k]:.4f} | {NB[k]:.4f} |')
L += ['', f'Spiral parameter Clb Cnr / (Clr Cnb) = {F["spiral_param"]:.2f} (below 1, so the spiral mode is slightly '
      'divergent: a straight wing with no dihedral). The hull as an AVL slender body costs about '
      f'{NB["Cn_beta"] - F["Cn_beta"]:.3f} of Cn_beta; slender-body theory overstates that on a short beamy hull, so treat the two columns as the bounds on Cn_beta '
      '(the yaml uses the lower one).', '',
      '### Drag, stall, speeds', '',
      f'- Drag polar CD = **{a["CD0"]:.4f} + {a["K"]:.4f} CL^2** (Oswald e = {a["oswald_e"]:.2f}), fitted over CL 0.1 to 1.25 '
      'to: AVL Trefftz-plane induced drag (tail included) + XFOIL 4417 profile drag at the cruise Re x1.10 for '
      'seams and gaps + XFOIL tail-section drag + hull, nacelles and floats by the Raymer component method + 5 % '
      f'misc. At cruise the tails add {a["CD_breakdown_at_cruise"]["tail_profile"]:.4f} and the bodies '
      f'{a["CD_breakdown_at_cruise"]["bodies"]:.4f}.',
      f'- That e is optimistic next to typical flight-test values (Raymer\'s empirical estimate for this wing is '
      f'{AP["derived"]["e"]:.2f}); for robustness runs try K x1.2 and CD0 x1.2.',
      f'- CL_max = **{a["CL_max"]:.3f}** (critical section: the first wing strip in AVL to reach the XFOIL 4417 cl_max x0.92 '
      f'at the stall Reynolds number), at {a["stall"]["full"][1]:.1f} deg body alpha.',
      f'- Stall speed {a["stall"]["full"][2]:.1f} m/s full, {a["stall"]["empty"][2]:.1f} m/s empty (sea level).',
      f'- Best L/D {p["requirements"]["full"]["LD_max"]:.1f} at {p["requirements"]["full"]["V_md"]:.1f} m/s (full); '
      f'minimum power at {p["requirements"]["full"]["V_mp"]:.1f} m/s.', '',
      '## Airfoils (XFOIL, Ncrit 9)', '',
      '| section | Re (20 m/s) | alpha_L0 | cl_alpha [/rad] | cl_max | cd_min | cm c/4 | x_ac/c | control tau (XFOIL / thin airfoil) |',
      '|---|---:|---:|---:|---:|---:|---:|---:|---|']
for name, chord in (('wing_4417', g.get('c_bar')), ('htail_0016', 0.239), ('fin_0012', 0.31)):
    S_ = XF['sections'][name]
    re = 20 * S_['chord_m'] / NU
    k = min(S_['reynolds'], key=lambda kk: abs(float(kk) - re))
    r = S_['reynolds'][k]
    fl = S_['flap']
    L.append(f'| NACA {S_["naca"]} ({fl["surface"]}) | {float(k) / 1e3:.0f}k | {r["alpha_L0_deg"]:.2f} deg | {r["cl_alpha_per_rad"]:.2f} | '
             f'{r["cl_max"]:.3f} at {r["alpha_cl_max_deg"]:.1f} deg | {r["cd_min"]:.4f} | {r["cm0"]:.4f} | {r["x_ac_over_c"]:.3f} | '
             f'{fl["tau_xfoil"]:.3f} / {fl["tau_thin_airfoil"]:.3f} |')
L += ['', 'All Reynolds numbers (150k to 600k), cl_min, (cl/cd)max and the polar fits are in `airfoils/report.md`.', '',
      '## Check in the team\'s 6-DOF code', '',
      '`matlab_check/run_check.m` loads `cl415_params.json` into the unchanged FireFightingUAVF26-27 MATLAB tools and runs '
      'the full nonlinear trim (thrust tilt and thrust-line moment included), the linearisation and the MIL-F-8785C '
      'mode check at the six design points (the Optimization Toolbox isn\'t installed here, so a small '
      'Levenberg-Marquardt stand-in replaces fsolve for this check only).', '',
      '| point | V [m/s] | gamma | alpha [deg] | de [deg] | throttle | short period wn / zeta | phugoid zeta | dutch roll wn / zeta | roll tau [s] | spiral t2 [s] | Level 1 |',
      '|---|---:|---:|---:|---:|---:|---|---:|---|---:|---:|---|']
dpts = {d['id']: d for d in json.load(open(os.path.join(HERE, 'cl415_params.json')))['design_points']}
for r in CHK:
    M = {x['name']: x for x in r['modes']}
    bad = [n for n, x in M.items() if x['verdict'] not in ('Level 1', 'n/a')]
    sp, ph, dr, rl, spi = M['short period'], M['phugoid'], M['dutch roll'], M['roll subsidence'], M['spiral']
    d = dpts[r['id']]
    L.append(f'| {r["id"]} ({d["mass"]}) | {d["V"]} | {d["gamma_deg"]:.0f} | {r["alpha_deg"]:.2f} | {r["de_deg"]:.2f} | {r["dT"]:.3f} | '
             f'{sp["wn"]:.1f} / {sp["zeta"]:.2f} | {ph["zeta"]:.3f} | {dr["wn"]:.2f} / {dr["zeta"]:.2f} | {rl["tau"]:.2f} | '
             f'{spi["t_x"]:.1f}{" (div)" if spi["lambda"][0] > 0 else ""} | {"yes" if not bad else "spiral only" if bad == ["spiral"] else ", ".join(bad)} |')
L += ['', 'Every point trims. Short period, phugoid, dutch roll and roll mode are Level 1 everywhere. The spiral '
      'diverges slowly (time to double 4.5 to 12.7 s), which misses the Level 1 time but is trivial for any '
      'roll-attitude loop; 2 to 3 deg of dihedral would cure it on the airframe if wanted. The thrust line sits 0.16 m '
      'above the CG, so adding power pitches the nose down (the full trim needs 0.8 deg less down-elevator than the '
      'static balance in the yaml\'s trim_reference).', '',
      '## Propulsion: ready for whatever you buy', '']
R = p['requirements']
L += ['### What the airframe needs (full tank, 10.5 kg, sea level)', '',
      '| case | thrust [N] | note |', '|---|---:|---|',
      f'| cruise, 20 m/s | {R["full"]["T_cruise20"]:.1f} | CL {R["full"]["CL_cruise20"]:.2f} |',
      f'| best L/D, {R["full"]["V_md"]:.1f} m/s | {R["full"]["D_min"]:.1f} | minimum propulsive power {R["full"]["P_min_prop_W"]:.0f} W at {R["full"]["V_mp"]:.1f} m/s |',
      f'| climb 2.5 m/s at 1.3 Vs ({R["full"]["V_climb"]:.1f} m/s) | {R["full"]["T_climb"]:.1f} | |',
      f'| water take-off hump (about 8 m/s) | {R["full"]["T_hump"]:.1f} | hull resistance R/W = {p["hump_R_over_W"]} (typical stepped flying-boat hull), the sizing case |',
      f'| recommended static thrust, both motors | **{p["T_required_static_recommended"]:.0f}** | hump or climb +15 %, T/W {p["T_required_static_recommended"] / (m["full"]["m"] * 9.81):.2f} |', '',
      '### Candidates', '',
      f'APC thin-electric props (published CT/CP data, `propulsion/apc/`) on generic 41xx-50xx class motors of different Kv, '
      f'6S {p["battery"]["capacity_ah"]:.0f} Ah pack ({p["battery"]["energy_wh"]:.0f} Wh, 80 % usable). Feasible means: hump thrust '
      '+15 %, at most 55 A per motor static, under the APC rpm limit, cruise below 85 % throttle, and at least 15 mm '
      'between the prop tip and the hull (15 in and 16 in props hit that limit with the nacelles where they are).', '',
      '| prop | kV | static thrust [N] | A per motor | hump thrust [N] | cruise throttle | cruise power [W] | endurance [min] | Vmax [m/s] |',
      '|---|---:|---:|---:|---:|---:|---:|---:|---:|']
for c in sorted([c for c in p['candidates'] if c['feasible']], key=lambda c: c['cruise_p_batt']):
    L.append(f'| {c["prop"]} | {c["kv"]} | {c["static_thrust_total"]:.0f} | {c["static_i_motor"]:.0f} | {c["hump_thrust_total"]:.0f} | '
             f'{c["cruise"]["throttle"]:.2f} | {c["cruise_p_batt"]:.0f} | {c["endurance_min"]:.0f} | {c["v_max"]} |')
bt = p['baseline']
bf = p['fits'][bt]
L += ['', f'The yaml carries **{bt.replace("_", " ")}**, the set the builders of this airframe fly (Hacker A50-16S, '
      'a four-blade scale prop cut to the diameter the hull clears), which is also what the Gazebo model uses: '
      f'momentum model T = {bf["momentum_a_T_static_total"]:.1f} dT^2 - {bf["momentum_b"]:.3f} dT V '
      f'(fit rms {bf["momentum_fit_rms_N"]:.1f} N). Gazebo MulticopterMotorModel for it: motorConstant '
      f'{bf["gz_motorConstant"]:.3e}, momentConstant {bf["gz_momentConstant"]:.4f}, maxRotVelocity {bf["gz_maxRotVelocity"]:.0f} rad/s.', '',
      'Each feasible candidate also has `propulsion/maps/<prop>_<kv>kv_6S_{thrust_N,battery_current_A,battery_power_W,rpm}.csv`: '
      'airspeed 0 to 30 m/s down the rows, throttle 0 to 1 across, ready for `interp2` or a lookup table block.', '',
      'When the hardware is picked:', '',
      '```bash',
      '.venv/Scripts/python flight_params/propulsion/evaluate.py --prop 14x7E --kv 450 --rm 0.056 --i0 0.96 --cells 6',
      '```', '',
      'It prints the requirement check, the momentum-model `T_static_total` and `b` for the yaml, the Gazebo motor '
      'numbers and the PX4 cruise throttle, and writes the maps. Any APC prop works once its `PER3_<name>.dat` is in '
      '`propulsion/apc/`; Rm and I0 come from the motor datasheet.', '',
      '## PX4 starting values', '',
      '| parameter | value | from |', '|---|---:|---|',
      f'| FW_AIRSPD_STALL | {a["stall"]["full"][2]:.1f} | stall, full tank |',
      f'| FW_AIRSPD_MIN | {1.2 * a["stall"]["full"][2]:.1f} | 1.2 Vs full |',
      '| FW_AIRSPD_TRIM | 20.0 | design cruise |',
      f'| FW_AIRSPD_MAX | 28.0 | about 2 Vs; level-flight Vmax runs {min(c["v_max"] for c in p["candidates"] if c["feasible"]):.0f} to '
      f'{max(c["v_max"] for c in p["candidates"] if c["feasible"]):.0f} m/s across the candidates, so check yours |',
      f'| FW_THR_TRIM | {D["trim_reference"]["loaded"]["dT"]:.2f} | baseline propulsion, full tank (re-run evaluate.py for yours) |',
      f'| FW_PSP_OFF | {D["trim_reference"]["loaded"]["alpha_deg"]:.1f} | trim pitch at cruise (deg) |', '',
      '## Assumptions and limits', '',
      '- Aero is linear in alpha up to the stall blend (the team model\'s sigmoid). AVL has no viscous effects beyond '
      'what XFOIL adds to drag and control power; hull lift and side force come only from AVL\'s slender-body model.',
      '- No propeller slipstream or prop normal force, no ground or water effect on the aero, no flaps.',
      '- XFOIL assumes a clean model (Ncrit 9). A rough, painted or fabric surface will have more drag and a lower cl_max.',
      '- The water take-off hump uses R/W = 0.20, a typical value for stepped flying-boat hulls, not a tank test of this '
      'hull. Spray, waves and the chine shape can push it higher, hence the 15 % margin.',
      '- The motor model is generic (Rm and I0 scaled from one 41xx class point). Swap in datasheet values with '
      '`evaluate.py`. The momentum model is a fit; the CSV maps are the better source.',
      '- The Gazebo model in `cl415_description` still runs its four LiftDrag elements from the earlier hand estimates; '
      'these AVL/XFOIL numbers can replace them (or drive an AdvancedLiftDrag block) as a next step.']
open(os.path.join(HERE, 'README.md'), 'w', encoding='utf-8').write('\n'.join(L) + '\n')
print('\n'.join(L))
