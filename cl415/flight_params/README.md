# CL-415 UAV: flight simulation parameters

Everything a flight sim needs for the 2.51 m CL-415 UAV, worked out from the CAD and the SolidWorks mass properties. `cl415_params.yaml` is a drop-in file for the team's `params.yaml` schema v1 (FireFightingUAVF26-27, FRD body axes); `cl415_flight_data.json` has every number below plus the extras (empty-CG reference, airfoil polars, all propulsion candidates).

## Files

| file | what |
|---|---|
| `cl415_params.yaml`, `.json` | the team schema: geometry, mass (SolidWorks), aero derivatives, propulsion, actuators, speeds, trims, design points |
| `cl415_flight_data.json` | all numbers, both CG references, AVL trims, airfoil data, every propulsion candidate |
| `airfoils/` | XFOIL polars (`polars/*.csv`), plots, `report.md` with the section properties per Reynolds number |
| `avl/` | the AVL model (`cl415_full.avl`, `cl415_empty.avl`, body files) and its stability-derivative outputs |
| `propulsion/` | APC prop data, the motor/prop/battery model, thrust maps for each feasible candidate, `evaluate.py` |
| `matlab_check/` | the team's trim/linearize/modes run on these numbers (`check_output.txt`) |
| `plots/aero_and_thrust.png` | stall, drag polar, thrust required vs available |

Rebuild: `run_xfoil.py` (airfoils), `avl/build_avl.py`, `build_flight_params.py`, then `matlab -batch "run('flight_params/matlab_check/run_check.m')"` and `write_report.py`.

## Reference values

| | value |
|---|---:|
| span b | 2.51 m |
| wing area S | 0.698 m^2 |
| mean chord c | 0.278 m |
| aspect ratio | 9.025 |
| wing incidence to the hull datum | 2.65 deg |
| h-stab / fin / finlets area | 0.2237 / 0.1264 / 0.04803 m^2 |
| aileron span, chord ratio | y 0.793 to 1.195 m, 0.25 |
| elevator / rudder chord ratio | 0.40 / 0.31 (root) to 0.45 (tip) |
| hull length | 1.688 m |
| prop diameter (CAD) | 0.3554 m |
| prop hubs from the full-tank CG (FRD) | x +0.265, y +/-0.297, z -0.161 m (0.16 m above the CG) |

## Mass and inertia (SolidWorks)

FRD axes about each case's own CG, team convention J = [Ixx 0 -Ixz; 0 Iyy 0; -Ixz 0 Izz], Ixz = +integral(x z dm).

| | full tank | empty |
|---|---:|---:|
| mass [kg] | 10.50000000000788 | 7.5000000000114255 |
| CG [% MAC] | 27.1 | 28.0 |
| CG (x, z) from the wing LE root, REP-103 [m] | (-0.0754, -0.1007) | (-0.0779, -0.0659) |
| Ixx [kg m^2] | 1.2097 | 1.1698 |
| Iyy [kg m^2] | 0.8972 | 0.8529 |
| Izz [kg m^2] | 1.8900 | 1.8744 |
| Ixz [kg m^2] | 0.0488 | 0.0466 |

Ixy and Iyz are zero to within 1e-5 (symmetric aircraft).

## Aerodynamic model

Stability and control derivatives from AVL 3.52 (vortex lattice) on the CAD geometry: NACA 4417 camber on the wing, lift-slope factor from XFOIL per surface, hull and nacelles as slender bodies. All per radian, body alpha (alpha = 0 is the hull datum, the wing sits at +2.65 deg), rates as p b/2V, q c/2V, r b/2V. Control derivatives are AVL's inviscid values times the XFOIL viscous flap factor: aileron 0.830, elevator 0.903 (also covers the rudder cut-outs AVL doesn't see), rudder 0.832.

### Longitudinal

| | about full-tank CG | about empty CG | quick hand check |
|---|---:|---:|---|
| CL0 | 0.5105 | 0.5105 | 2.65 deg incidence + camber, less tail download |
| CL_alpha | 5.5928 | 5.5928 | wing alone 4.85 (Helmbold), plus tail |
| CL_q | 9.9840 | 10.1425 |  |
| CL_de | 0.7431 | 0.7431 |  |
| Cm0 | 0.0554 | 0.0563 |  |
| Cm_alpha | -1.5343 | -1.4469 | static margin 27 % (full), neutral point 55 % MAC |
| Cm_q | -20.8295 | -20.7283 | tail only -2 eta V_H a_t l_t/c = -18.0 |
| Cm_de | -2.1020 | -2.0959 |  |
| CD_de | 0.0105 | same | XFOIL elevator drag at 10 deg |

### Lateral-directional

| | about full-tank CG | about empty CG | without hull/nacelle bodies |
|---|---:|---:|---:|
| CY_beta | -0.4699 | -0.4699 | -0.4826 |
| Cl_beta | -0.0601 | -0.0536 | -0.0477 |
| Cn_beta | 0.0720 | 0.0715 | 0.1520 |
| CY_p | -0.0094 | 0.0036 | -0.0163 |
| Cl_p | -0.5492 | -0.5476 | -0.5485 |
| Cn_p | -0.0294 | -0.0314 | -0.0304 |
| CY_r | 0.2022 | 0.2013 | 0.3536 |
| Cl_r | 0.1809 | 0.1780 | 0.1762 |
| Cn_r | -0.1198 | -0.1195 | -0.1183 |
| CY_da | -0.0113 | -0.0113 | -0.0113 |
| Cl_da | 0.2083 | 0.2085 | 0.2076 |
| Cn_da | -0.0123 | -0.0123 | -0.0123 |
| CY_dr | 0.2221 | 0.2221 | 0.2249 |
| Cl_dr | 0.0242 | 0.0211 | 0.0245 |
| Cn_dr | -0.0800 | -0.0797 | -0.0810 |

Spiral parameter Clb Cnr / (Clr Cnb) = 0.55 (below 1, so the spiral mode is slightly divergent: a straight wing with no dihedral). The hull as an AVL slender body costs about 0.080 of Cn_beta; slender-body theory overstates that on a short beamy hull, so treat the two columns as the bounds on Cn_beta (the yaml uses the lower one).

### Drag, stall, speeds

- Drag polar CD = **0.0328 + 0.0380 CL^2** (Oswald e = 0.93), fitted over CL 0.1 to 1.25 to: AVL Trefftz-plane induced drag (tail included) + XFOIL 4417 profile drag at the cruise Re x1.10 for seams and gaps + XFOIL tail-section drag + hull, nacelles and floats by the Raymer component method + 5 % misc. At cruise the tails add 0.0049 and the bodies 0.0136.
- That e is optimistic next to typical flight-test values (Raymer's empirical estimate for this wing is 0.78); for robustness runs try K x1.2 and CD0 x1.2.
- CL_max = **1.236** (critical section: the first wing strip in AVL to reach the XFOIL 4417 cl_max x0.92 at the stall Reynolds number), at 7.7 deg body alpha.
- Stall speed 13.9 m/s full, 11.8 m/s empty (sea level).
- Best L/D 14.2 at 16.1 m/s (full); minimum power at 15.3 m/s.

## Airfoils (XFOIL, Ncrit 9)

| section | Re (20 m/s) | alpha_L0 | cl_alpha [/rad] | cl_max | cd_min | cm c/4 | x_ac/c | control tau (XFOIL / thin airfoil) |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| NACA 4417 (aileron) | 400k | -4.32 deg | 6.03 | 1.547 at 14.5 deg | 0.0094 | -0.1046 | 0.234 | 0.505 / 0.609 |
| NACA 0016 (elevator) | 300k | 0.00 deg | 6.10 | 1.242 at 17.5 deg | 0.0091 | 0.0000 | 0.226 | 0.754 / 0.746 |
| NACA 0012 (rudder) | 400k | -0.00 deg | 6.83 | 1.204 at 14.0 deg | 0.0067 | 0.0000 | 0.253 | 0.609 / 0.732 |

All Reynolds numbers (150k to 600k), cl_min, (cl/cd)max and the polar fits are in `airfoils/report.md`.

## Check in the team's 6-DOF code

`matlab_check/run_check.m` loads `cl415_params.json` into the unchanged FireFightingUAVF26-27 MATLAB tools and runs the full nonlinear trim (thrust tilt and thrust-line moment included), the linearisation and the MIL-F-8785C mode check at the six design points (the Optimization Toolbox isn't installed here, so a small Levenberg-Marquardt stand-in replaces fsolve for this check only).

| point | V [m/s] | gamma | alpha [deg] | de [deg] | throttle | short period wn / zeta | phugoid zeta | dutch roll wn / zeta | roll tau [s] | spiral t2 [s] | Level 1 |
|---|---:|---:|---:|---:|---:|---|---:|---|---:|---:|---|
| CL (loaded) | 20.0 | 0 | 0.99 | 0.05 | 0.524 | 10.5 / 0.58 | 0.110 | 4.31 / 0.25 | 0.08 | 9.8 (div) | spiral only |
| AL (loaded) | 18.1 | -4 | 2.40 | -0.27 | 0.348 | 9.5 / 0.58 | 0.104 | 3.92 / 0.25 | 0.09 | 12.4 (div) | yes |
| SL (loaded) | 15.3 | 0 | 5.98 | -4.03 | 0.448 | 7.9 / 0.57 | 0.077 | 3.37 / 0.27 | 0.11 | 5.0 (div) | spiral only |
| CU (unloaded) | 20.0 | 0 | -1.00 | 1.74 | 0.508 | 11.4 / 0.63 | 0.152 | 4.35 / 0.25 | 0.08 | 12.7 (div) | spiral only |
| AU (unloaded) | 15.3 | -4 | 2.39 | -0.26 | 0.294 | 8.7 / 0.63 | 0.102 | 3.38 / 0.26 | 0.10 | 10.9 (div) | spiral only |
| SU (unloaded) | 13.0 | 0 | 5.80 | -3.63 | 0.380 | 7.3 / 0.62 | 0.072 | 2.94 / 0.28 | 0.12 | 4.5 (div) | spiral only |

Every point trims. Short period, phugoid, dutch roll and roll mode are Level 1 everywhere. The spiral diverges slowly (time to double 4.5 to 12.7 s), which misses the Level 1 time but is trivial for any roll-attitude loop; 2 to 3 deg of dihedral would cure it on the airframe if wanted. The thrust line sits 0.16 m above the CG, so adding power pitches the nose down (the full trim needs 0.8 deg less down-elevator than the static balance in the yaml's trim_reference).

## Propulsion: ready for whatever you buy

### What the airframe needs (full tank, 10.5 kg, sea level)

| case | thrust [N] | note |
|---|---:|---|
| cruise, 20 m/s | 8.0 | CL 0.60 |
| best L/D, 16.1 m/s | 7.3 | minimum propulsive power 112 W at 15.3 m/s |
| climb 2.5 m/s at 1.3 Vs (18.1 m/s) | 21.7 | |
| water take-off hump (about 8 m/s) | 25.3 | hull resistance R/W = 0.2 (typical stepped flying-boat hull), the sizing case |
| recommended static thrust, both motors | **29** | hump or climb +15 %, T/W 0.28 |

### Candidates

APC thin-electric props (published CT/CP data, `propulsion/apc/`) on generic 41xx-50xx class motors of different Kv, 6S 16 Ah pack (355 Wh, 80 % usable). Feasible means: hump thrust +15 %, at most 55 A per motor static, under the APC rpm limit, cruise below 85 % throttle, and at least 15 mm between the prop tip and the hull (15 in and 16 in props hit that limit with the nacelles where they are).

| prop | kV | static thrust [N] | A per motor | hump thrust [N] | cruise throttle | cruise power [W] | endurance [min] | Vmax [m/s] |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 13x10E | 380 | 51 | 23 | 47 | 0.63 | 248 | 69 | 32.5 |
| 13x10E | 450 | 68 | 35 | 63 | 0.54 | 248 | 69 | 38.0 |
| 13x10E | 530 | 89 | 53 | 83 | 0.45 | 248 | 69 | 44.0 |
| 14x10E | 450 | 81 | 42 | 74 | 0.52 | 255 | 67 | 39.0 |
| 14x10E | 380 | 61 | 27 | 55 | 0.61 | 255 | 67 | 33.25 |
| 13x8E | 380 | 49 | 19 | 42 | 0.74 | 268 | 64 | 28.0 |
| 13x8E | 530 | 87 | 46 | 78 | 0.53 | 268 | 64 | 38.5 |
| 13x8E | 450 | 66 | 30 | 58 | 0.62 | 268 | 64 | 33.0 |
| 11x9-4 | 380 | 47 | 22 | 42 | 0.68 | 269 | 63 | 30.25 |
| 11x9-4 | 450 | 62 | 34 | 57 | 0.58 | 269 | 63 | 35.75 |
| 11x9-4 | 530 | 81 | 52 | 75 | 0.49 | 269 | 63 | 41.25 |
| 14x85E | 450 | 78 | 37 | 69 | 0.58 | 270 | 63 | 35.25 |
| 14x85E | 380 | 58 | 24 | 51 | 0.69 | 270 | 63 | 30.0 |
| 13.5x10.5-4 (scaled 155x12-4) | 365 | 93 | 47 | 85 | 0.56 | 286 | 60 | 36.75 |
| 12x7E-3 | 530 | 88 | 48 | 76 | 0.58 | 295 | 58 | 35.0 |
| 12x7E-3 | 380 | 50 | 20 | 41 | 0.81 | 295 | 58 | 25.25 |
| 12x7E-3 | 450 | 67 | 32 | 57 | 0.68 | 295 | 58 | 30.0 |
| 14x7E | 530 | 97 | 50 | 85 | 0.56 | 299 | 57 | 36.25 |
| 14x7E | 450 | 74 | 33 | 63 | 0.66 | 299 | 57 | 31.0 |
| 14x7E | 380 | 55 | 21 | 46 | 0.78 | 299 | 57 | 26.25 |
| 11x6-4 | 530 | 71 | 38 | 61 | 0.65 | 324 | 53 | 31.75 |
| 11x6-4 | 450 | 53 | 25 | 44 | 0.76 | 324 | 53 | 27.0 |

The yaml carries **13.5x10.5-4 hacker a50 16s 6S**, the set the builders of this airframe fly (Hacker A50-16S, a four-blade scale prop cut to the diameter the hull clears), which is also what the Gazebo model uses: momentum model T = 102.2 dT^2 - 1.921 dT V (fit rms 2.5 N). Gazebo MulticopterMotorModel for it: motorConstant 7.929e-05, momentConstant 0.0253, maxRotVelocity 767 rad/s.

Each feasible candidate also has `propulsion/maps/<prop>_<kv>kv_6S_{thrust_N,battery_current_A,battery_power_W,rpm}.csv`: airspeed 0 to 30 m/s down the rows, throttle 0 to 1 across, ready for `interp2` or a lookup table block.

When the hardware is picked:

```bash
.venv/Scripts/python flight_params/propulsion/evaluate.py --prop 14x7E --kv 450 --rm 0.056 --i0 0.96 --cells 6
```

It prints the requirement check, the momentum-model `T_static_total` and `b` for the yaml, the Gazebo motor numbers and the PX4 cruise throttle, and writes the maps. Any APC prop works once its `PER3_<name>.dat` is in `propulsion/apc/`; Rm and I0 come from the motor datasheet.

## PX4 starting values

| parameter | value | from |
|---|---:|---|
| FW_AIRSPD_STALL | 13.9 | stall, full tank |
| FW_AIRSPD_MIN | 16.7 | 1.2 Vs full |
| FW_AIRSPD_TRIM | 20.0 | design cruise |
| FW_AIRSPD_MAX | 28.0 | about 2 Vs; level-flight Vmax runs 25 to 44 m/s across the candidates, so check yours |
| FW_THR_TRIM | 0.52 | baseline propulsion, full tank (re-run evaluate.py for yours) |
| FW_PSP_OFF | 0.9 | trim pitch at cruise (deg) |

## Assumptions and limits

- Aero is linear in alpha up to the stall blend (the team model's sigmoid). AVL has no viscous effects beyond what XFOIL adds to drag and control power; hull lift and side force come only from AVL's slender-body model.
- No propeller slipstream or prop normal force, no ground or water effect on the aero, no flaps.
- XFOIL assumes a clean model (Ncrit 9). A rough, painted or fabric surface will have more drag and a lower cl_max.
- The water take-off hump uses R/W = 0.20, a typical value for stepped flying-boat hulls, not a tank test of this hull. Spray, waves and the chine shape can push it higher, hence the 15 % margin.
- The motor model is generic (Rm and I0 scaled from one 41xx class point). Swap in datasheet values with `evaluate.py`. The momentum model is a fit; the CSV maps are the better source.
- The Gazebo model in `cl415_description` still runs its four LiftDrag elements from the earlier hand estimates; these AVL/XFOIL numbers can replace them (or drive an AdvancedLiftDrag block) as a next step.
