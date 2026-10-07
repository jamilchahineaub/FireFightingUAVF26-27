# Tasks

<<<<<<< HEAD
`[x]` done and tested, `[ ]` open. Tick items in the PR that completes them.

## Zahi: controller infrastructure

- [x] Repo skeleton (`params/ matlab/ tests/ sim/ docs/`)
- [x] `params.yaml` schema v1: geometry, both mass configs, aero derivatives with source tags, propulsion, actuators, trim references, sign conventions
- [ ] Review schema v1 and tell Michael it is frozen
- [x] YAML to JSON conversion (`make params`) and `load_params.m` with sha256
- [ ] Install the pre-commit hook (`make hooks`) on every machine
- [x] Record the two data inconsistencies in `docs/decisions.md` (D-01, D-02)
- [ ] Get Dr. Kaddouh's decision on D-02 (propulsion model for design)
- [x] `eom.m`, `aero.m`, `propulsion.m`, `actuator.m`, `hydro_stub.m` with unit tests
- [x] `trim.m`; regression against report Table 3 (`test_trim_cruise`)
- [x] `linearize.m` with nonlinear vs linear doublet overlay test
- [x] `modes.m` against MIL-F-8785C, raw and Froude-scaled
- [x] Trim and linearize at the six design points, save `results/lin_points.mat`
- [ ] Write the transfer-function table to `results/tf_table.md`
- [x] `run_all_tests.m` with PASS/FAIL and yaml hash
- [x] `px4_export.m` (QGC format)
- [ ] Confirm PX4 parameter names in Michael's SITL: `param show FW_RR* FW_PR* FW_Y* FW_LND*`
- [x] Lateral derivatives from Aerosonde, tagged as placeholders
- [ ] Run the full suite in MATLAB (only verified in Octave so far)

Next, after the above:

- [ ] Pitch-rate and roll-rate PI+FF at C14, A14, C10, A10 (PM >= 45 deg, GM >= 6 dB)
- [ ] Attitude loops, >= 5x separation from rate loops
- [ ] Robustness sweep: Cm_alpha, Cm_q x {0.7, 1, 1.3}, mass x {0.9, 1, 1.1}
- [ ] Write `sim/px4_params.params`

## Michael: Gazebo / PX4 environment

- [ ] Ubuntu 24.04 (or 22.04), PX4-Autopilot v1.17 cloned recursively, `Tools/setup/ubuntu.sh`, `make px4_sitl`
- [x] Gazebo Harmonic only; `gz sim --versions` shows no Ionic/Jetty
- [ ] Fill in `docs/versions.md` (OS, PX4 tag, models submodule commit)
- [ ] QGroundControl, PlotJuggler, `pip install pyulog mavsdk pymavlink jinja2 pyyaml`
- [ ] Smoke tests S1-S8 in `docs/pretest_checklist.md`, in order
- [ ] Check the Gazebo build has the AdvancedLiftDrag pitching-moment fix (gz-sim #3695)
- [ ] Water world W1: 10 kg box, one buoyancy grade, DART with bullet collision, 2 ms step. Draft m/(rho A) within 2 %, heave freq sqrt(rho g A/m) within 5 %, RTF >= 1
- [ ] W1b: add Hydrodynamics damping, zeta about 0.3
- [ ] Copy `advanced_plane` to `prandtl_amph` and airframe to `4099_gz_prandtl_amph`, register in CMake, fly it unchanged
- [x] `sim/gen_sdf.py` with parse-back test (mass, CG, inertia)
- [ ] Move the generated SDF into the PX4 model folder; copy the `<control_surface>` and motor blocks from `advanced_plane`
- [ ] Add the downward lidar from `x500_lidar_down`; write down the `SIM_GZ_*` channel mapping
- [ ] `sim/touchdown.py`: sink rate, pitch, roll, airspeed at land-detected
- [ ] `sim/run_batch.py`: headless launch, MAVSDK mission, wait for landed, copy log
- [ ] W2: multi-box hull floats at correct draft and pitch at 14 and 10 kg
- [ ] Pre-test checklist, all 28 items

## Handoffs

- Zahi to Michael: frozen schema, then the propulsion decision so the Gazebo motor matches T(V, dT)
- Michael to Zahi: running SITL for parameter names; AdvancedLiftDrag vs `aero.m` parity (same alpha, V: lift and Cm within 5 %)
- Both: agree surface deflection limits and sign conventions (used by `px4_export.m` and SDF joint limits)

## Questions for Dr. Kaddouh

1. Propulsion model for design (D-02). Any motor/prop data?
2. Control surface layout on front and rear wings.
3. SolidWorks delivery date (inertia tensor, geometry).
4. Touchdown envelope numbers and Monte Carlo pass threshold.
=======
## Michael: simulation

- [x] Set up PX4 + Gazebo Harmonic (Ubuntu 24.04, PX4 v1.17; check the stock Cessna flies and logs open in PlotJuggler)
- [ ] Get our plane flying in Gazebo (copy `advanced_plane`, load the SDF from `make sdf`, add control surfaces and both motors)
- [ ] Make it float (water world at z = 0, hull as a few boxes, DART physics; a plain box should sit at the right depth first)
- [ ] Check Gazebo aero matches MATLAB (same angle of attack and speed should give the same lift and pitching moment, within 5 %)
- [ ] Landing test tools (script that pulls sink rate, pitch, roll, airspeed at touchdown from the log; batch runner for repeated landings)

## Zahi: controller

- [x] Aircraft model in MATLAB (6-DOF, trim, linearization, modes, all tested)
- [ ] Design the inner loops (pitch and roll rate, then attitude; phase margin 45 deg, gain margin 6 dB)
- [ ] Design altitude/airspeed and heading control (TECS and path following, like PX4)
- [ ] Robustness check (vary pitch derivatives by 30 % and mass by 10 %, margins must hold)
- [ ] Export gains to PX4 (`px4_export.m`; confirm parameter names in Michael's sim)

## Together

- [ ] Fly our controller in Gazebo (cruise, turns, approach)
- [ ] Water landing (constant attitude, slow sink, about 100 runs with wind)
- [ ] Replace placeholder data (AVL aero model, SolidWorks inertia)
- [ ] Later: move to AirSim/UE5 for high-fidelity water and RL

## Questions for Dr. Kaddouh

1. Which thrust model to use (the report's cruise throttle doesn't match the installed power; see `docs/decisions.md` D-02)
2. Control surface layout on the front and rear wings
3. When the SolidWorks model will be ready
4. Touchdown limits (sink rate, pitch) and how many landings must pass
>>>>>>> 1acc67952a327054086ece17dbc08b860afa30ae
