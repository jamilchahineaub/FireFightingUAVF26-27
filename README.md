# prandtl-uav — flight control for the subscale amphibious PrandtlPlane

AUB VIP Research, supervisor Dr. Bilal Kaddouh. Team: Jamil Chahine (control lead), Zahi (control, simulation), Michael/Tohme (simulation, onboarding).

Goal for this semester: a flight controller designed against explicit specifications on a verified model, ported to PX4, flown in Gazebo Harmonic, and landing on water inside a defined touchdown envelope. Method over luck: nothing goes into the simulator that has not passed the MATLAB tests first. The full plan is in the onboarding document and the two sprint plans (`docs/`).

## Layout

```
params/     params.yaml (single source of truth), yaml2json.py, schema.md
matlab/     model, trim, linearisation, mode analysis, PX4 export
  util/     quaternions, ISA, RK4, state builder
tests/      run_all_tests.m + one test per check (MATLAB and Octave)
sim/        gen_sdf.py (Jinja2 -> Gazebo SDF), templates/, test_gen_sdf.py, generated/
docs/       decisions.md (decision log), versions.md, pretest_checklist.md
results/    lin_points.mat (six design points), other outputs (git-ignored)
```

## Quick start

```bash
pip install pyyaml jinja2            # once
make params                          # params.yaml -> params.json
make sdf                             # generate the Gazebo model SDFs
make test-octave                     # or `make test` with MATLAB
```

In MATLAB/Octave:

```matlab
addpath matlab matlab/util
P = load_params();                                   % everything from params.yaml
[xt, ut, info] = trim(P, struct('V', 20, 'gamma', 0, 'mass', 'loaded'));
L = linearize(xt, ut, P, struct('mass', 'loaded'));  % A, B, lon/lat blocks, lon6 legacy form
T = modes(L, P);                                     % mode table vs MIL-F-8785C
G = tf_extract(L, struct('print', true));            % q/de, theta/de, p/da, phi/da, V/dT, ...
R = run_six_points();                                % all six design points -> results/
```

## Model in one paragraph

Rigid-body 6-DOF in FRD body axes with a quaternion attitude and the full inertia tensor (`eom.m`). Aerodynamics in stability-derivative form with a sigmoid stall blend calibrated to CL_max, the sizing-tool drag polar, and a PrandtlPlane mixing hook (`aero.m`). Two thrust models (`propulsion.m`): the legacy constant-power law that reproduces the May-2026 trims, and a momentum-theory law anchored to the real installed power (see `docs/decisions.md` D-02). First-order servos with position and rate limits (`actuator.m`). A hydrodynamic hook (`hydro_stub.m`) for the water-landing phase.

## Status (28 Sep 2026)

- 9/9 tests pass under Octave 8.4 (`make test-octave`).
- Static balance reproduces the report's Table 3 exactly; full trim deviations are explained by thrust tilt and the thrust-line moment (D-01).
- Linear vs nonlinear doublet overlays agree within 1 % on rates and attitude; unloaded M_δT = +1.03 rad/s² (report +1.02).
- Six design points trimmed and linearised; mode table printed by `run_six_points`.
- Lateral derivatives are Aerosonde placeholders (D-05). Inertia is a mass-station model (D-06). AVL and CAD replace both.

## Rules

1. Numbers live in `params.yaml`. If you typed a number into a `.m` or `.sdf`, you did it wrong.
2. Every result states the `params.yaml` sha256 and the propulsion model it used.
3. A test that fails is fixed, not deleted.
4. Anything tagged `placeholder` is not a finding.
