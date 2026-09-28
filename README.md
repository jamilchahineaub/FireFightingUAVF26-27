# prandtl-uav

Flight control for the 1:11 amphibious PrandtlPlane UAV (AUB VIP, Dr. Bilal Kaddouh).

Target this semester: controller designed and verified in MATLAB, ported to PX4, flown in Gazebo Harmonic, landing on water.

Task list: [TASKS.md](TASKS.md). Decisions: [docs/decisions.md](docs/decisions.md).

## Layout

```
params/   params.yaml, yaml2json.py, schema.md
matlab/   model, trim, linearize, modes, PX4 export
tests/    run_all_tests.m and one file per test
sim/      gen_sdf.py, templates/, test_gen_sdf.py
docs/     decisions.md, versions.md, pretest_checklist.md
results/  generated, not tracked
```

## Setup

```bash
pip install pyyaml jinja2
make params        # params.yaml -> params.json
make sdf           # sim/generated/prandtl_amph_{loaded,unloaded}.sdf
make test          # MATLAB
make test-octave   # Octave
```

```matlab
addpath matlab matlab/util
P = load_params();
[xt, ut, info] = trim(P, struct('V', 20, 'gamma', 0, 'mass', 'loaded'));
L = linearize(xt, ut, P, struct('mass', 'loaded'));
modes(L, P);
tf_extract(L, struct('print', true));
run_six_points();
```

## Conventions

- All numbers live in `params/params.yaml`. Don't hard-code values in `.m` or `.sdf` files.
- Body frame FRD, inertial NED. Gazebo is FLU; conversion happens only in `sim/gen_sdf.py`.
- Every result records the `params.yaml` sha256 and the propulsion model used.
- Values tagged `aerosonde-placeholder` are not PrandtlPlane data.
