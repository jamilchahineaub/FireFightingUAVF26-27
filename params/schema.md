# params.yaml schema v1

Adding or changing a value: just edit. Adding, removing or renaming a key: bump `meta.schema_version` and add a line to `docs/decisions.md`.

| Section | Contents | Used by |
|---|---|---|
| `meta` | version, frames. `sha256` is added by `yaml2json.py` | all |
| `sign_conventions` | de, da, dr, dL, throttle, alpha, beta | reference |
| `environment` | g, rho_sl, rho_water, isa | `eom.m`, SDF |
| `geometry` | b, S, c_bar, AR, oswald_e, L_fus | `aero.m`, `mass_props.m`, SDF |
| `mass_properties` | airframe, payload, loaded/unloaded, Ixz, cad block | `mass_props.m`, `gen_sdf.py` |
| `aero.stall`, `aero.drag_polar` | CL_max, blend_M, CD0, K | `aero.m`, SDF |
| `aero.longitudinal`, `aero.lateral` | `{value, source}` per derivative | `aero.m`, SDF |
| `aero.mixing` | 2x2 front/rear mix | SDF, PX4 allocation |
| `propulsion` | model choice, both model parameters, thrust line | `propulsion.m`, `mass_props.m` |
| `actuators` | tau, delta_max_deg, rate_max_deg_s | `actuator.m`, `px4_export.m`, SDF |
| `speeds` | cruise, stall, approach and slow factors | design scripts |
| `trim_reference` | report Table 3 and tolerances | `test_trim_cruise.m` |
| `design_points` | C14 A14 S14 C10 A10 S10 | `run_six_points.m` |
| `hydro` | enabled, water level, hull boxes | `eom.m`, SDF |
| `control_specs` | margins, separation, robustness grid | loop design |

Source tags: `sizing tool`, `handbook`, `fit-to-reference-trims`, `aerosonde-placeholder`, `avl`, `cad`.
