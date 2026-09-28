# params.yaml schema (version 1)

Frozen 2026-09-28. Structural changes (new sections, renamed keys) bump
`meta.schema_version` and need a note in `docs/decisions.md`. Adding a value
inside an existing section, or changing a number, does not.

| Section | Keys | Read by |
|---|---|---|
| `meta` | schema_version, aircraft, units, body_frame (FRD), inertial_frame (NED). `sha256` and `source_file` are added by `yaml2json.py`, never written by hand. | everything |
| `sign_conventions` | text definitions of δe, δa, δr, δL, throttle, α, β and the PX4 note | humans, `px4_export.m` header |
| `environment` | g, rho_sl, rho_water, isa flag | `eom.m`, `isa_density.m`, `gen_sdf.py` |
| `geometry` | b, S, c_bar, AR, oswald_e, L_fus | `aero.m`, `mass_props.m`, SDF |
| `mass_properties` | use_cad, airframe {m, m_rounded, kx_over_b, ky_over_Lfus}, payload {m, r_from_airframe_cg_frd}, loaded/unloaded {m, m_rounded}, use_rounded_mass, Ixz_airframe, cad {...} | `mass_props.m`, `gen_sdf.py` (mirror) |
| `aero.stall` | CL_max, blend_M | `load_params.m` (calibrates alpha_stall), SDF |
| `aero.drag_polar` | CD0, K | `aero.m`, SDF |
| `aero.longitudinal` | each `{value, source[, note]}`: CL0 CL_alpha CL_q CL_de CL_dL CD_de Cm0 Cm_alpha Cm_q Cm_de Cm_dL | `aero.m`, SDF |
| `aero.lateral` | each `{value, source}`: CY0 Cl0 Cn0 CY_beta Cl_beta Cn_beta CY_p Cl_p Cn_p CY_r Cl_r Cn_r CY_da Cl_da Cn_da CY_dr Cl_dr Cn_dr | `aero.m`, SDF |
| `aero.mixing` | Mix (2×2), note | SDF / PX4 allocation |
| `propulsion` | n_motors, P_elec_max_*, eta_p, model, legacy_constant_power {...}, momentum {...}, thrust_line {...} | `propulsion.m`, `mass_props.m`, SDF |
| `actuators` | elevator/aileron/rudder/direct_lift {tau, delta_max_deg, rate_max_deg_s}, throttle {tau, min, max} | `actuator.m`, `px4_export.m`, SDF joint limits |
| `speeds` | V_cruise, V_stall_loaded, V_stall_unloaded, approach_factor, slow_factor | design scripts |
| `trim_reference` | altitude_m, cruise_loaded/unloaded {V, mass, gamma_deg, CL, alpha_deg, de_deg, thrust_N, dT}, tolerance, full_trim_tolerance | `test_trim_cruise.m` |
| `design_points` | list of {id, mass, V, gamma_deg, basis} | `run_six_points.m` |
| `hydro` | enabled, water_z_ned, hull_boxes [{pose, size}] | `eom.m` hook, SDF collisions |
| `control_specs` | margins, separation, step sizes, robustness grid | design scripts (Day 2) |

## Source tags

`handbook` · `fit-to-reference-trims` · `aerosonde-placeholder` · `avl` · `cad` · `sizing tool`.
A result may only be presented as a PrandtlPlane property when every coefficient it depends on is `avl`, `cad` or `sizing tool`.

## Workflow

```
edit params/params.yaml
make params            # -> params/params.json (+ sha256)
make sdf               # -> sim/generated/prandtl_amph_{loaded,unloaded}.sdf + .json sidecars
make test-octave       # or: make test (MATLAB)
```
