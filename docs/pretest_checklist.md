# Pre-test verification checklist (Plan 2, section 2.9)

Owner: Michael. Mark each item PASS/FAIL with the date. Nothing flies a control test until every row is PASS on the pinned stack.

| # | Item | Pass criterion | Status | Date |
|---|---|---|---|---|
| 1 | `lsb_release -a` recorded in versions.md | Ubuntu 24.04 (or 22.04) | | |
| 2 | `gz sim --versions` | Harmonic only, no Ionic/Jetty | | |
| 3 | PX4 `git describe --tags` recorded | v1.17.x | | |
| 4 | PX4-gazebo-models submodule commit recorded | | | |
| 5 | gz-sim build includes the AdvancedLiftDrag pitching-moment fix (gz-sim #3695) | yes | | |
| 6 | YAML sha256 in the SDF header | matches `params.json` | | |
| 7 | YAML sha256 in the PX4 log (boot message or user param) | present | | |
| 8 | `gz sdf -k model.sdf` | OK | | |
| 9 | SDF mass/CG/inertia vs YAML (`python3 sim/test_gen_sdf.py`) | PASS | | |
| 10 | Aero coefficients in SDF vs YAML | exact | | |
| 11 | QGC actuator test: +pitch command | nose-up moment (elevator TE up) | | |
| 12 | +roll command | right-wing-down moment | | |
| 13 | +yaw command | nose-right moment | | |
| 14 | Each motor | forward thrust, opposite spin, no net yaw | | |
| 15 | Airspeed sensor | ≈0 at rest; within 0.5 m/s of gz velocity in tow | | |
| 16 | `ekf2 status` | no innovation failures | | |
| 17 | Arming checks | arms without overrides | | |
| 18 | Real-time factor | ≥ 1.0 | | |
| 19 | gz console `-v 4` | no repeating physics warnings | | |
| 20 | Water plane at z=0, plain box (W1) | draft = m/(ρA) within 2 % | | |
| 21 | Multi-box hull at 14 and 10 kg (W2) | draft and static pitch match hand calc | | |
| 22 | Damping sign (W1b) | heave decays, ζ ≈ 0.3 | | |
| 23 | Land detector on water | `landed` flips within LNDFW_TRIG_TIME + 1 s | | |
| 24 | `distance_sensor` | sane, draft-corrected | | |
| 25 | `param show` vs `sim/px4_params.params` | identical | | |
| 26 | `ulog_info` topics | vehicle_local_position, vehicle_attitude, airspeed_validated, actuator_outputs, vehicle_land_detected, wind | | |
| 27 | Cessna regression (S2) | takeoff → loiter → land | | |
| 28 | Two batch runs, same seed | touchdown metrics within 1 % | | |

## Smoke tests (Plan 2, section 2.3)

| # | Action | Pass | Status |
|---|---|---|---|
| S1 | `make px4_sitl gz_rc_cessna` | `pxh>` prompt, no ERROR, QGC connects | |
| S2 | QGC mission takeoff → loiter → land | ≥2 laps, lands, disarms | |
| S3 | `make px4_sitl gz_advanced_plane` | flies S2 | |
| S4 | Newest `.ulg` opens in PlotJuggler + Flight Review | | |
| S5 | `gz topic -e -t /world/default/stats -n 5` | RTF ≥ 1.0 | |
| S6 | `gz topic -l` | clock, joint/servo/motor, IMU, air-pressure topics | |
| S7 | `HEADLESS=1 make px4_sitl gz_advanced_plane` + scripted mission | | |
| S8 | `PX4_GZ_STANDALONE=1` mode | connects once gz server is up | |

## Water bring-up (Plan 2, section 2.6)

| Test | Setup | Pass | Status |
|---|---|---|---|
| W1 | 0.5×0.3×0.3 m box, 10 kg, dropped 0.2 m, DART + bullet collision, one buoyancy grade | draft 0.0668 m ±2 %, heave ω ≈ 12.1 rad/s ±5 %, no NaN, RTF ≥ 1 | |
| W1b | add `<zW>` ≈ −73 for ζ ≈ 0.3 | log-decrement ζ = 0.30 ±0.05 | |
| W2 | one link, 3–5 hull boxes, 14 then 10 kg | displaced volume = m/ρ, static pitch matches, stable 60 s | |
| W3 | + wings + AdvancedLiftDrag, drop at trim from 10 m, no PX4 | lift and Cm within 5 % of `aero.m` at same α, V | |
| W4 | + PX4 airframe 4099, world `water` | floats at correct draft before arming; S1–S3 equivalents pass | |
