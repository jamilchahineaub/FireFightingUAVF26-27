# Decisions

## D-01 Trim reference (2026-09-28)

Report Table 3 comes from a static balance: L = W, Cm_aero = 0, T = D. `static_trim.m` reproduces it exactly.

`trim.m` also includes thrust tilt and the thrust-line moment, so it differs from Table 3:

| | alpha | de |
|---|---|---|
| loaded | -0.09 deg | +0.06 deg |
| unloaded | -0.09 deg | +0.39 deg |

Both shifts match the analytical prediction (`test_trim_cruise`). Design uses `trim.m`. Linearized unloaded M_dT = +1.03 rad/s^2 (report: +1.02).

## D-02 Propulsion model (2026-09-28, needs sign-off)

The report trims (27.9 % / 24.4 % throttle at 20 m/s) imply T = P dT / V with P = 1200 W propulsive at full throttle. Installed is 959 W electric, about 670 W propulsive at eta_p = 0.70. It also contradicts T/W = 0.245 (33.6 N static).

`propulsion.m` has two models:

- `legacy_constant_power`: reproduces the report. Used for regression only.
- `momentum`: T = a dT^2 - b dT V, a = 33.6 N, anchored at 16.7 N at 20 m/s and dT = 0.75 (placeholder until prop data). Cruise throttle is about 75 %.

Set `propulsion.model` in `params.yaml` after Dr. Kaddouh decides.

## D-03 PX4 gain conversion

K_PX4 = K_MATLAB [rad surface per rad/s] / delta_max [rad], designed at FW_AIRSPD_TRIM. Sign differences are fixed with servo reversal, not by negating gains. Implemented in `px4_export.m`.

## D-04 Stall blend

With the blend centre at CL_max / CL_alpha, the blended lift peaks at about 0.75 CL_max. The centre is solved so the peak equals CL_max (17.8 deg centre, CL 1.37 at 15.3 deg). Same calculation in `load_params.m` and `gen_sdf.py`.

## D-05 Lateral derivatives

All lateral coefficients are Aerosonde values (Beard & McLain). They give an unstable spiral (t2 about 6 s) and dutch roll zeta about 0.3. Not PrandtlPlane results. Replace with AVL.

## D-06 Inertia

Mass-station model, kx = 0.25 b, ky = 0.30 L_fus. L_fus = 1.49 m is back-fitted so Iyy = 2.0 kg m^2, which reproduces the report's M_dT. Ixz = 0. Replace with SolidWorks (`mass_properties.use_cad: true`).
