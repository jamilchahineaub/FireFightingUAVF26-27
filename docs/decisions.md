# Decision log

One entry per decision that affects the model or the numbers. Newest last.
Format: ID, date, decision, why, consequence, who signs off.

---

## D-01 · 2026-09-28 · Reference trims are a static balance; the full trim differs by known physics

**Finding.** Table 3 of the May-2026 report (loaded: α 6.33°, δe −1.83°, δT 0.279; unloaded: α 4.46°, δe −0.59°, δT 0.244) is reproduced *exactly* by a static balance L = W, Cm_aero = 0, T = D. The full nonlinear trim in `trim.m` includes two effects that balance leaves out:

- thrust tilt T·sin α (≈1.3 % of weight) → α lower by ≈0.09°
- the thrust-line moment in the unloaded aircraft (thrust line 0.0343 m below the CG) → δe more positive by ≈0.33°

Both deviations are predicted analytically and checked in `test_trim_cruise.m`.

**Decision.** Keep both: `static_trim.m` is the regression reference (proves the coefficient set is the report's) and the initial guess; `trim.m` is what the controller is designed on. The linearised M_δT = +1.03 rad/s² (unloaded) reproduces the report's +1.02, so the thrust-pitch coupling is in the plant either way.

**Sign-off.** Jamil. Mention to Dr. Kaddouh on Tuesday as a modelling note, not a discrepancy.

---

## D-02 · 2026-09-28 · Two propulsion models; the legacy one is not physical

**Finding.** The report's trims imply a constant-power thrust law T = P·δT/V with P ≈ 1200 W propulsive at full throttle. Installed power is 959 W electric → ≤ 671 W propulsive at η_p = 0.70. At 20 m/s the legacy law would need 1714 W shaft at full throttle. It also contradicts T/W = 0.245 (33.6 N static). The cruise throttle of 27.9 % is therefore an artefact of the legacy law; a physical model gives roughly 70 % at cruise.

**Decision.** `propulsion.m` carries both. `legacy_constant_power` is the default *only* so the regression test against the report passes and the drop-study numbers can be reproduced. `momentum` (T = a·δT² − b·δT·V, a = 33.6 N, anchored to 16.7 N at 20 m/s) becomes the design default once Dr. Kaddouh signs off; its `dT_anchor = 0.75` is a placeholder until propeller/motor data exist. Switch by editing `propulsion.model` in `params.yaml`; the six-point batch takes the model name as an argument.

**Consequence.** Trim throttle, the X_u derivative and TECS throttle gains all change with the model. Report which model every result used (the batch prints it).

**Sign-off.** Needs Dr. Kaddouh (Tuesday). Tohme prepares the one-slide explanation.

---

## D-03 · 2026-09-28 · Gain conversion to PX4

PX4 fixed-wing rate-loop gains are normalised actuator output per rad/s; the control allocation maps ±1 to ±δ_max. A MATLAB gain in rad-of-surface per rad/s (numerically equal to deg per deg/s) converts as **K_PX4 = K_MATLAB / δ_max[rad]**, designed at `FW_AIRSPD_TRIM` because PX4 applies its own airspeed scaling. Sign differences (PX4 +pitch torque = nose-up, our +δe = nose-down) are handled by servo reversal in the actuator setup, never by negating gains. Implemented in `px4_export.m`. Parameter names must be confirmed in the pinned SITL before the file is used.

---

## D-04 · 2026-09-28 · Stall blend calibrated to CL_max

The Beard & McLain sigmoid blend (also used by Gazebo AdvancedLiftDrag) is already 50 % flat-plate at its centre, so putting the centre at CL_max/CL_α caps the blended lift at ≈0.75·CL_max and silently raises the stall speed by 15 %. `load_params.m` and `gen_sdf.py` both calibrate the blend centre so that the *peak* of the blended curve equals CL_max (centre 17.8°, peak 1.368 at 15.3°). `alpha_stall_lin` (13.9°) is kept for the "stall-blend-affected" flag on trims.

---

## D-05 · 2026-09-28 · Lateral-directional derivatives are placeholders

All lateral coefficients are Beard & McLain Aerosonde values tagged `aerosonde-placeholder`. They make the pipeline run and produce an unstable spiral (t₂ ≈ 6 s) and a dutch roll of ζ ≈ 0.3. None of this is a PrandtlPlane property. No lateral result is presented as a finding until the AVL box-wing model replaces them.

---

## D-06 · 2026-09-28 · Inertia: mass-station model with a back-fitted fuselage length

`L_fus = 1.49 m` is chosen so the airframe I_yy ≈ 2.0 kg·m², which reproduces the report's M_δT = +1.02 rad/s² and the 0.04 kg·m² drop in I_yy when the tank empties. The 1:11 CL-415 fuselage would be 1.80 m (I_yy ≈ 2.9). `I_xz = 0` explicitly. All superseded when the SolidWorks tensor arrives (`mass_properties.use_cad: true`).

---

## Open questions for Dr. Kaddouh (Tuesday 30 Sept)

1. D-02: approve the momentum thrust model as the design default? Do we have motor/prop data?
2. Control-surface layout on front and rear wings → fixes the mixing matrix and δ_max values.
3. SolidWorks delivery date (inertia tensor, OML).
4. Touchdown envelope numbers (onboarding doc Table 3) and the 90 % Monte Carlo threshold.
