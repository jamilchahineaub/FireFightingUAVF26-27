# Step 3: mass properties

Numbers measured by SolidWorks (33.4.1) on the parts in `cad/solidworks/`, each part at the density that gives its budget mass. Frame: REP-103, origin at the wing leading edge root, which is also the leading edge of the mean chord because the wing is rectangular. CG % MAC is taken along the body X axis: (x_LE - x_cg) / MAC.

## Mass breakdown

| item | SolidWorks part | URDF link | mass [kg] | density [kg/m^3] |
|---|---|---|---:|---:|
| Hull: composite skin, frames, step, wing saddle (1.70) plus wiring, connectors, plumbing and fasteners (0.38), spread over the hull skin | fuselage | fuselage | 2.080 | 839 |
| Left wing half, foam core + glass skin + spar | left_wing | left_wing | 0.500 | 46 |
| Right wing half | right_wing | right_wing | 0.500 | 46 |
| Left aileron | left_aileron | left_aileron | 0.035 | 74 |
| Right aileron | right_aileron | right_aileron | 0.035 | 74 |
| H-stab incl. finlets | h_stab | h_stab | 0.140 | 32 |
| Elevator (both halves + torque tube) | elevator | elevator | 0.060 | 39 |
| V-stab | v_stab | v_stab | 0.090 | 39 |
| Rudder | rudder | rudder | 0.035 | 60 |
| Left nacelle fairing | left_nacelle | left_nacelle | 0.100 | 402 |
| Right nacelle fairing | right_nacelle | right_nacelle | 0.100 | 402 |
| Left 14x7 prop + spinner | left_propeller | left_propeller | 0.050 | 364 |
| Right 14x7 prop + spinner | right_propeller | right_propeller | 0.050 | 364 |
| Left float + pylon | left_float | left_float | 0.130 | 791 |
| Right float + pylon | right_float | right_float | 0.130 | 791 |
| Water tank, empty, with outlet valve | water_tank | water_tank | 0.250 | 955 |
| Left motor, 600-800 W outrunner | eq_left_motor | left_nacelle | 0.300 | 3395 |
| Right motor | eq_right_motor | right_nacelle | 0.300 | 3395 |
| Left ESC | eq_left_esc | left_nacelle | 0.070 | 2222 |
| Right ESC | eq_right_esc | right_nacelle | 0.070 | 2222 |
| Left aileron servo | eq_left_aileron_servo | left_wing | 0.035 | 3241 |
| Right aileron servo | eq_right_aileron_servo | right_wing | 0.035 | 3241 |
| Avionics: FC, GPS, telemetry, RC receiver, power module, airspeed | eq_avionics | fuselage | 0.300 | 1071 |
| Elevator servo | eq_elevator_servo | fuselage | 0.035 | 1151 |
| Rudder servo | eq_rudder_servo | fuselage | 0.035 | 1151 |
| Tank door servo | eq_door_servo | fuselage | 0.035 | 1151 |
| Battery, 6S 16 Ah LiPo (355 Wh) | eq_battery | fuselage | 2.000 | 2060 |
| **Dry total** | | | **7.500** | |
| Water, 3 L | water_3L | water_tank | 3.000 | 1000 |
| **Take-off total** | | | **10.500** | |

The density column is just budget mass over CAD volume, so it is an equivalent density. The hull, nacelles and floats are hollow skins (2.0, 1.5 and 1.5 mm), so their mass sits at the skin, as it does on the real thing. Wings and tail surfaces are solid (foam core). Equipment items are boxes or cylinders of their real size at their mounting point.

Where the numbers come from: composite hull at about 1.3 kg/m^2 of skin plus frames, foam/glass wing at about 1.4 kg/m^2 of planform, 600-800 W outrunners with 14x7 props, a 355 Wh battery (roughly 40 min at a 500 W cruise), 35 g servos. The battery takes whatever is left of the 7.5 kg dry budget.

## CG check and battery position

- hull floor, right ahead of the tank: battery centre at (0.141, 0.000, -0.218) m -> CG 17.7 % MAC empty, 19.8 % MAC full (CAD-mesh estimate)
- tray on top of the water tank: battery centre at (0.033, 0.000, -0.097) m -> CG 28.0 % MAC empty, 27.1 % MAC full (CAD-mesh estimate)
- The first spot puts the CG ahead of the 25-30 % band, so the battery moved 108 mm aft and 121 mm up, onto a tray on top of the water tank.
- Fit check of the battery box: 0.00 cm^3 outside the hull cavity, 0.00 cm^3 overlap with the tank and the other equipment.
- With the SolidWorks numbers: **28.0 % MAC empty, 27.1 % MAC full**, inside the 25-30 % band.

## Aircraft totals (SolidWorks)

| | tank full | tank empty |
|---|---:|---:|
| mass [kg] | 10.500 | 7.500 |
| CG x [m] | -0.0754 | -0.0779 |
| CG y [m] | 0.0000 | 0.0000 |
| CG z [m] | -0.1007 | -0.0659 |
| CG [% MAC] | 27.1 | 28.0 |
| Ixx about CG [kg m^2] | 1.2097 | 1.1698 |
| Iyy about CG [kg m^2] | 0.8972 | 0.8529 |
| Izz about CG [kg m^2] | 1.8900 | 1.8744 |
| Ixz about CG [kg m^2] | 0.0488 | 0.0466 |
| Ixy about CG [kg m^2] | -0.0000 | -0.0000 |
| Iyz about CG [kg m^2] | -0.0000 | -0.0000 |

Products of inertia are in tensor form (Ixz = -integral of xz dm), which is what URDF and SDF expect. SolidWorks prints +integral of xz dm in its Mass Properties window, so the signs there are flipped.

Filling the tank moves the CG by 2.4 mm in x and -34.8 mm in z (0.9 % MAC): the water centroid sits on the wing quarter chord, next to the dry CG, and low in the hull.

## Cross-checks

- Sum of the parts vs the SolidWorks assembly (Full / Empty configurations): mass differs by 1.9e-10 / 1.8e-10 kg, CG by 0.0000 / 0.0000 mm, inertia by 2.1e-10 / 2.0e-10 kg m^2.
- SolidWorks vs the CadQuery mesh estimate, whole aircraft: CG 27.14 vs 27.14 % MAC full.
- Per part, the largest CoM difference is 0.15 mm (left_nacelle) and the largest inertia difference 0.27 % (right_propeller); the mesh is a little smaller than the exact surfaces.

## Per-link inertials (as written to the URDF)

Inertia about each link CoM, in the link frame. Fixed links share the base_link frame; control surfaces sit on their hinge lines; the propeller frames are turned so their Z axis points forward.

| link | parts | mass [kg] | CoM in link frame [m] | ixx | iyy | izz | ixy | ixz | iyz |
|---|---|---:|---|---:|---:|---:|---:|---:|---:|
| fuselage | fuselage, eq_avionics, eq_elevator_servo, eq_rudder_servo, eq_door_servo, eq_battery | 4.4850 | (-0.0503, 0.0000, -0.1161) | 3.544e-02 | 5.117e-01 | 5.040e-01 | 6.61e-06 | 6.31e-03 | -3.81e-06 |
| left_wing | left_wing, eq_left_aileron_servo | 0.5350 | (-0.1150, 0.6289, 0.0037) | 6.798e-02 | 2.095e-03 | 6.993e-02 | -5.22e-04 | -7.43e-05 | -6.46e-05 |
| right_wing | right_wing, eq_right_aileron_servo | 0.5350 | (-0.1150, -0.6289, 0.0037) | 6.799e-02 | 2.094e-03 | 6.994e-02 | 5.24e-04 | -7.44e-05 | 6.42e-05 |
| left_aileron | left_aileron | 0.0350 | (-0.0184, -0.0000, -0.0026) | 4.633e-04 | 1.441e-05 | 4.751e-04 | 0.00e+00 | -1.75e-06 | 0.00e+00 |
| right_aileron | right_aileron | 0.0350 | (-0.0184, -0.0000, -0.0026) | 4.633e-04 | 1.441e-05 | 4.751e-04 | 0.00e+00 | -1.75e-06 | 0.00e+00 |
| h_stab | h_stab | 0.1400 | (-0.8819, 0.0000, 0.1267) | 9.822e-03 | 2.198e-04 | 9.950e-03 | -8.94e-09 | 7.67e-08 | -3.80e-08 |
| elevator | elevator | 0.0600 | (-0.0251, 0.0000, 0.0000) | 4.951e-03 | 4.283e-05 | 4.989e-03 | -2.75e-11 | 6.46e-11 | 1.98e-10 |
| v_stab | v_stab | 0.0900 | (-0.8811, 0.0000, 0.1350) | 1.204e-03 | 1.759e-03 | 5.749e-04 | -7.84e-09 | 5.63e-04 | -3.29e-09 |
| rudder | rudder | 0.0350 | (-0.0802, 0.0000, 0.1767) | 4.790e-04 | 5.200e-04 | 4.309e-05 | -2.58e-10 | 9.13e-05 | -7.30e-11 |
| left_nacelle | left_nacelle, eq_left_motor, eq_left_esc | 0.4700 | (0.1030, 0.2973, 0.0547) | 3.160e-04 | 3.440e-03 | 3.359e-03 | 1.40e-07 | -9.12e-05 | 7.14e-08 |
| right_nacelle | right_nacelle, eq_right_motor, eq_right_esc | 0.4700 | (0.1030, -0.2973, 0.0547) | 3.161e-04 | 3.440e-03 | 3.359e-03 | 1.40e-07 | -9.11e-05 | 6.71e-08 |
| left_propeller | left_propeller | 0.0500 | (-0.0000, 0.0000, 0.0079) | 6.452e-05 | 6.452e-05 | 1.110e-04 | -4.17e-11 | -7.42e-10 | 1.95e-10 |
| right_propeller | right_propeller | 0.0500 | (-0.0000, -0.0000, 0.0079) | 6.452e-05 | 6.453e-05 | 1.110e-04 | 1.40e-09 | -2.60e-10 | -1.63e-10 |
| left_float | left_float | 0.1300 | (-0.0800, 1.1517, -0.1176) | 2.736e-04 | 9.504e-04 | 7.415e-04 | -3.63e-08 | -4.46e-05 | 1.43e-09 |
| right_float | right_float | 0.1300 | (-0.0800, -1.1517, -0.1176) | 2.736e-04 | 9.504e-04 | 7.415e-04 | -3.63e-08 | -4.46e-05 | 1.43e-09 |
| water_tank (full) | water_tank, water_3L | 3.2500 | (-0.0694, 0.0000, -0.1877) | 9.279e-03 | 1.405e-02 | 1.742e-02 | 0.00e+00 | 0.00e+00 | 0.00e+00 |
| water_tank (empty) | water_tank | 0.2500 | (-0.0694, 0.0000, -0.1877) | 1.185e-03 | 1.636e-03 | 1.972e-03 | 0.00e+00 | 0.00e+00 | 0.00e+00 |
