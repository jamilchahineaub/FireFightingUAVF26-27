# Step 5: headless Gazebo checks

Gazebo Harmonic (gz-sim 8.9) with ROS 2 Jazzy in the `AMR` WSL distro, launched through `ros2 launch cl415_description sim.launch.py headless:=true ...` by `verify/run_sim_tests.sh`. `verify/sim_probe.py` records `/cl415/odometry` and `/joint_states` through the bridge. Plots in `verify/plots/`.

## Floating on water

Expected attitude from the collision boxes (what the graded buoyancy system integrates), solved in `verify/hydrostatics.py`: buoyancy equals weight and the moments about the CG vanish. The level-hull case is unstable in roll, so the solution leans onto one float, like the real aircraft at rest.

| run | base z [m] | keel depth [m] | roll [deg] | pitch [deg] | z ripple last 5 s [mm] | status |
|---|---:|---:|---:|---:|---:|---|
| full, no damping | 0.2475 | 0.0627 | -0.04 | -4.32 | 74.4 | floats, no blow-up (max speed 0.58 m/s) |
| full, damped | 0.2452 | 0.0649 | -3.22 | -4.52 | 0.8 | floats, no blow-up (max speed 0.08 m/s) |
| empty, damped | 0.2674 | 0.0428 | -4.37 | -5.54 | 5.7 | floats, no blow-up (max speed 0.21 m/s) |
| predicted, tank full | 0.2383 | 0.0718 | +/-2.73 | -3.39 | | exact box clipping |
| predicted, tank empty | 0.2516 | 0.0585 | +/-3.38 | -3.62 | | exact box clipping |

Pitch in ROS convention (positive = nose down), so the negative values mean bow up. The real CAD hull would float level at a keel draft of 85 mm (full) and 72 mm (empty); the collision boxes are flat-bottomed, so they float a bit shallower than the vee hull.

## Sitting on hard ground

Spawned at z = 0.318 m. After 15 s: base z = 0.3065 m (keel box bottom at -0.0000 m), roll -0.00 deg, pitch +0.00 deg, z ripple 0.00 mm, max speed after 2 s 0.000 m/s. It rests on the flat bottoms of the hull boxes, with no jitter or drift.

## Control surfaces and motors (on water, damped)

| joint | command at 3 s [rad] | reached at 5.5 s [rad] | rise time 10-90 % [s] | after +1.0 rad command at 6 s |
|---|---:|---:|---:|---|
| left_aileron_joint | +0.30 | +0.3001 | 0.040 |  |
| right_aileron_joint | -0.30 | -0.3001 | 0.040 |  |
| elevator_joint | +0.20 | +0.2014 | 0.040 | max 0.4363 rad (limit 0.4363) |
| rudder_joint | +0.40 | +0.3998 | 0.040 |  |

Motors at 600 rad/s from 8 to 14 s: joint speed +60.0 (left) and -60.0 (right) rad/s, i.e. 600 / rotorVelocitySlowdownSim = 60 with opposite signs, as set for counter-rotation. The aircraft moved +13.919 m along +x (forward) in those 6 s and reached 4.27 m/s, so thrust points forward. Heading changed by +149.2 deg (the rudder was at +0.4 rad, trailing edge left, so some yaw is expected once it moves).

