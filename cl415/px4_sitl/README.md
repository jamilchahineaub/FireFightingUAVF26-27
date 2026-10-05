# CL-415 UAV in PX4 SITL + Gazebo Harmonic

The CL-415 model flying under PX4 (v1.16.2) in Gazebo Harmonic, with a ground station on Windows. Two variants:
a runway version on a sim-only tricycle gear, and a water version on a flying-boat hull model that takes off
from and lands on calm water. Everything here is generated from the project numbers (SolidWorks mass, AVL and
XFOIL aero, APC prop data), nothing is hand-tuned in the SDF.

## Running it

PX4 and Gazebo live in the WSL distro `AMR` (Ubuntu 24.04, ROS 2 Jazzy's Gazebo Harmonic). From a Windows
shell (Git Bash):

```bash
# runway flight
wsl -d AMR -- bash /mnt/c/Users/jamil/Desktop/jim/uni/CL415-VIP/FireFightingUAVF26-27/cl415/px4_sitl/run_sim.sh
# water take-off and landing
wsl -d AMR -- VARIANT=water bash /mnt/c/Users/jamil/Desktop/jim/uni/CL415-VIP/FireFightingUAVF26-27/cl415/px4_sitl/run_sim.sh
# fly the demo mission (takeoff, circuit, auto landing); add --water for the water pattern
wsl -d AMR -- ~/px4_venv/bin/python -u /mnt/c/Users/jamil/Desktop/jim/uni/CL415-VIP/FireFightingUAVF26-27/cl415/px4_sitl/fly_mission.py --water
# plots and numbers from the last flight log
wsl -d AMR -- ~/px4_venv/bin/python /mnt/c/Users/jamil/Desktop/jim/uni/CL415-VIP/FireFightingUAVF26-27/cl415/px4_sitl/analyse_flight.py
# stop everything
wsl -d AMR -- bash /mnt/c/Users/jamil/Desktop/jim/uni/CL415-VIP/FireFightingUAVF26-27/cl415/px4_sitl/stop_sim.sh
```

`run_sim.sh` starts the Gazebo server with the chosen world (the aircraft is already in it), the Gazebo GUI
(skip with `HEADLESS=1`), PX4 attached to the model, and on the water variant a recorder for the hull state. It
prints the WSL IP for the ground station. Logs go to `~/cl415_sim_logs/` in WSL; PX4's own ulogs are under
`~/PX4-Autopilot/build/px4_sitl_default/rootfs/log/`.

### Ground station: Mission Planner on Windows

Mission Planner is installed on this PC (`C:\Program Files (x86)\Mission Planner`). `bash px4_sitl/mission_planner.sh`
starts it and prints the settings. In Mission Planner pick the connection type **UDPCl** (top right), click
**Connect**, enter the WSL IP (`192.168.62.95` at the time of writing, `run_sim.sh` prints the current one) and
port **18570**. That is PX4's GCS link; it answers whoever sends the first heartbeat, so the Windows side opens the
connection and no firewall rule is needed. Tested from Windows with pymavlink: heartbeat in 0.9 s, 1100
messages in 3 s. Mission Planner is built around ArduPilot, so with PX4 the HUD, map, missions and modes work
while its parameter and tuning pages don't match PX4's parameters; QGroundControl (`QGC=1` starts it inside WSL)
is the native tool when those are needed.

Mission uploads from the script use PX4's second MAVLink link (UDP 14540), so a ground station can stay
connected while `fly_mission.py` runs.

### Recording a flight

```bash
wsl -d AMR -- bash /mnt/c/Users/jamil/Desktop/jim/uni/CL415-VIP/FireFightingUAVF26-27/cl415/px4_sitl/record_flight.sh        # VARIANT=water by default
.venv/Scripts/python px4_sitl/make_highlights.py                                                 # on windows afterwards
```

`record_flight.sh` flies the mission with two server-side video recorders running at 25 fps in simulation time:
an external chase camera (a weightless camera model that `libCl415ChaseCamera.so` keeps 5.5 m behind and 1.3 m
above the aircraft, following its heading but not its roll) and the onboard tail camera on the fin. The videos
land in `flight_logs/` as `water_chase_<stamp>.mp4` and `water_tailcam_<stamp>.mp4` with the hull state of the
run; `make_highlights.py` cuts the take-off and the landing from both cameras side by side into
`water_highlights_<stamp>.mp4` and a sheet of key frames. Gazebo 8's GUI video button has no service behind
it, which is why the recording is done in the server (it also works headless). With both cameras rendering the
simulation runs at roughly a third of real time; the videos are unaffected, and PX4 runs in lockstep with it.
The Windows side of the picture, Mission Planner, is yours to record: connect it as above, then Win+Alt+R
(Xbox Game Bar) records the active window.

## What is in here

| file | what |
|---|---|
| `make_px4_model.py` | generates both models, both worlds, both PX4 airframes and `px4_model_summary.json` |
| `models/cl415_px4/` | runway model: PX4 sensors, AdvancedLiftDrag aero, prop-map propulsion, tricycle gear |
| `models/cl415_px4_water/` | water model: the same without gear, plus the flying-boat hull plugin |
| `worlds/cl415_runway.sdf`, `cl415_water.sdf` | PX4-compatible worlds (same location and magnetic field as PX4's default world) |
| `airframes/4050_gz_cl415`, `4051_gz_cl415_water` | PX4 airframe files (actuators, limits, rate-loop feed-forwards, TECS, take-off and landing settings) |
| `gz_plugins/` | the three Gazebo system plugins below, built by `build_plugins.sh` into `~/cl415_gz_plugins` |
| `setup_px4.sh`, `build_px4.sh`, `install_airframe.sh` | clone PX4 v1.16.2, build SITL against the ROS vendor Gazebo libs (no sudo), install the airframes |
| `run_sim.sh`, `stop_sim.sh`, `mission_planner.sh` | start and stop the stack, start the ground station |
| `fly_mission.py` | uploads and flies the demo mission over MAVLink, prints a live trace |
| `analyse_flight.py` | ulog + hull state to a summary json and a plot in `flight_logs/` |

### Model

- Mass and inertia from SolidWorks (10.5 kg full), CG at 27 % MAC.
- Aero: one `AdvancedLiftDrag` system with the AVL stability and control derivatives from `flight_params/`
  (per radian, moments about the full-tank CG, controls scaled by the XFOIL viscous factors). The plugin's stall
  blend has a fixed width, so `CL0`, `CLa` and `alpha_stall` are fitted to reproduce the AVL lift curve from
  -5 to 6 deg and peak at CL_max = 1.236 (fit rms 0.012).
- Propulsion: `libCl415PropellerMap.so`. PX4's motor command (0 to 1000) is throttle; each prop reads thrust,
  shaft torque and rpm from maps over airspeed and throttle made by `flight_params/propulsion/prop_tools.py`
  from APC data and the motor model. Baseline hardware: Hacker A50-16S (365 kV) on 6S with a 13.5 in four-blade
  (APC 15.5x12-4 scaled to the size that clears the hull), 93 N static for the pair. Counter-rotating (left CW,
  right CCW seen from behind), see the water section for why. The reaction torque and the thrust-line offset
  (hubs 0.16 m above the CG) are in the physics.
- Sensors: IMU at the CG, barometer, GNSS, a pitot ahead of the nose, all under the names PX4's `gz_bridge`
  subscribes to. The magnetometer is `libCl415Magnetometer.so`, which publishes the body-frame field from the true
  attitude: Harmonic's own magnetometer sensor reports a field that PX4's bridge only corrects for yaw, so in a
  banked turn the heading wandered by 20 deg, the EKF gyro-bias estimates ran away and the first water-free
  circuit ended in a landing abort. With the plugin the EKF yaw error is 2.2 deg rms over a full circuit.
- Servos: `/model/cl415/servo_0..3` (left aileron, right aileron, elevator, rudder), position controlled, 25/25/30
  deg travel. PX4's sign (+ = trailing edge up, rudder right) is the opposite of the model joints, handled by the
  `SIM_GZ_SV_MINA/MAXA` angles in the airframe.
- Runway gear (sim only, the aircraft is a flying boat): nose wheel and mains at the step, 2 deg nose-up on the
  ground, tail clearance 10 deg.

### PX4 airframe

Standard plane with two motors and four control surfaces. Rate-loop feed-forwards come from the AVL derivatives
at 20 m/s (`FW_RR_FF 0.38`, `FW_PR_FF 0.53`, `FW_YR_FF 0.18`), the P and I gains are moderate starting values
that flew without retuning. Speeds: stall 13.9 m/s, minimum 16.7, trim 20, maximum 28. Take-off uses PX4's
runway-takeoff logic (heading hold, pitch held, rotation at 16.7 m/s), landing uses the auto-land with flare.
Rotor positions are left at zero in the allocator on the runway variant: with the real 0.16 m vertical offset
the allocator counted the thrust moment in its own units and saturated the elevator nose-down on the first
take-off attempt. The moment is in the physics and the pitch integrator trims it.

## Flight controls check (runway, `flight_logs/2026-10-03_21_53_10`)

Full mission in AUTO.MISSION: runway take-off, climb to 60 m, a 2.4 km circuit at 20 m/s, approach and
auto-landing, armed to disarmed with no operator input.

| | sim | prediction |
|---|---|---|
| ground roll to lift-off | 36 m, 4.4 s, 19.0 m/s | T/W 0.9, so short |
| cruise throttle at 20 m/s, 60 m | 0.550 | 0.556 (prop maps + drag polar) |
| cruise pitch / elevator | 0.7 deg / 0.8 deg TE up | 0.9 deg / 0.9 deg TE down static balance; the 1.7 deg elevator difference is the thrust-line moment and the elevator's own lift, both in the sim and not in the static balance |
| altitude tracking rms (cruise and turns) | 1.8 m to the TECS reference | |
| airspeed tracking rms | 0.63 m/s | |
| max bank | 42 deg (limit 40) | |
| touchdown | 0.64 m/s sink, 16.3 m/s, 1.3 deg nose up, 145 m roll-out (no brakes) | |
| EKF attitude error vs truth, rms | roll 0.44, pitch 0.79, yaw 2.2 deg | |

Signs and authority of all four surfaces are confirmed by the closed loops: a wrong sign diverges, and the
feed-forwards alone put the rate loops close to their setpoints. The roll overshoot in turns (42 vs 40 deg) is
the only thing worth tuning.

## Water operations

### Why a custom hull plugin and not Gazebo's USV plugins

Gazebo Harmonic ships `Buoyancy` and `Hydrodynamics`, the pair used for surface and underwater vehicles. Neither
fits an aircraft:

- `Hydrodynamics` assumes a fully submerged body and applies its damping all the time, so it would keep dragging
  the aircraft once airborne.
- `Buoyancy` in graded mode clips the collision boxes against the water plane with gz-math's
  `Box::VolumeBelow`, which is wrong in the installed gz-math 7.5.2 whenever the plane misses the box centre
  (`gz_plugins` test: a 0.44 x 0.21 x 0.29 m box with the water 5 cm off its centre gives 0.0103 m^3 against the
  exact 0.0088; at 8 cm it gives 0.0183 against 0.0208; the error is the same at any tilt). That bug was already
  on the list from the earlier Gazebo water test.

`libCl415FlyingBoatHull.so` (`gz_plugins/FlyingBoatHull.cc`) does the whole job for a hull made of boxes and
switches itself off cleanly when the hull leaves the water:

- exact clipping of every box against the water plane (six tetrahedra per box) for the submerged volume, its
  centroid, the wetted part of each face and the waterplane;
- buoyancy at the submerged centroid;
- a plate force on the wetted bottom, sides and top, `0.5 rho v^2 A sin(tau) (k cos(tau) + cd sin(tau))`, pushing
  into the hull only while the face moves into the water: planing lift at small angles (k from Savitsky, 0.4 for
  the 20 deg deadrise forebody, less behind the step), bluff drag at large angles (slamming, sideways drift);
- skin friction on every wetted face (Cf 0.005);
- form drag per group of boxes (hull, each float) on the largest submerged cross-section, `cd_form` 0.10 for the
  hull and 0.15 for the floats;
- linear heave damping scaled to 0.6 of critical so the static float settles;
- a state topic `/cl415/hull` (volume, buoyancy, resistance, planing lift, draft, speed) that `run_sim.sh` records.

The elements are the eight collision boxes of the hull (six) and the floats (two). The two bow boxes are tilted
8 and 18 deg bow-up in the plugin so their bottoms rise toward the bow like the real keel line (the physics
collision boxes are unchanged).

Floating check against `verify/hydrostatics.py`: buoyancy 102.9 N for 103.0 N of weight, keel draft 81 mm
(hydrostatics: 85 mm for the CAD hull, a little less for the flat-bottomed boxes), 3.4 deg bow up, 2.7 deg of
heel onto one float, at rest with no residual motion.

### Take-off and landing technique in PX4 terms

Airframe `4051_gz_cl415_water`:

- take-off: `RWTO_TKOFF 1` with `RWTO_PSP 5` (hold the nose up while the hull climbs onto the step), a 3 s
  throttle ramp, rotation at 16.7 m/s, climb-out pitch at least 8 deg (no tail to strike on water);
- heading on the water: the air rudder has nothing at 6 m/s, so the allocator is given the real lateral motor
  positions (`CA_ROTOR0/1_PY` +-0.297) and steers with differential thrust, as a pilot would;
- landing, glassy-water style: a 4 deg approach, flare from 2.5 m to 4 to 9 deg nose up at 0.4 m/s sink so the
  step touches first, throttle to idle, then the hull resistance stops it; PX4 calls it landed once below its
  land-detector speeds and disarms.

### First attempts and what they showed

The first water run stuck at the hump: full throttle, 6.5 m/s, 5 deg nose down, circling left at 20 m radius.
The hull state log made the reason plain: resistance over weight was 0.06 at 2 m/s and 0.23 at 4 m/s (a normal
hump), then 0.82 at 6.5 m/s with the keel 17 to 30 cm under. The thrust line gives about 15 Nm nose-down at full
power, the elevator has 5 Nm at that speed, and flat box bottoms give no lift when the bow is pressed down, so the
bow dug in and the submerged deck was pushed further down. The same-rotation prop torque rolled it onto the left
float, whose drag turned it. Three fixes, all with a physical basis: the bow rocker above (forebody planing lift
ahead of the CG once the nose drops, about 21 Nm at 6.5 m/s), counter-rotating props (free on an electric twin,
removes the torque roll and the asymmetric slipstream), and differential thrust for heading hold on the water.

### Second round of fixes

With the bow rocker and the afterbody at the sternpost angle the hull still pinned itself at 7 m/s (the tilted
afterbody became a lifting surface whenever it was wet, which it is at rest) and, with the wake hollow, spun
once the afterbody dried while the forebody was still deep (all the wetted side area ahead of the CG). The hull
model got two more pieces of real flying-boat physics: the wake hollow behind the step blends in only once the
hull is on the step (6 to 9 m/s), and the plate force on the box sides fades out with it (chine separation: a
planing hull's sides run dry). The wing panels were added as elements so a dropped wing tip meets water, after
one run water-looped and capsized at 8 m/s and then slid along inverted with the wing passing through the
surface. The open-loop test (`hull_test.sh`, 80 % throttle, fixed elevator, no autopilot) then gave a textbook
resistance curve: R/W 0.05 at 1 m/s, 0.36 at the 6 m/s hump, 0.15 at 12 m/s, 0.07 to 0.09 planing, with the main
forebody surface carrying 0.7 of the weight and the afterbody dry.

The last blocker was in PX4, not the water: its runway-takeoff logic resets the attitude integrators until
lift-off, so on the water it held the 5 deg pitch target with proportional elevator only, 21 % of travel, against
the thrust-line moment (14 Nm) plus the resistance acting at the keel 0.2 m below the CG (15 Nm). The hull skimmed
flat at 12.6 m/s with its whole bottom wet (26 N of skin friction alone). The water airframe now gives the loop
the back pressure a seaplane pilot applies: `RWTO_PSP 12`, `FW_P_TC 0.25`, `TRIM_PITCH 0.10` and the low-airspeed
differential trim `FW_DTRIM_P_VMIN 0.25` (full below 16.7 m/s, gone at cruise).

### Water flight results (`flight_logs/2026-10-03_22_59_02`, hull state in `hull_state_water6.jsonl`)

Full cycle in AUTO.MISSION: water take-off heading east, climb to 50 m, a 1.1 km circuit, a 4 deg approach and a
water landing back near the start, armed to disarmed with no operator input.

| | result |
|---|---|
| at rest | draft 81 mm, 3.4 deg bow up, 2.7 deg heel onto one float |
| take-off run | 25 m, 4.8 s from full power to the keel clear of the water, lift-off at 14.2 m/s |
| hump | resistance/weight 0.25 at 5 m/s in the open-loop curve; a 0.38 transient at 10 m/s in the autopilot run as the hull rotated and the afterbody touched |
| planing | the forebody surface carried up to 0.94 of the weight, resistance/weight 0.19 to 0.21 from 7 to 13 m/s |
| after lift-off | 22 deg pitch and a 21 deg roll excursion (the 12 deg taxi target plus the trims carry into the first second of flight), recovered within 3 s |
| circuit | throttle 0.55 at 19.8 m/s (predicted 0.556), altitude 1.3 m rms to the TECS reference, airspeed 0.5 m/s rms |
| approach | 4 deg slope at 16.6 m/s, flare from 2.5 m |
| touchdown | 14.4 m/s, 0.6 to 0.75 m/s sink, 3.5 deg nose up; the hull skipped on planing lift (buoyancy 4 % of weight in the first 2 s), then settled to the 80 mm draft |
| stop | 8.7 s and about 55 m from first touch, landed detection and auto-disarm from PX4 |

What a real seaplane pilot would recognise: the aircraft needs full back pressure to break the hull free of the
water, the rotation is abrupt once it does, and the glassy-water landing is a long flat skip. What they would
not: the take-off is short because the static thrust-to-weight is 0.9 and there are no waves. Things to tune
next are the rotation (a lower `RWTO_PSP` once the hull is on the step, or PX4's rotation airspeed) and the
flare sink rate, which comes out above the 0.4 m/s asked for.

## Things that were missing and got fixed on the way

1. PX4's build wants a NuttX tag even for SITL; the shallow clone had none (`setup_px4.sh` fetches it).
2. The prop plugin found its maps only through `model://` URIs, not paths relative to the SDF.
3. The allocator with real rotor positions saturated the elevator against the thrust-line moment at take-off
   (PX4 "attitude failure (pitch)" and failsafe 2 s after lift-off).
4. Harmonic's magnetometer in banked flight (see Sensors), the cause of a landing abort and runaway gyro biases.
5. PX4's Gazebo ESC check calls a motor offline when its command is exactly 0, so `SIM_GZ_EC_MIN` is 1.
6. Auto-landing aborted for a missing terrain estimate (no rangefinder on the model): `FW_LND_USETER 0`.
7. `DO_LAND_START` must be uploaded in `MAV_FRAME_MISSION`, PX4 rejects it in a global frame.
8. Hull: the flat collision boxes have no rocker and the real thrust line pushes the nose down at full power.
9. Hull: the afterbody needs the step's wake hollow, or it lifts the stern before the forebody can plane; the
   hollow must come in with planing speed (6 to 9 m/s), or the hull spins when its side area is all forward.
10. Hull: box sides act as fins once planing unless they fade with chine separation; a water-looped, capsized
    aircraft slid along inverted because the wing had no water model (wing elements added).
11. PX4 runway take-off resets the attitude integrators until lift-off: fine with a nose wheel, 5 deg of
    elevator on the water. Taxi pitch target, attitude time constant and the low-airspeed pitch trim give it the
    back pressure.
12. `stop_sim.sh` left the `gz topic` recorders alive, so several of them wrote the same hull-state file at once.

## Still missing

- Waves and wind: calm water only. The VRX wave field would be the next step for sea states.
- The hull model is physical but uncalibrated beyond the hydrostatics check; the hump (0.25 to 0.36 of the
  weight here) and porpoising behaviour want a comparison with flying-boat tank data (NACA hull series) or a
  tow test of the real hull. The wake-hollow depth and speeds and the chine fade are the least certain knobs.
- Rotation off the water is abrupt (22 deg pitch, 21 deg roll excursion): a two-stage pitch target or an earlier
  rotation airspeed in PX4, or a wing-incidence check in the CAD, would soften it.
- Spray and prop slipstream over the tail, both of which help a real seaplane rotate.
- A rangefinder for the flare (the flare now runs on GNSS/baro height, sink at touch 0.6 to 0.75 m/s against
  the 0.4 m/s asked for).
- Empty-tank variant and the water drop itself (mass and CG change in flight).
- Mission Planner's parameter and tuning pages are ArduPilot's; PX4 tuning goes through QGC.
- The runway roll overshoot (42 vs 40 deg bank) and a second look at the pitch-loop gains at 1.2 Vs.
