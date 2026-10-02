# Tasks

## Michael: simulation

- [x] Set up PX4 + Gazebo Harmonic (Ubuntu 24.04, PX4 v1.17; check the stock Cessna flies and logs open in PlotJuggler)
- [ ] Get our plane flying in Gazebo (copy `advanced_plane`, load the SDF from `make sdf`, add control surfaces and both motors)
- [ ] Make it float (water world at z = 0, hull as a few boxes, DART physics; a plain box should sit at the right depth first)
- [ ] Check Gazebo aero matches MATLAB (same angle of attack and speed should give the same lift and pitching moment, within 5 %)
- [ ] Landing test tools (script that pulls sink rate, pitch, roll, airspeed at touchdown from the log; batch runner for repeated landings)

## Zahi: controller

- [x] Aircraft model in MATLAB (6-DOF, trim, linearization, modes, all tested)
- [ ] Design the inner loops (pitch and roll rate, then attitude; phase margin 45 deg, gain margin 6 dB)
- [ ] Design altitude/airspeed and heading control (TECS and path following, like PX4)
- [ ] Robustness check (vary pitch derivatives by 30 % and mass by 10 %, margins must hold)
- [ ] Export gains to PX4 (`px4_export.m`; confirm parameter names in Michael's sim)

## Together

- [ ] Fly our controller in Gazebo (cruise, turns, approach)
- [ ] Water landing (constant attitude, slow sink, about 100 runs with wind)
- [ ] Replace placeholder data (AVL aero model, SolidWorks inertia)
- [ ] Later: move to AirSim/UE5 for high-fidelity water and RL

## Questions for Dr. Kaddouh

1. Which thrust model to use (the report's cruise throttle doesn't match the installed power; see `docs/decisions.md` D-02)
2. Control surface layout on the front and rear wings
3. When the SolidWorks model will be ready
4. Touchdown limits (sink rate, pitch) and how many landings must pass
