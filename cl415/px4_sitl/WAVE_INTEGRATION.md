# CL-415 wave integration

Extends Jamil's existing CL-415 hull plugin with VRX wave-height sampling,
mutex-protected wavefield access, Reynolds-dependent ITTC-1957 friction,
and per-element wetted-length diagnostics.

## Dependencies and launch
Requires ROS 2 Jazzy, Gazebo Harmonic, and a built VRX jazzy workspace
at ~/vrx_ws, including libWaves.so and the coast_waves model.
The hull plugin samples local wave height using a horizontal plane
per element; wave slopes and orbital fluid velocities are not included.

Build from this directory:
    bash build_plugins.sh

Launch the wave world:
    VARIANT=water PX4_DIR="$HOME/PX4-Autopilot-cl415" bash run_sim.sh

The tested CL-415 PX4 baseline is v1.16.2.

## Diagnostics
Hull-state records now contain four values per element:
volume, plate force, drag, and wetted length.
The updated analyzer expects this format; older records require conversion.
The taxi diagnostic requires COM_RC_IN_MODE=1 for MAVLink manual input.
Restore COM_RC_IN_MODE=4 before returning to the autonomous baseline.

## W1 status
The separate 10 kg box world is work in progress.
Graded buoyancy defines water below z=0 and neglects air buoyancy above.
Floating behavior, draft, frequency, Bullet collision configuration,
and real-time factor still require verification.
