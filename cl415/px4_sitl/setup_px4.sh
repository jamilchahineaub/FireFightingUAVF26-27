#!/bin/bash
# px4 sitl for the cl415 in wsl, no sudo needed (gz libs come from the ros jazzy vendor packages)
#   wsl -d AMR -- bash /mnt/c/Users/jamil/Desktop/jim/uni/CL415-VIP/FireFightingUAVF26-27/cl415/px4_sitl/setup_px4.sh
set -e
PX4_TAG=${PX4_TAG:-v1.16.2}
PX4_DIR=${PX4_DIR:-$HOME/PX4-Autopilot}
VENV=${VENV:-$HOME/px4_venv}

if [ ! -d "$PX4_DIR/.git" ]; then
  git clone --branch "$PX4_TAG" --depth 1 --recursive --shallow-submodules -j 8 \
    https://github.com/PX4/PX4-Autopilot.git "$PX4_DIR"
fi

# the shallow clone has no nuttx tags, and px4's version header wants one (sitl doesn't use nuttx)
NX="$PX4_DIR/platforms/nuttx/NuttX/nuttx"
if [ -z "$(git -C "$NX" tag --list 'nuttx-*')" ]; then
  timeout 300 git -C "$NX" fetch -q --depth 1 origin '+refs/tags/nuttx-*:refs/tags/nuttx-*'     || git -C "$NX" tag nuttx-0.0.0
fi

# python deps in a venv (ubuntu 24.04 blocks system pip)
if [ ! -x "$VENV/bin/python" ]; then
  python3 -m venv --system-site-packages "$VENV"
fi
"$VENV/bin/pip" install -q --upgrade pip
"$VENV/bin/pip" install -q -r "$PX4_DIR/Tools/setup/requirements.txt" ninja pymavlink mavsdk pyulog
echo "setup done: $PX4_DIR, venv $VENV"
