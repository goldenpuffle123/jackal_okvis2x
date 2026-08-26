#!/usr/bin/env bash
# Run a ros1_bridge executable out of the built bridge_ws overlay.
#   bash scripts/run-bridge.sh <executable> [args...]
# No `set -u`: colcon's generated setup.bash reads $COLCON_TRACE unguarded.
set -eo pipefail

root="${PIXI_PROJECT_ROOT:-${CONDA_PREFIX%/.pixi/envs/*}}"
exe="${1:?usage: run-bridge.sh <executable> [args...]}"
shift

libdir="$root/bridge_ws/install/ros1_bridge/lib/ros1_bridge"
if [ ! -x "$libdir/$exe" ]; then
  echo "run-bridge: $exe not built -- run: pixi run -e bridge build-bridge" >&2
  exit 1
fi

source "$root/bridge_ws/install/setup.bash"

# ROS 2 site-packages back in front of Noetic's (see bridge-activation.sh).
if [ -n "${JACKAL_ROS2_SITE_PACKAGES:-}" ]; then
  export PYTHONPATH="$JACKAL_ROS2_SITE_PACKAGES:${PYTHONPATH:-}"
fi

# Exec directly rather than via `ros2 run`: immune to the shuffle above.
exec "$libdir/$exe" "$@"
