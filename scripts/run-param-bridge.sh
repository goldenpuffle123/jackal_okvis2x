#!/usr/bin/env bash
# Run parameter_bridge against an explicit topic whitelist.
#   bash scripts/run-param-bridge.sh [config.yaml]
# parameter_bridge reads its topic list from the ROS 1 parameter server, not a
# file, so the whitelist is uploaded under /jackal_bridge first (clear it with
# `rosparam delete /jackal_bridge`).
set -eo pipefail

root="${PIXI_PROJECT_ROOT:-${CONDA_PREFIX%/.pixi/envs/*}}"
config="${1:-$root/bridge_config/bridge_topics.yaml}"
ns=/jackal_bridge

if [ ! -f "$config" ]; then
  echo "run-param-bridge: no such config: $config" >&2
  exit 1
fi

# rosparam is ROS 1 python, so upload BEFORE run-bridge.sh reorders PYTHONPATH.
echo "run-param-bridge: uploading $config to $ROS_MASTER_URI under $ns"
rosparam load "$config" "$ns"

exec bash "$root/scripts/run-bridge.sh" parameter_bridge \
  "$ns/topics" "$ns/services_1_to_2" "$ns/services_2_to_1"
