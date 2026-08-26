#!/usr/bin/env bash
# Show where the ROS 1 side of this env is pointed, and whether it answers.
echo "ROS_MASTER_URI = ${ROS_MASTER_URI:-<unset>}"
echo "ROS_IP         = ${ROS_IP:-<unset>}"
echo "ROS_HOSTNAME   = ${ROS_HOSTNAME:-<unset>}"
if rosnode list >/dev/null 2>&1; then
  echo "master         = reachable ($(rosnode list 2>/dev/null | wc -l) nodes)"
else
  echo "master         = NOT reachable"
fi
