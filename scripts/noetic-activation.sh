# Noetic env activation -- ROS 1 only, so just the master/callback setup.
_jackal_root="${CONDA_PREFIX%/.pixi/envs/*}"
. "$_jackal_root/scripts/ros1-net.sh"
unset _jackal_root
