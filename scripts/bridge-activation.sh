# Bridge env activation -- put the ROS 1 (Noetic) tree alongside this ROS 2
# (Jazzy) env so ros1_bridge can build and run against both.
_jackal_root="${CONDA_PREFIX%/.pixi/envs/*}"

# ROS 1 tree from the SEPARATE noetic env (sibling of this env's prefix).
source "${CONDA_PREFIX%/*}/noetic/setup.bash"
source "$CONDA_PREFIX/setup.bash"

# catkin's cache generator warns on stdout when ROS_DISTRO != noetic and then
# parses that warning as Python. Label as noetic to silence it.
export ROS_DISTRO=noetic

# RoboStack's setup.bash dedups, so sourcing Jazzy last does NOT put it ahead
# of noetic -- noetic then shadows Jazzy for packages in both distros and the
# build yields zero conversion pairs. Force the ROS 2 prefix to the front.
export CMAKE_PREFIX_PATH="$CONDA_PREFIX:$CMAKE_PREFIX_PATH"

# Sourcing (vs conda-activating) noetic doesn't put its lib dir on the loader
# path, so libroscpp.so goes missing at runtime.
export LD_LIBRARY_PATH="${CONDA_PREFIX%/*}/noetic/lib:$LD_LIBRARY_PATH"

# Sourcing noetic also puts its site-packages FIRST, where it shadows every
# ROS 2 python message package. The build needs ROS 1's python (catkin_pkg,
# genmsg, rospkg), so leave the order alone here; run-bridge.sh flips it for
# the run tasks. Do ROS 2 CLI work in the other envs.
export JACKAL_ROS2_SITE_PACKAGES="$(
  ls -d "$CONDA_PREFIX"/lib/python3*/site-packages 2>/dev/null | head -n1)"

. "$_jackal_root/scripts/ros1-net.sh"
. "$_jackal_root/scripts/ros2-dds.sh"

unset _jackal_root
