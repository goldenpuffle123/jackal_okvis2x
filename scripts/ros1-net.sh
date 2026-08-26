# Shared ROS 1 network setup for the `noetic` and `bridge` envs.
# A script, not [activation.env]: activation.env hard-overrides the caller, so
# `ROS_MASTER_URI=... pixi run` would be silently ignored.

# The Jackal's onboard master. Override: JACKAL_MASTER=localhost pixi run ...
: "${JACKAL_MASTER:=192.168.131.1}"

# Treat catkin's own default (set by its activation hook, before this runs) as
# "nobody asked"; honour anything else the caller set.
case "${ROS_MASTER_URI:-}" in
  "" | "http://localhost:11311")
    export ROS_MASTER_URI="http://${JACKAL_MASTER}:11311"
    ;;
esac
export JACKAL_MASTER

# ROS 1 hands the master a callback address and the robot connects back to it,
# so ROS_IP must be our address on the master's network. Ask the kernel which
# source address it would use rather than hardcoding one.
if [ -z "${ROS_IP:-}" ] && [ -z "${ROS_HOSTNAME:-}" ]; then
  _j_host="${ROS_MASTER_URI#*://}"
  _j_host="${_j_host%%:*}"
  _j_host="${_j_host%%/*}"
  # `ip route get` only takes addresses, so resolve names first.
  case "$_j_host" in
    *[!0-9.]*)
      _j_host="$(getent ahostsv4 "$_j_host" 2>/dev/null |
        awk 'NR==1 {print $1}')"
      ;;
  esac
  _j_ip="$(ip route get "${_j_host:-none}" 2>/dev/null |
    sed -n 's/.*[[:space:]]src[[:space:]]\([0-9.]*\).*/\1/p' | head -n1)"
  if [ -n "$_j_ip" ]; then
    export ROS_IP="$_j_ip"
  else
    echo "ros1-net: no route to master ($ROS_MASTER_URI); ROS_IP left unset" >&2
  fi
  unset _j_host _j_ip
fi

# ROS_HOSTNAME=localhost with a remote master means the robot calls back to
# itself: cheap to detect, miserable to debug.
case "${ROS_HOSTNAME:-}" in
  localhost | 127.0.0.1)
    case "$ROS_MASTER_URI" in
      *localhost* | *127.0.0.1*) ;;
      *)
        echo "ros1-net: WARNING ROS_HOSTNAME=$ROS_HOSTNAME with a remote" \
          "master ($ROS_MASTER_URI) -- the robot cannot call back." >&2
        ;;
    esac
    ;;
esac
