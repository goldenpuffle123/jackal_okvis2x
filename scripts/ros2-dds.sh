# Shared ROS 2 / DDS setup, so the bridge lands in the same graph as the sensor
# and okvis envs. Must match [feature.jazzy.activation.env] in pixi.toml.
: "${ROS_DOMAIN_ID:=0}"
export ROS_DOMAIN_ID
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

# Keep discovery on loopback: everything ROS 2 here runs on this laptop, and
# the SUBNET default would multicast out of the robot link too. Set
# ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET to let a second ROS 2 machine in.
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"
