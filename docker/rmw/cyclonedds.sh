#!/usr/bin/env bash
# Install Eclipse Cyclone DDS RMW and make it the default.
set -euo pipefail

apt-get update
apt-get install -y "ros-${ROS_DISTRO}-rmw-cyclonedds-cpp"

cat > /etc/profile.d/ipc_rmw.sh <<'EOS'
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
EOS
chmod +x /etc/profile.d/ipc_rmw.sh
