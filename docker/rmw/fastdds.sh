#!/usr/bin/env bash
# Fast DDS (rmw_fastrtps_cpp).
set -euo pipefail

apt-get update
apt-get install -y "ros-${ROS_DISTRO}-rmw-fastrtps-cpp"

cat > /etc/profile.d/ipc_rmw.sh <<'EOS'
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
EOS
chmod +x /etc/profile.d/ipc_rmw.sh
