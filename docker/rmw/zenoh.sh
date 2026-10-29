#!/usr/bin/env bash
# Zenoh RMW implementation (rmw_zenoh_cpp).
set -euo pipefail

apt-get update
apt-get install -y "ros-${ROS_DISTRO}-rmw-zenoh-cpp"

cat > /etc/profile.d/ipc_rmw.sh <<'EOS'
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_ROUTER_CHECK_ATTEMPTS=-1
EOS
chmod +x /etc/profile.d/ipc_rmw.sh

# Zenoh needs a router (rmw_zenohd) running before any rmw_zenoh_cpp node can
# discover peers. This entrypoint starts it, waits for it to report ready,
# then execs the requested command.
cat > /usr/local/bin/ipc_zenoh_entrypoint.sh <<'EOS'
#!/bin/bash
set -e

source "/opt/ros/${ROS_DISTRO}/setup.bash" --
source /ros2_ws/install/setup.bash
source /etc/profile.d/ipc_rmw.sh

ZENOHD_LOG=/tmp/zenohd.log
: > "${ZENOHD_LOG}"
ros2 run rmw_zenoh_cpp rmw_zenohd >"${ZENOHD_LOG}" 2>&1 &
ZENOHD_PID=$!

for _ in $(seq 1 200); do
    if ! kill -0 "${ZENOHD_PID}" 2>/dev/null; then
        echo "[ipc_zenoh_entrypoint] rmw_zenohd exited early; log:" >&2
        cat "${ZENOHD_LOG}" >&2
        exit 1
    fi
    if grep -q "Started Zenoh router" "${ZENOHD_LOG}" 2>/dev/null; then
        break
    fi
    sleep 0.1
done

if ! grep -q "Started Zenoh router" "${ZENOHD_LOG}" 2>/dev/null; then
    echo "[ipc_zenoh_entrypoint] rmw_zenohd did not report ready within 20s; log:" >&2
    cat "${ZENOHD_LOG}" >&2
    exit 1
fi

exec "$@"
EOS
chmod +x /usr/local/bin/ipc_zenoh_entrypoint.sh
