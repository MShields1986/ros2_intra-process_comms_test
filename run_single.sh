#!/bin/bash
# Usage: ./run_single.sh [rmw_service] [launch args...]
# where rmw_service is one of: cyclonedds, fastdds, zenoh (default cyclonedds)
#
# Examples:
#   ./run_single.sh
#   ./run_single.sh cyclonedds cases:=11,12,18
#   ./run_single.sh fastdds message_type:=sensor_msgs/Image payload_bytes:=6220800
#   ./run_single.sh cyclonedds cases:=custom custom_publish:=unique_ptr \
#       custom_subscriptions:=unique_ptr,const_ref custom_external_subscribers:=1
set -euo pipefail

RMW_SERVICE="${1:-cyclonedds}"
shift || true

# Ensure the host-side data dir exists so the mount works cleanly.
mkdir -p data/results

docker compose -f docker/docker-compose.yaml run --rm --build "${RMW_SERVICE}" \
    ros2 launch ipc_zero_copy_tests ipc_cases.launch.py "$@"
