#!/bin/bash
# Usage:
#   ./run_all.sh                                # default matrix
#   ./run_all.sh "cyclonedds zenoh"             # override just the RMW list
#   MATRIX_MESSAGES="sensor_msgs/PointCloud2 sensor_msgs/Image std_msgs/Float64" \
#   MATRIX_PAYLOADS="1024 1840000 8388608" \
#   ./run_all.sh
#
# Runs every blog case for the cartesian product RMW x message x payload.
# Each run prints a summary table and writes one CSV to data/results.
#
# Every axis is a space-separated list and can be overridden from the
# environment. Fixed-size message types ignore the payload axis, so they run
# once rather than once per payload.
set -euo pipefail

MATRIX_RMWS="${MATRIX_RMWS:-cyclonedds fastdds zenoh}"
MATRIX_MESSAGES="${MATRIX_MESSAGES:-sensor_msgs/PointCloud2}"
MATRIX_PAYLOADS="${MATRIX_PAYLOADS:-1840000}"
MATRIX_CASES="${MATRIX_CASES:-all}"
MATRIX_ITERATIONS="${MATRIX_ITERATIONS:-100}"
MATRIX_IPC="${MATRIX_IPC:-true}"

if [[ $# -ge 1 && -n "${1:-}" ]]; then
    MATRIX_RMWS="$1"
fi

# Must match the types with fill_payload() overloads in
# src/ipc_zero_copy_tests/include/ipc_zero_copy_tests/payload_traits.hpp.
VARIABLE_SIZE_TYPES=" sensor_msgs/PointCloud2 sensor_msgs/Image sensor_msgs/LaserScan
    sensor_msgs/JointState sensor_msgs/PointCloud nav_msgs/Path std_msgs/String
    std_msgs/ByteMultiArray std_msgs/Float32MultiArray std_msgs/Float64MultiArray
    std_msgs/Int8MultiArray std_msgs/Int16MultiArray std_msgs/Int32MultiArray
    std_msgs/Int64MultiArray std_msgs/UInt8MultiArray std_msgs/UInt16MultiArray
    std_msgs/UInt32MultiArray std_msgs/UInt64MultiArray "

payloads_for() {
    if [[ "${VARIABLE_SIZE_TYPES}" == *[[:space:]]"$1"[[:space:]]* ]]; then
        echo "$MATRIX_PAYLOADS"
    else
        # Fixed-size type: payload_bytes is ignored, run once.
        echo "${MATRIX_PAYLOADS%% *}"
    fi
}

COMPOSE=(docker compose -f docker/docker-compose.yaml)
mkdir -p data/results

total=0
for _ in $MATRIX_RMWS; do
    for msg in $MATRIX_MESSAGES; do
        for _ in $(payloads_for "$msg"); do
            total=$((total + 1))
        done
    done
done

echo "[run_all] matrix: $total runs"
echo "           rmws=[$MATRIX_RMWS]"
echo "           messages=[$MATRIX_MESSAGES]"
echo "           payloads=[$MATRIX_PAYLOADS]"
echo "           cases=$MATRIX_CASES  iterations=$MATRIX_ITERATIONS  ipc=$MATRIX_IPC"

i=0
failed=0
for svc in $MATRIX_RMWS; do
    # Build each image once up front rather than on every run.
    "${COMPOSE[@]}" build "$svc"
    for msg in $MATRIX_MESSAGES; do
        for payload in $(payloads_for "$msg"); do
            i=$((i + 1))
            echo
            echo "=========================================================="
            echo "[run_all] ($i/$total) svc=$svc msg=$msg payload=${payload}B"
            echo "=========================================================="
            if ! "${COMPOSE[@]}" run --rm "$svc" \
                ros2 launch ipc_zero_copy_tests ipc_cases.launch.py \
                    message_type:="$msg" \
                    payload_bytes:="$payload" \
                    cases:="$MATRIX_CASES" \
                    iterations:="$MATRIX_ITERATIONS" \
                    use_intra_process_comms:="$MATRIX_IPC"
            then
                echo "[run_all] run $i failed (svc=$svc msg=$msg payload=$payload)" >&2
                failed=$((failed + 1))
            fi
        done
    done
done

echo
echo "[run_all] done: $((total - failed))/$total runs succeeded; CSVs in data/results/"
[[ $failed -eq 0 ]]
