#!/usr/bin/env bash
# Start a CARLA simulator instance for integration tests.
# Called by the test infrastructure with port number as $1.
# Blocks until CARLA is ready to accept RPC connections, then exits.
# CARLA continues running as a detached background process.
#
# Environment variables:
#   CARLA_DIR              Path to CARLA installation (default: ~/Downloads/CARLA_0.9.16)
#   CARLA_START_ATTEMPTS   Number of start attempts before giving up (default: 3)
#   CARLA_READY_TIMEOUT    Seconds to wait for TCP ready per attempt (default: 150)
#   CARLA_SKIP_PATCH       Set to 1 to skip applying config patches
#   CARLA_MEMORY_MAX       cgroup memory cap via systemd-run (default: 6G, set to "" to disable)
set -e

SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
CARLA_DIR="${CARLA_DIR:-$HOME/Downloads/CARLA_0.9.16}"
PORT="${1:-2000}"
MAX_ATTEMPTS="${CARLA_START_ATTEMPTS:-3}"
READY_TIMEOUT="${CARLA_READY_TIMEOUT:-150}"
CARLA_MEMORY_MAX="${CARLA_MEMORY_MAX:-10G}"

export DISPLAY="${DISPLAY:-:1}"
export VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json

# Apply config patches unless explicitly skipped
if [ "${CARLA_SKIP_PATCH:-0}" != "1" ]; then
    "$SCRIPT_DIR/carla-config/apply.sh" "$CARLA_DIR" >&2
fi

start_once() {
    # Kill any existing CARLA on this port
    pkill -9 -f "CarlaUE4.*carla-rpc-port=${PORT}" 2>/dev/null || true
    # Stop any leftover systemd scope from a previous run
    systemctl --user stop "carla-test-${PORT}.scope" 2>/dev/null || true
    sleep 2

    cd "$CARLA_DIR"

    # -opengl: workaround for driver 580 / RTX 5090 Vulkan hang (GitHub Discussion #7076)
    # -nosound: suppress ALSA errors on headless servers
    CARLA_CMD=(./CarlaUE4.sh -quality-level=Low -nosound -RenderOffScreen -opengl
        -carla-rpc-port="$PORT")

    # Launch CARLA inside a memory-capped systemd scope when possible.
    # systemd-run --scope exec()s into the command, so $! is the CARLA PID.
    # MemorySwapMax is not set so CARLA can use swap — it needs it when RAM is tight.
    if [ -n "$CARLA_MEMORY_MAX" ] && command -v systemd-run &>/dev/null; then
        echo "Starting CARLA with memory cap ${CARLA_MEMORY_MAX} (systemd-run --scope)" >&2
        nohup systemd-run --user --scope \
            -p MemoryMax="${CARLA_MEMORY_MAX}" \
            --unit="carla-test-${PORT}" \
            "${CARLA_CMD[@]}" \
            > /tmp/carla-test-port-${PORT}.log 2>&1 &
    else
        echo "Starting CARLA without memory cap (systemd-run unavailable or CARLA_MEMORY_MAX unset)" >&2
        nohup "${CARLA_CMD[@]}" \
            > /tmp/carla-test-port-${PORT}.log 2>&1 &
    fi
    CARLA_PID=$!
    echo "Started CARLA PID $CARLA_PID on port $PORT" >&2

    # Wait for TCP ready while monitoring process liveness
    for i in $(seq 1 "$READY_TIMEOUT"); do
        if ! kill -0 "$CARLA_PID" 2>/dev/null; then
            echo "CARLA PID $CARLA_PID died after ${i}s" >&2
            return 1
        fi
        if timeout 2 bash -c "echo > /dev/tcp/127.0.0.1/${PORT}" 2>/dev/null; then
            echo "CARLA TCP ready after ${i}s on port $PORT" >&2
            # Extra wait for full RPC initialization after TCP accepts
            sleep 5
            if kill -0 "$CARLA_PID" 2>/dev/null; then
                return 0
            else
                echo "CARLA died right after TCP ready" >&2
                return 1
            fi
        fi
        sleep 1
    done

    echo "CARLA did not become ready within ${READY_TIMEOUT}s" >&2
    kill -9 "$CARLA_PID" 2>/dev/null || true
    return 1
}

for attempt in $(seq 1 "$MAX_ATTEMPTS"); do
    echo "Starting CARLA (attempt $attempt/$MAX_ATTEMPTS)..." >&2
    if start_once; then
        echo "CARLA ready on port $PORT" >&2
        exit 0
    fi
    echo "Attempt $attempt failed, retrying..." >&2
    sleep 3
done

echo "ERROR: CARLA failed to start after $MAX_ATTEMPTS attempts" >&2
echo "Check logs: /tmp/carla-test-port-${PORT}.log" >&2
exit 1
