#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
CARLACTL="$SCRIPT_DIR/simulators/carlactl"

# Default values
CARLA_VERSION="0.9.16"
TIMEOUT_SECONDS=60
REQUESTED_EXAMPLES=()

# State
LOG_DIR=""
CARLA_STARTED=false
PASSED_COUNT=0
FAILED_COUNT=0
TOTAL_TIME=0

usage() {
    echo "Usage: $0 [OPTIONS] [EXAMPLES...]"
    echo ""
    echo "Run CARLA Rust examples with automatic server management and logging."
    echo ""
    echo "OPTIONS:"
    echo "  -v, --version VERSION    CARLA version to use (default: 0.9.16)"
    echo "  -t, --timeout SECONDS    Per-example timeout in seconds (default: 60)"
    echo "  -h, --help              Show this help message"
    echo ""
    echo "EXAMPLES:"
    echo "  Optional list of specific examples to run."
    echo "  If omitted, all examples are discovered via cargo metadata."
    echo ""
    echo "EXAMPLES:"
    echo "  $0                           # Run all examples with CARLA 0.9.16"
    echo "  $0 --version 0.9.14          # Run all examples with CARLA 0.9.14"
    echo "  $0 connect world_info        # Run specific examples"
    echo "  $0 -t 120 spawn_vehicle      # Run with 120s timeout"
    exit 0
}

log() {
    echo "$@" >&2
}

error() {
    echo "Error: $@" >&2
}

# Parse command line arguments
parse_args() {
    while [[ $# -gt 0 ]]; do
        case "$1" in
            -h|--help)
                usage
                ;;
            -v|--version)
                if [[ $# -lt 2 ]]; then
                    error "--version requires an argument"
                    exit 1
                fi
                CARLA_VERSION="$2"
                shift 2
                ;;
            -t|--timeout)
                if [[ $# -lt 2 ]]; then
                    error "--timeout requires an argument"
                    exit 1
                fi
                TIMEOUT_SECONDS="$2"
                shift 2
                ;;
            -*)
                error "Unknown option: $1"
                usage
                ;;
            *)
                REQUESTED_EXAMPLES+=("$1")
                shift
                ;;
        esac
    done
}

# Check dependencies
check_dependencies() {
    local missing=()

    if ! command -v jq &> /dev/null; then
        missing+=("jq")
    fi

    if ! command -v timeout &> /dev/null; then
        missing+=("timeout")
    fi

    if [[ ! -x "$CARLACTL" ]]; then
        error "carlactl not found at: $CARLACTL"
        exit 1
    fi

    if [[ ${#missing[@]} -gt 0 ]]; then
        error "Missing required dependencies: ${missing[*]}"
        echo "Install with: sudo apt install ${missing[*]}" >&2
        exit 1
    fi
}

# Discover examples using cargo metadata
discover_examples() {
    log "Discovering examples via cargo metadata..."

    local examples
    examples=$(cd "$REPO_ROOT" && cargo metadata --format-version 1 --no-deps 2>/dev/null | \
        jq -r '.packages[] | select(.name == "carla") | .targets[] | select(.kind[] == "example") | .name')

    if [[ -z "$examples" ]]; then
        error "No examples found"
        exit 1
    fi

    echo "$examples"
}

# Filter examples based on requested list
filter_examples() {
    local all_examples=("$@")

    if [[ ${#REQUESTED_EXAMPLES[@]} -eq 0 ]]; then
        # Return all examples
        printf '%s\n' "${all_examples[@]}"
        return
    fi

    # Filter to requested examples
    local filtered=()
    for requested in "${REQUESTED_EXAMPLES[@]}"; do
        local found=false
        for example in "${all_examples[@]}"; do
            if [[ "$example" == "$requested" ]]; then
                filtered+=("$example")
                found=true
                break
            fi
        done

        if [[ "$found" == false ]]; then
            error "Unknown example: $requested"
            echo "Available examples: ${all_examples[*]}" >&2
            exit 1
        fi
    done

    printf '%s\n' "${filtered[@]}"
}

# Create log directory
create_log_dir() {
    local timestamp=$(date +%Y%m%d-%H%M%S)
    LOG_DIR="$REPO_ROOT/logs/run-examples-$timestamp"
    mkdir -p "$LOG_DIR"
    log "Log directory: $LOG_DIR"
}

# Check if CARLA is already running
check_carla_status() {
    if "$CARLACTL" status "$CARLA_VERSION" &>/dev/null; then
        return 0  # Running
    else
        return 1  # Not running
    fi
}

# Restart CARLA server (for clean state before each example)
restart_carla() {
    local log_suffix="${1:-restart}"

    log "Restarting CARLA $CARLA_VERSION for clean state..."

    if ! "$CARLACTL" restart "$CARLA_VERSION" &> "$LOG_DIR/carla-$log_suffix.log"; then
        error "Failed to restart CARLA $CARLA_VERSION"
        cat "$LOG_DIR/carla-$log_suffix.log" >&2
        exit 1
    fi

    CARLA_STARTED=true

    # Wait for server to be ready (using TCP connection check)
    log -n "Waiting for server to be ready"
    local max_attempts=60
    local attempt=0

    while [[ $attempt -lt $max_attempts ]]; do
        # Try to connect to CARLA port 2000
        if timeout 1 bash -c "echo > /dev/tcp/localhost/2000" 2>/dev/null; then
            log ""

            # Wait a moment for server to stabilize
            sleep 2

            # Verify with carlactl status that server is still running
            if check_carla_status; then
                log "CARLA ready."
                return 0
            else
                error "CARLA accepted connection but then crashed"
                error "Check logs: journalctl --user -u carla@$CARLA_VERSION.service -n 50"
                exit 1
            fi
        fi
        log -n "."
        sleep 1
        attempt=$((attempt + 1))
    done

    log ""
    error "Server failed to become ready after ${max_attempts}s"
    exit 1
}

# Stop CARLA server
stop_carla() {
    if [[ "$CARLA_STARTED" == true ]]; then
        log ""
        log "Stopping CARLA $CARLA_VERSION..."
        "$CARLACTL" stop "$CARLA_VERSION" &> "$LOG_DIR/carla-shutdown.log" || true
        CARLA_STARTED=false
    fi
}

# Cleanup on exit
cleanup() {
    stop_carla
}

# Build examples
build_examples() {
    log ""
    log "Building examples (CARLA_VERSION=$CARLA_VERSION)..."

    cd "$REPO_ROOT"
    if ! CARLA_VERSION="$CARLA_VERSION" make build &> "$LOG_DIR/build.log"; then
        error "Build failed. See: $LOG_DIR/build.log"
        tail -20 "$LOG_DIR/build.log" >&2
        exit 1
    fi

    log "Build complete."
}

# Run a single example
run_example() {
    local example_name="$1"
    local index="$2"
    local total="$3"
    local log_file="$LOG_DIR/$example_name.log"
    local example_binary="$REPO_ROOT/target/dev-release/examples/$example_name"

    # Restart CARLA before each example for clean state
    log ""
    log "[$index/$total] Preparing to run: $example_name"
    restart_carla "restart-before-$example_name"

    local start_time=$(date +%s)
    local exit_code=0

    # Run the binary directly (already built by make build)
    # Use 'timeout' command and capture its exit code
    timeout "$TIMEOUT_SECONDS" "$example_binary" &> "$log_file"
    exit_code=$?

    local end_time=$(date +%s)
    local duration=$((end_time - start_time))
    TOTAL_TIME=$((TOTAL_TIME + duration))

    # Check if CARLA crashed during the example
    local carla_crashed=false
    if ! check_carla_status; then
        carla_crashed=true
    fi

    # Format output
    local padding=$(printf '%.0s.' {1..20})
    local display_name="${example_name}${padding}"
    display_name="${display_name:0:20}"

    # Check exit code and CARLA status
    # 0 = success
    # 124 = timeout killed it
    # 134 = aborted (SIGABRT)
    # 137 = killed (SIGKILL)
    # anything else = failure
    if [[ "$carla_crashed" == true ]]; then
        log "[$index/$total] $display_name CARLA CRASH (${duration}s) ⚠️"
        FAILED_COUNT=$((FAILED_COUNT + 1))
        echo "CARLA_CRASH ${duration}s exit=$exit_code" > "$LOG_DIR/$example_name.result"
    elif [[ $exit_code -eq 0 ]]; then
        log "[$index/$total] $display_name PASS (${duration}s)"
        PASSED_COUNT=$((PASSED_COUNT + 1))
        echo "PASS ${duration}s" > "$LOG_DIR/$example_name.result"
    elif [[ $exit_code -eq 124 ]]; then
        log "[$index/$total] $display_name TIMEOUT (${TIMEOUT_SECONDS}s)"
        FAILED_COUNT=$((FAILED_COUNT + 1))
        echo "TIMEOUT ${TIMEOUT_SECONDS}s" > "$LOG_DIR/$example_name.result"
    elif [[ $exit_code -eq 134 ]]; then
        log "[$index/$total] $display_name ABORT (${duration}s)"
        FAILED_COUNT=$((FAILED_COUNT + 1))
        echo "ABORT ${duration}s (SIGABRT)" > "$LOG_DIR/$example_name.result"
    elif [[ $exit_code -eq 137 ]]; then
        log "[$index/$total] $display_name KILLED (${duration}s)"
        FAILED_COUNT=$((FAILED_COUNT + 1))
        echo "KILLED ${duration}s (SIGKILL)" > "$LOG_DIR/$example_name.result"
    else
        log "[$index/$total] $display_name FAIL (${duration}s, exit=$exit_code)"
        FAILED_COUNT=$((FAILED_COUNT + 1))
        echo "FAIL ${duration}s exit=$exit_code" > "$LOG_DIR/$example_name.result"
    fi
}

# Generate summary report
generate_summary() {
    local summary_file="$LOG_DIR/summary.txt"

    {
        echo "CARLA Rust Examples - Test Run Summary"
        echo "========================================"
        echo ""
        echo "CARLA Version: $CARLA_VERSION"
        echo "Date: $(date)"
        echo "Timeout: ${TIMEOUT_SECONDS}s per example"
        echo ""
        echo "Results:"
        echo "--------"

        # Read all results
        for result_file in "$LOG_DIR"/*.result; do
            if [[ -f "$result_file" ]]; then
                local example_name=$(basename "$result_file" .result)
                local result=$(cat "$result_file")
                printf "%-25s %s\n" "$example_name" "$result"
            fi
        done

        echo ""
        echo "Summary:"
        echo "--------"
        echo "Passed:  $PASSED_COUNT"
        echo "Failed:  $FAILED_COUNT"
        echo "Total:   $((PASSED_COUNT + FAILED_COUNT))"
        echo "Time:    ${TOTAL_TIME}s"
    } > "$summary_file"

    echo ""
    cat "$summary_file"
}

# Main execution
main() {
    parse_args "$@"
    check_dependencies

    # Discover and filter examples
    local all_examples
    readarray -t all_examples < <(discover_examples)
    log "Found ${#all_examples[@]} examples"

    local examples_to_run
    readarray -t examples_to_run < <(filter_examples "${all_examples[@]}")

    if [[ ${#examples_to_run[@]} -eq 0 ]]; then
        error "No examples to run"
        exit 1
    fi

    # Setup
    create_log_dir
    trap cleanup EXIT INT TERM

    # Build first (before starting CARLA)
    build_examples

    # Run examples (each will restart CARLA for clean state)
    log ""
    log "Running ${#examples_to_run[@]} examples (logs: $LOG_DIR/)"
    log "Each example will restart CARLA for isolation"
    log ""

    local index=1
    local total=${#examples_to_run[@]}

    for example in "${examples_to_run[@]}"; do
        run_example "$example" "$index" "$total"
        index=$((index + 1))
    done

    # Generate summary
    generate_summary

    # Exit with failure if any examples failed
    if [[ $FAILED_COUNT -gt 0 ]]; then
        exit 1
    fi
}

main "$@"
