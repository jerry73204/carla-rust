# carla-rust justfile
# Run `just --list` to see all available recipes

# Default Cargo profile for builds
profile := "dev-release"

# Supported CARLA versions
versions := "0.9.14 0.9.15 0.9.16 0.10.0"

# Show all available recipes (default)
@_default:
    just --list

# Build library for all supported CARLA versions (parallel)
build:
    #!/usr/bin/env bash
    set -euo pipefail
    parallel --keep-order --halt now,fail=1 --tag \
        'CARLA_VERSION={} CARGO_TARGET_DIR=target/carla-{} cargo build --profile {{ profile }}' \
        ::: {{ versions }}
    echo "All versions built successfully!"

# Build all targets (lib, tests, examples) for all supported CARLA versions (parallel)
build-all:
    #!/usr/bin/env bash
    set -euo pipefail
    parallel --keep-order --halt now,fail=1 --tag \
        'CARLA_VERSION={} CARGO_TARGET_DIR=target/carla-{} cargo build --all-targets --profile {{ profile }}' \
        ::: {{ versions }}
    echo "All versions built successfully!"

# Run unit tests for all supported CARLA versions (parallel)
test:
    #!/usr/bin/env bash
    set -euo pipefail
    parallel --keep-order --halt now,fail=1 --tag \
        'CARLA_VERSION={} CARGO_TARGET_DIR=target/carla-{} cargo nextest run --no-tests pass --no-fail-fast --cargo-profile {{ profile }}' \
        ::: {{ versions }}
    echo "All versions tested successfully!"

# Run unit tests only — no CARLA simulator needed
test-unit:
    #!/usr/bin/env bash
    set -euo pipefail
    cargo nextest run --cargo-profile {{ profile }} -E 'kind(lib)' --no-tests pass --no-fail-fast

# Run integration tests — starts CARLA, runs tests, stops CARLA on exit.
# Use `just test-integration-external` to skip start/stop and manage CARLA yourself.
test-integration carla_version="0.9.16":
    #!/usr/bin/env bash
    set -euo pipefail
    BASE_PORT="${CARLA_TEST_BASE_PORT:-2000}"
    SERVERS="${CARLA_TEST_SERVERS:-1}"
    _cleanup() {
        echo "Stopping CARLA test servers..." >&2
        pkill -9 -f "CarlaUE4.*-carla-rpc-port" 2>/dev/null || true
        rm -f tmp/test-servers/port-*.pid
    }
    trap _cleanup EXIT
    # Start one server per pool slot in parallel, wait for all to be TCP-ready
    for i in $(seq 0 $((SERVERS - 1))); do
        PORT=$((BASE_PORT + i * 10))
        ./scripts/start-carla-test.sh "$PORT" &
    done
    wait
    CARLA_VERSION={{ carla_version }} \
    CARGO_TARGET_DIR=target/carla-{{ carla_version }} \
        cargo nextest run --cargo-profile {{ profile }} -E 'kind(test)' --no-fail-fast

# Run integration tests against an already-running CARLA simulator.
# Start CARLA first:  ./scripts/start-carla-test.sh 2000
# Stop CARLA after:   pkill -f CarlaUE4
test-integration-external carla_version="0.9.16":
    #!/usr/bin/env bash
    set -euo pipefail
    CARLA_VERSION={{ carla_version }} \
    CARGO_TARGET_DIR=target/carla-{{ carla_version }} \
        cargo nextest run --cargo-profile {{ profile }} -E 'kind(test)' --no-fail-fast

# Run Rust and C++ checks for all supported CARLA versions
check: check-rust check-cpp

# Run Rust checks (fmt + clippy on lib) for all supported CARLA versions (parallel)
check-rust:
    #!/usr/bin/env bash
    set -euo pipefail
    cargo +nightly fmt --check
    parallel --keep-order --halt now,fail=1 --tag \
        'CARLA_VERSION={} CARGO_TARGET_DIR=target/carla-{} cargo clippy -- -D warnings' \
        ::: {{ versions }}

# Run Rust checks on all targets (lib, tests, examples) for all supported CARLA versions (parallel)
check-rust-all:
    #!/usr/bin/env bash
    set -euo pipefail
    cargo +nightly fmt --check
    parallel --keep-order --halt now,fail=1 --tag \
        'CARLA_VERSION={} CARGO_TARGET_DIR=target/carla-{} cargo clippy --all-targets -- -D warnings' \
        ::: {{ versions }}

# Run C++ checks (format + tidy)
check-cpp: check-cpp-format check-cpp-tidy

# Run C++ format check
check-cpp-format:
    find carla-sys/csrc -name "*.hpp" -o -name "*.cpp" | xargs clang-format --dry-run --Werror --style=file

# Run clang-tidy
check-cpp-tidy:
    #!/usr/bin/env bash
    set -euo pipefail
    # Use the default version's headers for tidy
    VERSION="${CARLA_VERSION:-0.10.0}"
    TARGET_DIR="target/carla-${VERSION}"
    CARLA_INCLUDE=$(find "${TARGET_DIR}"/{{ profile }}/build/carla-sys-*/out/libcarla_client/*/include -maxdepth 0 -type d 2>/dev/null | head -1 || true)
    if [ -z "$CARLA_INCLUDE" ]; then
        echo "Error: CARLA headers not found. Run 'just build' first."
        exit 1
    else
        echo "Using CARLA headers from: $CARLA_INCLUDE"
        echo "Running clang-tidy on all files..."
        CPP_STD=$([[ "$VERSION" == "0.10.0" ]] && echo "c++20" || echo "c++14")
        clang-tidy $(find carla-sys/csrc -name "*.hpp" -o -name "*.cpp") -- -std=$CPP_STD -Icarla-sys/csrc -I$CARLA_INCLUDE
    fi

# Run all CI checks (build, check, test)
ci: build check-rust check-cpp-format test

# Format Rust and C++ code
format: format-rust format-cpp

# Format Rust code only
format-rust:
    cargo +nightly fmt --all

# Format C++ code only
format-cpp:
    find carla-sys/csrc -name "*.hpp" -o -name "*.cpp" | xargs clang-format -i --style=file

# Remove build artifacts for all CARLA versions
clean:
    rm -rf target/carla-*

# Build documentation
doc:
    #!/usr/bin/env bash
    set -euo pipefail
    VERSION="${CARLA_VERSION:-0.10.0}"
    CARLA_VERSION="${VERSION}" CARGO_TARGET_DIR="target/carla-${VERSION}" \
        cargo doc --no-deps --open

# Regenerate bindings for all CARLA versions (downloads prebuilt packages)
regen-bindings:
    #!/usr/bin/env bash
    set -euo pipefail

    echo "Regenerating bindings for all CARLA versions using prebuilt packages..."
    echo ""

    for VERSION in {{ versions }}; do
        echo "=================================================="
        echo "Regenerating bindings for CARLA ${VERSION}..."
        echo "=================================================="

        CARLA_VERSION="${VERSION}" CARGO_TARGET_DIR="target/carla-${VERSION}" \
        cargo build -p carla-sys --features save-bindings --profile {{ profile }}

        echo ""
        echo "✓ Bindings saved: carla-sys/generated/bindings.${VERSION}.rs"
        echo ""
    done

    echo "=================================================="
    echo "All bindings regenerated successfully!"
    echo "=================================================="

# Build prebuilt package for a specific CARLA version (requires source dir and version)
# Example: just build-prebuilt ~/carla 0.9.16
build-prebuilt CARLA_SRC_DIR VERSION:
    #!/usr/bin/env bash
    set -euo pipefail

    # Validate CARLA source directory exists
    if [ ! -d "{{ CARLA_SRC_DIR }}" ]; then
        echo "Error: CARLA source directory does not exist: {{ CARLA_SRC_DIR }}"
        exit 1
    fi

    # Validate version format
    if ! echo "{{ VERSION }}" | grep -qE '^0\.(9\.(14|15|16)|10\.0)$'; then
        echo "Error: Invalid CARLA version: {{ VERSION }}"
        echo "Supported versions: 0.9.14, 0.9.15, 0.9.16, 0.10.0"
        exit 1
    fi

    echo "Building prebuilt package for CARLA {{ VERSION }} from {{ CARLA_SRC_DIR }}"
    echo ""

    CARLA_DIR="{{ CARLA_SRC_DIR }}" \
    CARLA_VERSION="{{ VERSION }}" \
    CARGO_TARGET_DIR="target/carla-{{ VERSION }}" \
    cargo build -p carla-sys --features build-prebuilt --profile {{ profile }}

    echo ""
    echo "✓ Prebuilt package created: carla-sys/generated/libcarla_client.{{ VERSION }}-$(rustc -vV | grep host | cut -d' ' -f2).tar.zstd"
    echo "✓ Bindings saved: carla-sys/generated/bindings.{{ VERSION }}.rs"

# Simulate docs.rs build (CARLA 0.10.0, docs-only feature, no bindings generation)
doc-docsrs:
    #!/usr/bin/env bash
    set -euo pipefail
    echo "Building documentation as docs.rs would (CARLA 0.10.0, docs-only feature)..."
    CARLA_VERSION=0.10.0 RUSTDOCFLAGS="--cfg carla_version_0100" cargo doc -p carla --no-deps --no-default-features --features docs-only --open
