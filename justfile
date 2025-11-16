# carla-rust justfile
# Run `just --list` to see all available recipes

# Show all available recipes (default)
@_default:
    just --list

# Build with dev-release profile
build:
    cargo build --all-targets --profile dev-release

# Run unit tests
test:
    cargo nextest run --no-tests pass --no-fail-fast --cargo-profile dev-release

# Format Rust and C++ code
format: format-rust format-cpp

# Format Rust code only
format-rust:
    cargo +nightly fmt --all

# Format C++ code only
format-cpp:
    find carla-sys/csrc -name "*.hpp" -o -name "*.cpp" | xargs clang-format -i --style=file

# Run Rust and C++ lints
lint: lint-rust lint-cpp

# Run Rust lints only
lint-rust:
    cargo +nightly fmt --check
    cargo clippy --all-targets -- -D warnings

# Run C++ lints (format + tidy)
lint-cpp: lint-cpp-format lint-cpp-tidy

# Run C++ format check only
lint-cpp-format:
    find carla-sys/csrc -name "*.hpp" -o -name "*.cpp" | xargs clang-format --dry-run --Werror --style=file

# Run clang-tidy only
lint-cpp-tidy:
    #!/usr/bin/env bash
    set -euo pipefail
    CARLA_INCLUDE=$(find target/dev-release/build/carla-sys-*/out/libcarla_client/*/include -type d 2>/dev/null | head -1)
    if [ -z "$CARLA_INCLUDE" ]; then
        CARLA_INCLUDE=$(find target/debug/build/carla-sys-*/out/libcarla_client/*/include -type d 2>/dev/null | head -1)
    fi
    if [ -z "$CARLA_INCLUDE" ]; then
        echo "Error: CARLA headers not found. Run 'just build' first."
        exit 1
    else
        echo "Using CARLA headers from: $CARLA_INCLUDE"
        echo "Running clang-tidy on all files..."
        clang-tidy $(find carla-sys/csrc -name "*.hpp" -o -name "*.cpp") -- -std=c++14 -Icarla-sys/csrc -I$CARLA_INCLUDE
    fi

# Remove build artifacts
clean:
    cargo clean

# Build documentation
doc:
    cargo doc --no-deps --open

# Regenerate bindings for all CARLA versions (downloads prebuilt packages)
regen-bindings:
    #!/usr/bin/env bash
    set -euo pipefail

    echo "Regenerating bindings for all CARLA versions using prebuilt packages..."
    echo ""

    for VERSION in 0.9.14 0.9.15 0.9.16; do
        echo "=================================================="
        echo "Regenerating bindings for CARLA ${VERSION}..."
        echo "=================================================="

        CARLA_VERSION="${VERSION}" \
        cargo build -p carla-sys --features save-bindings --profile dev-release

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
    if ! echo "{{ VERSION }}" | grep -qE '^0\.9\.(14|15|16)$'; then
        echo "Error: Invalid CARLA version: {{ VERSION }}"
        echo "Supported versions: 0.9.14, 0.9.15, 0.9.16"
        exit 1
    fi

    echo "Building prebuilt package for CARLA {{ VERSION }} from {{ CARLA_SRC_DIR }}"
    echo ""

    CARLA_DIR="{{ CARLA_SRC_DIR }}" \
    CARLA_VERSION="{{ VERSION }}" \
    cargo build -p carla-sys --features build-prebuilt --profile dev-release

    echo ""
    echo "✓ Prebuilt package created: carla-sys/generated/libcarla_client.{{ VERSION }}-$(rustc -vV | grep host | cut -d' ' -f2).tar.zstd"
    echo "✓ Bindings saved: carla-sys/generated/bindings.{{ VERSION }}.rs"

# Simulate docs.rs build (CARLA 0.9.16, docs-only feature, no bindings generation)
doc-docsrs:
    #!/usr/bin/env bash
    set -euo pipefail
    echo "Building documentation as docs.rs would (CARLA 0.9.16, docs-only feature)..."
    CARLA_VERSION=0.9.16 RUSTDOCFLAGS="--cfg carla_version_0916" cargo doc -p carla --no-deps --no-default-features --features docs-only --open
