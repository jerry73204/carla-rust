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
