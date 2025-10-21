# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

This is a Rust client library for the CARLA simulator (version 0.9.14). The project uses FFI bindings to interface with the CARLA C++ client library and provides a safe, idiomatic Rust API.

## Crate Architecture

This is a Cargo workspace with three interdependent crates:

- **carla**: The main client library that users interact with. Provides high-level Rust abstractions over CARLA's C++ API.
- **carla-sys**: FFI bindings generated using [autocxx](https://github.com/google/autocxx). Links to `libcarla_client` and generates bindings from CARLA headers.
- **carla-src**: Build utility crate that handles downloading/building the CARLA C++ client library.

The dependency flow is: `carla` → `carla-sys` → `carla-src`

## Build System

### Standard Build (Recommended)

```bash
cargo build
```

This downloads prebuilt `libcarla_client` binaries from the index (if available for your target).

### Building with Custom CARLA Source

To use a custom CARLA repository:

```bash
# Clone and checkout CARLA 0.9.14
git clone https://github.com/carla-simulator/carla.git
cd carla
git checkout 0.9.14

# Build with custom source
export CARLA_DIR=/path/to/carla
cargo build --features build-lib
```

### Build Features

carla-sys supports these features:
- `build-lib`: Compile `libcarla_client` from source (requires CARLA_DIR)
- `save-lib`: Save compiled library as tarball for distribution
- `save-bindgen`: Save generated FFI bindings
- `docs-only`: Skip native library linking (for docs.rs)

### Environment Requirements

**Ubuntu 22.04+ users must use clang 12** due to compatibility issues:

```bash
sudo apt install clang-12 libclang-12-dev
export LLVM_CONFIG_PATH=/usr/bin/llvm-config-12
export LIBCLANG_PATH=/usr/lib/llvm-12/lib
export LIBCLANG_STATIC_PATH=/usr/lib/llvm-12/lib
export CLANG_PATH=/usr/bin/clang-12
```

## Testing

```bash
# Run all tests
cargo test

# Run tests for specific crate
cargo test -p carla
cargo test -p carla-sys
cargo test -p carla-src

# Run specific test
cargo test <test_name>
```

## Code Organization

### carla crate

- `src/client/`: Core client types - Actor, World, Client, Blueprint, Waypoint, etc.
- `src/geom/`: Geometry types with extension traits (Location, Rotation, Transform, Vector2D/3D)
- `src/rpc/`: RPC message types (VehicleControl, WeatherParameters, etc.)
- `src/sensor/`: Sensor data types
- `src/traffic_manager/`: Traffic manager bindings
- `src/road/`: Road topology types
- `src/prelude`: Commonly used extension traits

### carla-sys crate

- `src/ffi.rs`: Main FFI bindings using autocxx's `include_cpp!` macro
- `src/impls.rs`: Manual trait implementations for FFI types
- `csrc/`: Custom C++ code that replaces autocxx-incompatible classes
- `build.rs`: Downloads/extracts prebuilt binaries or builds from source

### carla-src crate

Utility functions for building `libcarla_client` from CARLA source. Used by carla-sys's build script.

## Key Technical Details

### FFI Strategy

- Uses [autocxx](https://github.com/google/autocxx) to generate C++ bindings automatically
- Custom C++ wrappers in `carla-sys/csrc/` handle types that autocxx can't process
- FFI boundaries are in `carla-sys`; safe Rust wrappers are in `carla`

### Extension Traits Pattern

The crate uses extension traits extensively to add Rust-idiomatic methods to FFI types:
- `ActorBase` trait for common actor operations
- `LocationExt`, `RotationExt`, `TransformExt` for geometry types
- `SensorDataBase` for sensor data
- `TimestampExt` for timestamp operations

All commonly used traits are re-exported in `carla::prelude`.

### Build Script Complexity

The `carla-sys/build.rs` handles multiple build scenarios:
1. Download prebuilt binaries from URL index (default)
2. Use prebuilt library from `CARLA_DIR`
3. Build from source with `build-lib` feature
4. Generate docs without native library (docs-only feature)

Prebuilt binaries are indexed in `carla-sys/index.json5` by target triple and version.

## Development Commands

```bash
# Build all crates
cargo build

# Build with all features
cargo build --all-features

# Check without building
cargo check

# Run clippy
cargo clippy

# Format code
cargo fmt

# Build documentation
cargo doc --no-deps --open

# Clean build artifacts
cargo clean
```

## Common Development Patterns

When adding new bindings:
1. Add C++ header includes to `carla-sys/src/ffi.rs`
2. Add type/function declarations to autocxx's `generate!` and `safety!` blocks
3. Add safe Rust wrappers in appropriate `carla/src/` modules
4. Export new types through module hierarchy

When types can't be handled by autocxx:
1. Create C++ wrapper in `carla-sys/csrc/carla_rust.hpp`
2. Implement in `carla-sys/csrc/carla_rust`
3. Include in FFI bindings
4. Add Rust wrapper in `carla` crate
