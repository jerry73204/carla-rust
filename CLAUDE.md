# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository Overview

This is a Rust client library for CARLA simulator (autonomous driving simulator). It provides safe Rust bindings to CARLA's C++ client library through a layered architecture:

1. **carla-sys** - Low-level FFI bindings using autocxx
2. **carla** - High-level, idiomatic Rust API
3. **carla-src** - Build utilities for CARLA C++ library

The repository was originally compatible with CARLA simulator version 0.9.14, but the submodule has been updated to 0.10.0 which introduces significant API changes.

## Build Commands

### Basic Build
```bash
cargo build
```

### Build with CARLA from Source
```bash
cargo build --features build-lib
```

### Build with Pre-installed CARLA
```bash
export CARLA_DIR=/path/to/carla/simulator
cargo build
```

### Ubuntu 22.04+ (Clang 12 Required)
```bash
sudo apt install clang-12 libclang-12-dev
export LLVM_CONFIG_PATH=/usr/bin/llvm-config-12
export LIBCLANG_PATH=/usr/lib/llvm-12/lib
export LIBCLANG_STATIC_PATH=/usr/lib/llvm-12/lib
export CLANG_PATH=/usr/bin/clang-12
cargo build
```

### Development Commands
```bash
# Check compilation without building
cargo check

# Build and open documentation
cargo doc --open

# Run tests (note: no unit tests currently exist)
cargo test

# Build with specific features
cargo build --features save-lib      # Save built library as tarball
cargo build --features save-bindgen  # Save generated bindings
cargo build --features docs-only     # Skip build for docs.rs
```

## Architecture

### FFI Layer (carla-sys)
- Uses **autocxx** to generate bindings from C++ headers
- Custom C++ headers in `csrc/` wrap CARLA's API
- `src/ffi.rs` defines exposed types using `include_cpp!` macro
- `build.rs` handles:
  - Downloading pre-built binaries (default)
  - Building from source with `build-lib` feature
  - Generating FFI bindings

### High-Level API (carla)
Key modules:
- `client/` - Client connection, world management, actor spawning
- `sensor/` - Sensor data structures and handling
- `traffic_manager/` - Traffic simulation control
- `geom.rs` - Geometry primitives (Location, Rotation, Transform)
- `rpc/` - RPC protocol structures

### Pre-built Binary System
- Pre-built binaries downloaded from GitHub releases
- URLs defined in `carla-sys/index.json5`
- Currently only supports `x86_64-unknown-linux-gnu`
- Compressed with zstd

## Key Dependencies
- **autocxx/cxx** - C++ FFI generation
- **nalgebra** - Linear algebra operations
- **ndarray** - N-dimensional arrays for sensor data
- **anyhow** - Error handling

## Important Notes
- The `carla-simulator/` directory contains the full CARLA source as a git submodule
- No example files exist despite README mentioning them
- No unit tests are present in the codebase
- The project uses Cargo workspace with resolver v2

## CARLA 0.10.0 API Changes

The CARLA simulator has been updated to version 0.10.0, which introduces major changes:

### Major Changes
- **Unreal Engine 5.5**: Migrated from UE 4.26 to 5.5
- **Physics Engine**: Changed from PhysX to Chaos
- **Build System**: Migrated from Make to CMake
- **Native ROS2 Support**: New `carla/ros2/` module with native publishers/subscribers
- **Python Support**: Now supports Python 3.8-3.12 (dropped 3.7)

### API Changes
- **Removed**: Light Manager API (`LightManager.h` still exists but is deprecated)
- **Removed**: RSS (Road Safety) functionality
- **New**: Vehicle door control (`OpenDoor`/`CloseDoor` methods)
- **New**: Native ROS2 integration without external bridge
- **Modified**: Semantic/instance segmentation sensors refactored

### C++ API Structure
- `carla/client/` - Main client API (Client, World, Actor, Vehicle, etc.)
- `carla/sensor/` - Sensor data structures and serialization
- `carla/ros2/` - Native ROS2 publishers and subscribers
- `carla/rpc/` - Remote procedure call data structures
- `carla/road/` - OpenDRIVE road network representation
- `carla/trafficmanager/` - Traffic management system

### Potential Binding Updates Needed
1. Remove or deprecate Light Manager bindings
2. Add vehicle door control bindings (`VehicleDoor` enum)
3. Consider adding ROS2 native support bindings
4. Update build process to handle CMake instead of Make
5. Verify all sensor bindings still work with refactored segmentation

## Development Utilities

### LibCarla C Build Command
```bash
cd libcarla_c && rm -rf build && cmake -B build -S . -DCMAKE_INSTALL_PREFIX=./install && cmake --build build && cmake --install build
```

## Memories

- Run the script libcarla_c/build_libcarla_c.sh to build the C library for testing.
- Please prefix commands with `source /opt/ros/humble/setup.bash` to enable ROS2 libraries.
- Run `cargo build --all-targets --all-features` at the top-level directory to build the whole Rust workspace.