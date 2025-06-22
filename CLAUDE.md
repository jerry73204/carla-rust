# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository Overview

This is a Rust client library for CARLA simulator (autonomous driving simulator). It provides safe Rust bindings to CARLA's C++ client library through a layered architecture:

1. **carla-sys** - FFI layer using CXX for C++ interoperability
2. **carla** - High-level, idiomatic Rust API

The repository is compatible with CARLA simulator version 0.10.0. The project uses a Cargo workspace with resolver v2.

## Build Commands

### Basic Build
```bash
# CARLA_ROOT must be set to the CARLA source directory
# Recommended: Use jerry73204's fork with CMake support
# git clone -b 0.10.0-libcarla-client-install https://github.com/jerry73204/carla.git
export CARLA_ROOT=/path/to/carla-source
cargo build --all-targets
```

### Ubuntu 22.04+ (Clang 12 Required)
```bash
sudo apt install clang-12 libclang-12-dev
export LLVM_CONFIG_PATH=/usr/bin/llvm-config-12
export LIBCLANG_PATH=/usr/lib/llvm-12/lib
export LIBCLANG_STATIC_PATH=/usr/lib/llvm-12/lib
export CLANG_PATH=/usr/bin/clang-12
cargo build --all-targets
```

### Development Commands
```bash
# Check compilation without building
cargo check --all-targets

# Build and open documentation
cargo doc --open

# Run tests (CARLA_ROOT must be set)
CARLA_ROOT=/path/to/carla-rust/carla-simulator cargo test --all-targets --all-features

# Build with specific features
cargo build --all-targets --features save-lib      # Save built library as tarball
cargo build --all-targets --features save-bindgen  # Save generated bindings
cargo build --all-targets --features docs-only     # Skip build for docs.rs
```

## Architecture

### FFI Layer (carla-sys)
- Uses **CXX** to generate safe FFI bindings
- Custom C++ bridge code in `cpp/carla_sys_bridge.cpp` wraps CARLA's API
- Headers in `include/carla_sys_bridge.h` define the C++ interface
- `src/ffi.rs` defines the Rust side of the CXX bridge
- Provides thin wrappers around CARLA C++ types with zero-copy where possible

### High-Level API (carla)
Key modules:
- `client/` - Client connection, world management, actor spawning
- `sensor/` - Sensor data structures and handling  
- `traffic_manager/` - Traffic simulation control
- `road/` - Map, waypoints, junctions, and navigation
- `actor/` - Vehicles, walkers, traffic lights/signs
- `geom.rs` - Geometry primitives (Location, Rotation, Transform)
- `rpc/` - RPC protocol structures

All submodules are public for direct imports (e.g., `use carla::client::Client`).

### Build System (carla-src)
- Downloads and builds CARLA C++ client library from source
- Pre-built binaries can be downloaded from GitHub releases
- Supports `x86_64-unknown-linux-gnu` platform
- Uses CMake for building CARLA C++ library
- Handles library path configuration for linking

## Key Dependencies
- **cxx** - Safe C++ FFI bridge (replaced autocxx)
- **nalgebra** - Linear algebra operations
- **ndarray** - N-dimensional arrays for sensor data  
- **anyhow/thiserror** - Error handling
- **derive_more** - Derive trait implementations
- **static_assertions** - Compile-time assertions

## Important Notes
- The `carla-simulator/` directory contains the full CARLA source as a git submodule
- Example files are in `carla-sys/examples/` and `carla/examples/` directories
- Integration tests are in `carla/tests/` directory
- Uses zero-copy wrapper types for collections (WaypointList, TransformList, etc.)
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

### LibCarla C Build Command (for testing)
```bash
cd libcarla_c && ./build_libcarla_c.sh
```

Or manually:
```bash
cd libcarla_c && rm -rf build && cmake -B build -S . -DCMAKE_INSTALL_PREFIX=./install && cmake --build build && cmake --install build
```

## Development Guidelines


### Missing FFI Functionality
When implementing Rust API functions that require FFI functionality not available in carla-sys:

**DO:**
- Use `todo!("FunctionName not yet implemented - missing FFI function FunctionName_Something")` for clear failure
- Add detailed TODO comments explaining what FFI functions are needed
- Use `#[allow(unused_variables)]` if needed to silence warnings on parameters

**DON'T:**
- Return dummy values (empty Vec, 0, false, etc.) without explicit TODO markers
- Use regular comments without todo!() macro for missing functionality
- Create silent errors that hide missing features from users

**Example of correct approach:**
```rust
pub fn get_affected_lane_waypoints(&self) -> Vec<crate::road::Waypoint> {
    // TODO: Implement using carla-sys FFI interface
    // This requires adding TrafficLight_GetAffectedLaneWaypoints FFI function
    todo!("TrafficLight::get_affected_lane_waypoints not yet implemented - missing FFI function TrafficLight_GetAffectedLaneWaypoints")
}
```

**Example of incorrect approach (causes silent errors):**
```rust
pub fn get_affected_lane_waypoints(&self) -> Vec<crate::road::Waypoint> {
    // This function would require additional FFI functions
    Vec::new()  # Silent error - user won't know this doesn't work
}
```

### FFI Implementation Priority
1. First implement FFI functions in carla-sys (C++ bridge + Rust declarations)
2. Then implement high-level Rust API using the FFI functions
3. Use todo!() macro for any missing pieces to make them explicit
4. Document in comments what specific FFI functions are needed

## Testing Framework

The Rust API includes a comprehensive testing framework that mirrors the C++ LibCarla tests:

### Test Structure
- **Unit Tests** (`carla/tests/unit/`) - Test individual components without server
- **Integration Tests** (`carla/tests/integration/`) - Test with real or mock CARLA server  
- **Benchmarks** (`carla/benches/`) - Performance benchmarks

### Running Tests
```bash
# Set up test fixtures (one time)
./setup_test_fixtures.sh

# Run unit tests
cargo test --lib

# Run all tests with mock server
cargo test

# Run integration tests with real CARLA server
CARLA_SERVER_HOST=localhost cargo test -- --ignored

# Run benchmarks
cargo bench
```

### Test Guidelines
- Mirror each C++ test with equivalent Rust test
- Use `todo!()` for unimplemented test cases
- Property-based tests for complex invariants
- Mock server for CI/CD testing

For detailed test design, see [TEST.md](./TEST.md).

## Memories

- Run the script `libcarla_c/build_libcarla_c.sh` to build the C library for testing.
- Please prefix commands with `source /opt/ros/humble/setup.bash` to enable ROS2 libraries.
- Run `cargo build --all-targets --all-features` at the top-level directory to build the whole Rust workspace.
- Run `cargo test --all-targets --all-features` to build and test the whole Rust project.
- Please use `client.rs` + `client/submod.rs` pattern instead of `client/mod.rs`, similar for other Rust modules.
- The carla-sys crate only provides C++ interface and essential items for Rust type conversion.
- If any FFI item is needed by the Rust library but is missing in the FFI crate, leave `todo!()` and comments in the Rust library instead of silent errors.
- Use clang-format to format C/C++ code. Be aware that it might break the source code. Use `// clang-format on` and `// clang-format off` marks to protect lines that would be broken by clang-format.
- Format code whenever you make changes to the source code. Build and test the project whenever you finish a feature.
- If a method, function or a fraction of code is not finished yet in Rust, leave a `todo!()` with TODO comments. Don't return a placeholder or dummy value. It would cause silent errors.
- The methods and functions in carla/ Rust API follow the Rust convention. For example, `id()` is preferred over `get_id()`.
- FFI types and functions are private to users in Rust API.
- Tests in upstream C++ source code should have Rust counterparts in our Rust API. Our Rust tests records the corresponding test in C++ in comments.