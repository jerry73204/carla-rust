# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

This is a Rust client library for the CARLA simulator. The project uses FFI bindings to interface with the CARLA C++ client library and provides a safe, idiomatic Rust API.

**Supported CARLA Versions:** 0.9.14, 0.9.15, 0.9.16 (default: 0.9.16)

## Best Practices

### Testing Strategy

**❌ DO NOT create tests in `carla/tests/`**

This project does NOT use the `tests/` directory for integration tests. Instead:

**✅ DO create examples in `carla/examples/`**

Examples serve dual purposes:
1. **Documentation** - Users can see working code
2. **Validation** - Automated testing via `scripts/run-examples.sh`

**Testing workflow:**
```bash
# 1. Create example (not test) in carla/examples/
vim carla/examples/my_feature_demo.rs

# 2. Build with dev-release profile
make build

# 3. Test manually
cargo run --example my_feature_demo --profile dev-release

# 4. Test with automation (requires CARLA simulator)
./scripts/run-examples.sh my_feature_demo
```

**Why this approach?**
- Examples are more useful to users than hidden test code
- Examples are easier to run manually for debugging
- Automated test infrastructure via `scripts/run-examples.sh`
- CARLA requires exclusive access, so serial testing is needed anyway

**What about unit tests?**
Unit tests stay in source files using `#[cfg(test)]`:
```rust
// src/rpc/some_type.rs
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_data_structure() {
        // Test implementation
    }
}
```

### Build Commands

**❌ DO NOT use `cargo build` directly**

**✅ DO use `make build` or add `--profile dev-release`**

```bash
# Recommended
make build

# If using cargo directly, always add --profile dev-release
cargo build --all-targets --profile dev-release
cargo test --profile dev-release
cargo run --example my_example --profile dev-release
```

Why? The `dev-release` profile is required for:
- Examples to be placed in correct directory for test automation
- Adequate performance for CARLA simulations
- Debug symbols for troubleshooting

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
cargo build --features build-prebuilt
```

### Build Features

carla-sys supports these features:
- `build-prebuilt`: Build from CARLA source and save prebuilt libraries and bindings for distribution (requires CARLA_DIR)
- `docs-only`: Skip native library linking (for docs.rs)

### CARLA Version Selection

The CARLA version can be selected using the `CARLA_VERSION` environment variable:

```bash
# Build for CARLA 0.9.16 (default)
cargo build

# Build for CARLA 0.9.14
CARLA_VERSION=0.9.14 cargo build

# Build for CARLA 0.9.15
CARLA_VERSION=0.9.15 cargo build

# Build for CARLA 0.9.16 explicitly
CARLA_VERSION=0.9.16 cargo build
```

When `build-prebuilt` feature is enabled, the version determines:
- Which boost library version to link against (1.80.0 for 0.9.14, 1.82.0 for 0.9.15, 1.84.0 for 0.9.16)
- Which header files to use from CARLA_DIR

When using prebuilt libraries, the version determines:
- Which prebuilt tarball to download from index.json5
- Which vendored headers to use from `carla-sys/headers/{VERSION}/`

### Environment Requirements

**LLVM/clang compatibility**: The build requires LLVM versions 11-13 due to autocxx compatibility issues with LLVM 14+. The build system automatically detects and configures any compatible version found.

**Supported versions:** LLVM 11, 12, 13
**Unsupported versions:** LLVM 14 and newer

Installation and configuration:

```bash
sudo apt install clang-12 libclang-12-dev
```

The build script automatically detects versions in this preference order: 13, 12, 11 (newest compatible first).

**Important:** On systems with multiple LLVM versions, autocxx may default to the newest version (e.g., LLVM 14+), which is incompatible. The build script automatically overrides this by setting the appropriate environment variables when it detects LLVM 11-13. You can manually override by setting `LLVM_CONFIG_PATH` before building.

## Testing

### Testing Philosophy: Examples as Tests

**IMPORTANT:** This project does NOT use `carla/tests/` for integration tests. Instead, we use **examples as executable tests**.

**Why?**
- Examples serve dual purposes: documentation AND validation
- Users can see working code that actually runs against CARLA
- Examples are easier to discover and run manually
- The `scripts/run-examples.sh` script provides automated test infrastructure

**What goes where:**

1. **Unit tests** (`#[cfg(test)]` in source files):
   - Data structure creation and validation
   - Type conversions
   - Pure functions with no I/O
   - No CARLA simulator required

2. **Examples** (`carla/examples/*.rs`):
   - Integration tests requiring CARLA simulator
   - Demonstrations of API usage
   - Feature validation
   - Both simple demos and complex scenarios

**Testing examples:**

```bash
# Run all examples with automatic CARLA management
./scripts/run-examples.sh

# Run specific examples
./scripts/run-examples.sh spawn_vehicle walker_control

# Run with specific CARLA version
./scripts/run-examples.sh --version 0.9.14

# Run with custom timeout (default: 60s)
./scripts/run-examples.sh --timeout 120
```

The `run-examples.sh` script:
- Automatically starts/stops CARLA for each example
- Restarts CARLA between examples for isolation
- Captures logs for debugging
- Generates a summary report with pass/fail status
- Detects CARLA crashes during example execution

### CARLA Simulator Setup

The `simulators/` directory contains symlinks to CARLA simulator binaries.

**Setup:**

```bash
# Remove placeholder symlinks
rm scripts/simulators/carla-0.9.14
rm scripts/simulators/carla-0.9.15
rm scripts/simulators/carla-0.9.16

# Create symlinks to your CARLA installations
ln -s /path/to/your/CARLA_0.9.14 scripts/simulators/carla-0.9.14
ln -s /path/to/your/CARLA_0.9.15 scripts/simulators/carla-0.9.15
ln -s /path/to/your/CARLA_0.9.16 scripts/simulators/carla-0.9.16
```

See `scripts/simulators/README.md` for detailed setup instructions.

### Writing Examples

When creating new examples:

1. **Name examples descriptively:**
   - `spawn_vehicle.rs` - Clear what it demonstrates
   - `walker_control.rs` - Shows walker movement
   - `vehicle_physics_demo.rs` - Complex physics interactions

2. **Include documentation:**
   ```rust
   //! Brief description of what this example demonstrates.
   //!
   //! This example shows how to:
   //! - Item 1
   //! - Item 2
   //!
   //! Run with: `cargo run --example example_name`
   ```

3. **Make examples robust:**
   - Handle connection failures gracefully
   - Print helpful messages about what's happening
   - Clean up resources when possible (note: actor cleanup is manual)
   - Exit with meaningful status codes (0 = success)

4. **Examples should demonstrate, not test:**
   - Focus on showing how to use the API
   - Print output so users can see what's happening
   - Avoid complex assertions (those go in unit tests)

### Unit Test Pattern

Unit tests stay in source files using `#[cfg(test)]`:

```rust
// src/rpc/walker_bone_control.rs
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bone_transform_creation() {
        let bone = BoneTransformDataIn {
            bone_name: "crl_arm__L".to_string(),
            transform: Transform::default(),
        };
        assert_eq!(bone.bone_name, "crl_arm__L");
    }
}
```

Run with: `cargo test`

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
3. Build from source with `build-prebuilt` feature
4. Generate docs without native library (docs-only feature)

Prebuilt binaries are indexed in `carla-sys/index.json5` by target triple and version.

### Prebuilt Library System

#### Index Structure

The `carla-sys/index.json5` file maps version-target combinations to download URLs and SHA256 checksums:

```json5
{
    "0.9.14-x86_64-unknown-linux-gnu": {
        "url": "https://github.com/.../libcarla_client.0.9.14-x86_64-unknown-linux-gnu.tar.zstd",
        "sha256": "94064aee24500d4c3f0fa4c4720ab20803dc0c8ae28ba65a49b4912285ec17e0"
    },
    "0.9.15-x86_64-unknown-linux-gnu": {
        "url": "https://github.com/.../libcarla_client.0.9.15-x86_64-unknown-linux-gnu.tar.zstd",
        "sha256": "..."
    },
    "0.9.16-x86_64-unknown-linux-gnu": {
        "url": "https://github.com/.../libcarla_client.0.9.16-x86_64-unknown-linux-gnu.tar.zstd",
        "sha256": "1dc1e69d824f545931ff746e4507ee62478569812dcf2cb1a0b261e20703c15b"
    }
}
```

#### Security: SHA256 Verification

All downloaded prebuilt libraries are verified using SHA256 checksums:
- The build script calculates the hash of downloaded tarballs
- Compares against expected hash from `index.json5`
- Build fails with clear error if verification fails
- Successful verification shows: `cargo:warning=SHA256 verification passed for {version}`

**Implementation:** See `download_tarball()` in `carla-sys/build.rs:275-320`

#### Build Artifact Storage

When building with different CARLA versions, artifacts are stored efficiently:

**Native libraries (C++ code):** NOT overwritten
- Downloaded tarballs: `target/debug/build/carla-sys-*/out/libcarla_client.{VERSION}-{TARGET}.tar.zstd`
- Extracted libraries: `target/debug/build/carla-sys-*/out/libcarla_client/libcarla_client.{VERSION}-{TARGET}/`
- Multiple versions coexist in the same `OUT_DIR`

**Rust compiled artifacts:** Overwritten when switching versions
- `target/debug/libcarla_sys.rlib` - recompiled for the new version (~30-40 seconds)

**Storage requirements:**
- CARLA 0.9.14: ~49M tarball + ~350M extracted = ~400M
- CARLA 0.9.15: ~65M tarball + ~650M extracted = ~715M
- CARLA 0.9.16: ~76M tarball + ~850M extracted = ~925M
- Total with all versions: ~2G in `OUT_DIR`

**Switching versions:**
```bash
# Build 0.9.14 (downloads and caches)
CARLA_VERSION=0.9.14 cargo build

# Switch to 0.9.15 (downloads and caches, 0.9.14 artifacts remain)
CARLA_VERSION=0.9.15 cargo build

# Switch to 0.9.16 (downloads and caches, previous artifacts remain)
CARLA_VERSION=0.9.16 cargo build

# Back to 0.9.14 (uses cached artifacts, only recompiles Rust code)
CARLA_VERSION=0.9.14 cargo build
```

#### Creating Prebuilt Libraries

To create new prebuilt library distributions:

1. Build from CARLA source with `build-prebuilt` feature:
```bash
export CARLA_DIR=/path/to/carla
CARLA_VERSION=0.9.16 cargo build -p carla-sys --features build-prebuilt
```

2. This generates tarball in `carla-sys/generated/`:
```
libcarla_client.{VERSION}-{TARGET}.tar.zstd
```

3. Calculate SHA256 and upload to GitHub releases:
```bash
sha256sum carla-sys/generated/*.tar.zstd
```

4. Update `carla-sys/index.json5` with URL and SHA256 hash

**Build time comparison:**
- From source with `build-prebuilt`: 15-20 minutes
- Download prebuilt: 30-40 seconds

**Supported versions:** 0.9.14, 0.9.15, 0.9.16

**See also:** `PREBUILT_COMPARISON.md` for build reproducibility verification

## Development Commands

### Build System

**IMPORTANT:** Use `make build` for development builds, or add `--profile dev-release` when using cargo directly.

The Makefile provides consistent build configuration:

```bash
# Build all crates, tests, and examples (RECOMMENDED)
make build

# Equivalent cargo command (if not using make):
cargo build --all-targets --profile dev-release
```

**Why `--profile dev-release`?**
- Uses the `dev-release` profile (optimized but with debug info)
- Optimization level 2 makes the code fast enough for debugging
- Includes debug symbols for debugging with gdb/lldb
- Faster than `dev` profile, debug-able unlike `release` profile
- **Required for examples** - the `scripts/run-examples.sh` script expects binaries in `target/dev-release/examples/`

**Profile details:**
- `dev-release` profile is defined in `Cargo.toml`
- Binaries are placed in `target/dev-release/` (not `target/debug/` or `target/release/`)
- Examples must be built with this profile to work with test automation

### Other Commands

```bash
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

# Run unit tests (use dev-release for consistency)
cargo test --profile dev-release

# Run specific example manually
cargo run --example spawn_vehicle --profile dev-release

# Run examples with automatic CARLA management
./scripts/run-examples.sh
```

**Why not use standard profiles?**
- `dev` profile: Too slow for testing, especially with large simulations
- `release` profile: No debug symbols, hard to debug issues
- `dev-release` profile: Best of both worlds for development

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
