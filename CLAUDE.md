# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

This is a Rust client library for the CARLA simulator. The project uses FFI bindings to interface with the CARLA C++ client library and provides a safe, idiomatic Rust API.

**Supported CARLA Versions:** 0.9.14, 0.9.16 (default: 0.9.16)

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

# Build for CARLA 0.9.16 explicitly
CARLA_VERSION=0.9.16 cargo build
```

When `build-prebuilt` feature is enabled, the version determines:
- Which boost library version to link against (1.80.0 for 0.9.14, 1.84.0 for 0.9.16)
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
- CARLA 0.9.16: ~76M tarball + ~850M extracted = ~925M
- Total with both versions: ~1.2G in `OUT_DIR`

**Switching versions:**
```bash
# Build 0.9.14 (downloads and caches)
CARLA_VERSION=0.9.14 cargo build

# Switch to 0.9.16 (downloads and caches, 0.9.14 artifacts remain)
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
- From source with `build-prebuilt`: 15-20 minutes (CARLA 0.9.16)
- Download prebuilt: 30-40 seconds

**See also:** `PREBUILT_COMPARISON.md` for build reproducibility verification

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
