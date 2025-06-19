# carla-src

CARLA source code management utilities for the carla-rust project.

## Overview

This crate handles validating and using CARLA simulator source code needed to build the Rust bindings. The `CARLA_ROOT` environment variable must be set to point to a valid CARLA source directory.

## Usage

### Basic Usage

```rust
use carla_src::CarlaSource;

// Requires CARLA_ROOT to be set
let source = CarlaSource::new()?;

// Get paths for compilation
let include_dirs = source.include_dirs();
let lib_dirs = source.lib_dirs();
```

### Setting CARLA_ROOT

```bash
export CARLA_ROOT=/path/to/carla/source
cargo build
```

### Direct Path Usage

```rust
use carla_src::CarlaSource;

// Use a specific directory
let source = CarlaSource::from_local("/path/to/carla/source")?;
```

## Source Requirements

The CARLA source directory must contain:

- `LibCarla/source/carla/` - Core CARLA client library source
- `LibCarla/source/third-party/` - Third-party dependencies
- `CMakeLists.txt` - Build configuration (in root directory)
- `CHANGELOG.md` - Version information

## Environment Variables

- `CARLA_ROOT`: Path to local CARLA source directory (required)

## Recommended CARLA Source

For best compatibility, use jerry73204's CARLA fork:

```bash
git clone -b 0.10.0-libcarla-client-install https://github.com/jerry73204/carla.git
export CARLA_ROOT=/path/to/carla
```

This fork includes the necessary CMake configuration for building the LibCarla client library independently from the full CARLA simulator.

## Platform Support

Currently only Linux is supported.

## License

It is distributed under MIT license. Please see [LICENSE.txt](../LICENSE.txt) file for full license text.