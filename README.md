# Carla Simulator Client Library in Rust

Rust client library for Carla simulator. It is compatible with
simulator version 0.10.0.

It is recommended to fix the clang version to 12 on newer systems such
as Ubuntu 22.04. See the [Troubleshooting](#troubleshooting) section
to find instructions.

## Documentation

To get started, it's recommended to read the API documentation.

- [API documentation](https://docs.rs/carla)
- [crates.io](https://crates.io/crates/carla)

### Project Documentation

Additional project documentation can be found in the `doc/` directory:

- [`doc/PROGRESS.md`](doc/PROGRESS.md) - Implementation progress and roadmap
- [`doc/ACTOR.md`](doc/ACTOR.md) - Actor system documentation and testing notes
- [`doc/EXAMPLES.md`](doc/EXAMPLES.md) - Guide to the example programs
- [`doc/SENSOR_CALLBACK_SUMMARY.md`](doc/SENSOR_CALLBACK_SUMMARY.md) - Sensor callback implementation details
- [`doc/API.md`](doc/API.md) - Plan for Python API to Rust type conversion tool


## Architecture

This project consists of multiple crates:

- **carla-sys**: Low-level FFI bindings to CARLA's C++ client library
  - Handles C++ compilation and linking
  - Generates Rust types from CARLA's Python API YAML documentation
  - Provides both FFI and stub implementations for docs.rs compatibility
- **carla**: High-level, safe Rust API
  - Provides idiomatic Rust interface
  - Imports generated types from carla-sys
- **carla-codegen**: Code generation tool
  - Converts CARLA Python API YAML to Rust types
  - Used by carla-sys during development
- **carla-src**: CARLA source management
  - Downloads and manages CARLA source code

### Code Generation

As of version 0.13.0, code generation has been moved from the `carla` crate to `carla-sys` for better architecture. Generated types are pre-generated and shipped with the source code, so normal builds don't require CARLA_ROOT or code generation.

## Usage

### Prerequisites

You need a CARLA source directory with the necessary build configuration. We recommend using jerry73204's fork which includes the required CMake setup for building only the LibCarla client library:

```bash
# Clone the CARLA repository with LibCarla client installation support
git clone -b 0.10.0-libcarla-client-install https://github.com/jerry73204/carla.git
cd carla

# Set CARLA_ROOT to this directory
export CARLA_ROOT=$(pwd)
```

Alternatively, if you already have the repository cloned:

```bash
export CARLA_ROOT=/path/to/carla-source
```

### Building

Add `carla` crate to `Cargo.toml`:

```toml
[dependencies]
carla = "0.13"
```

Then build your project:

```bash
cargo build
```

The first build may take longer as it generates Rust bindings and compiles the C++ library.


## Troubleshooting

### Build failed with clang >= 14 (on Ubuntu 22.04 or newer)

Currently `carla-sys` is unable to build with newer clang version >= 14.
If you are Ubuntu users, it's recommended to configure
clang-sys to use clang 12.

```sh
sudo apt install clang-12 libclang-12-dev
export LLVM_CONFIG_PATH=/usr/bin/llvm-config-12
export LIBCLANG_PATH=/usr/lib/llvm-12/lib
export LIBCLANG_STATIC_PATH=/usr/lib/llvm-12/lib
export CLANG_PATH=/usr/bin/clang-12
```

## License

It is an open source project and is distributed with MIT
license. Please check the [license file](LICENSE.txt).
