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
