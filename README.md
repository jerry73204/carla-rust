# Carla Simulator Client Library in Rust

Rust client library for Carla simulator.

**Supported Versions:** CARLA 0.9.14, 0.9.16 (default: 0.9.16)

To select a specific version, set the `CARLA_VERSION` environment variable:
```bash
# Use CARLA 0.9.16 (default)
cargo build

# Use CARLA 0.9.14
CARLA_VERSION=0.9.14 cargo build
```

It is recommended to fix the clang version to 12 on newer systems such
as Ubuntu 22.04. See the [Troubleshooting](#troubleshooting) section
to find instructions.

## Documentation

To get started, it's recommended to read the API documentation and
learn from the simple example [here](carla/examples/spawn.rs).

- [API documentation](https://docs.rs/carla)
- [crates.io](https://crates.io/crates/carla)
- [Examples](carla/examples)


## Usage

Add `carla` crate to `Cargo.toml` and get everything. You may wait for
longer time to generate Rust bindings in the first build.

If you prefer to manually build Carla C++ client library. Please read
 [this guide](doc/use_prebuilt_client_lib.md) to learn instructions.


## Troubleshooting

### Build failed with clang >= 14 (on Ubuntu 22.04 or newer)

Currently `carla-sys` requires LLVM/clang versions 11-13 due to autocxx compatibility.
The build system automatically detects and configures a compatible version if available.

**Supported LLVM versions:** 11, 12, 13
**Unsupported LLVM versions:** 14 and newer

**To fix:**

1. Install a compatible LLVM version:
   ```sh
   sudo apt install clang-12 libclang-12-dev
   ```

2. The build script will automatically detect and configure it. If auto-detection
   doesn't work, manually set these environment variables:
   ```sh
   export LLVM_CONFIG_PATH=/usr/bin/llvm-config-12
   export LIBCLANG_PATH=/usr/lib/llvm-12/lib
   export LIBCLANG_STATIC_PATH=/usr/lib/llvm-12/lib
   export CLANG_PATH=/usr/bin/clang-12
   ```

**Note:** On systems with multiple LLVM versions, autocxx may default to a newer
incompatible version. The build script automatically overrides this if it detects
LLVM 11-13.

## For Maintainers: Building Prebuilt Libraries

If you need to build prebuilt `libcarla_client` libraries for distribution:

```bash
./scripts/build_prebuilt.sh /path/to/carla
```

This automated script builds and packages the library for distribution. For detailed documentation, see [carla-sys/README.md](carla-sys/README.md#building-prebuilt-libcarla_client-libraries).

## License

It is an open source project and is distributed with MIT
license. Please check the [license file](LICENSE.txt).
