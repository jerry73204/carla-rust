# Carla Simulator Client Library in Rust

Rust client library for Carla simulator. It is compatible with
simulator version 0.9.14.

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

### Build failed with clang >= 14

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
