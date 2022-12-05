# Carla Simulator Client Library in Rust

Rust client library for Carla simulator. It is compatible with
simulator version 0.9.13.

## Documentation

To get started, it's recommended to read the API documentation and
learn from the simple example [here](carla/examples/spawn.rs).

- [API documentation](https://docs.rs/carla)
- [crates.io](https://crates.io/crates/carla)
- [Examples](carla/examples)

## Usage

Add this crate to `Cargo.toml` and get everything. You may experience
long waiting time in the first compilation to compile native
libraries. The binaries are cached and it can save build time in later
builds.

If you prefer to manually build Carla C++ client library. Please refer
to the [this section](use-pre-built-carla-client-library).

## Use Pre-built Carla Client Library

In order to allow the build script to use pre-built C++ library, the
`CARLA_DIR` environment variable must be set.

Currently, the Rust library requires a patched Carla source code.

```bash
wget https://github.com/jerry73204/carla/archive/6b98a4a41128929044e814bc051c043003e0bbfd.tar.gz
tar -xvf 6b98a4a41128929044e814bc051c043003e0bbfd.tar.gz
cd carla-6b98a4a41128929044e814bc051c043003e0bbfd
```

Build the C++ client library.

```bash
make LibCarla.client.release
```

Set` CARLA_DIR` to the Carla source directory. Then, build the Rust
client library.

```bash
export CARLA_DIR=/path/to/carla/dir
cargo build
```

## License

It is an open source project and is distributed with MIT
license. Please check the [license file](LICENSE.txt).
