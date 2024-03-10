# carla-sys

This crate provides Rust FFI bindings for CARLA simulator. It links to
pre-built `libcarla_client` libraries and generates FFI bindings using
[autocxx](https://github.com/google/autocxx). Additional C++ code in
[csrc](csrc) directory replaces some classes and functions that cannot
be correctly handled by autocxx.

It is part of [carla](../carla/README.md) crate and normally you don't
have to include this package in your project.

## Use Custom CARLA Source Code

In the case that you want to use a custom CARLA repository. Set the
`CARLA_DIR` environment variable to point to your CARLA repo and build
carla-sys with additional features.

```bash
// Prepare and compile CARLA repo.
git clone https://github.com/carla-simulator/carla.git $HOME/my-carla
```

```bash
// Set CARLA_DIR and build your Rust project.
export CARLA_DIR=$HOME/my-carla

git clone https://github.com/jerry73204/carla-rust.git
cd carla-rust/carla-sys
cargo build \
    --features build-lib \
    --features save-lib \
    --features save-bindgen
```

## License

It is distributed under MIT license. Please see
[LICENSE.txt](../LICENSE.txt) file for full license text.
