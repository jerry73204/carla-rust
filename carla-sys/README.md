# carla-sys

This crate provides Rust FFI bindings for Carla simulator. It
leverages [autocxx](https://github.com/google/autocxx) to generate
most symbols in libcarla C++ library, and replaces some classes and
functions in `csrc` that are correctly handled by autocxx.

The C++ source code is automatically provided by
[carla-src](../carla-src/README.md) crate. If you prefer to use custom
source directory, set the directory path to `CARLA_DIR` environment
variable.
