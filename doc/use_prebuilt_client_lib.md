# Use Pre-built Carla Client Library

In order to allow the build script to use pre-built C++ library, the
`CARLA_DIR` environment variable must be set to locate the Carla
source repository.

Currently, the Rust library requires a patched Carla source code.

```bash
git clone https://github.com/carla-simulator/carla.git
cd carla
git checkout 0.9.14
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
