# Carla Simulator Client Library in Rust

[![Crates.io](https://img.shields.io/crates/v/carla.svg)](https://crates.io/crates/carla)
[![Documentation](https://docs.rs/carla/badge.svg)](https://docs.rs/carla)
[![License](https://img.shields.io/crates/l/carla.svg)](LICENSE.txt)

Rust client library for CARLA simulator.

**Supported Versions:** CARLA 0.9.14, 0.9.15, 0.9.16, 0.10.0, 1.0

## Usage

Add `carla` to your `Cargo.toml` and name the CARLA version you target with a feature:

```toml
[dependencies]
carla = { version = "0.13", features = ["carla-0916"] }
```

The CARLA client API differs between versions, so the version must be chosen at build
time. Enable **exactly one** of these features:

| Feature      | CARLA  |
|--------------|--------|
| `carla-0914` | 0.9.14 |
| `carla-0915` | 0.9.15 |
| `carla-0916` | 0.9.16 |
| `carla-0100` | 0.10.0 |
| `carla-10`   | 1.0    |

The features are mutually exclusive: enabling two (for instance two crates in one
dependency graph asking for different versions) is a build error that names both. Enabling
none is also a build error -- there is no silent default.

The `CARLA_VERSION` environment variable, when set, overrides the feature (useful for
building one checkout against several servers):

```bash
cargo build                         # uses the feature in Cargo.toml
CARLA_VERSION=0.9.15 cargo build    # overrides it
```

The selected version is available at run time as `carla::CARLA_VERSION`; compare it with
`Client::server_version()` to refuse a server of another version. The `docs-only` feature
(used by docs.rs) builds without a version selected and documents the 0.10.0 API.

### CARLA 1.0

No prebuilt `libcarla_client` for 1.0 is published yet, so `carla-10` builds against a
CARLA 1.0 source tree whose client library you have built. Unreal Engine is not needed: the
client builds with the system compiler and libstdc++, and needs CMake 3.27.2 or newer.

```bash
GIT_LFS_SKIP_SMUDGE=1 git clone --depth 1 -b 1.0 https://github.com/carla-simulator/carla.git
cd carla
cmake -B Build/libstdcxx-release -G Ninja -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_CARLA_CLIENT=ON -DBUILD_CARLA_SERVER=OFF -DBUILD_CARLA_UNREAL=OFF \
    -DBUILD_PYTHON_API=OFF -DBUILD_EXAMPLES=OFF -DBUILD_LIBCARLA_TESTS=OFF -DENABLE_ROS2=OFF
cmake --build Build/libstdcxx-release --target carla-client
cd - && CARLA_DIR=/path/to/carla cargo build --features carla-10
```

CARLA 1.0's `CMake/Dependencies.cmake` always asks Boost for its `python` library, which
Boost's CMake refuses while `BUILD_PYTHON_API` is off. Until that is fixed upstream, remove
`python` from `BOOST_INCLUDED_PROJECTS` when building the client alone.

What 1.0 changes in this crate:

- `Vehicle::telemetry_data()` is available again, with fewer fields than 0.9.16's (no
  `drag`; per wheel only `lat_slip`, `long_slip` and `omega`).
- `EpisodeSettings` has `deterministic_collisions` and `adjust_signs_height_to_ground`.
- `Map::transform_to_geolocation()` goes through the map's projection; as on 0.10, +y lowers
  the latitude (0.9.x raises it).
- The traffic manager has no dormant-vehicle respawn (`set_respawn_dormant_vehicles`,
  `set_boundaries_respawn_dormant_vehicles`, `set_max_boundaries`) and no
  `set_keep_right_percentage`.
- `destroy()` always asks the server, and `is_alive()` stays `true` from the spawn until a
  later snapshot says otherwise.
- `CityObjectLabel::Rock` is new.

### Quick Start Example

```rust
use carla::client::Client;

fn main() -> anyhow::Result<()> {
    // Connect to CARLA server
    let client = Client::new("localhost", 2000, None);
    let mut world = client.world();

    // Get a vehicle blueprint
    let blueprint_library = world.blueprint_library_filtered("vehicle.*")?;
    let vehicle_bp = blueprint_library[0].clone();

    // Spawn a vehicle
    let spawn_points = world.map()?.recommended_spawn_points()?;
    let vehicle = world.spawn_actor(&vehicle_bp, &spawn_points[0])?;

    println!("Spawned vehicle: {:?}", vehicle.id());

    Ok(())
}
```

For more examples, see the [examples directory](carla/examples).

## Documentation

- [API documentation](https://docs.rs/carla) - Complete API reference
- [Examples](carla/examples) - Code examples demonstrating various features
- [crates.io](https://crates.io/crates/carla) - Published crate


## Development

This section is for developers working on the carla-rust codebase itself.

### Building from Source

Clone the repository and use `just` for development tasks:

```bash
# Build all crates (uses dev-release profile)
just build

# Run tests
just test

# Format code
just format

# Run linters
just lint

# Build documentation
just doc
```

See the [justfile](justfile) for all available commands.

### Project Structure

This is a Cargo workspace with three crates:

- **carla** - High-level Rust client library (main crate)
- **carla-sys** - FFI bindings to CARLA C++ library
- **carla-src** - Build utilities for CARLA source code

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

### Building Prebuilt Libraries (Maintainers)

If you need to build prebuilt `libcarla_client` libraries for distribution:

```bash
# Build prebuilt package for a specific CARLA version
just build-prebuilt /path/to/carla/source 0.9.16

# Regenerate bindings for all versions (downloads prebuilt packages)
just regen-bindings
```

For detailed documentation, see [carla-sys/README.md](carla-sys/README.md#building-prebuilt-libcarla_client-libraries).

## License

It is an open source project and is distributed with MIT
license. Please check the [license file](LICENSE.txt).
