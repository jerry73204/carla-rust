# carla-sys

This crate provides Rust FFI bindings for CARLA simulator. It links to
pre-built `libcarla_client` libraries and generates FFI bindings using
[autocxx](https://github.com/google/autocxx). Additional C++ code in
[csrc](csrc) directory replaces some classes and functions that cannot
be correctly handled by autocxx.

**Supported CARLA Versions:** 0.9.14, 0.9.16 (default: 0.9.16)

It is part of [carla](../carla/README.md) crate and normally you don't
have to include this package in your project.

## Version Selection

Use the `CARLA_VERSION` environment variable to select which CARLA version to build against:

```bash
# Use CARLA 0.9.16 (default)
cargo build

# Use CARLA 0.9.14
CARLA_VERSION=0.9.14 cargo build
```

The version selection affects:
- Which prebuilt library is downloaded (when available)
- Which vendored headers are used for binding generation
- Which boost version is linked (1.80.0 for 0.9.14, 1.84.0 for 0.9.16)

## Building Prebuilt libcarla_client Libraries

This section documents how to build prebuilt `libcarla_client` libraries for distribution. These prebuilt libraries allow users to build carla-sys without requiring the full CARLA source code.

### Prerequisites

**LLVM/clang compatibility**: Building requires LLVM versions 11-13. LLVM 14+ is not supported due to autocxx compatibility issues.

```bash
# Install compatible LLVM version (Ubuntu/Debian)
sudo apt install clang-13 libclang-13-dev

# Or use LLVM 12 or 11
sudo apt install clang-12 libclang-12-dev
```

The build script automatically detects compatible LLVM versions (preference order: 13, 12, 11). To manually override:

```bash
export LLVM_CONFIG_PATH=/usr/bin/llvm-config-13
export LIBCLANG_PATH=/usr/lib/llvm-13/lib
export LIBCLANG_STATIC_PATH=/usr/lib/llvm-13/lib
export CLANG_PATH=/usr/bin/clang-13
```

### Building from CARLA Source

#### Option 1: Automated Script (Recommended)

Use the provided build script with your CARLA directory path:

```bash
cd /path/to/carla-rust
./scripts/build_prebuilt.sh /path/to/carla

# Or use CARLA_DIR environment variable
export CARLA_DIR=/path/to/carla
./scripts/build_prebuilt.sh
```

The script will:
- Validate the CARLA installation
- Check LLVM/clang compatibility
- Build with the `build-prebuilt` feature
- Display generated artifacts and next steps

#### Option 2: Manual Build

1. **Clone and prepare CARLA repository** (version 0.9.14):

```bash
git clone https://github.com/carla-simulator/carla.git
cd carla
git checkout 0.9.14
```

2. **Set CARLA_DIR and build with the build-prebuilt feature**:

```bash
export CARLA_DIR=/path/to/carla

cd /path/to/carla-rust/carla-sys
cargo build --features build-prebuilt
```

**The build-prebuilt feature**:
- Compiles `libcarla_client` from CARLA source (requires CARLA_DIR)
- Saves compiled library as tarball in `generated/` directory
- Saves generated FFI bindings to `generated/bindings.rs`

3. **Locate the generated artifacts**:

After a successful build, the following artifacts are created in `carla-sys/generated/`:

```
carla-sys/generated/
├── libcarla_client.{VERSION}-{TARGET}.tar.zstd  # Prebuilt library tarball
└── bindings.rs                                   # Generated FFI bindings
```

Example: `libcarla_client.0.9.14-x86_64-unknown-linux-gnu.tar.zstd`

### Using Prebuilt Libraries

The prebuilt tarballs can be distributed via:

1. **GitHub Releases**: Upload the tarball to a release
2. **Update index.json5**: Add an entry mapping the target to the download URL

```json5
{
    "0.9.14-x86_64-unknown-linux-gnu": "https://github.com/user/repo/releases/download/tag/libcarla_client.0.9.14-x86_64-unknown-linux-gnu.tar.zstd"
}
```

The build script automatically downloads and uses prebuilt libraries from the index when the `build-prebuilt` feature is not enabled.

### Building for Multiple Targets

To create prebuilt libraries for different platforms, repeat the build process on each target platform (e.g., Linux x86_64, Windows, macOS).

## Use Custom CARLA Source Code

To use a custom CARLA repository without creating prebuilt libraries:

```bash
# Set CARLA_DIR to your custom CARLA installation
export CARLA_DIR=$HOME/my-carla

# Build without the build-prebuilt feature
cargo build
```

This uses the existing prebuilt library from your CARLA installation without creating a distributable tarball.

## License

It is distributed under MIT license. Please see
[LICENSE.txt](../LICENSE.txt) file for full license text.
