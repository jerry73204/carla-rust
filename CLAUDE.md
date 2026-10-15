# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Temporary Files

**IMPORTANT:** When creating temporary files for testing or experimentation, always use the project's `tmp/` directory (e.g., `/home/aeon/repos/carla-rust/tmp/test_file.rs`). Do NOT use system `/tmp`.

## Overview

Rust client library for the CARLA simulator. Uses FFI bindings (via [autocxx](https://github.com/google/autocxx)) to interface with the CARLA C++ client library and provides safe, idiomatic Rust wrappers.

**Supported CARLA Versions:** 0.9.14, 0.9.15, 0.9.16, 0.10.0 (no default: a consumer enables exactly one `carla-XXXX` feature, or sets `CARLA_VERSION`)

## Crate Architecture

Cargo workspace with three crates (`carla` -> `carla-sys` -> `carla-src`):

- **carla**: High-level Rust API (`src/client/`, `src/geom/`, `src/rpc/`, `src/sensor/`, `src/traffic_manager/`, `src/road/`, `src/prelude`)
- **carla-sys**: FFI bindings (`src/bindings.rs`, `csrc/` for C++ wrappers, `build.rs` for prebuilt binary handling)
- **carla-src**: Build utility for compiling `libcarla_client` from source

## Build Rules

**Always use `just` or `--profile dev-release`:**

```bash
just build                                          # Recommended (all CARLA versions)
cargo build --all-targets --profile dev-release     # If using cargo directly
```

The `dev-release` profile (defined in `Cargo.toml`) is required for example automation and adequate simulation performance. Run `just --list` for all commands.

**CARLA version selection:** `CARLA_VERSION=0.9.14 cargo build` (overrides the `carla-0914`/`carla-0915`/`carla-0916`/`carla-0100` feature; with neither set the build fails)

Each version uses a separate target directory (`target/carla-${VERSION}`).

**Environment:** Requires `libclang-dev` for autocxx.

## Testing Rules

There are two categories of tests:

- **Unit tests** (`#[cfg(test)]`): inline in source files, test pure logic, no CARLA simulator needed. Run with `just test-unit`.
- **Integration tests** (`carla/tests/*.rs`): test against a live CARLA simulator. Each file is a scenario (e.g. `connect.rs`, `vehicle_lifecycle.rs`). Run with `just test-integration` (starts CARLA automatically) or `just test-integration-external` (CARLA already running).

When adding a new integration test scenario, create `carla/tests/my_scenario.rs` with `mod common;` at the top. No registration needed — nextest auto-discovers all `#[test]` functions in `carla/tests/*.rs`. Prefix tests with `tNN_` for deterministic ordering.

Examples (`carla/examples/`) are runnable documentation, not integration tests. Run with `cargo run --profile dev-release --example <name>`.

See `docs/reference/testing.md` for full infrastructure details.

## Linting

```bash
just check         # Full check: Rust (fmt + clippy) + C++ (format + tidy)
just check-rust    # fmt + clippy for all CARLA versions
just format        # Auto-format Rust + C++
```

Common clippy: use `!collection.is_empty()` not `collection.len() > 0`.

## FFI Patterns

### Adding New Bindings

1. Add C++ header includes to `carla-sys/src/bindings.rs`
2. Add `generate!()` declarations in autocxx's block
3. Add safe Rust wrappers in `carla/src/` modules

When autocxx can't handle a type, create a C++ wrapper in `carla-sys/csrc/carla_rust/`. Debug missing methods with `cargo doc -p carla-sys --no-deps`.

### FFI Exception Safety

All C++ methods that make RPC calls (can throw) are wrapped for exception safety:

**C++ side:** Methods take `FfiError& error` as their last parameter and wrap the body with `ffi_call(error, default_val, [&]() { ... })` or `ffi_call_void(error, [&]() { ... })`. Templates are in `carla-sys/csrc/carla_rust/client/result.hpp`.

**Rust side:** Methods return `crate::Result<T>` and use the `with_ffi_error()` helper from `carla/src/error/ffi.rs`:

```rust
pub fn weather(&self) -> crate::Result<WeatherParameters> {
    with_ffi_error("weather", |e| self.inner.GetWeather(e))
}
```

**Noexcept methods** (cached data, local pointer casts) do NOT need wrapping — they return values directly without `Result`. Examples: `ActorBase::id()`, `Waypoint::transform()`, `Vehicle::control()`.

When adding a new C++ method:
- If it makes an RPC call or can throw: add `FfiError& error` as last param, wrap with `ffi_call`/`ffi_call_void`, return `Result<T>` in Rust
- If it only accesses cached/local data: mark `noexcept` in C++, return bare `T` in Rust

## Error Handling

All public API methods that communicate with the CARLA server return `crate::Result<T>` (alias for `std::result::Result<T, CarlaError>`).

- `CarlaError` hierarchy: `Connection`, `Resource`, `Operation`, `Validation`, `Map`, `Sensor`, `Internal`, `Io`
- Helper methods: `is_timeout()`, `is_not_found()`, `is_retriable()`, etc.
- `Client::connect()` returns `Result` (no `Default` impl)
- See `carla/src/error.rs` for full type definitions

## Geometry Types

Geometry types (`Location`, `Rotation`, `Vector3D`, `Vector2D`, `GeoLocation`, `Transform`) are native Rust `#[repr(C)]` types with direct field access and verified memory layout compatibility with C++.

- Use `as_ffi()`, `from_ffi()`, `into_ffi()` for FFI conversion
- Old nalgebra patterns (`.to_na()`, `translation.vector.x`) no longer exist
- Use `transform.location.x`, `map.waypoint_at(&location)`, `velocity.length()` etc.
- See `docs/archive/geom.md` for migration details

## Prebuilt Libraries

Prebuilt `libcarla_client` binaries are indexed in `carla-sys/index.json5` (URL + SHA256 per version-target). The build script downloads, verifies, and caches them. See `carla-sys/build.rs` for details.

To create/publish prebuilt packages: `just build-prebuilt /path/to/carla/source 0.9.16`

### Where the CARLA patches live

The prebuilts are not stock CARLA. Each is built from a branch of the fork
[jerry73204/carla](https://github.com/jerry73204/carla), tagged `carla-rust/<version>-<n>`; the
GitHub release under that tag holds the tarball `index.json5` points at. To carry a new LibCarla
fix: branch from the newest `carla-rust/<version>-<n>` tag, commit (cherry-pick upstream with
`-x`), build, tag `-<n+1>`, upload the tarball to that release, then bump `index.json5` (URL +
SHA256) in one commit. Never point `index.json5` at a release that does not exist yet.

0.9.16 stack on tag `0.9.16`: `-2` (cmake 3.5, `build_libcarla.sh`, libpng link), `-3` (+
worker-thread exception containment, `Setup.sh` system-clang fallback). Pending as `-4`: +
upstream `e7e076f` "Removed TM mutex" (a synchronous TM runs `Step()` on the ticking thread with
no handshake; 0.9.16's handshake can lose a wakeup and hang `World::tick` forever) and
`last_frame` initialised; not yet tagged or released.

### Building a 0.9.16 prebuilt without UnrealEngine

`build-prebuilt` clones UnrealEngine (~8 GB) unless `Unreal/UnrealEngine/Engine` exists; the
fork's `Setup.sh` (from `-3`) falls back to the system clang when UE's toolchain is absent, so a
client-only build needs neither:

```bash
GIT_LFS_SKIP_SMUDGE=1 git clone -b <branch> https://github.com/jerry73204/carla.git carla
mkdir -p carla/Unreal/UnrealEngine/{Engine,Build}
touch carla/Unreal/UnrealEngine/Build/OneTimeSetupPerformed
just build-prebuilt "$PWD/carla" 0.9.16
```

It deletes `Build/`, runs `git checkout -B ue4/0.9.16` in the tree (the version string comes from
the branch name) and rebuilds boost, rpclib and recast from source (~20 min). The result is built
with the system clang where `-3` used UE's clang 10.0.1; both link libstdc++.
`GIT_LFS_SKIP_SMUDGE=1` matters: the client build needs none of the LFS content.

To try a built tree without publishing, set `CARLA_DIR=<tree>` on a consumer build (without
`build-prebuilt`): carla-sys installs from the tree into its own `generated/` directory instead
of downloading.

## Documentation

- `docs/roadmap/` — Development roadmap (numbered 1-5)
- `docs/reference/` — API coverage, version differences, testing strategy
- `docs/archive/` — Completed/superseded docs (geom migration, old examples)
