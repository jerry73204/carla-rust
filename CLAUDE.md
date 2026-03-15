# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Temporary Files

**IMPORTANT:** When creating temporary files for testing or experimentation, always use the project's `tmp/` directory (e.g., `/home/aeon/repos/carla-rust/tmp/test_file.rs`). Do NOT use system `/tmp`.

## Overview

Rust client library for the CARLA simulator. Uses FFI bindings (via [autocxx](https://github.com/google/autocxx)) to interface with the CARLA C++ client library and provides safe, idiomatic Rust wrappers.

**Supported CARLA Versions:** 0.9.14, 0.9.15, 0.9.16, 0.10.0 (default: 0.10.0)

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

**CARLA version selection:** `CARLA_VERSION=0.9.14 cargo build`

Each version uses a separate target directory (`target/carla-${VERSION}`).

**Environment:** Requires `libclang-dev` for autocxx.

## Testing Rules

- **DO NOT** create tests in `carla/tests/`. Use `carla/examples/` instead — examples serve as both documentation and integration tests.
- **Unit tests** (`#[cfg(test)]`) stay in source files for pure logic (no CARLA simulator needed).
- Run examples: `./scripts/run-examples.sh [example_names...]`
- See `docs/reference/testing.md` and `scripts/simulators/README.md` for setup details.

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

## Documentation

- `docs/roadmap/` — Development roadmap (numbered 1-5)
- `docs/reference/` — API coverage, version differences, testing strategy
- `docs/archive/` — Completed/superseded docs (geom migration, old examples)
