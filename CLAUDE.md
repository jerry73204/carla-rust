# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Temporary Files

**IMPORTANT:** When creating temporary files for testing or experimentation, always use the project's `tmp/` directory (e.g., `/home/aeon/repos/carla-rust/tmp/test_file.rs`). Do NOT use system `/tmp`.

## Overview

Rust client library for the CARLA simulator. Uses FFI bindings (via [autocxx](https://github.com/google/autocxx)) to interface with the CARLA C++ client library and provides safe, idiomatic Rust wrappers.

**Supported CARLA Versions:** 0.9.14, 0.9.15, 0.9.16 (default: 0.9.16)

## Crate Architecture

Cargo workspace with three crates (`carla` -> `carla-sys` -> `carla-src`):

- **carla**: High-level Rust API (`src/client/`, `src/geom/`, `src/rpc/`, `src/sensor/`, `src/traffic_manager/`, `src/road/`, `src/prelude`)
- **carla-sys**: FFI bindings (`src/ffi.rs`, `csrc/` for C++ wrappers, `build.rs` for prebuilt binary handling)
- **carla-src**: Build utility for compiling `libcarla_client` from source

## Build Rules

**Always use `just` or `--profile dev-release`:**

```bash
just build                                          # Recommended
cargo build --all-targets --profile dev-release     # If using cargo directly
```

The `dev-release` profile (defined in `Cargo.toml`) is required for example automation and adequate simulation performance. Run `just --list` for all commands.

**CARLA version selection:** `CARLA_VERSION=0.9.14 cargo build`

**Environment:** Requires `libclang-dev` for autocxx.

## Testing Rules

- **DO NOT** create tests in `carla/tests/`. Use `carla/examples/` instead — examples serve as both documentation and integration tests.
- **Unit tests** (`#[cfg(test)]`) stay in source files for pure logic (no CARLA simulator needed).
- Run examples: `./scripts/run-examples.sh [example_names...]`
- See `docs/reference/testing.md` and `scripts/simulators/README.md` for setup details.

## Linting

```bash
just lint-rust    # fmt + clippy
```

Common clippy: use `!collection.is_empty()` not `collection.len() > 0`.

## FFI Patterns

When adding new bindings:
1. Add C++ header includes to `carla-sys/src/ffi.rs`
2. Add `generate!()` declarations in autocxx's block
3. Add safe Rust wrappers in `carla/src/` modules

When autocxx can't handle a type, create a C++ wrapper in `carla-sys/csrc/carla_rust/`. Debug missing methods with `cargo doc -p carla-sys --no-deps`.

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
