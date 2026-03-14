# Error Handling & FFI Exception Safety

**Priority:** HIGH
**Status:** In Progress (infrastructure done, methods not yet wrapped)
**Estimated Effort:** 3-4 weeks

## Overview

This document covers two related initiatives:
1. **Structured error types** — Replace `anyhow::Error` with a `CarlaError` hierarchy so users can pattern-match on specific failure modes
2. **FFI exception safety** — Wrap every C++ method in try-catch so no exception can abort the Rust process

## 5.1 Error Type Design

### Current State

- `carla` crate uses `anyhow::Result<T>` everywhere — no pattern matching possible
- C++ exceptions (`TimeoutException`, `std::runtime_error`, etc.) become opaque strings
- Users cannot distinguish connection errors from validation errors

### Proposed Hierarchy

```
CarlaError (top-level enum)
├── Connection (timeout, disconnected, invalid endpoint)
├── Resource (not found, destroyed, unavailable)
├── Operation (spawn failed, physics disabled, simulation error)
├── Validation (invalid blueprint/attribute, out of bounds)
├── Map (load failed, invalid waypoint, topology error)
├── Sensor (data unavailable, listen failed)
├── Internal (FFI error, unexpected state)
└── Io (file operations)
```

Uses `thiserror` for derive macros. Full type definitions are in `carla/src/error.rs`.

### Work Items

- [x] Create `CarlaError` hierarchy in `carla/src/error.rs`
- [x] Add `thiserror` dependency
- [x] Create `FfiErrorKind` enum matching C++ `ErrorKind`
- [x] Implement `parse_ffi_error()` converter
- [x] Add `CarlaError::is_*()` helper methods (is_timeout, is_not_found, is_retriable, is_validation_error, is_operation_error, is_map_error, is_sensor_error, is_internal_error)
- [x] Add `ResultExt` context extension trait
- [x] Migrate high-priority APIs from `anyhow::Result` to `carla::Result`
- [x] Update examples to demonstrate error handling patterns

## 5.2 FFI Exception Safety

### Problem

Every C++ method called through the FFI boundary can throw exceptions. Currently only 2 out of ~80+ exposed methods are wrapped in try-catch. When the CARLA server disconnects or throws, the C++ exception propagates through autocxx and **aborts** the Rust process.

### Goal

Make the entire C++ FFI layer exception-free. Every method exposed to Rust either:
1. Catches all C++ exceptions and reports them via `FfiError&` out-parameter, or
2. Is genuinely `noexcept` (e.g., simple getters on cached data)

### Current Infrastructure

**C++ side** (`carla-sys/csrc/carla_rust/error.hpp`):
- `ErrorKind` enum (Success, Timeout, NotFound, InvalidArgument, RuntimeError, OutOfRange, Unknown)
- `classify_exception()` — pattern-matches exception messages
- `CARLA_TRY` / `CARLA_CATCH` macros (to be replaced by templates)

**C++ error container** (`carla-sys/csrc/carla_rust/client/result.hpp`):
- `FfiError` class with `kind()`, `message()`, `has_error()`, `set()`, `clear()`
- `ffi_try_connect()` / `ffi_try_get_world()` (to be replaced)

**Rust side** (`carla/src/error.rs`, `carla/src/error/ffi.rs`):
- Full `CarlaError` hierarchy
- `parse_ffi_error()` converter
- `Client::try_connect()` and `Client::try_world()` use `FfiError` pattern

### Design: C++ Templates

Replace macros with type-safe templates:

```cpp
template <typename F, typename R = std::invoke_result_t<F>>
R ffi_call(FfiError& error, R default_val, F&& fn);

template <typename F>
void ffi_call_void(FfiError& error, F&& fn);
```

### Design: Rust Helper

```rust
fn check_ffi_error(error: &FfiError, operation: &str) -> Result<()>;
```

### Noexcept Methods (No Wrapping Needed)

- `FfiActor::GetId()`, `GetTypeId()`, `GetDisplayId()` — cached identifiers
- `FfiActor::IsAlive()`, `IsDormant()`, `IsActive()` — cached state
- `FfiWorld::GetId()` — cached episode ID
- `FfiWorldSnapshot` accessors — in-memory snapshot data
- Type conversion methods (`to_vehicle()`, `to_sensor()`) — local pointer cast

## 5.3 Template Infrastructure

- [ ] Add `ffi_call<F, R>` template to `error.hpp`
- [ ] Add `ffi_call_void<F>` template to `error.hpp`
- [ ] Add `check_ffi_error()` helper to `carla/src/error/ffi.rs`
- [ ] Verify templates compile with all supported CARLA versions

## 5.4 World + Client

Highest priority — these methods are called every tick or during connection setup.

**FfiWorld** (~30 methods):
- [ ] `GetSnapshot`, `WaitForTick`, `Tick` — main loop
- [ ] `GetMap`, `GetBlueprintLibrary`, `GetActors` — startup/queries
- [ ] `ApplySettings`, `GetSettings`, `GetWeather`, `SetWeather` — config
- [ ] `GetSpectator`, `TrySpawnActor` — spawning
- [ ] All remaining RPC-calling methods

**FfiClient** (~20 methods):
- [ ] Constructor with `FfiError&` (replaces `ffi_try_connect`)
- [ ] `GetWorld(FfiError&)` (replaces `ffi_try_get_world`)
- [ ] `LoadWorld`, `ReloadWorld`, `GenerateOpenDriveWorld` — map loading
- [ ] Recording/replay methods
- [ ] Batch operation methods

**Rust API cleanup:**
- [ ] Remove `Client::try_connect()` — `connect()` returns `Result` directly
- [ ] Remove `Client::try_world()` — `world()` returns `Result` directly
- [ ] Delete `ffi_try_connect()` / `ffi_try_get_world()` free functions

## 5.5 Actor + Vehicle

**FfiActor** (~25 RPC methods):
- [ ] Position/velocity queries: `GetLocation`, `GetTransform`, `GetVelocity`, etc.
- [ ] Mutations: `SetLocation`, `SetTransform`, `Destroy`, etc.
- [ ] Physics: `AddImpulse`, `AddForce`, `SetSimulatePhysics`, etc.

**FfiVehicle** (~15 methods):
- [ ] `ApplyControl` — every tick
- [ ] `SetAutopilot`, physics/steering/door/light methods

## 5.6 Sensor

- [ ] `FfiSensor::Listen` and `Stop` with `FfiError&`
- [ ] Rust `Sensor::listen()` and `Sensor::stop()` return `Result<()>`

## 5.7 Remaining Types

- [ ] `FfiMap` — waypoint generation, junction queries
- [ ] `FfiWaypoint` — road network queries
- [ ] `FfiTrafficLight` — state/timing mutations
- [ ] `FfiTrafficManager` — all methods
- [ ] `FfiWalker` / `FfiWalkerAIController` — movement/navigation
- [ ] `FfiLightManager` — state mutations

## 5.8 Final Cleanup

- [ ] Audit: grep for any remaining C++ method without `FfiError&` that calls RPC
- [ ] Remove `CARLA_TRY` / `CARLA_CATCH` macros
- [ ] Update all examples to use `Result` returns
- [ ] Add reconnection example demonstrating `CarlaError::Connection` handling

## Done Criteria

- [ ] No C++ exception can abort the Rust process under any server state
- [ ] All public Rust API methods return `Result<T>`
- [ ] `cargo test` and `cargo doc` pass
- [ ] `CARLA_TRY`/`CARLA_CATCH` macros removed
- [ ] `ffi_try_connect`/`ffi_try_get_world` free functions removed
- [ ] `try_connect()`/`try_world()` Rust methods removed
- [ ] At least one example demonstrates reconnection on `CarlaError::Connection`
