# Error Handling & FFI Exception Safety

**Priority:** HIGH
**Status:** Complete
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

### Infrastructure

**C++ side** (`carla-sys/csrc/carla_rust/error.hpp`):
- `ErrorKind` enum (Success, Timeout, NotFound, InvalidArgument, RuntimeError, OutOfRange, Unknown)
- `classify_exception()` — pattern-matches exception messages

**C++ error container** (`carla-sys/csrc/carla_rust/client/result.hpp`):
- `FfiError` class with `kind()`, `message()`, `has_error()`, `set()`, `clear()`
- `ffi_call<R, F>` and `ffi_call_void<F>` templates for exception-safe wrapping

**Rust side** (`carla/src/error.rs`, `carla/src/error/ffi.rs`):
- Full `CarlaError` hierarchy
- `parse_ffi_error()` converter
- `check_ffi_error()` helper
- `with_ffi_error()` helper — eliminates 5-line boilerplate per FFI call

### Noexcept Methods (No Wrapping Needed)

- `FfiActor::GetId()`, `GetTypeId()`, `GetDisplayId()` — cached identifiers
- `FfiActor::IsAlive()`, `IsDormant()`, `IsActive()` — cached state
- `FfiActor::GetBoundingBox()` — cached data
- `FfiWorld::GetId()` — cached episode ID
- `FfiWorldSnapshot` accessors — in-memory snapshot data
- `FfiVehicle::GetControl()`, `GetLightState()`, `GetSpeedLimit()`, etc. — cached state
- `FfiWaypoint` simple getters — `GetId()`, `GetRoadId()`, `GetTransform()`, etc.
- `FfiMap::GetName()`, `GetOpenDrive()`, `GetRecommendedSpawnPoints()` — cached data
- `FfiTrafficLight` getters — `GetState()`, `GetGreenTime()`, `IsFrozen()`, etc.
- `FfiSensor::IsListening()` — cached state
- Type conversion methods (`to_vehicle()`, `to_sensor()`) — local pointer cast

## 5.3 Template Infrastructure

- [x] Add `ffi_call<R, F>` template to `client/result.hpp`
- [x] Add `ffi_call_void<F>` template to `client/result.hpp`
- [x] Add `check_ffi_error()` helper to `carla/src/error/ffi.rs`
- [x] Add `with_ffi_error()` helper to `carla/src/error/ffi.rs`
- [x] Verify templates compile with all supported CARLA versions
- [x] Refactor `ffi_try_connect` / `ffi_try_get_world` to use templates

## 5.4 World + Client

- [x] **FfiClient** (~26 methods): All wrapped with `FfiError&`
  - `FfiClient::Create()` static factory (replaces `ffi_try_connect`)
  - `GetWorld`, `LoadWorld`, `ReloadWorld`, `GenerateOpenDriveWorld`
  - `GetTimeout`, `SetTimeout`, `GetClientVersion`, `GetServerVersion`
  - `GetAvailableMaps`, `GetInstanceTM`
  - Recording/replay methods
  - Batch operation methods
  - File transfer methods
- [x] **FfiWorld** (~35 methods): All RPC methods wrapped
  - `GetMap`, `GetBlueprintLibrary`, `GetActors`, `GetSpectator`
  - `ApplySettings`, `GetSettings`, `GetWeather`, `SetWeather`
  - `Tick`, traffic light methods, environment object methods
  - `TrySpawnActor` and `WaitForTick` stay as-is (own error handling)
- [x] **Rust API cleanup:**
  - `Client::connect()` returns `Result` directly (absorbed `try_connect`)
  - `Client::world()` returns `Result` directly (absorbed `try_world`)
  - `Client::default()` removed
  - Legacy `ffi_try_connect`/`ffi_try_get_world` free functions deleted

## 5.5 Actor + Vehicle

- [x] **FfiActor** (~25 RPC methods): All wrapped
  - Position/velocity queries: `GetLocation`, `GetTransform`, `GetVelocity`, etc.
  - Mutations: `SetLocation`, `SetTransform`, `Destroy`, etc.
  - Physics: `AddImpulse`, `AddForce`, `SetSimulatePhysics`, etc.
  - 0.9.16 component/bone/socket methods
- [x] **FfiVehicle** (~15 methods): All wrapped
  - `ApplyControl`, `SetAutopilot`, physics/steering/door/light methods
  - `GetPhysicsControl` now returns `unique_ptr`
  - Telemetry data (0.9.16)

## 5.6 Sensor + Walker

- [x] `FfiSensor::Listen` and `Stop` with `FfiError&`
- [x] `FfiWalker`: `ApplyControl`, `SetBonesTransform`, `BlendPose`, `GetPoseFromAnimation`
- [x] `FfiWalkerAIController`: `Start`, `Stop`, `GoToLocation`, `SetMaxSpeed`, `GetRandomLocation`

## 5.7 Remaining Types

- [x] `FfiMap` — waypoint generation, junction queries, landmark queries
- [x] `FfiWaypoint` — navigation (`GetNext`, `GetPrevious`, `GetLeft`, `GetRight`), lane markings, landmarks
- [x] `FfiTrafficLight` — state/timing mutations, group operations, waypoint queries
- [x] `FfiTrafficManager` — all ~36 methods wrapped
- [x] `FfiLightManager` — all ~27 methods wrapped

## 5.8 Final Cleanup

- [x] Audit: grep for any remaining C++ method without `FfiError&` that calls RPC
- [x] Remove `CARLA_TRY` / `CARLA_CATCH` macros and `ErrorInfo` struct from `error.hpp`
- [x] Remove legacy `ffi_try_connect`/`ffi_try_get_world` free functions
- [x] Update all ~60 examples to use `Result` returns
- [x] Update all doc-test examples across 12+ library files
- [x] Fix all internal callers (agents module, actor_builder, etc.)
- [ ] Add reconnection example demonstrating `CarlaError::Connection` handling

## Done Criteria

- [x] No C++ exception can abort the Rust process under any server state
- [x] All public Rust API methods return `Result<T>`
- [x] `cargo check --all-targets` passes (all CARLA versions)
- [x] `cargo clippy -- -D warnings` passes
- [x] `cargo test --lib` passes (97 tests)
- [x] `CARLA_TRY`/`CARLA_CATCH` macros removed
- [x] `ffi_try_connect`/`ffi_try_get_world` free functions removed
- [x] `try_connect()`/`try_world()` Rust methods removed
- [ ] At least one example demonstrates reconnection on `CarlaError::Connection`

## Summary of Changes

108 files changed, +3,851 / -2,889 lines:
- 13 C++ headers: Added `FfiError&` out-parameter to ~200 methods
- 12 Rust API files: All RPC methods return `crate::Result<T>`
- ~60 example files updated for `Result` handling
- Internal callers (agents, actor_builder, etc.) updated
- Legacy macros and free functions removed
