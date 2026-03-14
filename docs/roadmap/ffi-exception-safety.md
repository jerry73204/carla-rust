# FFI Exception Safety

**Priority:** HIGH
**Status:** In Progress (infrastructure done, methods not yet wrapped)
**Dependencies:** error-handling.md (Phases 1-2 complete)

## Problem

Every C++ method called through the FFI boundary can throw exceptions. Currently only 2 out of ~80+ exposed methods are wrapped in try-catch. When the CARLA server disconnects or throws for any reason, the C++ exception propagates through autocxx and **aborts** the Rust process.

The `TimeoutException` from CARLA's RPC layer is the most common crash, but `std::runtime_error`, `std::invalid_argument`, and `std::out_of_range` can also occur.

## Goal

Make the entire C++ FFI layer exception-free. Every method exposed to Rust either:
1. Catches all C++ exceptions and reports them via `FfiError&` out-parameter, or
2. Is genuinely `noexcept` (e.g., simple getters on cached data)

On the Rust side, all public API methods return `Result<T>` instead of panicking.

## Done Criteria

- [ ] No C++ exception can abort the Rust process under any CARLA server state (connected, disconnected, crashed, restarting)
- [ ] All public Rust API methods return `Result<T>`
- [ ] `cargo test` passes (unit tests, doc tests)
- [ ] At least one example demonstrates reconnection on `CarlaError::Connection`
- [ ] Old `try_connect()` / `try_world()` removed; `connect()` / `world()` return `Result` directly
- [ ] `CARLA_TRY` / `CARLA_CATCH` macros removed; `ffi_call` / `ffi_call_void` templates used everywhere

## Current State

### What exists

- **C++ error infrastructure** (`carla-sys/csrc/carla_rust/error.hpp`):
  - `ErrorKind` enum (Success, Timeout, NotFound, InvalidArgument, RuntimeError, OutOfRange, Unknown)
  - `classify_exception()` — pattern-matches exception messages
  - `CARLA_TRY` / `CARLA_CATCH` macros (to be removed)

- **C++ error container** (`carla-sys/csrc/carla_rust/client/result.hpp`):
  - `FfiError` class with `kind()`, `message()`, `has_error()`, `set()`, `clear()`
  - `ffi_try_connect()` — wraps `FfiClient` constructor (to be removed)
  - `ffi_try_get_world()` — wraps `FfiClient::GetWorld()` (to be removed)

- **Rust error types** (`carla/src/error.rs`, `carla/src/error/ffi.rs`):
  - Full `CarlaError` hierarchy (Connection, Resource, Operation, Validation, Map, Sensor, Internal)
  - `FfiErrorKind` enum matching C++ `ErrorKind`
  - `parse_ffi_error()` — converts C++ error kind + message to `CarlaError`

- **Rust API** (`carla/src/client/*.rs`):
  - `Client::try_connect()` and `Client::try_world()` use `FfiError` pattern
  - All other methods call C++ directly without exception handling

---

## Design

### C++ template for exception-safe calls

Replace the `CARLA_TRY`/`CARLA_CATCH` macros with type-safe templates in `error.hpp`:

```cpp
namespace carla_rust::error {

/// Wrap a callable in try-catch. On exception, populate error and return default_val.
template <typename F, typename R = std::invoke_result_t<F>>
R ffi_call(FfiError& error, R default_val, F&& fn) {
    error.clear();
    try {
        return fn();
    } catch (const std::exception& e) {
        error.set(static_cast<int32_t>(classify_exception(e)), e.what());
        return default_val;
    } catch (...) {
        error.set(static_cast<int32_t>(ErrorKind::Unknown), "Unknown C++ exception");
        return default_val;
    }
}

/// Void specialization — no return value, no default needed.
template <typename F>
void ffi_call_void(FfiError& error, F&& fn) {
    error.clear();
    try {
        fn();
    } catch (const std::exception& e) {
        error.set(static_cast<int32_t>(classify_exception(e)), e.what());
    } catch (...) {
        error.set(static_cast<int32_t>(ErrorKind::Unknown), "Unknown C++ exception");
    }
}

}  // namespace carla_rust::error
```

### C++ wrapper method pattern

Every method on Ffi wrapper classes gains an `FfiError&` parameter:

```cpp
// Pointer-returning:
std::unique_ptr<FfiWorldSnapshot> GetSnapshot(FfiError& error) const {
    return ffi_call(error, nullptr, [&] {
        return std::make_unique<FfiWorldSnapshot>(inner_.GetSnapshot());
    });
}

// Value-returning:
uint64_t Tick(size_t millis, FfiError& error) {
    return ffi_call(error, uint64_t{0}, [&] {
        return inner_.Tick(time_duration::milliseconds(millis));
    });
}

// Void:
void SetWeather(const WeatherParameters& weather, FfiError& error) {
    ffi_call_void(error, [&] {
        inner_.SetWeather(weather);
    });
}
```

### WaitForTick special case

`WaitForTick` catches `TimeoutException` separately because timeout is expected behavior (returns `None`), not an error:

```cpp
std::unique_ptr<FfiWorldSnapshot> WaitForTick(size_t millis, FfiError& error) const {
    error.clear();
    try {
        auto snapshot = inner_.WaitForTick(time_duration::milliseconds(millis));
        return std::make_unique<FfiWorldSnapshot>(std::move(snapshot));
    } catch (const TimeoutException&) {
        return nullptr;  // Timeout is Ok(None), not Err
    } catch (const std::exception& e) {
        error.set(static_cast<int32_t>(classify_exception(e)), e.what());
        return nullptr;
    } catch (...) {
        error.set(static_cast<int32_t>(ErrorKind::Unknown), "Unknown C++ exception");
        return nullptr;
    }
}
```

### Rust-side helper

```rust
/// Check FfiError after a C++ call and convert to Result.
fn check_ffi_error(error: &FfiError, operation: &str) -> Result<()> {
    if error.has_error() {
        let msg = error.message();
        Err(parse_ffi_error(
            error.kind(),
            msg.to_str().unwrap_or("unknown"),
            Some(operation),
        ))
    } else {
        Ok(())
    }
}

// Usage:
pub fn snapshot(&self) -> Result<WorldSnapshot> {
    let mut error = FfiError::new().within_unique_ptr();
    let ptr = self.inner.GetSnapshot(error.pin_mut());
    check_ffi_error(&error, "get_snapshot")?;
    Ok(unsafe { WorldSnapshot::from_cxx(ptr).unwrap_unchecked() })
}

pub fn wait_for_tick_or_timeout(&self, timeout: Duration) -> Result<Option<WorldSnapshot>> {
    let mut error = FfiError::new().within_unique_ptr();
    let ptr = self.inner.WaitForTick(timeout.as_millis() as usize, error.pin_mut());
    check_ffi_error(&error, "wait_for_tick")?;
    Ok(WorldSnapshot::from_cxx(ptr))  // None on timeout, Some on success
}
```

### Noexcept methods (no wrapping needed)

Some methods access only cached local data without calling the CARLA server:

- `FfiActor::GetId()`, `GetTypeId()`, `GetDisplayId()` — cached identifiers
- `FfiActor::IsAlive()`, `IsDormant()`, `IsActive()` — cached state
- `FfiWorld::GetId()` — cached episode ID
- `FfiWorldSnapshot` accessors — in-memory snapshot data
- Type conversion methods (`to_vehicle()`, `to_sensor()`) — local pointer cast

Everything that calls `inner_->SomeMethod()` where `SomeMethod` hits the RPC layer needs wrapping.

---

## Phase 0: Template infrastructure

Add `ffi_call` / `ffi_call_void` templates and the Rust-side `check_ffi_error` helper.

### Work items

- [ ] Add `ffi_call<F, R>` template to `carla-sys/csrc/carla_rust/error.hpp`
- [ ] Add `ffi_call_void<F>` template to `carla-sys/csrc/carla_rust/error.hpp`
- [ ] Add `check_ffi_error()` helper to `carla/src/error/ffi.rs`
- [ ] Verify templates compile with all supported CARLA versions (0.9.14, 0.9.15, 0.9.16)
- [ ] Verify `FfiError` is accessible from all wrapper headers (include path)

### Done criteria

- [ ] Templates defined and compiling
- [ ] `check_ffi_error()` available in Rust
- [ ] `cargo build` passes for all version-gated configs

---

## Phase 1: World + Client

These methods are called every tick or during connection setup. A crash here kills the bridge.

### Work items — FfiWorld (`carla-sys/csrc/carla_rust/client/world.hpp`)

- [ ] `GetSnapshot(FfiError&)` — main loop, every tick
- [ ] `WaitForTick(millis, FfiError&)` — main loop, every tick (special: `TimeoutException` → nullptr, not error)
- [ ] `Tick(millis, FfiError&)` — sync mode main loop
- [ ] `GetMap(FfiError&)` — startup
- [ ] `GetBlueprintLibrary(FfiError&)` — vehicle spawn
- [ ] `ApplySettings(settings, timeout, FfiError&)` — config
- [ ] `GetSettings(FfiError&)` — config
- [ ] `GetActors(FfiError&)` — queries
- [ ] `GetActorsByIds(ids, FfiError&)` — queries
- [ ] `GetActor(id, FfiError&)` — queries
- [ ] `GetWeather(FfiError&)` — environment
- [ ] `SetWeather(weather, FfiError&)` — environment
- [ ] `GetSpectator(FfiError&)` — camera control
- [ ] `TrySpawnActor(..., FfiError&)` — spawning (currently noexcept but may still throw from inner calls)
- [ ] `LoadLevelLayer(layers, FfiError&)` — asset loading
- [ ] `UnloadLevelLayer(layers, FfiError&)` — asset unloading
- [ ] `GetRandomLocationFromNavigation(FfiError&)` — nav mesh
- [ ] `GetVehiclesLightStates(FfiError&)` — queries
- [ ] `GetTrafficSign(landmark, FfiError&)` — queries
- [ ] `GetTrafficLight(landmark, FfiError&)` — queries
- [ ] `GetTrafficLightFromOpenDRIVE(sign_id, FfiError&)` — queries
- [ ] `ResetAllTrafficLights(FfiError&)` — mutations
- [ ] `FreezeAllTrafficLights(frozen, FfiError&)` — mutations
- [ ] `GetLightManager(FfiError&)` — queries
- [ ] `GetLevelBBs(tag, FfiError&)` — queries
- [ ] `GetTrafficLightsFromWaypoint(wp, dist, FfiError&)` — queries
- [ ] `GetTrafficLightsInJunction(junc_id, FfiError&)` — queries
- [ ] `SetPedestriansCrossFactor(pct, FfiError&)` — config
- [ ] `SetPedestriansSeed(seed, FfiError&)` — config
- [ ] `GetEnvironmentObjects(tag, FfiError&)` — queries
- [ ] `EnableEnvironmentObjects(ids, enable, FfiError&)` — mutations
- [ ] `ProjectPoint(loc, dir, dist, FfiError&)` — raycasting
- [ ] `GroundProjection(loc, dist, FfiError&)` — raycasting
- [ ] `CastRay(start, end, FfiError&)` — raycasting
- [ ] `MakeDebugHelper(FfiError&)` — debug
- [ ] `GetNamesOfAllObjects(FfiError&)` — queries
- [ ] Version-gated methods: `GetIMUISensorGravity`, `SetIMUISensorGravity`, `SetAnnotationsTraverseTranslucency`, `IsWeatherEnabled`

### Work items — FfiClient (`carla-sys/csrc/carla_rust/client/client.hpp`)

- [ ] Constructor: add `FfiError&` parameter (replaces `ffi_try_connect` free function)
- [ ] `GetWorld(FfiError&)` — replaces `ffi_try_get_world` free function
- [ ] `LoadWorld(map, reset, FfiError&)` — map loading
- [ ] `LoadWorldIfDifferent(map, reset, FfiError&)` — map loading (0.9.15+)
- [ ] `ReloadWorld(reset, FfiError&)` — map loading
- [ ] `GenerateOpenDriveWorld(odr, params, reset, FfiError&)` — map generation
- [ ] `GetAvailableMaps(FfiError&)` — queries
- [ ] `GetInstanceTM(port, FfiError&)` — traffic manager
- [ ] `RequestFile(name, FfiError&)` — file transfer
- [ ] `GetRequiredFiles(folder, download, FfiError&)` — file transfer
- [ ] `SetFilesBaseFolder(path, FfiError&)` — file transfer
- [ ] `StartRecorder(filename, additional_data, FfiError&)` — recording
- [ ] `StopRecorder(FfiError&)` — recording
- [ ] `ShowRecorderFileInfo(filename, show_all, FfiError&)` — recording
- [ ] `ShowRecorderCollisions(filename, t1, t2, FfiError&)` — recording
- [ ] `ShowRecorderActorsBlocked(filename, min_time, min_dist, FfiError&)` — recording
- [ ] `ReplayFile(filename, start, duration, follow_id, sensors, FfiError&)` — replay
- [ ] `StopReplayer(keep_actors, FfiError&)` — replay
- [ ] `SetReplayerTimeFactor(factor, FfiError&)` — replay
- [ ] `SetReplayerIgnoreHero(ignore, FfiError&)` — replay
- [ ] `SetReplayerIgnoreSpectator(ignore, FfiError&)` — replay (0.9.15+)
- [ ] `ApplyBatch(batch, do_tick, FfiError&)` — batch operations
- [ ] `ApplyBatchSync(batch, do_tick, FfiError&)` — batch operations

### Work items — Rust API

- [ ] `World` methods in `carla/src/client/world.rs` — all return `Result<T>`
- [ ] `Client` methods in `carla/src/client/carla_client.rs` — all return `Result<T>`
- [ ] Remove `Client::try_connect()` — `Client::connect()` now returns `Result`
- [ ] Remove `Client::try_world()` — `Client::world()` now returns `Result`
- [ ] Update `bindings.rs` — remove `generate!` for `ffi_try_connect`, `ffi_try_get_world`

### Work items — Cleanup

- [ ] Delete `ffi_try_connect()` from `result.hpp`
- [ ] Delete `ffi_try_get_world()` from `result.hpp`
- [ ] Delete `CARLA_TRY` / `CARLA_CATCH` macros from `error.hpp`

### Done criteria

- [ ] All `FfiWorld` methods take `FfiError&` (except noexcept: `GetId`, `clone`)
- [ ] All `FfiClient` methods take `FfiError&` (except noexcept: `GetTimeout`, `SetTimeout`, `GetClientVersion`, `GetServerVersion`)
- [ ] All `World` Rust methods return `Result<T>`
- [ ] All `Client` Rust methods return `Result<T>`
- [ ] `ffi_try_connect` / `ffi_try_get_world` free functions deleted
- [ ] `try_connect()` / `try_world()` Rust methods deleted
- [ ] `cargo build` passes
- [ ] Examples using `Client::connect` / `Client::world` updated

---

## Phase 2: Actor + Vehicle

### Work items — FfiActor (`carla-sys/csrc/carla_rust/client/actor.hpp`)

- [ ] `GetLocation(FfiError&)` — position queries
- [ ] `GetTransform(FfiError&)` — pose queries
- [ ] `GetVelocity(FfiError&)` — velocity queries
- [ ] `GetAngularVelocity(FfiError&)` — angular velocity
- [ ] `GetAcceleration(FfiError&)` — acceleration
- [ ] `SetLocation(loc, FfiError&)` — teleportation
- [ ] `SetTransform(transform, FfiError&)` — teleportation
- [ ] `SetTargetVelocity(vec, FfiError&)` — physics
- [ ] `SetTargetAngularVelocity(vec, FfiError&)` — physics
- [ ] `EnableConstantVelocity(vec, FfiError&)` — physics
- [ ] `DisableConstantVelocity(FfiError&)` — physics
- [ ] `AddImpulse1(vec, FfiError&)` / `AddImpulse2(impulse, loc, FfiError&)` — physics
- [ ] `AddForce1(force, FfiError&)` / `AddForce2(force, loc, FfiError&)` — physics
- [ ] `AddAngularImpulse(vec, FfiError&)` — physics
- [ ] `AddTorque(vec, FfiError&)` — physics
- [ ] `SetSimulatePhysics(enabled, FfiError&)` — physics
- [ ] `SetEnableGravity(enabled, FfiError&)` — physics
- [ ] `GetBoundingBox(FfiError&)` — geometry
- [ ] `GetParent(FfiError&)` — actor tree
- [ ] `GetWorld(FfiError&)` — world access
- [ ] `GetAttributes(FfiError&)` — metadata
- [ ] `Destroy(FfiError&)` — cleanup
- [ ] `SetCollisions(enabled, FfiError&)` — 0.9.15+
- [ ] `SetActorDead(FfiError&)` — 0.9.15+
- [ ] Version-gated methods: `GetComponentWorldTransform`, `GetComponentRelativeTransform`, `GetBoneWorldTransforms`, `GetBoneRelativeTransforms`, `GetComponentNames`, `GetBoneNames`, `GetSocketWorldTransforms`, `GetSocketRelativeTransforms`, `GetSocketNames`, `GetActorName`, `GetActorClassName`

### Work items — FfiVehicle (`carla-sys/csrc/carla_rust/client/vehicle.hpp`)

- [ ] `ApplyControl(control, FfiError&)` — every tick
- [ ] `SetAutopilot(enabled, tm_port, FfiError&)` — traffic manager
- [ ] `ShowDebugTelemetry(enabled, FfiError&)` — debug
- [ ] `ApplyPhysicsControl(control, FfiError&)` — physics
- [ ] `ApplyAckermannControl(control, FfiError&)` — steering
- [ ] `ApplyAckermannControllerSettings(settings, FfiError&)` — steering
- [ ] `OpenDoor(door, FfiError&)` / `CloseDoor(door, FfiError&)` — doors
- [ ] `SetLightState(state, FfiError&)` — lights
- [ ] `SetWheelSteerDirection(loc, degrees, FfiError&)` — wheels
- [ ] `SetWheelPitchAngle(loc, degrees, FfiError&)` — wheels
- [ ] `RestorePhysXPhysics(FfiError&)` — physics engine
- [ ] `EnableCarSim(simfile, FfiError&)` — physics engine
- [ ] `UseCarSimRoad(enabled, FfiError&)` — physics engine
- [ ] `EnableChronoPhysics(..., FfiError&)` — physics engine

### Work items — Rust API

- [ ] `Actor` methods in `carla/src/client/actor.rs` — all return `Result<T>`
- [ ] `Vehicle` methods in `carla/src/client/vehicle.rs` — all return `Result<T>`

### Done criteria

- [ ] All `FfiActor` RPC methods take `FfiError&` (except noexcept: `GetId`, `GetTypeId`, `GetDisplayId`, `GetParentId`, `GetSemanticTags`, `IsAlive`, `IsDormant`, `IsActive`, `GetActorState`, `as_builtin`, `to_vehicle`, `to_sensor`, `to_traffic_sign`, `to_traffic_light`)
- [ ] All `FfiVehicle` methods take `FfiError&`
- [ ] All `Actor` / `Vehicle` Rust methods return `Result<T>`
- [ ] `cargo build` passes

---

## Phase 3: Sensor

### Work items — FfiSensor (`carla-sys/csrc/carla_rust/client/sensor.hpp`)

- [ ] `Listen(caller_ptr, fn_ptr, deleter_ptr, FfiError&)` — callback registration
- [ ] `Stop(FfiError&)` — streaming teardown

### Work items — Rust API

- [ ] `Sensor` methods in `carla/src/client/sensor.rs` — return `Result<T>`

### Done criteria

- [ ] Both `FfiSensor` methods take `FfiError&`
- [ ] `Sensor::listen()` and `Sensor::stop()` return `Result<()>`
- [ ] `cargo build` passes

---

## Phase 4: Remaining types

### Work items — FfiMap (`carla-sys/csrc/carla_rust/client/map.hpp`)

- [ ] `GenerateWaypoints(distance, FfiError&)` — road network
- [ ] `GetJunction(waypoint, FfiError&)` — road network
- [ ] All other map query methods that hit RPC

### Work items — FfiWaypoint (`carla-sys/csrc/carla_rust/client/waypoint.hpp`)

- [ ] `GetJunction(FfiError&)` — road network
- [ ] All other waypoint methods that hit RPC

### Work items — FfiTrafficLight (`carla-sys/csrc/carla_rust/client/traffic_light.hpp`)

- [ ] `SetState(state, FfiError&)` — state mutation
- [ ] `SetGreenTime(time, FfiError&)` — timing
- [ ] `SetYellowTime(time, FfiError&)` — timing
- [ ] `SetRedTime(time, FfiError&)` — timing
- [ ] `Freeze(freeze, FfiError&)` — state
- [ ] `ResetGroup(FfiError&)` — group reset

### Work items — FfiTrafficManager (`carla-sys/csrc/carla_rust/traffic_manager/traffic_manager.hpp`)

- [ ] All traffic manager methods — review and wrap

### Work items — FfiWalker (`carla-sys/csrc/carla_rust/client/walker.hpp`)

- [ ] `ApplyControl(control, FfiError&)` — movement
- [ ] `SetBonesTransformFfi(bones, FfiError&)` — skeletal animation
- [ ] `BlendPose(blend, FfiError&)` — animation

### Work items — FfiWalkerAIController (`carla-sys/csrc/carla_rust/client/walker_ai_controller.hpp`)

- [ ] `Start(FfiError&)` — AI init
- [ ] `Stop(FfiError&)` — AI shutdown
- [ ] `GetRandomLocation(FfiError&)` — nav mesh query
- [ ] `GoToLocation(loc, FfiError&)` — navigation
- [ ] `SetMaxSpeed(speed, FfiError&)` — config

### Work items — FfiLightManager (`carla-sys/csrc/carla_rust/client/light_manager.hpp`)

- [ ] `SetActive(id, active, FfiError&)` — state
- [ ] `SetColor(id, color, FfiError&)` — state
- [ ] `SetIntensity(id, intensity, FfiError&)` — state
- [ ] `SetLightState(id, state, FfiError&)` — state
- [ ] `SetLightGroup(id, group, FfiError&)` — state

### Work items — Rust API

- [ ] `Map` methods — return `Result<T>`
- [ ] `Waypoint` methods — return `Result<T>`
- [ ] `TrafficLight` methods — return `Result<T>`
- [ ] `TrafficManager` methods — return `Result<T>`
- [ ] `Walker` methods — return `Result<T>`
- [ ] `WalkerAIController` methods — return `Result<T>`
- [ ] `LightManager` methods — return `Result<T>`

### Done criteria

- [ ] All RPC-calling methods across all types take `FfiError&`
- [ ] All corresponding Rust methods return `Result<T>`
- [ ] `cargo build` passes

---

## Phase 5: Final cleanup and examples

### Work items

- [ ] Audit: grep for any remaining C++ method without `FfiError&` that calls RPC (ensure none missed)
- [ ] Remove `CARLA_TRY` / `CARLA_CATCH` macro definitions from `error.hpp`
- [ ] Remove `ErrorInfo` struct from `error.hpp` if unused (FfiError in result.hpp is the replacement)
- [ ] Update all examples in `carla/examples/` to use `Result` returns
- [ ] Update `docs/roadmap/error-handling.md` status to reflect completion
- [ ] Add reconnection example demonstrating `CarlaError::Connection` handling
- [ ] Update CHANGELOG

### Done criteria

- [ ] `grep -r "CARLA_TRY\|CARLA_CATCH" carla-sys/` returns no results
- [ ] `grep -rn "ffi_try_connect\|ffi_try_get_world\|try_connect\|try_world" carla/src/` returns no results
- [ ] No panicking FFI calls remain (all methods either take `FfiError&` or are genuinely noexcept)
- [ ] All examples compile and run
- [ ] `cargo test` passes
- [ ] `cargo doc` builds without warnings
