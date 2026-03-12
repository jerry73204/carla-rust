# Version Feature Gates

## Overview

The crate supports CARLA versions 0.9.14, 0.9.15, 0.9.16, and 0.10.0. Version-specific
features are gated using Rust `#[cfg]` attributes and C++ `#ifdef` preprocessor directives.

## Flag Naming Convention

### Cumulative flags (`carla_XXXX` — "this version or later")

Set for the specified version and all later versions. Use these for features that
were added in a version and remain available in all later versions.

| Flag          | 0.9.14 | 0.9.15 | 0.9.16 | 0.10.0 |
|---------------|--------|--------|--------|--------|
| `carla_0915`  |        | x      | x      | x      |
| `carla_0916`  |        |        | x      | x      |
| `carla_0100`  |        |        |        | x      |

### Exact flags (`carla_version_XXXX` — "exactly this version")

Set for only the specified version. Use these for features that exist in
exactly one version (added then removed, or API signature differences).

| Flag                  | 0.9.14 | 0.9.15 | 0.9.16 | 0.10.0 |
|-----------------------|--------|--------|--------|--------|
| `carla_version_0914`  | x      |        |        |        |
| `carla_version_0915`  |        | x      |        |        |
| `carla_version_0916`  |        |        | x      |        |
| `carla_version_0100`  |        |        |        | x      |

### C++ defines

The C++ side mirrors this pattern:

- **Exact:** `CARLA_VERSION_0914`, `CARLA_VERSION_0915`, `CARLA_VERSION_0916`, `CARLA_VERSION_0100`
- **Cumulative:** `CARLA_VERSION_0915_PLUS`, `CARLA_VERSION_0916_PLUS`

## Usage Guide

### Feature added in 0.9.15, present in all later versions

```rust
#[cfg(carla_0915)]
pub fn load_world_if_different(&self, map_name: &str) -> World { ... }
```

```cpp
#ifdef CARLA_VERSION_0915_PLUS
    FfiWorld LoadWorldIfDifferent(std::string map_name) const { ... }
#endif
```

### Feature only in 0.9.16 (removed in 0.10.0)

```rust
#[cfg(carla_version_0916)]
pub fn telemetry_data(&self) -> VehicleTelemetryData { ... }
```

```cpp
#ifdef CARLA_VERSION_0916
    FfiVehicleTelemetryData GetTelemetryData() const { ... }
#endif
```

### Feature absent in exactly one version

```rust
// SetKeepRightPercentage: present in 0.9.14, 0.9.15, 0.10.0 — absent in 0.9.16
#[cfg(not(carla_version_0916))]
pub fn set_keep_right_percentage(&mut self, actor: &A, percentage: f32) { ... }
```

```cpp
#ifndef CARLA_VERSION_0916
    void SetKeepRightPercentage(...) { ... }
#endif
```

### Feature only in 0.10.0

```rust
#[cfg(carla_version_0100)]  // or carla_0100 (equivalent for latest)
pub fn get_actor_name(&self) -> String { ... }
```

### Different API signatures per version

```rust
// TrySpawnActor has extra socket_name param only in 0.9.16
#[cfg(carla_version_0916)]
let actor = world.try_spawn_actor(&bp, &transform, None, AttachmentType::Rigid, "");
#[cfg(not(carla_version_0916))]
let actor = world.try_spawn_actor(&bp, &transform, None, AttachmentType::Rigid);
```

## Where flags are set

- **`carla-sys/build.rs`** — Sets exact `carla_version_*` cfg flags for carla-sys crate,
  C++ defines (exact + cumulative) for header compilation
- **`carla/build.rs`** — Sets both exact `carla_version_*` and cumulative `carla_*` flags
  for the carla crate

## Current version-specific features

### 0.9.16-only (`carla_version_0916`)

These exist only in 0.9.16 (removed in 0.10.0):

- `VehicleTelemetryData` / `WheelTelemetryData`
- `Vehicle::telemetry_data()`
- `Vehicle::set_wheel_pitch_angle()` / `wheel_pitch_angle()`
- `Vehicle::restore_phys_x_physics()`
- `TrySpawnActor` with `socket_name` parameter
- `BoundingBox::actor_id` field
- `Actor::GetComponentWorldTransform()` / `GetComponentRelativeTransform()`
- `Actor::GetBoneWorldTransforms()` / `GetBoneRelativeTransforms()`
- `Actor::GetComponentNames()` / `GetBoneNames()` / `GetSocketNames()`
- `World::GetIMUISensorGravity()` / `SetIMUISensorGravity()`

### 0.9.15+ (`carla_0915`)

- `Client::load_world_if_different()`
- `Client::set_replayer_ignore_spectator()`
- `Actor::SetCollisions()`
- `Actor::SetActorDead()`

### Absent in 0.9.16 only (`not(carla_version_0916)`)

- `TrafficManager::set_keep_right_percentage()`
