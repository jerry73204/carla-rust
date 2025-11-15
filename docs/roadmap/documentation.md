# Documentation Roadmap

This document tracks the documentation improvement plan for the carla crate, ensuring comprehensive API coverage with Python API cross-references.

## Progress Tracking

Last Updated: 2025-11-15

| Phase | Status | Completion | Priority | Est. Time |
|-------|--------|------------|----------|-----------|
| Phase 1: Crate & Module Docs | Not Started | 0% | Critical | 4-6 hours |
| Phase 2: Core Types (World, Client, Vehicle) | Partial | 60% | Critical | 8-10 hours |
| Phase 3: Waypoint & Navigation | Not Started | 0% | High | 10-12 hours |
| Phase 4: Walker Ecosystem | Not Started | 0% | High | 6-8 hours |
| Phase 5: Traffic & Sensors | Not Started | 0% | Medium | 8-10 hours |
| Phase 6: Advanced Features | Not Started | 0% | Medium | 6-8 hours |
| Phase 7: Completeness Pass | Not Started | 0% | Low | 10-15 hours |

**Overall Completion**: ~15% (rough estimate based on type coverage)

## Current State Analysis

### Crate-Level Documentation
- ✅ Excellent overview with examples
- ⚠️ Version number needs update (currently shows 0.9.15, should be 0.9.16 or dynamic)
- ✅ Good architecture explanation
- ✅ Multi-version support documented

### Module-Level Documentation

| Module | Status | Notes |
|--------|--------|-------|
| `client` | ⚠️ Partial | Module doc exists but needs expansion |
| `geom` | ✅ Good | Well-documented with examples |
| `rpc` | ❌ Missing | No module-level docs |
| `sensor` | ❌ Missing | No module-level docs |
| `traffic_manager` | ❌ Missing | No module-level docs |
| `road` | ❌ Missing | No module-level docs |
| `agents` | ❌ Missing | No module-level docs |

### Type Coverage Matrix

**Legend**:
- ✅ Complete (type + all methods documented with Python refs)
- ⚠️ Partial (type doc exists but missing method docs or Python refs)
- ❌ Missing (no documentation)

#### client Module

| Type | Docs | Python Ref | Methods | Priority |
|------|------|------------|---------|----------|
| Client | ✅ | ✅ | ✅ All 15 methods | Done |
| World | ✅ | ✅ | ⚠️ ~50% methods | P1 - High |
| Vehicle | ✅ | ✅ | ⚠️ ~60% methods | P1 - High |
| Actor | ⚠️ | ✅ | ⚠️ 15/30 methods | P2 - High |
| ActorBlueprint | ✅ | ✅ | ✅ All methods | Done |
| BlueprintLibrary | ✅ | ✅ | ✅ All methods | Done |
| **Waypoint** | ⚠️ | ❌ | ❌ **0/85+ methods** | **P3 - Critical** |
| Walker | ❌ | ❌ | ❌ All methods | P4 - High |
| WalkerAIController | ❌ | ❌ | ❌ All methods | P4 - High |
| TrafficLight | ❌ | ❌ | ❌ All methods | P5 - Medium |
| TrafficSign | ❌ | ❌ | ❌ All methods | P5 - Medium |
| Junction | ❌ | ❌ | ❌ All methods | P5 - Medium |
| Landmark | ❌ | ❌ | ❌ All methods | P5 - Medium |
| Light | ❌ | ❌ | ❌ All methods | P6 - Medium |
| LightManager | ❌ | ❌ | ❌ All methods | P6 - Medium |
| Map | ⚠️ | ⚠️ | ⚠️ ~40% methods | P3 - High |
| Sensor | ⚠️ | ⚠️ | ❌ Most methods | P5 - Medium |
| WorldSnapshot | ❌ | ❌ | ❌ All methods | P5 - Medium |
| Timestamp | ❌ | ❌ | ❌ All methods | P5 - Medium |

#### geom Module

| Type | Docs | Python Ref | Methods | Priority |
|------|------|------------|---------|----------|
| Location | ✅ | ✅ | ✅ All methods | Done |
| Rotation | ✅ | ✅ | ✅ All methods | Done |
| Transform | ✅ | ✅ | ✅ All methods | Done |
| Vector2D | ✅ | ✅ | ✅ All methods | Done |
| Vector3D | ✅ | ✅ | ✅ All methods | Done |
| BoundingBox | ✅ | ✅ | ✅ All methods | Done |
| GeoLocation | ✅ | ✅ | ✅ All methods | Done |

#### rpc Module

| Type | Docs | Python Ref | Methods | Priority |
|------|------|------------|---------|----------|
| VehicleControl | ✅ | ✅ | ✅ All fields | Done |
| WeatherParameters | ✅ | ✅ | ✅ All fields | Done |
| VehiclePhysicsControl | ⚠️ | ⚠️ | ⚠️ Some fields | P6 - Medium |
| WalkerControl | ❌ | ❌ | ❌ All fields | P4 - High |
| WalkerBoneControlIn | ❌ | ❌ | ❌ All fields | P6 - Low |
| VehicleTelemetryData | ❌ | ❌ | ❌ All fields | P6 - Medium |
| TrafficLightState | ❌ | ❌ | N/A (enum) | P5 - Medium |
| AttachmentType | ❌ | ❌ | N/A (enum) | P7 - Low |

#### sensor Module

| Type | Docs | Python Ref | Methods | Priority |
|------|------|------------|---------|----------|
| Image | ❌ | ❌ | ❌ All methods | P5 - Medium |
| LidarMeasurement | ❌ | ❌ | ❌ All methods | P5 - Medium |
| RadarMeasurement | ❌ | ❌ | ❌ All methods | P5 - Medium |
| CollisionEvent | ❌ | ❌ | ❌ All methods | P5 - Medium |
| GnssMeasurement | ❌ | ❌ | ❌ All methods | P6 - Low |
| IMUMeasurement | ❌ | ❌ | ❌ All methods | P6 - Low |
| LaneInvasionEvent | ❌ | ❌ | ❌ All methods | P6 - Low |

#### traffic_manager Module

| Type | Docs | Python Ref | Methods | Priority |
|------|------|------------|---------|----------|
| TrafficManager | ❌ | ❌ | ❌ All methods | P6 - Medium |

#### road Module

| Type | Docs | Python Ref | Methods | Priority |
|------|------|------------|---------|----------|
| LaneMarking | ❌ | ❌ | ❌ All methods | P7 - Low |
| LaneType | ❌ | ❌ | N/A (enum) | P7 - Low |

#### agents Module

| Type | Docs | Python Ref | Methods | Priority |
|------|------|------------|---------|----------|
| LocalPlanner | ❌ | ❌ | ❌ All methods | P6 - Low |
| VehiclePIDController | ❌ | ❌ | ❌ All methods | P6 - Low |

## Documentation Guidelines

### Python API Cross-Reference Format

Every type and method should include a link to the corresponding Python API documentation:

```rust
/// Controls a vehicle in the simulation.
///
/// This type corresponds to the Python API's
/// [carla.Vehicle](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Vehicle).
///
/// # Example
/// ```no_run
/// # use carla::client::{Client, Vehicle};
/// # let client = Client::new("localhost", 2000, 2);
/// # let world = client.world();
/// let vehicle = world.spawn_actor(...)?;
/// vehicle.set_autopilot(true);
/// ```
pub struct Vehicle { ... }
```

For methods:

```rust
/// Enables or disables autopilot for this vehicle.
///
/// See [carla.Vehicle.set_autopilot](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Vehicle.set_autopilot)
/// in the Python API.
///
/// # Arguments
/// * `enabled` - Whether to enable autopilot
///
/// # Example
/// ```no_run
/// # use carla::client::Vehicle;
/// # let vehicle: Vehicle = todo!();
/// vehicle.set_autopilot(true);
/// ```
pub fn set_autopilot(&self, enabled: bool) { ... }
```

### Version-Aware Documentation

When documenting version-specific features:

```rust
/// Vehicle telemetry data including wheel information.
///
/// **Available in CARLA 0.9.16+ only.**
///
/// This type corresponds to the Python API's
/// [carla.VehicleTelemetryData](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.VehicleTelemetryData).
#[cfg(carla_0916)]
pub struct VehicleTelemetryData { ... }
```

### Module Documentation Template

```rust
//! Brief description of the module's purpose.
//!
//! This module provides types and functions for [specific domain].
//! It corresponds to the `carla.[module]` namespace in the Python API.
//!
//! # Key Types
//! - [`Type1`] - Brief description
//! - [`Type2`] - Brief description
//!
//! # Examples
//! ```no_run
//! // Common usage example
//! ```
//!
//! # Python API Reference
//! See the [carla.[module]](https://carla.readthedocs.io/en/0.9.16/python_api/#carlamodule)
//! documentation for the Python equivalent.
```

## Implementation Plan

### Phase 1: Crate & Module Documentation (Critical Priority)
**Est. Time**: 4-6 hours

- [ ] Update crate-level doc version number (0.9.15 → 0.9.16 or dynamic)
- [ ] Add module docs for `rpc`
- [ ] Add module docs for `sensor`
- [ ] Add module docs for `traffic_manager`
- [ ] Add module docs for `road`
- [ ] Add module docs for `agents`
- [ ] Expand `client` module docs with key type overview

**Completion Criteria**: All modules have comprehensive module-level documentation with Python API links.

### Phase 2: Complete Core Types (Critical Priority)
**Est. Time**: 8-10 hours

Focus on completing documentation for partially-documented high-usage types:

- [ ] **World**: Document remaining ~50% of methods
  - [ ] `spawn_actor()`, `try_spawn_actor()`
  - [ ] `get_actors()`, `get_actor()`
  - [ ] `get_spectator()`, `get_settings()`, `apply_settings()`
  - [ ] `tick()`, `wait_for_tick()`
  - [ ] `on_tick()`, `remove_on_tick()`
  - [ ] Add Python API links to all methods

- [ ] **Vehicle**: Document remaining ~40% of methods
  - [ ] `apply_physics_control()`, `get_physics_control()`
  - [ ] `set_light_state()`, `get_light_state()`
  - [ ] `open_door()`, `close_door()`
  - [ ] `set_wheel_steer_direction()`
  - [ ] `get_traffic_light_state()`
  - [ ] Add Python API links to all methods

- [ ] **Actor**: Document remaining ~50% of methods
  - [ ] `get_world()`, `get_location()`, `get_transform()`
  - [ ] `get_velocity()`, `get_angular_velocity()`, `get_acceleration()`
  - [ ] `set_target_velocity()`, `set_target_angular_velocity()`
  - [ ] `enable_constant_velocity()`, `disable_constant_velocity()`
  - [ ] `add_impulse()`, `add_force()`, `add_angular_impulse()`, `add_torque()`
  - [ ] Add Python API links to all methods

**Completion Criteria**: World, Vehicle, and Actor types have 100% method documentation with Python refs.

### Phase 3: Waypoint & Navigation (High Priority)
**Est. Time**: 10-12 hours

**Critical Gap**: Waypoint has 85+ undocumented methods.

- [ ] Add type-level documentation for `Waypoint` with Python API link
- [ ] Document navigation methods:
  - [ ] `next()`, `previous()`, `next_until_lane_end()`, `previous_until_lane_start()`
  - [ ] `get_left_lane()`, `get_right_lane()`
  - [ ] `get_junction()`
  - [ ] `get_landmarks()`, `get_landmarks_of_type()`
- [ ] Document properties/getters:
  - [ ] `id()`, `transform()`, `road_id()`, `section_id()`, `lane_id()`
  - [ ] `s()`, `is_junction()`, `lane_width()`, `lane_change()`
  - [ ] `lane_type()`, `right_lane_marking()`, `left_lane_marking()`
- [ ] Document all remaining methods with Python API links
- [ ] Complete `Map` documentation:
  - [ ] `get_spawn_points()`
  - [ ] `get_waypoint()`, `get_waypoint_xodr()`
  - [ ] `get_topology()`
  - [ ] `generate_waypoints()`
  - [ ] `get_all_landmarks()`, `get_all_landmarks_of_type()`

**Completion Criteria**: Waypoint and Map types have 100% method documentation.

### Phase 4: Walker Ecosystem (High Priority)
**Est. Time**: 6-8 hours

Document the pedestrian/walker API:

- [ ] **Walker**:
  - [ ] Add type-level docs with Python API link
  - [ ] Document `apply_control()`
  - [ ] Document `get_control()`
  - [ ] Document bone control methods (if exposed)
  - [ ] Add usage examples

- [ ] **WalkerAIController**:
  - [ ] Add type-level docs with Python API link
  - [ ] Document `start()`, `stop()`, `go_to_location()`
  - [ ] Document `set_max_speed()`
  - [ ] Add usage examples

- [ ] **WalkerControl** (rpc):
  - [ ] Document all fields
  - [ ] Add Python API link
  - [ ] Add usage examples

- [ ] **WalkerBoneControlIn** (rpc):
  - [ ] Document all fields
  - [ ] Add Python API link
  - [ ] Note version availability

**Completion Criteria**: Complete walker ecosystem documentation with examples.

### Phase 5: Traffic & Sensors (Medium Priority)
**Est. Time**: 8-10 hours

- [ ] **Traffic Types**:
  - [ ] `TrafficLight`: All methods + Python refs
  - [ ] `TrafficSign`: All methods + Python refs
  - [ ] `Junction`: All methods + Python refs
  - [ ] `Landmark`: All methods + Python refs
  - [ ] `TrafficLightState` enum

- [ ] **Sensor Data Types**:
  - [ ] `Image`: Document pixel access, save methods
  - [ ] `LidarMeasurement`: Document point cloud access
  - [ ] `RadarMeasurement`: Document detection access
  - [ ] `CollisionEvent`: Document collision data
  - [ ] `GnssMeasurement`, `IMUMeasurement`, `LaneInvasionEvent`

- [ ] **Sensor Base**:
  - [ ] Complete `Sensor` documentation
  - [ ] Document listening/callback patterns

**Completion Criteria**: All traffic and sensor types fully documented.

### Phase 6: Advanced Features (Medium Priority)
**Est. Time**: 6-8 hours

- [ ] **Physics & Control**:
  - [ ] Complete `VehiclePhysicsControl` documentation
  - [ ] Document `VehicleTelemetryData` (0.9.16+)
  - [ ] Document `WheelPhysicsControl`

- [ ] **Lighting**:
  - [ ] `Light`: All methods + Python refs
  - [ ] `LightManager`: All methods + Python refs
  - [ ] `VehicleLightState`

- [ ] **Traffic Manager**:
  - [ ] `TrafficManager`: All methods + Python refs
  - [ ] Document auto-lane-change, collision detection, etc.

- [ ] **Agents** (if exposing to users):
  - [ ] `LocalPlanner`
  - [ ] `VehiclePIDController`

**Completion Criteria**: All advanced features documented.

### Phase 7: Completeness Pass (Low Priority)
**Est. Time**: 10-15 hours

- [ ] Audit all types for missing Python API links
- [ ] Ensure all public methods have documentation
- [ ] Add examples to complex types
- [ ] Document enums and type aliases
- [ ] Review and improve existing documentation
- [ ] Add cross-references between related types
- [ ] Verify all links work (Python API URLs)
- [ ] Add "See also" sections where appropriate
- [ ] Document common patterns and best practices
- [ ] Add troubleshooting/FAQ section to crate docs

**Completion Criteria**: 100% documentation coverage with consistent quality.

## Quick Wins (Can be done anytime)

These are small, independent tasks that can be completed quickly:

- [ ] Add Python API link to `VehicleControl`
- [ ] Add Python API link to `WeatherParameters`
- [ ] Document `TrafficLightState` enum values
- [ ] Document `AttachmentType` enum values
- [ ] Add examples to `Transform::look_at()`
- [ ] Document `GeoLocation` fields
- [ ] Add module doc for `prelude`

## Metrics & Tracking

### Current Coverage Estimates

- **Crate-level docs**: 90% (needs version update)
- **Module-level docs**: 30% (2/7 modules)
- **Type-level docs**: 25% (~40/~150 types)
- **Method-level docs**: 15% (~100/~700 methods)
- **Python API cross-refs**: 20% (~30/~150 types)

### Target Coverage

- **Crate-level docs**: 100%
- **Module-level docs**: 100%
- **Type-level docs**: 95%+ (excluding internal/unstable types)
- **Method-level docs**: 90%+ (excluding obvious getters/setters)
- **Python API cross-refs**: 95%+ (all public types)

## Notes

### Version-Specific Documentation Strategy

For features only available in specific CARLA versions:
1. Use `#[cfg(carla_0916)]` attribute on types/methods
2. Add "**Available in CARLA 0.9.16+ only**" note in docs
3. Link to version-specific Python API docs (e.g., `/en/0.9.16/`)

### Python API URL Format

Standard format for Python API links:
- Types: `https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TypeName`
- Methods: `https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TypeName.method_name`
- Enums: `https://carla.readthedocs.io/en/0.9.16/python_api/#carla.EnumName`

### Documentation Testing

All examples in documentation should:
- Use `no_run` attribute (require CARLA simulator)
- Be syntactically valid (checked by `cargo test --doc`)
- Follow the project's example conventions

### Maintenance

This document should be updated:
- When new types/methods are added to the crate
- When Python API documentation structure changes
- When completing each phase (mark checkboxes)
- Quarterly to review progress and adjust priorities
