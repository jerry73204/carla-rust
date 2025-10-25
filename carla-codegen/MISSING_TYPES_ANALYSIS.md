# Missing Types Analysis for CARLA Code Generation

This document provides a comprehensive analysis of types that are referenced in the generated code but are either missing or incorrectly referenced.

## Summary

After analyzing the generated code in `test_crate/`, we found several categories of issues:

1. **Incorrect Module Paths**: Types are generated but referenced with wrong module paths
2. **Missing Standard Library Imports**: Common types like `HashMap` not imported
3. **Undefined Types**: Some types are referenced but never generated
4. **Module Structure Mismatches**: Generated code expects modules that don't exist

## 1. Types That Exist But Are Referenced Incorrectly

These types ARE generated in `src/carla/` but are referenced with incorrect paths:

### Incorrect `crate::` References
- `ActorState` - Referenced as `crate::ActorState` instead of `crate::carla::ActorState`
- `TrafficLightState` - Referenced as `crate::TrafficLightState` instead of `crate::carla::TrafficLightState`
- `VehicleLightState` - Referenced as `crate::VehicleLightState` instead of `crate::carla::VehicleLightState`
- `WorldSnapshot` - Referenced as `crate::WorldSnapshot` instead of `crate::carla::WorldSnapshot`
- `WorldSettings` - Referenced as `crate::WorldSettings` instead of `crate::carla::WorldSettings`
- `Timestamp` - Referenced as `crate::Timestamp` instead of `crate::carla::Timestamp`
- And many more...

### Files Confirmed to Exist
The following files exist in `src/carla/`:
- `actor_state.rs` (defines `ActorState`)
- `traffic_light_state.rs` (defines `TrafficLightState`)
- `vehicle_light_state.rs` (defines `VehicleLightState`)
- `world_snapshot.rs`
- `world_settings.rs`
- `timestamp.rs`

## 2. Missing Standard Library Imports

### HashMap
- Used in multiple files without importing `std::collections::HashMap`
- Affected files:
  - `actor.rs:10` - `pub attributes: HashMap<String, String>`
  - `world.rs:72` - Return type `HashMap<String, String>`
  - And others

## 3. Types That Don't Exist Anywhere

### ActorOrId
- Referenced in:
  - `command/set_enable_gravity.rs:10` as `crate::actor::ActorOrId`
  - `command/show_debug_telemetry.rs:10` as `crate::actor::ActorOrId`
- This type is handled specially in the type resolver (`type_resolver.rs:254`) but never generated
- Should be a union type or enum representing either an Actor or an ID

### Missing Name Variations
Some types are referenced with different names than generated:
- `AckermannVehicleControl` referenced but file is `vehicle_ackermann_control.rs`
- `GBufferTextureID` referenced but file is `g_buffer_texture_id.rs`

## 4. Module Structure Issues

### Missing `road` Module
Multiple references to `crate::road::*` types:
- `crate::road::Waypoint`
- `crate::road::Junction`
- `crate::road::Map`
- `crate::road::Landmark`
- `crate::road::LaneMarking`

But all these types are actually generated in `src/carla/`:
- `waypoint.rs`
- `junction.rs`
- `map.rs`
- `landmark.rs`
- `lane_marking.rs`

### Missing `sensor` Submodules
References to nested sensor types:
- `crate::sensor::TextureColor`
- `crate::sensor::TextureFloatColor`

But these are actually in `src/carla/`:
- `texture_color.rs`
- `texture_float_color.rs`

## 5. Root Causes

1. **Type Mapping Confusion**: The code generator has type mappings that remap `carla.X` to different module paths (like `crate::road::X`), but then generates all types in the `carla` module anyway.

2. **Missing Type Generation**: `ActorOrId` is handled specially in the type resolver but no corresponding type is generated.

3. **Import Generation**: The code generator doesn't add necessary imports for standard library types.

4. **Module Path Resolution**: When referencing types, the generator uses mapped paths from config instead of where types are actually placed.

## 6. Recommended Fixes

### Immediate Fixes Needed

1. **Fix Type References**: 
   - Change all `crate::TypeName` to `crate::carla::TypeName`
   - Change all `crate::road::TypeName` to `crate::carla::TypeName`
   - Change all `crate::sensor::TypeName` to `crate::carla::TypeName`

2. **Generate Missing Types**:
   - Create `ActorOrId` enum or type alias
   - Ensure all referenced types are generated

3. **Add Missing Imports**:
   - Add `use std::collections::HashMap;` where needed
   - Add other standard library imports as needed

4. **Fix Name Mismatches**:
   - Ensure referenced type names match generated file names
   - Handle CamelCase vs snake_case conversions properly

### Long-term Solution

As documented in `doc/BUGS.md`, the best solution is to:
1. Remove the type mapping feature from the code generator
2. Generate all types in `crate::carla::*` namespace consistently
3. Let the hand-written API layer (`carla` and `carla-sys` crates) handle module organization

## 7. Statistics

- Total compilation errors: ~141
- Missing module references: ~30 instances of `could not find road`
- Missing type references: ~37 different types
- Missing imports: Multiple `HashMap` usage without import
- Files generated: 84 type files in `src/carla/`
- Files with errors: Most files have at least one reference error