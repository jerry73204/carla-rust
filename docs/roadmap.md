# CARLA Rust API Development Roadmap

This document outlines the planned development of missing Rust APIs for the CARLA client library. Work is organized into phases, with each phase including specific implementation tasks and comprehensive test coverage.

## Overview

The carla-rust library currently provides core functionality for:
- Basic actor management (spawn, destroy, get/set transforms)
- Vehicle control and physics
- Sensor data collection (camera, lidar, radar, GNSS, IMU)
- Traffic Manager integration
- Map and waypoint navigation
- Traffic lights and signs
- Light management

This roadmap focuses on filling gaps to achieve feature parity with the C++ client library.

**Total Phases:** 14 (Phase 0 - Phase 13)
**Estimated Total Effort:** ~27-35 weeks
**Note:** All work items are tracked with checkboxes for easy progress monitoring.

## Implementation Priority

**⚠️ IMPORTANT: API Implementation First, Examples Second**

Development must follow this priority order:

### Core API Development (Phases 1-9) - **HIGH PRIORITY**
Focus on implementing missing Rust APIs before creating examples:
- **Phase 1-9**: Core APIs, sensors, recording, batch operations, etc.
- These phases provide the foundation for all examples
- Must achieve substantial progress before Phase 10+

## Development Approach

**FFI and Rust API Work Are Done Together**

Each API feature requires work in both `carla-sys` (FFI bindings) and `carla` (Rust API). These are NOT separate tasks to be deferred - they must be completed together to expose each API:

### Implementation Pattern

Every API work item follows this structure:

1. **FFI Work (carla-sys):**
   - Add C++ header includes to `carla-sys/src/ffi.rs`
   - Add types to autocxx `generate!` block
   - Add methods to autocxx `safety!` block
   - Test FFI bindings compile for all supported CARLA versions (0.9.14, 0.9.15, 0.9.16)
   - Handle version-specific features with `#[cfg(carla_09XX)]` attributes

2. **Rust API (carla):**
   - Create safe Rust wrapper types
   - Implement idiomatic methods with proper error handling
   - Add extension traits where appropriate
   - Export new types through module hierarchy
   - Document public API with examples

3. **Tests:**
   - Unit tests for data structures and conversions
   - Integration tests with running simulator (marked with `#[serial]` and `#[ignore]`)
   - Version-specific tests with appropriate `#[cfg]` attributes

### Work Items in Phases

All work items in Phases 1-9 now include explicit FFI and Rust API breakdowns. This makes it clear that:
- FFI work is NOT deferred - it's part of completing the feature
- Both layers must be implemented to consider the API "complete"
- Version compatibility is handled during FFI work

### What IS Deferred

Only features that require **external dependencies** (not just FFI work) are deferred:
- RSS Sensor (requires Intel RSS library integration)
- Phase 13 Specialized Examples (require external services/SDKs)

### Example Implementation (Phases 10-13) - **DEPENDENT ON APIs**
Examples can only be implemented after their required APIs are available:
- **Phase 10**: Requires APIs from Phases 3, 4 (Recording, Physics, Weather)
- **Phase 11**: Requires APIs from Phases 5, 6 (Batch, Sensors, Walker AI)
- **Phase 12**: Requires APIs from Phase 2 (Debug utilities)
- **Phase 13**: Deferred/documentation only

**Recommended Approach:**
1. Complete Phases 1-6 (core APIs) before starting Phase 10 examples
2. Implement high-priority APIs from each phase first
3. Create examples as APIs become available to validate functionality
4. Use Phase 0 examples (already complete) as templates

### Phase Dependency Matrix

| Example Phase              | Required API Phases                                                             | Minimum Progress Needed      |
|----------------------------|---------------------------------------------------------------------------------|------------------------------|
| Phase 10 (Simple Examples) | Phase 3 (Recording)<br>Phase 4 (Vehicle Physics)<br>Weather API                 | 80% completion               |
| Phase 11 (Intermediate)    | Phase 5 (Batch Operations)<br>Phase 6 (Sensor Features)<br>Walker AI Controller | 80% completion               |
| Phase 12 (Advanced)        | Phase 2 (Debug Utilities)<br>All sensor APIs<br>UI framework integration        | 90% completion               |
| Phase 13 (Specialized)     | N/A                                                                             | Deferred - no implementation |

**Critical Path for Basic Examples (Phase 10):**
1. Phase 3: Recording and Playback → Enables recording examples
2. Phase 4: Advanced Vehicle Features → Enables physics examples
3. Weather API (simple) → Enables weather example
4. Spectator API → Enables vehicle gallery

**Critical Path for Traffic Generation (Phase 11):**
1. Phase 5: Batch Operations → Enables multi-actor spawning
2. Walker AI Controller (Phase 1 continuation) → Enables walker AI
3. Traffic Manager enhancements → Enables traffic scenarios

---

## Phase 0: Cargo Examples for Demonstration

**Priority:** Critical
**Estimated Effort:** 1 week
**Status:** ✅ **COMPLETE**

### Work Items

- [x] **CARLA Simulator Setup**
  - Download and install CARLA simulator (0.9.14, 0.9.15, 0.9.16)
  - Configure simulator launch scripts
  - Document system requirements and dependencies
  - Create helper scripts for version switching

- [x] **Basic Examples**
  - Files: `carla/examples/*.rs`
  - Create self-contained examples demonstrating core functionality
  - Examples do not require shared test utilities
  - Each example includes clear documentation and minimal error handling

- [x] **Example Categories**
  - **Basic**: Connection, world info, blueprints
  - **Vehicles**: Spawn, control, transforms, attributes
  - **Walkers**: Spawn, control, movement directions

- [x] **Documentation**
  - File: `carla/examples/README.md`
  - Document how to run examples
  - Explain simulator setup requirements
  - Provide troubleshooting guide
  - List all available examples with descriptions

### Examples Created

#### Basic Examples
- ✅ `connect.rs` - Connect to CARLA and get server version
- ✅ `world_info.rs` - Get world information (map, spawn points, actors)
- ✅ `blueprints.rs` - Query and filter blueprint library

#### Vehicle Examples
- ✅ `spawn_vehicle.rs` - Spawn single vehicle
- ✅ `multiple_vehicles.rs` - Spawn multiple vehicles
- ✅ `vehicle_transform.rs` - Get vehicle transforms and location
- ✅ `vehicle_attributes.rs` - Access vehicle attributes

### Example Usage Pattern

```rust
// Example: carla/examples/spawn_vehicle.rs
use carla::client::{ActorBase, Client};

fn main() {
    let client = Client::connect("localhost", 2000, None)
        .expect("Failed to connect to CARLA simulator");

    let world = client.world();
    let blueprint_library = world.blueprint_library();

    let vehicle_bp = blueprint_library
        .find("vehicle.tesla.model3")
        .expect("Failed to find Tesla Model 3 blueprint");

    let spawn_points = world.map().recommended_spawn_points();
    let spawn_point = spawn_points.get(0).expect("No spawn points available");

    let vehicle = world
        .spawn_actor(&vehicle_bp, &spawn_point)
        .expect("Failed to spawn vehicle");

    println!("Vehicle spawned: ID {}", vehicle.id());
}
```

Run with: `cargo run --example spawn_vehicle`

---

## Phase 1: Walker/Pedestrian Control

**Priority:** High
**Estimated Effort:** 3-4 weeks (includes FFI work)
**Status:** ✅ **COMPLETE** (Oct 28, 2025)
- Core functionality: 100% complete
- Advanced features: 60% complete (40% deferred due to FFI limitations)
- Examples: 2 comprehensive examples passing all tests

### Work Items

- [x] **Walker Actor Type**
  - File: `carla/src/client/walker.rs`
  - Implement `Walker` struct wrapping `carla::client::Walker`
  - Add Walker-specific control methods

- [x] **WalkerAIController** (Partial - 3/5 methods implemented)
  - **FFI Work (carla-sys):** ✅ **COMPLETE**
    - File: `carla-sys/src/bindings.rs` - Added WalkerAIController header
    - File: `carla-sys/csrc/carla_rust/client/walker_ai_controller.hpp` - Created FFI wrapper
    - Generated autocxx bindings for `FfiWalkerAIController`
    - Verified compilation on CARLA 0.9.16
  - **Rust API (carla):** 🟡 **PARTIAL** (60% complete)
    - File: `carla/src/client/walker_ai_controller.rs` - Created ✅
    - Struct `WalkerAIController` wrapping FFI type ✅
    - Implemented methods:
      - `start(&self)` - Start AI control ✅
      - `stop(&self)` - Stop AI control ✅
      - `set_max_speed(&self, speed: f32)` - Set walking speed (m/s) ✅
      - `go_to_location()` - Deferred (autocxx const reference parameter issues)
      - `get_random_location()` - Deferred (boost::optional FFI wrapper needed)
    - Added to `carla/src/client.rs` exports ✅
    - Module exported in `carla/src/client.rs` ✅
  - **Tests:** ✅ **COMPLETE**
    - Validated via `walker_integration_demo.rs` example
    - All core methods (start/stop/set_max_speed) verified working
    - AI controller attachment to walker parent verified
  - **Notes:**
    - 2 methods deferred due to FFI limitations (can be addressed in future iteration)
    - Core functionality (start/stop/speed control) is fully operational
    - Example demonstrates all 3 implemented methods successfully

- [x] **WalkerControl RPC Type**
  - Re-exported from `carla-sys`
  - `WalkerControl` struct with:
    - `direction: Vector3D` - Walking direction
    - `speed: f32` - Walking speed
    - `jump: bool` - Jump flag

- [x] **WalkerBoneControl** (Partial - 3/5 methods implemented)
  - **FFI Work (carla-sys):** ✅ **COMPLETE**
    - File: `carla-sys/src/bindings.rs` - Added WalkerBoneControl headers
    - File: `carla-sys/csrc/carla_rust/client/walker.hpp` - Added bone control methods
    - Generated autocxx bindings for:
      - `carla::rpc::WalkerBoneControlIn`
      - `carla::rpc::WalkerBoneControlOut`
      - `carla::rpc::BoneTransformDataIn`
      - `carla::rpc::BoneTransformDataOut`
    - Added methods to FfiWalker wrapper:
      - `SetBonesTransform(const WalkerBoneControlIn&)` ✅
      - `GetBonesTransform()` → WalkerBoneControlOut ✅
      - `BlendPose(float)` ✅
    - Verified compilation on CARLA 0.9.16
  - **Rust API (carla):** 🟡 **PARTIAL** (60% complete)
    - File: `carla/src/rpc/walker_bone_control.rs` - Created ✅
    - Rust type definitions:
      - `BoneTransformDataIn` - Input bone transform ✅
      - `BoneTransformDataOut` - Output bone transform (struct only, no FFI conversion) ✅
      - `WalkerBoneControlIn` - Input collection ✅
      - `WalkerBoneControlOut` - Output collection (struct only, no FFI conversion) ✅
    - File: `carla/src/client/walker.rs` - Methods implemented:
      - `blend_pose(&self, blend: f32)` - Blend animation pose (0.0-1.0) ✅
      - `show_pose(&self)` - Show custom pose (blend=1.0) ✅
      - `hide_pose(&self)` - Hide custom pose (blend=0.0) ✅
      - `set_bones()` - Deferred (autocxx moveit wrapper complexity)
      - `get_bones_transform()` - Deferred (opaque type field access)
    - Module exported in `carla/src/rpc.rs` ✅
  - **Tests:** ✅ **COMPLETE**
    - Validated via two examples:
      - `walker_bone_control_demo.rs` - Demonstrates data structure creation
      - `walker_integration_demo.rs` - Demonstrates pose blending methods
    - All core methods (blend_pose/show_pose/hide_pose) verified working
  - **Notes:**
    - 2 methods deferred due to autocxx limitations with complex type conversions
    - Core pose blending functionality is fully operational
    - Advanced bone manipulation (set_bones/get_bones_transform) requires custom C++ wrappers
    - FFI bindings are complete and ready for future wrapper implementation

### Examples Created

#### Walker Examples (Phase 0)
- ✅ `spawn_walker.rs` - Spawn single walker/pedestrian
- ✅ `walker_control.rs` - Apply walker movement control (direction, speed)
- ✅ `walker_directions.rs` - Demonstrate different movement directions
- ✅ `multiple_walkers.rs` - Spawn multiple walkers

#### Walker Advanced Examples (Phase 1)
- ✅ `walker_bone_control_demo.rs` - Bone transform data structures and manipulation
  - Demonstrates BoneTransformDataIn and WalkerBoneControlIn usage
  - Shows bone naming conventions (crl_arm__L, crl_leg__R, etc.)
  - No simulator required (data structure demo)

- ✅ `walker_integration_demo.rs` - Comprehensive walker functionality integration
  - **Demo 1**: Walker spawning and lifecycle
  - **Demo 2**: Walker control (direction, speed, jump)
  - **Demo 3**: Pose blending (blend_pose, show_pose, hide_pose)
  - **Demo 4**: Walker location queries
  - **Demo 5**: WalkerAIController spawning with parent attachment
  - **Demo 6**: AI controller start/stop control
  - **Demo 7**: AI controller speed settings
  - **Demo 8**: Walker AI-controlled movement verification
  - Requires running CARLA simulator
  - All 8 demos pass successfully

---

## Phase 2: Debug and Visualization Utilities

**Priority:** Medium
**Estimated Effort:** 2-3 weeks (includes FFI work)
**Status:** ✅ **COMPLETE**

### Work Items

- [x] **DebugHelper**
  - **FFI Work (carla-sys):**
    - ✅ File: `carla-sys/csrc/carla_rust/client/debug_helper.hpp` (custom FFI wrapper using POD types)
    - ✅ File: `carla-sys/src/bindings.rs` (autocxx generation declarations)
    - ✅ Implemented all drawing methods using FFI-friendly POD types (FfiLocation, FfiColor, etc.)
    - ✅ Free functions approach (FfiDebugHelper_DrawPoint, etc.) to work around autocxx limitations
    - ✅ All methods verified to compile and link
  - **Rust API (carla):**
    - ✅ File: `carla/src/client/debug_helper.rs` (351 lines, fully documented)
    - ✅ All drawing methods implemented:
      - `draw_point(&self, location, size, color, life_time, persistent_lines)`
      - `draw_line(&self, begin, end, thickness, color, life_time, persistent_lines)`
      - `draw_arrow(&self, begin, end, thickness, arrow_size, color, life_time, persistent_lines)`
      - `draw_box(&self, bbox, rotation, thickness, color, life_time, persistent_lines)`
      - `draw_string(&self, location, text, draw_shadow, color, life_time, persistent_lines)`
    - ✅ Comprehensive documentation with examples for each method

- [x] **Color Type**
  - **FFI Work (carla-sys):**
    - ✅ Existing FfiColor POD type in `carla-sys/csrc/carla_rust/sensor/data/color.hpp`
    - ✅ Already generated in bindings.rs
  - **Rust API (carla):**
    - ✅ File: `carla/src/rpc/color.rs` (201 lines)
    - ✅ `Color` struct with `r, g, b, a` fields (u8 values)
    - ✅ Implemented `new()`, `new_rgba()` constructors
    - ✅ 8 predefined color constants: RED, GREEN, BLUE, YELLOW, CYAN, MAGENTA, WHITE, BLACK
    - ✅ `From<Color> for FfiColor` conversion
    - ✅ Comprehensive documentation

- [x] **World Debug Access**
  - **Rust API (carla):**
    - ✅ File: `carla/src/client/world.rs`
    - ✅ Added `debug(&mut self) -> DebugHelper` method (line 622)
    - ✅ C++ bridge in `carla-sys/csrc/carla_rust/client/world.hpp`: `MakeDebugHelper()` method

- [x] **Example Implementation**
  - ✅ File: `carla/examples/debug_visualization.rs` (220 lines)
  - ✅ Demonstrates all drawing features: points, lines, arrows, boxes, text
  - ✅ Uses all 8 predefined colors
  - ✅ Fully documented with usage instructions

- [x] **Geometry Convenience Methods**
  - ✅ Added `Location::new()` via `LocationExt` trait
  - ✅ Added `BoundingBox::new()` for simple construction

### Test Cases

#### Unit Tests
- `test_color_creation` - Create colors with RGBA values
- `test_color_constants` - Verify predefined color values
- `test_color_to_native` - Convert to C++ color type

#### Integration Tests
- `test_debug_draw_point` - Draw point and verify visibility
- `test_debug_draw_line` - Draw line between two points
- `test_debug_draw_arrow` - Draw arrow with proper direction
- `test_debug_draw_box` - Draw bounding box
- `test_debug_draw_string` - Draw text at location
- `test_debug_shapes_lifetime` - Verify shapes disappear after lifetime
- `test_debug_multiple_shapes` - Draw multiple shapes simultaneously
- `test_debug_shape_colors` - Verify shape colors are correct

---

## Phase 3: Recording and Playback

**Priority:** Medium
**Estimated Effort:** 2-3 weeks (includes FFI work)
**Status:** Not Started

### Work Items

- [ ] **Client Recording Methods**
  - **FFI Work (carla-sys):**
    - File: `carla-sys/src/ffi.rs`
    - Verify `carla::client::Client` recording methods are exposed:
      - `StartRecorder(std::string, bool)`
      - `StopRecorder()`
      - `ShowRecorderFileInfo(std::string, bool)` → returns std::string
      - `ShowRecorderCollisions(std::string, char, char)` → returns std::string
      - `ShowRecorderActorsBlocked(std::string, float, float)` → returns std::string
    - Add to `safety!` block if not already present
    - Test across all CARLA versions
  - **Rust API (carla):**
    - File: `carla/src/client/carla_client.rs`
    - Implement recording methods:
      - `start_recorder(&self, filename: &str, additional_data: bool)` - Start recording
      - `stop_recorder(&self)` - Stop recording
      - `show_recorder_file_info(&self, filename: &str, show_all: bool) -> String` - Get recording info
      - `show_recorder_collisions(&self, filename: &str, category1: char, category2: char) -> String` - Query collisions
      - `show_recorder_actors_blocked(&self, filename: &str, min_time: f32, min_distance: f32) -> String` - Find blocked actors
  - **Tests:**
    - Integration tests: Record and verify file creation
    - Integration tests: Query collision and blocking data

- [ ] **Replay Methods**
  - **FFI Work (carla-sys):**
    - File: `carla-sys/src/ffi.rs`
    - Verify `carla::client::Client` replay methods are exposed:
      - `ReplayFile(std::string, float, float, uint32_t, bool)`
      - `StopReplayer(bool)`
      - `SetReplayerTimeFactor(float)`
      - `SetReplayerIgnoreHero(bool)`
      - `SetReplayerIgnoreSpectator(bool)` (0.9.15+ only)
    - Add to `safety!` block if not already present
    - Use `#[cfg(carla_0915)]` for SetReplayerIgnoreSpectator
  - **Rust API (carla):**
    - File: `carla/src/client/carla_client.rs`
    - Implement replay methods:
      - `replay_file(&self, filename: &str, start_time: f32, duration: f32, follow_id: u32, replay_sensors: bool)` - Replay recording
      - `stop_replayer(&self, keep_actors: bool)` - Stop replay
      - `set_replayer_time_factor(&self, time_factor: f32)` - Control replay speed
      - `set_replayer_ignore_hero(&self, ignore_hero: bool)` - Ignore hero vehicle in replay
      - `#[cfg(carla_0915)] set_replayer_ignore_spectator(&self, ignore: bool)` - Ignore spectator (0.9.15+)
  - **Tests:**
    - Integration tests: Replay and verify actor positions
    - Integration tests: Time factor affects replay speed

- [ ] **Recorder Info Types**
  - **FFI Work (carla-sys):**
    - Recording info is returned as strings from C++ API
    - No additional FFI types needed (parsed from string in Rust)
  - **Rust API (carla):**
    - File: `carla/src/rpc/recorder_info.rs`
    - Parse recorder info strings into structured types
    - `RecorderFileInfo` - Parsed file metadata
    - `RecorderCollision` - Collision event data
    - `RecorderActorBlocked` - Blocked actor data
  - **Tests:**
    - Unit tests: Parse recorder info strings

### Test Cases

#### Unit Tests
- `test_recorder_filename_validation` - Validate recording filenames
- `test_replay_time_factor_bounds` - Validate time factor range

#### Integration Tests
- `test_start_stop_recorder` - Start and stop recording
- `test_record_simple_scenario` - Record vehicle movement
- `test_replay_recorded_scenario` - Replay recorded scenario
- `test_replay_with_follow_camera` - Follow specific actor during replay
- `test_replay_time_factor` - Speed up/slow down replay
- `test_recorder_file_info` - Get recording metadata
- `test_recorder_collision_query` - Query recorded collisions
- `test_recorder_blocked_actors` - Find actors stuck during recording
- `test_replay_without_sensors` - Replay without sensor data
- `test_stop_replayer_keep_actors` - Stop replay, keep spawned actors

---

## Phase 4: Advanced Vehicle Features (0.9.14+)

**Priority:** Low
**Estimated Effort:** 2-3 weeks (includes FFI work + version handling)
**Status:** ✅ Complete

### Work Items

- [x] **Ackermann Control** (0.9.14+)
  - **FFI Work (carla-sys):**
    - File: `carla-sys/src/ffi.rs`
    - Verify `carla::rpc::AckermannControl` is exposed (should already exist)
    - Verify `carla::rpc::AckermannControllerSettings` is exposed
    - Add Vehicle methods to `safety!` block:
      - `ApplyAckermannControl(AckermannControl)`
      - `GetAckermannControllerSettings()` → returns AckermannControllerSettings
      - `ApplyAckermannControllerSettings(AckermannControllerSettings)`
    - Test on 0.9.14, 0.9.15, 0.9.16
  - **Rust API (carla):**
    - File: `carla/src/rpc/ackermann_control.rs` - Verify existing implementation
    - File: `carla/src/client/vehicle.rs` - Add methods:
      - `apply_ackermann_control(&mut self, control: &AckermannControl)`
      - `get_ackermann_controller_settings(&self) -> AckermannControllerSettings`
      - `apply_ackermann_controller_settings(&mut self, settings: &AckermannControllerSettings)`
  - **Tests:**
    - Integration tests: Apply Ackermann control and verify steering

- [x] **Vehicle Failure State** (0.9.14+)
  - **FFI Work (carla-sys):** ✅ Complete
    - File: `carla-sys/src/ffi.rs`
    - Add `#include "carla/rpc/VehicleFailureState.h"` to includes
    - Add to `generate!` block: `carla::rpc::VehicleFailureState`
    - Add Vehicle method to `safety!` block: `GetFailureState()` → returns VehicleFailureState
  - **Rust API (carla):** ✅ Complete
    - File: `carla/src/rpc/vehicle_failure_state.rs`
    - Re-export C++ enum `VehicleFailureState`: None, Rollover, Engine, TirePuncture
    - File: `carla/src/client/vehicle.rs` - Added method:
      - `failure_state(&self) -> VehicleFailureState`
  - **Tests:** N/A (requires simulator)

- [x] **Vehicle Telemetry** (0.9.16+ only)
  - **FFI Work (carla-sys):** ✅ Complete
    - File: `carla-sys/src/bindings.rs`
    - Added `#[cfg(carla_0916)]` gated includes:
      - `#include "carla/rpc/VehicleTelemetryData.h"`
      - `#include "carla/rpc/WheelTelemetryData.h"`
    - Created FFI wrapper `FfiVehicleTelemetryData` for opaque type handling
    - Added to `generate!` block: `carla_rust::rpc::FfiVehicleTelemetryData`
    - Added Vehicle method: `GetTelemetryData()` → returns FfiVehicleTelemetryData
  - **Rust API (carla):** ✅ Complete
    - File: `carla/src/rpc/vehicle_telemetry_data.rs`
    - `#[cfg(carla_0916)]` gated structs: `VehicleTelemetryData`, `WheelTelemetryData`
    - File: `carla/src/client/vehicle.rs` - Added method:
      - `#[cfg(carla_0916)] telemetry_data(&self) -> VehicleTelemetryData`
  - **Tests:** N/A (requires simulator)
  - **Example:** `carla/examples/vehicle_advanced_features.rs`

- [x] **Wheel Pitch Control** (0.9.16+ only)
  - **FFI Work (carla-sys):** ✅ Complete (methods already exposed)
    - File: `carla-sys/csrc/carla_rust/client/vehicle.hpp`
    - Methods: `SetWheelPitchAngle()`, `GetWheelPitchAngle()`, `RestorePhysXPhysics()`
  - **Rust API (carla):** ✅ Complete
    - File: `carla/src/client/vehicle.rs` - Added methods:
      - `#[cfg(carla_0916)] set_wheel_pitch_angle(&self, wheel: VehicleWheelLocation, degrees: f32)`
      - `#[cfg(carla_0916)] wheel_pitch_angle(&self, wheel: VehicleWheelLocation) -> f32`
      - `#[cfg(carla_0916)] restore_phys_x_physics(&self)`
  - **Tests:** N/A (requires simulator)
  - **Example:** `carla/examples/vehicle_advanced_features.rs`
  - **Note:** `GetVehicleBoneWorldTransforms()` deferred - requires autocxx support for `std::vector<Transform>`

- [x] **Vehicle Doors** (0.9.13+)
  - **FFI Work (carla-sys):** ✅ Complete (methods already exposed)
    - File: `carla-sys/csrc/carla_rust/client/vehicle.hpp`
    - `carla::rpc::VehicleDoor` enum already exposed
    - Methods: `OpenDoor()`, `CloseDoor()`
  - **Rust API (carla):** ✅ Complete
    - File: `carla/src/rpc.rs` - Re-export VehicleDoor enum
    - File: `carla/src/client/vehicle.rs` - Added methods:
      - `open_door(&self, door: VehicleDoor)`
      - `close_door(&self, door: VehicleDoor)`
  - **Tests:** N/A (requires simulator)
  - **Example:** `carla/examples/vehicle_advanced_features.rs`

### Test Cases

#### Unit Tests
- `test_ackermann_control_creation` - Create Ackermann control
- `test_ackermann_settings_validation` - Validate controller settings
- `test_vehicle_failure_state_enum` - Verify failure state values
- `test_wheel_location_enum` - Verify wheel location enum

#### Integration Tests
- `test_apply_ackermann_control` - Apply Ackermann steering to vehicle
- `test_ackermann_controller_settings` - Get and set controller settings
- `test_vehicle_failure_detection` - Trigger and detect rollover failure
- `test_vehicle_telemetry_0916` - Get telemetry data (0.9.16 only)
- `test_wheel_pitch_control_0916` - Control wheel pitch (0.9.16 only)
- `test_vehicle_bone_transforms_0916` - Get bone transforms (0.9.16 only)
- `test_vehicle_open_close_doors` - Open and close vehicle doors
- `test_vehicle_door_physics` - Verify door physics simulation

---

## Phase 5: Batch Operations and Commands

**Priority:** Medium
**Estimated Effort:** 3 weeks
**Status:** Not Started

### Work Items

- [ ] **Command System**
  - File: `carla/src/rpc/command.rs`
  - Enum `Command` with variants:
    - `SpawnActor` - Spawn actor with blueprint
    - `DestroyActor` - Remove actor
    - `ApplyVehicleControl` - Apply vehicle control
    - `ApplyWalkerControl` - Apply walker control
    - `ApplyTransform` - Set actor transform
    - `ApplyWalkerState` - Set walker state
    - `ApplyTargetVelocity` - Set target velocity
    - `ApplyTargetAngularVelocity` - Set angular velocity
    - `ApplyImpulse` - Apply impulse
    - `ApplyForce` - Apply force
    - `ApplyAngularImpulse` - Apply angular impulse
    - `ApplyTorque` - Apply torque
    - `SetSimulatePhysics` - Enable/disable physics
    - `SetEnableGravity` - Enable/disable gravity
    - `SetAutopilot` - Enable/disable autopilot
    - `ShowDebugTelemetry` - Enable/disable telemetry
    - `SetVehicleLightState` - Set vehicle lights
    - `ApplyLocation` - Set location (0.9.14+)
    - `ApplyVehiclePhysicsControl` - Set physics control (0.9.12+)
    - `SetTrafficLightState` - Set traffic light state (0.9.14+)
    - `ConsoleCommand` - Execute console command (0.9.14+)

- [ ] **Batch Processing**
  - File: `carla/src/client/carla_client.rs`
  - Methods:
    - `apply_batch(commands)` - Apply commands in batch
    - `apply_batch_sync(commands, due_tick_cputime)` - Apply batch synchronously

- [ ] **Command Response**
  - File: `carla/src/rpc/command_response.rs`
  - Response type indicating success/failure and actor ID for spawns

### Test Cases

#### Unit Tests
- `test_command_creation` - Create each command type
- `test_command_serialization` - Serialize commands for RPC
- `test_command_response_parsing` - Parse command responses

#### Integration Tests
- `test_batch_spawn_actors` - Spawn multiple actors in one batch
- `test_batch_destroy_actors` - Destroy multiple actors in batch
- `test_batch_mixed_commands` - Mix spawn, control, and destroy commands
- `test_batch_sync_vs_async` - Compare sync and async batch execution
- `test_batch_spawn_response` - Verify spawn command returns actor IDs
- `test_batch_error_handling` - Handle failed commands in batch
- `test_batch_vehicle_control` - Control multiple vehicles in batch
- `test_batch_physics_operations` - Batch physics enable/disable
- `test_batch_large_scale` - Process 100+ commands in single batch

---

## Phase 6: Advanced Sensor Features

**Priority:** Medium
**Estimated Effort:** 2 weeks
**Status:** Not Started

### Work Items

- [ ] **DVS Camera Sensor** (0.9.10+)
  - File: `carla/src/sensor/data/dvs_event_array.rs`
  - Dynamic Vision Sensor for event-based vision
  - `DVSEvent` struct with timestamp, x, y, polarity

- [ ] **Optical Flow Camera** (0.9.12+)
  - File: `carla/src/sensor/data/optical_flow_image.rs`
  - 2-channel image with motion vectors
  - Convert to visualization format

- [ ] **Normals Sensor** (0.9.14+)
  - Already exists as camera sensor with normals view mode
  - Verify functionality

- [ ] **RSS Sensor** (0.9.7+) - **DEFERRED**
  - File: `carla/src/sensor/data/rss_response.rs`
  - Responsibility-Sensitive Safety sensor
  - **Deferred Reason:** Requires Intel RSS library integration (external dependency, not just FFI work)
  - Complex integration requiring:
    - Intel ad-rss-lib C++ library
    - Additional build dependencies
    - Extensive RSS-specific domain knowledge
  - **Note:** Unlike other items in this phase, this is legitimately deferred due to external dependencies, not FFI complexity

- [ ] **GBuffer Access** (0.9.14+)
  - Methods:
    - `Sensor::listen_to_gbuffer(gbuffer_id, callback)` - Access specific GBuffer
  - GBuffer types: SceneColor, SceneDepth, SceneStencil, GBufferA, etc.

### Test Cases

#### Unit Tests
- `test_dvs_event_creation` - Create DVS events
- `test_optical_flow_pixel_access` - Access optical flow pixels
- `test_gbuffer_id_enum` - Verify GBuffer type enum

#### Integration Tests
- `test_dvs_camera_events` - Capture DVS events
- `test_dvs_event_stream` - Process continuous event stream
- `test_optical_flow_capture` - Capture optical flow
- `test_optical_flow_visualization` - Convert to RGB for visualization
- `test_normals_sensor_capture` - Capture surface normals
- `test_gbuffer_access` - Access GBuffer textures
- `test_gbuffer_scene_depth` - Retrieve scene depth buffer
- `test_multiple_gbuffers` - Access multiple GBuffers simultaneously

---

## Phase 7: Advanced World Operations

**Priority:** Low
**Estimated Effort:** 2 weeks
**Status:** Not Started

### Work Items

- [ ] **Environment Objects**
  - File: `carla/src/client/world.rs`
  - Methods (0.9.11+):
    - `World::get_environment_objects(object_type)` - Get objects by type
    - `World::enable_environment_objects(env_objects, enable)` - Show/hide objects
  - Already has `EnvironmentObject` in RPC

- [ ] **Level Layers** (0.9.11+)
  - Methods:
    - `World::load_map_layer(map_layers)` - Load map layer
    - `World::unload_map_layer(map_layers)` - Unload map layer
  - Already has `MapLayer` enum in RPC

- [ ] **Freeze Traffic Lights** (0.9.10+)
  - Methods:
    - `World::freeze_all_traffic_lights(frozen)` - Freeze all traffic lights
    - `TrafficLight::reset_group()` - Reset traffic light group

- [ ] **Vehicles Light States** (0.9.10+)
  - Methods:
    - `World::get_vehicles_light_states()` - Get all vehicle light states at once
  - Batch operation for performance

- [ ] **Load World If Different** (0.9.15+)
  - Methods:
    - `Client::load_world_if_different(map_name)` - Conditional map loading

### Test Cases

#### Unit Tests
- `test_map_layer_enum` - Verify map layer enum values
- `test_environment_object_type_filter` - Test object type filtering

#### Integration Tests
- `test_get_environment_objects` - Query environment objects
- `test_enable_disable_environment_objects` - Show and hide objects
- `test_load_unload_map_layer` - Load and unload opt layers
- `test_freeze_traffic_lights` - Freeze all traffic lights
- `test_unfreeze_traffic_lights` - Unfreeze and verify state changes
- `test_traffic_light_reset_group` - Reset synchronized traffic lights
- `test_get_all_vehicle_lights` - Batch query vehicle light states
- `test_load_world_if_different` - Avoid unnecessary map reload
- `test_load_world_if_different_forces_load` - Force load when different

---

## Phase 8: Navigation and Path Planning

**Priority:** Low
**Estimated Effort:** 1 week
**Status:** Not Started

### Work Items

- [ ] **Waypoint Generation Methods**
  - Already mostly implemented in `carla/src/client/waypoint.rs`
  - Verify completeness of:
    - `Waypoint::next(distance)` - Get next waypoints
    - `Waypoint::previous(distance)` - Get previous waypoints
    - `Waypoint::get_left_lane()` / `get_right_lane()`

- [ ] **Junction Waypoint Navigation**
  - File: `carla/src/client/junction.rs`
  - Already exists, verify:
    - `Junction::get_waypoints(lane_type)` - Get junction waypoints

- [ ] **Map Topology**
  - File: `carla/src/client/map.rs`
  - Verify:
    - `Map::get_topology()` - Get road network topology
    - Returns pairs of waypoints representing road segments

### Test Cases

#### Unit Tests
- `test_waypoint_distance_calculation` - Verify distance between waypoints

#### Integration Tests
- `test_waypoint_next_generation` - Generate next waypoints
- `test_waypoint_previous_generation` - Generate previous waypoints
- `test_waypoint_lane_change` - Change lanes using waypoints
- `test_waypoint_junction_navigation` - Navigate through junctions
- `test_map_topology` - Get complete road network topology
- `test_topology_connectivity` - Verify all road connections

---

## Phase 9: Additional Utilities and Refinements

**Priority:** Low
**Estimated Effort:** 2 weeks
**Status:** Not Started

### Work Items

- [ ] **Improved Error Handling**
  - Better error types and messages
  - Custom error enum for CARLA-specific errors
  - Timeout handling and recovery

- [ ] **Async/Await Support**
  - Investigate async sensor callbacks
  - Async world tick and operations
  - Tokio integration examples

- [ ] **Example Programs**
  - File: `carla/examples/`
  - Comprehensive examples for each major feature:
    - `spawn_vehicles.rs` - Basic vehicle spawning
    - `vehicle_control.rs` - Manual vehicle control
    - `sensor_tutorial.rs` - Sensor setup and data collection
    - `traffic_manager_demo.rs` - Traffic Manager usage
    - `walker_navigation.rs` - Pedestrian control
    - `debug_drawing.rs` - Debug visualization
    - `recording_playback.rs` - Record and replay
    - `batch_operations.rs` - Efficient batch commands

- [ ] **Documentation Improvements**
  - Complete API documentation for all public types
  - Tutorial series in docs/
  - Migration guides from C++/Python

- [ ] **Performance Profiling**
  - Benchmark common operations
  - Optimize critical paths
  - Memory usage analysis

### Test Cases

#### Unit Tests
- `test_error_types` - Verify error type conversions
- `test_timeout_handling` - Handle timeout errors gracefully

#### Integration Tests
- `test_all_examples_compile` - Verify all examples compile
- `test_example_spawn_vehicles` - Run spawn_vehicles example
- `test_example_sensor_tutorial` - Run sensor tutorial
- `test_concurrent_operations` - Multiple clients/threads
- `test_high_frequency_operations` - Stress test at high rates

#### Performance Tests
- `bench_spawn_destroy_cycle` - Measure spawn/destroy performance
- `bench_sensor_throughput` - Measure sensor data throughput
- `bench_batch_vs_individual` - Compare batch vs individual commands
- `bench_memory_usage` - Monitor memory during long sessions

---

## Phase 10: Simple Example Implementations

**Priority:** Medium (Dependent on Phases 3-4)
**Estimated Effort:** 2-3 weeks
**Status:** Not Started
**⚠️ Prerequisites:** Must complete Phase 3 (Recording) and Phase 4 (Vehicle Physics) APIs first

### Overview

Implement Rust equivalents of simple Python examples that demonstrate single features or basic combinations. These examples serve as introductory material and test basic API functionality.

**Note**: These examples cannot be implemented until their prerequisite APIs are available. See "Implementation Priority" section above.

### Prerequisites

- Phase 3: Recording and Playback APIs
- Phase 4: Advanced Vehicle Features (physics control)
- Weather API implementation
- Spectator camera access

### Work Items

- [ ] **Core Example Infrastructure**
  - File: `carla/examples/README.md`
  - Update with new examples
  - Document dependencies and run instructions
  - Add troubleshooting section

- [ ] **Tutorial Example** (`tutorial.rs`)
  - Spawn vehicle with autopilot
  - Attach camera sensor
  - Save sensor data to disk
  - Demonstrates: Basic workflow, sensors, autopilot

- [ ] **Vehicle Gallery** (`vehicle_gallery.rs`)
  - Iterate through all vehicle blueprints
  - Spawn each vehicle temporarily
  - Rotate spectator camera around vehicle
  - Demonstrates: Blueprint iteration, spectator control

- [ ] **Dynamic Weather** (`dynamic_weather.rs`)
  - Implement weather parameter control
  - Animate sun position over time
  - Simulate storm cycles
  - Demonstrates: Weather system, world tick

- [ ] **Vehicle Physics** (`vehicle_physics.rs`)
  - Apply impulse and force to vehicles
  - Compare physics effects
  - Use synchronous mode for precision
  - Demonstrates: Physics control, synchronous mode

- [ ] **Recording Examples** (3 files)
  - `start_recording.rs` - Start/stop recorder
  - `start_replaying.rs` - Replay recordings
  - `show_recorder_info.rs` - Query recording metadata
  - Demonstrates: Recording API, metadata queries

### Test Cases

#### Integration Tests
- `test_tutorial_example` - Run tutorial example end-to-end
- `test_weather_animation` - Verify weather parameter changes
- `test_physics_impulse` - Apply impulse and measure effect
- `test_recording_playback` - Record and replay scenario
- `test_recorder_queries` - Query collision and blocked actor data

### Dependencies

**New APIs Needed**:
```rust
// Weather control
impl World {
    pub fn get_weather(&self) -> WeatherParameters;
    pub fn set_weather(&mut self, weather: &WeatherParameters);
}

// Spectator access
impl World {
    pub fn get_spectator(&self) -> Actor;
}
```

**See**: `docs/example_implementation_plan.md` for detailed specifications

---

## Phase 11: Intermediate Example Implementations

**Priority:** Medium (Dependent on Phases 5-6)
**Estimated Effort:** 3-4 weeks
**Status:** Not Started
**⚠️ Prerequisites:** Must complete Phase 5 (Batch Operations) and Phase 6 (Sensor Features) APIs first

### Overview

Implement examples that combine multiple features and demonstrate more complex scenarios. These examples are essential for understanding real-world usage patterns.

**Note**: These examples cannot be implemented until their prerequisite APIs are available. See "Implementation Priority" section above.

### Prerequisites

- Phase 5: Batch Operations and Commands
- Phase 6: Advanced Sensor Features
- Traffic Manager enhancements
- Walker AI Controller implementation
- Synchronous mode utilities

### Work Items

- [ ] **Generate Traffic** (`generate_traffic.rs`)
  - Batch spawn vehicles and walkers
  - Configure Traffic Manager
  - Spawn walker AI controllers
  - Demonstrates: Batch commands, Traffic Manager, walker AI

- [ ] **Sensor Synchronization** (`sensor_synchronization.rs`)
  - Spawn multiple sensors (cameras, lidar, radar)
  - Synchronize data collection
  - Use queue-based collection pattern
  - Demonstrates: Multi-sensor setup, synchronization

- [ ] **Synchronous Mode** (`synchronous_mode.rs`)
  - Create sync mode context manager
  - Spawn vehicle with RGB and semantic cameras
  - Display camera feeds (simple renderer)
  - Demonstrates: Synchronous simulation, sensor sync

- [ ] **Automatic Control** (`automatic_control.rs`)
  - Enable autopilot on vehicle
  - Display HUD with vehicle info
  - Simple keyboard controls for camera
  - Demonstrates: Autopilot, basic UI

- [ ] **Sensor Coordination** (2 examples)
  - `lidar_to_camera.rs` - Transform lidar points to camera view
  - `visualize_multiple_sensors.rs` - Multi-sensor display
  - Demonstrates: Coordinate transformations, sensor fusion

- [ ] **GBuffer Access** (`tutorial_gbuffer.rs`)
  - Access GBuffer textures
  - Save scene depth, normals, etc.
  - Demonstrates: Advanced rendering features

### Test Cases

#### Integration Tests
- `test_batch_traffic_spawn` - Spawn 50 vehicles + 20 walkers
- `test_sensor_synchronization` - Verify all sensors tick together
- `test_sync_mode_context` - Context manager lifecycle
- `test_autopilot_navigation` - Vehicle navigates autonomously
- `test_lidar_camera_transform` - Coordinate transformation accuracy

### Dependencies

**New APIs Needed**:
```rust
// Walker AI Controller (Phase 1)
impl WalkerAIController {
    pub fn start(&mut self);
    pub fn stop(&mut self);
    pub fn go_to_location(&mut self, location: &Location);
    pub fn set_max_speed(&mut self, speed: f32);
}

// Random navigation
impl World {
    pub fn get_random_location_from_navigation(&self) -> Option<Location>;
    pub fn set_pedestrians_cross_factor(&mut self, percentage: f32);
}
```

**External Crates**:
- `winit` - Window creation and events (for UI examples)
- `pixels` - Simple pixel buffer rendering

**See**: `docs/example_implementation_plan.md` for implementation patterns

---

## Phase 12: Advanced Example Implementations

**Priority:** Low (Dependent on Phase 2 + UI frameworks)
**Estimated Effort:** 4-5 weeks
**Status:** Not Started
**⚠️ Prerequisites:** Must complete Phase 2 (Debug utilities) and integrate UI frameworks first

### Overview

Implement complex examples with advanced features, external dependencies, and sophisticated interactions. These demonstrate professional-grade CARLA applications.

**Note**: These examples cannot be implemented until their prerequisite APIs are available. See "Implementation Priority" section above.

### Prerequisites

- Phase 2: Debug and Visualization Utilities
- All sensor APIs complete
- UI framework integration (winit + pixels)
- Math library integration (nalgebra)
- Image processing utilities

### Work Items

- [ ] **Manual Control** (`manual_control.rs`)
  - Full interactive vehicle control (WASD keys)
  - Real-time HUD display
  - Camera switching and controls
  - Demonstrates: Complete interactive demo, UI integration

- [ ] **Bounding Boxes** (2 examples)
  - `bounding_boxes.rs` - 2D/3D bbox generation and display
  - `client_bounding_boxes.rs` - Client-side bbox computation
  - Camera projection matrix implementation
  - Instance segmentation decoding
  - Demonstrates: Computer vision, projection math

- [ ] **Advanced Visualization** (3 examples)
  - `draw_skeleton.rs` - Walker skeleton visualization
  - `no_rendering_mode.rs` - Large-scale simulation (100+ vehicles)
  - `open3d_lidar.rs` - Lidar point cloud visualization (optional)
  - Demonstrates: Debug drawing, scalability, 3D viz

- [ ] **Hardware Input** (`manual_control_steeringwheel.rs`)
  - Steering wheel and pedal support
  - Force feedback (if available)
  - Demonstrates: Hardware integration

### Test Cases

#### Manual Testing Required
- Most examples in this phase require human interaction
- Create test checklist for manual verification
- Document expected behavior and controls

#### Automated Tests (Where Possible)
- `test_projection_matrix` - Verify camera projection math
- `test_bbox_computation` - 2D bbox from 3D coordinates
- `test_instance_segmentation_decode` - Decode actor IDs
- `test_large_scale_spawn` - 100 vehicles spawn successfully

### Dependencies

**External Crates**:
- **winit** (v0.29+) - Window and event handling
- **pixels** (v0.13+) - 2D rendering
- **nalgebra** (v0.32+) - Matrix and vector math
- **image** (v0.24+) - Image encoding/decoding
- **gilrs** (v0.10+) - Game controller/wheel input

**New Utilities**:
```rust
// Camera projection (new module)
pub mod camera_projection {
    pub fn build_projection_matrix(width: u32, height: u32, fov: f32) -> Matrix3<f32>;
    pub fn world_to_camera(point: Location, camera_transform: &Transform) -> Vector3<f32>;
    pub fn project_to_2d(point_3d: Vector3<f32>, k_matrix: &Matrix3<f32>) -> (f32, f32);
}

// Instance segmentation utilities
impl SemanticSegmentationImage {
    pub fn decode_instance_ids(&self) -> (Vec<u8>, Vec<u16>);
}
```

**See**: `docs/example_implementation_plan.md` for UI patterns and design guidelines

---

## Phase 13: Specialized Examples (Reference/Future)

**Priority:** Very Low (Deferred)
**Estimated Effort:** Varies by integration
**Status:** Documentation Only
**⚠️ Note:** No implementation planned - documentation reference only

### Overview

Document external integration examples from the Python repository. Most are deferred as they require external systems, proprietary SDKs, or are better served by separate integration crates.

**Note**: These examples are explicitly deferred. Implementation is not planned unless there is specific user demand.

### Examples (Reference Only)

- **InvertedAI Traffic** (`invertedai_traffic.py`)
  - Status: Deferred - requires InvertedAI API subscription
  - Integration point: Traffic generation via external AI

- **NVIDIA Cosmos Generation** (`carla_cosmos_gen.py`)
  - Status: Deferred - requires NVIDIA Cosmos SDK
  - Integration point: Synthetic data generation

- **V2X Communication** (`V2XDemo.py`, `test_addsecondvx.py`)
  - Status: Deferred - requires V2X protocol implementation
  - Integration point: Vehicle-to-vehicle/infrastructure communication

- **ROS2 Bridge** (`ros2/*`)
  - Status: Future - recommend separate `carla-ros2` crate
  - Integration point: ROS2 message conversion and topics

- **RSS Safety** (`rss/*`)
  - Status: Deferred - requires Intel RSS library bindings
  - Integration point: Safety validation and constraint checking

### Work Items

- [ ] **Integration Documentation**
  - Document FFI requirements for each integration
  - Identify Rust crate alternatives where available
  - Create integration guide for common patterns

- [ ] **External Crate Recommendations**
  - ROS2: `rclrs` (existing ROS2 Rust bindings)
  - V2X: Protocol-specific crates (when available)
  - RSS: Custom FFI bindings (complex undertaking)

### Rationale

These integrations are deferred because they:
1. Require external paid services (InvertedAI)
2. Require proprietary SDKs (NVIDIA Cosmos)
3. Are better served by dedicated integration crates (ROS2)
4. Require extensive additional FFI work (RSS)

**Recommendation**: Document integration patterns but don't implement unless there's specific user demand.

---

## Cross-Phase Testing

### Integration Test Scenarios

These tests verify interactions between multiple phases:

1. **Complete Autonomous Driving Scenario**
   - Spawn vehicle with sensors
   - Enable Traffic Manager
   - Navigate to destination
   - Handle traffic lights and pedestrians
   - Record entire session
   - Replay and verify behavior

2. **Multi-Agent Simulation**
   - Spawn 50 vehicles and 100 pedestrians
   - Control subset with AI
   - Monitor collisions and blocked actors
   - Use debug drawing for visualization
   - Batch operations for efficiency

3. **Sensor Data Collection Pipeline**
   - Spawn vehicle with RGB, depth, semantic, lidar
   - Navigate using autopilot
   - Collect and save sensor data
   - Verify data consistency across sensors
   - Test different weather and lighting conditions

4. **Dynamic World Manipulation**
   - Load map with layers
   - Spawn environment objects
   - Enable/disable layers at runtime
   - Freeze and manipulate traffic lights
   - Record and replay with different configurations

---

## Testing Infrastructure

### Test Organization

```
carla/
  tests/
    unit/              # Unit tests for individual components
      rpc/             # RPC type tests
      client/          # Client API tests
      sensor/          # Sensor data tests
    integration/       # Integration tests requiring simulator
      vehicle/         # Vehicle-related integration tests
      sensor/          # Sensor integration tests
      walker/          # Pedestrian integration tests
      world/           # World operations tests
      batch/           # Batch operations tests
    performance/       # Performance benchmarks
      spawn.rs
      sensor_throughput.rs
      batch_operations.rs
```

### Test Requirements

1. **Unit Tests**
   - No simulator required
   - Fast execution (< 1 second per test)
   - Test data structures, serialization, and pure logic

2. **Integration Tests**
   - Require running CARLA simulator
   - Use `#[ignore]` attribute for CI without simulator
   - Run with `cargo test -- --ignored` when simulator available
   - Test actual simulation behavior

3. **Performance Tests**
   - Benchmarks using criterion
   - Measure latency, throughput, memory
   - Track regressions over time

### Continuous Integration

- Unit tests run on every commit
- Integration tests run nightly with simulator
- Performance tests run weekly, track trends
- Test coverage target: >80% for unit tests

---

## Version Compatibility Strategy

All new features must consider version compatibility:

1. **Version-Specific Features**
   - Use `#[cfg(carla_0914)]`, `#[cfg(carla_0915)]`, `#[cfg(carla_0916)]` attributes
   - Document version requirements in API docs
   - Test across all supported versions (0.9.14, 0.9.15, 0.9.16)

2. **Deprecation Policy**
   - Features removed in newer CARLA versions get deprecation warnings
   - Maintain backward compatibility for at least one major version
   - Clear migration guides in documentation

3. **Testing Matrix**
   - Test each phase against all supported CARLA versions
   - Version-specific tests use appropriate `#[cfg]` attributes
   - CI runs tests for each version separately

---

## Success Criteria

Each phase is considered complete when:

1. All work items implemented and merged
2. Unit tests achieve >90% code coverage
3. Integration tests pass consistently
4. Documentation complete with examples
5. Code reviewed and approved
6. Tested across all supported CARLA versions

---

## References

- [CARLA Python API Documentation](https://carla.readthedocs.io/en/latest/python_api/)
- [CARLA Python Examples](https://github.com/carla-simulator/carla/tree/master/PythonAPI/examples)
- [CARLA C++ Client LibCarla Source](https://github.com/carla-simulator/carla/tree/master/LibCarla/source/carla/client)
- [CARLA CHANGELOG](https://github.com/carla-simulator/carla/blob/master/CHANGELOG.md)
- [carla-rust API Differences](./carla_api_differences.md)
- [Example Implementation Plan](./example_implementation_plan.md)
