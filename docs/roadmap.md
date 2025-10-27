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

**Total Phases:** 10 (Phase 0 - Phase 9)
**Estimated Total Effort:** ~18-21 weeks
**Note:** All work items are tracked with checkboxes for easy progress monitoring.

---

## Phase 0: Development Environment Setup

**Priority:** Critical
**Estimated Effort:** 1 week
**Status:** ✅ **COMPLETE**

### Work Items

- [x] **CARLA Simulator Setup**
  - Download and install CARLA simulator (0.9.14, 0.9.15, 0.9.16)
  - Configure simulator launch scripts
  - Document system requirements and dependencies
  - Create helper scripts for version switching

- [x] **Test Infrastructure for Exclusive Simulator Access**
  - File: `carla/tests/test_utils/simulator_lock.rs`
  - Implement test synchronization using file locks or mutex
  - Ensure only one integration test runs against simulator at a time
  - Add `#[serial]` attribute support using `serial_test` crate
  - Create test harness that manages simulator connection lifecycle

- [x] **Simulator Test Utilities**
  - File: `carla/tests/test_utils/mod.rs`
  - Helper functions:
    - `with_simulator<F>(test_fn: F)` - Run test with exclusive simulator access
    - `connect_to_simulator()` - Connect with retry logic
    - `cleanup_all_actors()` - Clean up test artifacts
    - `wait_for_simulator_ready()` - Wait for simulator initialization

- [x] **CI/CD Configuration**
  - Configure GitHub Actions or CI system
  - Set up simulator environment for integration tests
  - Add test matrix for all CARLA versions (0.9.14, 0.9.15, 0.9.16)
  - Configure test timeout and retry policies

- [x] **Documentation**
  - File: `docs/testing.md`
  - Document how to run integration tests
  - Explain simulator setup requirements
  - Provide troubleshooting guide
  - Add examples of exclusive test patterns

### Test Cases

#### Unit Tests
- ✅ `test_simulator_lock_acquisition` - Verify lock prevents concurrent access
- ✅ `test_simulator_lock_release` - Verify lock is released after test
- ✅ `test_simulator_lock_timeout` - Handle stuck lock scenarios
- ✅ `test_simulator_lock_retry_success` - Successful retry after lock release

#### Integration Tests
- ✅ `test_exclusive_simulator_access` - Verify only one test runs at a time
- ✅ `test_simulator_connection` - Connect to running simulator
- ✅ `test_simulator_reconnection` - Reconnect after disconnection
- ✅ `test_cleanup_between_tests` - Verify clean state between tests
- ✅ `test_simulator_ready_detection` - Detect when simulator is ready
- ✅ `test_with_simulator_helper` - Test with_simulator helper
- ✅ `test_with_simulator_client_helper` - Test with_simulator_client helper
- ✅ `test_vehicle_spawn` - Spawn single vehicle
- ✅ `test_multiple_vehicle_spawn` - Spawn multiple vehicles
- ✅ `test_vehicle_transform` - Get vehicle transform
- ✅ `test_vehicle_attributes` - Access vehicle attributes
- ✅ `test_blueprint_filtering` - Filter blueprints

### Example Test Pattern

```rust
use serial_test::serial;
use carla_test_utils::with_simulator;

#[test]
#[serial]  // Ensures exclusive access to simulator
#[ignore]  // Run only when simulator is available
fn test_vehicle_spawn() {
    with_simulator(|world| {
        // Test runs with exclusive simulator access
        let blueprint = world.get_blueprint_library()
            .find("vehicle.tesla.model3")
            .unwrap();
        let spawn_point = world.get_map().get_spawn_points()[0];
        let vehicle = world.spawn_actor(&blueprint, &spawn_point).unwrap();

        assert!(vehicle.is_alive());

        // Cleanup handled automatically by with_simulator
    });
}
```

---

## Phase 1: Walker/Pedestrian Control

**Priority:** High
**Estimated Effort:** 2-3 weeks
**Status:** Not Started

### Work Items

- [ ] **Walker Actor Type**
  - File: `carla/src/client/walker.rs`
  - Implement `Walker` struct wrapping `carla::client::Walker`
  - Add Walker-specific control methods
  - Add bone and animation control

- [ ] **WalkerAIController**
  - File: `carla/src/client/walker_ai_controller.rs`
  - Implement `WalkerAIController` wrapping C++ class
  - Methods:
    - `start()` - Start AI control
    - `stop()` - Stop AI control
    - `go_to_location(location)` - Navigate to target
    - `set_max_speed(speed)` - Set walking speed

- [ ] **WalkerControl RPC Type**
  - File: `carla/src/rpc/walker_control.rs`
  - `WalkerControl` struct with:
    - `direction: Vector3D` - Walking direction
    - `speed: f32` - Walking speed
    - `jump: bool` - Jump flag

- [ ] **WalkerBoneControl** (0.9.13+)
  - File: `carla/src/rpc/walker_bone_control.rs`
  - Bone manipulation for custom animations
  - Integration with `Walker::set_bones()` and `Walker::blend_pose()`

### Test Cases

#### Unit Tests
- `test_walker_control_creation` - Create WalkerControl with valid parameters
- `test_walker_control_default` - Default WalkerControl values
- `test_walker_bone_control_serialization` - Serialize/deserialize bone data

#### Integration Tests
- `test_spawn_walker` - Spawn walker actor in simulation
- `test_walker_movement` - Apply walker control and verify movement
- `test_walker_ai_controller_spawn` - Spawn AI controller for walker
- `test_walker_ai_navigation` - Navigate walker to target location
- `test_walker_speed_control` - Set and verify walker speed
- `test_walker_bone_manipulation` - Set custom bone poses
- `test_walker_animation_blend` - Blend custom pose with animation
- `test_multiple_walkers_no_collision` - Spawn multiple walkers, verify navigation

---

## Phase 2: Debug and Visualization Utilities

**Priority:** Medium
**Estimated Effort:** 2 weeks
**Status:** Not Started

### Work Items

- [ ] **DebugHelper**
  - File: `carla/src/client/debug_helper.rs`
  - Implement `DebugHelper` wrapping `carla::client::DebugHelper`
  - Drawing methods:
    - `draw_point(location, size, color, life_time)`
    - `draw_line(begin, end, thickness, color, life_time)`
    - `draw_arrow(begin, end, thickness, arrow_size, color, life_time)`
    - `draw_box(box, rotation, thickness, color, life_time)`
    - `draw_string(location, text, draw_shadow, color, life_time)`

- [ ] **Color Type**
  - File: `carla/src/rpc/color.rs`
  - `Color` struct with `r, g, b, a` fields
  - Predefined color constants (RED, GREEN, BLUE, WHITE, etc.)

- [ ] **World Debug Access**
  - Add `World::debug()` method returning `DebugHelper`
  - Integration with existing World API

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
**Estimated Effort:** 2-3 weeks
**Status:** Not Started

### Work Items

- [ ] **Client Recording Methods**
  - File: `carla/src/client/carla_client.rs`
  - Methods:
    - `start_recorder(filename, additional_data)` - Start recording
    - `stop_recorder()` - Stop recording
    - `show_recorder_file_info(filename, show_all)` - Get recording info
    - `show_recorder_collisions(filename, category1, category2)` - Query collisions
    - `show_recorder_actors_blocked(filename, min_time, min_distance)` - Find blocked actors

- [ ] **Replay Methods**
  - Methods:
    - `replay_file(filename, start_time, duration, follow_id, replay_sensors)` - Replay recording
    - `stop_replayer(keep_actors)` - Stop replay
    - `set_replayer_time_factor(time_factor)` - Control replay speed
    - `set_replayer_ignore_hero(ignore_hero)` - Ignore hero vehicle in replay
    - `set_replayer_ignore_spectator(ignore)` - Ignore spectator (0.9.15+)

- [ ] **Recorder Info Types**
  - File: `carla/src/rpc/recorder_info.rs`
  - Structs for recording metadata

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
**Estimated Effort:** 2 weeks
**Status:** Not Started

### Work Items

- [ ] **Ackermann Control** (0.9.14+)
  - File: `carla/src/rpc/ackermann_control.rs`
  - Already exists but needs verification
  - Vehicle methods:
    - `apply_ackermann_control(control)` - Apply Ackermann steering
    - `get_ackermann_controller_settings()` - Get controller settings
    - `apply_ackermann_controller_settings(settings)` - Update settings

- [ ] **Vehicle Failure State** (0.9.14+)
  - File: `carla/src/rpc/vehicle_failure_state.rs`
  - Enum for failure types (Rollover, Engine, Brake, TirePuncture, etc.)
  - Method: `Vehicle::get_failure_state()`

- [ ] **Vehicle Telemetry** (0.9.16+ only)
  - File: `carla/src/rpc/vehicle_telemetry_data.rs`
  - Feature-gated with `#[cfg(carla_0916)]`
  - Method: `Vehicle::get_telemetry_data()`
  - Detailed physics data (suspension, wheels, etc.)

- [ ] **Wheel Pitch Control** (0.9.16+ only)
  - Feature-gated methods:
    - `Vehicle::set_wheel_pitch_angle(wheel_location, angle)`
    - `Vehicle::get_wheel_pitch_angle(wheel_location)`
    - `Vehicle::get_vehicle_bone_world_transforms()`
    - `Vehicle::restore_phys_x_physics()`

- [ ] **Vehicle Doors** (0.9.13+)
  - Methods:
    - `Vehicle::open_door(door_type)` - Open specific door
    - `Vehicle::close_door(door_type)` - Close specific door
  - Already has `VehicleDoor` enum in RPC

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

- [ ] **RSS Sensor** (0.9.7+)
  - File: `carla/src/sensor/data/rss_response.rs`
  - Responsibility-Sensitive Safety sensor
  - Complex integration with Intel RSS library
  - May be deferred due to external dependencies

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
- [CARLA C++ Client LibCarla Source](https://github.com/carla-simulator/carla/tree/master/LibCarla/source/carla/client)
- [CARLA CHANGELOG](https://github.com/carla-simulator/carla/blob/master/CHANGELOG.md)
- [carla-rust API Differences](./carla_api_differences.md)
