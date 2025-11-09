# Cross-Phase Testing

**[← Back to Roadmap Index](../roadmap.md)**

This document covers the cross-phase testing strategy for the carla-rust project. This includes test infrastructure, methodologies, and best practices that span across all development phases.

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

---

## Phase 10: Test Examples Organization

**Priority:** HIGH (enables automated testing)
**Estimated Effort:** 2-3 weeks
**Status:** [ ] Planned

### Overview

Organize all unit and integration tests from Phases 2-9 into comprehensive example programs that can be executed by `scripts/run-examples.sh`. Each example connects to CARLA, sets up the necessary scenario, and runs related tests together.

**Rationale:**
- Tests require a running CARLA server with configured scenarios
- Cannot run tests in parallel (CARLA server limitation)
- Examples provide both documentation and automated testing
- Existing `run-examples.sh` infrastructure can execute tests sequentially

### Test Organization Strategy

Tests are grouped by feature area into comprehensive example programs. Each example:
1. Connects to CARLA server
2. Sets up required scenario (spawn vehicles, sensors, etc.)
3. Runs all related tests for that feature area
4. Reports results with clear pass/fail status
5. Cleans up resources

### Work Items

#### 10.1: Debug and Visualization Tests (`test_debug_draw.rs`)

**From Phase 2** - Debug drawing and color utilities (11 tests)

**Setup:**
- Spawn a vehicle at a spawn point
- Position spectator camera to view debug shapes

**Tests:**
- [ ] `test_color_creation` - Create colors with RGBA values
- [ ] `test_color_constants` - Verify predefined color values
- [ ] `test_color_to_native` - Convert to C++ color type
- [ ] `test_debug_draw_point` - Draw point and verify visibility
- [ ] `test_debug_draw_line` - Draw line between two points
- [ ] `test_debug_draw_arrow` - Draw arrow with proper direction
- [ ] `test_debug_draw_box` - Draw bounding box
- [ ] `test_debug_draw_string` - Draw text at location
- [ ] `test_debug_shapes_lifetime` - Verify shapes disappear after lifetime
- [ ] `test_debug_multiple_shapes` - Draw multiple shapes simultaneously
- [ ] `test_debug_shape_colors` - Verify shape colors are correct

**Success Criteria:**
- All color operations work correctly
- Debug shapes render at correct positions
- Lifetimes respected (shapes disappear when expected)

---

#### 10.2: Recording and Playback Tests (`test_recording.rs`)

**From Phase 3** - Recorder/replayer functionality (12 tests)

**Setup:**
- Spawn 2-3 vehicles in Town03
- Drive vehicles for 10 seconds to create interesting scenario
- Record the scenario

**Tests:**
- [ ] `test_recorder_filename_validation` - Validate recording filenames
- [ ] `test_replay_time_factor_bounds` - Validate time factor range
- [ ] `test_start_stop_recorder` - Start and stop recording
- [ ] `test_record_simple_scenario` - Record vehicle movement
- [ ] `test_replay_recorded_scenario` - Replay recorded scenario
- [ ] `test_replay_with_follow_camera` - Follow specific actor during replay
- [ ] `test_replay_time_factor` - Speed up/slow down replay
- [ ] `test_recorder_file_info` - Get recording metadata
- [ ] `test_recorder_collision_query` - Query recorded collisions
- [ ] `test_recorder_frame_accuracy` - Verify frame-accurate replay
- [ ] `test_recorder_actor_bounds` - Query recorded actor movements
- [ ] `test_replay_pause_resume` - Pause and resume replay

**Success Criteria:**
- Record and replay scenarios accurately
- Metadata correctly extracted from recordings
- Time factor affects replay speed
- Camera following works during replay

---

#### 10.3: Advanced Vehicle Features Tests (`test_vehicle_advanced.rs`)

**From Phase 4** - Vehicle physics, doors, lights (12 tests)

**Setup:**
- Spawn a vehicle (Tesla Model 3 or similar)
- Ensure vehicle has physics enabled

**Tests:**
- [ ] `test_vehicle_door_open_close` - Open/close individual doors
- [ ] `test_vehicle_door_states` - Query all door states
- [ ] `test_vehicle_wheel_count` - Get vehicle wheel count
- [ ] `test_vehicle_wheel_states` - Query individual wheel states
- [ ] `test_failure_state_creation` - Create failure states
- [ ] `test_apply_failure_state` - Apply failure to vehicle
- [ ] `test_vehicle_with_failures` - Drive with active failures
- [ ] `test_light_state_creation` - Create light states
- [ ] `test_light_state_flags` - Verify light state bitflags
- [ ] `test_apply_light_state` - Apply lights to vehicle
- [ ] `test_vehicle_light_query` - Query current light states
- [ ] `test_multiple_light_states` - Combine multiple lights

**Success Criteria:**
- Doors open/close correctly
- Wheel states reported accurately
- Failure states affect vehicle behavior
- Light states apply and query correctly

---

#### 10.4: Batch Operations Tests (`test_batch_commands.rs`)

**From Phase 5** - Batch command execution (12 tests)

**Setup:**
- Prepare list of spawn points
- Have blueprint library ready

**Tests:**
- [ ] `test_batch_spawn_actors` - Spawn multiple actors in batch
- [ ] `test_batch_destroy_actors` - Destroy multiple actors in batch
- [ ] `test_batch_apply_vehicle_control` - Control multiple vehicles
- [ ] `test_batch_apply_walker_control` - Control multiple walkers
- [ ] `test_batch_command_response` - Verify individual responses
- [ ] `test_batch_error_handling` - Handle failed commands in batch
- [ ] `test_batch_partial_failure` - Some commands succeed, some fail
- [ ] `test_batch_order_preservation` - Commands execute in order
- [ ] `test_empty_batch` - Handle empty batch gracefully
- [ ] `test_large_batch` - Handle 100+ commands
- [ ] `test_batch_performance` - Verify performance benefit
- [ ] `test_mixed_command_types` - Mix spawn/destroy/control commands

**Success Criteria:**
- Batch operations faster than individual commands
- Partial failures handled gracefully
- Response order matches command order
- Large batches (100+) execute successfully

---

#### 10.5: Advanced Sensor Tests (`test_sensors_advanced.rs`)

**From Phase 6** - DVS, optical flow, normals sensors (10 tests)

**Setup:**
- Spawn vehicle with multiple advanced sensors
- Drive vehicle through dynamic scene

**Tests:**
- [ ] `test_dvs_event_creation` - Create DVS events
- [ ] `test_dvs_camera_events` - Capture DVS events
- [ ] `test_dvs_event_stream` - Process continuous event stream
- [ ] `test_optical_flow_capture` - Capture optical flow
- [ ] `test_optical_flow_visualization` - Convert to RGB for visualization
- [ ] `test_normals_sensor_capture` - Capture surface normals
- [ ] `test_normals_world_space` - Verify normals in world space
- [ ] `test_dvs_high_frequency` - Handle high-frequency events
- [ ] `test_optical_flow_motion` - Verify flow vectors match motion
- [ ] `test_sensor_synchronization_advanced` - Sync DVS with RGB camera

**Success Criteria:**
- DVS events captured during camera motion
- Optical flow vectors represent scene motion
- Normals correctly oriented in world space
- Advanced sensors synchronize with standard cameras

---

#### 10.6: World Operations Tests (`test_world_advanced.rs`)

**From Phase 7** - Traffic lights, landmarks, environment (11 tests)

**Setup:**
- Load Town03 (has many traffic lights)
- Spawn vehicle near intersection

**Tests:**
- [ ] `test_get_traffic_lights` - Query all traffic lights
- [ ] `test_traffic_light_state_change` - Change traffic light state
- [ ] `test_traffic_light_timing` - Set green/yellow/red durations
- [ ] `test_traffic_light_freeze` - Freeze traffic light state
- [ ] `test_get_landmarks` - Query all landmarks
- [ ] `test_landmarks_by_type` - Filter landmarks by type
- [ ] `test_landmark_waypoints` - Get waypoints for landmark
- [ ] `test_environment_objects_query` - Query environment objects
- [ ] `test_environment_object_enable_disable` - Toggle object visibility
- [ ] `test_reset_all_traffic_lights` - Reset all lights to default
- [ ] `test_world_tick_timeout` - Verify tick timeout behavior

**Success Criteria:**
- Traffic lights change state correctly
- Timing parameters respected
- Landmarks correctly queried and filtered
- Environment objects can be toggled

---

#### 10.7: Navigation Tests (`test_navigation.rs`)

**From Phase 8** - Waypoint, topology, routing (7 tests)

**Setup:**
- Load map with complex road network (Town03)
- Spawn vehicle at starting position

**Tests:**
- [ ] `test_get_waypoint_at_location` - Find nearest waypoint
- [ ] `test_waypoint_next` - Get next waypoints
- [ ] `test_waypoint_lane_change` - Get lane change options
- [ ] `test_topology_generation` - Generate road topology
- [ ] `test_route_planning` - Plan route between two points
- [ ] `test_route_waypoints` - Get waypoints along route
- [ ] `test_waypoint_transform` - Get transform for waypoint

**Success Criteria:**
- Waypoints correctly located on roads
- Next waypoints follow lane geometry
- Lane changes identified correctly
- Routes generated between valid points

---

#### 10.8: Utility Functions Tests (`test_utils.rs`)

**From Phase 9** - Transforms, strings, environment vars (20 tests)

**Setup:**
- Minimal setup, mostly pure functions
- Spawn one vehicle for transform tests

**Tests:**

**Transform Operations (10 tests):**
- [ ] `test_transform_multiplication` - Combine transforms
- [ ] `test_transform_inverse` - Invert transform
- [ ] `test_transform_point` - Transform point to world space
- [ ] `test_transform_vector` - Transform vector (no translation)
- [ ] `test_transform_identity` - Identity transform behavior
- [ ] `test_transform_chain` - Chain multiple transforms
- [ ] `test_transform_parent_child` - Parent-child relationships
- [ ] `test_transform_world_to_local` - World to local conversion
- [ ] `test_transform_rotation_only` - Rotation-only transforms
- [ ] `test_transform_translation_only` - Translation-only transforms

**String/Utility Tests (10 tests):**
- [ ] `test_actor_id_from_string` - Parse actor ID from string
- [ ] `test_string_to_actor_id_invalid` - Handle invalid ID strings
- [ ] `test_episode_id_equality` - Compare episode IDs
- [ ] `test_episode_id_string_conversion` - Convert episode ID to string
- [ ] `test_get_client_version` - Get CARLA client version string
- [ ] `test_get_server_version` - Get CARLA server version string
- [ ] `test_version_comparison` - Compare version strings
- [ ] `test_environment_variable_access` - Access CARLA env vars
- [ ] `test_debug_flag_checking` - Check debug flags
- [ ] `test_timeout_config` - Verify timeout configurations

**Success Criteria:**
- Transform math matches expected results
- String parsing handles valid and invalid inputs
- Version information correctly retrieved
- Environment configuration accessible

---

### Implementation Guidelines

#### Example Structure Pattern

```rust
//! <Feature Area> Tests
//!
//! Tests for <brief description>
//!
//! # Test Categories
//! - Unit tests: <list>
//! - Integration tests: <list>
//!
//! Run with:
//! ```bash
//! cargo run --example test_<name> --profile dev-release
//! ```

use carla::client::Client;
use std::time::Duration;

fn main() {
    println!("=== <Feature> Tests ===\n");

    // Connect to CARLA
    let client = Client::connect("127.0.0.1", 2000, None);
    let mut world = client.world();

    // Setup scenario
    setup_scenario(&mut world);

    // Run tests
    let mut passed = 0;
    let mut failed = 0;

    run_test("test_name", || { /* test code */ }, &mut passed, &mut failed);

    // Report results
    println!("\n=== Results ===");
    println!("Passed: {}", passed);
    println!("Failed: {}", failed);
    std::process::exit(if failed > 0 { 1 } else { 0 });
}

fn run_test<F>(name: &str, test_fn: F, passed: &mut i32, failed: &mut i32)
where
    F: FnOnce() -> Result<(), Box<dyn std::error::Error>>,
{
    print!("Testing {}... ", name);
    match test_fn() {
        Ok(_) => {
            println!("✓ PASS");
            *passed += 1;
        }
        Err(e) => {
            println!("✗ FAIL: {}", e);
            *failed += 1;
        }
    }
}
```

#### Test Organization Principles

1. **Group by Feature Area:** Related tests in same example
2. **Shared Setup:** One scenario setup for all tests in example
3. **Independent Tests:** Each test should not depend on previous tests
4. **Clear Output:** Print test name and pass/fail status
5. **Exit Code:** Exit with code 1 if any test fails (for CI/automation)
6. **Documentation:** Explain what's being tested and why

#### Benefits

- ✅ Tests run automatically via `scripts/run-examples.sh`
- ✅ One CARLA connection per feature area (not per test)
- ✅ Examples serve as both tests and documentation
- ✅ Clear pass/fail reporting for CI integration
- ✅ Easy to run individual test suites during development
- ✅ Scenario setup code shows how to use the API

### Timeline

**Week 1:**
- Days 1-2: Implement test_debug_draw.rs (10.1)
- Days 3-4: Implement test_recording.rs (10.2)
- Day 5: Implement test_vehicle_advanced.rs (10.3)

**Week 2:**
- Days 1-2: Implement test_batch_commands.rs (10.4)
- Day 3: Implement test_sensors_advanced.rs (10.5)
- Days 4-5: Implement test_world_advanced.rs (10.6)

**Week 3:**
- Days 1-2: Implement test_navigation.rs (10.7)
- Day 3: Implement test_utils.rs (10.8)
- Days 4-5: Integration testing, CI setup, documentation

**Total: 13-15 days**

### Phase 10 Success Criteria

- [ ] All 95 tests organized into 8 comprehensive examples
- [ ] Examples executable via `scripts/run-examples.sh`
- [ ] Each example exits with code 0 (success) or 1 (failure)
- [ ] Clear test output shows which tests passed/failed
- [ ] Documented test patterns for future additions
- [ ] CI integration ready (all examples can run sequentially)

---

## References

- [CARLA Python API Documentation](https://carla.readthedocs.io/en/latest/python_api/)
- [CARLA Python Examples](https://github.com/carla-simulator/carla/tree/master/PythonAPI/examples)
- [CARLA C++ Client LibCarla Source](https://github.com/carla-simulator/carla/tree/master/LibCarla/source/carla/client)
- [CARLA CHANGELOG](https://github.com/carla-simulator/carla/blob/master/CHANGELOG.md)
- [carla-rust API Differences](./carla_api_differences.md)
- [Example Implementation Plan](./example_implementation_plan.md)
