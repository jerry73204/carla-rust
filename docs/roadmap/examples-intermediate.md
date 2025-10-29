# Intermediate Example Implementations (Phase 11)

**[← Back to Roadmap Index](../roadmap.md)**

This document covers Phase 11: Intermediate Example Implementations (Headless/No GUI). These examples demonstrate more complex CARLA operations suitable for headless operation, automated testing, or CI/CD environments.

---

## Phase 11: Intermediate Example Implementations (Headless/No GUI)

**Priority:** Medium
**Estimated Effort:** 3-4 weeks
**Status:** ✅ COMPLETE (6/6 headless examples implemented)
**✅ Completion Date:** 2025-10-29
**⚠️ Important Note:** These are **simplified headless versions** for API demonstration. Full GUI versions matching Python examples are in Phase 12.

### Overview

Implement intermediate examples that combine multiple features without GUI requirements. These demonstrate core functionality and are useful for:
- Automated testing and CI/CD pipelines
- Headless servers and cloud environments
- API validation and learning
- Data collection workflows

**Differences from Python Examples:**
- Python's `automatic_control.py`, `synchronous_mode.py`, and `visualize_multiple_sensors.py` use pygame for real-time visualization
- Our Phase 11 versions are simplified for headless operation (console output or save to disk)
- Full GUI versions matching Python behavior are planned for Phase 12

**Implementation Status:** All six headless examples successfully implemented using APIs from Phase 1 (Walker AI), Phase 3 (Recording), Phase 4 (Vehicle Physics), Phase 5 (Batch Operations), Phase 8 (Navigation), and Phase 9 (Camera Projection, Actor Destruction, Image Persistence).

### Completed Work Items

- [x] **Automatic Control (Headless)** (`automatic_control.rs`) ✅
  - File: `carla/examples/automatic_control.rs`
  - **Python Equivalent:** `automatic_control.py` (has pygame GUI with camera view)
  - **Our Implementation:** Headless console-only version
  - Enables autopilot on vehicle
  - Displays real-time telemetry (location, velocity, control inputs) to console
  - 60-second monitoring session with formatted table output
  - Tests manual control override
  - Demonstrates: Autopilot, telemetry queries, control override, actor cleanup
  - **Key APIs Used:**
    - `Vehicle::set_autopilot()`
    - `Vehicle::control()` - Get current control inputs
    - `Vehicle::transform()`, `Vehicle::velocity()`
    - `Vehicle::apply_control()` - Manual control override
    - `Actor::destroy()` - Cleanup
  - **Test Status:** ✅ Compiled successfully, runs with CARLA server
  - **Note:** Full GUI version with camera display planned for Phase 12

- [x] **Synchronous Mode (Headless)** (`synchronous_mode.rs`) ✅
  - File: `carla/examples/synchronous_mode.rs`
  - **Python Equivalent:** `synchronous_mode.py` (has pygame GUI with camera display)
  - **Our Implementation:** Headless console-only version
  - Enables synchronous simulation (fixed timestep)
  - Spawns vehicle with autopilot
  - Attaches RGB and semantic segmentation cameras
  - Runs 100 ticks (5 seconds at 20 FPS)
  - Verifies frame synchronization between sensors via console output
  - Demonstrates: Synchronous mode, fixed delta time, tick-based simulation, sensor sync
  - **Key APIs Used:**
    - `WorldSettings::synchronous_mode = true`
    - `WorldSettings::fixed_delta_seconds = Some(0.05)`
    - `World::apply_settings()`
    - `World::tick()` - Advance simulation by one frame
    - Multiple camera sensors with `listen()` callbacks
    - `AttachmentType::SpringArm` - Camera mount type
  - **Test Status:** ✅ Compiled successfully, demonstrates perfect sensor synchronization
  - **Note:** Full GUI version with camera display planned for Phase 12

- [x] **Sensor Synchronization** (`sensor_synchronization.rs`) ✅
  - File: `carla/examples/sensor_synchronization.rs`
  - **Python Equivalent:** `sensor_synchronization.py` (console-only, no GUI)
  - **Our Implementation:** ✅ Matches Python behavior
  - Spawns 4 different sensor types: RGB, Depth, Semantic, Lidar
  - All sensors attached to moving vehicle
  - 30-second monitoring of frame rates and synchronization
  - Calculates synchronization quality metrics via console output
  - Demonstrates: Multi-sensor setup, coordinate frames, data rates, sync verification
  - **Key APIs Used:**
    - Multiple sensor blueprints (camera.rgb, camera.depth, camera.semantic_segmentation, lidar.ray_cast)
    - `Sensor::listen()` with Arc<Mutex<>> for synchronized counting
    - `Image::try_from()` for camera data
    - `LidarMeasurement::try_from()` for lidar data
    - `LidarMeasurement::len()` - Total point count
    - Separate transforms for cameras vs lidar placement
  - **Test Status:** ✅ Compiled successfully, monitors 4 sensors simultaneously

- [x] **Generate Traffic** (`generate_traffic.rs`) ✅
  - File: `carla/examples/generate_traffic.rs` (295 lines)
  - **Python Equivalent:** `generate_traffic.py` (console-only, no GUI)
  - **Our Implementation:** ✅ Matches Python behavior
  - Spawns 30 vehicles with autopilot using batch operations
  - Spawns 50 walkers with AI controllers
  - Demonstrates pedestrian navigation and random waypoint generation
  - 60-second traffic simulation with console monitoring
  - Demonstrates: Batch spawn commands, Walker AI controllers, traffic scenarios
  - **Key APIs Used:**
    - `Command::spawn_actor()` - Batch vehicle/walker spawning
    - `Command::set_autopilot()` - Batch autopilot enable
    - `Client::apply_batch_sync()` - Synchronous batch execution
    - `WalkerAIController::start()`, `stop()`, `set_max_speed()`, `go_to_location()`
    - `World::random_location_from_navigation()` - Random sidewalk locations
    - `World::set_pedestrians_cross_factor()` - Pedestrian behavior config
  - **Test Status:** ✅ Compiled successfully, spawns traffic with AI control

- [x] **LiDAR-to-Camera Projection** (`lidar_to_camera.rs`) ✅
  - File: `carla/examples/lidar_to_camera.rs` (321 lines)
  - **Python Equivalent:** `lidar_to_camera.py` (saves to disk, no GUI)
  - **Our Implementation:** ✅ Matches Python behavior
  - Projects 3D LiDAR points onto 2D camera images
  - Demonstrates sensor fusion and coordinate transformations
  - Uses synchronous mode for perfect sensor synchronization
  - Colors points by distance using viridis colormap
  - Saves 50 annotated images showing LiDAR overlay to `_out/` directory
  - Demonstrates: Sensor fusion, camera projection math, coordinate transforms
  - **Key APIs Used:**
    - `carla::sensor::camera::build_projection_matrix()` - Camera intrinsic matrix
    - `carla::sensor::camera::world_to_camera()` - World-to-camera transform
    - `carla::sensor::camera::project_to_2d()` - 3D-to-2D projection
    - `SensorDataBase::frame()` - Frame number for synchronization
    - `LidarMeasurement::as_slice()` - Point cloud iteration
    - `Image::save_to_disk()` - Save annotated images
  - **Test Status:** ✅ Compiled successfully, demonstrates sensor fusion

- [x] **Multiple Sensor Visualization (Headless)** (`visualize_multiple_sensors.rs`) ✅
  - File: `carla/examples/visualize_multiple_sensors.rs` (300 lines)
  - **Python Equivalent:** `visualize_multiple_sensors.py` (pygame GUI with 2×3 grid display)
  - **Our Implementation:** Headless version - saves to disk instead of real-time display
  - Spawns 4 sensor types on single vehicle: RGB, Depth, Semantic, LiDAR
  - Captures synchronized sensor data for 20 frames
  - Saves RGB, depth, and semantic segmentation images to `_out/sensors/` directory
  - Logs LiDAR point cloud statistics to console
  - Demonstrates: Multi-sensor setup, synchronized capture, data persistence
  - **Key APIs Used:**
    - Multiple sensor blueprints and attribute configuration
    - `World::spawn_actor_opt()` - Attach sensors to parent vehicle
    - `Sensor::listen()` - Multiple concurrent sensor callbacks
    - `Image::save_to_disk()` - Save sensor images
    - `LidarMeasurement::len()`, `channel_count()` - LiDAR stats
    - `Arc<Mutex<>>` - Thread-safe data sharing between callbacks
  - **Test Status:** ✅ Compiled successfully, saves multi-sensor outputs
  - **Note:** Full GUI version with real-time 2×3 grid display planned for Phase 12

- [x] **Camera Projection Utilities** ✅
  - **Priority:** HIGH (Required for lidar_to_camera.rs, visualize_multiple_sensors.rs)
  - **Implementation:** Pure Rust (no FFI required)
  - **Completion Date:** 2025-10-29
  - **Rust API (carla):**
    - File: `carla/src/sensor/camera.rs` (NEW - 268 lines)
    - ✅ `build_projection_matrix(width, height, fov) -> Matrix3<f32>`
    - ✅ `world_to_camera(point, camera_transform) -> Vector3<f32>`
    - ✅ `project_to_2d(point_3d, k_matrix) -> (f32, f32)`
    - ✅ 5 comprehensive unit tests passing
    - ✅ Complete documentation with examples
  - **Features:**
    - Pinhole camera model implementation
    - UE4 to standard camera coordinate conversion
    - 3D-to-2D projection with normalization
    - Based on CARLA Python lidar_to_camera.py reference
  - **Usage:** Sensor fusion, LiDAR-to-camera projection, bounding box projection
  - **Unblocked Examples:** lidar_to_camera.rs, visualize_multiple_sensors.rs

- [x] **Collision and Lane Invasion Sensors** ✅
  - **Priority:** MEDIUM
  - **Completion Date:** 2025-10-29
  - **Rust API (carla):**
    - File: `carla/src/sensor/data/collision_event.rs`
    - ✅ `CollisionEvent::actor()` - Sensor owner
    - ✅ `CollisionEvent::other_actor()` - Collision partner (or None for static objects)
    - ✅ `CollisionEvent::normal_impulse()` - Force vector (N·s)
    - ✅ Complete module and method documentation
    - ✅ Usage examples in doc comments
  - **Rust API (carla):**
    - File: `carla/src/sensor/data/lane_invasion_event.rs`
    - ✅ `LaneInvasionEvent::actor()` - Sensor owner
    - ✅ `LaneInvasionEvent::crossed_lane_markings()` - Lane marking details
    - ✅ Complete module and method documentation
    - ✅ Usage examples in doc comments
  - **Usage:** Safety monitoring, collision detection, lane departure warnings

### Deferred Work Items

- [ ] **GBuffer Access** (`tutorial_gbuffer.rs`) - DEFERRED
  - Requires: GBuffer texture APIs (Phase 6 - deferred)
  - Access GBuffer textures (depth, normals, etc.)
  - Demonstrates: Advanced rendering features
  - **Status:** Deferred pending Phase 6 GBuffer API implementation

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

