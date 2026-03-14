# CARLA Rust Example Implementation Plan

This document provides a comprehensive analysis of Python examples from the CARLA repository and guides their implementation in Rust.

## Table of Contents

1. [Overview](#overview)
2. [Example Categories](#example-categories)
3. [Implementation Phases](#implementation-phases)
4. [Detailed Example Matrix](#detailed-example-matrix)
5. [API Gap Analysis](#api-gap-analysis)
6. [Rust Implementation Guidelines](#rust-implementation-guidelines)
7. [External Dependencies](#external-dependencies)
8. [Testing Strategy](#testing-strategy)

---

## Overview

The CARLA Python repository contains 36+ example scripts demonstrating various features. This plan categorizes them by complexity and provides a roadmap for Rust implementation.

**⚠️ IMPORTANT: This is a Planning Document**

Examples in Phases 10-13 **cannot be implemented** until their required APIs are available. Focus on API implementation (Phases 1-9 in roadmap.md) before starting example development.

**Current Status:**
- ✅ **11 examples** already implemented in Rust (Phase 0)
- 📋 **25 examples** planned for implementation (Phases 10-13)
- ⚠️ **Blocked**: Examples depend on APIs from Phases 1-9

**Implementation Order:**
1. **First**: Complete API phases (roadmap.md Phases 1-9)
2. **Then**: Implement examples (Phases 10-13 described here)

**Goals:**
- Provide idiomatic Rust examples for all core CARLA features
- Maintain consistency with Python API where sensible
- Leverage Rust's type safety and error handling
- Create reusable patterns for CARLA Rust development

---

## Example Categories

### Already Implemented (Phase 0) ✅

| Example Name       | Rust File               | Description                        |
|--------------------|-------------------------|------------------------------------|
| Connect            | `connect.rs`            | Basic connection to simulator      |
| Blueprints         | `blueprints.rs`         | Query and filter blueprint library |
| World Info         | `world_info.rs`         | Get map, spawn points, actors      |
| Spawn Vehicle      | `spawn_vehicle.rs`      | Spawn single vehicle               |
| Multiple Vehicles  | `multiple_vehicles.rs`  | Spawn multiple vehicles            |
| Spawn Walker       | `spawn_walker.rs`       | Spawn single pedestrian            |
| Multiple Walkers   | `multiple_walkers.rs`   | Spawn multiple pedestrians         |
| Vehicle Transform  | `vehicle_transform.rs`  | Get/set vehicle transforms         |
| Vehicle Attributes | `vehicle_attributes.rs` | Query vehicle attributes           |
| Walker Control     | `walker_control.rs`     | Apply walker movement              |
| Walker Directions  | `walker_directions.rs`  | Test different movement vectors    |

**Coverage**: Core actor management, basic vehicle and walker operations.

### To Implement

Examples grouped into implementation phases based on:
- **Complexity**: Simple → Intermediate → Advanced → Specialized
- **API Dependencies**: What features need to be implemented first
- **Priority**: User value and demonstration completeness

---

## Implementation Phases

**⚠️ REMINDER**: These phases are blocked until their prerequisite APIs are implemented. See `roadmap.md` for API implementation phases (1-9).

### Phase 10: Simple Examples (Priority: Medium)

**Target**: 9 examples demonstrating single features or simple combinations

**Estimated Effort**: 2-3 weeks

**⚠️ BLOCKED UNTIL**: Phase 3, 4 APIs are 80%+ complete

**Prerequisites**:
- Phase 3: Recording and Playback APIs (must implement first)
- Phase 4: Advanced Vehicle Features (must implement first)
- Weather API (simple wrapper needed - must implement first)

#### Examples to Implement

| # | Example                      | Python File                       | Complexity | Priority |
|---|------------------------------|-----------------------------------|------------|----------|
| 1 | Tutorial                     | `tutorial.py`                     | ⭐ Simple  | High     |
| 2 | Vehicle Gallery              | `vehicle_gallery.py`              | ⭐ Simple  | Medium   |
| 3 | Dynamic Weather              | `dynamic_weather.py`              | ⭐ Simple  | High     |
| 4 | Vehicle Physics              | `vehicle_physics.py`              | ⭐ Simple  | Medium   |
| 5 | Start Recording              | `start_recording.py`              | ⭐ Simple  | High     |
| 6 | Start Replaying              | `start_replaying.py`              | ⭐ Simple  | High     |
| 7 | Show Recorder File Info      | `show_recorder_file_info.py`      | ⭐ Simple  | Medium   |
| 8 | Show Recorder Collisions     | `show_recorder_collisions.py`     | ⭐ Simple  | Medium   |
| 9 | Show Recorder Blocked Actors | `show_recorder_actors_blocked.py` | ⭐ Simple  | Medium   |

**Key Features Demonstrated**:
- Camera sensor attachment and callbacks
- Weather parameter control
- Physics impulse and force application
- Recording and playback functionality
- Recording metadata queries

---

### Phase 11: Intermediate Examples (Priority: Medium)

**Target**: 8 examples combining multiple features

**Estimated Effort**: 3-4 weeks

**⚠️ BLOCKED UNTIL**: Phase 5, 6 APIs are 80%+ complete

**Prerequisites**:
- Phase 5: Batch Operations and Commands (must implement first)
- Phase 6: Advanced Sensor Features (must implement first)
- Traffic Manager enhancements (must implement first)
- Synchronous mode support (must implement first)

#### Examples to Implement

| # | Example                    | Python File                     | Complexity    | Priority |
|---|----------------------------|---------------------------------|---------------|----------|
| 1 | Generate Traffic           | `generate_traffic.py`           | ⭐⭐ Moderate | High     |
| 2 | Sensor Synchronization     | `sensor_synchronization.py`     | ⭐⭐ Moderate | High     |
| 3 | Synchronous Mode           | `synchronous_mode.py`           | ⭐⭐ Moderate | High     |
| 4 | Automatic Control          | `automatic_control.py`          | ⭐⭐ Moderate | Medium   |
| 5 | Lidar to Camera            | `lidar_to_camera.py`            | ⭐⭐ Moderate | Medium   |
| 6 | Visualize Multiple Sensors | `visualize_multiple_sensors.py` | ⭐⭐ Moderate | Medium   |
| 7 | Get Component Test         | `get_component_test.py`         | ⭐ Simple     | Low      |
| 8 | Tutorial GBuffer           | `tutorial_gbuffer.py`           | ⭐⭐ Moderate | Low      |

**Key Features Demonstrated**:
- Batch actor spawning with Traffic Manager
- Walker AI controllers
- Synchronous simulation mode
- Multi-sensor data collection
- Coordinate transformations
- GBuffer access

---

### Phase 12: Advanced Examples (Priority: Medium)

**Target**: 8 examples with complex interactions

**Estimated Effort**: 4-5 weeks

**Prerequisites**:
- UI framework decision (winit + pixels recommended)
- Image processing crate integration
- Projection math utilities
- Phase 2: Debug Helper for drawing

#### Examples to Implement

| # | Example                       | Python File                       | Complexity     | Priority |
|---|-------------------------------|-----------------------------------|----------------|----------|
| 1 | Manual Control                | `manual_control.py`               | ⭐⭐⭐ Complex | High     |
| 2 | Manual Control Steering Wheel | `manual_control_steeringwheel.py` | ⭐⭐⭐ Complex | Low      |
| 3 | Bounding Boxes                | `bounding_boxes.py`               | ⭐⭐⭐ Complex | Medium   |
| 4 | Client Bounding Boxes         | `client_bounding_boxes.py`        | ⭐⭐⭐ Complex | Medium   |
| 5 | Draw Skeleton                 | `draw_skeleton.py`                | ⭐⭐⭐ Complex | Low      |
| 6 | No Rendering Mode             | `no_rendering_mode.py`            | ⭐⭐⭐ Complex | Medium   |
| 7 | Open3D Lidar                  | `open3d_lidar.py`                 | ⭐⭐⭐ Complex | Low      |
| 8 | Manual Control CarSim/Chrono  | `manual_control_*.py`             | ⭐⭐⭐ Complex | Low      |

**Key Features Demonstrated**:
- Interactive keyboard/mouse control
- Hardware input (steering wheels)
- 2D/3D bounding box computation
- Camera projection matrices
- Walker skeleton visualization
- Large-scale scenarios (100+ vehicles)
- Lidar point cloud visualization
- External physics integrations

**Design Considerations**:
- **UI Framework**: Recommend `winit` + `pixels` for simple rendering
  - Alternative: `bevy` for more complex 3D vis
  - Avoid pygame dependency (Python-specific)
- **Input Handling**: Use `gilrs` for gamepad/wheel support
- **Math**: `nalgebra` or `glam` for projection matrices
- **Image Processing**: `image` crate for sensor data

---

### Phase 13: Specialized Examples (Priority: Low/Future)

**Target**: External integration examples (most deferred)

**Estimated Effort**: Varies by integration

**Status**: Documentation only, implementation deferred

#### Examples (Reference Only)

| # | Example            | Python File             | Integration    | Status   |
|---|--------------------|-------------------------|----------------|----------|
| 1 | InvertedAI Traffic | `invertedai_traffic.py` | InvertedAI API | Deferred |
| 2 | CARLA Cosmos Gen   | `carla_cosmos_gen.py`   | NVIDIA Cosmos  | Deferred |
| 3 | V2X Demo           | `V2XDemo.py`            | V2X Protocol   | Deferred |
| 4 | Test Add Second VX | `test_addsecondvx.py`   | V2X Testing    | Deferred |
| 5 | NVIDIA Integration | `nvidia/*`              | NVIDIA Tools   | Deferred |
| 6 | ROS2 Bridge        | `ros2/*`                | ROS2           | Future   |
| 7 | RSS Safety         | `rss/*`                 | Intel RSS      | Deferred |

**Rationale for Deferral**:
- Require external service APIs (InvertedAI)
- Require proprietary SDKs (NVIDIA Cosmos)
- Better served by separate integration crates (ROS2)
- Complex C++ library bindings (RSS)

**Documentation Approach**:
- Document integration points in Rust API
- Provide FFI examples where applicable
- Link to external crates when available

---

## Detailed Example Matrix

### Legend

- **Complexity**: ⭐ Simple | ⭐⭐ Moderate | ⭐⭐⭐ Complex
- **Priority**: High (core demos) | Medium (useful) | Low (specialized)
- **Status**: ✅ Done | 🚧 In Progress | 📋 Planned | ⏸️ Deferred

### Complete Matrix

| Example Name                        | Python File                       | Phase | Complexity | Priority | Required APIs                    | External Deps   | Status |
|-------------------------------------|-----------------------------------|-------|------------|----------|----------------------------------|-----------------|--------|
| **Already Implemented**             |                                   |       |            |          |                                  |                 |        |
| Connect                             | `connect.rs`                      | 0     | ⭐         | High     | Client                           | -               | ✅     |
| Blueprints                          | `blueprints.rs`                   | 0     | ⭐         | High     | World, BlueprintLibrary          | -               | ✅     |
| World Info                          | `world_info.rs`                   | 0     | ⭐         | High     | World, Map                       | -               | ✅     |
| Spawn Vehicle                       | `spawn_vehicle.rs`                | 0     | ⭐         | High     | World, Actor                     | -               | ✅     |
| Multiple Vehicles                   | `multiple_vehicles.rs`            | 0     | ⭐         | High     | World, Actor                     | -               | ✅     |
| Spawn Walker                        | `spawn_walker.rs`                 | 0     | ⭐         | High     | World, Walker                    | -               | ✅     |
| Multiple Walkers                    | `multiple_walkers.rs`             | 0     | ⭐         | High     | World, Walker                    | -               | ✅     |
| Vehicle Transform                   | `vehicle_transform.rs`            | 0     | ⭐         | High     | Actor, Transform                 | -               | ✅     |
| Vehicle Attributes                  | `vehicle_attributes.rs`           | 0     | ⭐         | High     | Actor, Attributes                | -               | ✅     |
| Walker Control                      | `walker_control.rs`               | 0     | ⭐         | High     | Walker, WalkerControl            | -               | ✅     |
| Walker Directions                   | `walker_directions.rs`            | 0     | ⭐         | High     | Walker, WalkerControl            | -               | ✅     |
| **Phase 10: Simple Examples**       |                                   |       |            |          |                                  |                 |        |
| Tutorial                            | `tutorial.py`                     | 10    | ⭐         | High     | Actor, Camera, Autopilot         | -               | 📋     |
| Vehicle Gallery                     | `vehicle_gallery.py`              | 10    | ⭐         | Medium   | Blueprints, Spectator            | -               | 📋     |
| Dynamic Weather                     | `dynamic_weather.py`              | 10    | ⭐         | High     | Weather, WorldSettings           | -               | 📋     |
| Vehicle Physics                     | `vehicle_physics.py`              | 10    | ⭐         | Medium   | Vehicle, Physics, Impulse        | -               | 📋     |
| Start Recording                     | `start_recording.py`              | 10    | ⭐         | High     | Recorder, Batch                  | -               | 📋     |
| Start Replaying                     | `start_replaying.py`              | 10    | ⭐         | High     | Recorder                         | -               | 📋     |
| Show File Info                      | `show_recorder_file_info.py`      | 10    | ⭐         | Medium   | Recorder queries                 | -               | 📋     |
| Show Collisions                     | `show_recorder_collisions.py`     | 10    | ⭐         | Medium   | Recorder queries                 | -               | 📋     |
| Show Blocked                        | `show_recorder_actors_blocked.py` | 10    | ⭐         | Medium   | Recorder queries                 | -               | 📋     |
| **Phase 11: Intermediate Examples** |                                   |       |            |          |                                  |                 |        |
| Generate Traffic                    | `generate_traffic.py`             | 11    | ⭐⭐       | High     | Batch, TrafficManager, WalkerAI  | -               | 📋     |
| Sensor Sync                         | `sensor_synchronization.py`       | 11    | ⭐⭐       | High     | Sensors, SyncMode, Queue         | -               | 📋     |
| Synchronous Mode                    | `synchronous_mode.py`             | 11    | ⭐⭐       | High     | SyncMode, Sensor, Camera         | -               | 📋     |
| Automatic Control                   | `automatic_control.py`            | 11    | ⭐⭐       | Medium   | Autopilot, HUD display           | UI framework    | 📋     |
| Lidar to Camera                     | `lidar_to_camera.py`              | 11    | ⭐⭐       | Medium   | Lidar, Camera, Transform         | -               | 📋     |
| Multi-Sensor Viz                    | `visualize_multiple_sensors.py`   | 11    | ⭐⭐       | Medium   | Multiple sensors                 | UI framework    | 📋     |
| Component Test                      | `get_component_test.py`           | 11    | ⭐         | Low      | Component API                    | -               | 📋     |
| Tutorial GBuffer                    | `tutorial_gbuffer.py`             | 11    | ⭐⭐       | Low      | GBuffer, Camera                  | -               | 📋     |
| **Phase 12: Advanced Examples**     |                                   |       |            |          |                                  |                 |        |
| Manual Control                      | `manual_control.py`               | 12    | ⭐⭐⭐     | High     | All vehicle APIs, HUD            | winit, pixels   | 📋     |
| Steering Wheel                      | `manual_control_steeringwheel.py` | 12    | ⭐⭐⭐     | Low      | Vehicle control, Input           | gilrs, winit    | 📋     |
| Bounding Boxes                      | `bounding_boxes.py`               | 12    | ⭐⭐⭐     | Medium   | Camera, Projection, Instance Seg | nalgebra, image | 📋     |
| Client BB                           | `client_bounding_boxes.py`        | 12    | ⭐⭐⭐     | Medium   | Advanced BB, Debug draw          | nalgebra        | 📋     |
| Draw Skeleton                       | `draw_skeleton.py`                | 12    | ⭐⭐⭐     | Low      | Walker bones, Debug draw         | -               | 📋     |
| No Rendering                        | `no_rendering_mode.py`            | 12    | ⭐⭐⭐     | Medium   | Large scale, No render mode      | -               | 📋     |
| Open3D Lidar                        | `open3d_lidar.py`                 | 12    | ⭐⭐⭐     | Low      | Lidar, Point cloud               | bevy/3D viz     | 📋     |
| CarSim/Chrono                       | `manual_control_carsim.py`        | 12    | ⭐⭐⭐     | Low      | External physics                 | Physics libs    | 📋     |
| **Phase 13: Specialized/Deferred**  |                                   |       |            |          |                                  |                 |        |
| InvertedAI                          | `invertedai_traffic.py`           | 13    | ⭐⭐⭐     | -        | External API                     | InvertedAI SDK  | ⏸️      |
| Cosmos Gen                          | `carla_cosmos_gen.py`             | 13    | ⭐⭐⭐     | -        | External API                     | NVIDIA Cosmos   | ⏸️      |
| V2X Demo                            | `V2XDemo.py`                      | 13    | ⭐⭐⭐     | -        | V2X protocol                     | V2X libs        | ⏸️      |
| V2X Test                            | `test_addsecondvx.py`             | 13    | ⭐⭐       | -        | V2X protocol                     | V2X libs        | ⏸️      |
| NVIDIA                              | `nvidia/*`                        | 13    | ⭐⭐⭐     | -        | NVIDIA tools                     | NVIDIA SDK      | ⏸️      |
| ROS2                                | `ros2/*`                          | 13    | ⭐⭐⭐     | -        | ROS2 messages                    | ROS2, rclrs     | ⏸️      |
| RSS                                 | `rss/*`                           | 13    | ⭐⭐⭐     | -        | RSS library                      | Intel RSS       | ⏸️      |

---

## API Gap Analysis

### APIs Needed for Phase 10

#### Weather Control
```rust
// Currently missing - needs simple implementation
impl World {
    pub fn get_weather(&self) -> WeatherParameters { }
    pub fn set_weather(&mut self, weather: &WeatherParameters) { }
}

pub struct WeatherParameters {
    pub cloudiness: f32,
    pub precipitation: f32,
    pub precipitation_deposits: f32,
    pub wind_intensity: f32,
    pub sun_azimuth_angle: f32,
    pub sun_altitude_angle: f32,
    pub fog_density: f32,
    pub fog_distance: f32,
    pub wetness: f32,
    // ... etc
}
```

#### Autopilot Control
```rust
// Partially exists, needs verification
impl Vehicle {
    pub fn set_autopilot(&mut self, enabled: bool) { }
    pub fn set_autopilot_tm_port(&mut self, enabled: bool, tm_port: u16) { }
}
```

#### Spectator Camera
```rust
// May exist, needs verification
impl World {
    pub fn get_spectator(&self) -> Actor { }
}
```

### APIs Needed for Phase 11

#### Walker AI Controller
```rust
// Missing - marked as deferred in Phase 1
pub struct WalkerAIController {
    // ...
}

impl WalkerAIController {
    pub fn start(&mut self) { }
    pub fn stop(&mut self) { }
    pub fn go_to_location(&mut self, location: Location) { }
    pub fn set_max_speed(&mut self, speed: f32) { }
}
```

#### Random Location Generation
```rust
// Missing - needed for walker spawning
impl World {
    pub fn get_random_location_from_navigation(&self) -> Option<Location> { }
    pub fn set_pedestrians_cross_factor(&mut self, percentage: f32) { }
    pub fn set_pedestrians_seed(&mut self, seed: u32) { }
}
```

#### Synchronous Mode Utilities
```rust
// Exists but may need ergonomic wrappers
impl World {
    pub fn tick(&mut self) -> FrameId { }
    pub fn wait_for_tick(&self, timeout: Duration) -> WorldSnapshot { }
    pub fn on_tick(&self, callback: impl FnMut(WorldSnapshot)) -> CallbackHandle { }
}
```

### APIs Needed for Phase 12

#### Debug Drawing
```rust
// Planned in Phase 2 - accelerate if needed for examples
impl DebugHelper {
    pub fn draw_point(&mut self, location: Location, size: f32, color: Color, lifetime: f32) { }
    pub fn draw_line(&mut self, begin: Location, end: Location, thickness: f32, color: Color, lifetime: f32) { }
    pub fn draw_box(&mut self, bbox: BoundingBox, rotation: Rotation, thickness: f32, color: Color, lifetime: f32) { }
    pub fn draw_string(&mut self, location: Location, text: &str, color: Color, lifetime: f32) { }
}
```

#### Camera Projection Utilities
```rust
// New module needed for bounding box examples
pub mod camera_projection {
    pub fn build_projection_matrix(width: u32, height: u32, fov: f32) -> Matrix3<f32> { }
    pub fn world_to_camera(point: Location, camera_transform: &Transform) -> Vector3<f32> { }
    pub fn project_to_2d(point_3d: Vector3<f32>, k_matrix: &Matrix3<f32>) -> (f32, f32) { }
}
```

#### Instance Segmentation Decoding
```rust
// Utility for bounding box detection
impl SemanticSegmentationImage {
    pub fn decode_instance_ids(&self) -> (Vec<u8>, Vec<u16>) {
        // Returns (semantic_labels, actor_ids)
    }
}
```

---

## Rust Implementation Guidelines

### Project Structure

```
carla/examples/
  ├── basic/           # Phase 10 examples
  │   ├── tutorial.rs
  │   ├── dynamic_weather.rs
  │   └── ...
  ├── intermediate/    # Phase 11 examples
  │   ├── generate_traffic.rs
  │   ├── sensor_sync.rs
  │   └── ...
  ├── advanced/        # Phase 12 examples
  │   ├── manual_control.rs
  │   ├── bounding_boxes.rs
  │   └── ...
  └── utils/           # Shared utilities
      ├── mod.rs
      ├── camera_projection.rs
      ├── sensor_queue.rs
      └── hud.rs
```

### Error Handling Pattern

```rust
// Recommended pattern for examples
use anyhow::{Context, Result};

fn main() -> Result<()> {
    println!("Connecting to CARLA simulator...");
    let client = Client::connect("localhost", 2000, None)
        .context("Failed to connect to simulator")?;

    let mut world = client.world();

    // Use ? operator for clean error propagation
    let blueprint_library = world.blueprint_library();
    let vehicle_bp = blueprint_library
        .find("vehicle.tesla.model3")
        .context("Tesla Model 3 blueprint not found")?;

    // ... example code ...

    Ok(())
}
```

**Benefits**:
- Clear error messages with context
- Consistent `Result` return types
- Compatible with `?` operator
- User-friendly stack traces

### Sensor Callback Patterns

#### Synchronous Queue Pattern (Recommended for Examples)

```rust
use std::sync::mpsc::{channel, Receiver};

fn setup_camera(world: &mut World, vehicle: &Vehicle) -> Receiver<CameraImage> {
    let (tx, rx) = channel();

    let camera_bp = world.blueprint_library().find("sensor.camera.rgb")?;
    let camera = world.spawn_actor(
        &camera_bp,
        &Transform::default(),
        Some(vehicle),
    )?;

    // Sensor listen returns a callback handle
    camera.listen(move |data| {
        if let Ok(image) = data.try_into() {
            let _ = tx.send(image);
        }
    });

    rx
}

fn main() -> Result<()> {
    // ...setup...
    let camera_rx = setup_camera(&mut world, &vehicle)?;

    loop {
        world.tick();

        // Non-blocking receive for latest frame
        if let Ok(image) = camera_rx.try_recv() {
            process_image(image);
        }
    }
}
```

#### Async Pattern (For Future Async API)

```rust
// Future possibility - not currently implemented
async fn sensor_stream(sensor: &Camera) -> impl Stream<Item = CameraImage> {
    // Returns async stream of sensor data
}

#[tokio::main]
async fn main() -> Result<()> {
    let camera = setup_camera()?;
    let mut stream = sensor_stream(&camera);

    while let Some(image) = stream.next().await {
        process_image(image);
    }

    Ok(())
}
```

### Resource Cleanup Pattern

```rust
// RAII-style cleanup with Drop
struct Simulation {
    _client: Client,
    world: World,
    actors: Vec<ActorId>,
}

impl Simulation {
    fn new() -> Result<Self> {
        let client = Client::connect("localhost", 2000, None)?;
        let world = client.world();
        Ok(Self {
            _client: client,
            world,
            actors: Vec::new(),
        })
    }

    fn spawn_vehicle(&mut self, bp: &ActorBlueprint, transform: &Transform) -> Result<Vehicle> {
        let vehicle = self.world.spawn_actor(bp, transform)?;
        self.actors.push(vehicle.id());
        Ok(vehicle)
    }
}

impl Drop for Simulation {
    fn drop(&mut self) {
        println!("Cleaning up {} actors...", self.actors.len());
        for actor_id in &self.actors {
            if let Ok(actor) = self.world.get_actor(*actor_id) {
                let _ = actor.destroy();
            }
        }
    }
}

fn main() -> Result<()> {
    let mut sim = Simulation::new()?;

    // Actors automatically cleaned up when sim drops
    sim.spawn_vehicle(&blueprint, &transform)?;

    // ... run simulation ...

    Ok(()) // cleanup happens here
}
```

### Synchronous Mode Context Manager

```rust
// Similar to Python's context manager pattern
pub struct SyncMode<'a> {
    world: &'a mut World,
    original_settings: WorldSettings,
    sensors: Vec<Sensor>,
    queues: Vec<Receiver<SensorData>>,
}

impl<'a> SyncMode<'a> {
    pub fn new(world: &'a mut World, fps: u32, sensors: Vec<Sensor>) -> Result<Self> {
        let original_settings = world.get_settings();

        let mut settings = original_settings.clone();
        settings.synchronous_mode = true;
        settings.fixed_delta_seconds = Some(1.0 / fps as f64);
        world.apply_settings(settings)?;

        let queues = sensors.iter().map(|sensor| {
            let (tx, rx) = channel();
            sensor.listen(move |data| {
                let _ = tx.send(data);
            });
            rx
        }).collect();

        Ok(Self {
            world,
            original_settings,
            sensors,
            queues,
        })
    }

    pub fn tick(&mut self, timeout: Duration) -> Result<Vec<SensorData>> {
        self.world.tick();

        let mut data = Vec::new();
        for queue in &self.queues {
            data.push(queue.recv_timeout(timeout)?);
        }

        Ok(data)
    }
}

impl Drop for SyncMode<'_> {
    fn drop(&mut self) {
        let _ = self.world.apply_settings(self.original_settings.clone());
    }
}

// Usage
fn main() -> Result<()> {
    let mut world = client.world();
    let camera = spawn_camera(&mut world)?;
    let lidar = spawn_lidar(&mut world)?;

    let mut sync = SyncMode::new(&mut world, 20, vec![camera, lidar])?;

    for _ in 0..100 {
        let data = sync.tick(Duration::from_secs(1))?;
        process_sensor_data(data);
    }

    Ok(()) // sync mode disabled automatically
}
```

### Traffic Manager Integration

```rust
// Batch spawning with Traffic Manager
fn spawn_traffic(client: &Client, world: &mut World, num_vehicles: usize) -> Result<Vec<Vehicle>> {
    let tm = client.get_traffic_manager(8000)?;

    let blueprints = world.blueprint_library().filter("vehicle.*");
    let spawn_points = world.map().get_spawn_points();

    let mut batch = Vec::new();
    for (i, transform) in spawn_points.iter().enumerate().take(num_vehicles) {
        let bp = &blueprints[i % blueprints.len()];
        batch.push(Command::SpawnActor {
            blueprint: bp.clone(),
            transform: *transform,
            attach_to: None,
        }.then(Command::SetAutopilot {
            actor: FutureActor,
            enabled: true,
            tm_port: 8000,
        }));
    }

    let responses = client.apply_batch_sync(&batch)?;

    let vehicles: Vec<_> = responses
        .into_iter()
        .filter_map(|r| r.actor_id)
        .filter_map(|id| world.get_actor(id).ok()?.try_into().ok())
        .collect();

    Ok(vehicles)
}
```

---

## External Dependencies

### Recommended Crates

#### Core Dependencies (Already Used)
- `carla` - Main CARLA Rust API
- `anyhow` - Error handling
- `log` / `env_logger` - Logging

#### UI and Visualization (Phase 11-12)
- **winit** (v0.29+) - Cross-platform windowing
  - Use: Window creation, event handling
  - Examples: manual_control, synchronous_mode

- **pixels** (v0.13+) - Simple 2D rendering
  - Use: Displaying camera images, HUD overlays
  - Examples: manual_control, bounding_boxes

- **bevy** (v0.13+) - Full-featured game engine (optional)
  - Use: 3D visualization, complex UIs
  - Examples: open3d_lidar (if implemented)

#### Math and Linear Algebra
- **nalgebra** (v0.32+) - Robust linear algebra
  - Use: Projection matrices, transformations
  - Examples: bounding_boxes, lidar_to_camera

- **glam** (v0.24+) - Simpler, faster alternative
  - Use: Basic vector/matrix ops
  - Choose one: nalgebra (feature-rich) OR glam (performance)

#### Input Handling
- **gilrs** (v0.10+) - Game controller input
  - Use: Steering wheel, gamepad support
  - Examples: manual_control_steeringwheel

#### Image Processing
- **image** (v0.24+) - Image encoding/decoding
  - Use: Saving sensor images, format conversion
  - Examples: tutorial, bounding_boxes

#### Serialization
- **serde** (v1.0+) - Already used for JSON
  - Use: Saving bounding box annotations
  - Examples: bounding_boxes

#### Async (Future)
- **tokio** (v1.35+) - Async runtime (if async sensors added)
  - Use: Async sensor streams
  - Status: Future enhancement

### Dependency Strategy

1. **Minimize Dependencies**
   - Start with simple examples using only `carla` + `anyhow`
   - Add UI/math crates only when needed

2. **Feature Gates**
   - Use Cargo features for optional examples
   - Example: `[features] manual-control = ["winit", "pixels"]`

3. **Workspace Organization**
   - Keep examples in `carla/examples/`
   - Complex examples can be workspace members if needed

### Python Library Replacements

| Python Library | Rust Replacement | Notes |
|---------------|------------------|-------|
| pygame | winit + pixels | Event loop + pixel buffer |
| numpy | nalgebra/ndarray | Matrix ops, arrays |
| opencv-python | opencv-rust | Computer vision (if needed) |
| open3d | bevy/kiss3d | 3D visualization |
| queue.Queue | std::sync::mpsc | Thread-safe channel |
| argparse | clap | CLI argument parsing |
| logging | log + env_logger | Structured logging |

**Note**: Some Python libraries (pygame's specific features) don't have direct Rust equivalents. Design examples to be Rust-idiomatic rather than literal ports.

---

## Testing Strategy

### Example Testing Approach

1. **Compile-Time Tests**
   ```bash
   # Ensure all examples compile
   cargo check --examples
   cargo build --examples --profile dev-release
   ```

2. **Runtime Tests** (with simulator)
   ```bash
   # Use the existing run-examples.sh script
   ./scripts/run-examples.sh --version 0.9.16
   ```

3. **Documentation Tests**
   - Include doc tests in example files
   - Test small code snippets

### Example Template

```rust
//! Example Name
//!
//! Description of what this example demonstrates.
//!
//! Prerequisites:
//! - CARLA simulator must be running
//! - Map: Town10HD_Opt (default)
//!
//! Run with:
//! ```bash
//! cargo run --example example_name
//! ```

use carla::client::{Client, ActorBase};
use anyhow::{Context, Result};

fn main() -> Result<()> {
    env_logger::init();

    println!("Example: Example Name");
    println!("Connecting to CARLA simulator...");

    let client = Client::connect("localhost", 2000, None)
        .context("Failed to connect to CARLA simulator")?;

    println!("Connected! Server version: {}", client.get_server_version());

    // Example code here

    println!("\nExample completed successfully!");
    Ok(())
}
```

### Integration with CI

- Examples in Phase 10-11: Add to `run-examples.sh`
- Examples in Phase 12: Require manual testing (UI)
- Mark UI examples with `#[cfg(feature = "ui-examples")]`

---

## Next Steps

1. **Implement Phase 10 Examples**
   - Start with `tutorial.rs` (most fundamental)
   - Add `dynamic_weather.rs` (popular feature)
   - Complete recording examples

2. **Address API Gaps**
   - Implement Weather API
   - Complete Recording API (Phase 3)
   - Add missing physics methods

3. **Create Shared Utilities**
   - Camera projection helpers
   - Sensor queue abstraction
   - HUD rendering utilities

4. **Update Documentation**
   - Add examples to `carla/examples/README.md`
   - Link from main crate documentation
   - Create beginner tutorials

---

## Revision History

| Date | Version | Changes |
|------|---------|---------|
| 2025-10-28 | 1.0 | Initial document creation |

---

## References

- [CARLA Python Examples](https://github.com/carla-simulator/carla/tree/master/PythonAPI/examples)
- [CARLA Python API Docs](https://carla.readthedocs.io/en/latest/python_api/)
- [carla-rust Roadmap](./roadmap.md)
- [Rust API Design Guidelines](https://rust-lang.github.io/api-guidelines/)
