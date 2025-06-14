# CARLA Rust Library Progress Report

## Overview

This document tracks the progress of creating a complete Rust API that mirrors the C++ API structure for CARLA simulator version 0.10.0. The architecture is now based on a two-layer approach:

1. **CXX Direct FFI** (`carla-cxx`) - Low-level CXX bindings to CARLA C++ API
2. **High-Level Rust API** (`carla`) - Idiomatic Rust API mirroring C++ CARLA structure

## New Architecture Proposal

### Layer Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    carla (High-Level API)                   │
│  ┌─────────────────┬─────────────────┬─────────────────┐    │
│  │  carla::client  │  carla::sensor  │ carla::traffic  │    │
│  │     ::World     │     ::Camera    │   ::TrafficMgr  │    │
│  │     ::Actor     │     ::Lidar     │   ::TrafficLt   │    │
│  │     ::Vehicle   │     ::Radar     │                 │    │
│  │     ::Walker    │     ::IMU       │                 │    │
│  └─────────────────┴─────────────────┴─────────────────┘    │
├─────────────────────────────────────────────────────────────┤
│                   carla-cxx (FFI Layer)                     │
│  ┌─────────────────────────────────────────────────────────┐ │
│  │            CXX Bridge to CARLA C++ API                 │ │
│  │  • Direct C++ interop without C wrapper               │ │
│  │  • Memory-safe through CXX abstractions               │ │
│  │  • Exception → Result conversion                      │ │
│  └─────────────────────────────────────────────────────────┘ │
├─────────────────────────────────────────────────────────────┤
│                    CARLA C++ Library                        │
│            (carla::client, carla::sensor, etc.)             │
└─────────────────────────────────────────────────────────────┘
```

### New `carla/` Crate Structure

The new `carla/` crate will mirror the C++ CARLA API structure exactly, providing an idiomatic Rust interface:

```
carla/
├── src/
│   ├── lib.rs                    // Main crate exports, mirroring carla:: namespace
│   ├── client/                   // carla::client equivalents
│   │   ├── mod.rs
│   │   ├── client.rs            // carla::client::Client
│   │   ├── world.rs             // carla::client::World  
│   │   ├── actor.rs             // carla::client::Actor (base)
│   │   ├── vehicle.rs           // carla::client::Vehicle
│   │   ├── walker.rs            // carla::client::Walker
│   │   ├── sensor.rs            // carla::client::Sensor
│   │   ├── traffic_light.rs     // carla::client::TrafficLight
│   │   ├── traffic_sign.rs      // carla::client::TrafficSign
│   │   ├── blueprint.rs         // carla::client::ActorBlueprint
│   │   └── blueprint_library.rs // carla::client::BlueprintLibrary
│   ├── sensor/                  // carla::sensor equivalents
│   │   ├── mod.rs
│   │   ├── data.rs             // carla::sensor::data
│   │   ├── camera.rs           // Camera sensor implementations
│   │   ├── lidar.rs            // LiDAR sensor implementations  
│   │   ├── radar.rs            // Radar sensor implementations
│   │   ├── imu.rs              // IMU sensor implementations
│   │   ├── gnss.rs             // GNSS sensor implementations
│   │   ├── collision.rs        // Collision detector
│   │   └── lane_invasion.rs    // Lane invasion detector
│   ├── traffic_manager/         // carla::traffic_manager equivalents
│   │   ├── mod.rs
│   │   ├── traffic_manager.rs  // carla::traffic_manager::TrafficManager
│   │   └── types.rs            // Traffic manager specific types
│   ├── road/                   // carla::road equivalents
│   │   ├── mod.rs
│   │   ├── map.rs              // carla::road::Map
│   │   ├── waypoint.rs         // carla::road::element::Waypoint
│   │   ├── junction.rs         // carla::road::Junction
│   │   ├── lane.rs             // carla::road::Lane
│   │   └── road_types.rs       // Road-related enums and types
│   ├── geom/                   // carla::geom equivalents
│   │   ├── mod.rs
│   │   ├── location.rs         // carla::geom::Location
│   │   ├── rotation.rs         // carla::geom::Rotation
│   │   ├── transform.rs        // carla::geom::Transform
│   │   ├── vector.rs           // carla::geom::Vector2D, Vector3D
│   │   └── bounding_box.rs     // carla::geom::BoundingBox
│   ├── rpc/                    // carla::rpc equivalents
│   │   ├── mod.rs
│   │   ├── actor_id.rs         // carla::rpc::ActorId
│   │   ├── vehicle_control.rs  // carla::rpc::VehicleControl
│   │   ├── walker_control.rs   // carla::rpc::WalkerControl
│   │   └── command.rs          // carla::rpc::Command
│   ├── time/                   // carla::time equivalents
│   │   ├── mod.rs
│   │   └── timestamp.rs        // carla::time::Timestamp
│   └── streaming/              // carla::streaming (if needed)
│       ├── mod.rs
│       └── sensor_stream.rs    // Sensor data streaming
├── examples/                   // Usage examples
│   ├── getting_started.rs
│   ├── spawn_vehicles.rs
│   ├── sensor_tutorial.rs
│   └── traffic_management.rs
└── tests/                      // Integration tests
    ├── client_tests.rs
    ├── sensor_tests.rs
    └── traffic_tests.rs
```

### Design Principles

#### 1. Mirror C++ API Structure
- **Namespace Mapping**: `carla::client::Client` → `carla::client::Client`
- **Method Names**: C++ snake_case → Rust snake_case (natural mapping)
- **Type Hierarchy**: Preserve C++ inheritance through Rust traits
- **Error Handling**: C++ exceptions → Rust `Result<T, E>`

#### 2. Rust-Specific Improvements
- **Memory Safety**: Automatic through Rust ownership
- **Type Safety**: Strong typing for IDs, handles, and state
- **Error Handling**: Comprehensive `Result` types with detailed error info
- **Iterators**: Rust iterators for collections (actor lists, waypoints, etc.)
- **Traits**: Common behavior through traits (e.g., `ActorT`, `SensorT`)

#### 3. API Design Patterns

```rust
// Example API structure mirroring C++

// carla::client::Client equivalent
pub struct Client {
    inner: carla_cxx::ClientWrapper,
}

impl Client {
    pub fn new(host: &str, port: u16, worker_threads: Option<usize>) -> Result<Self, ClientError> {
        let inner = carla_cxx::ClientWrapper::new(host, port)?;
        // Configure worker threads if specified
        Ok(Self { inner })
    }
    
    pub fn get_world(&self) -> World {
        World::from(self.inner.get_world())
    }
    
    pub fn get_available_maps(&self) -> Result<Vec<String>, ClientError> {
        self.inner.get_available_maps()
    }
    
    pub fn load_world(&self, map_name: &str) -> Result<World, ClientError> {
        let world = self.inner.load_world(map_name)?;
        Ok(World::from(world))
    }
}

// carla::client::World equivalent
pub struct World {
    inner: carla_cxx::WorldWrapper,
}

impl World {
    pub fn get_blueprint_library(&self) -> BlueprintLibrary {
        BlueprintLibrary::from(self.inner.get_blueprint_library())
    }
    
    pub fn spawn_actor(
        &self, 
        blueprint: &ActorBlueprint,
        transform: &Transform,
        attach_to: Option<&Actor>
    ) -> Result<Actor, SpawnError> {
        let actor = self.inner.spawn_actor(blueprint, transform, attach_to)?;
        Ok(Actor::from(actor))
    }
    
    pub fn get_actors(&self) -> ActorList {
        ActorList::from(self.inner.get_actors())
    }
    
    pub fn tick(&self, timeout: Option<Duration>) -> Result<WorldSnapshot, WorldError> {
        let snapshot = self.inner.tick(timeout.unwrap_or(Duration::from_secs(2)))?;
        Ok(WorldSnapshot::from(snapshot))
    }
}

// carla::client::Actor base with type-safe downcasting
pub struct Actor {
    inner: carla_cxx::ActorWrapper,
}

impl Actor {
    pub fn get_id(&self) -> ActorId { /* ... */ }
    pub fn get_type_id(&self) -> String { /* ... */ }
    pub fn get_transform(&self) -> Transform { /* ... */ }
    pub fn set_transform(&self, transform: &Transform) -> Result<(), ActorError> { /* ... */ }
    
    // Type-safe downcasting
    pub fn as_vehicle(&self) -> Option<Vehicle> {
        if self.inner.is_vehicle() {
            Some(Vehicle { inner: self.inner.clone() })
        } else {
            None
        }
    }
    
    pub fn as_walker(&self) -> Option<Walker> { /* ... */ }
    pub fn as_sensor(&self) -> Option<Sensor> { /* ... */ }
}

// carla::client::Vehicle with vehicle-specific methods
pub struct Vehicle {
    inner: carla_cxx::ActorWrapper, // or VehicleWrapper if specialized
}

impl Vehicle {
    pub fn apply_control(&self, control: &VehicleControl) -> Result<(), VehicleError> {
        self.inner.apply_vehicle_control(control)
    }
    
    pub fn get_control(&self) -> VehicleControl {
        self.inner.get_vehicle_control()
    }
    
    pub fn set_autopilot(&self, enabled: bool, traffic_manager_port: Option<u16>) -> Result<(), VehicleError> {
        self.inner.set_autopilot(enabled, traffic_manager_port.unwrap_or(8000))
    }
}

// Type-safe error handling
#[derive(Debug, thiserror::Error)]
pub enum ClientError {
    #[error("Connection failed: {0}")]
    ConnectionFailed(String),
    #[error("Invalid host: {0}")]
    InvalidHost(String),
    #[error("Timeout: {0}")]
    Timeout(Duration),
}

#[derive(Debug, thiserror::Error)]
pub enum SpawnError {
    #[error("Spawn location occupied")]
    LocationOccupied,
    #[error("Invalid blueprint: {0}")]
    InvalidBlueprint(String),
    #[error("World error: {0}")]
    WorldError(#[from] WorldError),
}
```

#### 4. Trait-Based Architecture

```rust
// Common actor behaviors
pub trait ActorT {
    fn get_id(&self) -> ActorId;
    fn get_transform(&self) -> Transform;
    fn set_transform(&self, transform: &Transform) -> Result<(), ActorError>;
    fn destroy(&self) -> Result<(), ActorError>;
}

// Sensor-specific behaviors  
pub trait SensorT: ActorT {
    type DataType: SensorData;
    
    fn listen<F>(&self, callback: F) -> Result<(), SensorError>
    where F: Fn(Self::DataType) + Send + 'static;
    
    fn stop(&self) -> Result<(), SensorError>;
    fn is_listening(&self) -> bool;
}

// Vehicle-specific behaviors
pub trait VehicleT: ActorT {
    fn apply_control(&self, control: &VehicleControl) -> Result<(), VehicleError>;
    fn get_control(&self) -> VehicleControl;
    fn get_velocity(&self) -> Vector3D;
    fn get_acceleration(&self) -> Vector3D;
}
```

## Current Status (January 2025)

**Overall Progress**: ~30% complete (carla-cxx complete, new carla architecture proposed)

### Recent Updates
- 🔄 **New Architecture Proposed** (January 2025) - Complete redesign of carla crate to mirror C++ API
- ✅ **Old carla Crate Removed** (January 2025) - Clean slate for new implementation
- ✅ **CXX Foundation Complete** (January 2025) - carla-cxx provides solid FFI foundation
- ✅ **C++ API Analysis** (January 2025) - Comprehensive mapping of C++ to Rust structures

### Implementation Status

#### ✅ Foundation Complete
- **carla-cxx FFI Layer**: 75% complete, major systems implemented
- **Build System**: Stable CXX-based compilation
- **Core Types**: All geometry and basic types implemented
- **Error Handling**: CXX exception to Result conversion working

#### 🔄 New carla Crate (Proposed)
- **Architecture**: Designed to mirror C++ CARLA exactly
- **Modules**: Complete module structure planned
- **Type Safety**: Enhanced with Rust's type system
- **Memory Safety**: Automatic through Rust ownership
- **API Compatibility**: 1:1 mapping with C++ API

## Module Implementation Plan

### Priority 1: Core Infrastructure
| Module | Status | Notes |
|--------|--------|--------|
| **carla::client::Client** | 📝 Planned | Connection management, world loading |
| **carla::client::World** | 📝 Planned | Actor spawning, tick management |
| **carla::client::Actor** | 📝 Planned | Base actor with type-safe downcasting |
| **carla::geom** | 📝 Planned | Transform, Location, Rotation types |

### Priority 2: Actor System  
| Module | Status | Notes |
|--------|--------|--------|
| **carla::client::Vehicle** | 📝 Planned | Vehicle control and physics |
| **carla::client::Walker** | 📝 Planned | Pedestrian simulation |
| **carla::client::Sensor** | 📝 Planned | Base sensor functionality |
| **carla::client::Blueprint** | 📝 Planned | Actor spawning system |

### Priority 3: Sensor System
| Module | Status | Notes |
|--------|--------|--------|
| **carla::sensor::Camera** | 📝 Planned | RGB, depth, semantic cameras |
| **carla::sensor::Lidar** | 📝 Planned | Point cloud sensors |
| **carla::sensor::Radar** | 📝 Planned | Detection sensors |
| **carla::sensor::IMU** | 📝 Planned | Inertial measurement |

### Priority 4: Advanced Features
| Module | Status | Notes |
|--------|--------|--------|
| **carla::traffic_manager** | 📝 Planned | Traffic simulation |
| **carla::road::Map** | 📝 Planned | Road network navigation |
| **carla::road::Waypoint** | 📝 Planned | Path planning |
| **carla::streaming** | 📝 Planned | Sensor data streaming |

## API Design Examples

### Client Usage
```rust
use carla::{Client, Transform, Location, Rotation};

// Connect to CARLA - mirrors carla::client::Client
let client = Client::new("localhost", 2000, None)?;
let world = client.get_world();

// Load a different map
let new_world = client.load_world("Town02")?;
```

### Actor Management
```rust
// Get blueprint library - mirrors C++ exactly
let blueprint_lib = world.get_blueprint_library();
let vehicle_bp = blueprint_lib.find("vehicle.tesla.model3")?;

// Spawn actor with type safety
let spawn_transform = Transform::new(
    Location::new(0.0, 0.0, 0.5),
    Rotation::default()
);

let actor = world.spawn_actor(&vehicle_bp, &spawn_transform, None)?;

// Type-safe downcasting (Rust improvement over C++)
if let Some(vehicle) = actor.as_vehicle() {
    let control = VehicleControl {
        throttle: 0.8,
        steer: 0.0,
        brake: 0.0,
        ..Default::default()
    };
    vehicle.apply_control(&control)?;
}
```

### Sensor Usage
```rust
// Camera sensor with callback - mirrors C++ pattern
let camera_bp = blueprint_lib.find("sensor.camera.rgb")?;
let camera_transform = Transform::new(Location::new(2.0, 0.0, 1.5), Rotation::default());

let camera = world.spawn_actor(&camera_bp, &camera_transform, Some(&vehicle))?
    .as_sensor()
    .expect("Should be sensor");

// Type-safe sensor listening
camera.listen(|image: RGBImage| {
    println!("Received image: {}x{}", image.width(), image.height());
    // Process image data...
})?;
```

### Traffic Manager
```rust
// Traffic manager - direct C++ API mapping
let traffic_manager = client.get_traffic_manager(8000)?;

// Register vehicles for autopilot
traffic_manager.set_global_speed_percentage(-30.0)?; // 30% slower
traffic_manager.register_vehicles(&[&vehicle])?;
vehicle.set_autopilot(true, Some(8000))?;
```

## Implementation Roadmap

### Phase 1: Foundation (2-3 weeks)
1. **Project Setup**: New carla crate structure
2. **Core Types**: Implement carla::geom module 
3. **Client/World**: Basic connection and world management
4. **Actor Base**: Basic actor system with type-safe downcasting

### Phase 2: Actor System (3-4 weeks)  
1. **Vehicle Control**: Complete vehicle implementation
2. **Walker System**: Pedestrian simulation
3. **Blueprint System**: Actor spawning framework
4. **Actor Collections**: Lists and iterators

### Phase 3: Sensor System (4-5 weeks)
1. **Camera Sensors**: RGB, depth, semantic segmentation
2. **LiDAR/Radar**: Point cloud and detection sensors
3. **Sensor Callbacks**: Async data streaming
4. **Sensor Synchronization**: Multi-sensor coordination

### Phase 4: Advanced Features (4-6 weeks)
1. **Traffic Manager**: Complete traffic simulation
2. **Map System**: Road network and waypoints  
3. **Performance**: Optimization and benchmarking
4. **Documentation**: Complete API documentation

### Phase 5: Polish (2-3 weeks)
1. **Examples**: Comprehensive example collection
2. **Testing**: Integration test suite
3. **Error Handling**: Comprehensive error types
4. **Documentation**: Tutorials and guides

## Technical Decisions

### Memory Management
- **Rust Ownership**: Automatic memory safety through Rust's ownership system
- **CXX Bridge**: Safe C++ object lifetime management through CXX
- **RAII**: Automatic cleanup of CARLA resources through Drop trait

### Error Handling
- **Result Types**: All fallible operations return `Result<T, E>`
- **Error Hierarchy**: Structured error types using `thiserror`
- **Error Context**: Rich error information with context chains

### Async/Threading
- **Sensor Callbacks**: Thread-safe callbacks using channels
- **World Ticking**: Async-compatible world simulation
- **Connection Management**: Async client connections when beneficial

### Type Safety
- **Strong Types**: Wrapper types for IDs, handles, and state
- **Trait System**: Behavior-based abstractions
- **Compile-time Safety**: Prevent common C++ API misuse

## Testing Strategy

### Unit Tests
- **Individual Modules**: Test each module in isolation
- **Type Conversions**: Verify C++ ↔ Rust type conversions
- **Error Handling**: Ensure proper error propagation

### Integration Tests  
- **End-to-End**: Complete workflows from client to sensors
- **Performance**: Benchmarks against C++ client
- **Memory Safety**: Stress tests for memory leaks

### Example Programs
- **Tutorial Examples**: Step-by-step learning examples
- **Real-world Usage**: Complex simulation scenarios
- **Performance Demos**: Optimized usage patterns

## Migration Path

### For Existing Users
- **Clear Migration Guide**: Step-by-step migration from old API
- **Compatibility Layer**: Temporary facade for critical migrations
- **Documentation**: Side-by-side API comparisons

### Breaking Changes
- **v0.13.0**: Complete API redesign (major version bump)
- **Clean Slate**: No backward compatibility (justified by major improvements)
- **Future Stability**: Commitment to API stability post-v1.0

---

*Last Updated: January 2025*  
*Status: Architecture Redesigned, Implementation Starting*  
*Target: Full C++ API Parity with Rust Improvements*