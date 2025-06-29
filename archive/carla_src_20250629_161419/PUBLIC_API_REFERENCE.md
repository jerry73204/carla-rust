# CARLA Rust Public API Reference

**Library Version**: As archived on 2025-06-29  
**Target CARLA Version**: 0.10.0

## Overview

This document captures the public API surface of the carla crate at the time of archiving. The API follows CARLA's C++ client library structure exactly, providing a safe Rust interface.

## Core Type Aliases

```rust
pub type Result<T, E = CarlaError> = std::result::Result<T, E>;
```

## Public Constants

```rust
pub const VERSION: &str = env!("CARGO_PKG_VERSION");
pub const CARLA_VERSION: &str = "0.10.0";
```

## Public Modules

### 1. `actor` - Actor Management
Manages all entities in the simulation including vehicles, pedestrians, sensors, and traffic infrastructure.

**Key Types**:
- Base actor traits and functionality
- `Vehicle` - Car, truck, motorcycle actors
- `Walker` - Pedestrian actors
- `TrafficLight` - Traffic light control
- `TrafficSign` - Static traffic signs
- Sensor actors:
  - `Camera` - RGB, depth, semantic segmentation
  - `Lidar` - 3D point cloud sensor
  - `Radar` - Radar sensor
  - `Gnss` - GPS sensor
  - `Imu` - Inertial measurement unit
  - `CollisionSensor` - Collision detection
  - `LaneInvasionSensor` - Lane crossing detection
  - `ObstacleDetectionSensor` - Obstacle detection
  - `DvsCameraSensor` - Dynamic vision sensor
  - `RssSensor` - Road safety sensor

### 2. `batch` - Batch Commands
Executes multiple commands atomically.

**Key Types**:
- Batch command execution utilities

### 3. `client` - Client Connection
Main entry point for connecting to CARLA server.

**Key Types**:
- `Client` - Main client connection
- `World` - Simulation world management
- `ActorBlueprint` - Actor templates
- `BlueprintLibrary` - Blueprint collection

**Re-exported**: `pub use client::Client;`

### 4. `geom` - Geometry Types
3D geometry primitives for positions and transforms.

**Key Types**:
- `Location` - 3D position (x, y, z)
- `Rotation` - 3D rotation (pitch, yaw, roll)
- `Transform` - Combined location and rotation
- `Vector3D` - 3D vector
- `BoundingBox` - Axis-aligned bounding box

### 5. `road` - Road Network
OpenDRIVE road network navigation.

**Key Types**:
- `Map` - Road network map
- `Waypoint` - Navigation points
- `Junction` - Road junctions
- `Lane` - Lane information
- `Landmark` - Road landmarks and signs
- Lane types and marking enums

### 6. `sensor_data` - Sensor Data
Data structures for sensor outputs.

**Key Types**:
- `Image` - Camera image data
- `LidarMeasurement` - Point cloud data
- `RadarMeasurement` - Radar detections
- `GnssMeasurement` - GPS data
- `ImuMeasurement` - IMU data
- `CollisionEvent` - Collision information
- `LaneInvasionEvent` - Lane crossing events
- `ObstacleDetectionEvent` - Obstacle detections
- Callback handling for asynchronous data

### 7. `streaming` - Data Streaming
Sensor data streaming infrastructure.

**Key Types**:
- `SensorStream` - Asynchronous sensor data stream

### 8. `time` - Time Management
Simulation time handling.

**Key Types**:
- `Timestamp` - Simulation timestamps
- Time utilities

### 9. `traffic_manager` - Traffic Control
AI traffic simulation control.

**Key Types**:
- `TrafficManager` - Traffic behavior control
- Traffic management configuration types

### 10. `error` - Error Handling
Comprehensive error types.

**Key Types**:
- `CarlaError` - Main error type

**Re-exported**: `pub use error::CarlaError;`

### 11. `traits` - Core Traits
Trait definitions for actors and other types.

**Key Traits**:
- `ActorT` - Base actor trait
- `VehicleT` - Vehicle-specific operations
- `WalkerT` - Pedestrian operations
- `SensorT` - Sensor operations
- Other specialized traits

**Re-exported**: 
- `pub use actor::{ActorExt, SensorExt};`

### 12. `utils` - Utilities
Helper functions and types.

## Import Patterns

The library supports flexible import patterns:

```rust
// Direct module imports
use carla::client::Client;
use carla::actor::vehicle::Vehicle;
use carla::geom::{Location, Transform};

// Re-exported common types
use carla::{Client, CarlaError, Result};

// Trait imports for extension methods
use carla::traits::{ActorT, VehicleT};
use carla::{ActorExt, SensorExt};
```

## Thread Safety

**Important**: The library uses `Rc` instead of `Arc` because CARLA's C++ client is not thread-safe. Types are not `Send` or `Sync`. For multi-threaded usage, wrap in `Arc<Mutex<T>>` or use channels.

## Features

- `codegen` - Enables code generation features (when available)

## Architecture Notes

1. **Zero-cost abstractions** over CXX FFI layer
2. **1:1 mapping** with CARLA C++ API
3. **RAII** for automatic resource management
4. **Result-based** error handling throughout
5. **Type-safe** actor conversions and operations