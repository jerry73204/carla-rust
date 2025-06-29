# Carla Crate Source Archive

**Archive Date**: 2025-06-29 16:14:19 UTC  
**Source Location**: `/home/aeon/repos/carla-rust/carla/src`  
**Archive Location**: `/home/aeon/repos/carla-rust/archive/carla_src_20250629_161419`

## Archive Contents

This archive contains the complete source code of the carla crate implementation as of the archive date.

### Module Structure

1. **Core Modules**
   - `lib.rs` - Main library entry point
   - `error.rs` - Error types and handling
   - `traits.rs` - Core trait definitions
   - `batch.rs` - Batch command execution
   - `time.rs` - Time handling utilities

2. **Client API** (`client/`)
   - `client.rs` - Main client module
   - `client_impl.rs` - Client implementation
   - `blueprint.rs` - Blueprint definitions
   - `blueprint_library.rs` - Blueprint library
   - `world.rs` - World management

3. **Actor System** (`actor/`)
   - `base.rs` - Base actor functionality
   - `vehicle.rs` - Vehicle actors
   - `walker.rs` - Pedestrian actors
   - `traffic_light.rs` - Traffic light actors
   - `traffic_sign.rs` - Traffic sign actors
   - Various sensor actors (camera, lidar, radar, etc.)

4. **Sensor Data** (`sensor_data/`)
   - Data structures for all sensor types
   - Callback handling system
   - Image, LiDAR, radar, and other sensor data formats

5. **Road Network** (`road/`)
   - `map.rs` - Map management
   - `waypoint.rs` - Waypoint navigation
   - `junction.rs` - Junction handling
   - `lane.rs` - Lane information
   - `landmark.rs` - Road landmarks

6. **Geometry** (`geom/`)
   - `location.rs` - 3D position
   - `rotation.rs` - 3D rotation
   - `transform.rs` - Combined transform
   - `vector.rs` - Vector operations
   - `bounding_box.rs` - Collision bounds

7. **Traffic Management** (`traffic_manager/`)
   - `manager.rs` - Traffic manager implementation
   - `types.rs` - Traffic manager types

8. **Streaming** (`streaming/`)
   - `sensor_stream.rs` - Sensor data streaming

### File Count
Total archived files: 65 Rust source files

### Purpose
This archive was created to preserve the current state of the carla crate implementation before major refactoring or API changes.