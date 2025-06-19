# CARLA Rust Client Implementation Progress

## Overview
Tracking progress of CARLA Rust client implementation across two layers:
- **FFI (carla-sys)**: Low-level FFI bindings using CXX
- **Rust API (carla)**: High-level idiomatic Rust interface

## Architecture
```
carla (Rust API) -> carla-sys (FFI/CXX) -> CARLA C++ Client Library
```

## Recent Updates (2025-01)
- Migrated from autocxx to CXX for better stability and performance
- Updated to CARLA 0.10.0 (from 0.9.14)
- Simplified architecture to 2 main crates instead of 3

## Feature Implementation Status

### Core Client
| Feature           | FFI Status  | Rust API Status | Tests       |
|-------------------|-------------|-----------------|-------------|
| Client connection | ✅ Complete | ✅ Complete     | ⚠️  Basic    |
| World management  | ✅ Complete | ✅ Complete     | ⚠️  Basic    |
| Blueprint library | ✅ Complete | ✅ Complete     | ✅ Complete |
| Actor spawning    | ✅ Complete | ✅ Complete     | ⚠️  Basic    |
| Recording/Replay  | ✅ Complete | ✅ Complete     | ❌ None     |
| Map loading       | ✅ Complete | ✅ Complete     | ❌ None     |

### Actors
| Actor Type    | FFI Status  | Rust API Status | Tests   |
|---------------|-------------|-----------------|---------|
| Base Actor    | ✅ Complete | ✅ Complete     | ❌ None |
| Vehicle       | ✅ Complete | ✅ Complete     | ❌ None |
| Walker        | ✅ Complete | ✅ Complete     | ❌ None |
| Traffic Light | ✅ Complete | ✅ Complete     | ⚠️  Basic |
| Traffic Sign  | ✅ Complete | ✅ Complete     | ❌ None |

### Sensors
| Sensor Type            | FFI Status  | Rust API Status | Tests   |
|------------------------|-------------|-----------------|---------|
| Base Sensor            | ✅ Complete | ✅ Complete     | ❌ None |
| Camera (RGB/Depth/Seg) | ✅ Complete | ✅ Complete     | ❌ None |
| LiDAR                  | ✅ Complete | ✅ Complete     | ❌ None |
| Radar                  | ✅ Complete | ✅ Complete     | ❌ None |
| GNSS                   | ✅ Complete | ✅ Complete     | ❌ None |
| IMU                    | ✅ Complete | ✅ Complete     | ❌ None |
| Collision              | ✅ Complete | ✅ Complete     | ❌ None |
| Lane Invasion          | ✅ Complete | ✅ Complete     | ❌ None |
| DVS                    | ✅ Complete | ✅ Complete     | ❌ None |
| Obstacle Detection     | ✅ Complete | ✅ Complete     | ❌ None |
| RSS                    | ✅ Complete | ✅ Complete     | ❌ None |

### Traffic Manager
| Feature          | FFI Status  | Rust API Status | Tests   |
|------------------|-------------|-----------------|---------|
| Basic control    | ✅ Complete | ✅ Complete     | ❌ None |
| Vehicle behavior | ✅ Complete | ✅ Complete     | ❌ None |
| Route planning   | ✅ Complete | ✅ Complete     | ❌ None |
| Synchronous mode | ✅ Complete | ✅ Complete     | ❌ None |

### Road/Map
| Feature   | FFI Status  | Rust API Status | Tests   |
|-----------|-------------|-----------------|---------|
| Map info  | ✅ Complete | ✅ Complete     | ❌ None |
| Waypoints | ✅ Complete | ✅ Complete     | ❌ None |
| Lanes     | ✅ Complete | ✅ Complete     | ❌ None |
| Junctions | ✅ Complete | ✅ Complete     | ❌ None |
| OpenDRIVE | ✅ Complete | ✅ Complete     | ❌ None |
| Landmarks | ✅ Complete | ✅ Complete     | ❌ None |
| Topology  | ✅ Complete | ✅ Complete     | ❌ None |

### Utilities
| Feature         | FFI Status  | Rust API Status | Tests   |
|-----------------|-------------|-----------------|---------|
| Geometry types  | ✅ Complete | ✅ Complete     | ❌ None |
| Weather         | ✅ Complete | ✅ Complete     | ❌ None |
| Physics control | ✅ Complete | ✅ Complete     | ❌ None |
| Light Manager   | ❌ Removed* | ❌ Removed*     | N/A     |

*Deprecated in CARLA 0.10.0

## TODO Priority List

### High Priority
1. **Sensor Data Retrieval**
   - [x] Implement Sensor_GetFrame FFI function
   - [x] Implement Sensor_GetTimestamp FFI function
   - [x] Complete data() methods for all sensor types
   - [ ] Add sensor data callbacks/streaming

2. **Collection Wrapper Performance**
   - [x] Implement zero-copy WaypointList wrapper
   - [x] Implement zero-copy TransformList wrapper
   - [x] Implement zero-copy LocationList wrapper
   - [x] Implement zero-copy Topology wrapper
   - [ ] Add benchmarks for wrapper performance

### Medium Priority
4. **Test Coverage**
   - [ ] Add integration tests for all actor types
   - [ ] Add Traffic Manager tests
   - [ ] Add sensor data validation tests
   - [ ] Add async/streaming tests

5. **Documentation and Examples**
   - [ ] Update examples to use new public submodule imports
   - [ ] Add examples for waypoint navigation
   - [ ] Add examples for Traffic Manager usage
   - [ ] Document zero-copy wrapper pattern

### Low Priority
6. **Advanced Features**
   - [ ] Walker bone control implementation
   - [ ] OpenDRIVE parsing and generation
   - [ ] Custom sensor plugin support
   - [ ] ROS2 integration bindings

## Known Issues
- Sensor attribute retrieval requires FFI functions
- Walker bone names not exposed through FFI
- Some sensor data types have todo!() placeholders
- No async/streaming support for sensors yet
- Map::junctions() requires additional FFI support
- Map::transform_from_geolocation() not yet implemented

## Testing Requirements
- Unit tests for each module
- Integration tests requiring CARLA server
- Performance benchmarks for sensor data
- Memory safety validation tests
