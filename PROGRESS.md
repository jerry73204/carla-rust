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
- Migrated test framework from `cargo test` to `cargo nextest`
- Added ActorExt trait implementations for all sensor types

## Feature Implementation Status

### Core Client
| Feature           | FFI Status  | Rust API Status | Tests       |
|-------------------|-------------|-----------------|-------------|
| Client connection | ✅ Complete | ✅ Complete     | ✅ Written   |
| World management  | ✅ Complete | ✅ Complete     | ✅ Written   |
| Blueprint library | ✅ Complete | ✅ Complete     | ✅ Written   |
| Actor spawning    | ✅ Complete | ✅ Complete     | ✅ Written   |
| Recording/Replay  | ✅ Complete | ✅ Complete     | ❌ None     |
| Map loading       | ✅ Complete | ✅ Complete     | ❌ None     |

### Actors
| Actor Type    | FFI Status  | Rust API Status | Tests   |
|---------------|-------------|-----------------|---------|
| Base Actor    | ✅ Complete | ✅ Complete     | ✅ Written |
| Vehicle       | ✅ Complete | ✅ Complete     | ✅ Written |
| Walker        | ✅ Complete | ✅ Complete     | ✅ Written |
| Traffic Light | ✅ Complete | ✅ Complete     | ✅ Written |
| Traffic Sign  | ✅ Complete | ✅ Complete     | ✅ Written |

### Sensors
| Sensor Type            | FFI Status  | Rust API Status | Tests   |
|------------------------|-------------|-----------------|---------|
| Base Sensor            | ✅ Complete | ✅ Complete     | ✅ Written |
| Camera (RGB/Depth/Seg) | ✅ Complete | ✅ Complete     | ✅ Written |
| LiDAR                  | ✅ Complete | ⚠️  Partial*    | ✅ Written |
| Radar                  | ✅ Complete | ✅ Complete     | ✅ Written |
| GNSS                   | ✅ Complete | ✅ Complete     | ✅ Written |
| IMU                    | ✅ Complete | ✅ Complete     | ✅ Written |
| Collision              | ✅ Complete | ✅ Complete     | ✅ Written |
| Lane Invasion          | ✅ Complete | ✅ Complete     | ✅ Written |
| DVS                    | ✅ Complete | ✅ Complete     | ✅ Written |
| Obstacle Detection     | ✅ Complete | ✅ Complete     | ✅ Written |
| RSS                    | ✅ Complete | ✅ Complete     | ✅ Written |

*LiDAR has todo!() for timestamp/transform extraction

### Traffic Manager
| Feature          | FFI Status  | Rust API Status | Tests   |
|------------------|-------------|-----------------|---------|
| Basic control    | ✅ Complete | ✅ Complete     | ✅ Written |
| Vehicle behavior | ✅ Complete | ✅ Complete     | ✅ Written |
| Route planning   | ✅ Complete | ✅ Complete     | ✅ Written |
| Synchronous mode | ✅ Complete | ✅ Complete     | ✅ Written |

### Road/Map
| Feature   | FFI Status  | Rust API Status | Tests   |
|-----------|-------------|-----------------|---------|
| Map info  | ✅ Complete | ✅ Complete     | ✅ Unit    |
| Waypoints | ✅ Complete | ✅ Complete     | ✅ Unit    |
| Lanes     | ✅ Complete | ✅ Complete     | ✅ Unit    |
| Junctions | ✅ Complete | ✅ Complete     | ✅ Unit    |
| OpenDRIVE | ✅ Complete | ⚠️  Partial**   | ⚠️  TODO*** |
| Landmarks | ✅ Complete | ✅ Complete     | ✅ Unit    |
| Topology  | ✅ Complete | ✅ Complete     | ✅ Unit    |

**OpenDRIVE integration tests have todo!() placeholders
***OpenDRIVE tests fail with "not yet implemented"

### Utilities
| Feature         | FFI Status  | Rust API Status | Tests   |
|-----------------|-------------|-----------------|---------|
| Geometry types  | ✅ Complete | ✅ Complete     | ✅ Unit    |
| Weather         | ✅ Complete | ✅ Complete     | ✅ Unit    |
| Physics control | ✅ Complete | ✅ Complete     | ✅ Unit    |
| Light Manager   | ❌ Removed* | ❌ Removed*     | N/A     |

*Deprecated in CARLA 0.10.0

## Test Results Summary (2025-01)

Using `make test-server` (equivalent to `cargo nextest run --all-targets --no-fail-fast --features test-carla-server`):
- **Unit Tests**: 84 tests run: 84 passed ✅
- **Integration Tests**: 99 tests total
  - 8 OpenDRIVE tests fail with todo!() ⚠️
  - 58 tests abort (SIGABRT) - require running CARLA server ❌
  - 33 tests pass ✅
- **Doc Tests**: 1 failed (outdated API usage) ❌

## TODO Priority List

### Immediate Blockers (Fix First)
1. **CARLA Server Connection Issues** ✅ FIXED
   - [x] Improved server connection error messages
   - [x] Added server version check to verify communication
   - [x] Tests now fail with clear messages instead of silent skipping
   - [x] Fixed SIGABRT crashes by adding C++ exception handling in FFI layer
   - [x] Fixed null pointer handling in Rust wrapper layer
   - [x] Tests now show proper error messages instead of crashing

2. **OpenDRIVE Implementation**
   - [ ] Implement waypoint query API (test_get_waypoint)
   - [ ] Implement road structure API (test_parse_road)
   - [ ] Complete junction parsing (test_parse_junctions)
   - [ ] Fix all 8 failing OpenDRIVE tests

3. **LiDAR Data Completion**
   - [ ] Implement timestamp extraction from FFI
   - [ ] Implement transform extraction from FFI
   - [ ] Remove todo!() placeholders in lidar.rs

4. **Documentation Test Fix**
   - [ ] Update lib.rs doc example to use `into_vehicle()` instead of `to_vehicle()`
   - [ ] Update to use correct trait imports (remove `traits::VehicleT`)

### High Priority
5. **Sensor Data Retrieval**
   - [x] Implement Sensor_GetFrame FFI function
   - [x] Implement Sensor_GetTimestamp FFI function
   - [x] Complete data() methods for all sensor types
   - [ ] Add sensor data callbacks/streaming

6. **Collection Wrapper Performance**
   - [x] Implement zero-copy WaypointList wrapper
   - [x] Implement zero-copy TransformList wrapper
   - [x] Implement zero-copy LocationList wrapper
   - [x] Implement zero-copy Topology wrapper
   - [ ] Add benchmarks for wrapper performance

### Medium Priority
7. **Test Infrastructure**
   - [x] Migrate to cargo nextest
   - [ ] Fix integration test server connection handling
   - [ ] Add proper test fixtures and test data
   - [ ] Implement test server auto-start/stop
   - [ ] Add timeout handling for server tests

8. **Documentation and Examples**
   - [ ] Update examples to use new public submodule imports
   - [ ] Add examples for waypoint navigation
   - [ ] Add examples for Traffic Manager usage
   - [ ] Document zero-copy wrapper pattern

### Low Priority
9. **Advanced Features**
   - [ ] Walker bone control implementation
   - [ ] OpenDRIVE parsing and generation
   - [ ] Custom sensor plugin support
   - [ ] ROS2 integration bindings

## Known Issues
- **Critical**: Integration tests crash with SIGABRT when CARLA server not running
- **Critical**: OpenDRIVE tests have todo!() placeholders blocking functionality
- **Critical**: LiDAR timestamp/transform extraction not implemented
- Sensor attribute retrieval requires FFI functions
- Walker bone names not exposed through FFI
- No async/streaming support for sensors yet
- Map::junctions() requires additional FFI support
- Map::transform_from_geolocation() not yet implemented

## Testing Requirements
- Unit tests for each module
- Integration tests requiring CARLA server
- Performance benchmarks for sensor data
- Memory safety validation tests

## Build and Test Commands

Use the provided Makefile for consistent build and test execution:

```bash
# Build the project
make build

# Run tests without CARLA server
make test

# Run tests with CARLA server (server must be running)
make test-server

# Run linting
make lint

# Clean build artifacts
make clean
```
