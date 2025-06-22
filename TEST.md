# CARLA Rust Testing Framework Design

## Overview

This document outlines the testing framework for the CARLA Rust client library, designed to mirror and extend the testing approach used in the upstream C++ LibCarla library. The framework ensures comprehensive testing of the Rust API while maintaining compatibility with CARLA's testing philosophy.

**Note**: This document focuses exclusively on client-side tests. Server-side tests (buffer management, server threading, etc.) are not applicable to the Rust client library and have been excluded.

## Test Structure

Following Rust conventions, unit tests are placed within their respective modules using `#[cfg(test)]` blocks, while integration tests that require external dependencies or test multiple modules together are placed in the `tests/` directory.

### Directory Layout

```
carla/
├── src/                       # Source code with embedded unit tests
│   ├── client/
│   │   ├── mod.rs            # Client implementation
│   │   └── world.rs          # World implementation + unit tests
│   ├── sensor/
│   │   └── data.rs           # Sensor data + unit tests
│   ├── geom.rs               # Geometry module + unit tests
│   └── rpc/                  # RPC module + unit tests
├── tests/                     # Integration tests only
│   ├── common/               # Common test utilities
│   │   ├── mod.rs           # Test helpers and utilities
│   │   └── server.rs        # Mock CARLA server for testing
│   ├── fixtures/             # Symlinks to test data
│   │   └── opendrive/       # → carla-simulator/test_data/opendrive/
│   ├── client_integration.rs # Client-server integration tests
│   ├── world_integration.rs  # World interaction tests
│   ├── actor_integration.rs  # Actor management tests
│   └── sensor_integration.rs # Sensor data tests
├── benches/                  # Performance benchmarks
│   ├── streaming_bench.rs    # Data streaming benchmarks
│   └── geometry.rs           # Geometry computation benchmarks
└── examples/                 # Example code (also serve as tests)
    ├── basic_client.rs       # Basic client usage
    └── sensor_fusion.rs      # Advanced sensor usage

carla-sys/
├── src/
│   └── lib.rs               # FFI bindings + unit tests
└── tests/
    └── ffi_integration.rs   # FFI integration tests
```

## Test Categories

### 1. Unit Tests

Unit tests focus on individual components without requiring a CARLA server connection. Following Rust conventions, these tests are embedded within their respective modules.

#### Test Module Structure

Each Rust module contains its tests at the bottom of the file:

```rust
// carla/src/module_name.rs

// Public API implementation
pub struct MyStruct { ... }
pub fn my_function() { ... }

// Private helper functions
fn helper() { ... }

// Unit tests for this module
#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_my_struct() { ... }
    
    #[test]
    fn test_my_function() { ... }
}
```

#### Geometry Tests (from C++ test_geom.cpp)
```rust
// carla/src/geom.rs
pub struct Location {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

// ... implementation ...

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_single_point_no_transform() {
        let transform = Transform::new(
            Location::new(0.0, 0.0, 0.0),
            Rotation::new(0.0, 0.0, 0.0)
        );
        
        let mut point = Location::new(1.0, 1.0, 1.0);
        transform.transform_point(&mut point);
        
        assert_relative_eq!(point.x, 1.0, epsilon = 0.001);
        assert_relative_eq!(point.y, 1.0, epsilon = 0.001);
        assert_relative_eq!(point.z, 1.0, epsilon = 0.001);
    }

    #[test]
    fn test_single_point_translation() {
        let transform = Transform::new(
            Location::new(2.0, 5.0, 7.0),
            Rotation::new(0.0, 0.0, 0.0)
        );
        
        let mut point = Location::new(1.0, 1.0, 1.0);
        transform.transform_point(&mut point);
        
        assert_relative_eq!(point.x, 3.0, epsilon = 0.001);
        assert_relative_eq!(point.y, 6.0, epsilon = 0.001);
        assert_relative_eq!(point.z, 8.0, epsilon = 0.001);
    }

    #[test]
    fn test_bounding_box_transform() {
        // Test bounding box transformations
    }

    #[test]
    fn test_vector_operations() {
        let v1 = Vector3D::new(1.0, 2.0, 3.0);
        let v2 = Vector3D::new(4.0, 5.0, 6.0);
        
        let dot = v1.dot(&v2);
        assert_relative_eq!(dot, 32.0, epsilon = 0.001);
        
        let cross = v1.cross(&v2);
        assert_relative_eq!(cross.x, -3.0, epsilon = 0.001);
        assert_relative_eq!(cross.y, 6.0, epsilon = 0.001);
        assert_relative_eq!(cross.z, -3.0, epsilon = 0.001);
    }
}
```

#### World Tests (from C++ test_world.cpp)
```rust
// carla/src/client/world.rs
pub struct World {
    // ... fields ...
}

// ... implementation ...

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_world_settings() {
        // Test world settings manipulation
        let settings = WorldSettings {
            synchronous_mode: true,
            fixed_delta_seconds: Some(0.05),
            ..Default::default()
        };
        
        assert!(settings.synchronous_mode);
        assert_eq!(settings.fixed_delta_seconds, Some(0.05));
    }

    #[test]
    fn test_weather_parameters() {
        let weather = WeatherParameters {
            cloudiness: 50.0,
            precipitation: 30.0,
            sun_altitude_angle: 45.0,
            ..Default::default()
        };
        
        assert_eq!(weather.cloudiness, 50.0);
        assert_eq!(weather.precipitation, 30.0);
    }
}
```

#### Sensor Data Tests (from C++ test_image.cpp)
```rust
// carla/src/sensor/data.rs
pub struct Image {
    // ... fields ...
}

// ... implementation ...

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    #[cfg(not(debug_assertions))] // Only in release mode (performance)
    fn test_depth_conversion() {
        let width = 256;
        let height = 256;
        let mut image = Image::new(width, height, ImageFormat::Depth);
        
        // Test depth to grayscale conversion
        image.convert_to_grayscale();
        assert_eq!(image.format(), ImageFormat::Grayscale);
    }

    #[test]
    fn test_image_io_support() {
        println!("PNG support = {}", crate::image::has_png_support());
        println!("JPEG support = {}", crate::image::has_jpeg_support());
        println!("TIFF support = {}", crate::image::has_tiff_support());
    }
}
```

### 2. Integration Tests

Integration tests are placed in the `tests/` directory. Tests that require a running CARLA server use the `test-carla-server` feature flag:

```toml
# In Cargo.toml
[features]
test-carla-server = []  # Enable tests that require CARLA server running
```

Run server-dependent tests with:
```bash
cargo test --features test-carla-server
```

#### Client Integration Tests
```rust
// carla/tests/client_integration.rs
use carla::client::Client;
use std::time::Duration;

const TEST_TIMEOUT: Duration = Duration::from_secs(10);

#[test]
#[cfg(feature = "test-carla-server")]
fn test_client_connection() {
    // Assumes CARLA server is running on localhost:2000
    let client = Client::connect("localhost", 2000, TEST_TIMEOUT)
        .expect("Failed to connect to server");
    
    let version = client.get_server_version();
    assert!(version.starts_with("0.10"));
}

#[test]
#[cfg(feature = "test-carla-server")]
fn test_available_maps() {
    let client = Client::connect("localhost", 2000, TEST_TIMEOUT).unwrap();
    let maps = client.get_available_maps();
    assert!(!maps.is_empty());
}
```

#### OpenDrive Integration Tests (from C++ test_opendrive.cpp)
```rust
// carla/tests/opendrive_integration.rs
use carla::road::Map;
use std::fs;
use approx::assert_relative_eq;

#[test]
fn test_opendrive_parsing() {
    let opendrive_content = fs::read_to_string(
        "tests/fixtures/opendrive/simple_road.xodr"
    ).expect("Failed to read OpenDrive file");
    
    let map = Map::from_opendrive(&opendrive_content)
        .expect("Failed to parse OpenDrive");
    
    // Test road elevation
    let road = map.get_road(1).unwrap();
    let elevation = road.get_elevation_at(10.0);
    assert_relative_eq!(elevation, 0.0, epsilon = 0.001);
}
```

### 3. Property-Based Tests

Property-based tests can be placed either in unit test modules or integration tests, depending on their scope:

```rust
// carla/src/geom.rs - Property tests within the module

#[cfg(test)]
mod tests {
    use super::*;
    use proptest::prelude::*;

    proptest! {
        #[test]
        fn test_transform_inverse(
            x in -1000.0..1000.0,
            y in -1000.0..1000.0,
            z in -1000.0..1000.0,
            pitch in -180.0..180.0,
            yaw in -180.0..180.0,
            roll in -180.0..180.0
        ) {
            let transform = Transform::new(
                Location::new(x, y, z),
                Rotation::new(pitch, yaw, roll)
            );
            
            let inverse = transform.inverse();
            let identity = transform.compose(&inverse);
            
            prop_assert!(identity.location.distance(&Location::zero()) < 0.001);
            prop_assert!(identity.rotation.pitch.abs() < 0.001);
            prop_assert!(identity.rotation.yaw.abs() < 0.001);
            prop_assert!(identity.rotation.roll.abs() < 0.001);
        }

        #[test]
        fn test_transform_associativity(
            t1: Transform,
            t2: Transform,
            t3: Transform
        ) {
            let left = t1.compose(&t2).compose(&t3);
            let right = t1.compose(&t2.compose(&t3));
            
            prop_assert!(left.approx_eq(&right, 0.001));
        }
    }
}
```

## Test Fixtures and Data

### Symlink Strategy

Create symlinks to test data from the carla-simulator submodule:

```bash
# Setup script (setup_test_fixtures.sh)
#!/bin/bash
cd carla/tests/fixtures

# Create symlinks to test data
ln -sf ../../../carla-simulator/PythonAPI/util/opendrive opendrive
ln -sf ../../../carla-simulator/Unreal/CarlaUnreal/Plugins/CarlaTools/Content/MapGenerator/Misc/OpenDrive/TemplateOpenDrive.xodr template.xodr
```

### Test Data Categories

1. **OpenDrive Files** - Road network definitions
2. **Image Data** - Sample sensor outputs
3. **Configuration Files** - Traffic manager configs
4. **Mock Data** - Synthetic test data


## Benchmarking

Performance benchmarks to track regressions:

```rust
// carla/benches/streaming_bench.rs
use criterion::{black_box, criterion_group, criterion_main, Criterion};
use carla::streaming::Stream;

fn benchmark_stream_processing(c: &mut Criterion) {
    c.bench_function("process 1MB sensor data", |b| {
        let data = vec![0u8; 1024 * 1024];
        b.iter(|| {
            Stream::process_sensor_data(black_box(&data))
        });
    });
}

criterion_group!(benches, benchmark_stream_processing);
criterion_main!(benches);
```

## Test Utilities

### Common Test Helpers

```rust
// carla/tests/common/mod.rs
use std::time::Duration;

pub const TEST_TIMEOUT: Duration = Duration::from_secs(10);

/// Retry a test operation up to max_attempts times
pub fn retry_test<F, T, E>(mut f: F, max_attempts: u32) -> Result<T, E>
where
    F: FnMut() -> Result<T, E>,
{
    for _ in 0..max_attempts - 1 {
        if let Ok(result) = f() {
            return Ok(result);
        }
        std::thread::sleep(Duration::from_millis(100));
    }
    f()
}

/// Compare floating point values with tolerance
#[macro_export]
macro_rules! assert_near {
    ($left:expr, $right:expr, $tolerance:expr) => {
        let diff = ($left - $right).abs();
        assert!(
            diff < $tolerance,
            "assertion failed: `(left ≈ right)`\n  left: `{:?}`,\n right: `{:?}`,\n  diff: `{:?}`,\n  tolerance: `{:?}`",
            $left, $right, diff, $tolerance
        );
    };
}
```

## Continuous Integration

### Test Matrix

1. **Unit Tests** - Run on every commit
2. **Integration Tests without server** - Run on every PR (tests that don't need CARLA server)
3. **Integration Tests with server** - Run on every PR with `test-carla-server` feature
4. **Performance Benchmarks** - Run weekly to track regressions

### CI Configuration

```yaml
# .github/workflows/test.yml
name: Tests

on: [push, pull_request]

jobs:
  unit-tests:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v2
      - name: Setup test fixtures
        run: ./setup_test_fixtures.sh
      - name: Run unit tests
        run: cargo test --lib --bins --all-features

  integration-tests:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v2
      - name: Setup test fixtures
        run: ./setup_test_fixtures.sh
      - name: Start CARLA server
        run: ./carla.sh start headless
      - name: Run integration tests with CARLA server
        run: cargo test --test '*' --features test-carla-server -- --test-threads=1

  doc-tests:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v2
      - name: Run documentation tests
        run: cargo test --doc --all-features
```

## Test Coverage

### Coverage Goals

- **Unit Test Coverage**: > 80% for core modules
- **Integration Test Coverage**: All public APIs
- **Critical Path Coverage**: 100% for safety-critical components

### Coverage Reporting

```bash
# Install tarpaulin
cargo install cargo-tarpaulin

# Run tests with coverage
cargo tarpaulin --out Html --output-dir coverage
```

## Testing Best Practices

1. **Mirror C++ Tests**: Each C++ test should have a Rust equivalent
   - Unit tests from C++ go in their corresponding Rust module's `#[cfg(test)]` block
   - Integration tests from C++ go in `carla/tests/`
   - Maintain the same test coverage and scenarios as C++
2. **Test Placement**: Follow Rust conventions
   - Unit tests: In the same file as the code they test
   - Integration tests: In `tests/` directory
   - Benchmarks: In `benches/` directory
3. **Deterministic Tests**: Use fixed seeds for random data
4. **Isolated Tests**: Each test should be independent
5. **Fast Tests**: Unit tests should complete in < 100ms
6. **Descriptive Names**: Test names should describe what they test
7. **Error Messages**: Assertions should have helpful error messages
8. **Documentation**: Complex tests should have explanatory comments

## Test Implementation Progress

### Test Coverage Summary

- **Total C++ client-side test cases identified**: ~35 (excluding server-only tests)
- **Implemented in Rust**: 22 (from C++) + 36 (additional)
- **Implementation rate**: 63% (C++ client tests only)

#### Breakdown by Category:
- **Geometry Tests**: 10/10 C++ tests ✅ + 17 additional Rust tests
- **Image/Sensor Tests**: 3/3 C++ tests ✅ + 7 additional Rust tests
- **OpenDrive Tests**: 6/8 C++ tests (2 require server) + 12 additional Rust tests
- **Other Tests**: 3/14 C++ tests remaining (excluding server-only tests)

### Priority Order for Implementation

1. **High Priority** (Core functionality):
   - Geometry tests (Transform, Location, Rotation) ✅ Completed
   - Image/sensor data tests ✅ Completed
   - Basic RPC tests

2. **Medium Priority** (Common use cases):
   - OpenDrive parsing tests ✅ Completed
   - Client connection tests
   - MessagePack serialization tests

3. **Low Priority** (Advanced features):
   - Streaming tests
   - Complex async tests
   - Performance benchmarks

### Geometry Tests (from test_geom.cpp)

| C++ Test Case                                          | Rust Implementation Location     | Status         | Notes                              |
|--------------------------------------------------------|----------------------------------|----------------|------------------------------------|
| `single_point_no_transform`                            | `carla/src/geom/transform.rs`    | ✅ Implemented | Test identity transform            |
| `single_point_translation`                             | `carla/src/geom/transform.rs`    | ✅ Implemented | Test translation only              |
| `single_point_transform_inverse_transform_coherence`   | `carla/src/geom/transform.rs`    | ✅ Implemented | Test transform/inverse consistency |
| `bbox_get_local_vertices_get_world_vertices_coherence` | `carla/src/geom/bounding_box.rs` | ✅ Implemented | Test bounding box transformations  |
| `single_point_rotation`                                | `carla/src/geom/transform.rs`    | ✅ Implemented | Test rotation only                 |
| `single_point_translation_and_rotation`                | `carla/src/geom/transform.rs`    | ✅ Implemented | Test combined transform            |
| `distance`                                             | `carla/src/geom.rs`              | ✅ Implemented | Test distance calculations         |
| `nearest_point_segment`                                | `carla/src/geom.rs`              | ✅ Implemented | Test nearest segment finding       |
| `forward_vector`                                       | `carla/src/geom/rotation.rs`     | ✅ Implemented | Test rotation forward vectors      |
| `nearest_point_arc`                                    | `carla/src/geom.rs`              | ✅ Implemented | Test arc distance calculations     |

### Image/Sensor Tests (from test_image.cpp)

| C++ Test Case           | Rust Implementation Location      | Status         | Notes                             |
|-------------------------|-----------------------------------|----------------|-----------------------------------|
| `support`               | `carla/src/sensor_data/data.rs`   | ✅ Implemented | Test image format support         |
| `depth`                 | `carla/src/sensor_data/camera.rs` | ✅ Implemented | Test depth conversion             |
| `semantic_segmentation` | `carla/src/sensor_data/camera.rs` | ✅ Implemented | Test semantic segmentation colors |


### OpenDrive Tests (from test_opendrive.cpp)

| C++ Test Case          | Rust Implementation Location           | Status         | Notes                      |
|------------------------|----------------------------------------|----------------|----------------------------|
| `parse_files`          | `carla/tests/opendrive_integration.rs` | ✅ Implemented | Basic file parsing         |
| `parse_road_links`     | `carla/tests/opendrive_integration.rs` | ✅ Implemented | Test road connectivity     |
| `parse_junctions`      | `carla/tests/opendrive_integration.rs` | ✅ Implemented | Test junction parsing      |
| `parse_road`           | `carla/tests/opendrive_integration.rs` | ✅ Implemented | Test road structure        |
| `parse_road_elevation` | `carla/tests/opendrive_integration.rs` | ✅ Implemented | Test elevation profiles    |
| `parse_geometry`       | `carla/tests/opendrive_integration.rs` | ✅ Implemented | Test road geometry         |
| `iterate_waypoints`    | `carla/tests/opendrive_integration.rs` | ⚠️ Partial      | Requires server connection |
| `get_waypoint`         | `carla/tests/opendrive_integration.rs` | ⚠️ Partial      | Requires server connection |

### Streaming Tests (from test_streaming.cpp)

| C++ Test Case            | Rust Implementation Location           | Status         | Notes                   |
|--------------------------|----------------------------------------|----------------|-------------------------|
| `stream_outlives_server` | `carla/src/streaming/mod.rs`           | ❌ Not Started | Test object lifetime    |
| `multi_stream`           | `carla/tests/streaming_integration.rs` | ❌ Not Started | Test concurrent streams |

### Other Tests

| C++ Test File       | Rust Implementation Location  | Status         | Notes                     |
|---------------------|-------------------------------|----------------|---------------------------|
| `test_listview.cpp` | `carla/src/utils/listview.rs` | ❌ Not Started | ListView utility          |
| `test_msgpack.cpp`  | `carla/src/rpc/msgpack.rs`    | ❌ Not Started | MessagePack serialization |
| `test_vector3D.cpp` | `carla/src/geom/vector.rs`    | ✅ Implemented | Vector3D operations       |

### Additional Rust Tests (not in C++)

| Test Case                     | Rust Implementation Location     | Status         | Notes                        |
|-------------------------------|----------------------------------|----------------|------------------------------|
| `test_vector_operations`      | `carla/src/geom/vector.rs`       | ✅ Implemented | Vector dot/cross products    |
| `test_vector_arithmetic`      | `carla/src/geom/vector.rs`       | ✅ Implemented | Vector arithmetic operations |
| `test_vector_normalization`   | `carla/src/geom/vector.rs`       | ✅ Implemented | Vector normalization         |
| `test_vector_length`          | `carla/src/geom/vector.rs`       | ✅ Implemented | Vector length calculations   |
| `test_vector_distance`        | `carla/src/geom/vector.rs`       | ✅ Implemented | Vector distance calculations |
| `test_vector_2d_operations`   | `carla/src/geom/vector.rs`       | ✅ Implemented | 2D vector operations         |
| `test_transform_composition`  | `carla/src/geom/transform.rs`    | ✅ Implemented | Transform multiplication     |
| `test_transform_inverse`      | `carla/src/geom/transform.rs`    | ✅ Implemented | Transform inverse operations |
| `test_transform_point`        | `carla/src/geom/transform.rs`    | ✅ Implemented | Point transformation         |
| `test_compose`                | `carla/src/geom/transform.rs`    | ✅ Implemented | Transform composition        |
| `test_rotation_normalization` | `carla/src/geom/rotation.rs`     | ✅ Implemented | Angle normalization          |
| `test_rotation_inverse`       | `carla/src/geom/rotation.rs`     | ✅ Implemented | Rotation inverse             |
| `test_bounding_box_creation`  | `carla/src/geom/bounding_box.rs` | ✅ Implemented | BBox creation                |
| `test_bounding_box_min_max`   | `carla/src/geom/bounding_box.rs` | ✅ Implemented | BBox min/max corners         |
| `test_bounding_box_vertices`  | `carla/src/geom/bounding_box.rs` | ✅ Implemented | BBox vertex generation       |
| `test_bounding_box_contains`  | `carla/src/geom/bounding_box.rs` | ✅ Implemented | Point containment test       |
| `test_bounding_box_transform` | `carla/src/geom/bounding_box.rs` | ✅ Implemented | BBox transformation          |

## Future Enhancements

1. **Fuzzing**: Add fuzzing for protocol parsing
2. **Mutation Testing**: Ensure test quality
3. **Visual Regression**: For image processing tests
4. **Performance Tracking**: Automated performance regression detection
5. **Test Generation**: Auto-generate tests from C++ tests
