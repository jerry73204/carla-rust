# Rust API Design

## Overview

This document details the design of the high-level Rust API for the CARLA client library. The API aims to provide a safe, ergonomic, and idiomatic Rust interface while maintaining the full functionality of CARLA's C++ client library.

## Design Goals

### 1. **Rust-First Experience**
- APIs follow Rust naming conventions and idioms
- Leverage Rust's type system for compile-time safety
- Use standard Rust patterns (Result, Option, Iterator, Builder)
- Integrate naturally with the Rust ecosystem

### 2. **Zero-Cost Abstractions**
- High-level APIs compile down to efficient code
- Minimal runtime overhead compared to direct C++ usage
- Zero-copy data structures where possible
- Efficient memory management with RAII

### 3. **Safety and Correctness**
- Memory safety through Rust's ownership system
- Thread safety where appropriate
- Prevent common C++ pitfalls (dangling pointers, memory leaks)
- Clear error handling with comprehensive error types

### 4. **Ergonomic Usage**
- Intuitive method names and signatures
- Comprehensive documentation with examples
- Consistent patterns across the entire API
- Builder patterns for complex configuration

## API Structure

### Core Modules

```rust
carla/
├── actor/           // Actor wrappers and trait extensions
├── client/          // Client connection and builders
├── world/           // World wrapper and operations
├── sensor/          // Sensor data wrappers and extensions
├── traffic_manager/ // Traffic simulation control  
├── road/            // Maps, waypoints, navigation
├── geom/            // Geometry trait extensions
├── collections/     // Type-safe collections
└── error/           // Error types and handling
```

### Module Design Pattern

Each module follows the hybrid architecture pattern:

```rust
// carla/src/lib.rs
pub mod actor;
pub mod client;
pub mod world;
pub mod geom;

// Public API exports (wrapper types)
pub use actor::Actor;
pub use client::{Client, ClientBuilder};
pub use world::World;

// Re-export simple types with trait extensions
pub use carla_sys::generated::{Vector3D, Transform, Location};

// Prelude for trait extensions
pub mod prelude {
    pub use crate::actor::ActorExt;
    pub use crate::world::WorldExt;
    pub use crate::geom::{Vector3DExt, TransformExt};
}
```

## Type System Design

### Hybrid Type System

The hybrid architecture uses three patterns for different type categories:

#### **Complex Types (Mirror/Wrapper Pattern)**

```rust
// Generated in carla-sys
pub mod generated {
    pub struct Actor {
        pub id: u32,
        pub type_id: String,
    }
    
    impl Actor {
        pub fn get_velocity(&self) -> Result<Vector3D> {
            // Generated FFI call
        }
    }
}

// Wrapper in carla crate
pub struct Actor {
    inner: carla_sys::generated::Actor,
    world_ref: std::sync::Weak<World>,
}

impl Actor {
    // Delegate to generated
    pub fn get_velocity(&self) -> Result<Vector3D> {
        self.inner.get_velocity()
    }
    
    // Ergonomic API
    pub fn velocity(&self) -> Result<Vector3D> {
        self.get_velocity()
    }
    
    // Safe variants
    pub fn try_velocity(&self) -> Option<Vector3D> {
        self.velocity().ok()
    }
    
    // carla-specific functionality
    pub fn world(&self) -> Option<Arc<World>> {
        self.world_ref.upgrade()
    }
}
```

#### **Simple Types (Trait Extension Pattern)**

```rust
// Re-export generated type
pub use carla_sys::generated::Transform;

// Extend via traits
pub trait TransformExt {
    fn translate(&self, offset: &Vector3D) -> Transform;
    fn forward_vector(&self) -> Vector3D;
}

impl TransformExt for Transform {
    fn translate(&self, offset: &Vector3D) -> Transform {
        Transform {
            location: Location {
                x: self.location.x + offset.x,
                y: self.location.y + offset.y,
                z: self.location.z + offset.z,
            },
            rotation: self.rotation,
        }
    }
}
```

#### **New Types (Pure carla Implementation)**

```rust
// carla-specific functionality not in Python API
pub struct ActorCollection {
    actors: Vec<Actor>,
    world: std::sync::Weak<World>,
}

impl ActorCollection {
    pub fn vehicles(&self) -> impl Iterator<Item = &Actor> {
        self.actors.iter().filter(|a| a.is_vehicle())
    }
}
```

### Lifetime Management

Use Rust's lifetime system to enforce correct resource usage:

```rust
pub struct Waypoint<'w> {
    inner: carla_sys::Waypoint,
    _world: PhantomData<&'w World>,    // Waypoint cannot outlive World
}

impl<'w> Waypoint<'w> {
    pub fn next(&self, distance: f32) -> Option<Waypoint<'w>> {
        // Returns waypoint with same lifetime
    }
}
```

### Collection Types

Zero-copy collection wrappers for efficient data access:

```rust
pub struct ActorList<'w> {
    inner: &'w [carla_sys::Actor],
    world: &'w World,
}

impl<'w> ActorList<'w> {
    pub fn len(&self) -> usize {
        self.inner.len()
    }
    
    pub fn iter(&self) -> impl Iterator<Item = Actor> + 'w {
        self.inner.iter().map(|a| Actor::from_ffi(a, self.world))
    }
    
    pub fn filter_vehicles(&self) -> impl Iterator<Item = Vehicle> + 'w {
        self.iter().filter_map(|a| a.try_into_vehicle().ok())
    }
}
```

## Error Handling Design

### Comprehensive Error Types

```rust
#[derive(Debug, thiserror::Error)]
pub enum CarlaError {
    #[error("Connection failed: {reason}")]
    ConnectionFailed { reason: String },
    
    #[error("Actor {id} not found")]
    ActorNotFound { id: u32 },
    
    #[error("Invalid blueprint: {name}")]
    InvalidBlueprint { name: String },
    
    #[error("Spawn failed: {reason}")]
    SpawnFailed { reason: String },
    
    #[error("Sensor error: {sensor_type} - {reason}")]
    SensorError { sensor_type: String, reason: String },
    
    #[error("Traffic manager error: {reason}")]
    TrafficManagerError { reason: String },
    
    #[error("FFI error: {source}")]
    FfiError { 
        #[from]
        source: carla_sys::FfiError 
    },
}

pub type Result<T> = std::result::Result<T, CarlaError>;
```

### Error Context and Recovery

```rust
impl Actor {
    pub fn set_location(&self, location: Location) -> Result<()> {
        self.inner.set_location(location.into())
            .map_err(|e| CarlaError::FfiError { source: e })
            .with_context(|| format!("Failed to set location for actor {}", self.id))
    }
    
    pub fn try_set_location(&self, location: Location) -> Result<bool> {
        match self.set_location(location) {
            Ok(()) => Ok(true),
            Err(CarlaError::FfiError { .. }) => Ok(false), // Non-fatal FFI error
            Err(e) => Err(e), // Propagate other errors
        }
    }
}
```

## Method Design Patterns

### Naming Conventions

Follow Rust conventions consistently:

```rust
// ✅ Good - Rust idiomatic names
impl Actor {
    pub fn id(&self) -> u32 { /* ... */ }              // Not get_id()
    pub fn location(&self) -> Location { /* ... */ }    // Not get_location()
    pub fn set_location(&mut self, loc: Location) { /* ... */ }
    pub fn is_alive(&self) -> bool { /* ... */ }        // Not get_is_alive()
    pub fn type_id(&self) -> &str { /* ... */ }         // Not get_type_id()
}

// ❌ Avoid - C++ style names
impl Actor {
    pub fn get_id(&self) -> u32 { /* ... */ }
    pub fn GetLocation(&self) -> Location { /* ... */ }
    pub fn getTypeId(&self) -> &str { /* ... */ }
}
```

### Parameter Design

Use appropriate parameter types for different use cases:

```rust
impl World {
    // Borrowing for read-only operations
    pub fn spawn_actor(&self, blueprint: &Blueprint, transform: Transform) -> Result<Actor>;
    
    // Taking ownership for configuration
    pub fn set_weather(&mut self, weather: Weather) -> Result<()>;
    
    // References for large objects
    pub fn query_actors(&self, filter: &ActorFilter) -> ActorList;
    
    // Generic bounds for flexibility
    pub fn find_actors<F>(&self, predicate: F) -> Vec<Actor>
    where
        F: Fn(&Actor) -> bool;
}
```

### Return Type Design

Choose return types based on semantics:

```rust
impl World {
    // Fallible operations return Result
    pub fn spawn_actor(&self, blueprint: &Blueprint, transform: Transform) -> Result<Actor>;
    
    // Optional values use Option
    pub fn find_actor(&self, id: u32) -> Option<Actor>;
    
    // Collections use Vec or custom types
    pub fn actors(&self) -> ActorList;
    
    // Iterators for lazy evaluation
    pub fn actors_iter(&self) -> impl Iterator<Item = Actor>;
}
```

## Builder Pattern Implementation

### Configuration Objects

Use builders for complex configuration:

```rust
#[derive(Debug, Clone)]
pub struct ClientBuilder {
    host: String,
    port: u16,
    timeout: Duration,
    worker_threads: usize,
    retry_attempts: u32,
}

impl ClientBuilder {
    pub fn new() -> Self {
        Self {
            host: "localhost".to_string(),
            port: 2000,
            timeout: Duration::from_secs(5),
            worker_threads: 1,
            retry_attempts: 3,
        }
    }
    
    pub fn host<S: Into<String>>(mut self, host: S) -> Self {
        self.host = host.into();
        self
    }
    
    pub fn port(mut self, port: u16) -> Self {
        self.port = port;
        self
    }
    
    pub fn timeout(mut self, timeout: Duration) -> Self {
        self.timeout = timeout;
        self
    }
    
    pub fn connect(self) -> Result<Client> {
        Client::connect_with_config(self)
    }
}

// Usage
let client = ClientBuilder::new()
    .host("192.168.1.100")
    .port(3000)
    .timeout(Duration::from_secs(10))
    .connect()?;
```

### Sensor Configuration

```rust
#[derive(Debug, Clone)]
pub struct CameraBuilder {
    image_size_x: u32,
    image_size_y: u32,
    fov: f32,
    sensor_tick: f32,
    enable_postprocess_effects: bool,
}

impl CameraBuilder {
    pub fn new() -> Self {
        Self {
            image_size_x: 800,
            image_size_y: 600,
            fov: 90.0,
            sensor_tick: 0.0,
            enable_postprocess_effects: true, 
        }
    }
    
    pub fn resolution(mut self, width: u32, height: u32) -> Self {
        self.image_size_x = width;
        self.image_size_y = height;
        self
    }
    
    pub fn fov(mut self, fov: f32) -> Self {
        self.fov = fov;
        self
    }
    
    pub fn spawn_on(self, world: &World, transform: Transform) -> Result<Camera> {
        let blueprint = world.get_blueprint_library()
            .find("sensor.camera.rgb")?
            .configure(self)?;
        
        world.spawn_actor(&blueprint, transform)?.try_into()
    }
}

// Usage
let camera = CameraBuilder::new()
    .resolution(1920, 1080)
    .fov(110.0)
    .spawn_on(&world, transform)?;
```

## Async and Concurrency Design

### Future-Ready APIs

Design APIs to support async in the future:

```rust
// Current synchronous API
impl Client {
    pub fn get_world(&self) -> Result<World> { /* ... */ }
}

// Future async API (planned)
impl Client {
    pub async fn get_world_async(&self) -> Result<World> { /* ... */ }
    
    // Keep sync version for compatibility
    pub fn get_world(&self) -> Result<World> {
        block_on(self.get_world_async())
    }
}
```

### Thread Safety

Carefully design thread safety:

```rust
// Client is Send + Sync for multi-threaded usage
pub struct Client {
    inner: Arc<carla_sys::Client>,  // Thread-safe shared state
}

unsafe impl Send for Client {}
unsafe impl Sync for Client {}

// World has exclusive access semantics
pub struct World {
    inner: carla_sys::World,  // Not thread-safe
    _client_ref: Arc<Client>, // Ensure client outlives world
}

// World is Send but not Sync (exclusive access)
unsafe impl Send for World {}
```

## Sensor Data Handling

### Type-Safe Sensor Data

```rust
pub enum SensorData {
    Image(ImageData),
    PointCloud(PointCloudData),
    Radar(RadarData),
    Lidar(LidarData),
    SemanticSegmentation(SemanticSegmentationData),
}

impl SensorData {
    pub fn as_image(&self) -> Option<&ImageData> {
        match self {
            SensorData::Image(data) => Some(data),
            _ => None,
        }
    }
    
    pub fn try_into_image(self) -> Result<ImageData, Self> {
        match self {
            SensorData::Image(data) => Ok(data),
            other => Err(other),
        }
    }
}
```

### Zero-Copy Image Data

```rust
pub struct ImageData {
    inner: carla_sys::ImageData,
    width: u32,
    height: u32,
}

impl ImageData {
    pub fn width(&self) -> u32 { self.width }
    pub fn height(&self) -> u32 { self.height }
    
    // Zero-copy access to raw data
    pub fn raw_data(&self) -> &[u8] {
        self.inner.raw_data()
    }
    
    // Convenient typed access
    pub fn pixels(&self) -> &[Color] {
        // Safe cast due to layout guarantees
        unsafe {
            std::slice::from_raw_parts(
                self.raw_data().as_ptr() as *const Color,
                (self.width * self.height) as usize,
            )
        }
    }
    
    // Integration with image crates
    pub fn to_image_buffer(&self) -> ImageBuffer<Rgb<u8>, Vec<u8>> {
        ImageBuffer::from_raw(self.width, self.height, self.raw_data().to_vec())
            .expect("Invalid image dimensions")
    }
}
```

## Integration Patterns

### Serde Integration

```rust
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Transform {
    pub location: Location,
    pub rotation: Rotation,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Location {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

// Enables JSON serialization for configuration
let config = r#"
{
    "spawn_points": [
        {
            "location": {"x": 10.0, "y": 20.0, "z": 0.5},
            "rotation": {"pitch": 0.0, "yaw": 90.0, "roll": 0.0}
        }
    ]
}
"#;

let spawn_config: SpawnConfig = serde_json::from_str(config)?;
```

### nalgebra Integration

```rust
use nalgebra::{Vector3, Point3, Rotation3};

impl From<Location> for Point3<f32> {
    fn from(loc: Location) -> Self {
        Point3::new(loc.x, loc.y, loc.z)
    }
}

impl From<Vector3D> for Vector3<f32> {
    fn from(vec: Vector3D) -> Self {
        Vector3::new(vec.x, vec.y, vec.z)
    }
}

// Enables advanced mathematical operations
let velocity: Vector3<f32> = actor.get_velocity()?.into();
let speed = velocity.magnitude();
let normalized = velocity.normalize();
```

## Testing and Documentation

### Example-Driven Documentation

All public APIs include working examples:

```rust
impl World {
    /// Spawns a new actor in the world.
    ///
    /// # Arguments
    /// * `blueprint` - The actor blueprint defining type and attributes
    /// * `transform` - Initial position and rotation
    /// * `attach_to` - Optional parent actor for attachment
    ///
    /// # Returns
    /// The spawned actor instance
    ///
    /// # Errors
    /// Returns `CarlaError::SpawnFailed` if:
    /// - The spawn location is occupied
    /// - The blueprint is invalid
    /// - The attach_to actor doesn't exist
    ///
    /// # Example
    /// ```rust,no_run
    /// use carla::{ClientBuilder, Transform, Location};
    ///
    /// let client = ClientBuilder::new().connect()?;
    /// let world = client.world();
    /// 
    /// let blueprint = world.get_blueprint_library()
    ///     .find("vehicle.tesla.model3")?;
    ///     
    /// let spawn_point = world.get_map().get_spawn_points()[0];
    /// let actor = world.spawn_actor(&blueprint, spawn_point, None)?;
    /// 
    /// println!("Spawned actor with ID: {}", actor.id());
    /// # Ok::<(), carla::CarlaError>(())
    /// ```
    pub fn spawn_actor(
        &self, 
        blueprint: &Blueprint, 
        transform: Transform, 
        attach_to: Option<&Actor>
    ) -> Result<Actor> {
        // Implementation...
    }
}
```

### Integration Test Patterns

```rust
#[cfg(test)]
mod tests {
    use super::*;
    
    // Helper for tests requiring CARLA server
    fn with_carla_server<F>(test: F) -> Result<()> 
    where 
        F: FnOnce(&World) -> Result<()>
    {
        let client = ClientBuilder::new()
            .timeout(Duration::from_secs(1))
            .connect()
            .map_err(|_| {
                println!("Skipping test - CARLA server not available");
                return;
            })?;
            
        let world = client.world();
        test(&world)
    }
    
    #[test]
    fn test_actor_lifecycle() -> Result<()> {
        with_carla_server(|world| {
            let blueprint = world.get_blueprint_library()
                .find("vehicle.tesla.model3")?;
                
            let spawn_point = world.get_map().get_spawn_points()[0];
            let actor = world.spawn_actor(&blueprint, spawn_point, None)?;
            
            assert!(actor.is_alive());
            assert_eq!(actor.location(), spawn_point.location);
            
            actor.destroy()?;
            assert!(!actor.is_alive());
            
            Ok(())
        })
    }
}
```

## Performance Guidelines

### Memory Allocation

Minimize allocations in hot paths:

```rust
impl ActorList<'_> {
    // ✅ Good - iterator avoids allocation
    pub fn vehicles(&self) -> impl Iterator<Item = Vehicle> + '_ {
        self.iter().filter_map(|a| a.try_into_vehicle().ok())
    }
    
    // ❌ Avoid - allocates Vec unnecessarily
    pub fn vehicles_vec(&self) -> Vec<Vehicle> {
        self.vehicles().collect()
    }
}
```

### Caching Strategies

Cache frequently accessed data:

```rust
pub struct Actor {
    inner: carla_sys::Actor,
    
    // Cached values to avoid FFI calls
    id: u32,
    type_id: String,
    
    // Lazily computed values
    location_cache: RefCell<Option<(Instant, Location)>>,
}

impl Actor {
    pub fn location(&self) -> Location {
        let mut cache = self.location_cache.borrow_mut();
        
        // Return cached value if recent
        if let Some((timestamp, location)) = &*cache {
            if timestamp.elapsed() < Duration::from_millis(16) { // 60 FPS
                return *location;
            }
        }
        
        // Fetch fresh value and cache it
        let location = self.inner.get_location();
        *cache = Some((Instant::now(), location));
        location
    }
}
```

## Future Evolution

### Planned Enhancements

**Short Term**:
- Complete FFI implementation for all generated stubs
- Async API support for non-blocking operations
- Enhanced sensor data processing utilities
- Performance optimizations and caching

**Medium Term**:
- Custom derive macros for user types
- Advanced type-safe configuration builders
- Integration with popular Rust crates (tokio, serde, etc.)
- Streaming sensor data APIs

**Long Term**:
- WebAssembly compatibility
- Distributed simulation support
- Advanced AI/ML integration utilities
- Plugin system for custom behaviors

### API Stability

**Semver Commitment**:
- Major version (1.x.x): Breaking changes allowed
- Minor version (x.1.x): New features, backward compatible
- Patch version (x.x.1): Bug fixes only

**Deprecation Policy**:
- Deprecated APIs supported for at least one major version
- Clear upgrade paths provided in deprecation warnings
- Migration guides for major version upgrades

## Conclusion

The Rust API design prioritizes safety, ergonomics, and performance while maintaining full compatibility with CARLA's extensive feature set. By following Rust idioms and best practices, the API provides a natural and efficient interface for Rust developers working with autonomous vehicle simulation.

The layered architecture allows for incremental improvements while maintaining backward compatibility. The comprehensive error handling and documentation ensure that developers can quickly understand and effectively use the library in their projects.