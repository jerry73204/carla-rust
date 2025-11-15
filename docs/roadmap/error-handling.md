# Error Handling Design Proposal

**Priority:** MEDIUM
**Estimated Effort:** 1-2 weeks
**Status:** Proposed
**Dependencies:** None (can start immediately)

## Contents

- [Overview](#overview)
- [Current State Analysis](#current-state-analysis)
- [Proposed Error Type Design](#proposed-error-type-design)
- [Error Categories](#error-categories)
- [Implementation Plan](#implementation-plan)
- [Migration Strategy](#migration-strategy)
- [Examples](#examples)

## Overview

This document proposes a comprehensive error handling system for carla-rust that classifies errors into actionable categories, enabling users to handle specific error conditions (like connection failures) differently from others.

### Goals

1. **Actionable Error Classification**: Group errors by recovery strategy (retry, reconnect, abort, etc.)
2. **Rich Context**: Preserve original error messages while adding structured information
3. **User-Friendly**: Clear error messages with hints for resolution
4. **Backward Compatible**: Minimize breaking changes during migration
5. **Zero-Cost When Unused**: Error types should be lightweight

### Key Use Cases

**Use Case 1: Service Restart on Disconnection**
```rust
loop {
    match client.world().actors() {
        Ok(actors) => { /* process */ }
        Err(CarlaError::Connection(_)) => {
            // Specific handling for connection errors
            log::warn!("Lost connection to CARLA, reconnecting...");
            client = reconnect()?;
        }
        Err(e) => return Err(e), // Propagate other errors
    }
}
```

**Use Case 2: Timeout Retry Logic**
```rust
let mut retries = 3;
loop {
    match world.spawn_actor(&bp, &transform) {
        Ok(actor) => return Ok(actor),
        Err(CarlaError::Timeout(e)) if retries > 0 => {
            retries -= 1;
            log::warn!("Spawn timeout, retrying... ({} left)", retries);
            std::thread::sleep(Duration::from_millis(100));
        }
        Err(e) => return Err(e),
    }
}
```

**Use Case 3: Graceful Degradation**
```rust
match sensor.listen(callback) {
    Ok(()) => { /* normal operation */ }
    Err(CarlaError::Resource(ResourceError::NotFound { .. })) => {
        log::warn!("Sensor not available, using fallback");
        use_fallback_sensor()?;
    }
    Err(e) => return Err(e),
}
```

## Current State Analysis

### Existing Error Patterns

**carla crate** currently uses:
- `anyhow::Result<T>` - Opaque error type with context
- `anyhow::anyhow!()` - Ad-hoc error creation
- `std::io::Result<T>` - For file I/O operations

**Example from `carla/src/client/actor_builder.rs`:**
```rust
pub fn find(key: &str) -> Result<ActorBlueprint> {
    self.0
        .Find(&key.into())
        .ok_or_else(|| anyhow!("unable to find blueprint '{}'", key))
}
```

**carla-sys crate** (FFI layer):
- C++ exceptions are caught and converted to Rust panics/errors by cxx
- No structured error information crosses the FFI boundary
- Raw C++ exception messages are the only context

### Problems with Current Approach

1. **No Error Classification**: All errors are `anyhow::Error` - can't pattern match
2. **Lost Structure**: C++ exception types (TimeoutException, etc.) become opaque strings
3. **No Actionable Information**: Users can't distinguish connection errors from validation errors
4. **Context Loss**: Original C++ error type information is discarded
5. **Hard to Test**: Can't write tests that expect specific error types

### CARLA C++ Exception Types

From CARLA source analysis, the C++ library throws:
- **`TimeoutException`** - Network timeout (seen in world.hpp)
- **`std::runtime_error`** - General runtime errors
- **`std::invalid_argument`** - Invalid parameters
- **`std::out_of_range`** - Index/bounds errors

## Proposed Error Type Design

### Error Hierarchy

```
CarlaError (top-level enum)
├── Connection (ConnectionError)
│   ├── Timeout { operation, duration, ... }
│   ├── Disconnected { reason, ... }
│   └── InvalidEndpoint { host, port, ... }
├── Resource (ResourceError)
│   ├── NotFound { type, id, ... }
│   ├── AlreadyExists { type, id, ... }
│   ├── Destroyed { type, id, ... }
│   └── Unavailable { type, reason, ... }
├── Operation (OperationError)
│   ├── SpawnFailed { blueprint, transform, reason, ... }
│   ├── InvalidTransform { transform, reason, ... }
│   ├── PhysicsDisabled { actor_id, operation, ... }
│   └── SimulationError { message, ... }
├── Validation (ValidationError)
│   ├── InvalidBlueprint { blueprint_id, reason, ... }
│   ├── InvalidAttribute { name, value, reason, ... }
│   ├── OutOfBounds { value, min, max, field, ... }
│   └── InvalidConfiguration { setting, reason, ... }
├── Map (MapError)
│   ├── LoadFailed { map_name, reason, ... }
│   ├── InvalidWaypoint { location, reason, ... }
│   └── TopologyError { message, ... }
├── Sensor (SensorError)
│   ├── DataUnavailable { sensor_id, reason, ... }
│   ├── InvalidConfiguration { sensor_type, reason, ... }
│   └── ListenFailed { sensor_id, reason, ... }
└── Internal (InternalError)
    ├── FfiError { message, ... }
    ├── SerializationError { context, source, ... }
    └── UnexpectedState { description, ... }
```

### Core Types

```rust
// carla/src/error.rs

use std::fmt;
use std::time::Duration;

/// Main error type for all CARLA operations
#[derive(Debug, thiserror::Error)]
pub enum CarlaError {
    /// Connection-related errors (network, timeouts, disconnections)
    #[error(transparent)]
    Connection(#[from] ConnectionError),

    /// Resource-related errors (actors, blueprints not found, etc.)
    #[error(transparent)]
    Resource(#[from] ResourceError),

    /// Operation failures (spawn, physics, simulation errors)
    #[error(transparent)]
    Operation(#[from] OperationError),

    /// Validation errors (invalid parameters, out of bounds, etc.)
    #[error(transparent)]
    Validation(#[from] ValidationError),

    /// Map-related errors (loading, waypoints, topology)
    #[error(transparent)]
    Map(#[from] MapError),

    /// Sensor-related errors (data, configuration, listening)
    #[error(transparent)]
    Sensor(#[from] SensorError),

    /// Internal errors (FFI, unexpected states, bugs)
    #[error(transparent)]
    Internal(#[from] InternalError),

    /// I/O errors (file operations)
    #[error(transparent)]
    Io(#[from] std::io::Error),
}

/// Connection and network errors
#[derive(Debug, thiserror::Error)]
pub enum ConnectionError {
    /// Operation timed out waiting for server response
    #[error("Operation '{operation}' timed out after {duration:?}")]
    Timeout {
        operation: String,
        duration: Duration,
        #[source]
        source: Option<Box<dyn std::error::Error + Send + Sync>>,
    },

    /// Lost connection to CARLA server
    #[error("Disconnected from CARLA server: {reason}")]
    Disconnected {
        reason: String,
        #[source]
        source: Option<Box<dyn std::error::Error + Send + Sync>>,
    },

    /// Invalid server endpoint
    #[error("Invalid server endpoint: {host}:{port}")]
    InvalidEndpoint {
        host: String,
        port: u16,
        #[source]
        source: Option<Box<dyn std::error::Error + Send + Sync>>,
    },
}

/// Resource not found, unavailable, or destroyed
#[derive(Debug, thiserror::Error)]
pub enum ResourceError {
    /// Resource not found (actor, blueprint, map, etc.)
    #[error("{resource_type} not found: {identifier}")]
    NotFound {
        resource_type: ResourceType,
        identifier: String,
        context: Option<String>,
    },

    /// Resource already exists (duplicate ID, etc.)
    #[error("{resource_type} already exists: {identifier}")]
    AlreadyExists {
        resource_type: ResourceType,
        identifier: String,
    },

    /// Resource was destroyed/deleted
    #[error("{resource_type} was destroyed: {identifier}")]
    Destroyed {
        resource_type: ResourceType,
        identifier: String,
    },

    /// Resource temporarily unavailable
    #[error("{resource_type} unavailable: {reason}")]
    Unavailable {
        resource_type: ResourceType,
        reason: String,
    },
}

/// Types of resources in CARLA
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ResourceType {
    Actor,
    Blueprint,
    Map,
    Sensor,
    TrafficLight,
    Waypoint,
    Episode,
    Recording,
}

/// Operation failures during simulation
#[derive(Debug, thiserror::Error)]
pub enum OperationError {
    /// Failed to spawn actor
    #[error("Failed to spawn actor '{blueprint}' at {transform:?}: {reason}")]
    SpawnFailed {
        blueprint: String,
        transform: String, // formatted transform
        reason: String,
        #[source]
        source: Option<Box<dyn std::error::Error + Send + Sync>>,
    },

    /// Invalid transform (collision, out of bounds, etc.)
    #[error("Invalid transform {transform:?}: {reason}")]
    InvalidTransform {
        transform: String,
        reason: String,
    },

    /// Operation requires physics to be enabled
    #[error("Operation '{operation}' requires physics for actor {actor_id}")]
    PhysicsDisabled {
        actor_id: u32,
        operation: String,
    },

    /// General simulation error
    #[error("Simulation error: {message}")]
    SimulationError {
        message: String,
        #[source]
        source: Option<Box<dyn std::error::Error + Send + Sync>>,
    },
}

/// Parameter validation errors
#[derive(Debug, thiserror::Error)]
pub enum ValidationError {
    /// Invalid blueprint ID or configuration
    #[error("Invalid blueprint '{blueprint_id}': {reason}")]
    InvalidBlueprint {
        blueprint_id: String,
        reason: String,
    },

    /// Invalid attribute value
    #[error("Invalid attribute '{name}' = '{value}': {reason}")]
    InvalidAttribute {
        name: String,
        value: String,
        reason: String,
    },

    /// Value out of valid range
    #[error("Value {value} out of bounds [{min}, {max}] for field '{field}'")]
    OutOfBounds {
        field: String,
        value: String,
        min: String,
        max: String,
    },

    /// Invalid configuration
    #[error("Invalid configuration for '{setting}': {reason}")]
    InvalidConfiguration {
        setting: String,
        reason: String,
    },
}

/// Map-related errors
#[derive(Debug, thiserror::Error)]
pub enum MapError {
    /// Failed to load map
    #[error("Failed to load map '{map_name}': {reason}")]
    LoadFailed {
        map_name: String,
        reason: String,
        #[source]
        source: Option<Box<dyn std::error::Error + Send + Sync>>,
    },

    /// Invalid waypoint location
    #[error("Invalid waypoint at {location:?}: {reason}")]
    InvalidWaypoint {
        location: String,
        reason: String,
    },

    /// Map topology error
    #[error("Map topology error: {message}")]
    TopologyError {
        message: String,
    },
}

/// Sensor-related errors
#[derive(Debug, thiserror::Error)]
pub enum SensorError {
    /// Sensor data not available
    #[error("Sensor data unavailable for sensor {sensor_id}: {reason}")]
    DataUnavailable {
        sensor_id: u32,
        reason: String,
    },

    /// Invalid sensor configuration
    #[error("Invalid sensor configuration for '{sensor_type}': {reason}")]
    InvalidConfiguration {
        sensor_type: String,
        reason: String,
    },

    /// Failed to start listening
    #[error("Failed to listen to sensor {sensor_id}: {reason}")]
    ListenFailed {
        sensor_id: u32,
        reason: String,
        #[source]
        source: Option<Box<dyn std::error::Error + Send + Sync>>,
    },
}

/// Internal errors (FFI, unexpected states)
#[derive(Debug, thiserror::Error)]
pub enum InternalError {
    /// FFI boundary error
    #[error("FFI error: {message}")]
    FfiError {
        message: String,
        #[source]
        source: Option<Box<dyn std::error::Error + Send + Sync>>,
    },

    /// Serialization/deserialization error
    #[error("Serialization error in {context}")]
    SerializationError {
        context: String,
        #[source]
        source: Box<dyn std::error::Error + Send + Sync>,
    },

    /// Unexpected internal state (likely a bug)
    #[error("Unexpected state: {description}")]
    UnexpectedState {
        description: String,
    },
}

/// Result type for CARLA operations
pub type Result<T> = std::result::Result<T, CarlaError>;

impl ResourceType {
    pub fn as_str(&self) -> &'static str {
        match self {
            ResourceType::Actor => "Actor",
            ResourceType::Blueprint => "Blueprint",
            ResourceType::Map => "Map",
            ResourceType::Sensor => "Sensor",
            ResourceType::TrafficLight => "TrafficLight",
            ResourceType::Waypoint => "Waypoint",
            ResourceType::Episode => "Episode",
            ResourceType::Recording => "Recording",
        }
    }
}

impl fmt::Display for ResourceType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}
```

## Error Categories

### 1. Connection Errors (Retryable with Reconnect)

**When**: Network issues, server unavailable, timeouts
**Recovery Strategy**: Reconnect, retry with backoff
**Examples**:
- Connection timeout when waiting for world tick
- Server disconnected during operation
- Invalid host/port configuration

### 2. Resource Errors (Check Availability)

**When**: Resource not found, already destroyed, unavailable
**Recovery Strategy**: Check existence, use alternative, recreate
**Examples**:
- Actor not found (already destroyed)
- Blueprint doesn't exist in library
- Map not available on server

### 3. Operation Errors (Validate Preconditions)

**When**: Operation failed due to simulation state
**Recovery Strategy**: Fix preconditions, retry, skip operation
**Examples**:
- Spawn failed (collision with existing actor)
- Physics operation on actor with physics disabled
- Invalid transform (out of map bounds)

### 4. Validation Errors (Fix Parameters)

**When**: Invalid user input, out-of-bounds values
**Recovery Strategy**: Fix input, use defaults, prompt user
**Examples**:
- Invalid blueprint attribute value
- Parameter out of valid range
- Malformed configuration

### 5. Map Errors (Map-Specific Handling)

**When**: Map loading, waypoint queries, topology issues
**Recovery Strategy**: Load different map, fallback to spawn points
**Examples**:
- Map file not found
- Waypoint generation failed
- Road topology inconsistent

### 6. Sensor Errors (Sensor-Specific Handling)

**When**: Sensor data, configuration, listening issues
**Recovery Strategy**: Reconfigure sensor, use fallback data
**Examples**:
- Sensor data not ready
- Invalid sensor configuration
- Listen callback failed

### 7. Internal Errors (Report Bug)

**When**: FFI errors, unexpected states, library bugs
**Recovery Strategy**: Report issue, restart process
**Examples**:
- FFI type conversion failed
- Unexpected null pointer
- Invariant violated

## Implementation Plan

### Phase 1: Core Types (Week 1, Days 1-2)

**Goal**: Define error types and basic infrastructure

**Tasks**:
1. Create `carla/src/error.rs` with all error types
2. Add `thiserror = "1.0"` to `carla/Cargo.toml`
3. Re-export from `carla/src/lib.rs`: `pub use error::{CarlaError, Result};`
4. Write unit tests for error type construction and display

**Deliverables**:
- `carla/src/error.rs` (~400 lines)
- Error type unit tests
- Updated `Cargo.toml`

### Phase 2: FFI Error Classification (Week 1, Days 3-5)

**Goal**: Capture and classify C++ exceptions at FFI boundary

**Tasks**:
1. Create `carla-sys/csrc/carla_rust/error.hpp` with error detection helpers
2. Implement `classify_exception()` to detect C++ exception types
3. Add error code returns to FFI functions (return `int` instead of `void`)
4. Update existing FFI wrappers to catch and classify exceptions
5. Create Rust helper `parse_ffi_error()` to convert error codes + messages

**Example FFI Error Handling**:
```cpp
// carla-sys/csrc/carla_rust/error.hpp
namespace carla_rust {
namespace error {

enum class ErrorKind {
    Success = 0,
    Timeout = 1,
    NotFound = 2,
    InvalidArgument = 3,
    RuntimeError = 4,
    Unknown = 5,
};

inline ErrorKind classify_exception(const std::exception& e) {
    const char* what = e.what();

    // Check for timeout
    if (std::string(what).find("timeout") != std::string::npos ||
        std::string(what).find("Timeout") != std::string::npos) {
        return ErrorKind::Timeout;
    }

    // Check for not found
    if (std::string(what).find("not found") != std::string::npos ||
        std::string(what).find("does not exist") != std::string::npos) {
        return ErrorKind::NotFound;
    }

    // std::invalid_argument
    if (dynamic_cast<const std::invalid_argument*>(&e)) {
        return ErrorKind::InvalidArgument;
    }

    // std::runtime_error
    if (dynamic_cast<const std::runtime_error*>(&e)) {
        return ErrorKind::RuntimeError;
    }

    return ErrorKind::Unknown;
}

} // namespace error
} // namespace carla_rust
```

**Deliverables**:
- `carla-sys/csrc/carla_rust/error.hpp` (~150 lines)
- Updated FFI wrappers with error classification
- Rust FFI error parsing helpers

### Phase 3: Incremental API Migration (Week 2)

**Goal**: Migrate high-priority APIs to use new error types

**Priority 1 (Critical - Connection/Resource):**
- `Client::connect()` - ConnectionError::InvalidEndpoint, Timeout
- `World::actors()` - ConnectionError::Timeout, Disconnected
- `World::spawn_actor()` - OperationError::SpawnFailed, ValidationError
- `ActorBlueprint::find()` - ResourceError::NotFound

**Priority 2 (High - Operations):**
- `Actor::destroy()` - ResourceError::Destroyed
- `World::load_world()` - MapError::LoadFailed
- `Sensor::listen()` - SensorError::ListenFailed

**Priority 3 (Medium - Remaining APIs):**
- Waypoint APIs - MapError::InvalidWaypoint
- Vehicle physics - OperationError::PhysicsDisabled
- Attribute setters - ValidationError::InvalidAttribute

**Migration Pattern**:
```rust
// Before (anyhow)
pub fn find(key: &str) -> anyhow::Result<ActorBlueprint> {
    self.0
        .Find(&key.into())
        .ok_or_else(|| anyhow!("unable to find blueprint '{}'", key))
}

// After (CarlaError)
pub fn find(key: &str) -> Result<ActorBlueprint> {
    self.0
        .Find(&key.into())
        .ok_or_else(|| ResourceError::NotFound {
            resource_type: ResourceType::Blueprint,
            identifier: key.to_string(),
            context: Some("ActorBlueprint::find".to_string()),
        }.into())
}
```

**Deliverables**:
- Migrated APIs using new error types
- Updated examples demonstrating error handling
- Updated documentation with error scenarios

### Phase 4: Helper Utilities (Week 2, End)

**Goal**: Add convenience functions and utilities

**Tasks**:
1. Add `CarlaError::is_*()` helper methods:
   ```rust
   impl CarlaError {
       pub fn is_connection_error(&self) -> bool { ... }
       pub fn is_timeout(&self) -> bool { ... }
       pub fn is_not_found(&self) -> bool { ... }
       pub fn is_retriable(&self) -> bool { ... }
   }
   ```

2. Add context extension traits:
   ```rust
   pub trait ResultExt<T> {
       fn with_context<F>(self, f: F) -> Result<T>
       where
           F: FnOnce() -> String;
   }
   ```

3. Add error classification from anyhow (migration helper):
   ```rust
   impl From<anyhow::Error> for CarlaError {
       fn from(e: anyhow::Error) -> Self {
           // Try to parse error message and classify
           classify_anyhow_error(e)
       }
   }
   ```

**Deliverables**:
- Helper methods on CarlaError
- Extension traits for Result
- Migration helpers

## Migration Strategy

### Approach: Gradual Migration with Compatibility Layer

**Goal**: Minimize breaking changes while enabling new error handling

### Step 1: Dual Result Types (Compatibility Phase)

Keep both `anyhow::Result` and `CarlaError::Result` during migration:

```rust
// Old code still works
pub mod v1 {
    pub use anyhow::Result;  // Re-export for compatibility
}

// New code uses CarlaError
pub mod v2 {
    pub use crate::error::{CarlaError, Result};
}

// Public API offers both
pub fn find_v1(key: &str) -> anyhow::Result<ActorBlueprint> { ... }
pub fn find(key: &str) -> crate::error::Result<ActorBlueprint> { ... }
```

### Step 2: Deprecation Warnings

Mark old APIs with deprecation:

```rust
#[deprecated(since = "0.13.0", note = "Use find() which returns CarlaError instead")]
pub fn find_v1(key: &str) -> anyhow::Result<ActorBlueprint> {
    self.find().map_err(|e| anyhow::anyhow!("{}", e))
}
```

### Step 3: Example Updates

Update all examples to use new error types:

```rust
// Before
let actor = world.spawn_actor(&bp, &transform)?;

// After
let actor = world.spawn_actor(&bp, &transform)
    .map_err(|e| match e {
        CarlaError::Operation(OperationError::SpawnFailed { reason, .. }) => {
            eprintln!("Spawn failed: {}. Try different location.", reason);
            e
        }
        _ => e,
    })?;
```

### Step 4: Major Version Bump

When migration is complete:
- Bump to carla 0.13.0 (minor version bump, deprecations)
- Remove deprecated v1 functions in 1.0.0 (major version)

### Breaking Change Considerations

**Minimal Breaking Changes** (users mostly need to update imports):
- Change `anyhow::Result` to `carla::Result`
- Change `anyhow::Error` to `carla::CarlaError`
- Existing error handling with `?` operator still works

**Enhanced Error Handling** (optional, users can opt-in):
- Pattern match on specific error types
- Use `is_*()` helpers for classification
- Implement retry logic based on error category

## Examples

### Example 1: Connection Retry with Exponential Backoff

```rust
use carla::{Client, CarlaError, ConnectionError};
use std::time::Duration;

fn connect_with_retry(host: &str, port: u16, max_retries: u32) -> Result<Client> {
    let mut backoff = Duration::from_millis(100);

    for attempt in 0..max_retries {
        match Client::connect(host, port, None) {
            Ok(client) => return Ok(client),
            Err(CarlaError::Connection(ConnectionError::Timeout { .. })) => {
                eprintln!("Connection timeout (attempt {}/{}), retrying in {:?}...",
                    attempt + 1, max_retries, backoff);
                std::thread::sleep(backoff);
                backoff *= 2;  // Exponential backoff
            }
            Err(CarlaError::Connection(ConnectionError::InvalidEndpoint { host, port, .. })) => {
                eprintln!("Invalid endpoint {}:{}", host, port);
                return Err(CarlaError::Connection(ConnectionError::InvalidEndpoint {
                    host: host.clone(),
                    port,
                    source: None,
                }));
            }
            Err(e) => return Err(e),  // Other errors are not retryable
        }
    }

    Err(CarlaError::Connection(ConnectionError::Timeout {
        operation: "connect".to_string(),
        duration: backoff,
        source: None,
    }))
}
```

### Example 2: Spawn with Fallback Locations

```rust
use carla::{CarlaError, OperationError, World};

fn spawn_vehicle_with_fallback(world: &World, blueprint: &ActorBlueprint) -> Result<Actor> {
    let spawn_points = world.map().recommended_spawn_points();

    for (i, transform) in spawn_points.iter().enumerate() {
        match world.spawn_actor(blueprint, transform) {
            Ok(actor) => {
                println!("Spawned at spawn point {}", i);
                return Ok(actor);
            }
            Err(CarlaError::Operation(OperationError::SpawnFailed { reason, .. })) => {
                eprintln!("Spawn point {} failed: {}, trying next...", i, reason);
                continue;
            }
            Err(e) => return Err(e),  // Other errors are fatal
        }
    }

    Err(CarlaError::Operation(OperationError::SpawnFailed {
        blueprint: blueprint.id().to_string(),
        transform: "all spawn points".to_string(),
        reason: "All spawn points failed".to_string(),
        source: None,
    }))
}
```

### Example 3: Service with Graceful Degradation

```rust
use carla::{CarlaError, ResourceError, SensorError};

struct AutonomousAgent {
    vehicle: Vehicle,
    camera: Option<Sensor>,
    lidar: Option<Sensor>,
}

impl AutonomousAgent {
    fn setup_sensors(&mut self, world: &World) -> Result<()> {
        // Try to attach camera
        match self.attach_camera(world) {
            Ok(sensor) => {
                self.camera = Some(sensor);
                println!("Camera sensor attached");
            }
            Err(CarlaError::Resource(ResourceError::NotFound { .. })) => {
                eprintln!("Camera blueprint not available, using vision-free mode");
            }
            Err(CarlaError::Sensor(SensorError::ListenFailed { reason, .. })) => {
                eprintln!("Camera listen failed: {}, continuing without camera", reason);
            }
            Err(e) => return Err(e),  // Fatal errors
        }

        // Try to attach lidar
        match self.attach_lidar(world) {
            Ok(sensor) => {
                self.lidar = Some(sensor);
                println!("Lidar sensor attached");
            }
            Err(CarlaError::Resource(ResourceError::NotFound { .. })) => {
                eprintln!("Lidar blueprint not available, using camera-only mode");
            }
            Err(e) => return Err(e),
        }

        // Ensure at least one sensor is available
        if self.camera.is_none() && self.lidar.is_none() {
            return Err(CarlaError::Operation(OperationError::SimulationError {
                message: "No sensors available for autonomous agent".to_string(),
                source: None,
            }));
        }

        Ok(())
    }
}
```

### Example 4: Error Logging and Metrics

```rust
use carla::{CarlaError, ConnectionError};
use std::sync::atomic::{AtomicU64, Ordering};

struct ErrorMetrics {
    connection_errors: AtomicU64,
    timeout_errors: AtomicU64,
    resource_errors: AtomicU64,
    other_errors: AtomicU64,
}

impl ErrorMetrics {
    fn record_error(&self, error: &CarlaError) {
        match error {
            CarlaError::Connection(ConnectionError::Timeout { .. }) => {
                self.timeout_errors.fetch_add(1, Ordering::Relaxed);
                log::warn!("Timeout error: {}", error);
            }
            CarlaError::Connection(_) => {
                self.connection_errors.fetch_add(1, Ordering::Relaxed);
                log::error!("Connection error: {}", error);
            }
            CarlaError::Resource(_) => {
                self.resource_errors.fetch_add(1, Ordering::Relaxed);
                log::warn!("Resource error: {}", error);
            }
            _ => {
                self.other_errors.fetch_add(1, Ordering::Relaxed);
                log::error!("Error: {}", error);
            }
        }
    }

    fn report(&self) {
        println!("Error Statistics:");
        println!("  Connection: {}", self.connection_errors.load(Ordering::Relaxed));
        println!("  Timeout: {}", self.timeout_errors.load(Ordering::Relaxed));
        println!("  Resource: {}", self.resource_errors.load(Ordering::Relaxed));
        println!("  Other: {}", self.other_errors.load(Ordering::Relaxed));
    }
}
```

## Benefits

### For Users

1. **Actionable Error Handling**: Match on specific error types to implement recovery strategies
2. **Better Debugging**: Rich context (operation, resource type, parameters) in error messages
3. **Service Resilience**: Distinguish transient errors (retry) from permanent errors (abort)
4. **Type Safety**: Compiler-checked error handling patterns
5. **Documentation**: Error types document failure modes of each API

### For Maintainers

1. **Structured Testing**: Write tests expecting specific error types
2. **Clearer Bugs**: Error categorization helps identify root causes
3. **API Evolution**: Add new error variants without breaking existing code
4. **Instrumentation**: Error metrics and logging per category

### For Library

1. **Professional Error Handling**: Matches expectations of production Rust code
2. **Better Error Messages**: Contextual information instead of raw C++ exceptions
3. **Composable**: Error types can be extended for custom use cases
4. **Zero-Cost Abstraction**: `thiserror` generates efficient code

## Testing Strategy

### Unit Tests

```rust
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_connection_error_display() {
        let error = ConnectionError::Timeout {
            operation: "spawn_actor".to_string(),
            duration: Duration::from_secs(5),
            source: None,
        };
        assert_eq!(
            format!("{}", error),
            "Operation 'spawn_actor' timed out after 5s"
        );
    }

    #[test]
    fn test_error_classification() {
        let error: CarlaError = ResourceError::NotFound {
            resource_type: ResourceType::Blueprint,
            identifier: "vehicle.tesla.model3".to_string(),
            context: None,
        }.into();

        assert!(error.is_not_found());
        assert!(!error.is_timeout());
    }

    #[test]
    fn test_error_chain() {
        let io_error = std::io::Error::new(std::io::ErrorKind::NotFound, "file not found");
        let error: CarlaError = io_error.into();

        assert!(matches!(error, CarlaError::Io(_)));
    }
}
```

### Integration Tests

```rust
#[test]
fn test_spawn_with_invalid_blueprint() {
    let client = Client::connect("localhost", 2000, None).unwrap();
    let world = client.world();
    let bp_lib = world.blueprint_library();

    let result = bp_lib.find("invalid.blueprint.name");
    assert!(matches!(
        result,
        Err(CarlaError::Resource(ResourceError::NotFound { .. }))
    ));
}

#[test]
fn test_connection_timeout() {
    // Connect to non-existent server
    let result = Client::connect("invalid.host", 9999, Some(1));
    assert!(matches!(
        result,
        Err(CarlaError::Connection(ConnectionError::Timeout { .. }))
    ));
}
```

## Future Enhancements

### 1. Error Recovery Traits

```rust
pub trait Retriable {
    fn should_retry(&self) -> bool;
    fn backoff_duration(&self, attempt: u32) -> Duration;
}

impl Retriable for CarlaError {
    fn should_retry(&self) -> bool {
        matches!(self,
            CarlaError::Connection(ConnectionError::Timeout { .. }) |
            CarlaError::Operation(OperationError::SpawnFailed { .. })
        )
    }
}
```

### 2. Error Context Builder

```rust
pub struct ErrorContext {
    operation: String,
    actor_id: Option<u32>,
    additional: HashMap<String, String>,
}

impl ErrorContext {
    pub fn new(operation: impl Into<String>) -> Self { ... }
    pub fn with_actor(mut self, id: u32) -> Self { ... }
    pub fn with_info(mut self, key: impl Into<String>, value: impl Into<String>) -> Self { ... }
}
```

### 3. Error Callbacks

```rust
pub trait ErrorHandler: Send + Sync {
    fn handle_error(&self, error: &CarlaError);
}

pub fn set_global_error_handler(handler: Box<dyn ErrorHandler>) { ... }
```

## Compatibility Notes

### Semver Considerations

- **0.13.0**: Introduce new error types, deprecate anyhow usage
- **0.14.0**: Complete migration, all APIs use CarlaError
- **1.0.0**: Remove deprecated anyhow-based functions

### Backward Compatibility

Users can gradually migrate by:
1. Adding `use carla::{CarlaError, Result};` to modules
2. Replacing `anyhow::Result` with `carla::Result`
3. Optionally adding error-specific handling

Existing code with `?` operator continues to work unchanged.

## References

- [Rust Error Handling Book](https://doc.rust-lang.org/book/ch09-00-error-handling.html)
- [thiserror crate](https://docs.rs/thiserror/)
- [CARLA Python API Exceptions](https://carla.readthedocs.io/en/latest/python_api/)

---

**Status:** Proposed (Awaiting approval and prioritization)
**Next Steps:** Review proposal, estimate effort, schedule implementation
