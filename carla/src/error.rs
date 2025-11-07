//! Error types for CARLA operations.
//!
//! This module provides structured error types that classify failures into actionable categories,
//! enabling users to handle specific error conditions (like connection failures) differently from others.
//!
//! # Error Categories
//!
//! - [`ConnectionError`] - Network issues, timeouts, disconnections (→ reconnect, retry)
//! - [`ResourceError`] - Not found, destroyed, unavailable (→ check availability, recreate)
//! - [`OperationError`] - Spawn failures, physics errors (→ validate preconditions, retry)
//! - [`ValidationError`] - Invalid parameters, out of bounds (→ fix input)
//! - [`MapError`] - Map loading, waypoint issues (→ load different map, fallback)
//! - [`SensorError`] - Data unavailable, configuration errors (→ reconfigure, use fallback)
//! - [`InternalError`] - FFI errors, bugs (→ report issue)
//!
//! # Examples
//!
//! ## Connection Retry with Exponential Backoff
//!
//! ```no_run
//! use carla::{CarlaError, Client, ConnectionError};
//! use std::time::Duration;
//!
//! fn connect_with_retry(host: &str, port: u16, max_retries: u32) -> carla::Result<Client> {
//!     let mut backoff = Duration::from_millis(100);
//!
//!     for attempt in 0..max_retries {
//!         match Client::connect(host, port, None) {
//!             Ok(client) => return Ok(client),
//!             Err(CarlaError::Connection(ConnectionError::Timeout { .. })) => {
//!                 eprintln!(
//!                     "Connection timeout (attempt {}/{}), retrying in {:?}...",
//!                     attempt + 1,
//!                     max_retries,
//!                     backoff
//!                 );
//!                 std::thread::sleep(backoff);
//!                 backoff *= 2; // Exponential backoff
//!             }
//!             Err(e) => return Err(e), // Other errors are not retryable
//!         }
//!     }
//!
//!     Err(CarlaError::Connection(ConnectionError::Timeout {
//!         operation: "connect".to_string(),
//!         duration: backoff,
//!         source: None,
//!     }))
//! }
//! ```
//!
//! ## Spawn with Fallback Locations
//!
//! ```no_run
//! use carla::{CarlaError, OperationError};
//! # use carla::client::{ActorBlueprint, World, Actor};
//!
//! fn spawn_vehicle_with_fallback(
//!     world: &World,
//!     blueprint: &ActorBlueprint,
//! ) -> carla::Result<Actor> {
//!     let spawn_points = world.map().recommended_spawn_points();
//!
//!     for (i, transform) in spawn_points.iter().enumerate() {
//!         match world.spawn_actor(blueprint, transform) {
//!             Ok(actor) => {
//!                 println!("Spawned at spawn point {}", i);
//!                 return Ok(actor);
//!             }
//!             Err(CarlaError::Operation(OperationError::SpawnFailed { reason, .. })) => {
//!                 eprintln!("Spawn point {} failed: {}, trying next...", i, reason);
//!                 continue;
//!             }
//!             Err(e) => return Err(e), // Other errors are fatal
//!         }
//!     }
//!
//!     Err(CarlaError::Operation(OperationError::SpawnFailed {
//!         blueprint: blueprint.id().to_string(),
//!         transform: "all spawn points".to_string(),
//!         reason: "All spawn points exhausted".to_string(),
//!         source: None,
//!     }))
//! }
//! ```

pub mod ffi;

use std::{fmt, time::Duration};

/// Main error type for all CARLA operations.
///
/// This enum categorizes errors into actionable groups, allowing users to handle
/// specific error conditions with appropriate recovery strategies.
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

/// Connection and network errors.
///
/// These errors typically indicate transient issues that may be resolved by:
/// - Retrying with exponential backoff
/// - Reconnecting to the server
/// - Checking network connectivity
#[derive(Debug, thiserror::Error)]
pub enum ConnectionError {
    /// Operation timed out waiting for server response.
    ///
    /// This usually indicates:
    /// - Network congestion
    /// - Server overload
    /// - Operation taking longer than expected
    ///
    /// **Recovery**: Retry with longer timeout or exponential backoff
    #[error("Operation '{operation}' timed out after {duration:?}")]
    Timeout {
        /// The operation that timed out
        operation: String,
        /// Duration waited before timeout
        duration: Duration,
        /// Optional source error
        #[source]
        source: Option<Box<dyn std::error::Error + Send + Sync>>,
    },

    /// Lost connection to CARLA server.
    ///
    /// This indicates the connection to the server was lost during an operation.
    ///
    /// **Recovery**: Reconnect to server and retry operation
    #[error("Disconnected from CARLA server: {reason}")]
    Disconnected {
        /// Reason for disconnection
        reason: String,
        /// Optional source error
        #[source]
        source: Option<Box<dyn std::error::Error + Send + Sync>>,
    },

    /// Invalid server endpoint configuration.
    ///
    /// This indicates the host/port combination is invalid or unreachable.
    ///
    /// **Recovery**: Check server configuration and network settings
    #[error("Invalid server endpoint: {host}:{port}")]
    InvalidEndpoint {
        /// Host address
        host: String,
        /// Port number
        port: u16,
        /// Optional source error
        #[source]
        source: Option<Box<dyn std::error::Error + Send + Sync>>,
    },
}

/// Resource not found, unavailable, or destroyed.
///
/// These errors indicate issues with CARLA resources (actors, blueprints, maps, etc.).
///
/// **Recovery strategies**:
/// - Check resource existence before use
/// - Use alternative resources
/// - Recreate destroyed resources
#[derive(Debug, thiserror::Error)]
pub enum ResourceError {
    /// Resource not found (actor, blueprint, map, etc.).
    ///
    /// This usually means:
    /// - Blueprint ID is invalid or not in library
    /// - Actor has been destroyed
    /// - Map is not available on server
    ///
    /// **Recovery**: Use different resource or check availability first
    #[error("{resource_type} not found: {identifier}{}", context.as_ref().map(|c| format!(" ({})", c)).unwrap_or_default())]
    NotFound {
        /// Type of resource that was not found
        resource_type: ResourceType,
        /// Identifier (ID, name, etc.)
        identifier: String,
        /// Optional context information
        context: Option<String>,
    },

    /// Resource already exists (duplicate ID, etc.).
    ///
    /// **Recovery**: Use existing resource or choose different identifier
    #[error("{resource_type} already exists: {identifier}")]
    AlreadyExists {
        /// Type of resource
        resource_type: ResourceType,
        /// Identifier
        identifier: String,
    },

    /// Resource was destroyed/deleted.
    ///
    /// This indicates attempting to use a resource that has been destroyed.
    ///
    /// **Recovery**: Recreate resource if needed
    #[error("{resource_type} was destroyed: {identifier}")]
    Destroyed {
        /// Type of resource
        resource_type: ResourceType,
        /// Identifier
        identifier: String,
    },

    /// Resource temporarily unavailable.
    ///
    /// **Recovery**: Wait and retry, or use alternative resource
    #[error("{resource_type} unavailable: {reason}")]
    Unavailable {
        /// Type of resource
        resource_type: ResourceType,
        /// Reason for unavailability
        reason: String,
    },
}

/// Types of resources in CARLA.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ResourceType {
    /// Actor (vehicle, walker, sensor, etc.)
    Actor,
    /// Blueprint (actor template)
    Blueprint,
    /// Map (world map)
    Map,
    /// Sensor (camera, lidar, etc.)
    Sensor,
    /// Traffic light
    TrafficLight,
    /// Waypoint (navigation point)
    Waypoint,
    /// Episode (simulation instance)
    Episode,
    /// Recording (replay data)
    Recording,
}

/// Operation failures during simulation.
///
/// These errors indicate that a simulation operation failed due to:
/// - Collision with existing objects
/// - Invalid simulation state
/// - Physics constraints
///
/// **Recovery strategies**:
/// - Validate preconditions before operation
/// - Retry with different parameters
/// - Skip operation if not critical
#[derive(Debug, thiserror::Error)]
pub enum OperationError {
    /// Failed to spawn actor.
    ///
    /// Common causes:
    /// - Spawn location collides with existing actor
    /// - Spawn location out of map bounds
    /// - Blueprint configuration invalid
    ///
    /// **Recovery**: Try different spawn location or validate location first
    #[error("Failed to spawn actor '{blueprint}' at {transform}: {reason}")]
    SpawnFailed {
        /// Blueprint ID
        blueprint: String,
        /// Spawn transform (formatted for display)
        transform: String,
        /// Reason for failure
        reason: String,
        /// Optional source error
        #[source]
        source: Option<Box<dyn std::error::Error + Send + Sync>>,
    },

    /// Invalid transform (collision, out of bounds, etc.).
    ///
    /// **Recovery**: Validate transform before use
    #[error("Invalid transform {transform}: {reason}")]
    InvalidTransform {
        /// Transform (formatted for display)
        transform: String,
        /// Reason for invalidity
        reason: String,
    },

    /// Operation requires physics to be enabled.
    ///
    /// **Recovery**: Enable physics on actor before operation
    #[error("Operation '{operation}' requires physics for actor {actor_id}")]
    PhysicsDisabled {
        /// Actor ID
        actor_id: u32,
        /// Operation that requires physics
        operation: String,
    },

    /// General simulation error.
    ///
    /// **Recovery**: Check simulation state and logs
    #[error("Simulation error: {message}")]
    SimulationError {
        /// Error message
        message: String,
        /// Optional source error
        #[source]
        source: Option<Box<dyn std::error::Error + Send + Sync>>,
    },
}

/// Parameter validation errors.
///
/// These errors indicate that user-provided parameters are invalid.
///
/// **Recovery**: Fix parameters and retry
#[derive(Debug, thiserror::Error)]
pub enum ValidationError {
    /// Invalid blueprint ID or configuration.
    ///
    /// **Recovery**: Check blueprint ID and attributes
    #[error("Invalid blueprint '{blueprint_id}': {reason}")]
    InvalidBlueprint {
        /// Blueprint ID
        blueprint_id: String,
        /// Reason for invalidity
        reason: String,
    },

    /// Invalid attribute value.
    ///
    /// **Recovery**: Use valid attribute value from blueprint
    #[error("Invalid attribute '{name}' = '{value}': {reason}")]
    InvalidAttribute {
        /// Attribute name
        name: String,
        /// Attribute value
        value: String,
        /// Reason for invalidity
        reason: String,
    },

    /// Value out of valid range.
    ///
    /// **Recovery**: Use value within specified range
    #[error("Value {value} out of bounds [{min}, {max}] for field '{field}'")]
    OutOfBounds {
        /// Field name
        field: String,
        /// Actual value
        value: String,
        /// Minimum valid value
        min: String,
        /// Maximum valid value
        max: String,
    },

    /// Invalid configuration.
    ///
    /// **Recovery**: Check configuration documentation
    #[error("Invalid configuration for '{setting}': {reason}")]
    InvalidConfiguration {
        /// Configuration setting name
        setting: String,
        /// Reason for invalidity
        reason: String,
    },
}

/// Map-related errors.
///
/// These errors occur during map operations.
///
/// **Recovery strategies**:
/// - Load different map
/// - Use spawn points instead of custom locations
/// - Check map availability on server
#[derive(Debug, thiserror::Error)]
pub enum MapError {
    /// Failed to load map.
    ///
    /// **Recovery**: Check map name and server availability
    #[error("Failed to load map '{map_name}': {reason}")]
    LoadFailed {
        /// Map name
        map_name: String,
        /// Reason for failure
        reason: String,
        /// Optional source error
        #[source]
        source: Option<Box<dyn std::error::Error + Send + Sync>>,
    },

    /// Invalid waypoint location.
    ///
    /// **Recovery**: Use recommended spawn points or validate location
    #[error("Invalid waypoint at {location}: {reason}")]
    InvalidWaypoint {
        /// Location (formatted for display)
        location: String,
        /// Reason for invalidity
        reason: String,
    },

    /// Map topology error.
    ///
    /// **Recovery**: Report issue, may indicate map data corruption
    #[error("Map topology error: {message}")]
    TopologyError {
        /// Error message
        message: String,
    },
}

/// Sensor-related errors.
///
/// These errors occur during sensor operations.
///
/// **Recovery strategies**:
/// - Reconfigure sensor
/// - Use fallback data source
/// - Check sensor availability
#[derive(Debug, thiserror::Error)]
pub enum SensorError {
    /// Sensor data not available.
    ///
    /// **Recovery**: Wait for sensor data or use last known data
    #[error("Sensor data unavailable for sensor {sensor_id}: {reason}")]
    DataUnavailable {
        /// Sensor actor ID
        sensor_id: u32,
        /// Reason for unavailability
        reason: String,
    },

    /// Invalid sensor configuration.
    ///
    /// **Recovery**: Check sensor blueprint attributes
    #[error("Invalid sensor configuration for '{sensor_type}': {reason}")]
    InvalidConfiguration {
        /// Sensor type
        sensor_type: String,
        /// Reason for invalidity
        reason: String,
    },

    /// Failed to start listening to sensor.
    ///
    /// **Recovery**: Check sensor state and try again
    #[error("Failed to listen to sensor {sensor_id}: {reason}")]
    ListenFailed {
        /// Sensor actor ID
        sensor_id: u32,
        /// Reason for failure
        reason: String,
        /// Optional source error
        #[source]
        source: Option<Box<dyn std::error::Error + Send + Sync>>,
    },
}

/// Internal errors (FFI, unexpected states).
///
/// These errors usually indicate bugs or unexpected conditions.
///
/// **Recovery**: Report issue with full error details
#[derive(Debug, thiserror::Error)]
pub enum InternalError {
    /// FFI boundary error.
    ///
    /// **Recovery**: Report issue, may indicate library bug
    #[error("FFI error: {message}")]
    FfiError {
        /// Error message
        message: String,
        /// Optional source error
        #[source]
        source: Option<Box<dyn std::error::Error + Send + Sync>>,
    },

    /// Serialization/deserialization error.
    ///
    /// **Recovery**: Report issue with data format
    #[error("Serialization error in {context}")]
    SerializationError {
        /// Context where serialization failed
        context: String,
        /// Source error
        #[source]
        source: Box<dyn std::error::Error + Send + Sync>,
    },

    /// Unexpected internal state (likely a bug).
    ///
    /// **Recovery**: Report issue with full error details
    #[error("Unexpected state: {description}")]
    UnexpectedState {
        /// Description of unexpected state
        description: String,
    },
}

/// Result type for CARLA operations.
pub type Result<T> = std::result::Result<T, CarlaError>;

impl ResourceType {
    /// Returns the string representation of this resource type.
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

impl CarlaError {
    /// Returns `true` if this is a connection-related error.
    ///
    /// # Examples
    ///
    /// ```
    /// # use carla::{CarlaError, ConnectionError};
    /// # use std::time::Duration;
    /// let error: CarlaError = ConnectionError::Timeout {
    ///     operation: "spawn".to_string(),
    ///     duration: Duration::from_secs(5),
    ///     source: None,
    /// }
    /// .into();
    ///
    /// assert!(error.is_connection_error());
    /// ```
    pub fn is_connection_error(&self) -> bool {
        matches!(self, CarlaError::Connection(_))
    }

    /// Returns `true` if this is a timeout error.
    ///
    /// # Examples
    ///
    /// ```
    /// # use carla::{CarlaError, ConnectionError};
    /// # use std::time::Duration;
    /// let error: CarlaError = ConnectionError::Timeout {
    ///     operation: "spawn".to_string(),
    ///     duration: Duration::from_secs(5),
    ///     source: None,
    /// }
    /// .into();
    ///
    /// assert!(error.is_timeout());
    /// ```
    pub fn is_timeout(&self) -> bool {
        matches!(
            self,
            CarlaError::Connection(ConnectionError::Timeout { .. })
        )
    }

    /// Returns `true` if this is a resource not found error.
    ///
    /// # Examples
    ///
    /// ```
    /// # use carla::{CarlaError, ResourceError, ResourceType};
    /// let error: CarlaError = ResourceError::NotFound {
    ///     resource_type: ResourceType::Blueprint,
    ///     identifier: "vehicle.tesla.model3".to_string(),
    ///     context: None,
    /// }
    /// .into();
    ///
    /// assert!(error.is_not_found());
    /// ```
    pub fn is_not_found(&self) -> bool {
        matches!(self, CarlaError::Resource(ResourceError::NotFound { .. }))
    }

    /// Returns `true` if this error should be retried.
    ///
    /// This includes:
    /// - Timeout errors
    /// - Disconnection errors
    /// - Some spawn failures
    ///
    /// # Examples
    ///
    /// ```
    /// # use carla::{CarlaError, ConnectionError};
    /// # use std::time::Duration;
    /// let error: CarlaError = ConnectionError::Timeout {
    ///     operation: "spawn".to_string(),
    ///     duration: Duration::from_secs(5),
    ///     source: None,
    /// }
    /// .into();
    ///
    /// assert!(error.is_retriable());
    /// ```
    pub fn is_retriable(&self) -> bool {
        matches!(
            self,
            CarlaError::Connection(ConnectionError::Timeout { .. })
                | CarlaError::Connection(ConnectionError::Disconnected { .. })
                | CarlaError::Operation(OperationError::SpawnFailed { .. })
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_connection_timeout_display() {
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
    fn test_resource_not_found_display() {
        let error = ResourceError::NotFound {
            resource_type: ResourceType::Blueprint,
            identifier: "vehicle.tesla.model3".to_string(),
            context: Some("BlueprintLibrary::find".to_string()),
        };
        assert_eq!(
            format!("{}", error),
            "Blueprint not found: vehicle.tesla.model3 (BlueprintLibrary::find)"
        );
    }

    #[test]
    fn test_error_classification() {
        let error: CarlaError = ResourceError::NotFound {
            resource_type: ResourceType::Blueprint,
            identifier: "test".to_string(),
            context: None,
        }
        .into();

        assert!(error.is_not_found());
        assert!(!error.is_timeout());
        assert!(!error.is_connection_error());
        assert!(!error.is_retriable());
    }

    #[test]
    fn test_timeout_is_retriable() {
        let error: CarlaError = ConnectionError::Timeout {
            operation: "spawn".to_string(),
            duration: Duration::from_secs(5),
            source: None,
        }
        .into();

        assert!(error.is_timeout());
        assert!(error.is_connection_error());
        assert!(error.is_retriable());
    }

    #[test]
    fn test_spawn_failed_is_retriable() {
        let error: CarlaError = OperationError::SpawnFailed {
            blueprint: "vehicle.tesla.model3".to_string(),
            transform: "(0, 0, 0)".to_string(),
            reason: "collision".to_string(),
            source: None,
        }
        .into();

        assert!(error.is_retriable());
        assert!(!error.is_connection_error());
    }

    #[test]
    fn test_resource_type_display() {
        assert_eq!(ResourceType::Actor.as_str(), "Actor");
        assert_eq!(ResourceType::Blueprint.as_str(), "Blueprint");
        assert_eq!(format!("{}", ResourceType::Map), "Map");
    }

    #[test]
    fn test_validation_error_display() {
        let error = ValidationError::OutOfBounds {
            field: "throttle".to_string(),
            value: "1.5".to_string(),
            min: "0.0".to_string(),
            max: "1.0".to_string(),
        };
        assert_eq!(
            format!("{}", error),
            "Value 1.5 out of bounds [0.0, 1.0] for field 'throttle'"
        );
    }

    #[test]
    fn test_io_error_conversion() {
        let io_error = std::io::Error::new(std::io::ErrorKind::NotFound, "file not found");
        let error: CarlaError = io_error.into();
        assert!(matches!(error, CarlaError::Io(_)));
    }
}
