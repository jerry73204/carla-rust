//! Error types for CARLA operations.
//!
//! This module provides comprehensive error handling that mirrors the exception
//! hierarchy from CARLA's C++ API while leveraging Rust's Result type system.

use std::time::Duration;
use thiserror::Error;

use crate::actor::VehicleControl;

/// Main error type for all CARLA operations.
#[derive(Debug, Error)]
pub enum CarlaError {
    /// Client connection errors
    #[error("Client error: {0}")]
    Client(#[from] ClientError),

    /// World operation errors
    #[error("World error: {0}")]
    World(#[from] WorldError),

    /// Actor operation errors
    #[error("Actor error: {0}")]
    Actor(#[from] ActorError),

    /// Sensor operation errors
    #[error("Sensor error: {0}")]
    Sensor(#[from] SensorError),

    /// Actor spawning errors
    #[error("Spawn error: {0}")]
    Spawn(#[from] SpawnError),

    /// Traffic manager errors
    #[error("Traffic manager error: {0}")]
    TrafficManager(#[from] TrafficManagerError),

    /// Map and waypoint errors
    #[error("Map error: {0}")]
    Map(#[from] MapError),

    /// Vehicle operation errors
    #[error("Vehicle error: {0}")]
    Vehicle(#[from] VehicleError),

    /// Walker (pedestrian) operation errors
    #[error("Walker error: {0}")]
    Walker(#[from] WalkerError),

    /// Actor destruction errors
    #[error("Destroy error: {0}")]
    Destroy(#[from] DestroyError),

    /// Generic CARLA runtime error
    #[error("CARLA runtime error: {0}")]
    Runtime(String),

    /// FFI layer error from carla-sys
    #[error("FFI error: {0}")]
    Ffi(#[from] anyhow::Error),
}

/// Errors related to client connection and communication.
#[derive(Debug, Error)]
pub enum ClientError {
    /// Failed to connect to CARLA server
    #[error("Connection failed to {host}:{port} - {reason}")]
    ConnectionFailed {
        /// Server hostname
        host: String,
        /// Server port
        port: u16,
        /// Failure reason
        reason: String,
    },

    /// Invalid hostname or address
    #[error("Invalid host: {0}")]
    InvalidHost(String),

    /// Connection timeout
    #[error("Connection timeout after {0:?}")]
    Timeout(Duration),

    /// Requested map not found
    #[error("Map '{map_name}' not found. Available maps: {available_maps:?}")]
    MapNotFound {
        /// Requested map name
        map_name: String,
        /// List of available maps
        available_maps: Vec<String>,
    },

    /// Server version mismatch
    #[error("Version mismatch: client {client_version}, server {server_version}")]
    VersionMismatch {
        /// Client library version
        client_version: String,
        /// Server version
        server_version: String,
    },

    /// RPC communication error
    #[error("RPC error: {0}")]
    Rpc(String),

    /// Server disconnect
    #[error("Server disconnected unexpectedly")]
    Disconnected,

    /// Server communication failed after connection
    #[error("Server communication failed: {reason}")]
    ServerCommunicationFailed {
        /// Failure reason
        reason: String,
    },
}

/// Errors related to world operations.
#[derive(Debug, Error)]
pub enum WorldError {
    /// World tick timeout
    #[error("World tick timeout after {0:?}")]
    TickTimeout(Duration),

    /// Invalid world state
    #[error("Invalid world state: {0}")]
    InvalidState(String),

    /// Map loading failed
    #[error("Failed to load map '{map_name}': {reason}")]
    MapLoadFailed {
        /// Map name that failed to load
        map_name: String,
        /// Failure reason
        reason: String,
    },

    /// Blueprint not found
    #[error("Blueprint not found: {0}")]
    BlueprintNotFound(String),

    /// Weather setting error
    #[error("Weather error: {0}")]
    Weather(String),

    /// Settings are invalid
    #[error("Settings invalid: {0}")]
    SettingsInvalid(String),

    /// Tick operation failed
    #[error("Tick failed: {0}")]
    TickFailed(String),
}

/// Errors related to actor operations.
#[derive(Debug, Error)]
pub enum ActorError {
    /// Actor not found
    #[error("Actor not found: ID {0}")]
    NotFound(u32),

    /// Invalid actor type for operation
    #[error("Invalid actor type: expected {expected}, got {actual}")]
    InvalidType {
        /// Expected actor type
        expected: String,
        /// Actual actor type
        actual: String,
    },

    /// Actor is destroyed
    #[error("Actor {0} is destroyed")]
    Destroyed(u32),

    /// Actor destruction failed
    #[error("Failed to destroy actor {0}")]
    DestroyFailed(u32),

    /// Transform operation failed
    #[error("Transform operation failed: {0}")]
    Transform(String),

    /// Attachment failed
    #[error("Failed to attach actor {child} to {parent}: {reason}")]
    AttachmentFailed {
        /// Child actor ID
        child: u32,
        /// Parent actor ID
        parent: u32,
        /// Failure reason
        reason: String,
    },

    /// Control application failed
    #[error("Control application failed: {0}")]
    ControlFailed(String),

    /// Generic operation failed
    #[error("Operation failed: {0}")]
    OperationFailed(String),

    /// Attribute is not modifiable
    #[error("Attribute '{0}' is not modifiable")]
    AttributeNotModifiable(String),

    /// Actor operation crashed
    #[error("Actor operation crashed: {operation} on actor {actor_id}")]
    ActorCrashed {
        /// Actor ID that crashed
        actor_id: u32,
        /// Operation that caused the crash
        operation: String,
    },

    /// Actor is in invalid state
    #[error("Actor {actor_id} is in invalid state for operation: {operation}")]
    ActorInvalid {
        /// Actor ID
        actor_id: u32,
        /// Operation attempted
        operation: String,
    },

    /// Traffic Manager is not available
    #[error("Traffic Manager is not available")]
    TrafficManagerUnavailable,

    /// Actor lifecycle violation
    #[error("Lifecycle violation for actor {actor_id}: {violation}")]
    LifecycleViolation {
        /// Actor ID
        actor_id: u32,
        /// Violation description
        violation: String,
    },
}

/// Errors related to actor spawning.
#[derive(Debug, Error)]
pub enum SpawnError {
    /// Spawn location is occupied
    #[error("Spawn location occupied at {location:?}")]
    LocationOccupied {
        /// Attempted spawn location
        location: crate::geom::Location,
    },

    /// Invalid blueprint for spawning
    #[error("Invalid blueprint: {blueprint_id} - {reason}")]
    InvalidBlueprint {
        /// Blueprint ID
        blueprint_id: String,
        /// Reason why invalid
        reason: String,
    },

    /// Blueprint attribute error
    #[error("Blueprint attribute error: {attribute} - {reason}")]
    AttributeError {
        /// Attribute name
        attribute: String,
        /// Error reason
        reason: String,
    },

    /// Spawn point not available
    #[error("No spawn points available")]
    NoSpawnPoints,

    /// Parent actor invalid for attachment
    #[error("Invalid parent actor: {0}")]
    InvalidParent(u32),

    /// Server-side spawn failure
    #[error("Server spawn failure: {0}")]
    ServerError(String),
}

/// Errors related to sensor operations.
#[derive(Debug, Error)]
pub enum SensorError {
    /// Sensor not listening
    #[error("Sensor {0} is not listening")]
    NotListening(u32),

    /// Already listening
    #[error("Sensor {0} is already listening")]
    AlreadyListening(u32),

    /// Invalid sensor configuration
    #[error("Invalid sensor configuration: {0}")]
    InvalidConfiguration(String),

    /// Sensor data processing error
    #[error("Data processing error: {0}")]
    DataProcessing(String),

    /// Callback registration failed
    #[error("Callback registration failed: {0}")]
    CallbackFailed(String),

    /// Sensor synchronization error
    #[error("Synchronization error: {0}")]
    Synchronization(String),
}

/// Errors related to traffic manager operations.
#[derive(Debug, Error)]
pub enum TrafficManagerError {
    /// Traffic manager not found on port
    #[error("Traffic manager not found on port {0}")]
    NotFound(u16),

    /// Vehicle not registered
    #[error("Vehicle {0} not registered with traffic manager")]
    VehicleNotRegistered(u32),

    /// Invalid traffic manager configuration
    #[error("Invalid configuration: {0}")]
    InvalidConfiguration(String),

    /// Vehicle behavior setting failed
    #[error("Failed to set vehicle behavior: {0}")]
    BehaviorFailed(String),

    /// Route planning failed
    #[error("Route planning failed: {0}")]
    RoutePlanningFailed(String),

    /// Synchronization mode error
    #[error("Synchronization mode error: {0}")]
    SynchronizationError(String),
}

/// Errors related to map and waypoint operations.
#[derive(Debug, Error)]
pub enum MapError {
    /// Waypoint not found
    #[error("Waypoint not found at location {location:?}")]
    WaypointNotFound {
        /// Requested location
        location: crate::geom::Location,
        /// Optional lane type filter
        lane_type: Option<String>,
    },

    /// Invalid lane ID
    #[error("Invalid lane ID: {0}")]
    InvalidLaneId(i32),

    /// Junction not found
    #[error("Junction not found: {0}")]
    JunctionNotFound(i32),

    /// Topology error
    #[error("Map topology error: {0}")]
    Topology(String),

    /// OpenDRIVE parsing error
    #[error("OpenDRIVE parsing error: {0}")]
    OpenDriveParsing(String),

    /// Navigation error
    #[error("Navigation error: {0}")]
    Navigation(String),
}

/// Errors related to vehicle operations.
#[derive(Debug, Error)]
pub enum VehicleError {
    /// Vehicle control command failed
    #[error("Vehicle control failed: {0}")]
    ControlFailed(String),

    /// Physics control application failed
    #[error("Vehicle physics control failed: {0}")]
    PhysicsControlFailed(String),

    /// Light state operation failed
    #[error("Vehicle light state operation failed: {0}")]
    LightStateFailed(String),

    /// Door operation failed
    #[error("Vehicle door operation failed: {0}")]
    DoorOperationFailed(String),

    /// Autopilot operation failed
    #[error("Vehicle autopilot operation failed: {0}")]
    AutopilotFailed(String),

    /// Invalid control parameters
    #[error("Invalid vehicle control: {parameter} = {value}")]
    InvalidControl {
        /// Parameter that was invalid
        parameter: String,
        /// Value that was invalid
        value: String,
    },
}

/// Errors related to actor destruction operations.
#[derive(Debug, Error)]
pub enum DestroyError {
    /// Actor was already destroyed
    #[error("Actor {actor_id} was already destroyed")]
    AlreadyDestroyed {
        /// The ID of the actor that was already destroyed
        actor_id: crate::actor::ActorId,
    },

    /// Server communication failed during destruction
    #[error("Failed to destroy actor {actor_id}: {message}")]
    ServerError {
        /// The ID of the actor being destroyed
        actor_id: crate::actor::ActorId,
        /// Error message from the server or network layer
        message: String,
    },

    /// Actor is in invalid state for destruction
    #[error("Actor {actor_id} is in invalid state for destruction")]
    InvalidActor {
        /// The ID of the invalid actor
        actor_id: crate::actor::ActorId,
    },

    /// RPC timeout during destruction
    #[error("Destruction of actor {actor_id} timed out after {timeout:?}")]
    Timeout {
        /// The ID of the actor being destroyed
        actor_id: crate::actor::ActorId,
        /// The timeout duration that was exceeded
        timeout: Duration,
    },
}

/// Errors related to walker (pedestrian) operations.
#[derive(Debug, Error)]
pub enum WalkerError {
    /// Walker control command failed
    #[error("Walker control failed: {0}")]
    ControlFailed(String),

    /// Walker pose control operation failed
    #[error("Walker pose control failed: {0}")]
    PoseControlFailed(String),

    /// Walker navigation operation failed
    #[error("Walker navigation failed: {0}")]
    NavigationFailed(String),

    /// Walker bone control operation failed
    #[error("Walker bone control failed: {0}")]
    BoneControlFailed(String),
}

impl From<VehicleControl> for String {
    fn from(control: VehicleControl) -> Self {
        format!(
            "VehicleControl(throttle: {}, steer: {}, brake: {})",
            control.throttle, control.steer, control.brake
        )
    }
}

/// Helper trait for converting carla-sys errors to our error types
pub trait FromCxxError {
    /// Convert from carla-sys error with context
    fn from_cxx_error(error: anyhow::Error, context: &str) -> Self;
}

impl FromCxxError for CarlaError {
    fn from_cxx_error(error: anyhow::Error, context: &str) -> Self {
        CarlaError::Runtime(format!("{}: {}", context, error))
    }
}

impl FromCxxError for ClientError {
    fn from_cxx_error(error: anyhow::Error, context: &str) -> Self {
        ClientError::Rpc(format!("{}: {}", context, error))
    }
}

impl FromCxxError for ActorError {
    fn from_cxx_error(error: anyhow::Error, context: &str) -> Self {
        ActorError::ControlFailed(format!("{}: {}", context, error))
    }
}

/// Result type alias for operations that can fail
pub type CarlaResult<T> = Result<T, CarlaError>;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geom::Location;

    #[test]
    fn test_carla_error_display() {
        let client_error = CarlaError::Client(ClientError::ConnectionFailed {
            host: "localhost".to_string(),
            port: 2000,
            reason: "Connection refused".to_string(),
        });

        let error_string = format!("{}", client_error);
        assert!(error_string.contains("Client error"));
        assert!(error_string.contains("Connection failed"));
        assert!(error_string.contains("localhost:2000"));
    }

    #[test]
    fn test_client_error_variants() {
        let connection_error = ClientError::ConnectionFailed {
            host: "192.168.1.100".to_string(),
            port: 2000,
            reason: "Timeout".to_string(),
        };

        let map_error = ClientError::MapNotFound {
            map_name: "Town01".to_string(),
            available_maps: vec!["Town02".to_string(), "Town03".to_string()],
        };

        let rpc_error = ClientError::Rpc("RPC call failed".to_string());

        // Test that errors can be formatted
        assert!(format!("{}", connection_error).contains("192.168.1.100"));
        assert!(format!("{}", map_error).contains("Town01"));
        assert!(format!("{}", rpc_error).contains("RPC call failed"));
    }

    #[test]
    fn test_world_error_variants() {
        let map_load_error = WorldError::MapLoadFailed {
            map_name: "InvalidMap".to_string(),
            reason: "Map file not found".to_string(),
        };

        let settings_error = WorldError::SettingsInvalid("Invalid weather settings".to_string());
        let tick_error = WorldError::TickFailed("Server not responding".to_string());

        assert!(format!("{}", map_load_error).contains("InvalidMap"));
        assert!(format!("{}", settings_error).contains("Invalid weather"));
        assert!(format!("{}", tick_error).contains("not responding"));
    }

    #[test]
    fn test_actor_error_variants() {
        let not_found_error = ActorError::NotFound(12345);
        let control_error = ActorError::ControlFailed("Autopilot failed".to_string());
        let lifecycle_error = ActorError::LifecycleViolation {
            actor_id: 678,
            violation: "destroyed state during get_transform".to_string(),
        };

        assert!(format!("{}", not_found_error).contains("12345"));
        assert!(format!("{}", control_error).contains("Autopilot failed"));
        assert!(format!("{}", lifecycle_error).contains("678"));
        assert!(format!("{}", lifecycle_error).contains("destroyed"));
    }

    #[test]
    fn test_spawn_error_variants() {
        let location = Location {
            x: 10.0,
            y: 20.0,
            z: 0.5,
        };
        let location_error = SpawnError::LocationOccupied { location };

        let blueprint_error = SpawnError::InvalidBlueprint {
            blueprint_id: "vehicle.invalid.model".to_string(),
            reason: "Blueprint not found".to_string(),
        };

        let no_spawn_error = SpawnError::NoSpawnPoints;

        assert!(format!("{}", location_error).contains("10"));
        assert!(format!("{}", blueprint_error).contains("vehicle.invalid.model"));
        assert!(format!("{}", no_spawn_error).contains("No spawn points"));
    }

    #[test]
    fn test_sensor_error_variants() {
        let not_listening = SensorError::NotListening(123);
        let already_listening = SensorError::AlreadyListening(456);
        let config_error = SensorError::InvalidConfiguration("Invalid image size".to_string());

        assert!(format!("{}", not_listening).contains("123"));
        assert!(format!("{}", already_listening).contains("456"));
        assert!(format!("{}", config_error).contains("Invalid image size"));
    }

    #[test]
    fn test_traffic_manager_error_variants() {
        let not_found = TrafficManagerError::NotFound(8000);
        let not_registered = TrafficManagerError::VehicleNotRegistered(789);

        assert!(format!("{}", not_found).contains("8000"));
        assert!(format!("{}", not_registered).contains("789"));
    }

    #[test]
    fn test_vehicle_error_variants() {
        let _control = VehicleControl {
            throttle: 1.5, // Invalid value
            ..Default::default()
        };

        let control_error = VehicleError::InvalidControl {
            parameter: "throttle".to_string(),
            value: "1.5".to_string(),
        };

        let physics_error = VehicleError::PhysicsControlFailed("Invalid mass".to_string());

        assert!(format!("{}", control_error).contains("throttle"));
        assert!(format!("{}", physics_error).contains("Invalid mass"));
    }

    #[test]
    fn test_map_error_variants() {
        let opendrive_error = MapError::OpenDriveParsing("Invalid XML".to_string());
        let waypoint_error = MapError::WaypointNotFound {
            location: Location {
                x: 100.0,
                y: 200.0,
                z: 1.0,
            },
            lane_type: None,
        };

        assert!(format!("{}", opendrive_error).contains("Invalid XML"));
        assert!(format!("{}", waypoint_error).contains("100"));
        assert!(format!("{}", waypoint_error).contains("200"));
    }

    #[test]
    fn test_error_conversion() {
        let client_error = ClientError::ConnectionFailed {
            host: "test".to_string(),
            port: 2000,
            reason: "test".to_string(),
        };

        let carla_error: CarlaError = client_error.into();

        match carla_error {
            CarlaError::Client(_) => {} // Expected
            _ => panic!("Expected Client error"),
        }
    }

    #[test]
    fn test_carla_result_type() {
        let success_value = 42;
        let success: CarlaResult<i32> = Ok(success_value);
        let failure: CarlaResult<i32> = Err(CarlaError::Runtime("Test error".to_string()));

        assert!(success.is_ok());
        assert!(failure.is_err());
        assert_eq!(success_value, 42);
    }

    #[test]
    fn test_error_debug_format() {
        let error = CarlaError::Runtime("Debug test".to_string());
        let debug_string = format!("{:?}", error);

        assert!(debug_string.contains("Runtime"));
        assert!(debug_string.contains("Debug test"));
    }

    #[test]
    fn test_nested_error_types() {
        let spawn_error = SpawnError::LocationOccupied {
            location: Location {
                x: 1.0,
                y: 2.0,
                z: 3.0,
            },
        };

        let carla_error = CarlaError::Spawn(spawn_error);

        match carla_error {
            CarlaError::Spawn(SpawnError::LocationOccupied { location }) => {
                assert_eq!(location.x, 1.0);
                assert_eq!(location.y, 2.0);
                assert_eq!(location.z, 3.0);
            }
            _ => panic!("Expected nested spawn error"),
        }
    }

    #[test]
    fn test_from_cxx_error_trait() {
        let anyhow_error = anyhow::anyhow!("Test FFI error");
        let client_error = ClientError::from_cxx_error(anyhow_error, "test_operation");

        match client_error {
            ClientError::Rpc(msg) => {
                assert!(msg.contains("test_operation"));
                assert!(msg.contains("Test FFI error"));
            }
            _ => panic!("Expected RPC error"),
        }
    }
}
