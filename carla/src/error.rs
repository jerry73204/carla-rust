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
