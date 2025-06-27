//! # CARLA Client and World Management
//!
//! This module provides the core client interface for connecting to CARLA servers
//! and managing simulation worlds. It mirrors CARLA's `carla::client` namespace
//! and provides the primary entry point for all CARLA operations.
//!
//! ## Core Components
//!
//! ### Client Connection
//! The [`Client`] struct handles server connections and high-level operations:
//! - Server connection management with automatic retry
//! - Map loading and world management
//! - Recording and playback functionality
//! - Server version compatibility checking
//!
//! ### World Management
//! The [`World`] struct represents the simulation environment:
//! - Actor spawning and management
//! - Weather and environmental controls
//! - Sensor synchronization
//! - Blueprint library access
//! - Traffic manager integration
//!
//! ### Blueprint System
//! Actor creation uses a blueprint-based system:
//! - [`BlueprintLibrary`]: Collection of all available actor types
//! - [`ActorBlueprint`]: Template for spawning specific actors
//! - [`ActorAttribute`]: Configurable parameters for actor customization
//!
//! ## Quick Start Example
//!
//! ```rust,no_run
//! use carla::{
//!     client::Client,
//!     geom::{Location, Transform},
//! };
//!
//! # fn main() -> carla::error::CarlaResult<()> {
//! // Connect to CARLA server
//! let client = Client::new("localhost", 2000, None)?;
//!
//! // Check server compatibility
//! println!("Client version: {}", client.client_version());
//! println!("Server version: {}", client.server_version()?);
//!
//! // Get the current world
//! let world = client.world()?;
//!
//! // Access blueprint library
//! let blueprint_lib = world.blueprint_library()?;
//! let vehicle_bp = blueprint_lib
//!     .find("vehicle.tesla.model3")?
//!     .ok_or("Blueprint not found")?;
//!
//! // Spawn an actor
//! let spawn_point = Transform::new(Location::new(0.0, 0.0, 0.5), Default::default());
//! let actor = world.spawn_actor(&vehicle_bp, &spawn_point, None)?;
//!
//! println!("Spawned actor with ID: {}", actor.id());
//! # Ok(())
//! # }
//! ```
//!
//! ## Advanced Features
//!
//! ### Map Management
//! ```rust,no_run
//! # use carla::client::Client;
//! # fn map_example() -> carla::error::CarlaResult<()> {
//! let client = Client::new("localhost", 2000, None)?;
//!
//! // List available maps
//! let maps = client.available_maps()?;
//! println!("Available maps: {:?}", maps);
//!
//! // Load a specific map
//! let world = client.load_world("Town01")?;
//!
//! // Or reload current map
//! let world = client.reload_world()?;
//! # Ok(())
//! # }
//! ```
//!
//! ### Recording and Playback
//! ```rust,no_run
//! # use carla::client::Client;
//! # fn recording_example() -> carla::error::CarlaResult<()> {
//! let client = Client::new("localhost", 2000, None)?;
//!
//! // Start recording
//! client.start_recording("my_recording.log")?;
//!
//! // ... run simulation ...
//!
//! // Stop recording
//! client.stop_recorder()?;
//!
//! // Playback the recording
//! client.replay_recording("my_recording.log")?;
//! # Ok(())
//! # }
//! ```
//!
//! ### Weather Control
//! ```rust,no_run
//! # use carla::client::Client;
//! use carla::client::WeatherParameters;
//!
//! # fn weather_example() -> carla::error::CarlaResult<()> {
//! let client = Client::new("localhost", 2000, None)?;
//! let world = client.world()?;
//!
//! // Set custom weather
//! let weather = WeatherParameters {
//!     cloudiness: 80.0,
//!     precipitation: 30.0,
//!     sun_altitude_angle: 45.0,
//!     fog_density: 20.0,
//!     ..Default::default()
//! };
//! world.set_weather(&weather)?;
//! # Ok(())
//! # }
//! ```
//!
//! ## Error Handling
//!
//! All client operations return [`CarlaResult<T>`](crate::error::CarlaResult) for
//! comprehensive error handling:
//!
//! - **ClientError::ConnectionFailed**: Cannot connect to server
//! - **ClientError::MapNotFound**: Requested map not available
//! - **WorldError::MapLoadFailed**: Map loading failed
//! - **SpawnError**: Actor spawning failures
//!
//! ## Thread Safety
//!
//! The client and world types are **not** thread-safe due to CARLA's underlying
//! C++ client limitations. For multi-threaded applications:
//!
//! - Use separate client connections per thread
//! - Wrap in `Arc<Mutex<T>>` if sharing is necessary
//! - Prefer message passing between threads
//!
//! ## Performance Tips
//!
//! - **Connection Pooling**: Reuse client connections when possible
//! - **Batch Operations**: Use batch spawning for multiple actors
//! - **Synchronous Mode**: Enable for deterministic simulations
//! - **Recording**: Use for debugging and analysis workflows

mod blueprint;
mod blueprint_library;
mod client_impl;
mod world;

// Re-export all client types for convenient access
pub use blueprint::{ActorAttribute, ActorAttributeType, ActorBlueprint, AttributeValue, RGBColor};
pub use blueprint_library::BlueprintLibrary;
pub use client_impl::Client;
pub use world::{WeatherParameters, World, WorldSettings, WorldSnapshot};
