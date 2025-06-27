//! # CARLA Actor System
//!
//! This module provides comprehensive support for all CARLA actors including vehicles,
//! walkers (pedestrians), sensors, and traffic infrastructure. The actor system is the
//! core of CARLA's simulation, representing all dynamic and static entities in the world.
//!
//! ## Actor Types
//!
//! CARLA supports several categories of actors:
//!
//! ### Vehicles
//! - **Cars, Trucks, Motorcycles**: Controllable vehicles with physics simulation
//! - **Control**: Throttle, steering, braking, gear shifting, handbrake
//! - **Physics**: Mass, drag, engine characteristics, wheel properties
//! - **Features**: Autopilot, traffic manager integration, door control, lighting
//!
//! ### Sensors
//! - **Cameras**: RGB, depth, semantic/instance segmentation, optical flow
//! - **LiDAR**: Point cloud generation with configurable parameters
//! - **Radar**: Object detection with velocity and distance measurements
//! - **IMU**: Inertial measurement with accelerometer and gyroscope data
//! - **GNSS**: GPS-like positioning with configurable noise
//! - **Collision**: Collision detection and impulse measurement
//! - **Lane Invasion**: Lane boundary crossing detection
//! - **Obstacle Detection**: Distance-based obstacle sensing
//!
//! ### Pedestrians (Walkers)
//! - **AI Control**: Autonomous walking behavior
//! - **Physics**: Collision detection and response
//! - **Animation**: Walking, running, idle states
//!
//! ### Traffic Infrastructure
//! - **Traffic Lights**: State control (red, yellow, green, off)
//! - **Traffic Signs**: Static signage information
//! - **Timing**: Configurable light cycle durations
//!
//! ## Architecture
//!
//! The actor system uses a trait-based design for type safety and flexibility:
//!
//! ```rust,no_run
//! use carla::{
//!     actor::{ActorExt, Camera, Vehicle},
//!     client::Client,
//! };
//!
//! # fn example() -> carla::error::CarlaResult<()> {
//! let client = Client::new("localhost", 2000, None)?;
//! let world = client.world()?;
//!
//! // Generic actor operations
//! let actors = world.actors()?;
//! for actor in actors.iter() {
//!     println!("Actor ID: {}, Type: {:?}", actor.id(), actor.type_id());
//!
//!     // Type-safe downcasting
//!     if let Ok(vehicle) = Vehicle::from_actor(actor.clone()) {
//!         println!("  Vehicle speed: {:.1} km/h", vehicle.speed());
//!     }
//!
//!     if let Ok(camera) = Camera::from_actor(actor.clone()) {
//!         println!("  Camera type: {:?}", camera.camera_type());
//!     }
//! }
//! # Ok(())
//! # }
//! ```
//!
//! ## Actor Lifecycle
//!
//! Actors follow a managed lifecycle:
//!
//! 1. **Spawning**: Created via `World::spawn_actor()` or batch operations
//! 2. **Active**: Participating in simulation, can be controlled and queried
//! 3. **Destruction**: Explicitly destroyed via `destroy()` or automatic cleanup
//!
//! ```rust,no_run
//! use carla::{
//!     actor::Vehicle,
//!     geom::{Location, Transform},
//! };
//!
//! # fn lifecycle_example() -> carla::error::CarlaResult<()> {
//! # let world = todo!(); // Would get from client
//! # let blueprint = todo!(); // Would get from blueprint library
//!
//! // Spawn a vehicle
//! let spawn_point = Transform::new(Location::new(0.0, 0.0, 0.5), Default::default());
//! let actor = world.spawn_actor(&blueprint, &spawn_point, None)?;
//!
//! // Convert to specific type
//! let vehicle = Vehicle::from_actor(actor).map_err(|_| "Not a vehicle")?;
//!
//! // Use the vehicle
//! let speed = vehicle.speed();
//!
//! // Explicit cleanup (optional - automatic on drop)
//! vehicle.destroy()?;
//! # Ok(())
//! # }
//! ```
//!
//! ## Error Handling
//!
//! Actor operations use comprehensive error types:
//!
//! - **ActorError::NotFound**: Actor no longer exists in simulation
//! - **ActorError::ControlFailed**: Control command rejected
//! - **ActorError::LifecycleViolation**: Invalid operation for actor state
//! - **SpawnError**: Actor spawning failures
//! - **SensorError**: Sensor-specific operation failures
//!
//! ## Performance Considerations
//!
//! - **Batch Operations**: Use `World::batch_spawn_actors()` for multiple actors
//! - **Actor Queries**: `World::actors()` returns all actors - filter as needed
//! - **Sensor Data**: Use callbacks for high-frequency sensor data
//! - **Memory**: Actors are automatically cleaned up on drop
//!
//! ## Examples
//!
//! See the `/examples` directory for comprehensive usage examples:
//! - `vehicle_showcase.rs` - Vehicle control and physics
//! - `sensor_sync.rs` - Multi-sensor data collection
//! - `generate_traffic.rs` - Traffic simulation setup

/// Attribute list wrapper
pub mod attribute_list;
/// Base actor functionality and types.
pub mod base;
/// Camera sensor types and functionality.
pub mod camera;
/// Collision sensor types and functionality.
pub mod collision;
/// Dynamic Vision Sensor (DVS) types and functionality.
pub mod dvs;
/// GNSS sensor types and functionality.
pub mod gnss;
/// Actor ID types and utilities.
pub mod id;
/// IMU sensor types and functionality.
pub mod imu;
/// Lane invasion sensor types and functionality.
pub mod lane_invasion;
/// LiDAR sensor types and functionality.
pub mod lidar;
/// Actor list types and utilities.
pub mod list;
/// Obstacle detection sensor types and functionality.
pub mod obstacle_detection;
/// Radar sensor types and functionality.
pub mod radar;
/// RSS (Road Safety) sensor types and functionality.
pub mod rss;
/// Base sensor functionality and types.
pub mod sensor;
/// Traffic light types and functionality.
pub mod traffic_light;
/// Traffic sign types and functionality.
pub mod traffic_sign;
/// Actor and sensor traits for common functionality.
pub mod traits;
/// Vehicle types and control functionality.
pub mod vehicle;
/// Walker (pedestrian) types and control functionality.
pub mod walker;

// Re-export all major actor types for convenient access
pub use attribute_list::AttributeList;
pub use base::{Actor, ActorSnapshot};
pub use camera::{Camera, CameraType};
pub use collision::CollisionSensor;
pub use dvs::DVSCamera;
pub use gnss::GNSS;
pub use id::ActorId;
pub use imu::IMU;
pub use lane_invasion::LaneInvasionSensor;
pub use lidar::LiDAR;
pub use list::ActorList;
pub use obstacle_detection::ObstacleDetectionSensor;
pub use radar::Radar;
pub use rss::RSSSensor;
pub use sensor::Sensor;
pub use traffic_light::{TrafficLight, TrafficLightState};
pub use traffic_sign::TrafficSign;
pub use traits::{ActorExt, SensorExt};
pub(crate) use traits::{ActorFfi, SensorFfi};
pub use vehicle::{
    Vehicle, VehicleControl, VehicleDoorType, VehicleLightState, VehiclePhysicsControl,
    VehicleTelemetryData,
};
pub use walker::{Walker, WalkerControl};

/// Macro for implementing sensor conversion methods between Actor, Sensor, and specific sensor types.
///
/// This macro generates methods for converting between:
/// - `Actor` ↔ specific sensor type (e.g., `Camera`, `LiDAR`)
/// - `Sensor` ↔ specific sensor type
///
/// # Parameters
/// - `$sensor_type`: The specific sensor type to implement conversions for
/// - `$check_method`: The method name to check if a sensor is of this specific type
///
/// # Generated Methods
/// - `from_actor(actor)` - Try to create sensor from Actor
/// - `into_actor(self)` - Convert sensor to Actor
/// - `as_actor(&self)` - Get reference as Actor
/// - `from_sensor(sensor)` - Try to create sensor from generic Sensor
/// - `into_sensor(self)` - Convert to generic Sensor
/// - `as_sensor(&self)` - Get reference as generic Sensor
#[macro_export]
macro_rules! impl_sensor_conversions {
    ($sensor_type:ty, $check_method:ident) => {
        impl $sensor_type {
            /// Try to create this type from a generic Actor.
            /// Returns the original Actor if the conversion fails.
            pub fn from_actor(actor: $crate::actor::Actor) -> Result<Self, $crate::actor::Actor> {
                match $crate::actor::Sensor::from_actor(actor) {
                    Ok(sensor) => Self::from_sensor(sensor).map_err(|s| s.into_actor()),
                    Err(actor) => Err(actor),
                }
            }

            /// Convert this type into a generic Actor.
            pub fn into_actor(self) -> $crate::actor::Actor {
                self.0.into_actor()
            }

            /// Get a reference to this type as an Actor.
            pub fn as_actor(&self) -> &$crate::actor::Actor {
                // This is not ideal as it creates a new Actor each time,
                // but necessary due to the C++ shared pointer semantics
                // The returned reference is to a leaked box to maintain lifetime safety
                Box::leak(Box::new(self.0.as_actor()))
            }

            /// Try to create this type from a generic Sensor.
            /// Returns the original Sensor if the conversion fails.
            pub fn from_sensor(
                sensor: $crate::actor::Sensor,
            ) -> Result<Self, $crate::actor::Sensor> {
                if sensor.$check_method() {
                    Ok(Self(sensor))
                } else {
                    Err(sensor)
                }
            }

            /// Convert this type into a generic Sensor.
            pub fn into_sensor(self) -> $crate::actor::Sensor {
                self.0
            }

            /// Get a reference to this type as a Sensor.
            pub fn as_sensor(&self) -> &$crate::actor::Sensor {
                &self.0
            }
        }
    };
}
