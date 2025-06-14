//! Common traits for CARLA actors and objects.
//!
//! This module provides trait-based abstractions that capture common behaviors
//! across different CARLA object types, enabling generic programming patterns.

use crate::{
    error::{ActorError, CarlaResult, SensorError},
    geom::{Transform, Vector3D},
    rpc::{VehicleControl, WalkerControl},
    sensor::SensorData,
    ActorId,
};

/// Common behavior for all CARLA actors.
///
/// This trait is implemented by [`Actor`](crate::client::Actor),
/// [`Vehicle`](crate::client::Vehicle), [`Walker`](crate::client::Walker),
/// and [`Sensor`](crate::client::Sensor).
pub trait ActorT {
    /// Get the unique actor ID.
    fn get_id(&self) -> ActorId;

    /// Get the actor's type ID string (e.g., "vehicle.tesla.model3").
    fn get_type_id(&self) -> String;

    /// Get the current world transform of the actor.
    fn get_transform(&self) -> Transform;

    /// Set the actor's world transform.
    ///
    /// # Errors
    /// Returns [`ActorError::Transform`] if the transform cannot be applied.
    fn set_transform(&self, transform: &Transform) -> CarlaResult<()>;

    /// Get the actor's current velocity in m/s.
    fn get_velocity(&self) -> Vector3D;

    /// Get the actor's current angular velocity in rad/s.
    fn get_angular_velocity(&self) -> Vector3D;

    /// Get the actor's current acceleration in m/s².
    fn get_acceleration(&self) -> Vector3D;

    /// Check if this actor is still alive in the simulation.
    fn is_alive(&self) -> bool;

    /// Destroy this actor, removing it from the simulation.
    ///
    /// # Errors
    /// Returns [`ActorError::Destroyed`] if already destroyed.
    fn destroy(&self) -> CarlaResult<()>;

    /// Enable or disable actor physics simulation.
    fn set_simulate_physics(&self, enabled: bool) -> CarlaResult<()>;

    /// Add an impulse to the actor (if physics-enabled).
    fn add_impulse(&self, impulse: &Vector3D) -> CarlaResult<()>;

    /// Add a force to the actor (if physics-enabled).
    fn add_force(&self, force: &Vector3D) -> CarlaResult<()>;

    /// Add a torque to the actor (if physics-enabled).
    fn add_torque(&self, torque: &Vector3D) -> CarlaResult<()>;
}

/// Trait for sensor-specific behavior.
///
/// Implemented by all sensor types to provide unified sensor management.
pub trait SensorT: ActorT {
    /// The type of data this sensor produces.
    type DataType: SensorData;

    /// Start listening for sensor data with a callback.
    ///
    /// # Arguments
    /// * `callback` - Function called when new data is available
    ///
    /// # Errors
    /// Returns [`SensorError::AlreadyListening`] if already listening,
    /// or [`SensorError::CallbackFailed`] if callback registration fails.
    fn listen<F>(&self, callback: F) -> Result<(), SensorError>
    where
        F: Fn(Self::DataType) + Send + 'static;

    /// Start listening with an async channel for sensor data.
    ///
    /// Returns a receiver that yields sensor data. Requires the `async` feature.
    #[cfg(feature = "async")]
    fn listen_async(&self) -> Result<tokio::sync::mpsc::Receiver<Self::DataType>, SensorError>;

    /// Stop listening for sensor data.
    ///
    /// # Errors
    /// Returns [`SensorError::NotListening`] if not currently listening.
    fn stop(&self) -> Result<(), SensorError>;

    /// Check if the sensor is currently listening.
    fn is_listening(&self) -> bool;

    /// Get the sensor's data frame number (increments with each measurement).
    fn get_frame(&self) -> u64;

    /// Get the timestamp of the last sensor measurement.
    fn get_timestamp(&self) -> f64;
}

/// Trait for vehicle-specific behavior.
///
/// Implemented by [`Vehicle`](crate::client::Vehicle) to provide vehicle control.
pub trait VehicleT: ActorT {
    /// Apply vehicle control input.
    ///
    /// # Arguments
    /// * `control` - Vehicle control commands (throttle, brake, steer, etc.)
    ///
    /// # Errors
    /// Returns [`ActorError::ControlFailed`] if control cannot be applied.
    fn apply_control(&self, control: &VehicleControl) -> CarlaResult<()>;

    /// Get the current vehicle control state.
    fn get_control(&self) -> VehicleControl;

    /// Enable or disable autopilot mode.
    ///
    /// # Arguments
    /// * `enabled` - Whether to enable autopilot
    /// * `traffic_manager_port` - Port of traffic manager (default: 8000)
    fn set_autopilot(&self, enabled: bool, traffic_manager_port: Option<u16>) -> CarlaResult<()>;

    /// Get vehicle physics control parameters.
    fn get_physics_control(&self) -> crate::rpc::VehiclePhysicsControl;

    /// Apply new physics control parameters.
    fn apply_physics_control(
        &self,
        physics_control: &crate::rpc::VehiclePhysicsControl,
    ) -> CarlaResult<()>;

    /// Get the current speed in km/h.
    fn get_speed(&self) -> f32 {
        // Convert m/s to km/h
        self.get_velocity().length() * 3.6
    }

    /// Enable/disable vehicle light state.
    fn set_light_state(&self, light_state: crate::rpc::VehicleLightState) -> CarlaResult<()>;

    /// Get current vehicle light state.
    fn get_light_state(&self) -> crate::rpc::VehicleLightState;

    /// Open/close vehicle door (CARLA 0.10.0 feature).
    fn set_door_state(
        &self,
        door_type: crate::rpc::VehicleDoorType,
        is_open: bool,
    ) -> CarlaResult<()>;

    /// Get telemetry data from the vehicle.
    fn get_telemetry_data(&self) -> crate::rpc::VehicleTelemetryData;
}

/// Trait for walker (pedestrian) specific behavior.
///
/// Implemented by [`Walker`](crate::client::Walker) for pedestrian simulation.
pub trait WalkerT: ActorT {
    /// Apply walker control input.
    ///
    /// # Arguments
    /// * `control` - Walker control commands (direction, speed, jump)
    fn apply_control(&self, control: &WalkerControl) -> CarlaResult<()>;

    /// Get the current walker control state.
    fn get_control(&self) -> WalkerControl;

    /// Get walker bone names for animation control.
    fn get_bones(&self) -> Vec<String>;

    /// Set bone transforms for custom walker animation.
    fn set_bones(&self, bone_transforms: &[(String, Transform)]) -> CarlaResult<()>;

    /// Get the current speed in m/s.
    fn get_speed(&self) -> f32 {
        self.get_velocity().length()
    }
}

/// Trait for objects that can be collected into lists.
///
/// Provides iteration and filtering capabilities for actor collections.
pub trait CollectionT<T> {
    /// Get the number of items in the collection.
    fn len(&self) -> usize;

    /// Check if the collection is empty.
    fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Get an item by index.
    fn get(&self, index: usize) -> Option<&T>;

    /// Find an item by predicate.
    fn find<P>(&self, predicate: P) -> Option<&T>
    where
        P: Fn(&T) -> bool;

    /// Filter items by predicate.
    fn filter<P>(&self, predicate: P) -> Vec<&T>
    where
        P: Fn(&T) -> bool;

    /// Convert to a vector.
    fn to_vec(&self) -> Vec<T>
    where
        T: Clone;
}

/// Marker trait for types that can be spawned in the world.
pub trait SpawnableT {
    /// The type returned after spawning.
    type SpawnedType;

    /// Get the blueprint ID for spawning.
    fn blueprint_id(&self) -> &str;
}

/// Trait for objects that have a bounding box.
pub trait BoundingBoxT {
    /// Get the object's bounding box in local coordinates.
    fn get_bounding_box(&self) -> crate::geom::BoundingBox;

    /// Get the object's bounding box in world coordinates.
    fn get_world_bounding_box(&self) -> crate::geom::BoundingBox {
        let _bbox = self.get_bounding_box();
        let _transform = if let Some(actor) = self.as_actor() {
            actor.get_transform()
        } else {
            Transform::default()
        };

        // Transform bounding box to world coordinates
        todo!("Implement bounding box transformation")
    }

    /// Attempt to cast to an actor (for transform access).
    fn as_actor(&self) -> Option<&dyn ActorT> {
        None
    }
}
