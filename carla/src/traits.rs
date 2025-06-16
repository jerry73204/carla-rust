//! Common traits for CARLA actors and objects.
//!
//! This module provides trait-based abstractions that capture common behaviors
//! across different CARLA object types, enabling generic programming patterns.

use crate::{
    actor::{
        VehicleControl, VehicleDoorType, VehicleLightState, VehiclePhysicsControl,
        VehicleTelemetryData, WalkerControl,
    },
    error::CarlaResult,
    geom::{BoundingBox, Transform, Vector3D},
    ActorId,
};

/// Common behavior for all CARLA actors.
///
/// This trait is implemented by [`Actor`](crate::client::Actor),
/// [`Vehicle`](crate::client::Vehicle), [`Walker`](crate::client::Walker),
/// and [`Sensor`](crate::client::Sensor).
pub trait ActorT {
    /// Get the unique actor ID.
    fn id(&self) -> ActorId;

    /// Get the actor's type ID string (e.g., "vehicle.tesla.model3").
    fn type_id(&self) -> String;

    /// Get the current world transform of the actor.
    fn transform(&self) -> Transform;

    /// Set the actor's world transform.
    ///
    /// # Errors
    /// Returns [`ActorError::Transform`] if the transform cannot be applied.
    fn set_transform(&self, transform: &Transform) -> CarlaResult<()>;

    /// Get the actor's current velocity in m/s.
    fn velocity(&self) -> Vector3D;

    /// Get the actor's current angular velocity in rad/s.
    fn angular_velocity(&self) -> Vector3D;

    /// Get the actor's current acceleration in m/s².
    fn acceleration(&self) -> Vector3D;

    /// Check if this actor is still alive in the simulation.
    fn is_alive(&self) -> bool;

    /// Enable or disable actor physics simulation.
    fn set_simulate_physics(&self, enabled: bool) -> CarlaResult<()>;

    /// Add an impulse to the actor (if physics-enabled).
    fn add_impulse(&self, impulse: &Vector3D) -> CarlaResult<()>;

    /// Add a force to the actor (if physics-enabled).
    fn add_force(&self, force: &Vector3D) -> CarlaResult<()>;

    /// Add a torque to the actor (if physics-enabled).
    fn add_torque(&self, torque: &Vector3D) -> CarlaResult<()>;

    /// Get the actor's bounding box in local space.
    fn bounding_box(&self) -> BoundingBox;
}

/// Trait for sensor-specific behavior.
///
/// Implemented by all sensor types to provide unified sensor management.
pub trait SensorT: ActorT {
    /// Start listening for sensor data.
    fn start_listening(&self);

    /// Stop listening for sensor data.
    fn stop_listening(&self);

    /// Check if the sensor is currently listening.
    fn is_listening(&self) -> bool;

    /// Check if new sensor data is available.
    fn has_new_data(&self) -> bool;

    /// Get sensor-specific attribute.
    fn attribute(&self, name: &str) -> Option<String>;

    /// Enable sensor recording to file.
    fn enable_recording(&self, filename: &str) -> CarlaResult<()>;

    /// Disable sensor recording.
    fn disable_recording(&self) -> CarlaResult<()>;
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
    fn control(&self) -> VehicleControl;

    /// Enable or disable autopilot mode.
    ///
    /// # Arguments
    /// * `enabled` - Whether to enable autopilot
    /// * `traffic_manager_port` - Port of traffic manager (default: 8000)
    fn set_autopilot(&self, enabled: bool, traffic_manager_port: Option<u16>) -> CarlaResult<()>;

    /// Get vehicle physics control parameters.
    fn physics_control(&self) -> VehiclePhysicsControl;

    /// Apply new physics control parameters.
    fn apply_physics_control(&self, physics_control: &VehiclePhysicsControl) -> CarlaResult<()>;

    /// Get the current speed in km/h.
    fn speed(&self) -> f32 {
        // Convert m/s to km/h
        self.velocity().length() * 3.6
    }

    /// Enable/disable vehicle light state.
    fn set_light_state(&self, light_state: VehicleLightState) -> CarlaResult<()>;

    /// Get current vehicle light state.
    fn light_state(&self) -> VehicleLightState;

    /// Open/close vehicle door (CARLA 0.10.0 feature).
    fn set_door_state(&self, door_type: VehicleDoorType, is_open: bool) -> CarlaResult<()>;

    /// Get telemetry data from the vehicle.
    fn telemetry_data(&self) -> VehicleTelemetryData;
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
    fn control(&self) -> WalkerControl;

    /// Get walker bone names for animation control.
    fn bones(&self) -> Vec<String>;

    /// Set bone transforms for custom walker animation.
    fn set_bones(&self, bone_transforms: &[(String, Transform)]) -> CarlaResult<()>;

    /// Get the current speed in m/s.
    fn speed(&self) -> f32 {
        self.velocity().length()
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
    fn get_bounding_box(&self) -> BoundingBox;

    /// Get the object's bounding box in world coordinates.
    fn get_world_bounding_box(&self) -> BoundingBox {
        let _bbox = self.get_bounding_box();
        let _transform = if let Some(actor) = self.as_actor() {
            actor.transform()
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
