use crate::{
    actor::ActorId,
    error::{CarlaResult, SensorError},
    geom::{BoundingBox, FromCxx, ToCxx, Transform, Vector3D},
    CarlaError,
};

pub(crate) trait ActorFfi {
    /// Get reference to the inner Actor for FFI operations
    fn as_actor_ffi(&self) -> &carla_sys::ActorWrapper;
}

/// Common behavior for all CARLA actors.
///
/// This trait is implemented by [`Actor`](crate::client::Actor),
/// [`Vehicle`](crate::client::Vehicle), [`Walker`](crate::client::Walker),
/// and [`Sensor`](crate::client::Sensor).
///
/// Note: Types that implement ActorExt should either implement ActorFfi
/// or provide their own implementations of all methods.
pub trait ActorExt {
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

// Blanket implementation for types that implement ActorFfi
// Note: Sensor implements ActorExt directly without ActorFfi
impl<T> ActorExt for T
where
    T: ActorFfi,
{
    fn id(&self) -> ActorId {
        self.as_actor_ffi().get_id()
    }

    fn type_id(&self) -> String {
        self.as_actor_ffi().get_type_id()
    }

    fn transform(&self) -> Transform {
        let simple_transform = self.as_actor_ffi().get_transform();
        Transform::from_cxx(simple_transform)
    }

    fn set_transform(&self, transform: &Transform) -> CarlaResult<()> {
        let simple_transform = transform.to_cxx();
        self.as_actor_ffi().set_transform(&simple_transform);
        Ok(())
    }

    fn velocity(&self) -> Vector3D {
        let simple_velocity = carla_sys::ffi::Actor_GetVelocity(self.as_actor_ffi().get_actor());
        Vector3D::from_cxx(simple_velocity)
    }

    fn angular_velocity(&self) -> Vector3D {
        let simple_angular_velocity =
            carla_sys::ffi::Actor_GetAngularVelocity(self.as_actor_ffi().get_actor());
        Vector3D::from_cxx(simple_angular_velocity)
    }

    fn acceleration(&self) -> Vector3D {
        let simple_acceleration =
            carla_sys::ffi::Actor_GetAcceleration(self.as_actor_ffi().get_actor());
        Vector3D::from_cxx(simple_acceleration)
    }

    fn is_alive(&self) -> bool {
        self.as_actor_ffi().is_alive()
    }

    fn set_simulate_physics(&self, enabled: bool) -> CarlaResult<()> {
        carla_sys::ffi::Actor_SetSimulatePhysics(self.as_actor_ffi().get_actor(), enabled);
        Ok(())
    }

    fn add_impulse(&self, impulse: &Vector3D) -> CarlaResult<()> {
        let simple_impulse = impulse.to_cxx();
        carla_sys::ffi::Actor_AddImpulse(self.as_actor_ffi().get_actor(), &simple_impulse);
        Ok(())
    }

    fn add_force(&self, force: &Vector3D) -> CarlaResult<()> {
        let simple_force = force.to_cxx();
        carla_sys::ffi::Actor_AddForce(self.as_actor_ffi().get_actor(), &simple_force);
        Ok(())
    }

    fn add_torque(&self, torque: &Vector3D) -> CarlaResult<()> {
        let simple_torque = torque.to_cxx();
        carla_sys::ffi::Actor_AddTorque(self.as_actor_ffi().get_actor(), &simple_torque);
        Ok(())
    }

    fn bounding_box(&self) -> BoundingBox {
        let simple_bbox = carla_sys::ffi::Actor_GetBoundingBox(self.as_actor_ffi().get_actor());
        BoundingBox::from_cxx(simple_bbox)
    }
}

pub(crate) trait SensorFfi {
    /// Get reference to the inner Sensor for FFI operations
    fn as_sensor_ffi(&self) -> &carla_sys::SensorWrapper;
}

/// Trait for sensor-specific behavior.
///
/// Implemented by all sensor types to provide unified sensor management.
pub trait SensorExt: SensorFfi {
    /// Start listening for sensor data.
    fn start_listening(&self) {
        self.as_sensor_ffi().listen();
    }

    /// Stop listening for sensor data.
    fn stop_listening(&self) {
        self.as_sensor_ffi().stop();
    }

    /// Check if the sensor is currently listening.
    fn is_listening(&self) -> bool {
        self.as_sensor_ffi().is_listening()
    }

    /// Check if new sensor data is available.
    fn has_new_data(&self) -> bool {
        self.as_sensor_ffi().has_new_data()
    }

    /// Get sensor-specific attribute.
    fn attribute(&self, name: &str) -> Option<String> {
        let _name = name;
        // TODO: Implement sensor attribute retrieval
        // This requires adding Sensor_GetAttribute FFI function or storing blueprint reference
        todo!(
            "Sensor::get_attribute not yet implemented - missing FFI function Sensor_GetAttribute"
        )
    }

    /// Enable sensor recording to file.
    fn enable_recording(&self, filename: &str) -> CarlaResult<()> {
        let _filename = filename;
        // Sensor recording is typically done at the client level, not sensor level
        // This would require additional FFI functions that aren't part of the basic sensor API
        Err(CarlaError::Sensor(SensorError::CallbackFailed(
            "enable_recording not available - use client-level recording".to_string(),
        )))
    }

    /// Disable sensor recording.
    fn disable_recording(&self) -> CarlaResult<()> {
        // Sensor recording is typically done at the client level, not sensor level
        // This would require additional FFI functions that aren't part of the basic sensor API
        Err(CarlaError::Sensor(SensorError::CallbackFailed(
            "disable_recording not available - use client-level recording".to_string(),
        )))
    }
}

impl<T> SensorExt for T where T: SensorFfi {}

// /// Trait for vehicle-specific behavior.
// ///
// /// Implemented by [`Vehicle`](crate::client::Vehicle) to provide vehicle control.
// pub trait VehicleExt: ActorExt {
//     /// Apply vehicle control input.
//     ///
//     /// # Arguments
//     /// * `control` - Vehicle control commands (throttle, brake, steer, etc.)
//     ///
//     /// # Errors
//     /// Returns [`ActorError::ControlFailed`] if control cannot be applied.
//     fn apply_control(&self, control: &VehicleControl) -> CarlaResult<()>;

//     /// Get the current vehicle control state.
//     fn control(&self) -> VehicleControl;

//     /// Enable or disable autopilot mode.
//     ///
//     /// # Arguments
//     /// * `enabled` - Whether to enable autopilot
//     /// * `traffic_manager_port` - Port of traffic manager (default: 8000)
//     fn set_autopilot(&self, enabled: bool, traffic_manager_port: Option<u16>) -> CarlaResult<()>;

//     /// Get vehicle physics control parameters.
//     fn physics_control(&self) -> VehiclePhysicsControl;

//     /// Apply new physics control parameters.
//     fn apply_physics_control(&self, physics_control: &VehiclePhysicsControl) -> CarlaResult<()>;

//     /// Get the current speed in km/h.
//     fn speed(&self) -> f32 {
//         // Convert m/s to km/h
//         self.velocity().length() * 3.6
//     }

//     /// Enable/disable vehicle light state.
//     fn set_light_state(&self, light_state: VehicleLightState) -> CarlaResult<()>;

//     /// Get current vehicle light state.
//     fn light_state(&self) -> VehicleLightState;

//     /// Open/close vehicle door (CARLA 0.10.0 feature).
//     fn set_door_state(&self, door_type: VehicleDoorType, is_open: bool) -> CarlaResult<()>;

//     /// Get telemetry data from the vehicle.
//     fn telemetry_data(&self) -> VehicleTelemetryData;
// }

// /// Trait for walker (pedestrian) specific behavior.
// ///
// /// Implemented by [`Walker`](crate::client::Walker) for pedestrian simulation.
// pub trait WalkerExt: ActorExt {
//     /// Apply walker control input.
//     ///
//     /// # Arguments
//     /// * `control` - Walker control commands (direction, speed, jump)
//     fn apply_control(&self, control: &WalkerControl) -> CarlaResult<()>;

//     /// Get the current walker control state.
//     fn control(&self) -> WalkerControl;

//     /// Get walker bone names for animation control.
//     fn bones(&self) -> Vec<String>;

//     /// Set bone transforms for custom walker animation.
//     fn set_bones(&self, bone_transforms: &[(String, Transform)]) -> CarlaResult<()>;

//     /// Get the current speed in m/s.
//     fn speed(&self) -> f32 {
//         self.velocity().length()
//     }
// }
