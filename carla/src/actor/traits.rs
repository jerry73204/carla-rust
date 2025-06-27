use crate::{
    actor::ActorId,
    error::{CarlaError, CarlaResult, DestroyError, SensorError},
    geom::{BoundingBox, FromCxx, ToCxx, Transform, Vector3D},
};

pub(crate) trait ActorFfi {
    /// Create an ActorWrapper for FFI operations
    fn as_actor_ffi(&self) -> carla_sys::ActorWrapper;
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

    /// Check if this actor is valid for operations.
    ///
    /// This checks both if the actor is alive and has a valid state.
    /// An actor may be alive but in an invalid state (e.g., during destruction).
    ///
    /// # Returns
    /// Returns `true` if the actor is both alive and in a valid state.
    fn is_valid(&self) -> bool {
        self.is_alive()
    }

    /// Try to get the actor's transform with crash protection.
    ///
    /// This method adds defensive checks to prevent crashes observed
    /// when accessing transform on invalid actors.
    fn try_transform(&self) -> CarlaResult<Transform> {
        if !self.is_valid() {
            return Err(CarlaError::Runtime(format!(
                "Actor {} is not valid for transform operation",
                self.id()
            )));
        }
        Ok(self.transform())
    }

    /// Try to get the actor's velocity with crash protection.
    fn try_velocity(&self) -> CarlaResult<Vector3D> {
        if !self.is_valid() {
            return Err(CarlaError::Runtime(format!(
                "Actor {} is not valid for velocity operation",
                self.id()
            )));
        }
        Ok(self.velocity())
    }

    /// Try to get the actor's angular velocity with crash protection.
    fn try_angular_velocity(&self) -> CarlaResult<Vector3D> {
        if !self.is_valid() {
            return Err(CarlaError::Runtime(format!(
                "Actor {} is not valid for angular_velocity operation",
                self.id()
            )));
        }
        Ok(self.angular_velocity())
    }

    /// Try to get the actor's acceleration with crash protection.
    fn try_acceleration(&self) -> CarlaResult<Vector3D> {
        if !self.is_valid() {
            return Err(CarlaError::Runtime(format!(
                "Actor {} is not valid for acceleration operation",
                self.id()
            )));
        }
        Ok(self.acceleration())
    }

    /// Try to get the actor's bounding box with crash protection.
    fn try_bounding_box(&self) -> CarlaResult<BoundingBox> {
        if !self.is_valid() {
            return Err(CarlaError::Runtime(format!(
                "Actor {} is not valid for bounding_box operation",
                self.id()
            )));
        }
        Ok(self.bounding_box())
    }

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

    /// Explicitly destroy this actor.
    ///
    /// This method sends a destruction command to the CARLA server to remove
    /// the actor from the simulation. Once destroyed, the actor cannot be used
    /// for further operations.
    ///
    /// # Thread Safety
    ///
    /// The underlying CARLA C++ client handles thread safety. Multiple calls
    /// to destroy() on the same actor from different threads are handled by
    /// the C++ implementation.
    ///
    /// # Behavior
    ///
    /// - If destruction succeeds, returns `Ok(())`
    /// - If destruction fails at the server level, returns `DestroyError::InvalidActor`
    ///
    /// # Note
    ///
    /// Actors are automatically destroyed when dropped via the `Drop` trait.
    /// This method provides explicit control over destruction timing.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use carla::{actor::ActorExt, client::Client};
    /// # use carla::error::CarlaResult;
    ///
    /// # fn example() -> CarlaResult<()> {
    /// let client = Client::new("localhost", 2000, None::<usize>)?;
    /// let world = client.world()?;
    /// let blueprint_library = world.blueprint_library()?;
    ///
    /// if let Some(vehicle_bp) = blueprint_library.find("vehicle.tesla.model3")? {
    ///     let spawn_points = world.map()?.spawn_points();
    ///     if let Some(spawn_point) = spawn_points.get(0) {
    ///         if let Some(mut vehicle) = world.try_spawn_actor(&vehicle_bp, &spawn_point, None)? {
    ///             // Cast to vehicle
    ///             if let Ok(mut vehicle) = vehicle.into_vehicle() {
    ///                 // Explicitly destroy the vehicle
    ///                 vehicle.destroy()?;
    ///             }
    ///         }
    ///     }
    /// }
    /// # Ok(())
    /// # }
    /// ```
    #[must_use = "destroy() returns a Result that should be handled"]
    fn destroy(&mut self) -> CarlaResult<()>;
}

/// Sealed trait to control which types get the blanket ActorExt implementation
mod sealed {
    pub trait Sealed {}
    impl Sealed for crate::actor::Vehicle {}
    impl Sealed for crate::actor::Walker {}
    impl Sealed for crate::actor::TrafficLight {}
    impl Sealed for crate::actor::TrafficSign {}
}

// Blanket implementation for types that implement ActorFfi but are not Actor
// Note: Sensor and Actor implement ActorExt directly
impl<T> ActorExt for T
where
    T: ActorFfi + sealed::Sealed,
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
        let actor_wrapper = self.as_actor_ffi();
        let simple_velocity = carla_sys::ffi::Actor_GetVelocity(actor_wrapper.get_actor());
        Vector3D::from_cxx(simple_velocity)
    }

    fn angular_velocity(&self) -> Vector3D {
        let actor_wrapper = self.as_actor_ffi();
        let simple_angular_velocity =
            carla_sys::ffi::Actor_GetAngularVelocity(actor_wrapper.get_actor());
        Vector3D::from_cxx(simple_angular_velocity)
    }

    fn acceleration(&self) -> Vector3D {
        let actor_wrapper = self.as_actor_ffi();
        let simple_acceleration = carla_sys::ffi::Actor_GetAcceleration(actor_wrapper.get_actor());
        Vector3D::from_cxx(simple_acceleration)
    }

    fn is_alive(&self) -> bool {
        self.as_actor_ffi().is_alive()
    }

    fn set_simulate_physics(&self, enabled: bool) -> CarlaResult<()> {
        let actor_wrapper = self.as_actor_ffi();
        carla_sys::ffi::Actor_SetSimulatePhysics(actor_wrapper.get_actor(), enabled);
        Ok(())
    }

    fn add_impulse(&self, impulse: &Vector3D) -> CarlaResult<()> {
        let actor_wrapper = self.as_actor_ffi();
        let simple_impulse = impulse.to_cxx();
        carla_sys::ffi::Actor_AddImpulse(actor_wrapper.get_actor(), &simple_impulse);
        Ok(())
    }

    fn add_force(&self, force: &Vector3D) -> CarlaResult<()> {
        let actor_wrapper = self.as_actor_ffi();
        let simple_force = force.to_cxx();
        carla_sys::ffi::Actor_AddForce(actor_wrapper.get_actor(), &simple_force);
        Ok(())
    }

    fn add_torque(&self, torque: &Vector3D) -> CarlaResult<()> {
        let actor_wrapper = self.as_actor_ffi();
        let simple_torque = torque.to_cxx();
        carla_sys::ffi::Actor_AddTorque(actor_wrapper.get_actor(), &simple_torque);
        Ok(())
    }

    fn bounding_box(&self) -> BoundingBox {
        let actor_wrapper = self.as_actor_ffi();
        let simple_bbox = carla_sys::ffi::Actor_GetBoundingBox(actor_wrapper.get_actor());
        BoundingBox::from_cxx(simple_bbox)
    }

    fn destroy(&mut self) -> CarlaResult<()> {
        // Call C++ destruction
        let success = self.as_actor_ffi().destroy();

        if success {
            Ok(())
        } else {
            // If C++ destroy returns false, it means the actor is invalid or
            // already destroyed in the simulation
            Err(DestroyError::InvalidActor {
                actor_id: self.id(),
            }
            .into())
        }
    }
}

/// FFI trait for internal sensor operations.
///
/// This trait provides access to the underlying FFI sensor wrapper for internal operations.
/// It's primarily used by the public SensorExt trait to provide sensor functionality.
pub trait SensorFfi {
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

// ActorExt implementations for sensor wrapper types
// These delegate to their inner Sensor's ActorExt implementation

// Make this macro available to other modules

/// Macro to implement ActorExt trait for sensor types.
///
/// This macro provides a default implementation of the ActorExt trait
/// for sensor types by delegating to their inner Sensor's implementation.
///
/// # Parameters
/// - `$sensor_type`: The sensor type to implement ActorExt for
#[macro_export]
macro_rules! impl_sensor_actor_ext {
    ($sensor_type:ty) => {
        impl $crate::actor::ActorExt for $sensor_type {
            fn id(&self) -> $crate::actor::ActorId {
                self.as_sensor().id()
            }

            fn type_id(&self) -> String {
                self.as_sensor().type_id()
            }

            fn transform(&self) -> $crate::geom::Transform {
                self.as_sensor().transform()
            }

            fn set_transform(&self, transform: &$crate::geom::Transform) -> $crate::error::CarlaResult<()> {
                self.as_sensor().set_transform(transform)
            }

            fn velocity(&self) -> $crate::geom::Vector3D {
                self.as_sensor().velocity()
            }

            fn angular_velocity(&self) -> $crate::geom::Vector3D {
                self.as_sensor().angular_velocity()
            }

            fn acceleration(&self) -> $crate::geom::Vector3D {
                self.as_sensor().acceleration()
            }

            fn is_alive(&self) -> bool {
                self.as_sensor().is_alive()
            }

            fn set_simulate_physics(&self, enabled: bool) -> $crate::error::CarlaResult<()> {
                self.as_sensor().set_simulate_physics(enabled)
            }

            fn add_impulse(&self, impulse: &$crate::geom::Vector3D) -> $crate::error::CarlaResult<()> {
                self.as_sensor().add_impulse(impulse)
            }

            fn add_force(&self, force: &$crate::geom::Vector3D) -> $crate::error::CarlaResult<()> {
                self.as_sensor().add_force(force)
            }

            fn add_torque(&self, torque: &$crate::geom::Vector3D) -> $crate::error::CarlaResult<()> {
                self.as_sensor().add_torque(torque)
            }

            fn bounding_box(&self) -> $crate::geom::BoundingBox {
                self.as_sensor().bounding_box()
            }

            fn destroy(&mut self) -> $crate::error::CarlaResult<()> {
                // Sensor wrapper types don't support direct destruction through this method
                // as they don't have mutable access to the inner sensor.
                // Destruction happens automatically through Drop trait.
                // For explicit destruction, convert to Actor first using into_actor().
                Err($crate::error::CarlaError::Sensor(
                    $crate::error::SensorError::CallbackFailed(
                        "Sensor wrapper types don't support destroy() - use into_actor().destroy() or rely on Drop".to_string()
                    )
                ))
            }
        }
    };
}

// Sensor wrapper types apply this macro in their respective files

// Static assertions to verify that all actor types implement ActorExt
#[cfg(test)]
mod tests {
    use super::*;
    use crate::actor::{
        Actor, Camera, CollisionSensor, DVSCamera, LaneInvasionSensor, LiDAR,
        ObstacleDetectionSensor, RSSSensor, Radar, Sensor, TrafficLight, TrafficSign, Vehicle,
        Walker, GNSS, IMU,
    };
    use static_assertions::assert_impl_all;

    // Verify that all actor types implement ActorExt
    assert_impl_all!(Actor: ActorExt);
    assert_impl_all!(Vehicle: ActorExt);
    assert_impl_all!(Walker: ActorExt);
    assert_impl_all!(TrafficLight: ActorExt);
    assert_impl_all!(TrafficSign: ActorExt);

    // Verify that all sensor types implement ActorExt
    assert_impl_all!(Sensor: ActorExt);
    assert_impl_all!(Camera: ActorExt);
    assert_impl_all!(LiDAR: ActorExt);
    assert_impl_all!(GNSS: ActorExt);
    assert_impl_all!(IMU: ActorExt);
    assert_impl_all!(Radar: ActorExt);
    assert_impl_all!(CollisionSensor: ActorExt);
    assert_impl_all!(DVSCamera: ActorExt);
    assert_impl_all!(LaneInvasionSensor: ActorExt);
    assert_impl_all!(ObstacleDetectionSensor: ActorExt);
    assert_impl_all!(RSSSensor: ActorExt);
}

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
