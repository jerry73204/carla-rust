//! Base actor functionality.

use crate::{
    client::{ActorId, Sensor, Vehicle, Walker},
    error::CarlaResult,
    geom::{FromCxx, ToCxx, Transform, Vector3D},
    traits::ActorT,
};

/// Generic actor in the simulation.
#[derive(Debug)]
pub struct Actor {
    /// Actor ID
    pub id: ActorId,
    /// Internal handle to carla-cxx Actor
    inner: carla_cxx::ActorWrapper,
}

impl Actor {
    /// Create a new Actor from a carla-cxx ActorWrapper
    pub fn new(inner: carla_cxx::ActorWrapper) -> Self {
        let id = inner.get_id();
        Self { id, inner }
    }

    /// Get reference to the inner ActorWrapper for FFI operations
    pub fn get_actor_wrapper(&self) -> &carla_cxx::ActorWrapper {
        &self.inner
    }

    /// Get reference to the inner Actor for FFI operations
    pub fn get_inner_actor(&self) -> &carla_cxx::Actor {
        self.inner.get_actor()
    }

    /// Try to cast this actor to a vehicle.
    pub fn as_vehicle(&self) -> Option<Vehicle> {
        if let Some(_vehicle_wrapper) =
            carla_cxx::VehicleWrapper::from_actor(self.inner.get_actor())
        {
            // TODO: Need Vehicle to Actor conversion in carla-cxx FFI - Vehicle_CastToActor() missing
            todo!("Actor::as_vehicle - need Vehicle to Actor conversion FFI function")
        } else {
            None
        }
    }

    /// Try to cast this actor to a walker.
    pub fn as_walker(&self) -> Option<Walker> {
        if let Some(_walker_wrapper) = carla_cxx::WalkerWrapper::from_actor(self.inner.get_actor())
        {
            // TODO: Need Walker to Actor conversion in carla-cxx FFI - Walker_CastToActor() missing
            todo!("Actor::as_walker - need Walker to Actor conversion FFI function")
        } else {
            None
        }
    }

    /// Try to cast this actor to a sensor.
    pub fn as_sensor(&self) -> Option<Sensor> {
        if let Some(_sensor_wrapper) = carla_cxx::SensorWrapper::from_actor(self.inner.get_actor())
        {
            // TODO: Need Sensor to Actor conversion in carla-cxx FFI - Sensor_CastToActor() missing
            todo!("Actor::as_sensor - need Sensor to Actor conversion FFI function")
        } else {
            None
        }
    }

    /// Get the actor's bounding box.
    pub fn get_bounding_box(&self) -> crate::geom::BoundingBox {
        // TODO: Implement using carla-cxx FFI interface
        todo!("Actor::get_bounding_box not yet implemented with carla-cxx FFI")
    }

    /// Get the actor's world bounding box.
    pub fn get_world_bounding_box(&self) -> crate::geom::BoundingBox {
        // TODO: Implement using carla-cxx FFI interface
        todo!("Actor::get_world_bounding_box not yet implemented with carla-cxx FFI")
    }

    /// Get semantic tags for this actor.
    pub fn get_semantic_tags(&self) -> Vec<u8> {
        // TODO: Implement using carla-cxx FFI interface
        todo!("Actor::get_semantic_tags not yet implemented with carla-cxx FFI")
    }

    /// Check if actor is a specific type.
    pub fn is_type(&self, type_pattern: &str) -> bool {
        let type_id = self.inner.get_type_id();
        type_id.contains(type_pattern)
    }
}

impl ActorT for Actor {
    fn get_id(&self) -> ActorId {
        self.inner.get_id()
    }

    fn get_type_id(&self) -> String {
        self.inner.get_type_id()
    }

    fn get_transform(&self) -> Transform {
        let simple_transform = self.inner.get_transform();
        Transform::from_cxx(simple_transform)
    }

    fn set_transform(&self, transform: &Transform) -> CarlaResult<()> {
        let simple_transform = transform.to_cxx();
        self.inner.set_transform(&simple_transform);
        Ok(())
    }

    fn get_velocity(&self) -> Vector3D {
        // TODO: Implement using carla-cxx FFI interface
        todo!("Actor::get_velocity not yet implemented with carla-cxx FFI")
    }

    fn get_angular_velocity(&self) -> Vector3D {
        // TODO: Implement using carla-cxx FFI interface
        todo!("Actor::get_angular_velocity not yet implemented with carla-cxx FFI")
    }

    fn get_acceleration(&self) -> Vector3D {
        // TODO: Implement using carla-cxx FFI interface
        todo!("Actor::get_acceleration not yet implemented with carla-cxx FFI")
    }

    fn is_alive(&self) -> bool {
        self.inner.is_alive()
    }

    fn destroy(&self) -> CarlaResult<()> {
        let success = self.inner.destroy();
        if success {
            Ok(())
        } else {
            Err(
                crate::error::ActorError::OperationFailed("Failed to destroy actor".to_string())
                    .into(),
            )
        }
    }

    fn set_simulate_physics(&self, enabled: bool) -> CarlaResult<()> {
        // TODO: Implement using carla-cxx FFI interface
        let _enabled = enabled;
        todo!("Actor::set_simulate_physics not yet implemented with carla-cxx FFI")
    }

    fn add_impulse(&self, impulse: &Vector3D) -> CarlaResult<()> {
        // TODO: Implement using carla-cxx FFI interface
        let _impulse = impulse;
        todo!("Actor::add_impulse not yet implemented with carla-cxx FFI")
    }

    fn add_force(&self, force: &Vector3D) -> CarlaResult<()> {
        // TODO: Implement using carla-cxx FFI interface
        let _force = force;
        todo!("Actor::add_force not yet implemented with carla-cxx FFI")
    }

    fn add_torque(&self, torque: &Vector3D) -> CarlaResult<()> {
        // TODO: Implement using carla-cxx FFI interface
        let _torque = torque;
        todo!("Actor::add_torque not yet implemented with carla-cxx FFI")
    }
}

/// Snapshot of an actor's state.
#[derive(Debug, Clone)]
pub struct ActorSnapshot {
    /// Actor ID
    pub id: ActorId,
    /// Actor transform
    pub transform: Transform,
    /// Actor velocity
    pub velocity: Vector3D,
    /// Actor angular velocity
    pub angular_velocity: Vector3D,
    /// Actor acceleration
    pub acceleration: Vector3D,
}
