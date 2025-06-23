//! Base actor functionality.

use super::{ActorId, Sensor, TrafficLight, Vehicle, Walker};
use crate::{
    actor::ActorFfi,
    geom::{FromCxx, ToCxx, Transform, Vector3D},
};

/// Generic actor in the simulation.
#[derive(Debug)]
pub struct Actor {
    /// Internal handle to carla-sys Actor
    inner: carla_sys::ActorWrapper,
    /// Flag to track if this actor has been explicitly destroyed
    destroyed: std::cell::Cell<bool>,
}

impl Actor {
    /// Create a new Actor from a carla-sys ActorWrapper
    pub fn from_cxx(inner: carla_sys::ActorWrapper) -> Self {
        Self {
            inner,
            destroyed: std::cell::Cell::new(false),
        }
    }

    /// Get reference to the inner Actor for FFI operations
    pub(crate) fn inner_actor(&self) -> &carla_sys::Actor {
        self.inner.get_actor()
    }

    /// Try to convert this actor to a vehicle, consuming self.
    /// Returns the Vehicle if successful, or the original Actor if not.
    pub fn into_vehicle(self) -> Result<Vehicle, Actor> {
        if let Some(vehicle_wrapper) = carla_sys::VehicleWrapper::from_actor(self.inner.get_actor())
        {
            match Vehicle::from_cxx(vehicle_wrapper) {
                Ok(vehicle) => Ok(vehicle),
                Err(_) => Err(self),
            }
        } else {
            Err(self)
        }
    }

    /// Try to convert this actor to a walker, consuming self.
    /// Returns the Walker if successful, or the original Actor if not.
    pub fn into_walker(self) -> Result<Walker, Actor> {
        if let Some(walker_wrapper) = carla_sys::WalkerWrapper::from_actor(self.inner.get_actor()) {
            match Walker::from_cxx(walker_wrapper) {
                Ok(walker) => Ok(walker),
                Err(_) => Err(self),
            }
        } else {
            Err(self)
        }
    }

    /// Try to convert this actor to a sensor, consuming self.
    /// Returns the Sensor if successful, or the original Actor if not.
    pub fn into_sensor(self) -> Result<Sensor, Actor> {
        if let Some(sensor_wrapper) = carla_sys::SensorWrapper::from_actor(self.inner.get_actor()) {
            match Sensor::from_cxx(sensor_wrapper) {
                Ok(sensor) => Ok(sensor),
                Err(_) => Err(self),
            }
        } else {
            Err(self)
        }
    }

    /// Try to convert this actor to a traffic light, consuming self.
    /// Returns the TrafficLight if successful, or the original Actor if not.
    pub fn into_traffic_light(self) -> Result<TrafficLight, Actor> {
        if let Some(traffic_light_wrapper) =
            carla_sys::TrafficLightWrapper::from_actor(self.inner.get_actor())
        {
            match TrafficLight::from_cxx(traffic_light_wrapper) {
                Ok(traffic_light) => Ok(traffic_light),
                Err(_) => Err(self),
            }
        } else {
            Err(self)
        }
    }

    // TODO: Return a AttributeList type wrapping a C++ std::vector<ActorAttributeValue>.
    /// Get actor attributes.
    pub fn attributes(&self) -> Vec<String> {
        carla_sys::ffi::Actor_GetAttributes(self.inner.get_actor())
    }

    /// Check if the actor is dormant.
    pub fn is_dormant(&self) -> bool {
        carla_sys::ffi::Actor_IsDormant(self.inner.get_actor())
    }

    /// Get the parent actor ID if this actor has a parent.
    pub fn parent_id(&self) -> Option<ActorId> {
        let parent_id = carla_sys::ffi::Actor_GetParentId(self.inner.get_actor());
        if parent_id.id == 0 {
            None
        } else {
            Some(parent_id.id)
        }
    }

    /// Get semantic tags for this actor.
    pub fn semantic_tags(&self) -> Vec<u8> {
        carla_sys::ffi::Actor_GetSemanticTags(self.inner.get_actor())
    }

    /// Get the current actor state.
    pub fn actor_state(&self) -> ActorState {
        let state = carla_sys::ffi::Actor_GetActorState(self.inner.get_actor());
        match state {
            0 => ActorState::Invalid,
            1 => ActorState::Active,
            2 => ActorState::Dormant,
            _ => ActorState::Invalid, // Default for unknown values
        }
    }

    /// Get the actor ID for this actor.
    ///
    /// This queries the underlying C++ actor for its unique ID.
    pub fn id(&self) -> ActorId {
        self.inner.get_id()
    }
}

impl ActorFfi for Actor {
    fn as_actor_ffi(&self) -> carla_sys::ActorWrapper {
        // Actor already contains ActorWrapper, so clone it
        // This is efficient since ActorWrapper contains SharedPtr which is cheap to clone
        self.inner.clone()
    }
}

impl super::ActorExt for Actor {
    fn id(&self) -> super::ActorId {
        self.inner.get_id()
    }

    fn type_id(&self) -> String {
        self.inner.get_type_id()
    }

    fn transform(&self) -> crate::geom::Transform {
        let simple_transform = self.inner.get_transform();
        crate::geom::Transform::from_cxx(simple_transform)
    }

    fn set_transform(&self, transform: &crate::geom::Transform) -> crate::error::CarlaResult<()> {
        let simple_transform = transform.to_cxx();
        self.inner.set_transform(&simple_transform);
        Ok(())
    }

    fn velocity(&self) -> Vector3D {
        let simple_velocity = carla_sys::ffi::Actor_GetVelocity(self.inner.get_actor());
        Vector3D::from_cxx(simple_velocity)
    }

    fn angular_velocity(&self) -> Vector3D {
        let simple_angular_velocity =
            carla_sys::ffi::Actor_GetAngularVelocity(self.inner.get_actor());
        Vector3D::from_cxx(simple_angular_velocity)
    }

    fn acceleration(&self) -> Vector3D {
        let simple_acceleration = carla_sys::ffi::Actor_GetAcceleration(self.inner.get_actor());
        Vector3D::from_cxx(simple_acceleration)
    }

    fn is_alive(&self) -> bool {
        !self.destroyed.get() && self.inner.is_alive()
    }

    fn set_simulate_physics(&self, enabled: bool) -> crate::error::CarlaResult<()> {
        carla_sys::ffi::Actor_SetSimulatePhysics(self.inner.get_actor(), enabled);
        Ok(())
    }

    fn add_impulse(&self, impulse: &Vector3D) -> crate::error::CarlaResult<()> {
        let simple_impulse = impulse.to_cxx();
        carla_sys::ffi::Actor_AddImpulse(self.inner.get_actor(), &simple_impulse);
        Ok(())
    }

    fn add_force(&self, force: &Vector3D) -> crate::error::CarlaResult<()> {
        let simple_force = force.to_cxx();
        carla_sys::ffi::Actor_AddForce(self.inner.get_actor(), &simple_force);
        Ok(())
    }

    fn add_torque(&self, torque: &Vector3D) -> crate::error::CarlaResult<()> {
        let simple_torque = torque.to_cxx();
        carla_sys::ffi::Actor_AddTorque(self.inner.get_actor(), &simple_torque);
        Ok(())
    }

    fn bounding_box(&self) -> crate::geom::BoundingBox {
        let simple_bbox = carla_sys::ffi::Actor_GetBoundingBox(self.inner.get_actor());
        crate::geom::BoundingBox::from_cxx(simple_bbox)
    }

    fn destroy(&mut self) -> crate::error::CarlaResult<()> {
        if self.destroyed.get() {
            // Already destroyed, return error
            return Err(crate::error::DestroyError::InvalidActor {
                actor_id: self.id(),
            }
            .into());
        }

        // Call C++ destruction
        let success = self.inner.destroy();

        if success {
            // Mark as destroyed to prevent double destruction
            self.destroyed.set(true);
            Ok(())
        } else {
            // If C++ destroy returns false, it means the actor is invalid or
            // already destroyed in the simulation
            Err(crate::error::DestroyError::InvalidActor {
                actor_id: self.id(),
            }
            .into())
        }
    }
}

impl Drop for Actor {
    fn drop(&mut self) {
        // Only destroy if not already explicitly destroyed
        if !self.destroyed.get() && self.inner.is_alive() {
            // Call C++ destruction without error handling in Drop
            let _ = self.inner.destroy();
        }
    }
}

/// Actor state in the simulation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ActorState {
    /// Invalid actor state
    Invalid,
    /// Actor is active in the simulation
    Active,
    /// Actor is dormant (not being simulated)
    Dormant,
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
