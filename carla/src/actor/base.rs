//! Base actor functionality.

use super::{ActorId, Sensor, TrafficLight, Vehicle, Walker};
use crate::{
    error::CarlaResult,
    geom::{BoundingBox, FromCxx, ToCxx, Transform, Vector3D},
    traits::ActorT,
};

/// Generic actor in the simulation.
#[derive(Debug)]
pub struct Actor {
    /// Internal handle to carla-sys Actor
    inner: carla_sys::ActorWrapper,
}

impl Actor {
    /// Create a new Actor from a carla-sys ActorWrapper
    pub fn from_cxx(inner: carla_sys::ActorWrapper) -> Self {
        Self { inner }
    }

    /// Get reference to the inner ActorWrapper for FFI operations
    pub(crate) fn actor_wrapper(&self) -> &carla_sys::ActorWrapper {
        &self.inner
    }

    /// Get reference to the inner Actor for FFI operations
    pub(crate) fn get_inner_actor(&self) -> &carla_sys::Actor {
        self.inner.get_actor()
    }

    /// Try to convert this actor to a vehicle, consuming self.
    /// Returns the Vehicle if successful, or the original Actor if not.
    pub fn to_vehicle(self) -> Result<Vehicle, Actor> {
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
    pub fn to_walker(self) -> Result<Walker, Actor> {
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
    pub fn to_sensor(self) -> Result<Sensor, Actor> {
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
    pub fn to_traffic_light(self) -> Result<TrafficLight, Actor> {
        let actor_id = self.id();
        if let Some(traffic_light_wrapper) =
            carla_sys::TrafficLightWrapper::from_actor(self.inner.get_actor())
        {
            match TrafficLight::from_cxx(traffic_light_wrapper, actor_id) {
                Ok(traffic_light) => Ok(traffic_light),
                Err(_) => Err(self),
            }
        } else {
            Err(self)
        }
    }

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
}

impl ActorT for Actor {
    fn id(&self) -> ActorId {
        self.inner.get_id()
    }

    fn type_id(&self) -> String {
        self.inner.get_type_id()
    }

    fn is_alive(&self) -> bool {
        self.inner.is_alive()
    }

    fn transform(&self) -> Transform {
        let simple_transform = self.inner.get_transform();
        Transform::from_cxx(simple_transform)
    }

    fn set_transform(&self, transform: &Transform) -> CarlaResult<()> {
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

    fn set_simulate_physics(&self, enabled: bool) -> CarlaResult<()> {
        carla_sys::ffi::Actor_SetSimulatePhysics(self.inner.get_actor(), enabled);
        Ok(())
    }

    fn add_impulse(&self, impulse: &Vector3D) -> CarlaResult<()> {
        let simple_impulse = impulse.to_cxx();
        carla_sys::ffi::Actor_AddImpulse(self.inner.get_actor(), &simple_impulse);
        Ok(())
    }

    fn add_force(&self, force: &Vector3D) -> CarlaResult<()> {
        let simple_force = force.to_cxx();
        carla_sys::ffi::Actor_AddForce(self.inner.get_actor(), &simple_force);
        Ok(())
    }

    fn add_torque(&self, torque: &Vector3D) -> CarlaResult<()> {
        let simple_torque = torque.to_cxx();
        carla_sys::ffi::Actor_AddTorque(self.inner.get_actor(), &simple_torque);
        Ok(())
    }

    fn bounding_box(&self) -> BoundingBox {
        let simple_bbox = carla_sys::ffi::Actor_GetBoundingBox(self.inner.get_actor());
        BoundingBox::from_cxx(simple_bbox)
    }
}

impl Drop for Actor {
    fn drop(&mut self) {
        // Only destroy if the actor is still alive
        if self.inner.is_alive() {
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
