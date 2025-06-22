//! Base actor functionality.

use super::{ActorId, Sensor, TrafficLight, Vehicle, Walker};
use crate::{
    actor::ActorFfi,
    geom::{Transform, Vector3D},
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

impl Drop for Actor {
    fn drop(&mut self) {
        // Check if the actor is still alive before attempting destruction
        if self.inner.is_alive() {
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
