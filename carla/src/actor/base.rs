//! Base actor functionality.

use super::{traits::ActorExt, ActorId, Sensor, TrafficLight, Vehicle, Walker};
use crate::{
    actor::ActorFfi,
    error::{CarlaError, CarlaResult},
    geom::{BoundingBox, FromCxx, ToCxx, Transform, Vector3D},
};
use std::{
    rc::Rc,
    sync::{
        atomic::{AtomicU8, Ordering},
        Arc,
    },
};

/// Internal state values for atomic tracking
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum InternalState {
    Alive = 0,
    Destroying = 1,
    Destroyed = 2,
    Invalid = 3,
}

impl From<u8> for InternalState {
    fn from(value: u8) -> Self {
        match value {
            0 => InternalState::Alive,
            1 => InternalState::Destroying,
            2 => InternalState::Destroyed,
            3 => InternalState::Invalid,
            _ => InternalState::Invalid,
        }
    }
}

/// Generic actor in the simulation.
#[derive(Debug)]
pub struct Actor {
    /// Internal handle to carla-sys Actor
    inner: Rc<carla_sys::ActorWrapper>,
    /// Atomic state tracking
    state: Arc<AtomicU8>,
}

impl Actor {
    /// Create a new Actor from a carla-sys ActorWrapper
    pub fn from_cxx(inner: carla_sys::ActorWrapper) -> Self {
        let initial_state = if inner.is_alive() {
            InternalState::Alive
        } else {
            InternalState::Invalid
        };

        Self {
            inner: Rc::new(inner),
            state: Arc::new(AtomicU8::new(initial_state as u8)),
        }
    }

    /// Get reference to the inner Actor for FFI operations
    pub(crate) fn inner_actor(&self) -> &carla_sys::Actor {
        self.inner.get_actor()
    }

    /// Get the inner ActorWrapper (for internal use only)
    pub(crate) fn inner_wrapper(&self) -> &carla_sys::ActorWrapper {
        &self.inner
    }

    /// Try to convert this actor to a vehicle, consuming self.
    /// Returns the Vehicle if successful, or the original Actor if not.
    pub fn into_vehicle(self) -> Result<Vehicle, Actor> {
        if let Some(vehicle_wrapper) =
            carla_sys::VehicleWrapper::from_actor(self.inner.get_shared_ptr())
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
        if let Some(walker_wrapper) =
            carla_sys::WalkerWrapper::from_actor(self.inner.get_shared_ptr())
        {
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
        if let Some(sensor_wrapper) =
            carla_sys::SensorWrapper::from_actor(self.inner.get_shared_ptr())
        {
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
            carla_sys::TrafficLightWrapper::from_actor(self.inner.get_shared_ptr())
        {
            match TrafficLight::from_cxx(traffic_light_wrapper) {
                Ok(traffic_light) => Ok(traffic_light),
                Err(_) => Err(self),
            }
        } else {
            Err(self)
        }
    }

    /// Get actor attributes.
    pub fn attributes(&self) -> super::AttributeList {
        let attr_strings = carla_sys::ffi::Actor_GetAttributes(self.inner.get_actor());
        super::AttributeList::from_strings(attr_strings)
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

    /// Try to get the actor's transform with crash protection.
    ///
    /// This method adds defensive checks to prevent crashes observed
    /// when accessing transform on invalid actors. The FFI layer also
    /// includes exception handling to catch C++ exceptions.
    pub fn try_transform(&self) -> CarlaResult<Transform> {
        self.validate_for_operation("transform")?;
        Ok(self.transform())
    }

    /// Try to get the actor's velocity with crash protection.
    ///
    /// This method adds defensive checks to prevent crashes observed
    /// when accessing velocity on invalid actors. The FFI layer also
    /// includes exception handling to catch C++ exceptions.
    pub fn try_velocity(&self) -> CarlaResult<Vector3D> {
        self.validate_for_operation("velocity")?;
        Ok(self.velocity())
    }

    /// Try to get the actor's angular velocity with crash protection.
    pub fn try_angular_velocity(&self) -> CarlaResult<Vector3D> {
        self.validate_for_operation("angular_velocity")?;
        Ok(self.angular_velocity())
    }

    /// Try to get the actor's acceleration with crash protection.
    pub fn try_acceleration(&self) -> CarlaResult<Vector3D> {
        self.validate_for_operation("acceleration")?;
        Ok(self.acceleration())
    }

    /// Try to get the actor's bounding box with crash protection.
    pub fn try_bounding_box(&self) -> CarlaResult<BoundingBox> {
        self.validate_for_operation("bounding_box")?;
        Ok(self.bounding_box())
    }

    /// Check if the actor is dormant safely.
    pub fn try_is_dormant(&self) -> CarlaResult<bool> {
        self.validate_for_operation("is_dormant")?;
        Ok(self.is_dormant())
    }

    /// Get semantic tags safely.
    pub fn try_semantic_tags(&self) -> CarlaResult<Vec<u8>> {
        self.validate_for_operation("semantic_tags")?;
        Ok(self.semantic_tags())
    }

    /// Get parent actor ID safely.
    pub fn try_parent_id(&self) -> CarlaResult<Option<ActorId>> {
        self.validate_for_operation("parent_id")?;
        Ok(self.parent_id())
    }

    /// Validate actor before operation.
    fn validate_for_operation(&self, operation: &str) -> CarlaResult<()> {
        let state = InternalState::from(self.state.load(Ordering::SeqCst));

        match state {
            InternalState::Alive => {
                if !self.inner.is_alive() {
                    self.state
                        .store(InternalState::Invalid as u8, Ordering::SeqCst);
                    Err(CarlaError::Runtime(format!(
                        "Actor {} is no longer alive for operation: {}",
                        self.id(),
                        operation
                    )))
                } else {
                    Ok(())
                }
            }
            InternalState::Destroying => Err(CarlaError::Runtime(format!(
                "Cannot perform {} on actor being destroyed",
                operation
            ))),
            InternalState::Destroyed => Err(CarlaError::Runtime(format!(
                "Cannot perform {} on destroyed actor",
                operation
            ))),
            InternalState::Invalid => Err(CarlaError::Runtime(format!(
                "Cannot perform {} on invalid actor",
                operation
            ))),
        }
    }
}

impl ActorFfi for Actor {
    fn as_actor_ffi(&self) -> carla_sys::ActorWrapper {
        // Actor already contains ActorWrapper, so clone it
        // This is efficient since ActorWrapper contains SharedPtr which is cheap to clone
        (*self.inner).clone()
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
        let state = InternalState::from(self.state.load(Ordering::SeqCst));
        state != InternalState::Destroyed && self.inner.is_alive()
    }

    fn is_valid(&self) -> bool {
        let state = InternalState::from(self.state.load(Ordering::SeqCst));
        if state != InternalState::Alive {
            return false;
        }

        if !self.inner.is_alive() {
            self.state
                .store(InternalState::Invalid as u8, Ordering::SeqCst);
            return false;
        }

        // Check actor state
        match self.actor_state() {
            ActorState::Active | ActorState::Dormant => true,
            ActorState::Invalid => false,
        }
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
        // Try to set state from Alive to Destroying
        let prev_state = self.state.compare_exchange(
            InternalState::Alive as u8,
            InternalState::Destroying as u8,
            Ordering::SeqCst,
            Ordering::SeqCst,
        );

        match InternalState::from(prev_state.unwrap_or(InternalState::Invalid as u8)) {
            InternalState::Alive => {
                // We successfully transitioned to Destroying
                // Get mutable access to destroy
                match Rc::get_mut(&mut self.inner) {
                    Some(inner) => {
                        let result = inner.destroy();

                        // Update state based on result
                        if result {
                            self.state
                                .store(InternalState::Destroyed as u8, Ordering::SeqCst);
                            Ok(())
                        } else {
                            // Destruction failed, revert to Alive
                            self.state
                                .store(InternalState::Alive as u8, Ordering::SeqCst);
                            Err(crate::error::DestroyError::InvalidActor {
                                actor_id: self.id(),
                            }
                            .into())
                        }
                    }
                    None => {
                        // Cannot get mutable access
                        self.state
                            .store(InternalState::Alive as u8, Ordering::SeqCst);
                        Err(CarlaError::Runtime(
                            "Cannot get mutable access to actor".to_string(),
                        ))
                    }
                }
            }
            InternalState::Destroying => Err(CarlaError::Runtime(
                "Actor is already being destroyed".to_string(),
            )),
            InternalState::Destroyed => Err(CarlaError::Runtime(
                "Actor is already destroyed".to_string(),
            )),
            InternalState::Invalid => Err(CarlaError::Runtime("Actor is invalid".to_string())),
        }
    }
}

impl Drop for Actor {
    fn drop(&mut self) {
        // Only destroy if the actor is still alive and not already destroyed
        let state = InternalState::from(self.state.load(Ordering::SeqCst));
        if state == InternalState::Alive && self.inner.is_alive() {
            // Try to get mutable access to destroy
            if let Some(inner) = Rc::get_mut(&mut self.inner) {
                let _ = inner.destroy();
            }
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
