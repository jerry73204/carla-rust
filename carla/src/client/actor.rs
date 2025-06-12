use super::ActorBase;
use crate::{
    geom::{Transform, Vector3D},
    utils::check_carla_error,
};
use anyhow::{anyhow, Result};
use carla_sys::*;

use crate::stubs::carla_attachment_type_t;
use std::ptr;

/// A base actor that represents a movable object in the simulation,
/// corresponding to `carla.Actor` in Python API.
#[derive(Clone, Debug)]
pub struct Actor {
    pub(crate) inner: *mut carla_actor_t,
}

impl Actor {
    // TODO: Implement actor classification when ActorKind and specific actor types are ready
    // pub fn into_kinds(self) -> ActorKind {
    //     // Classification logic will be implemented when specific actor types are ready
    // }

    /// Create an Actor from a raw C pointer.
    ///
    /// # Safety
    /// The pointer must be valid and not null.
    pub(crate) fn from_raw_ptr(ptr: *mut carla_actor_t) -> Result<Self> {
        if ptr.is_null() {
            return Err(anyhow!("Null actor pointer"));
        }
        Ok(Self { inner: ptr })
    }

    /// Get the actor's unique ID.
    pub fn id(&self) -> u32 {
        unsafe { carla_actor_get_id(self.inner) }
    }

    /// Get the actor's type ID (e.g., "vehicle.tesla.model3").
    pub fn type_id(&self) -> String {
        let type_id_ptr = unsafe { carla_actor_get_type_id(self.inner) };
        unsafe { crate::utils::c_string_to_rust(type_id_ptr) }
    }

    /// Get the actor's display ID for debugging.
    pub fn display_id(&self) -> String {
        let display_id_ptr = unsafe { carla_actor_get_display_id(self.inner) };
        unsafe { crate::utils::c_string_to_rust(display_id_ptr) }
    }

    /// Get the actor's current transform (location and rotation).
    pub fn get_transform(&self) -> Transform {
        let c_transform = unsafe { carla_actor_get_transform(self.inner) };
        Transform::from_c_transform(c_transform)
    }

    /// Set the actor's transform.
    pub fn set_transform(&self, transform: &Transform) -> Result<()> {
        let c_transform = transform.to_c_transform();
        let error = unsafe { carla_actor_set_transform(self.inner, &c_transform) };
        check_carla_error(error)
    }

    /// Get the actor's current velocity.
    pub fn get_velocity(&self) -> Vector3D {
        let c_velocity = unsafe { carla_actor_get_velocity(self.inner) };
        Vector3D::from_c_vector(c_velocity)
    }

    /// Get the actor's current angular velocity.
    pub fn get_angular_velocity(&self) -> Vector3D {
        let c_angular_velocity = unsafe { carla_actor_get_angular_velocity(self.inner) };
        Vector3D::from_c_vector(c_angular_velocity)
    }

    /// Get the actor's current acceleration.
    pub fn get_acceleration(&self) -> Vector3D {
        let c_acceleration = unsafe { carla_actor_get_acceleration(self.inner) };
        Vector3D::from_c_vector(c_acceleration)
    }

    /// Get the actor's parent actor, if any.
    pub fn get_parent(&self) -> Option<Actor> {
        let parent_ptr = unsafe { carla_actor_get_parent(self.inner) };
        if parent_ptr.is_null() {
            None
        } else {
            Actor::from_raw_ptr(parent_ptr).ok()
        }
    }

    /// Attach this actor to another actor.
    pub fn attach_to(
        &self,
        _other: &Actor,
        _attachment_type: carla_attachment_type_t,
    ) -> Result<()> {
        // TODO: Implement when carla_actor_attach_to is available
        todo!("Actor attachment not yet implemented in C API")
    }

    /// Check if the actor is still alive in the simulation.
    pub fn is_alive(&self) -> bool {
        unsafe { carla_actor_is_alive(self.inner) }
    }

    /// Destroy this actor.
    pub fn destroy(self) -> Result<()> {
        let error = unsafe { carla_actor_destroy(self.inner) };
        check_carla_error(error)
    }

    // TODO: Add methods for:
    // - get_attributes()
    // - set_attribute()
    // - get_world()
    // - get_semantic_tags()
}

impl ActorBase for Actor {
    fn raw_ptr(&self) -> *mut carla_actor_t {
        self.inner
    }
}

impl Drop for Actor {
    fn drop(&mut self) {
        if !self.inner.is_null() {
            unsafe {
                carla_actor_destroy(self.inner);
            }
            self.inner = ptr::null_mut();
        }
    }
}

// SAFETY: Actor wraps a thread-safe C API
unsafe impl Send for Actor {}
unsafe impl Sync for Actor {}
