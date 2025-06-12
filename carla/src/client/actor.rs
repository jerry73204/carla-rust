use super::{ActorBase, Vehicle, Walker};
use crate::{
    geom::{Transform, TransformExt, Vector3D, Vector3DExt},
    sensor::Sensor,
    stubs::{
        carla_actor_destroy_checked, carla_actor_get_parent, carla_actor_is_walker,
        carla_actor_set_transform_checked, carla_attachment_type_t,
    },
    utils::check_carla_error,
};
use anyhow::{anyhow, Result};
use carla_sys::*;

use std::ptr;

/// A base actor that represents a movable object in the simulation,
/// corresponding to `carla.Actor` in Python API.
#[derive(Clone, Debug)]
pub struct Actor {
    pub(crate) inner: *mut carla_actor_t,
}

impl Actor {
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

    /// Try to convert this Actor into a Vehicle.
    /// Returns Ok(Vehicle) if successful, or Err(Actor) if the conversion fails.
    pub fn try_into_vehicle(self) -> Result<Vehicle, Self> {
        if unsafe { carla_actor_is_vehicle(self.inner) } {
            match Vehicle::from_raw_ptr(self.inner as *mut carla_vehicle_t) {
                Ok(vehicle) => {
                    // Prevent the Actor from being dropped since we're transferring ownership
                    std::mem::forget(self);
                    Ok(vehicle)
                }
                Err(_) => Err(self),
            }
        } else {
            Err(self)
        }
    }

    /// Try to convert this Actor into a Walker.
    /// Returns Ok(Walker) if successful, or Err(Actor) if the conversion fails.
    pub fn try_into_walker(self) -> Result<Walker, Self> {
        if unsafe { carla_actor_is_walker(self.inner) } {
            match Walker::from_raw_ptr(self.inner) {
                Ok(walker) => {
                    // Prevent the Actor from being dropped since we're transferring ownership
                    std::mem::forget(self);
                    Ok(walker)
                }
                Err(_) => Err(self),
            }
        } else {
            Err(self)
        }
    }

    /// Try to convert this Actor into a Sensor.
    /// Returns Ok(Sensor) if successful, or Err(Actor) if the conversion fails.
    pub fn try_into_sensor(self) -> Result<Sensor, Self> {
        if unsafe { carla_actor_is_sensor(self.inner) } {
            match Sensor::from_raw_ptr(self.inner as *mut carla_sensor_t) {
                Ok(sensor) => {
                    // Prevent the Actor from being dropped since we're transferring ownership
                    std::mem::forget(self);
                    Ok(sensor)
                }
                Err(_) => Err(self),
            }
        } else {
            Err(self)
        }
    }

    /// Try to get a reference to this Actor as a Vehicle.
    /// Returns Some if this actor is a vehicle, None otherwise.
    pub fn as_vehicle(&self) -> Option<&Vehicle> {
        if unsafe { carla_actor_is_vehicle(self.inner) } {
            // SAFETY: We've verified this is a vehicle, and Vehicle is a newtype wrapper
            // around Actor with the same memory layout
            unsafe { Some(&*(self as *const Actor as *const Vehicle)) }
        } else {
            None
        }
    }

    /// Try to get a reference to this Actor as a Walker.
    /// Returns Some if this actor is a walker, None otherwise.
    pub fn as_walker(&self) -> Option<&Walker> {
        if unsafe { carla_actor_is_walker(self.inner) } {
            // SAFETY: We've verified this is a walker, and Walker is a newtype wrapper
            // around Actor with the same memory layout
            unsafe { Some(&*(self as *const Actor as *const Walker)) }
        } else {
            None
        }
    }

    /// Try to get a reference to this Actor as a Sensor.
    /// Returns Some if this actor is a sensor, None otherwise.
    pub fn as_sensor(&self) -> Option<&Sensor> {
        if unsafe { carla_actor_is_sensor(self.inner) } {
            // SAFETY: We've verified this is a sensor, and Sensor is a newtype wrapper
            // around Actor with the same memory layout
            unsafe { Some(&*(self as *const Actor as *const Sensor)) }
        } else {
            None
        }
    }

    /// Get the actor's unique ID.
    pub fn id(&self) -> u32 {
        unsafe { carla_actor_get_id(self.inner) }
    }

    /// Get the actor's type ID (e.g., "vehicle.tesla.model3").
    pub fn type_id(&self) -> String {
        let type_id_ptr = unsafe { carla_actor_get_type_id(self.inner) };
        unsafe { crate::utils::c_string_to_rust(type_id_ptr) }.unwrap_or_default()
    }

    /// Get the actor's display ID for debugging.
    pub fn display_id(&self) -> String {
        let display_id_ptr = unsafe { carla_actor_get_display_id(self.inner) };
        unsafe { crate::utils::c_string_to_rust(display_id_ptr) }.unwrap_or_default()
    }

    /// Get the actor's current transform (location and rotation).
    pub fn get_transform(&self) -> Transform {
        let c_transform = unsafe { carla_actor_get_transform(self.inner) };
        Transform::from_c_transform(c_transform)
    }

    /// Set the actor's transform.
    pub fn set_transform(&self, transform: &Transform) -> Result<()> {
        let c_transform = transform.to_c_transform();
        let error = unsafe { carla_actor_set_transform_checked(self.inner, &c_transform) };
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
        let error = unsafe { carla_actor_destroy_checked(self.inner) };
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

    fn id(&self) -> u32 {
        self.id()
    }

    fn type_id(&self) -> String {
        self.type_id()
    }

    fn get_transform(&self) -> Transform {
        self.get_transform()
    }

    fn is_alive(&self) -> bool {
        self.is_alive()
    }
}

impl Drop for Actor {
    fn drop(&mut self) {
        if !self.inner.is_null() {
            unsafe {
                carla_sys::carla_actor_destroy(self.inner);
            }
            self.inner = ptr::null_mut();
        }
    }
}

// SAFETY: Actor wraps a thread-safe C API
unsafe impl Send for Actor {}
unsafe impl Sync for Actor {}
