use super::{Actor, ActorAttributeValueList, World};
use crate::{
    geom::{Location, LocationExt, Transform, TransformExt, Vector3D, Vector3DExt},
    rpc::ActorId,
    utils::{check_carla_error, c_string_to_rust},
};
use anyhow::{anyhow, Result};
use carla_sys::*;
use nalgebra::{Isometry3, Translation3, Vector3};
use std::ptr;

/// This trait defines a basic actor in the simulation. It is
/// implemented on all actor type variants.
pub trait ActorBase: Clone {
    /// Get the raw C pointer to the actor.
    fn raw_ptr(&self) -> *mut carla_actor_t;

    fn into_actor(self) -> Result<Actor> {
        Actor::from_raw_ptr(self.raw_ptr())
    }

    fn id(&self) -> ActorId {
        unsafe { carla_actor_get_id(self.raw_ptr()) }
    }

    fn type_id(&self) -> Result<String> {
        let type_id_ptr = unsafe { carla_actor_get_type_id(self.raw_ptr()) };
        if type_id_ptr.is_null() {
            return Err(anyhow!("Failed to get actor type ID"));
        }
        unsafe { c_string_to_rust(type_id_ptr) }
    }

    fn display_id(&self) -> Result<String> {
        let display_id_ptr = unsafe { carla_actor_get_display_id(self.raw_ptr()) };
        if display_id_ptr.is_null() {
            return Err(anyhow!("Failed to get actor display ID"));
        }
        unsafe { c_string_to_rust(display_id_ptr) }
    }

    fn parent_id(&self) -> ActorId {
        // Note: Parent ID needs to be implemented in C wrapper
        0 // Placeholder
    }

    fn semantic_tags(&self) -> Vec<u8> {
        // Note: Semantic tags need to be implemented in C wrapper
        Vec::new() // Placeholder
    }

    fn parent(&self) -> Option<Actor> {
        // Note: Parent actor access needs to be implemented in C wrapper
        None // Placeholder
    }

    fn attributes(&self) -> Result<ActorAttributeValueList<'_>> {
        // Note: Actor attributes need to be implemented in C wrapper
        Err(anyhow!("Actor attributes not yet implemented in C wrapper"))
    }

    fn world(&self) -> Result<World> {
        // Note: World access from actor needs to be implemented in C wrapper
        Err(anyhow!("World access from actor not yet implemented in C wrapper"))
    }

    fn location(&self) -> Translation3<f32> {
        let c_location = unsafe { carla_actor_get_location(self.raw_ptr()) };
        Translation3::new(c_location.x, c_location.y, c_location.z)
    }

    fn transform(&self) -> Isometry3<f32> {
        let c_transform = unsafe { carla_actor_get_transform(self.raw_ptr()) };
        Transform::from_c_transform(c_transform).to_na()
    }

    fn velocity(&self) -> Vector3<f32> {
        let c_velocity = unsafe { carla_actor_get_velocity(self.raw_ptr()) };
        Vector3::new(c_velocity.x, c_velocity.y, c_velocity.z)
    }

    fn acceleration(&self) -> Vector3<f32> {
        let c_acceleration = unsafe { carla_actor_get_acceleration(self.raw_ptr()) };
        Vector3::new(c_acceleration.x, c_acceleration.y, c_acceleration.z)
    }

    fn angular_velocity(&self) -> Vector3<f32> {
        let c_angular_velocity = unsafe { carla_actor_get_angular_velocity(self.raw_ptr()) };
        Vector3::new(c_angular_velocity.x, c_angular_velocity.y, c_angular_velocity.z)
    }

    fn set_location(&self, location: &Translation3<f32>) {
        let c_location = carla_vector3d_t {
            x: location.x,
            y: location.y,
            z: location.z,
        };
        unsafe { carla_actor_set_location(self.raw_ptr(), &c_location) };
    }

    fn set_transform(&self, transform: &Isometry3<f32>) {
        let c_transform = Transform::from_na(transform).to_c_transform();
        unsafe { carla_actor_set_transform(self.raw_ptr(), &c_transform) };
    }

    fn set_target_velocity(&self, vector: &Vector3<f32>) {
        // Note: Target velocity setting needs to be implemented in C wrapper
        // For now, this is a no-op
    }

    fn set_target_angular_velocity(&self, vector: &Vector3<f32>) {
        // Note: Target angular velocity setting needs to be implemented in C wrapper
        // For now, this is a no-op
    }

    fn enable_constant_velocity(&self, vector: &Vector3<f32>) {
        // Note: Constant velocity enabling needs to be implemented in C wrapper
        // For now, this is a no-op
    }

    fn disable_constant_velocity(&self) {
        // Note: Constant velocity disabling needs to be implemented in C wrapper
        // For now, this is a no-op
    }

    fn add_impulse(&self, vector: &Vector3<f32>) {
        let c_impulse = carla_vector3d_t {
            x: vector.x,
            y: vector.y,
            z: vector.z,
        };
        unsafe { carla_actor_add_impulse(self.raw_ptr(), &c_impulse) };
    }

    fn add_impulse_at(&self, vector: &Vector3<f32>, location: &Vector3<f32>) {
        let c_impulse = carla_vector3d_t {
            x: vector.x,
            y: vector.y,
            z: vector.z,
        };
        let c_location = carla_vector3d_t {
            x: location.x,
            y: location.y,
            z: location.z,
        };
        unsafe { carla_actor_add_impulse_at_location(self.raw_ptr(), &c_impulse, &c_location) };
    }

    fn add_force(&self, vector: &Vector3<f32>) {
        let c_force = carla_vector3d_t {
            x: vector.x,
            y: vector.y,
            z: vector.z,
        };
        unsafe { carla_actor_add_force(self.raw_ptr(), &c_force) };
    }

    fn add_force_at(&self, vector: &Vector3<f32>, location: &Vector3<f32>) {
        let c_force = carla_vector3d_t {
            x: vector.x,
            y: vector.y,
            z: vector.z,
        };
        let c_location = carla_vector3d_t {
            x: location.x,
            y: location.y,
            z: location.z,
        };
        unsafe { carla_actor_add_force_at_location(self.raw_ptr(), &c_force, &c_location) };
    }

    fn add_angular_impulse(&self, vector: &Vector3<f32>) {
        let c_angular_impulse = carla_vector3d_t {
            x: vector.x,
            y: vector.y,
            z: vector.z,
        };
        unsafe { carla_actor_add_angular_impulse(self.raw_ptr(), &c_angular_impulse) };
    }

    fn add_torque(&self, vector: &Vector3<f32>) {
        let c_torque = carla_vector3d_t {
            x: vector.x,
            y: vector.y,
            z: vector.z,
        };
        unsafe { carla_actor_add_torque(self.raw_ptr(), &c_torque) };
    }

    fn set_simulate_physics(&self, enabled: bool) {
        unsafe { carla_actor_set_simulate_physics(self.raw_ptr(), enabled) };
    }

    fn set_enable_gravity(&self, enabled: bool) {
        unsafe { carla_actor_set_enable_gravity(self.raw_ptr(), enabled) };
    }

    fn is_alive(&self) -> bool {
        unsafe { carla_actor_is_alive(self.raw_ptr()) }
    }

    fn is_dormant(&self) -> bool {
        // Note: Dormant state check needs to be implemented in C wrapper
        false // Placeholder
    }

    fn is_active(&self) -> bool {
        // Note: Active state check needs to be implemented in C wrapper
        true // Placeholder
    }
}
