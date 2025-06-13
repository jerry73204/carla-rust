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
use nalgebra::{Isometry3, Translation3, Vector3};

use std::ptr;

/// Actor attribute information.
#[derive(Clone, Debug)]
pub struct ActorAttribute {
    /// The attribute ID/name.
    pub id: String,
    /// The attribute value.
    pub value: String,
    /// Whether the attribute is modifiable.
    pub is_modifiable: bool,
}

impl ActorAttribute {
    /// Create a new actor attribute.
    pub fn new(id: String, value: String, is_modifiable: bool) -> Self {
        Self {
            id,
            value,
            is_modifiable,
        }
    }
}

// TODO: Remove these placeholder types when proper TrafficLight(Actor) and TrafficSign(Actor) are implemented
/// Placeholder traffic light type for Actor conversion methods.
/// This will be replaced with a proper newtype wrapper in the future.
#[derive(Clone, Debug)]
pub struct TrafficLight(pub Actor);

/// Placeholder traffic sign type for Actor conversion methods.
/// This will be replaced with a proper newtype wrapper in the future.
#[derive(Clone, Debug)]
pub struct TrafficSign(pub Actor);

impl TrafficLight {
    /// Create a TrafficLight from a raw C pointer.
    /// TODO: Remove when proper TrafficLight implementation is available
    pub(crate) fn from_raw_ptr(ptr: *mut carla_actor_t) -> Result<Self> {
        let actor = Actor::from_raw_ptr(ptr)?;
        Ok(Self(actor))
    }
}

impl TrafficSign {
    /// Create a TrafficSign from a raw C pointer.
    /// TODO: Remove when proper TrafficSign implementation is available
    pub(crate) fn from_raw_ptr(ptr: *mut carla_actor_t) -> Result<Self> {
        let actor = Actor::from_raw_ptr(ptr)?;
        Ok(Self(actor))
    }
}

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

    /// Get the raw C pointer for this actor.
    pub(crate) fn raw_ptr(&self) -> *mut carla_actor_t {
        self.inner
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

    /// Try to convert this Actor into a TrafficLight.
    /// Returns Ok(TrafficLight) if successful, or Err(Actor) if the conversion fails.
    ///
    /// TODO: Update type checking when proper TrafficLight newtype wrapper is available
    pub fn try_into_traffic_light(self) -> Result<TrafficLight, Self> {
        if self.type_id().contains("traffic.traffic_light") {
            match TrafficLight::from_raw_ptr(self.inner) {
                Ok(traffic_light) => {
                    // Prevent the Actor from being dropped since we're transferring ownership
                    std::mem::forget(self);
                    Ok(traffic_light)
                }
                Err(_) => Err(self),
            }
        } else {
            Err(self)
        }
    }

    /// Try to convert this Actor into a TrafficSign.
    /// Returns Ok(TrafficSign) if successful, or Err(Actor) if the conversion fails.
    ///
    /// TODO: Update type checking when TrafficSign newtype wrapper is available
    pub fn try_into_traffic_sign(self) -> Result<TrafficSign, Self> {
        if self.type_id().contains("static.prop") || self.type_id().contains("traffic.sign") {
            match TrafficSign::from_raw_ptr(self.inner) {
                Ok(traffic_sign) => {
                    // Prevent the Actor from being dropped since we're transferring ownership
                    std::mem::forget(self);
                    Ok(traffic_sign)
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

    /// Try to get a reference to this Actor as a TrafficLight.
    /// Returns Some if this actor is a traffic light, None otherwise.
    ///
    /// TODO: Update type checking when proper TrafficLight newtype wrapper is available
    pub fn as_traffic_light(&self) -> Option<&TrafficLight> {
        if self.type_id().contains("traffic.traffic_light") {
            // SAFETY: We've verified this is a traffic light, and TrafficLight is a newtype wrapper
            // around Actor with the same memory layout
            unsafe { Some(&*(self as *const Actor as *const TrafficLight)) }
        } else {
            None
        }
    }

    /// Try to get a reference to this Actor as a TrafficSign.
    /// Returns Some if this actor is a traffic sign, None otherwise.
    ///
    /// TODO: Update type checking when TrafficSign newtype wrapper is available
    pub fn as_traffic_sign(&self) -> Option<&TrafficSign> {
        if self.type_id().contains("static.prop") || self.type_id().contains("traffic.sign") {
            // SAFETY: We've verified this is a traffic sign, and TrafficSign is a newtype wrapper
            // around Actor with the same memory layout
            unsafe { Some(&*(self as *const Actor as *const TrafficSign)) }
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

    /// Get the actor's current location.
    pub fn get_location(&self) -> Translation3<f32> {
        let c_location = unsafe { carla_actor_get_location(self.inner) };
        Translation3::new(c_location.x, c_location.y, c_location.z)
    }

    /// Set the actor's location.
    pub fn set_location(&mut self, location: &Translation3<f32>) {
        let c_location = carla_vector3d_t {
            x: location.x,
            y: location.y,
            z: location.z,
        };
        unsafe { carla_actor_set_location(self.inner, &c_location) };
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

    /// Enable or disable physics simulation for this actor.
    pub fn set_simulate_physics(&mut self, enabled: bool) {
        unsafe { carla_actor_set_simulate_physics(self.inner, enabled) };
    }

    /// Enable or disable gravity for this actor.
    pub fn set_enable_gravity(&mut self, enabled: bool) {
        unsafe { carla_actor_set_enable_gravity(self.inner, enabled) };
    }

    /// Add an impulse to the actor.
    pub fn add_impulse(&mut self, impulse: &Vector3<f32>) {
        let c_impulse = carla_vector3d_t {
            x: impulse.x,
            y: impulse.y,
            z: impulse.z,
        };
        unsafe { carla_actor_add_impulse(self.inner, &c_impulse) };
    }

    /// Add an impulse to the actor at a specific location.
    pub fn add_impulse_at_location(
        &mut self,
        impulse: &Vector3<f32>,
        location: &Translation3<f32>,
    ) {
        let c_impulse = carla_vector3d_t {
            x: impulse.x,
            y: impulse.y,
            z: impulse.z,
        };
        let c_location = carla_vector3d_t {
            x: location.x,
            y: location.y,
            z: location.z,
        };
        unsafe { carla_actor_add_impulse_at_location(self.inner, &c_impulse, &c_location) };
    }

    /// Add a force to the actor.
    pub fn add_force(&mut self, force: &Vector3<f32>) {
        let c_force = carla_vector3d_t {
            x: force.x,
            y: force.y,
            z: force.z,
        };
        unsafe { carla_actor_add_force(self.inner, &c_force) };
    }

    /// Add a force to the actor at a specific location.
    pub fn add_force_at_location(&mut self, force: &Vector3<f32>, location: &Translation3<f32>) {
        let c_force = carla_vector3d_t {
            x: force.x,
            y: force.y,
            z: force.z,
        };
        let c_location = carla_vector3d_t {
            x: location.x,
            y: location.y,
            z: location.z,
        };
        unsafe { carla_actor_add_force_at_location(self.inner, &c_force, &c_location) };
    }

    /// Add an angular impulse to the actor.
    pub fn add_angular_impulse(&mut self, angular_impulse: &Vector3<f32>) {
        let c_angular_impulse = carla_vector3d_t {
            x: angular_impulse.x,
            y: angular_impulse.y,
            z: angular_impulse.z,
        };
        unsafe { carla_actor_add_angular_impulse(self.inner, &c_angular_impulse) };
    }

    /// Add a torque to the actor.
    pub fn add_torque(&mut self, torque: &Vector3<f32>) {
        let c_torque = carla_vector3d_t {
            x: torque.x,
            y: torque.y,
            z: torque.z,
        };
        unsafe { carla_actor_add_torque(self.inner, &c_torque) };
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

    /// Get all attributes of this actor.
    ///
    /// Note: In CARLA's C API, attributes are typically accessed through blueprints.
    /// This method provides a placeholder for future API extensions.
    pub fn get_attributes(&self) -> Result<Vec<ActorAttribute>> {
        // TODO: Implement when C API provides actor instance attribute access
        // For now, return empty vector as actor attributes are managed at blueprint level
        Ok(Vec::new())
    }

    /// Set an attribute value for this actor.
    ///
    /// Note: In CARLA's C API, attributes are typically set on blueprints before spawning.
    /// This method provides a placeholder for future API extensions.
    pub fn set_attribute(&self, _id: &str, _value: &str) -> Result<()> {
        // TODO: Implement when C API provides actor instance attribute modification
        // For now, return error as actor attributes are typically immutable after spawning
        Err(anyhow!(
            "Actor attributes cannot be modified after spawning in current CARLA version"
        ))
    }

    // TODO: Add methods for:
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
