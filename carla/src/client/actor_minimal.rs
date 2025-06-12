use crate::{
    geom::{Transform, TransformExt},
    stubs::{carla_actor_blueprint_get_attribute, carla_actor_blueprint_has_attribute},
    utils::c_string_to_rust,
};
use anyhow::{anyhow, Result};
use carla_sys::*;
use nalgebra::{Isometry3, Translation3, Vector3};
use std::ptr;

/// Represents an actor in the CARLA simulation.
#[derive(Debug)]
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

    pub(crate) fn raw_ptr(&self) -> *mut carla_actor_t {
        self.inner
    }

    pub fn id(&self) -> u32 {
        unsafe { carla_actor_get_id(self.inner) }
    }

    pub fn type_id(&self) -> Result<String> {
        let type_id_ptr = unsafe { carla_actor_get_type_id(self.inner) };
        if type_id_ptr.is_null() {
            return Err(anyhow!("Failed to get actor type ID"));
        }
        unsafe { c_string_to_rust(type_id_ptr) }
    }

    pub fn display_id(&self) -> Result<String> {
        let display_id_ptr = unsafe { carla_actor_get_display_id(self.inner) };
        if display_id_ptr.is_null() {
            return Err(anyhow!("Failed to get actor display ID"));
        }
        unsafe { c_string_to_rust(display_id_ptr) }
    }

    pub fn is_alive(&self) -> bool {
        unsafe { carla_actor_is_alive(self.inner) }
    }

    pub fn transform(&self) -> Isometry3<f32> {
        let c_transform = unsafe { carla_actor_get_transform(self.inner) };
        Transform::from_c_transform(c_transform).to_na()
    }

    pub fn location(&self) -> Translation3<f32> {
        let c_location = unsafe { carla_actor_get_location(self.inner) };
        Translation3::new(c_location.x, c_location.y, c_location.z)
    }

    pub fn velocity(&self) -> Vector3<f32> {
        let c_velocity = unsafe { carla_actor_get_velocity(self.inner) };
        Vector3::new(c_velocity.x, c_velocity.y, c_velocity.z)
    }

    pub fn acceleration(&self) -> Vector3<f32> {
        let c_acceleration = unsafe { carla_actor_get_acceleration(self.inner) };
        Vector3::new(c_acceleration.x, c_acceleration.y, c_acceleration.z)
    }

    pub fn angular_velocity(&self) -> Vector3<f32> {
        let c_angular_velocity = unsafe { carla_actor_get_angular_velocity(self.inner) };
        Vector3::new(
            c_angular_velocity.x,
            c_angular_velocity.y,
            c_angular_velocity.z,
        )
    }

    pub fn set_transform(&mut self, transform: &Isometry3<f32>) {
        let c_transform = Transform::from_na(transform).to_c_transform();
        unsafe { carla_actor_set_transform(self.inner, &c_transform) };
    }

    pub fn set_location(&mut self, location: &Translation3<f32>) {
        let c_location = carla_vector3d_t {
            x: location.x,
            y: location.y,
            z: location.z,
        };
        unsafe { carla_actor_set_location(self.inner, &c_location) };
    }

    pub fn set_simulate_physics(&mut self, enabled: bool) {
        unsafe { carla_actor_set_simulate_physics(self.inner, enabled) };
    }

    pub fn set_enable_gravity(&mut self, enabled: bool) {
        unsafe { carla_actor_set_enable_gravity(self.inner, enabled) };
    }

    pub fn add_impulse(&mut self, impulse: &Vector3<f32>) {
        let c_impulse = carla_vector3d_t {
            x: impulse.x,
            y: impulse.y,
            z: impulse.z,
        };
        unsafe { carla_actor_add_impulse(self.inner, &c_impulse) };
    }

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

    pub fn add_force(&mut self, force: &Vector3<f32>) {
        let c_force = carla_vector3d_t {
            x: force.x,
            y: force.y,
            z: force.z,
        };
        unsafe { carla_actor_add_force(self.inner, &c_force) };
    }

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

    pub fn add_angular_impulse(&mut self, angular_impulse: &Vector3<f32>) {
        let c_angular_impulse = carla_vector3d_t {
            x: angular_impulse.x,
            y: angular_impulse.y,
            z: angular_impulse.z,
        };
        unsafe { carla_actor_add_angular_impulse(self.inner, &c_angular_impulse) };
    }

    pub fn add_torque(&mut self, torque: &Vector3<f32>) {
        let c_torque = carla_vector3d_t {
            x: torque.x,
            y: torque.y,
            z: torque.z,
        };
        unsafe { carla_actor_add_torque(self.inner, &c_torque) };
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

/// A blueprint library contains all available actor blueprints.
#[derive(Debug)]
pub struct BlueprintLibrary {
    pub(crate) inner: *mut carla_blueprint_library_t,
}

impl BlueprintLibrary {
    /// Create a BlueprintLibrary from a raw C pointer.
    ///
    /// # Safety
    /// The pointer must be valid and not null.
    pub(crate) fn from_raw_ptr(ptr: *mut carla_blueprint_library_t) -> Result<Self> {
        if ptr.is_null() {
            return Err(anyhow!("Null blueprint library pointer"));
        }
        Ok(Self { inner: ptr })
    }

    pub fn size(&self) -> usize {
        unsafe { carla_blueprint_library_size(self.inner) }
    }

    pub fn at(&self, index: usize) -> Option<ActorBlueprint> {
        let blueprint_ptr = unsafe { carla_blueprint_library_at(self.inner, index) };
        if blueprint_ptr.is_null() {
            None
        } else {
            ActorBlueprint::from_raw_ptr(blueprint_ptr).ok()
        }
    }

    pub fn find(&self, id: &str) -> Option<ActorBlueprint> {
        let id_cstr = std::ffi::CString::new(id).ok()?;
        let blueprint_ptr = unsafe { carla_blueprint_library_find(self.inner, id_cstr.as_ptr()) };
        if blueprint_ptr.is_null() {
            None
        } else {
            ActorBlueprint::from_raw_ptr(blueprint_ptr).ok()
        }
    }
}

impl Drop for BlueprintLibrary {
    fn drop(&mut self) {
        if !self.inner.is_null() {
            unsafe {
                carla_blueprint_library_free(self.inner);
            }
            self.inner = ptr::null_mut();
        }
    }
}

// SAFETY: BlueprintLibrary wraps a thread-safe C API
unsafe impl Send for BlueprintLibrary {}
unsafe impl Sync for BlueprintLibrary {}

/// A blueprint of an actor with all its attributes.
#[derive(Debug)]
pub struct ActorBlueprint {
    pub(crate) inner: *mut carla_actor_blueprint_t,
}

impl ActorBlueprint {
    /// Create an ActorBlueprint from a raw C pointer.
    ///
    /// # Safety
    /// The pointer must be valid and not null.
    pub(crate) fn from_raw_ptr(ptr: *mut carla_actor_blueprint_t) -> Result<Self> {
        if ptr.is_null() {
            return Err(anyhow!("Null actor blueprint pointer"));
        }
        Ok(Self { inner: ptr })
    }

    pub(crate) fn raw_ptr(&self) -> *mut carla_actor_blueprint_t {
        self.inner
    }

    pub fn id(&self) -> Result<String> {
        let id_ptr = unsafe { carla_actor_blueprint_get_id(self.inner) };
        if id_ptr.is_null() {
            return Err(anyhow!("Failed to get blueprint ID"));
        }
        unsafe { c_string_to_rust(id_ptr) }
    }

    pub fn has_tag(&self, tag: &str) -> bool {
        let tag_cstr = std::ffi::CString::new(tag).unwrap_or_default();
        unsafe { carla_actor_blueprint_has_tag(self.inner, tag_cstr.as_ptr()) }
    }

    pub fn match_tags(&self, wildcard_pattern: &str) -> bool {
        let pattern_cstr = std::ffi::CString::new(wildcard_pattern).unwrap_or_default();
        unsafe { carla_actor_blueprint_match_tags(self.inner, pattern_cstr.as_ptr()) }
    }

    /// Set an attribute value for this blueprint.
    ///
    /// # Arguments
    /// * `id` - The attribute ID/name
    /// * `value` - The value to set
    ///
    /// # Returns
    /// `true` if the attribute was successfully set, `false` otherwise
    pub fn set_attribute(&mut self, id: &str, value: &str) -> bool {
        let id_cstr = std::ffi::CString::new(id).unwrap_or_default();
        let value_cstr = std::ffi::CString::new(value).unwrap_or_default();
        unsafe {
            carla_actor_blueprint_set_attribute(self.inner, id_cstr.as_ptr(), value_cstr.as_ptr())
        }
    }

    /// Get an attribute value from this blueprint.
    ///
    /// # Arguments
    /// * `id` - The attribute ID/name to retrieve
    ///
    /// # Returns
    /// The attribute value as a string, or an error if the attribute doesn't exist
    pub fn get_attribute(&self, id: &str) -> Result<String> {
        let id_cstr =
            std::ffi::CString::new(id).map_err(|e| anyhow!("Invalid attribute ID: {}", e))?;
        let value_ptr =
            unsafe { carla_actor_blueprint_get_attribute(self.inner, id_cstr.as_ptr()) };

        if value_ptr.is_null() {
            return Err(anyhow!("Attribute '{}' not found", id));
        }

        unsafe { c_string_to_rust(value_ptr) }
    }

    /// Check if this blueprint contains a specific attribute.
    ///
    /// # Arguments
    /// * `id` - The attribute ID/name to check for
    pub fn has_attribute(&self, id: &str) -> bool {
        let id_cstr = std::ffi::CString::new(id).unwrap_or_default();
        unsafe { carla_actor_blueprint_has_attribute(self.inner, id_cstr.as_ptr()) }
    }

    /// Get all tags associated with this blueprint.
    pub fn get_tags(&self) -> Result<Vec<String>> {
        // TODO: Implement when C API provides carla_actor_blueprint_get_tags
        Err(anyhow!(
            "Blueprint tag enumeration not yet implemented in C API"
        ))
    }

    /// Get all attributes of this blueprint.
    pub fn get_attributes(&self) -> Result<std::collections::HashMap<String, String>> {
        // TODO: Implement when C API provides carla_actor_blueprint_get_all_attributes
        Err(anyhow!(
            "Blueprint attribute enumeration not yet implemented in C API"
        ))
    }

    /// Get the display name of this blueprint (human-readable name).
    pub fn display_name(&self) -> Result<String> {
        // For now, use the ID as display name
        // TODO: Implement when C API provides carla_actor_blueprint_get_display_name
        self.id()
    }

    /// Check if this blueprint is for a vehicle actor.
    pub fn is_vehicle(&self) -> bool {
        if let Ok(id) = self.id() {
            id.starts_with("vehicle.")
        } else {
            false
        }
    }

    /// Check if this blueprint is for a sensor actor.
    pub fn is_sensor(&self) -> bool {
        if let Ok(id) = self.id() {
            id.starts_with("sensor.")
        } else {
            false
        }
    }

    /// Check if this blueprint is for a walker (pedestrian) actor.
    pub fn is_walker(&self) -> bool {
        if let Ok(id) = self.id() {
            id.starts_with("walker.")
        } else {
            false
        }
    }

    /// Check if this blueprint is for a traffic light actor.
    pub fn is_traffic_light(&self) -> bool {
        if let Ok(id) = self.id() {
            id.contains("trafficlight")
        } else {
            false
        }
    }

    /// Check if this blueprint is for a traffic sign actor.
    pub fn is_traffic_sign(&self) -> bool {
        if let Ok(id) = self.id() {
            id.contains("trafficsign")
        } else {
            false
        }
    }

    /// Get the actor type from the blueprint ID.
    ///
    /// # Returns
    /// The main actor type (e.g., "vehicle", "sensor", "walker")
    pub fn actor_type(&self) -> Result<String> {
        let id = self.id()?;
        if let Some(main_type) = id.split('.').next() {
            Ok(main_type.to_string())
        } else {
            Err(anyhow!("Unable to determine actor type from ID: {}", id))
        }
    }

    /// Get the blueprint generation (if applicable for vehicles).
    pub fn generation(&self) -> Result<String> {
        // Try to get generation from attributes first
        if let Ok(gen) = self.get_attribute("generation") {
            return Ok(gen);
        }

        // Fall back to parsing from ID for vehicles
        if self.is_vehicle() {
            let id = self.id()?;
            // Vehicle IDs often have generation in them (e.g., vehicle.audi.a2)
            if let Some(generation_str) = id.split('.').nth(2) {
                Ok(generation_str.to_string())
            } else {
                Ok("unknown".to_string())
            }
        } else {
            Err(anyhow!(
                "Generation not applicable for non-vehicle blueprints"
            ))
        }
    }

    /// Configure the blueprint with common vehicle settings.
    ///
    /// # Arguments
    /// * `role_name` - The role name for the vehicle (e.g., "hero", "autopilot")
    ///
    /// # Returns
    /// `true` if all attributes were set successfully
    pub fn configure_vehicle(&mut self, role_name: &str) -> bool {
        if !self.is_vehicle() {
            return false;
        }

        self.set_attribute("role_name", role_name)
    }

    /// Configure the blueprint with common sensor settings.
    ///
    /// # Arguments
    /// * `tick` - Sensor tick rate in seconds
    ///
    /// # Returns
    /// `true` if the attribute was set successfully
    pub fn configure_sensor(&mut self, tick: f32) -> bool {
        if !self.is_sensor() {
            return false;
        }

        self.set_attribute("sensor_tick", &tick.to_string())
    }
}

// SAFETY: ActorBlueprint wraps a thread-safe C API
unsafe impl Send for ActorBlueprint {}
unsafe impl Sync for ActorBlueprint {}
