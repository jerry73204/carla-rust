use crate::{
    stubs::{carla_actor_blueprint_get_attribute, carla_actor_blueprint_has_attribute},
    utils::c_string_to_rust,
};
use anyhow::{anyhow, Result};
use carla_sys::*;
use std::ptr;

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
