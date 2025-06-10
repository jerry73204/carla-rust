use super::ActorBlueprint;
use crate::utils::{rust_string_to_c, check_carla_error};
use anyhow::{anyhow, Result};
use carla_sys::*;
use std::ptr;

/// Provides blueprints used to spawn actors, corresponding to
/// `carla.BlueprintLibrary` in Python API.
#[derive(Clone, Debug)]
pub struct BlueprintLibrary {
    inner: *mut carla_blueprint_library_t,
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

    pub fn filter(&self, pattern: &str) -> Result<Vec<ActorBlueprint>> {
        let pattern_cstring = rust_string_to_c(pattern)?;
        let mut count = 0;
        let blueprint_array = unsafe {
            carla_blueprint_library_filter(self.inner, pattern_cstring.as_ptr(), &mut count)
        };
        
        if blueprint_array.is_null() {
            return Ok(Vec::new());
        }
        
        let mut blueprints = Vec::with_capacity(count);
        for i in 0..count {
            let blueprint_ptr = unsafe { *blueprint_array.add(i) };
            if let Ok(blueprint) = ActorBlueprint::from_raw_ptr(blueprint_ptr) {
                blueprints.push(blueprint);
            }
        }
        
        Ok(blueprints)
    }

    pub fn find(&self, key: &str) -> Option<ActorBlueprint> {
        let key_cstring = rust_string_to_c(key).ok()?;
        let blueprint_ptr = unsafe {
            carla_blueprint_library_find(self.inner, key_cstring.as_ptr())
        };
        
        if blueprint_ptr.is_null() {
            None
        } else {
            ActorBlueprint::from_raw_ptr(blueprint_ptr).ok()
        }
    }

    pub fn get(&self, index: usize) -> Option<ActorBlueprint> {
        if index >= self.len() {
            return None;
        }
        
        let blueprint_ptr = unsafe {
            carla_blueprint_library_at(self.inner, index)
        };
        
        if blueprint_ptr.is_null() {
            None
        } else {
            ActorBlueprint::from_raw_ptr(blueprint_ptr).ok()
        }
    }

    pub fn iter(&self) -> impl Iterator<Item = ActorBlueprint> + '_ {
        (0..self.len()).map(|idx| self.get(idx).unwrap())
    }

    pub fn len(&self) -> usize {
        unsafe { carla_blueprint_library_size(self.inner) }
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
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

