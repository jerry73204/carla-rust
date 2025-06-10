use super::Action;
use anyhow::{anyhow, Result};
use carla_sys::*;
use std::{ptr, slice};

#[derive(Clone, Debug)]
pub struct ActionBuffer {
    inner: *mut carla_action_buffer_t,
}

impl ActionBuffer {
    /// Create an ActionBuffer from a raw C pointer.
    /// 
    /// # Safety
    /// The pointer must be valid and not null.
    pub(crate) fn from_raw_ptr(ptr: *mut carla_action_buffer_t) -> Result<Self> {
        if ptr.is_null() {
            return Err(anyhow!("Null action buffer pointer"));
        }
        Ok(Self { inner: ptr })
    }

    pub fn len(&self) -> usize {
        unsafe { carla_action_buffer_size(self.inner) }
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    pub fn get(&self, index: usize) -> Option<Action> {
        if index >= self.len() {
            return None;
        }
        
        let action_ptr = unsafe { carla_action_buffer_at(self.inner, index) };
        if action_ptr.is_null() {
            return None;
        }
        
        // TODO: Implement proper Action conversion once Waypoint is updated to C FFI
        // For now, return None until Waypoint C FFI is implemented
        None
    }

    pub fn iter(&self) -> impl Iterator<Item = Option<Action>> + '_ {
        (0..self.len()).map(move |i| self.get(i))
    }

}

impl Drop for ActionBuffer {
    fn drop(&mut self) {
        // Note: Action buffer data is managed by the traffic manager lifecycle
        // We don't need to free it separately
        self.inner = ptr::null_mut();
    }
}

// SAFETY: ActionBuffer wraps a thread-safe C API
unsafe impl Send for ActionBuffer {}
unsafe impl Sync for ActionBuffer {}
