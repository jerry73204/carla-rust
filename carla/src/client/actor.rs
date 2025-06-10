use crate::client::{Sensor, Vehicle};
use anyhow::{anyhow, Result};
use carla_sys::*;
use std::ptr;

use super::{ActorBase, ActorKind, TrafficLight, TrafficSign};

/// A base actor that represents a movable object in the simulation,
/// corresponding to `carla.Actor` in Python API.
#[derive(Clone, Debug)]
pub struct Actor {
    pub(crate) inner: *mut carla_actor_t,
}

impl Actor {
    /// Classify the actor into variants.
    pub fn into_kinds(self) -> ActorKind {
        let me = self;
        let me = match Vehicle::try_from(me) {
            Ok(me) => return me.into(),
            Err(me) => me,
        };
        let me = match Sensor::try_from(me) {
            Ok(me) => return me.into(),
            Err(me) => me,
        };
        let me = match TrafficLight::try_from(me) {
            Ok(me) => return me.into(),
            Err(me) => me,
        };
        let me = match TrafficSign::try_from(me) {
            Ok(me) => return me.into(),
            Err(me) => me,
        };

        me.into()
    }

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

