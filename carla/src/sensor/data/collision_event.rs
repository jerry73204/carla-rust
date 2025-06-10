use crate::{client::Actor, sensor::{SensorData, SensorDataBase}};
use anyhow::{anyhow, Result};
use carla_sys::*;
use nalgebra::Vector3;
use std::ptr;

#[derive(Clone, Debug)]
pub struct CollisionEvent {
    inner: *mut carla_collision_event_data_t,
}

impl CollisionEvent {
    /// Create a CollisionEvent from a raw C pointer.
    /// 
    /// # Safety
    /// The pointer must be valid and not null.
    pub(crate) fn from_raw_ptr(ptr: *mut carla_collision_event_data_t) -> Result<Self> {
        if ptr.is_null() {
            return Err(anyhow!("Null collision event data pointer"));
        }
        Ok(Self { inner: ptr })
    }

    pub fn actor(&self) -> Result<Actor> {
        let actor_ptr = unsafe { carla_collision_event_get_actor(self.inner) };
        Actor::from_raw_ptr(actor_ptr)
    }

    pub fn other_actor(&self) -> Option<Actor> {
        let other_actor_ptr = unsafe { carla_collision_event_get_other_actor(self.inner) };
        if other_actor_ptr.is_null() {
            None
        } else {
            Actor::from_raw_ptr(other_actor_ptr).ok()
        }
    }

    pub fn normal_impulse(&self) -> Vector3<f32> {
        let impulse = unsafe { carla_collision_event_get_normal_impulse(self.inner) };
        Vector3::new(impulse.x, impulse.y, impulse.z)
    }

}

impl SensorDataBase for CollisionEvent {
    fn raw_ptr(&self) -> *mut carla_sensor_data_t {
        self.inner as *mut carla_sensor_data_t
    }
}

impl TryFrom<SensorData> for CollisionEvent {
    type Error = SensorData;

    fn try_from(value: SensorData) -> std::result::Result<Self, Self::Error> {
        // Check if the sensor data is actually collision event data
        let data_type = value.data_type();
        if data_type == carla_sensor_data_type_t_CARLA_SENSOR_DATA_COLLISION {
            let collision_ptr = unsafe { carla_sensor_data_as_collision_event(value.inner) };
            if !collision_ptr.is_null() {
                return Ok(CollisionEvent { inner: collision_ptr });
            }
        }
        Err(value)
    }
}

impl Drop for CollisionEvent {
    fn drop(&mut self) {
        // Note: Collision event data is managed by the sensor data lifecycle
        // We don't need to free it separately
        self.inner = ptr::null_mut();
    }
}

// SAFETY: CollisionEvent wraps a thread-safe C API
unsafe impl Send for CollisionEvent {}
unsafe impl Sync for CollisionEvent {}
