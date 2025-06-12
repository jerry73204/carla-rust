use crate::{
    client::Actor,
    sensor::{SensorData, SensorDataBase},
};
use anyhow::{anyhow, Result};
use carla_sys::*;

use crate::stubs::{
    carla_obstacle_detection_event_data_t, carla_obstacle_detection_event_get_actor,
    carla_obstacle_detection_event_get_distance, carla_obstacle_detection_event_get_other_actor,
};
use std::ptr;

#[derive(Clone, Debug)]
pub struct ObstacleDetectionEvent {
    inner: *mut carla_obstacle_detection_event_data_t,
}

impl ObstacleDetectionEvent {
    /// Create an ObstacleDetectionEvent from a raw C pointer.
    ///
    /// # Safety
    /// The pointer must be valid and not null.
    pub(crate) fn from_raw_ptr(ptr: *mut carla_obstacle_detection_event_data_t) -> Result<Self> {
        if ptr.is_null() {
            return Err(anyhow!("Null obstacle detection event data pointer"));
        }
        Ok(Self { inner: ptr })
    }

    pub fn actor(&self) -> Result<Actor> {
        let actor_ptr = unsafe { carla_obstacle_detection_event_get_actor(self.inner) };
        if actor_ptr.is_null() {
            Err(anyhow!("No actor in obstacle detection event"))
        } else {
            Actor::from_raw_ptr(actor_ptr)
        }
    }

    pub fn other_actor(&self) -> Result<Actor> {
        let other_actor_ptr = unsafe { carla_obstacle_detection_event_get_other_actor(self.inner) };
        if other_actor_ptr.is_null() {
            Err(anyhow!("No other actor in obstacle detection event"))
        } else {
            Actor::from_raw_ptr(other_actor_ptr)
        }
    }

    pub fn distance(&self) -> f32 {
        unsafe { carla_obstacle_detection_event_get_distance(self.inner) as f32 }
    }
}

impl SensorDataBase for ObstacleDetectionEvent {
    fn raw_ptr(&self) -> *mut carla_sensor_data_t {
        self.inner as *mut carla_sensor_data_t
    }
}

impl TryFrom<SensorData> for ObstacleDetectionEvent {
    type Error = SensorData;

    fn try_from(value: SensorData) -> std::result::Result<Self, Self::Error> {
        // TODO: Implement when carla_sensor_data_as_obstacle_detection_event is available
        Err(value)
    }
}

impl Drop for ObstacleDetectionEvent {
    fn drop(&mut self) {
        // Note: Obstacle detection event data is managed by the sensor data lifecycle
        // We don't need to free it separately
        self.inner = ptr::null_mut();
    }
}

// SAFETY: ObstacleDetectionEvent wraps a thread-safe C API
unsafe impl Send for ObstacleDetectionEvent {}
unsafe impl Sync for ObstacleDetectionEvent {}
