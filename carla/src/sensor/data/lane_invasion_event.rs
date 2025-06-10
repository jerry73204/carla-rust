use crate::{client::Actor, road::element::LaneMarking, sensor::{SensorData, SensorDataBase}};
use anyhow::{anyhow, Result};
use carla_sys::*;
use std::ptr;

#[derive(Clone, Debug)]
pub struct LaneInvasionEvent {
    inner: *mut carla_lane_invasion_event_data_t,
}

impl LaneInvasionEvent {
    /// Create a LaneInvasionEvent from a raw C pointer.
    /// 
    /// # Safety
    /// The pointer must be valid and not null.
    pub(crate) fn from_raw_ptr(ptr: *mut carla_lane_invasion_event_data_t) -> Result<Self> {
        if ptr.is_null() {
            return Err(anyhow!("Null lane invasion event data pointer"));
        }
        Ok(Self { inner: ptr })
    }

    pub fn actor(&self) -> Result<Actor> {
        let actor_ptr = unsafe { carla_lane_invasion_event_get_actor(self.inner) };
        Actor::from_raw_ptr(actor_ptr)
    }

    pub fn crossed_lane_markings(&self) -> Vec<LaneMarking> {
        let mut count = 0;
        let markings_array = unsafe {
            carla_lane_invasion_event_get_crossed_lane_markings(self.inner, &mut count)
        };
        
        if markings_array.is_null() {
            return Vec::new();
        }
        
        let mut markings = Vec::with_capacity(count);
        for i in 0..count {
            let marking_ptr = unsafe { *markings_array.add(i) };
            if let Ok(marking) = LaneMarking::from_raw_ptr(marking_ptr) {
                markings.push(marking);
            }
        }
        markings
    }

}

impl SensorDataBase for LaneInvasionEvent {
    fn raw_ptr(&self) -> *mut carla_sensor_data_t {
        self.inner as *mut carla_sensor_data_t
    }
}

impl TryFrom<SensorData> for LaneInvasionEvent {
    type Error = SensorData;

    fn try_from(value: SensorData) -> std::result::Result<Self, Self::Error> {
        // Check if the sensor data is actually lane invasion event data
        let data_type = value.data_type();
        if data_type == carla_sensor_data_type_t_CARLA_SENSOR_DATA_LANE_INVASION {
            let lane_invasion_ptr = unsafe { carla_sensor_data_as_lane_invasion_event(value.inner) };
            if !lane_invasion_ptr.is_null() {
                return Ok(LaneInvasionEvent { inner: lane_invasion_ptr });
            }
        }
        Err(value)
    }
}

impl Drop for LaneInvasionEvent {
    fn drop(&mut self) {
        // Note: Lane invasion event data is managed by the sensor data lifecycle
        // We don't need to free it separately
        self.inner = ptr::null_mut();
    }
}

// SAFETY: LaneInvasionEvent wraps a thread-safe C API
unsafe impl Send for LaneInvasionEvent {}
unsafe impl Sync for LaneInvasionEvent {}
