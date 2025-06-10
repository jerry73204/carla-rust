use anyhow::{anyhow, Result};
use carla_sys::*;
use std::ptr;

pub use carla_sys::carla::road::element::{
    LaneMarking_Color, LaneMarking_LaneChange, LaneMarking_Type,
};

#[derive(Clone, Debug)]
pub struct LaneMarking {
    inner: *mut carla_lane_marking_t,
}

impl LaneMarking {
    /// Create a LaneMarking from a raw C pointer.
    /// 
    /// # Safety
    /// The pointer must be valid and not null.
    pub(crate) fn from_raw_ptr(ptr: *mut carla_lane_marking_t) -> Result<Self> {
        if ptr.is_null() {
            return Err(anyhow!("Null lane marking pointer"));
        }
        Ok(Self { inner: ptr })
    }

    pub fn type_(&self) -> LaneMarking_Type {
        unsafe { carla_lane_marking_get_type(self.inner) }
    }

    pub fn color(&self) -> LaneMarking_Color {
        unsafe { carla_lane_marking_get_color(self.inner) }
    }

    pub fn lane_change(&self) -> LaneMarking_LaneChange {
        unsafe { carla_lane_marking_get_lane_change(self.inner) }
    }

    pub fn width(&self) -> f64 {
        unsafe { carla_lane_marking_get_width(self.inner) }
    }

}

impl Drop for LaneMarking {
    fn drop(&mut self) {
        // Note: Lane marking data is managed by the map lifecycle
        // We don't need to free it separately
        self.inner = ptr::null_mut();
    }
}

// SAFETY: LaneMarking wraps a thread-safe C API
unsafe impl Send for LaneMarking {}
unsafe impl Sync for LaneMarking {}
