use super::RoadOption;
use crate::client::Waypoint;
use anyhow::{anyhow, Result};
use carla_sys::*;
use std::ptr;

pub type Action = (RoadOption, Waypoint);

#[derive(Clone, Debug)]
pub(crate) struct PrivateAction {
    inner: *mut carla_action_t,
}

impl PrivateAction {
    pub(crate) fn from_raw_ptr(ptr: *mut carla_action_t) -> Result<Self> {
        if ptr.is_null() {
            return Err(anyhow!("Null action pointer"));
        }
        Ok(Self { inner: ptr })
    }

    pub fn to_pair(&self) -> Action {
        let road_option = unsafe { carla_action_get_road_option(self.inner) };
        let waypoint_ptr = unsafe { carla_action_get_waypoint(self.inner) };
        
        let waypoint = if waypoint_ptr.is_null() {
            // TODO: Update Waypoint to use C FFI and implement proper default
            todo!("Waypoint C FFI conversion not yet implemented")
        } else {
            // TODO: Update Waypoint to use C FFI
            todo!("Waypoint C FFI conversion not yet implemented")
        };
        
        (road_option, waypoint)
    }
}

impl Drop for PrivateAction {
    fn drop(&mut self) {
        // Note: Action data is managed by the traffic manager lifecycle
        // We don't need to free it separately
        self.inner = ptr::null_mut();
    }
}
