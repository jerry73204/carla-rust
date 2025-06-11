use crate::{
    geom::{Transform, TransformExt},
    utils::c_string_to_rust,
};
use anyhow::{anyhow, Result};
use carla_sys::*;
use nalgebra::{Isometry3, Translation3};
use std::ptr;

/// A map provides the geometry of the world.
#[derive(Debug)]
pub struct Map {
    pub(crate) inner: *mut carla_map_t,
}

impl Map {
    /// Create a Map from a raw C pointer.
    ///
    /// # Safety
    /// The pointer must be valid and not null.
    pub(crate) fn from_raw_ptr(ptr: *mut carla_map_t) -> Result<Self> {
        if ptr.is_null() {
            return Err(anyhow!("Null map pointer"));
        }
        Ok(Self { inner: ptr })
    }

    pub fn name(&self) -> Result<String> {
        let name_ptr = unsafe { carla_map_get_name(self.inner) };
        if name_ptr.is_null() {
            return Err(anyhow!("Failed to get map name"));
        }
        unsafe { c_string_to_rust(name_ptr) }
    }

    pub fn lane_width(&self) -> f64 {
        unsafe { carla_map_get_lane_width(self.inner) }
    }

    pub fn get_spawn_points(&self) -> Result<Vec<Isometry3<f32>>> {
        let transform_list_ptr = unsafe { carla_map_get_spawn_points(self.inner) };
        if transform_list_ptr.is_null() {
            return Err(anyhow!("Failed to get spawn points"));
        }

        let size = unsafe { carla_transform_list_size(transform_list_ptr) };
        let mut spawn_points = Vec::with_capacity(size);

        for i in 0..size {
            let transform = unsafe { carla_transform_list_get(transform_list_ptr, i) };
            spawn_points.push(Transform::from_c_transform(transform).to_na());
        }

        unsafe { carla_transform_list_free(transform_list_ptr as *mut _) };
        Ok(spawn_points)
    }

    pub fn get_waypoint(
        &self,
        location: &Translation3<f32>,
        project_to_road: bool,
    ) -> Option<Waypoint> {
        let c_location = carla_vector3d_t {
            x: location.x,
            y: location.y,
            z: location.z,
        };

        let waypoint_ptr = unsafe {
            carla_map_get_waypoint(
                self.inner,
                &c_location,
                project_to_road,
                -2, // CARLA_LANE_TYPE_ANY
            )
        };

        if waypoint_ptr.is_null() {
            None
        } else {
            Waypoint::from_raw_ptr(waypoint_ptr).ok()
        }
    }

    pub fn generate_waypoints(&self, distance: f64) -> Result<Vec<Waypoint>> {
        let waypoint_list_ptr = unsafe { carla_map_generate_waypoints(self.inner, distance) };
        if waypoint_list_ptr.is_null() {
            return Err(anyhow!("Failed to generate waypoints"));
        }

        let size = unsafe { carla_waypoint_list_size(waypoint_list_ptr) };
        let mut waypoints = Vec::with_capacity(size);

        for i in 0..size {
            let waypoint_ptr = unsafe { carla_waypoint_list_get(waypoint_list_ptr, i) };
            if !waypoint_ptr.is_null() {
                if let Ok(waypoint) = Waypoint::from_raw_ptr(waypoint_ptr) {
                    waypoints.push(waypoint);
                }
            }
        }

        unsafe { carla_waypoint_list_free(waypoint_list_ptr as *mut _) };
        Ok(waypoints)
    }
}

impl Drop for Map {
    fn drop(&mut self) {
        if !self.inner.is_null() {
            unsafe {
                carla_map_free(self.inner);
            }
            self.inner = ptr::null_mut();
        }
    }
}

// SAFETY: Map wraps a thread-safe C API
unsafe impl Send for Map {}
unsafe impl Sync for Map {}

/// A waypoint is a 3D point on the map with extra information about the road.
#[derive(Debug)]
pub struct Waypoint {
    pub(crate) inner: *mut carla_waypoint_t,
}

impl Waypoint {
    /// Create a Waypoint from a raw C pointer.
    ///
    /// # Safety
    /// The pointer must be valid and not null.
    pub(crate) fn from_raw_ptr(ptr: *mut carla_waypoint_t) -> Result<Self> {
        if ptr.is_null() {
            return Err(anyhow!("Null waypoint pointer"));
        }
        Ok(Self { inner: ptr })
    }

    pub fn id(&self) -> u64 {
        unsafe { carla_waypoint_get_id(self.inner) }
    }

    pub fn road_id(&self) -> u32 {
        unsafe { carla_waypoint_get_road_id(self.inner) }
    }

    pub fn section_id(&self) -> u32 {
        unsafe { carla_waypoint_get_section_id(self.inner) }
    }

    pub fn lane_id(&self) -> i32 {
        unsafe { carla_waypoint_get_lane_id(self.inner) }
    }

    pub fn s(&self) -> f64 {
        unsafe { carla_waypoint_get_s(self.inner) }
    }

    pub fn is_junction(&self) -> bool {
        unsafe { carla_waypoint_is_junction(self.inner) }
    }

    pub fn lane_width(&self) -> f64 {
        unsafe { carla_waypoint_get_lane_width(self.inner) }
    }

    pub fn lane_change(&self) -> i32 {
        unsafe { carla_waypoint_get_lane_change(self.inner) }
    }

    pub fn lane_type(&self) -> i32 {
        unsafe { carla_waypoint_get_lane_type(self.inner) }
    }

    pub fn transform(&self) -> Isometry3<f32> {
        let c_transform = unsafe { carla_waypoint_get_transform(self.inner) };
        Transform::from_c_transform(c_transform).to_na()
    }

    pub fn next(&self, distance: f64) -> Option<Waypoint> {
        let waypoint_ptr = unsafe { carla_waypoint_get_next(self.inner, distance) };
        if waypoint_ptr.is_null() {
            None
        } else {
            Waypoint::from_raw_ptr(waypoint_ptr).ok()
        }
    }

    pub fn previous(&self, distance: f64) -> Option<Waypoint> {
        let waypoint_ptr = unsafe { carla_waypoint_get_previous(self.inner, distance) };
        if waypoint_ptr.is_null() {
            None
        } else {
            Waypoint::from_raw_ptr(waypoint_ptr).ok()
        }
    }

    pub fn get_right(&self) -> Option<Waypoint> {
        let waypoint_ptr = unsafe { carla_waypoint_get_right(self.inner) };
        if waypoint_ptr.is_null() {
            None
        } else {
            Waypoint::from_raw_ptr(waypoint_ptr).ok()
        }
    }

    pub fn get_left(&self) -> Option<Waypoint> {
        let waypoint_ptr = unsafe { carla_waypoint_get_left(self.inner) };
        if waypoint_ptr.is_null() {
            None
        } else {
            Waypoint::from_raw_ptr(waypoint_ptr).ok()
        }
    }
}

impl Drop for Waypoint {
    fn drop(&mut self) {
        if !self.inner.is_null() {
            unsafe {
                carla_waypoint_free(self.inner);
            }
            self.inner = ptr::null_mut();
        }
    }
}

// SAFETY: Waypoint wraps a thread-safe C API
unsafe impl Send for Waypoint {}
unsafe impl Sync for Waypoint {}
