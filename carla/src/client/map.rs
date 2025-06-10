use core::slice;

use super::{Junction, Landmark, LandmarkList, Waypoint, WaypointList};
use crate::{
    geom::{Location, LocationExt, Transform, TransformExt},
    road::{LaneId, LaneType, RoadId},
    utils::{check_carla_error, rust_string_to_c, c_string_to_rust, ArrayConversionExt},
};
use anyhow::{anyhow, Result};
use carla_sys::*;
use nalgebra::{Isometry3, Translation3};
use std::ptr;

/// Represents the map of the simulation, corresponding to `carla.Map`
/// in Python API.
#[derive(Debug)]
pub struct Map {
    inner: *mut carla_map_t,
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
        let name = unsafe { c_string_to_rust(name_ptr)? };
        Ok(name)
    }

    pub fn to_open_drive(&self) -> Result<String> {
        // Note: OpenDRIVE content access needs to be implemented in C wrapper
        Err(anyhow!("OpenDRIVE content access not yet implemented in C wrapper"))
    }

    pub fn recommended_spawn_points(&self) -> Result<RecommendedSpawnPoints> {
        let transform_list_ptr = unsafe { carla_map_get_spawn_points(self.inner) };
        if transform_list_ptr.is_null() {
            return Err(anyhow!("Failed to get spawn points"));
        }
        RecommendedSpawnPoints::from_raw_ptr(transform_list_ptr)
    }

    pub fn waypoint(&self, location: &Translation3<f32>) -> Option<Waypoint> {
        self.waypoint_opt(location, true, LaneType::Driving)
    }

    pub fn waypoint_opt(
        &self,
        location: &Translation3<f32>,
        project_to_road: bool,
        lane_type: LaneType,
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
                lane_type as i32,
            )
        };
        
        if waypoint_ptr.is_null() {
            None
        } else {
            Waypoint::from_raw_ptr(waypoint_ptr).ok()
        }
    }

    pub fn waypoint_xodr(
        &self,
        road_id: RoadId,
        lane_id: LaneId,
        distance: f32,
    ) -> Option<Waypoint> {
        let road_id_str = format!("{}", road_id);
        let road_id_cstring = rust_string_to_c(&road_id_str).ok()?;
        
        let waypoint_ptr = unsafe {
            carla_map_get_waypoint_xodr(
                self.inner,
                road_id_cstring.as_ptr(),
                lane_id,
                distance,
            )
        };
        
        if waypoint_ptr.is_null() {
            None
        } else {
            Waypoint::from_raw_ptr(waypoint_ptr).ok()
        }
    }

    pub fn generate_waypoints(&self, distance: f64) -> Result<WaypointList> {
        let waypoint_list_ptr = unsafe { carla_map_generate_waypoints(self.inner, distance) };
        if waypoint_list_ptr.is_null() {
            return Err(anyhow!("Failed to generate waypoints"));
        }
        WaypointList::from_raw_ptr(waypoint_list_ptr)
    }

    pub fn junction(&self, waypoint: &Waypoint) -> Junction {
        let junction = self.inner.GetJunction(&waypoint.inner);
        Junction::from_cxx(junction).unwrap()
    }

    pub fn all_landmarks(&self) -> LandmarkList {
        let ptr = self.inner.GetAllLandmarks().within_unique_ptr();
        LandmarkList::from_cxx(ptr).unwrap()
    }

    pub fn landmarks_from_id(&self, id: &str) -> LandmarkList {
        let ptr = self.inner.GetLandmarksFromId(id).within_unique_ptr();
        LandmarkList::from_cxx(ptr).unwrap()
    }

    pub fn all_landmarks_of_type(&self, type_: &str) -> LandmarkList {
        let ptr = self.inner.GetAllLandmarksOfType(type_).within_unique_ptr();
        LandmarkList::from_cxx(ptr).unwrap()
    }

    pub fn landmark_group(&self, landmark: &Landmark) -> LandmarkList {
        let ptr = self
            .inner
            .GetLandmarkGroup(&landmark.inner)
            .within_unique_ptr();
        LandmarkList::from_cxx(ptr).unwrap()
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

#[derive(Debug)]
pub struct RecommendedSpawnPoints {
    inner: *mut carla_transform_list_t,
}

impl RecommendedSpawnPoints {
    pub(crate) fn from_raw_ptr(ptr: *mut carla_transform_list_t) -> Result<Self> {
        if ptr.is_null() {
            return Err(anyhow!("Null transform list pointer"));
        }
        Ok(Self { inner: ptr })
    }

    pub fn len(&self) -> usize {
        unsafe { carla_transform_list_size(self.inner) }
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    pub fn as_slice(&self) -> Vec<Transform> {
        let len = self.len();
        let mut transforms = Vec::with_capacity(len);
        
        for i in 0..len {
            let c_transform = unsafe { carla_transform_list_get(self.inner, i) };
            let transform = Transform::from_c_transform(c_transform);
            transforms.push(transform);
        }
        
        transforms
    }

    pub fn get(&self, index: usize) -> Option<Isometry3<f32>> {
        if index >= self.len() {
            return None;
        }
        
        let c_transform = unsafe { carla_transform_list_get(self.inner, index) };
        let transform = Transform::from_c_transform(c_transform);
        Some(transform.to_na())
    }

    pub fn iter(&self) -> impl Iterator<Item = Isometry3<f32>> + Send + '_ {
        (0..self.len()).map(move |i| {
            let c_transform = unsafe { carla_transform_list_get(self.inner, i) };
            let transform = Transform::from_c_transform(c_transform);
            transform.to_na()
        })
    }

}

impl Drop for RecommendedSpawnPoints {
    fn drop(&mut self) {
        if !self.inner.is_null() {
            unsafe {
                carla_transform_list_free(self.inner);
            }
            self.inner = ptr::null_mut();
        }
    }
}

