use core::slice;

use super::{Junction, Landmark, LandmarkList, Waypoint, WaypointList};
use crate::{
    geom::{Location, LocationExt, Transform, TransformExt},
    road::{LaneId, LaneType, RoadId},
};
use autocxx::WithinUniquePtr;
use carla_sys::carla_rust::client::{FfiMap, FfiTransformList};
use cxx::{SharedPtr, UniquePtr};
use derivative::Derivative;
use nalgebra::{Isometry3, Translation3};
use static_assertions::assert_impl_all;

/// Represents the map of the simulation, corresponding to `carla.Map`
/// in Python API.
#[derive(Clone, Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct Map {
    #[derivative(Debug = "ignore")]
    inner: SharedPtr<FfiMap>,
}

impl Map {
    pub fn name(&self) -> String {
        self.inner.GetName().to_string()
    }

    pub fn to_open_drive(&self) -> String {
        self.inner.GetOpenDrive().to_string()
    }

    pub fn recommended_spawn_points(&self) -> RecommendedSpawnPoints {
        let ptr = self.inner.GetRecommendedSpawnPoints().within_unique_ptr();
        RecommendedSpawnPoints::from_cxx(ptr).unwrap()
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
        let location = Location::from_na_translation(location);
        let ptr = self
            .inner
            .GetWaypoint(&location, project_to_road, lane_type as i32);
        Waypoint::from_cxx(ptr)
    }

    pub fn waypoint_xodr(
        &self,
        road_id: RoadId,
        lane_id: LaneId,
        distance: f32,
    ) -> Option<Waypoint> {
        let ptr = self.inner.GetWaypointXODR(road_id, lane_id, distance);
        Waypoint::from_cxx(ptr)
    }

    pub fn generate_waypoints(&self, distance: f64) -> WaypointList {
        let waypoints = self.inner.GenerateWaypoints(distance);
        WaypointList::from_cxx(waypoints).unwrap()
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

impl Map {
    pub fn from_cxx(ptr: SharedPtr<FfiMap>) -> Option<Self> {
        if ptr.is_null() {
            None
        } else {
            Some(Self { inner: ptr })
        }
    }
}

#[derive(Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct RecommendedSpawnPoints {
    #[derivative(Debug = "ignore")]
    inner: UniquePtr<FfiTransformList>,
}

impl RecommendedSpawnPoints {
    pub fn len(&self) -> usize {
        self.inner.len()
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    pub fn as_slice(&self) -> &[Transform] {
        unsafe { slice::from_raw_parts(self.inner.data(), self.len()) }
    }

    pub fn get(&self, index: usize) -> Option<Isometry3<f32>> {
        Some(self.as_slice().get(index)?.to_na())
    }

    pub fn iter(&self) -> impl Iterator<Item = Isometry3<f32>> + Send + '_ {
        self.as_slice().iter().map(|trans| trans.to_na())
    }

    fn from_cxx(ptr: UniquePtr<FfiTransformList>) -> Option<Self> {
        if ptr.is_null() {
            None
        } else {
            Some(Self { inner: ptr })
        }
    }
}

assert_impl_all!(Map: Send, Sync);
