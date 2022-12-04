use crate::{
    geom::{Location, LocationExt, TransformExt},
    road::{LaneId, LaneType, RoadId},
};
use carla_sys::carla_rust::client::FfiMap;
use cxx::SharedPtr;
use derivative::Derivative;
use nalgebra::{Isometry3, Translation3};

use super::Waypoint;

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

    pub fn recommended_spawn_points(&self) -> Vec<Isometry3<f32>> {
        let pts = self.inner.GetRecommendedSpawnPoints();
        pts.iter().map(|trans| trans.to_na()).collect()
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
        let location = Location::from_na(location);
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
