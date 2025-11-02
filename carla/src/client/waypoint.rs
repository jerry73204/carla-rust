// SAFETY: This module uses unwrap_unchecked() for performance on methods guaranteed
// to never return null. See UNWRAP_REPLACEMENTS.md for detailed C++ code audit.

use super::{Junction, LandmarkList, WaypointList};
use crate::{
    geom::Transform,
    road::{
        element::{LaneMarking, LaneMarking_LaneChange},
        JuncId, LaneId, LaneType, RoadId, SectionId,
    },
};
use autocxx::WithinUniquePtr;
use carla_sys::carla_rust::client::FfiWaypoint;
use cxx::SharedPtr;
use derivative::Derivative;
use nalgebra::Isometry3;
use static_assertions::assert_impl_all;

/// A waypoint on a map.
#[derive(Clone, Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct Waypoint {
    #[derivative(Debug = "ignore")]
    pub(crate) inner: SharedPtr<FfiWaypoint>,
}

impl Waypoint {
    pub fn id(&self) -> u64 {
        self.inner.GetId()
    }

    pub fn road_id(&self) -> RoadId {
        self.inner.GetRoadId()
    }

    pub fn section_id(&self) -> SectionId {
        self.inner.GetSectionId()
    }

    pub fn lane_id(&self) -> LaneId {
        self.inner.GetLaneId()
    }

    pub fn distance(&self) -> f64 {
        self.inner.GetDistance()
    }

    pub fn transform(&self) -> Isometry3<f32> {
        Transform::from_ffi(self.inner.GetTransform()).to_na()
    }

    pub fn junction_id(&self) -> JuncId {
        self.inner.GetJunctionId()
    }

    pub fn is_junction(&self) -> bool {
        self.inner.IsJunction()
    }

    pub fn junction(&self) -> Option<Junction> {
        let ptr = self.inner.GetJunction();
        Junction::from_cxx(ptr)
    }

    pub fn lane_width(&self) -> f64 {
        self.inner.GetLaneWidth()
    }

    pub fn type_(&self) -> LaneType {
        self.inner.GetType()
    }

    pub fn next(&self, distance: f64) -> WaypointList {
        let ptr = self.inner.GetNext(distance).within_unique_ptr();
        unsafe { WaypointList::from_cxx(ptr).unwrap_unchecked() }
    }

    pub fn previous(&self, distance: f64) -> WaypointList {
        let ptr = self.inner.GetPrevious(distance).within_unique_ptr();
        unsafe { WaypointList::from_cxx(ptr).unwrap_unchecked() }
    }

    pub fn next_until_lane_end(&self, distance: f64) -> WaypointList {
        let ptr = self.inner.GetNextUntilLaneEnd(distance).within_unique_ptr();
        unsafe { WaypointList::from_cxx(ptr).unwrap_unchecked() }
    }

    pub fn previous_until_lane_start(&self, distance: f64) -> WaypointList {
        let ptr = self
            .inner
            .GetPreviousUntilLaneStart(distance)
            .within_unique_ptr();
        unsafe { WaypointList::from_cxx(ptr).unwrap_unchecked() }
    }

    pub fn left(&self) -> Waypoint {
        let ptr = self.inner.GetLeft();
        unsafe { Self::from_cxx(ptr).unwrap_unchecked() }
    }

    pub fn right(&self) -> Waypoint {
        let ptr = self.inner.GetRight();
        unsafe { Self::from_cxx(ptr).unwrap_unchecked() }
    }

    pub fn right_lane_marking(&self) -> Option<LaneMarking> {
        let ptr = self.inner.GetRightLaneMarking();
        LaneMarking::from_cxx(ptr)
    }

    pub fn left_lane_marking(&self) -> Option<LaneMarking> {
        let ptr = self.inner.GetLeftLaneMarking();
        LaneMarking::from_cxx(ptr)
    }

    pub fn lane_change(&self) -> LaneMarking_LaneChange {
        self.inner.GetLaneChange()
    }

    pub fn all_landmarks_in_distance(&self, distance: f64, stop_at_junction: bool) -> LandmarkList {
        let ptr = self
            .inner
            .GetAllLandmarksInDistance(distance, stop_at_junction)
            .within_unique_ptr();
        unsafe { LandmarkList::from_cxx(ptr).unwrap_unchecked() }
    }

    pub fn landmarks_of_type_in_distance(
        &self,
        distance: f64,
        filter_type: &str,
        stop_at_junction: bool,
    ) -> LandmarkList {
        let ptr = self
            .inner
            .GetLandmarksOfTypeInDistance(distance, filter_type, stop_at_junction)
            .within_unique_ptr();
        unsafe { LandmarkList::from_cxx(ptr).unwrap_unchecked() }
    }

    pub(crate) fn from_cxx(ptr: SharedPtr<FfiWaypoint>) -> Option<Self> {
        if ptr.is_null() {
            None
        } else {
            Some(Self { inner: ptr })
        }
    }
}

assert_impl_all!(Waypoint: Send, Sync);
