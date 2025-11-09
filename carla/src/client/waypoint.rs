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

    /// Returns the transform (position and rotation) of this waypoint.
    pub fn transform(&self) -> Transform {
        Transform::from_ffi(self.inner.GetTransform())
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

    pub fn left(&self) -> Option<Waypoint> {
        let ptr = self.inner.GetLeft();
        Self::from_cxx(ptr)
    }

    pub fn right(&self) -> Option<Waypoint> {
        let ptr = self.inner.GetRight();
        Self::from_cxx(ptr)
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

    /// Python-compatible alias for `all_landmarks_in_distance()`.
    ///
    /// Returns all landmarks from the current waypoint until the specified distance.
    ///
    /// # Arguments
    /// * `distance` - Maximum search distance in meters
    /// * `stop_at_junction` - If true, stops searching at junctions
    ///
    /// # Python API Equivalent
    /// ```python
    /// landmarks = waypoint.get_landmarks(distance, stop_at_junction=False)
    /// ```
    #[inline]
    pub fn get_landmarks(&self, distance: f64, stop_at_junction: bool) -> LandmarkList {
        self.all_landmarks_in_distance(distance, stop_at_junction)
    }

    /// Python-compatible alias for `landmarks_of_type_in_distance()`.
    ///
    /// Returns landmarks of a specific type from the current waypoint until the specified distance.
    ///
    /// # Arguments
    /// * `distance` - Maximum search distance in meters
    /// * `type_` - Landmark type to filter (e.g., "1000001" for speed limits)
    /// * `stop_at_junction` - If true, stops searching at junctions
    ///
    /// # Python API Equivalent
    /// ```python
    /// landmarks = waypoint.get_landmarks_of_type(distance, type, stop_at_junction=False)
    /// ```
    #[inline]
    pub fn get_landmarks_of_type(
        &self,
        distance: f64,
        type_: &str,
        stop_at_junction: bool,
    ) -> LandmarkList {
        self.landmarks_of_type_in_distance(distance, type_, stop_at_junction)
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
