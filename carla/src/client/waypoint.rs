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

/// A waypoint on a CARLA map representing a specific position on a road lane.
///
///  [`Waypoint`] is fundamental for navigation and path planning in CARLA. Each waypoint:
/// - Represents a point on a specific lane with position and orientation
/// - Provides methods to navigate forward/backward along lanes
/// - Allows lane changes and junction traversal
/// - Gives access to lane properties (width, type, markings)
/// - Can find nearby landmarks (traffic signs, speed limits, etc.)
///
/// Corresponds to [`carla.Waypoint`](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Waypoint) in the Python API.
///
/// # Examples
///
/// ```no_run
/// use carla::client::Client;
///
/// let client = Client::default();
/// let world = client.world();
/// let map = world.map();
///
/// // Get a waypoint at a specific location
/// let location = carla::geom::Location::new(10.0, 20.0, 0.3);
/// if let Some(waypoint) = map.waypoint_at(&location) {
///     println!("Lane ID: {}", waypoint.lane_id());
///     println!("Lane width: {}m", waypoint.lane_width());
///
///     // Navigate forward
///     let next_waypoints = waypoint.next(2.0); // 2 meters ahead
///
///     // Change lanes
///     if let Some(left_lane) = waypoint.left() {
///         println!("Can change to left lane");
///     }
/// }
/// ```
#[derive(Clone, Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct Waypoint {
    #[derivative(Debug = "ignore")]
    pub(crate) inner: SharedPtr<FfiWaypoint>,
}

impl Waypoint {
    /// Returns the unique identifier of this waypoint.
    ///
    /// See [carla.Waypoint.id](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Waypoint.id)
    /// in the Python API.
    pub fn id(&self) -> u64 {
        self.inner.GetId()
    }

    /// Returns the OpenDRIVE road ID.
    ///
    /// See [carla.Waypoint.road_id](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Waypoint.road_id)
    /// in the Python API.
    pub fn road_id(&self) -> RoadId {
        self.inner.GetRoadId()
    }

    /// Returns the OpenDRIVE section ID.
    ///
    /// See [carla.Waypoint.section_id](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Waypoint.section_id)
    /// in the Python API.
    pub fn section_id(&self) -> SectionId {
        self.inner.GetSectionId()
    }

    /// Returns the OpenDRIVE lane ID.
    ///
    /// See [carla.Waypoint.lane_id](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Waypoint.lane_id)
    /// in the Python API.
    pub fn lane_id(&self) -> LaneId {
        self.inner.GetLaneId()
    }

    /// Returns the distance from the beginning of the road to this waypoint (in meters).
    ///
    /// This is the "s" coordinate in the OpenDRIVE specification.
    ///
    /// See [carla.Waypoint.s](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Waypoint.s)
    /// in the Python API.
    pub fn distance(&self) -> f64 {
        self.inner.GetDistance()
    }

    /// Returns the transform (position and rotation) of this waypoint.
    ///
    /// See [carla.Waypoint.transform](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Waypoint.transform)
    /// in the Python API.
    pub fn transform(&self) -> Transform {
        Transform::from_ffi(self.inner.GetTransform())
    }

    /// Returns the OpenDRIVE junction ID (or -1 if not in a junction).
    ///
    /// See [carla.Waypoint.junction_id](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Waypoint.junction_id)
    /// in the Python API.
    pub fn junction_id(&self) -> JuncId {
        self.inner.GetJunctionId()
    }

    /// Returns `true` if this waypoint is inside a junction.
    ///
    /// See [carla.Waypoint.is_junction](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Waypoint.is_junction)
    /// in the Python API.
    pub fn is_junction(&self) -> bool {
        self.inner.IsJunction()
    }

    /// Returns the junction this waypoint belongs to (if any).
    ///
    /// See [carla.Waypoint.get_junction](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Waypoint.get_junction)
    /// in the Python API.
    pub fn junction(&self) -> Option<Junction> {
        let ptr = self.inner.GetJunction();
        Junction::from_cxx(ptr)
    }

    /// Returns the width of the lane in meters.
    ///
    /// See [carla.Waypoint.lane_width](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Waypoint.lane_width)
    /// in the Python API.
    pub fn lane_width(&self) -> f64 {
        self.inner.GetLaneWidth()
    }

    /// Returns the lane type (Driving, Sidewalk, Shoulder, etc.).
    ///
    /// See [carla.Waypoint.lane_type](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Waypoint.lane_type)
    /// in the Python API.
    pub fn type_(&self) -> LaneType {
        self.inner.GetType()
    }

    /// Returns waypoints at a specified distance in the forward direction.
    ///
    /// Returns all possible waypoints at the given distance (e.g., branching at junctions).
    ///
    /// See [carla.Waypoint.next](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Waypoint.next)
    /// in the Python API.
    ///
    /// # Arguments
    /// * `distance` - Distance in meters to the next waypoint(s)
    pub fn next(&self, distance: f64) -> WaypointList {
        let ptr = self.inner.GetNext(distance).within_unique_ptr();
        unsafe { WaypointList::from_cxx(ptr).unwrap_unchecked() }
    }

    /// Returns waypoints at a specified distance in the backward direction.
    ///
    /// Returns all possible waypoints at the given distance (e.g., branching at junctions).
    ///
    /// See [carla.Waypoint.previous](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Waypoint.previous)
    /// in the Python API.
    ///
    /// # Arguments
    /// * `distance` - Distance in meters to the previous waypoint(s)
    pub fn previous(&self, distance: f64) -> WaypointList {
        let ptr = self.inner.GetPrevious(distance).within_unique_ptr();
        unsafe { WaypointList::from_cxx(ptr).unwrap_unchecked() }
    }

    /// Returns waypoints at a specified distance ahead until reaching the end of the lane.
    ///
    /// See [carla.Waypoint.next_until_lane_end](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Waypoint.next_until_lane_end)
    /// in the Python API.
    ///
    /// # Arguments
    /// * `distance` - Distance between waypoints in meters
    pub fn next_until_lane_end(&self, distance: f64) -> WaypointList {
        let ptr = self.inner.GetNextUntilLaneEnd(distance).within_unique_ptr();
        unsafe { WaypointList::from_cxx(ptr).unwrap_unchecked() }
    }

    /// Returns waypoints at a specified distance backward until reaching the start of the lane.
    ///
    /// See [carla.Waypoint.previous_until_lane_start](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Waypoint.previous_until_lane_start)
    /// in the Python API.
    ///
    /// # Arguments
    /// * `distance` - Distance between waypoints in meters
    pub fn previous_until_lane_start(&self, distance: f64) -> WaypointList {
        let ptr = self
            .inner
            .GetPreviousUntilLaneStart(distance)
            .within_unique_ptr();
        unsafe { WaypointList::from_cxx(ptr).unwrap_unchecked() }
    }

    /// Returns the waypoint to the left lane, if available.
    ///
    /// Checks if a lane change to the left is possible based on lane markings.
    ///
    /// See [carla.Waypoint.get_left_lane](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Waypoint.get_left_lane)
    /// in the Python API.
    pub fn left(&self) -> Option<Waypoint> {
        let ptr = self.inner.GetLeft();
        Self::from_cxx(ptr)
    }

    /// Returns the waypoint to the right lane, if available.
    ///
    /// Checks if a lane change to the right is possible based on lane markings.
    ///
    /// See [carla.Waypoint.get_right_lane](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Waypoint.get_right_lane)
    /// in the Python API.
    pub fn right(&self) -> Option<Waypoint> {
        let ptr = self.inner.GetRight();
        Self::from_cxx(ptr)
    }

    /// Returns the lane marking on the right side of the lane.
    ///
    /// See [carla.Waypoint.right_lane_marking](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Waypoint.right_lane_marking)
    /// in the Python API.
    pub fn right_lane_marking(&self) -> Option<LaneMarking> {
        let ptr = self.inner.GetRightLaneMarking();
        LaneMarking::from_cxx(ptr)
    }

    /// Returns the lane marking on the left side of the lane.
    ///
    /// See [carla.Waypoint.left_lane_marking](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Waypoint.left_lane_marking)
    /// in the Python API.
    pub fn left_lane_marking(&self) -> Option<LaneMarking> {
        let ptr = self.inner.GetLeftLaneMarking();
        LaneMarking::from_cxx(ptr)
    }

    /// Returns the lane change permission for this waypoint.
    ///
    /// Indicates which lane changes are allowed (left, right, both, or none).
    ///
    /// See [carla.Waypoint.lane_change](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Waypoint.lane_change)
    /// in the Python API.
    pub fn lane_change(&self) -> LaneMarking_LaneChange {
        self.inner.GetLaneChange()
    }

    /// Returns all landmarks within a specified distance from this waypoint.
    ///
    /// # Arguments
    /// * `distance` - Maximum search distance in meters
    /// * `stop_at_junction` - If true, stops searching when reaching a junction
    ///
    /// # Returns
    /// A list of landmarks (traffic signs, speed limits, etc.) within the specified distance.
    pub fn all_landmarks_in_distance(&self, distance: f64, stop_at_junction: bool) -> LandmarkList {
        let ptr = self
            .inner
            .GetAllLandmarksInDistance(distance, stop_at_junction)
            .within_unique_ptr();
        unsafe { LandmarkList::from_cxx(ptr).unwrap_unchecked() }
    }

    /// Returns landmarks of a specific type within a specified distance.
    ///
    /// # Arguments
    /// * `distance` - Maximum search distance in meters
    /// * `filter_type` - Landmark type to filter (e.g., "1000001" for speed limits)
    /// * `stop_at_junction` - If true, stops searching when reaching a junction
    ///
    /// # Returns
    /// A list of landmarks matching the specified type within the distance.
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
    /// See [carla.Waypoint.get_landmarks](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Waypoint.get_landmarks)
    /// in the Python API.
    ///
    /// # Arguments
    /// * `distance` - Maximum search distance in meters
    /// * `stop_at_junction` - If true, stops searching at junctions
    #[inline]
    pub fn get_landmarks(&self, distance: f64, stop_at_junction: bool) -> LandmarkList {
        self.all_landmarks_in_distance(distance, stop_at_junction)
    }

    /// Python-compatible alias for `landmarks_of_type_in_distance()`.
    ///
    /// Returns landmarks of a specific type from the current waypoint until the specified distance.
    ///
    /// See [carla.Waypoint.get_landmarks_of_type](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Waypoint.get_landmarks_of_type)
    /// in the Python API.
    ///
    /// # Arguments
    /// * `distance` - Maximum search distance in meters
    /// * `type_` - Landmark type to filter (e.g., "1000001" for speed limits)
    /// * `stop_at_junction` - If true, stops searching at junctions
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
