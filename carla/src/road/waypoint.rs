//! Waypoint navigation and utilities.

use super::{Junction, LaneChange, LaneMarkingColor, LaneMarkingType, LaneType, WaypointList};
use crate::{error::CarlaResult, geom::Transform};

/// A waypoint on the road network.
pub struct Waypoint {
    /// Transform of the waypoint
    pub transform: Transform,
    /// Lane ID
    pub lane_id: i32,
    /// Section ID
    pub section_id: i32,
    /// Road ID
    pub road_id: i32,
    /// Junction ID (negative if not in junction)
    pub junction_id: i32,
    /// Lane width at this waypoint
    pub lane_width: f32,
    /// Lane change permissions
    pub lane_change: LaneChange,
    /// Lane type
    pub lane_type: LaneType,
    /// Lane marking type
    pub lane_marking_type: LaneMarkingType,
    /// Lane marking color
    pub lane_marking_color: LaneMarkingColor,

    /// Internal carla-sys wrapper
    wrapper: carla_sys::map::WaypointWrapper,
}

impl PartialEq for Waypoint {
    fn eq(&self, other: &Self) -> bool {
        self.transform == other.transform
            && self.lane_id == other.lane_id
            && self.section_id == other.section_id
            && self.road_id == other.road_id
            && self.junction_id == other.junction_id
    }
}

// Waypoint cannot be cloned because the C++ Waypoint is NonCopyable
// Users should get new waypoints from Map methods instead

impl std::fmt::Debug for Waypoint {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Waypoint")
            .field("transform", &self.transform)
            .field("lane_id", &self.lane_id)
            .field("section_id", &self.section_id)
            .field("road_id", &self.road_id)
            .field("junction_id", &self.junction_id)
            .field("lane_width", &self.lane_width)
            .field("lane_change", &self.lane_change)
            .field("lane_type", &self.lane_type)
            .field("lane_marking_type", &self.lane_marking_type)
            .field("lane_marking_color", &self.lane_marking_color)
            .finish()
    }
}

impl Waypoint {
    /// Get reference to the internal wrapper (for Map methods)
    pub(crate) fn wrapper(&self) -> &carla_sys::map::WaypointWrapper {
        &self.wrapper
    }

    /// Create a new Waypoint from a carla-sys WaypointWrapper
    pub(crate) fn from_cxx_wrapper(wrapper: carla_sys::map::WaypointWrapper) -> Self {
        let transform = Transform::from(wrapper.get_transform());
        let lane_change = match wrapper.get_lane_change() {
            carla_sys::map::LaneChange::None => LaneChange::None,
            carla_sys::map::LaneChange::Right => LaneChange::Right,
            carla_sys::map::LaneChange::Left => LaneChange::Left,
            carla_sys::map::LaneChange::Both => LaneChange::Both,
        };
        let lane_type = match wrapper.get_type() {
            carla_sys::map::LaneType::Driving => LaneType::Driving,
            carla_sys::map::LaneType::Sidewalk => LaneType::Sidewalk,
            carla_sys::map::LaneType::Shoulder => LaneType::Shoulder,
            carla_sys::map::LaneType::Biking => LaneType::Biking,
            carla_sys::map::LaneType::Stop => LaneType::Stop,
            carla_sys::map::LaneType::Parking => LaneType::Parking,
            _ => LaneType::Any,
        };
        let right_marking = wrapper.get_right_lane_marking();
        let left_marking = wrapper.get_left_lane_marking();

        Self {
            transform,
            lane_id: wrapper.get_lane_id(),
            section_id: wrapper.get_section_id() as i32,
            road_id: wrapper.get_road_id() as i32,
            junction_id: wrapper.get_junction_id() as i32,
            lane_width: wrapper.get_lane_width() as f32,
            lane_change,
            lane_type,
            lane_marking_type: match right_marking.marking_type {
                carla_sys::map::LaneMarkingType::Solid => LaneMarkingType::Solid,
                carla_sys::map::LaneMarkingType::Broken => LaneMarkingType::Broken,
                _ => LaneMarkingType::Other,
            },
            lane_marking_color: match right_marking.color {
                carla_sys::map::LaneMarkingColor::Standard => LaneMarkingColor::Standard,
                carla_sys::map::LaneMarkingColor::Blue => LaneMarkingColor::Blue,
                carla_sys::map::LaneMarkingColor::Green => LaneMarkingColor::Green,
                carla_sys::map::LaneMarkingColor::Red => LaneMarkingColor::Red,
                carla_sys::map::LaneMarkingColor::Yellow => LaneMarkingColor::Yellow,
                _ => LaneMarkingColor::Other,
            },
            wrapper,
        }
    }

    /// Get the next waypoints at the given distance.
    pub fn next(&self, distance: f64) -> CarlaResult<WaypointList> {
        let waypoint_vec = self.wrapper.get_next_vector(distance);
        Ok(WaypointList::new(waypoint_vec))
    }

    /// Get the previous waypoints at the given distance.
    pub fn previous(&self, distance: f64) -> CarlaResult<WaypointList> {
        let waypoint_vec = self.wrapper.get_previous_vector(distance);
        Ok(WaypointList::new(waypoint_vec))
    }

    /// Get the right lane waypoint.
    pub fn right_lane(&self) -> CarlaResult<Option<Waypoint>> {
        Ok(self
            .wrapper
            .get_right()
            .map(|w| Waypoint::from_cxx_wrapper(w)))
    }

    /// Get the left lane waypoint.
    pub fn left_lane(&self) -> CarlaResult<Option<Waypoint>> {
        Ok(self
            .wrapper
            .get_left()
            .map(|w| Waypoint::from_cxx_wrapper(w)))
    }

    /// Get the distance to another waypoint.
    pub fn distance(&self, other: &Waypoint) -> f64 {
        self.transform.location.distance(&other.transform.location)
    }

    /// Check if this waypoint is in a junction.
    pub fn is_junction(&self) -> bool {
        self.junction_id >= 0
    }

    /// Get the junction this waypoint belongs to.
    pub fn junction(&self) -> CarlaResult<Option<Junction>> {
        Ok(self
            .wrapper
            .get_junction()
            .map(|j| Junction::from_cxx_wrapper(j)))
    }
}
