//! Waypoint navigation and utilities.

use super::{Junction, LaneChange, LaneMarkingColor, LaneMarkingType, LaneType};
use crate::{error::CarlaResult, geom::Transform};

/// A waypoint on the road network.
#[derive(Debug, Clone, PartialEq)]
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
}

impl Waypoint {
    /// Get the next waypoints at the given distance.
    pub fn next(&self, distance: f64) -> CarlaResult<Vec<Waypoint>> {
        // TODO: Implement using carla-cxx FFI interface
        let _distance = distance;
        todo!("Waypoint::next not yet implemented with carla-cxx FFI")
    }

    /// Get the previous waypoints at the given distance.
    pub fn previous(&self, distance: f64) -> CarlaResult<Vec<Waypoint>> {
        // TODO: Implement using carla-cxx FFI interface
        let _distance = distance;
        todo!("Waypoint::previous not yet implemented with carla-cxx FFI")
    }

    /// Get the right lane waypoint.
    pub fn right_lane(&self) -> CarlaResult<Option<Waypoint>> {
        // TODO: Implement using carla-cxx FFI interface
        todo!("Waypoint::right_lane not yet implemented with carla-cxx FFI")
    }

    /// Get the left lane waypoint.
    pub fn left_lane(&self) -> CarlaResult<Option<Waypoint>> {
        // TODO: Implement using carla-cxx FFI interface
        todo!("Waypoint::left_lane not yet implemented with carla-cxx FFI")
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
        // TODO: Implement using carla-cxx FFI interface
        todo!("Waypoint::junction not yet implemented with carla-cxx FFI")
    }
}
