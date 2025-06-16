//! Junction utilities.

use super::{LaneType, Waypoint};
use crate::error::CarlaResult;

/// Road junction.
#[derive(Debug, Clone, PartialEq)]
pub struct Junction {
    /// Junction ID
    pub id: i32,
    /// Junction bounding box
    pub bounding_box: crate::geom::BoundingBox,
}

impl Junction {
    /// Get waypoints in this junction.
    pub fn waypoints(&self, lane_type: LaneType) -> CarlaResult<Vec<Waypoint>> {
        // TODO: Implement using carla-cxx FFI interface
        let _lane_type = lane_type;
        todo!("Junction::waypoints not yet implemented with carla-cxx FFI")
    }
}
