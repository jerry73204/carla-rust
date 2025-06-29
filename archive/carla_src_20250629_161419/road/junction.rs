//! Junction utilities.

use super::{LaneType, WaypointList};
use crate::{
    error::CarlaResult,
    geom::{BoundingBox, Location, Vector3D},
};

/// Road junction.
pub struct Junction {
    /// Internal junction wrapper
    wrapper: carla_sys::map::JunctionWrapper,
}

impl std::fmt::Debug for Junction {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Junction")
            .field("id", &self.id())
            .field("bounding_box", &self.bounding_box())
            .finish()
    }
}

impl Junction {
    /// Create a new Junction from a carla-sys JunctionWrapper
    pub(crate) fn from_cxx_wrapper(wrapper: carla_sys::map::JunctionWrapper) -> Self {
        Self { wrapper }
    }

    /// Get the junction ID.
    pub fn id(&self) -> i32 {
        self.wrapper.get_id() as i32
    }

    /// Get the junction bounding box.
    pub fn bounding_box(&self) -> BoundingBox {
        let bbox = self.wrapper.get_bounding_box();
        BoundingBox {
            location: Location {
                x: bbox.location.x,
                y: bbox.location.y,
                z: bbox.location.z,
            },
            extent: Vector3D {
                x: bbox.extent.x as f32,
                y: bbox.extent.y as f32,
                z: bbox.extent.z as f32,
            },
        }
    }

    /// Get waypoints in this junction.
    pub fn waypoints(&self, lane_type: LaneType) -> CarlaResult<WaypointList> {
        let cxx_lane_type = match lane_type {
            LaneType::Driving => carla_sys::map::LaneType::Driving,
            LaneType::Sidewalk => carla_sys::map::LaneType::Sidewalk,
            LaneType::Shoulder => carla_sys::map::LaneType::Shoulder,
            LaneType::Biking => carla_sys::map::LaneType::Biking,
            LaneType::Stop => carla_sys::map::LaneType::Stop,
            LaneType::Parking => carla_sys::map::LaneType::Parking,
            _ => carla_sys::map::LaneType::Any,
        };
        let waypoint_vec = self.wrapper.get_waypoints(cxx_lane_type);
        Ok(WaypointList::new(waypoint_vec))
    }
}
