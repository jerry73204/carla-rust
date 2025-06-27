//! Waypoint navigation and utilities.

use super::{
    Junction, Landmark, LaneChange, LaneMarkingColor, LaneMarkingType, LaneType, WaypointList,
};
use crate::{error::CarlaResult, geom::Transform};
// TODO: Re-enable when SimpleLandmark is available
// use carla_sys::ffi::SimpleLandmark;

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

impl Clone for Waypoint {
    fn clone(&self) -> Self {
        // Create a new waypoint with the same data but reset wrapper to zeroed memory
        // This is safe for cloning since we copy all the extracted field values
        Self {
            transform: self.transform,
            lane_id: self.lane_id,
            section_id: self.section_id,
            road_id: self.road_id,
            junction_id: self.junction_id,
            lane_width: self.lane_width,
            lane_change: self.lane_change,
            lane_type: self.lane_type,
            lane_marking_type: self.lane_marking_type,
            lane_marking_color: self.lane_marking_color,
            wrapper: unsafe {
                // Cloned waypoint gets zeroed wrapper - should be regenerated from map if needed
                std::mem::zeroed()
            },
        }
    }
}

// Note: Waypoint is cloneable but users should prefer getting fresh waypoints
// from Map methods when possible to ensure consistency with the C++ layer

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
        let _left_marking = wrapper.get_left_lane_marking();

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
        Ok(self.wrapper.get_right().map(Waypoint::from_cxx_wrapper))
    }

    /// Get the left lane waypoint.
    pub fn left_lane(&self) -> CarlaResult<Option<Waypoint>> {
        Ok(self.wrapper.get_left().map(Waypoint::from_cxx_wrapper))
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
        Ok(self.wrapper.get_junction().map(Junction::from_cxx_wrapper))
    }

    /// Get all landmarks within a distance from this waypoint.
    pub fn landmarks(&self, distance: f64) -> CarlaResult<Vec<Landmark>> {
        let landmarks = self.wrapper.get_all_landmarks_in_distance(distance, true);
        Ok(landmarks.into_iter().map(Landmark::from).collect())
    }

    /// Get landmarks of a specific type within a distance from this waypoint.
    pub fn landmarks_of_type(
        &self,
        landmark_type: &str,
        distance: f64,
    ) -> CarlaResult<Vec<Landmark>> {
        let landmarks =
            self.wrapper
                .get_landmarks_of_type_in_distance(distance, landmark_type, true);
        Ok(landmarks.into_iter().map(Landmark::from).collect())
    }

    /// Get reference to internal wrapper for FFI operations
    pub(crate) fn inner(&self) -> &carla_sys::map::WaypointWrapper {
        &self.wrapper
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geom::{Location, Rotation, Transform};

    fn create_test_waypoint() -> Waypoint {
        // Note: In real tests, this would use proper mock data
        Waypoint {
            transform: Transform {
                location: Location {
                    x: 10.0,
                    y: 20.0,
                    z: 0.5,
                },
                rotation: Rotation {
                    pitch: 0.0,
                    yaw: 90.0,
                    roll: 0.0,
                },
            },
            lane_id: 1,
            section_id: 0,
            road_id: 42,
            junction_id: -1,
            lane_width: 3.5,
            lane_change: LaneChange::Both,
            lane_type: LaneType::Driving,
            lane_marking_type: LaneMarkingType::Solid,
            lane_marking_color: LaneMarkingColor::Standard,
            wrapper: unsafe {
                // This is a test-only placeholder - in real tests we'd need proper mocking
                std::mem::zeroed()
            },
        }
    }

    #[test]
    fn test_waypoint_fields() {
        let waypoint = create_test_waypoint();

        assert_eq!(waypoint.lane_id, 1);
        assert_eq!(waypoint.section_id, 0);
        assert_eq!(waypoint.road_id, 42);
        assert_eq!(waypoint.junction_id, -1);
        assert_eq!(waypoint.lane_width, 3.5);
        assert_eq!(waypoint.lane_change, LaneChange::Both);
        assert_eq!(waypoint.lane_type, LaneType::Driving);
    }

    #[test]
    fn test_waypoint_transform() {
        let waypoint = create_test_waypoint();

        assert_eq!(waypoint.transform.location.x, 10.0);
        assert_eq!(waypoint.transform.location.y, 20.0);
        assert_eq!(waypoint.transform.location.z, 0.5);
        assert_eq!(waypoint.transform.rotation.yaw, 90.0);
    }

    #[test]
    fn test_waypoint_equality() {
        let waypoint1 = create_test_waypoint();
        let waypoint2 = create_test_waypoint();

        assert_eq!(waypoint1, waypoint2);

        // Test inequality with different lane_id
        let mut waypoint3 = create_test_waypoint();
        waypoint3.lane_id = 2;
        assert_ne!(waypoint1, waypoint3);
    }

    #[test]
    fn test_waypoint_is_junction() {
        let mut waypoint = create_test_waypoint();

        // Not in junction (negative junction_id)
        waypoint.junction_id = -1;
        assert!(!waypoint.is_junction());

        // In junction (non-negative junction_id)
        waypoint.junction_id = 0;
        assert!(waypoint.is_junction());

        waypoint.junction_id = 5;
        assert!(waypoint.is_junction());
    }

    #[test]
    fn test_waypoint_debug_format() {
        let waypoint = create_test_waypoint();
        let debug_string = format!("{:?}", waypoint);

        assert!(debug_string.contains("Waypoint"));
        assert!(debug_string.contains("lane_id"));
        assert!(debug_string.contains("road_id"));
        assert!(debug_string.contains("42")); // road_id value
    }

    #[test]
    fn test_lane_change_enum() {
        let lane_changes = [
            LaneChange::None,
            LaneChange::Right,
            LaneChange::Left,
            LaneChange::Both,
        ];

        assert_eq!(lane_changes.len(), 4);

        // Test Debug output
        assert_eq!(format!("{:?}", LaneChange::Both), "Both");
        assert_eq!(format!("{:?}", LaneChange::None), "None");
    }

    #[test]
    fn test_lane_type_enum() {
        let lane_types = [
            LaneType::None,
            LaneType::Driving,
            LaneType::Stop,
            LaneType::Shoulder,
            LaneType::Biking,
            LaneType::Sidewalk,
            LaneType::Border,
            LaneType::Restricted,
            LaneType::Parking,
            LaneType::Bidirectional,
            LaneType::Median,
            LaneType::Special1,
            LaneType::Special2,
            LaneType::Special3,
            LaneType::RoadWorks,
            LaneType::Tram,
            LaneType::Rail,
            LaneType::Entry,
            LaneType::Exit,
            LaneType::OffRamp,
            LaneType::OnRamp,
            LaneType::Any,
        ];

        assert!(lane_types.len() >= 20);

        // Test specific lane types
        assert_eq!(format!("{:?}", LaneType::Driving), "Driving");
        assert_eq!(format!("{:?}", LaneType::Sidewalk), "Sidewalk");
        assert_eq!(format!("{:?}", LaneType::OnRamp), "OnRamp");
    }

    #[test]
    fn test_lane_marking_type_enum() {
        let marking_types = [
            LaneMarkingType::Other,
            LaneMarkingType::Broken,
            LaneMarkingType::Solid,
            LaneMarkingType::SolidSolid,
            LaneMarkingType::SolidBroken,
            LaneMarkingType::BrokenSolid,
            LaneMarkingType::BrokenBroken,
            LaneMarkingType::BottsDots,
            LaneMarkingType::None,
        ];

        assert_eq!(marking_types.len(), 9);

        assert_eq!(format!("{:?}", LaneMarkingType::Solid), "Solid");
        assert_eq!(format!("{:?}", LaneMarkingType::Broken), "Broken");
        assert_eq!(format!("{:?}", LaneMarkingType::BottsDots), "BottsDots");
    }

    #[test]
    fn test_lane_marking_color_enum() {
        let marking_colors = [
            LaneMarkingColor::Standard,
            LaneMarkingColor::Blue,
            LaneMarkingColor::Green,
            LaneMarkingColor::Red,
            LaneMarkingColor::White,
            LaneMarkingColor::Yellow,
            LaneMarkingColor::Other,
        ];

        assert_eq!(marking_colors.len(), 7);

        assert_eq!(format!("{:?}", LaneMarkingColor::Standard), "Standard");
        assert_eq!(format!("{:?}", LaneMarkingColor::Yellow), "Yellow");
        assert_eq!(format!("{:?}", LaneMarkingColor::Red), "Red");
    }

    #[test]
    fn test_waypoint_realistic_values() {
        let highway_waypoint = Waypoint {
            transform: Transform {
                location: Location {
                    x: 100.0,
                    y: -50.0,
                    z: 0.2,
                },
                rotation: Rotation {
                    pitch: 0.0,
                    yaw: 180.0,
                    roll: 0.0,
                },
            },
            lane_id: -1, // Right lane in CARLA (negative for right)
            section_id: 0,
            road_id: 123,
            junction_id: -1,               // Not in junction
            lane_width: 3.7,               // Standard highway lane width
            lane_change: LaneChange::Left, // Can only change to left
            lane_type: LaneType::Driving,
            lane_marking_type: LaneMarkingType::Solid,
            lane_marking_color: LaneMarkingColor::White,
            wrapper: unsafe {
                // This is a test-only placeholder - in real tests we'd need proper mocking
                std::mem::zeroed()
            },
        };

        assert_eq!(highway_waypoint.lane_id, -1);
        assert_eq!(highway_waypoint.lane_width, 3.7);
        assert!(!highway_waypoint.is_junction());
        assert_eq!(highway_waypoint.lane_change, LaneChange::Left);
    }

    #[test]
    fn test_waypoint_distance_calculation() {
        let waypoint1 = Waypoint {
            transform: Transform {
                location: Location {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
                rotation: Rotation::default(),
            },
            ..create_test_waypoint()
        };

        let waypoint2 = Waypoint {
            transform: Transform {
                location: Location {
                    x: 3.0,
                    y: 4.0,
                    z: 0.0,
                },
                rotation: Rotation::default(),
            },
            ..create_test_waypoint()
        };

        let distance = waypoint1.distance(&waypoint2);
        // Distance should be 5.0 (3-4-5 triangle)
        assert!((distance - 5.0).abs() < 0.001);
    }
}
