//! Map and navigation functionality for CARLA simulator.
//!
//! This module provides access to the map's road network, waypoints,
//! junctions, and landmarks.

use cxx::SharedPtr;

use crate::ffi::{
    self, Junction, Map, SimpleGeoLocation, SimpleLaneMarking, SimpleLocation, SimpleTransform,
    SimpleWaypointInfo, Waypoint,
};

/// High-level wrapper for CARLA Map
pub struct MapWrapper {
    inner: SharedPtr<Map>,
}

impl MapWrapper {
    /// Create a new MapWrapper from a SharedPtr<Map>
    pub fn new(map: SharedPtr<Map>) -> Self {
        Self { inner: map }
    }

    /// Get the map's name
    pub fn get_name(&self) -> String {
        ffi::Map_GetName(&self.inner)
    }

    /// Get the OpenDRIVE XML content
    pub fn get_open_drive(&self) -> String {
        ffi::Map_GetOpenDrive(&self.inner)
    }

    /// Get recommended spawn points for the map
    pub fn get_recommended_spawn_points(&self) -> Vec<SimpleTransform> {
        ffi::Map_GetRecommendedSpawnPoints(&self.inner)
    }

    /// Get a waypoint at the specified location
    ///
    /// # Arguments
    /// * `location` - The location to query
    /// * `project_to_road` - If true, the location will be projected to the nearest road
    /// * `lane_type` - Filter by lane type (defaults to Driving lanes)
    pub fn get_waypoint(
        &self,
        location: &SimpleLocation,
        project_to_road: bool,
        lane_type: Option<LaneType>,
    ) -> Option<WaypointWrapper> {
        let lane_type_int = lane_type.unwrap_or(LaneType::Driving) as i32;
        let waypoint = ffi::Map_GetWaypoint(&self.inner, location, project_to_road, lane_type_int);
        if waypoint.is_null() {
            None
        } else {
            Some(WaypointWrapper::new(waypoint))
        }
    }

    /// Get a waypoint from OpenDRIVE coordinates
    pub fn get_waypoint_xodr(&self, road_id: u32, lane_id: i32, s: f64) -> Option<WaypointWrapper> {
        let waypoint = ffi::Map_GetWaypointXODR(&self.inner, road_id, lane_id, s);
        if waypoint.is_null() {
            None
        } else {
            Some(WaypointWrapper::new(waypoint))
        }
    }

    /// Generate waypoints distributed along the map at a given distance
    pub fn generate_waypoints(&self, distance: f64) -> Vec<SimpleWaypointInfo> {
        ffi::Map_GenerateWaypoints(&self.inner, distance)
    }

    /// Get the geo-reference of the map
    pub fn get_geo_reference(&self) -> SimpleGeoLocation {
        ffi::Map_GetGeoReference(&self.inner)
    }

    /// Get all crosswalk zones in the map
    pub fn get_all_crosswalk_zones(&self) -> Vec<SimpleLocation> {
        ffi::Map_GetAllCrosswalkZones(&self.inner)
    }

    /// Get the junction that a waypoint belongs to (if any)
    pub fn get_junction(&self, waypoint: &WaypointWrapper) -> Option<JunctionWrapper> {
        let junction = ffi::Map_GetJunction(&self.inner, &waypoint.inner);
        if junction.is_null() {
            None
        } else {
            Some(JunctionWrapper::new(junction))
        }
    }

    /// Get the map topology
    pub fn get_topology(&self) -> Vec<SimpleWaypointInfo> {
        ffi::Map_GetTopology(&self.inner)
    }
}

/// High-level wrapper for CARLA Waypoint
pub struct WaypointWrapper {
    inner: SharedPtr<Waypoint>,
}

impl WaypointWrapper {
    /// Create a new WaypointWrapper from a SharedPtr<Waypoint>
    pub fn new(waypoint: SharedPtr<Waypoint>) -> Self {
        Self { inner: waypoint }
    }

    /// Get the waypoint's unique ID
    pub fn get_id(&self) -> u64 {
        ffi::Waypoint_GetId(&self.inner)
    }

    /// Get the road ID
    pub fn get_road_id(&self) -> u32 {
        ffi::Waypoint_GetRoadId(&self.inner)
    }

    /// Get the section ID
    pub fn get_section_id(&self) -> u32 {
        ffi::Waypoint_GetSectionId(&self.inner)
    }

    /// Get the lane ID (negative for right lanes, positive for left lanes)
    pub fn get_lane_id(&self) -> i32 {
        ffi::Waypoint_GetLaneId(&self.inner)
    }

    /// Get the distance along the road (s coordinate in OpenDRIVE)
    pub fn get_distance(&self) -> f64 {
        ffi::Waypoint_GetDistance(&self.inner)
    }

    /// Get the waypoint's transform
    pub fn get_transform(&self) -> SimpleTransform {
        ffi::Waypoint_GetTransform(&self.inner)
    }

    /// Get the junction ID (0 if not in a junction)
    pub fn get_junction_id(&self) -> u32 {
        ffi::Waypoint_GetJunctionId(&self.inner)
    }

    /// Check if this waypoint is in a junction
    pub fn is_junction(&self) -> bool {
        ffi::Waypoint_IsJunction(&self.inner)
    }

    /// Get the junction this waypoint belongs to (if any)
    pub fn get_junction(&self) -> Option<JunctionWrapper> {
        let junction = ffi::Waypoint_GetJunction(&self.inner);
        if junction.is_null() {
            None
        } else {
            Some(JunctionWrapper::new(junction))
        }
    }

    /// Get the width of the lane at this waypoint
    pub fn get_lane_width(&self) -> f64 {
        ffi::Waypoint_GetLaneWidth(&self.inner)
    }

    /// Get the lane type
    pub fn get_type(&self) -> LaneType {
        LaneType::from_u32(ffi::Waypoint_GetType(&self.inner))
    }

    /// Get next waypoint at a certain distance
    pub fn get_next(&self, distance: f64) -> Option<WaypointWrapper> {
        let waypoint = ffi::Waypoint_GetNext(&self.inner, distance);
        if waypoint.is_null() {
            None
        } else {
            Some(WaypointWrapper::new(waypoint))
        }
    }

    /// Get previous waypoint at a certain distance
    pub fn get_previous(&self, distance: f64) -> Option<WaypointWrapper> {
        let waypoint = ffi::Waypoint_GetPrevious(&self.inner, distance);
        if waypoint.is_null() {
            None
        } else {
            Some(WaypointWrapper::new(waypoint))
        }
    }

    /// Get the waypoint in the right lane (if exists)
    pub fn get_right(&self) -> Option<WaypointWrapper> {
        let waypoint = ffi::Waypoint_GetRight(&self.inner);
        if waypoint.is_null() {
            None
        } else {
            Some(WaypointWrapper::new(waypoint))
        }
    }

    /// Get the waypoint in the left lane (if exists)
    pub fn get_left(&self) -> Option<WaypointWrapper> {
        let waypoint = ffi::Waypoint_GetLeft(&self.inner);
        if waypoint.is_null() {
            None
        } else {
            Some(WaypointWrapper::new(waypoint))
        }
    }

    /// Get the right lane marking
    pub fn get_right_lane_marking(&self) -> LaneMarking {
        let marking = ffi::Waypoint_GetRightLaneMarking(&self.inner);
        LaneMarking::from_simple(marking)
    }

    /// Get the left lane marking
    pub fn get_left_lane_marking(&self) -> LaneMarking {
        let marking = ffi::Waypoint_GetLeftLaneMarking(&self.inner);
        LaneMarking::from_simple(marking)
    }

    /// Get the allowed lane change directions
    pub fn get_lane_change(&self) -> LaneChange {
        LaneChange::from_u8(ffi::Waypoint_GetLaneChange(&self.inner))
    }
}

/// High-level wrapper for CARLA Junction
pub struct JunctionWrapper {
    inner: SharedPtr<Junction>,
}

impl JunctionWrapper {
    /// Create a new JunctionWrapper from a SharedPtr<Junction>
    pub fn new(junction: SharedPtr<Junction>) -> Self {
        Self { inner: junction }
    }

    /// Get the junction's ID
    pub fn get_id(&self) -> u32 {
        ffi::Junction_GetId(&self.inner)
    }

    /// Get the bounding box of the junction
    pub fn get_bounding_box(&self) -> crate::ffi::SimpleBoundingBox {
        ffi::Junction_GetBoundingBox(&self.inner)
    }
}

// TODO: Landmark wrapper disabled for now due to CXX compatibility issues
// pub struct LandmarkWrapper { ... }

/// Lane types in CARLA
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum LaneType {
    None = 0x00,
    Driving = 0x01,
    Stop = 0x02,
    Shoulder = 0x04,
    Biking = 0x08,
    Sidewalk = 0x10,
    Border = 0x20,
    Restricted = 0x40,
    Parking = 0x80,
    Bidirectional = 0x100,
    Median = 0x200,
    Special1 = 0x400,
    Special2 = 0x800,
    Special3 = 0x1000,
    RoadWorks = 0x2000,
    Tram = 0x4000,
    Rail = 0x8000,
    Entry = 0x10000,
    Exit = 0x20000,
    OffRamp = 0x40000,
    OnRamp = 0x80000,
    Any = 0xFFFFFFF,
}

impl LaneType {
    fn from_u32(value: u32) -> Self {
        match value {
            0x00 => LaneType::None,
            0x01 => LaneType::Driving,
            0x02 => LaneType::Stop,
            0x04 => LaneType::Shoulder,
            0x08 => LaneType::Biking,
            0x10 => LaneType::Sidewalk,
            0x20 => LaneType::Border,
            0x40 => LaneType::Restricted,
            0x80 => LaneType::Parking,
            0x100 => LaneType::Bidirectional,
            0x200 => LaneType::Median,
            0x400 => LaneType::Special1,
            0x800 => LaneType::Special2,
            0x1000 => LaneType::Special3,
            0x2000 => LaneType::RoadWorks,
            0x4000 => LaneType::Tram,
            0x8000 => LaneType::Rail,
            0x10000 => LaneType::Entry,
            0x20000 => LaneType::Exit,
            0x40000 => LaneType::OffRamp,
            0x80000 => LaneType::OnRamp,
            _ => LaneType::Any,
        }
    }
}

/// Lane marking types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum LaneMarkingType {
    Other = 0,
    Broken = 1,
    Solid = 2,
    SolidSolid = 3,
    SolidBroken = 4,
    BrokenSolid = 5,
    BrokenBroken = 6,
    BottsDots = 7,
    Grass = 8,
    Curb = 9,
    None = 10,
}

impl LaneMarkingType {
    pub fn from_u32(value: u32) -> Self {
        match value {
            0 => LaneMarkingType::Other,
            1 => LaneMarkingType::Broken,
            2 => LaneMarkingType::Solid,
            3 => LaneMarkingType::SolidSolid,
            4 => LaneMarkingType::SolidBroken,
            5 => LaneMarkingType::BrokenSolid,
            6 => LaneMarkingType::BrokenBroken,
            7 => LaneMarkingType::BottsDots,
            8 => LaneMarkingType::Grass,
            9 => LaneMarkingType::Curb,
            _ => LaneMarkingType::None,
        }
    }
}

/// Lane marking colors
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum LaneMarkingColor {
    Standard = 0,
    Blue = 1,
    Green = 2,
    Red = 3,
    Yellow = 4,
    Other = 5,
}

impl LaneMarkingColor {
    pub fn from_u8(value: u8) -> Self {
        match value {
            0 => LaneMarkingColor::Standard,
            1 => LaneMarkingColor::Blue,
            2 => LaneMarkingColor::Green,
            3 => LaneMarkingColor::Red,
            4 => LaneMarkingColor::Yellow,
            _ => LaneMarkingColor::Other,
        }
    }
}

/// Allowed lane change directions
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LaneChange {
    None,
    Right,
    Left,
    Both,
}

impl LaneChange {
    pub fn from_u8(value: u8) -> Self {
        match value {
            0x00 => LaneChange::None,
            0x01 => LaneChange::Right,
            0x02 => LaneChange::Left,
            0x03 => LaneChange::Both,
            _ => LaneChange::None,
        }
    }
}

/// Lane marking information
#[derive(Debug, Clone, Copy)]
pub struct LaneMarking {
    pub marking_type: LaneMarkingType,
    pub color: LaneMarkingColor,
    pub lane_change: LaneChange,
    pub width: f64,
}

impl LaneMarking {
    fn from_simple(simple: SimpleLaneMarking) -> Self {
        LaneMarking {
            marking_type: LaneMarkingType::from_u32(simple.lane_marking_type),
            color: LaneMarkingColor::from_u8(simple.color),
            lane_change: LaneChange::from_u8(simple.lane_change),
            width: simple.width,
        }
    }
}

/// Signal orientation
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum SignalOrientation {
    Positive = 0,
    Negative = 1,
    Both = 2,
}

impl SignalOrientation {
    fn from_i32(value: i32) -> Self {
        match value {
            0 => SignalOrientation::Positive,
            1 => SignalOrientation::Negative,
            2 => SignalOrientation::Both,
            _ => SignalOrientation::Both,
        }
    }
}
