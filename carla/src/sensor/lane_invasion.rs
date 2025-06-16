//! Lane invasion detection sensor.

use crate::{geom::Transform, road::LaneMarkingType, sensor::SensorData, time::Timestamp};

/// Lane invasion detection sensor data.
#[derive(Debug, Clone)]
pub struct LaneInvasionData {
    /// Sensor timestamp
    pub timestamp: Timestamp,
    /// Sensor transform when captured
    pub transform: Transform,
    /// Sensor ID
    pub sensor_id: u32,
    /// Crossed lane markings
    pub crossed_lane_markings: Vec<LaneMarkingInfo>,
}

/// Information about a crossed lane marking.
#[derive(Debug, Clone)]
pub struct LaneMarkingInfo {
    /// Type of lane marking
    pub marking_type: LaneMarkingType,
    /// Color of the lane marking
    pub color: LaneMarkingColor,
    /// Lane change allowed from this marking
    pub lane_change: LaneChange,
}

/// Lane marking colors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LaneMarkingColor {
    /// Standard (white)
    Standard,
    /// Blue
    Blue,
    /// Green
    Green,
    /// Red
    Red,
    /// White
    White,
    /// Yellow
    Yellow,
    /// Other
    Other,
}

impl LaneMarkingColor {
    /// Convert from u32 value
    pub fn from_u32(value: u32) -> Self {
        match value {
            0 => LaneMarkingColor::Standard,
            1 => LaneMarkingColor::Blue,
            2 => LaneMarkingColor::Green,
            3 => LaneMarkingColor::Red,
            4 => LaneMarkingColor::White,
            5 => LaneMarkingColor::Yellow,
            _ => LaneMarkingColor::Other,
        }
    }
}

/// Lane change permissions.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LaneChange {
    /// No lane change allowed
    None,
    /// Lane change to the right allowed
    Right,
    /// Lane change to the left allowed
    Left,
    /// Lane change in both directions allowed
    Both,
}

impl LaneChange {
    /// Convert from u8 value
    pub fn from_u8(value: u8) -> Self {
        match value {
            0 => LaneChange::None,
            1 => LaneChange::Right,
            2 => LaneChange::Left,
            3 => LaneChange::Both,
            _ => LaneChange::None,
        }
    }
}

impl SensorData for LaneInvasionData {
    fn timestamp(&self) -> Timestamp {
        self.timestamp
    }

    fn transform(&self) -> Transform {
        self.transform
    }

    fn sensor_id(&self) -> u32 {
        self.sensor_id
    }

    fn size(&self) -> usize {
        std::mem::size_of::<Self>()
    }
}

impl LaneInvasionData {
    /// Create LaneInvasionData from carla-cxx LaneInvasionData
    pub fn from_cxx(cxx_data: carla_cxx::sensor::LaneInvasionData) -> Self {
        let crossed_lane_markings = cxx_data
            .crossed_lane_markings
            .iter()
            .map(|marking| LaneMarkingInfo {
                marking_type: LaneMarkingType::from_u32(marking.lane_type),
                color: LaneMarkingColor::from_u32(marking.color),
                lane_change: LaneChange::from_u8(marking.lane_change),
            })
            .collect();

        Self {
            // TODO: Extract proper metadata from carla-cxx LaneInvasionEvent structure
            // This requires adding timestamp, transform, and sensor_id fields to carla-cxx LaneInvasionEvent
            timestamp: todo!("LaneInvasionData::from_cxx timestamp extraction not yet implemented - missing FFI metadata"),
            transform: todo!("LaneInvasionData::from_cxx transform extraction not yet implemented - missing FFI metadata"),
            sensor_id: todo!("LaneInvasionData::from_cxx sensor_id extraction not yet implemented - missing FFI metadata"),
            crossed_lane_markings,
        }
    }
}
