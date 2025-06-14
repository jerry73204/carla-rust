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
