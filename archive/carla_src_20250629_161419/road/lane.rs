//! Lane information and utilities.

use super::LaneType;

/// Lane information.
#[derive(Debug, Clone, PartialEq)]
pub struct Lane {
    /// Lane ID
    pub id: i32,
    /// Lane type
    pub lane_type: LaneType,
    /// Lane width
    pub width: f32,
}
