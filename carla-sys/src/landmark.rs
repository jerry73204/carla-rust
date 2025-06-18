//! Landmark and signal system for CARLA road infrastructure.
//!
//! This module provides interfaces for accessing and interpreting traffic signs,
//! speed limits, and other road infrastructure landmarks in the CARLA world.
//! Landmarks are essential for autonomous driving algorithms that need to
//! understand traffic regulations and road conditions.

use crate::{ffi::bridge, map::SignalOrientation};

/// Common landmark types found in CARLA based on OpenDRIVE standards.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum LandmarkType {
    /// General danger warning
    Danger,
    /// Yield sign (give way)
    Yield,
    /// Stop sign
    Stop,
    /// Speed limit signs
    SpeedLimit,
    /// Mandatory turn direction
    MandatoryDirection,
    /// Pedestrian crossing
    PedestrianCrossing,
    /// Bicycle crossing
    BicycleCrossing,
    /// Traffic light
    TrafficLight,
    /// Lanes merging
    LanesMerging,
    /// Unknown or other type
    Other,
}

impl LandmarkType {
    /// Parse landmark type from OpenDRIVE type string.
    pub fn from_type_string(type_str: &str) -> Self {
        match type_str {
            "101" => LandmarkType::Danger,
            "205" => LandmarkType::Yield,
            "206" => LandmarkType::Stop,
            "274" => LandmarkType::SpeedLimit,
            "209" => LandmarkType::MandatoryDirection,
            "133" => LandmarkType::PedestrianCrossing,
            "138" => LandmarkType::BicycleCrossing,
            "121" => LandmarkType::LanesMerging,
            s if s.starts_with("1000") => LandmarkType::TrafficLight, // Traffic light types
            _ => LandmarkType::Other,
        }
    }

    /// Get the OpenDRIVE type code for this landmark type.
    pub fn to_type_string(&self) -> &'static str {
        match self {
            LandmarkType::Danger => "101",
            LandmarkType::Yield => "205",
            LandmarkType::Stop => "206",
            LandmarkType::SpeedLimit => "274",
            LandmarkType::MandatoryDirection => "209",
            LandmarkType::PedestrianCrossing => "133",
            LandmarkType::BicycleCrossing => "138",
            LandmarkType::LanesMerging => "121",
            LandmarkType::TrafficLight => "1000001", // Generic traffic light
            LandmarkType::Other => "0",
        }
    }
}

/// Complete landmark information including location, properties, and regulatory data.
#[derive(Debug, Clone, PartialEq)]
pub struct LandmarkInfo {
    /// Unique landmark identifier
    pub id: String,
    /// Human-readable name
    pub name: String,
    /// OpenDRIVE type code
    pub type_code: String,
    /// Parsed landmark type
    pub landmark_type: LandmarkType,
    /// Subtype for additional classification
    pub sub_type: String,
    /// Distance from search origin point
    pub distance: f64,
    /// Position along road centerline (s-coordinate)
    pub s: f64,
    /// Whether this landmark can change state dynamically
    pub is_dynamic: bool,
    /// Which lane directions this landmark affects
    pub orientation: SignalOrientation,
    /// Vertical offset from road surface
    pub z_offset: f64,
    /// Country-specific coding system
    pub country: String,
    /// Numeric value (e.g., speed limit in km/h)
    pub value: f64,
    /// Unit of measurement for the value
    pub unit: String,
    /// Physical dimensions
    pub height: f64,
    pub width: f64,
    /// Text content displayed on sign
    pub text: String,
    /// Horizontal offset from road centerline
    pub h_offset: f64,
    /// Rotation angles
    pub pitch: f64,
    pub roll: f64,
}

impl From<bridge::SimpleLandmark> for LandmarkInfo {
    fn from(simple: bridge::SimpleLandmark) -> Self {
        LandmarkInfo {
            id: simple.id,
            name: simple.name,
            type_code: simple.type_.clone(),
            landmark_type: LandmarkType::from_type_string(&simple.type_),
            sub_type: simple.sub_type,
            distance: simple.distance,
            s: simple.s,
            is_dynamic: simple.is_dynamic,
            orientation: match simple.orientation {
                0 => SignalOrientation::Positive,
                1 => SignalOrientation::Negative,
                2 => SignalOrientation::Both,
                _ => SignalOrientation::Both,
            },
            z_offset: simple.z_offset,
            country: simple.country,
            value: simple.value,
            unit: simple.unit,
            height: simple.height,
            width: simple.width,
            text: simple.text,
            h_offset: simple.h_offset,
            pitch: simple.pitch,
            roll: simple.roll,
        }
    }
}

impl LandmarkInfo {
    /// Check if this landmark represents a speed limit sign.
    pub fn is_speed_limit(&self) -> bool {
        self.landmark_type == LandmarkType::SpeedLimit
    }

    /// Get the speed limit value if this is a speed limit sign.
    /// Returns the value in the original unit (usually km/h).
    pub fn get_speed_limit(&self) -> Option<f64> {
        if self.is_speed_limit() && self.value > 0.0 {
            Some(self.value)
        } else {
            None
        }
    }

    /// Check if this landmark represents a stop sign.
    pub fn is_stop_sign(&self) -> bool {
        self.landmark_type == LandmarkType::Stop
    }

    /// Check if this landmark represents a yield sign.
    pub fn is_yield_sign(&self) -> bool {
        self.landmark_type == LandmarkType::Yield
    }

    /// Check if this landmark represents a traffic light.
    pub fn is_traffic_light(&self) -> bool {
        self.landmark_type == LandmarkType::TrafficLight
    }

    /// Check if this landmark can change its state (e.g., traffic lights).
    pub fn is_dynamic(&self) -> bool {
        self.is_dynamic
    }

    /// Get a human-readable description of this landmark.
    pub fn get_description(&self) -> String {
        match self.landmark_type {
            LandmarkType::SpeedLimit => {
                if let Some(limit) = self.get_speed_limit() {
                    format!("Speed Limit: {:.0} {}", limit, self.unit)
                } else {
                    "Speed Limit Sign".to_string()
                }
            }
            LandmarkType::Stop => "Stop Sign".to_string(),
            LandmarkType::Yield => "Yield Sign".to_string(),
            LandmarkType::TrafficLight => "Traffic Light".to_string(),
            LandmarkType::PedestrianCrossing => "Pedestrian Crossing".to_string(),
            LandmarkType::BicycleCrossing => "Bicycle Crossing".to_string(),
            LandmarkType::MandatoryDirection => "Mandatory Direction".to_string(),
            LandmarkType::LanesMerging => "Lanes Merging".to_string(),
            LandmarkType::Danger => "Danger Warning".to_string(),
            LandmarkType::Other => {
                if !self.text.is_empty() {
                    format!("Sign: {}", self.text)
                } else if !self.name.is_empty() {
                    self.name.clone()
                } else {
                    format!("Landmark ({})", self.type_code)
                }
            }
        }
    }
}

/// Collection of utility functions for landmark filtering and analysis.
pub mod landmark_utils {
    use super::*;

    /// Filter landmarks to find only speed limit signs.
    pub fn filter_speed_limits(landmarks: &[LandmarkInfo]) -> Vec<&LandmarkInfo> {
        landmarks.iter().filter(|l| l.is_speed_limit()).collect()
    }

    /// Filter landmarks to find only stop signs.
    pub fn filter_stop_signs(landmarks: &[LandmarkInfo]) -> Vec<&LandmarkInfo> {
        landmarks.iter().filter(|l| l.is_stop_sign()).collect()
    }

    /// Filter landmarks to find only traffic lights.
    pub fn filter_traffic_lights(landmarks: &[LandmarkInfo]) -> Vec<&LandmarkInfo> {
        landmarks.iter().filter(|l| l.is_traffic_light()).collect()
    }

    /// Find the nearest landmark of a specific type.
    pub fn find_nearest_of_type(
        landmarks: &[LandmarkInfo],
        landmark_type: LandmarkType,
    ) -> Option<&LandmarkInfo> {
        landmarks
            .iter()
            .filter(|l| l.landmark_type == landmark_type)
            .min_by(|a, b| {
                a.distance
                    .partial_cmp(&b.distance)
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
    }

    /// Get the applicable speed limit from a collection of landmarks.
    /// Returns the speed limit with the smallest distance.
    pub fn get_applicable_speed_limit(landmarks: &[LandmarkInfo]) -> Option<f64> {
        landmarks
            .iter()
            .filter_map(|l| l.get_speed_limit().map(|limit| (limit, l.distance)))
            .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
            .map(|(limit, _)| limit)
    }

    /// Group landmarks by their type for analysis.
    pub fn group_by_type(
        landmarks: &[LandmarkInfo],
    ) -> std::collections::HashMap<LandmarkType, Vec<&LandmarkInfo>> {
        let mut groups = std::collections::HashMap::new();
        for landmark in landmarks {
            groups
                .entry(landmark.landmark_type)
                .or_insert_with(Vec::new)
                .push(landmark);
        }
        groups
    }
}
