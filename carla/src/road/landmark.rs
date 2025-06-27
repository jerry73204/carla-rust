//! Road landmarks and signage.

use crate::geom::Transform;
// TODO: Re-enable when SimpleLandmark is available
// use carla_sys::ffi::SimpleLandmark;

/// A road landmark (traffic sign, signal, etc.).
#[derive(Debug, Clone)]
pub struct Landmark {
    /// Unique identifier for this landmark
    pub id: String,
    /// Name of the landmark
    pub name: String,
    /// Type of landmark (e.g., "traffic_light", "stop_sign")
    pub landmark_type: String,
    /// Transform of the landmark in the world
    pub transform: Transform,
    /// Height of the landmark
    pub height: f64,
    /// Z offset of the landmark
    pub z_offset: f64,
    /// Country code for the landmark
    pub country: String,
    /// Sub-type of the landmark
    pub sub_type: String,
    /// Value associated with the landmark (e.g., speed limit)
    pub value: f64,
    /// Unit of the value (e.g., "km/h")
    pub unit: String,
    /// Text content of the landmark
    pub text: String,
}

impl Landmark {
    // TODO: Re-enable when SimpleLandmark is available
    // /// Create a Landmark from a SimpleLandmark FFI structure.
    /*
    pub fn from_simple(simple: SimpleLandmark) -> Self {
        Self {
            id: simple.id,
            name: simple.name,
            landmark_type: simple.type_,
            transform: Transform::from(simple.transform),
            height: simple.height,
            z_offset: simple.z_offset,
            country: simple.country,
            sub_type: simple.sub_type,
            value: simple.value,
            unit: simple.unit,
            text: simple.text,
        }
    }
    */

    /// Check if this landmark is a traffic light.
    pub fn is_traffic_light(&self) -> bool {
        self.landmark_type == "traffic_light"
    }

    /// Check if this landmark is a stop sign.
    pub fn is_stop_sign(&self) -> bool {
        self.landmark_type == "stop_sign" || self.landmark_type.contains("stop")
    }

    /// Check if this landmark is a speed limit sign.
    pub fn is_speed_limit(&self) -> bool {
        self.landmark_type.contains("speed") || self.landmark_type.contains("limit")
    }

    /// Get the speed limit value if this is a speed limit sign.
    pub fn speed_limit(&self) -> Option<f64> {
        if self.is_speed_limit() {
            Some(self.value)
        } else {
            None
        }
    }

    /// Check if this landmark is a yield sign.
    pub fn is_yield_sign(&self) -> bool {
        self.landmark_type.contains("yield") || self.landmark_type.contains("give_way")
    }

    /// Check if this landmark is regulatory (affects traffic rules).
    pub fn is_regulatory(&self) -> bool {
        self.is_traffic_light()
            || self.is_stop_sign()
            || self.is_speed_limit()
            || self.is_yield_sign()
    }
}

/// Types of landmarks commonly found in CARLA.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LandmarkType {
    /// Traffic light
    TrafficLight,
    /// Stop sign
    StopSign,
    /// Speed limit sign
    SpeedLimit,
    /// Yield/Give way sign
    YieldSign,
    /// Warning sign
    Warning,
    /// Information sign
    Information,
    /// Other/Unknown type
    Other,
}

impl LandmarkType {
    /// Convert a landmark type string to enum.
    pub fn from_string(s: &str) -> Self {
        match s.to_lowercase().as_str() {
            s if s.contains("traffic_light") => LandmarkType::TrafficLight,
            s if s.contains("stop") => LandmarkType::StopSign,
            s if s.contains("speed") || s.contains("limit") => LandmarkType::SpeedLimit,
            s if s.contains("yield") || s.contains("give_way") => LandmarkType::YieldSign,
            s if s.contains("warning") => LandmarkType::Warning,
            s if s.contains("info") => LandmarkType::Information,
            _ => LandmarkType::Other,
        }
    }

    /// Get the string representation of the landmark type.
    pub fn as_str(&self) -> &'static str {
        match self {
            LandmarkType::TrafficLight => "traffic_light",
            LandmarkType::StopSign => "stop_sign",
            LandmarkType::SpeedLimit => "speed_limit",
            LandmarkType::YieldSign => "yield_sign",
            LandmarkType::Warning => "warning",
            LandmarkType::Information => "information",
            LandmarkType::Other => "other",
        }
    }
}
