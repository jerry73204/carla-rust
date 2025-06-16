//! Road Safety (RSS) sensor implementations.

use crate::{geom::Transform, sensor_data::SensorData, time::Timestamp};

/// RSS (Road Safety) sensor data.
#[derive(Debug, Clone)]
pub struct RSSData {
    /// Sensor timestamp
    pub timestamp: Timestamp,
    /// Sensor transform when captured
    pub transform: Transform,
    /// Sensor ID
    pub sensor_id: u32,
    /// RSS response from the sensor
    pub response: RSSResponse,
}

/// RSS (Road Safety) response containing safety analysis.
#[derive(Debug, Clone)]
pub struct RSSResponse {
    /// Whether the RSS calculation is valid
    pub response_valid: bool,
    /// RSS proper response (serialized safety action)
    pub proper_response: String,
    /// Current RSS state snapshot (serialized)
    pub rss_state_snapshot: String,
    /// Situation analysis snapshot (serialized)
    pub situation_snapshot: String,
    /// World model used for calculation (serialized)
    pub world_model: String,
    /// Ego vehicle dynamics on route (serialized)
    pub ego_dynamics_on_route: String,
}

impl SensorData for RSSData {
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
        // Approximate size based on string lengths
        self.response.proper_response.len()
            + self.response.rss_state_snapshot.len()
            + self.response.situation_snapshot.len()
            + self.response.world_model.len()
            + self.response.ego_dynamics_on_route.len()
            + std::mem::size_of::<bool>()
    }
}

impl RSSData {
    /// Create RSSData from carla-cxx RssResponse
    pub fn from_cxx(cxx_response: carla_cxx::sensor::RssResponse) -> Self {
        let response = RSSResponse {
            response_valid: cxx_response.response_valid,
            proper_response: cxx_response.proper_response,
            rss_state_snapshot: cxx_response.rss_state_snapshot,
            situation_snapshot: cxx_response.situation_snapshot,
            world_model: cxx_response.world_model,
            ego_dynamics_on_route: cxx_response.ego_dynamics_on_route,
        };

        Self {
            // TODO: Extract proper metadata from carla-cxx RssResponse structure
            // This requires adding timestamp, transform, and sensor_id fields to carla-cxx RssResponse
            timestamp: todo!(
                "RSSData::from_cxx timestamp extraction not yet implemented - missing FFI metadata"
            ),
            transform: todo!(
                "RSSData::from_cxx transform extraction not yet implemented - missing FFI metadata"
            ),
            sensor_id: todo!(
                "RSSData::from_cxx sensor_id extraction not yet implemented - missing FFI metadata"
            ),
            response,
        }
    }

    /// Check if the RSS response is valid.
    pub fn is_valid(&self) -> bool {
        self.response.response_valid
    }

    /// Check if RSS indicates a safe state.
    pub fn is_safe(&self) -> bool {
        self.is_valid() && self.response.proper_response.contains("safe")
    }

    /// Check if RSS indicates an unsafe state requiring intervention.
    pub fn requires_intervention(&self) -> bool {
        self.is_valid()
            && (self.response.proper_response.contains("brake")
                || self.response.proper_response.contains("stop")
                || self.response.proper_response.contains("emergency"))
    }

    /// Parse RSS safety action from the proper response.
    pub fn get_safety_action(&self) -> RssSafetyAction {
        if !self.is_valid() {
            return RssSafetyAction::Invalid;
        }

        let response = self.response.proper_response.to_lowercase();

        if response.contains("emergency") || response.contains("critical") {
            RssSafetyAction::EmergencyBrake
        } else if response.contains("brake") {
            RssSafetyAction::Brake
        } else if response.contains("slow") || response.contains("decelerate") {
            RssSafetyAction::SlowDown
        } else if response.contains("maintain") {
            RssSafetyAction::MaintainSpeed
        } else if response.contains("safe") || response.contains("continue") {
            RssSafetyAction::Continue
        } else {
            RssSafetyAction::Unknown
        }
    }

    /// Get RSS analysis summary.
    pub fn get_analysis_summary(&self) -> RSSAnalysis {
        let safety_action = self.get_safety_action();
        let safety_level = match safety_action {
            RssSafetyAction::EmergencyBrake => SafetyLevel::Critical,
            RssSafetyAction::Brake => SafetyLevel::High,
            RssSafetyAction::SlowDown => SafetyLevel::Medium,
            RssSafetyAction::MaintainSpeed => SafetyLevel::Low,
            RssSafetyAction::Continue => SafetyLevel::Safe,
            RssSafetyAction::Invalid | RssSafetyAction::Unknown => SafetyLevel::Unknown,
        };

        let has_hazards = self.detect_hazards();
        let ego_speed = self.extract_ego_speed();
        let situation_type = self.classify_situation();

        RSSAnalysis {
            is_valid: self.is_valid(),
            safety_level,
            safety_action,
            has_hazards,
            ego_speed,
            situation_type,
            requires_immediate_action: matches!(
                safety_action,
                RssSafetyAction::EmergencyBrake | RssSafetyAction::Brake
            ),
        }
    }

    /// Extract ego vehicle speed from dynamics data (if available).
    fn extract_ego_speed(&self) -> Option<f32> {
        // Simple parsing of speed from ego dynamics string
        // In a real implementation, this would parse structured data
        if let Some(start) = self.response.ego_dynamics_on_route.find("speed") {
            if let Some(value_start) = self.response.ego_dynamics_on_route[start..].find(':') {
                let remaining = &self.response.ego_dynamics_on_route[start + value_start + 1..];
                if let Some(value_end) = remaining.find(',').or_else(|| remaining.find('}')) {
                    if let Ok(speed) = remaining[..value_end].trim().parse::<f32>() {
                        return Some(speed);
                    }
                }
            }
        }
        None
    }

    /// Detect potential hazards from RSS state.
    fn detect_hazards(&self) -> bool {
        let state = self.response.rss_state_snapshot.to_lowercase();
        state.contains("hazard")
            || state.contains("collision")
            || state.contains("unsafe")
            || state.contains("violation")
    }

    /// Classify the driving situation type.
    fn classify_situation(&self) -> SituationType {
        let situation = self.response.situation_snapshot.to_lowercase();

        if situation.contains("intersection") {
            SituationType::Intersection
        } else if situation.contains("highway") || situation.contains("freeway") {
            SituationType::Highway
        } else if situation.contains("urban") || situation.contains("city") {
            SituationType::Urban
        } else if situation.contains("parking") {
            SituationType::Parking
        } else if situation.contains("emergency") {
            SituationType::Emergency
        } else {
            SituationType::Unknown
        }
    }

    /// Get detailed RSS state information (requires parsing structured data).
    pub fn get_detailed_state(&self) -> RSSDetailedState {
        RSSDetailedState {
            longitudinal_safe: self.check_longitudinal_safety(),
            lateral_safe: self.check_lateral_safety(),
            intersection_safe: self.check_intersection_safety(),
            response_time_margin: self.calculate_response_time_margin(),
            safe_distance_margin: self.calculate_safe_distance_margin(),
        }
    }

    /// Check longitudinal (forward/backward) safety.
    fn check_longitudinal_safety(&self) -> bool {
        let state = self.response.rss_state_snapshot.to_lowercase();
        !state.contains("longitudinal_unsafe") && !state.contains("rear_collision_risk")
    }

    /// Check lateral (left/right) safety.
    fn check_lateral_safety(&self) -> bool {
        let state = self.response.rss_state_snapshot.to_lowercase();
        !state.contains("lateral_unsafe") && !state.contains("side_collision_risk")
    }

    /// Check intersection safety.
    fn check_intersection_safety(&self) -> bool {
        let state = self.response.rss_state_snapshot.to_lowercase();
        !state.contains("intersection_unsafe") && !state.contains("crossing_conflict")
    }

    /// Calculate response time margin (simplified).
    fn calculate_response_time_margin(&self) -> Option<f32> {
        // TODO: Implement proper RSS data parsing for response time margin calculation
        // This requires parsing structured RSS data from proper_response field
        todo!(
            "RSSData::calculate_response_time_margin not yet implemented - needs RSS data parsing"
        )
    }

    /// Calculate safe distance margin (simplified).
    fn calculate_safe_distance_margin(&self) -> Option<f32> {
        // TODO: Implement proper RSS data parsing for safe distance margin calculation
        // This requires parsing structured RSS data from rss_state_snapshot field
        todo!(
            "RSSData::calculate_safe_distance_margin not yet implemented - needs RSS data parsing"
        )
    }
}

/// RSS safety action recommendations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RssSafetyAction {
    /// RSS response is invalid
    Invalid,
    /// Continue normal operation
    Continue,
    /// Maintain current speed
    MaintainSpeed,
    /// Slow down gradually
    SlowDown,
    /// Apply brakes
    Brake,
    /// Emergency braking required
    EmergencyBrake,
    /// Unknown action
    Unknown,
}

/// Safety level classification.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SafetyLevel {
    /// Safe to continue
    Safe,
    /// Low risk situation
    Low,
    /// Medium risk situation
    Medium,
    /// High risk situation
    High,
    /// Critical situation requiring immediate action
    Critical,
    /// Unknown safety level
    Unknown,
}

/// Driving situation type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SituationType {
    /// Highway/freeway driving
    Highway,
    /// Urban city driving
    Urban,
    /// Intersection navigation
    Intersection,
    /// Parking maneuvers
    Parking,
    /// Emergency situation
    Emergency,
    /// Unknown situation
    Unknown,
}

/// RSS analysis summary.
#[derive(Debug, Clone)]
pub struct RSSAnalysis {
    /// Whether the RSS calculation is valid
    pub is_valid: bool,
    /// Current safety level
    pub safety_level: SafetyLevel,
    /// Recommended safety action
    pub safety_action: RssSafetyAction,
    /// Whether hazards are detected
    pub has_hazards: bool,
    /// Ego vehicle speed (if available)
    pub ego_speed: Option<f32>,
    /// Type of driving situation
    pub situation_type: SituationType,
    /// Whether immediate action is required
    pub requires_immediate_action: bool,
}

/// Detailed RSS state information.
#[derive(Debug, Clone)]
pub struct RSSDetailedState {
    /// Longitudinal safety status
    pub longitudinal_safe: bool,
    /// Lateral safety status
    pub lateral_safe: bool,
    /// Intersection safety status
    pub intersection_safe: bool,
    /// Response time margin (seconds)
    pub response_time_margin: Option<f32>,
    /// Safe distance margin (meters)
    pub safe_distance_margin: Option<f32>,
}
