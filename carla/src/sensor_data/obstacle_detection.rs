//! Obstacle Detection sensor implementations.

use crate::{actor::ActorId, geom::Transform, sensor_data::SensorData, time::Timestamp};

/// Obstacle Detection sensor data.
#[derive(Debug, Clone)]
pub struct ObstacleDetectionData {
    /// Sensor timestamp
    pub timestamp: Timestamp,
    /// Sensor transform when captured
    pub transform: Transform,
    /// Sensor ID
    pub sensor_id: u32,
    /// Detection events
    pub detections: Vec<ObstacleDetectionEvent>,
}

/// Individual obstacle detection event.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ObstacleDetectionEvent {
    /// Sensor's parent actor ID
    pub self_actor_id: ActorId,
    /// Detected obstacle actor ID
    pub other_actor_id: ActorId,
    /// Distance to obstacle in meters
    pub distance: f32,
}

impl ObstacleDetectionEvent {
    /// Create a new obstacle detection event.
    pub fn new(self_actor_id: ActorId, other_actor_id: ActorId, distance: f32) -> Self {
        Self {
            self_actor_id,
            other_actor_id,
            distance,
        }
    }

    /// Check if this is a close obstacle (within threshold).
    pub fn is_close_obstacle(&self, threshold: f32) -> bool {
        self.distance <= threshold
    }

    /// Check if this is a critical obstacle (very close).
    pub fn is_critical_obstacle(&self) -> bool {
        self.is_close_obstacle(5.0) // 5 meters
    }

    /// Get the threat level based on distance.
    pub fn threat_level(&self) -> ThreatLevel {
        if self.distance <= 2.0 {
            ThreatLevel::Critical
        } else if self.distance <= 5.0 {
            ThreatLevel::High
        } else if self.distance <= 10.0 {
            ThreatLevel::Medium
        } else {
            ThreatLevel::Low
        }
    }
}

/// Threat level classification for obstacles.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ThreatLevel {
    /// Critical threat (< 2m) - immediate action required
    Critical,
    /// High threat (2-5m) - caution required
    High,
    /// Medium threat (5-10m) - awareness required
    Medium,
    /// Low threat (> 10m) - informational
    Low,
}

impl SensorData for ObstacleDetectionData {
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
        self.detections.len() * std::mem::size_of::<ObstacleDetectionEvent>()
    }
}

impl ObstacleDetectionData {
    /// Create ObstacleDetectionData from carla-cxx ObstacleDetectionEvent
    pub fn from_cxx(cxx_event: carla_cxx::sensor::ObstacleDetectionEvent) -> Self {
        let detection = ObstacleDetectionEvent {
            self_actor_id: cxx_event.self_actor_id,
            other_actor_id: cxx_event.other_actor_id,
            distance: cxx_event.distance,
        };

        Self {
            // TODO: Extract proper metadata from carla-cxx ObstacleDetectionEvent structure
            // This requires adding timestamp, transform, and sensor_id fields to carla-cxx ObstacleDetectionEvent
            timestamp: todo!("ObstacleDetectionData::from_cxx timestamp extraction not yet implemented - missing FFI metadata"),
            transform: todo!("ObstacleDetectionData::from_cxx transform extraction not yet implemented - missing FFI metadata"),
            sensor_id: todo!("ObstacleDetectionData::from_cxx sensor_id extraction not yet implemented - missing FFI metadata"),
            detections: vec![detection],
        }
    }

    /// Create ObstacleDetectionData from multiple events
    pub fn from_events(
        timestamp: Timestamp,
        transform: Transform,
        sensor_id: u32,
        detections: Vec<ObstacleDetectionEvent>,
    ) -> Self {
        Self {
            timestamp,
            transform,
            sensor_id,
            detections,
        }
    }

    /// Get the total number of detected obstacles.
    pub fn obstacle_count(&self) -> usize {
        self.detections.len()
    }

    /// Check if any obstacles are detected.
    pub fn has_obstacles(&self) -> bool {
        !self.detections.is_empty()
    }

    /// Get the closest obstacle.
    pub fn get_closest_obstacle(&self) -> Option<ObstacleDetectionEvent> {
        self.detections
            .iter()
            .min_by(|a, b| a.distance.partial_cmp(&b.distance).unwrap())
            .copied()
    }

    /// Filter obstacles by distance range.
    pub fn filter_by_distance(
        &self,
        min_distance: f32,
        max_distance: f32,
    ) -> Vec<ObstacleDetectionEvent> {
        self.detections
            .iter()
            .filter(|d| d.distance >= min_distance && d.distance <= max_distance)
            .copied()
            .collect()
    }

    /// Get obstacles within a specific distance.
    pub fn get_obstacles_within(&self, distance: f32) -> Vec<ObstacleDetectionEvent> {
        self.detections
            .iter()
            .filter(|d| d.distance <= distance)
            .copied()
            .collect()
    }

    /// Filter obstacles by threat level.
    pub fn filter_by_threat_level(&self, level: ThreatLevel) -> Vec<ObstacleDetectionEvent> {
        self.detections
            .iter()
            .filter(|d| d.threat_level() == level)
            .copied()
            .collect()
    }

    /// Get critical obstacles (immediate threat).
    pub fn get_critical_obstacles(&self) -> Vec<ObstacleDetectionEvent> {
        self.filter_by_threat_level(ThreatLevel::Critical)
    }

    /// Get high-threat obstacles.
    pub fn get_high_threat_obstacles(&self) -> Vec<ObstacleDetectionEvent> {
        self.filter_by_threat_level(ThreatLevel::High)
    }

    /// Check if there are any critical obstacles.
    pub fn has_critical_obstacles(&self) -> bool {
        self.detections
            .iter()
            .any(|d| d.threat_level() == ThreatLevel::Critical)
    }

    /// Check if emergency braking should be triggered.
    pub fn should_emergency_brake(&self, emergency_distance: f32) -> bool {
        self.detections
            .iter()
            .any(|d| d.distance <= emergency_distance)
    }

    /// Get detection statistics.
    pub fn get_statistics(&self) -> ObstacleDetectionStatistics {
        if self.detections.is_empty() {
            return ObstacleDetectionStatistics::default();
        }

        let distances: Vec<f32> = self.detections.iter().map(|d| d.distance).collect();

        let min_distance = distances.iter().copied().fold(f32::INFINITY, f32::min);
        let max_distance = distances.iter().copied().fold(0.0, f32::max);
        let avg_distance = distances.iter().sum::<f32>() / distances.len() as f32;

        let critical_count = self.filter_by_threat_level(ThreatLevel::Critical).len();
        let high_count = self.filter_by_threat_level(ThreatLevel::High).len();
        let medium_count = self.filter_by_threat_level(ThreatLevel::Medium).len();
        let low_count = self.filter_by_threat_level(ThreatLevel::Low).len();

        ObstacleDetectionStatistics {
            total_obstacles: self.detections.len(),
            min_distance,
            max_distance,
            avg_distance,
            critical_threats: critical_count,
            high_threats: high_count,
            medium_threats: medium_count,
            low_threats: low_count,
        }
    }

    /// Generate a simple threat assessment.
    pub fn assess_threat(&self) -> ThreatAssessment {
        let stats = self.get_statistics();

        let overall_threat = if stats.critical_threats > 0 {
            ThreatLevel::Critical
        } else if stats.high_threats > 0 {
            ThreatLevel::High
        } else if stats.medium_threats > 0 {
            ThreatLevel::Medium
        } else if stats.low_threats > 0 {
            ThreatLevel::Low
        } else {
            return ThreatAssessment {
                overall_threat: ThreatLevel::Low,
                requires_action: false,
                recommended_action: "Continue normal operation".to_string(),
                closest_distance: None,
            };
        };

        let requires_action = matches!(overall_threat, ThreatLevel::Critical | ThreatLevel::High);

        let recommended_action = match overall_threat {
            ThreatLevel::Critical => "Emergency brake immediately".to_string(),
            ThreatLevel::High => "Reduce speed and prepare to stop".to_string(),
            ThreatLevel::Medium => "Maintain awareness and be ready to react".to_string(),
            ThreatLevel::Low => "Continue with normal operation".to_string(),
        };

        ThreatAssessment {
            overall_threat,
            requires_action,
            recommended_action,
            closest_distance: Some(stats.min_distance),
        }
    }
}

/// Obstacle detection statistics.
#[derive(Debug, Clone, PartialEq)]
pub struct ObstacleDetectionStatistics {
    /// Total number of detected obstacles
    pub total_obstacles: usize,
    /// Minimum distance to any obstacle
    pub min_distance: f32,
    /// Maximum distance to any obstacle
    pub max_distance: f32,
    /// Average distance to obstacles
    pub avg_distance: f32,
    /// Number of critical threats
    pub critical_threats: usize,
    /// Number of high threats
    pub high_threats: usize,
    /// Number of medium threats
    pub medium_threats: usize,
    /// Number of low threats
    pub low_threats: usize,
}

impl Default for ObstacleDetectionStatistics {
    fn default() -> Self {
        Self {
            total_obstacles: 0,
            min_distance: 0.0,
            max_distance: 0.0,
            avg_distance: 0.0,
            critical_threats: 0,
            high_threats: 0,
            medium_threats: 0,
            low_threats: 0,
        }
    }
}

/// Comprehensive threat assessment.
#[derive(Debug, Clone)]
pub struct ThreatAssessment {
    /// Overall threat level
    pub overall_threat: ThreatLevel,
    /// Whether immediate action is required
    pub requires_action: bool,
    /// Recommended action to take
    pub recommended_action: String,
    /// Distance to closest obstacle
    pub closest_distance: Option<f32>,
}
