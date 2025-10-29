//! Sensor modules
//!
//! Sensor implementations for CARLA:
//! - CollisionSensor: Collision detection and history
//! - LaneInvasionSensor: Lane marking crossing detection
//! - GnssSensor: GPS coordinates
//! - IMUSensor: Accelerometer, gyroscope, compass
//! - RadarSensor: Object detection with velocity

pub mod collision;
pub mod gnss;
pub mod imu;
pub mod lane_invasion;
pub mod radar;

// ✅ Subphase 12.5: Export sensors for use in main application
pub use collision::CollisionSensor;
pub use gnss::GnssSensor;
pub use imu::IMUSensor;
pub use lane_invasion::LaneInvasionSensor;
// pub use radar::RadarSensor; // TODO: Phase 8+
