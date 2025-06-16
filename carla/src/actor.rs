mod base;
mod camera;
mod collision;
mod dvs;
mod gnss;
mod imu;
mod lane_invasion;
mod lidar;
mod radar;
mod sensor;
mod traffic_light;
mod traffic_sign;
mod vehicle;
mod walker;

pub use base::{Actor, ActorSnapshot};
pub use camera::{Camera, CameraType};
pub use collision::CollisionSensor;
pub use dvs::DVSCamera;
pub use gnss::GNSS;
pub use imu::IMU;
pub use lane_invasion::LaneInvasionSensor;
pub use lidar::LiDAR;
pub use radar::Radar;
pub use sensor::Sensor;
pub use traffic_light::TrafficLight;
pub use traffic_sign::TrafficSign;
pub use vehicle::Vehicle;
pub use walker::Walker;

/// Unique identifier for actors in the simulation.
pub type ActorId = u32;
