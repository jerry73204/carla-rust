mod base;
mod camera;
mod lidar;
mod sensor;
mod traffic_light;
mod traffic_sign;
mod vehicle;
mod walker;

pub use base::{Actor, ActorSnapshot};
pub use camera::{Camera, CameraType};
pub use lidar::LiDAR;
pub use sensor::Sensor;
pub use traffic_light::TrafficLight;
pub use traffic_sign::TrafficSign;
pub use vehicle::Vehicle;
pub use walker::Walker;

/// Unique identifier for actors in the simulation.
pub type ActorId = u32;
