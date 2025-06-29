//! Collision detection sensor.
//!
//! This module provides Rust bindings for CARLA's CollisionEvent sensor data.
//! The API closely mirrors CARLA's C++ CollisionEvent class which provides:
//! - Base SensorData fields (timestamp, transform, sensor info)
//! - GetOtherActor() -> The actor that was collided with
//! - GetNormalImpulse() -> The collision impulse as a 3D vector
//!
//! Additional methods are minimal convenience functions for common operations
//! on the data that CARLA already provides.

use crate::{
    actor::ActorId,
    geom::{Transform, Vector3D},
    sensor_data::SensorData,
    time::Timestamp,
};

/// Collision detection sensor data.
#[derive(Debug, Clone)]
pub struct CollisionData {
    /// Sensor timestamp
    pub timestamp: Timestamp,
    /// Sensor transform when captured
    pub transform: Transform,
    /// Sensor ID
    pub sensor_id: u32,
    /// Actor that was hit
    pub other_actor_id: ActorId,
    /// Normal impulse applied during collision
    pub normal_impulse: Vector3D,
}

impl SensorData for CollisionData {
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

impl CollisionData {
    /// Create CollisionData from carla-sys CollisionData
    pub fn from_cxx(cxx_data: carla_sys::sensor::CollisionData) -> Self {
        Self {
            timestamp: Timestamp::new(
                cxx_data.timestamp.frame,
                cxx_data.timestamp.elapsed_seconds,
                cxx_data.timestamp.delta_seconds,
                cxx_data.timestamp.platform_timestamp,
            ),
            transform: Transform::new(
                crate::geom::Location::new(
                    cxx_data.transform.location.x,
                    cxx_data.transform.location.y,
                    cxx_data.transform.location.z,
                ),
                crate::geom::Rotation::new(
                    cxx_data.transform.rotation.pitch as f32,
                    cxx_data.transform.rotation.yaw as f32,
                    cxx_data.transform.rotation.roll as f32,
                ),
            ),
            sensor_id: cxx_data.sensor_id,
            other_actor_id: cxx_data.other_actor_id,
            normal_impulse: Vector3D::new(
                cxx_data.normal_impulse.x as f32,
                cxx_data.normal_impulse.y as f32,
                cxx_data.normal_impulse.z as f32,
            ),
        }
    }

    /// Get the magnitude of the collision impulse.
    ///
    /// This is a convenience method that calculates the length of the normal_impulse vector.
    /// While CARLA C++ API doesn't provide this directly, it's a basic mathematical operation
    /// on the impulse vector data that CARLA does provide via GetNormalImpulse().
    pub fn impulse_magnitude(&self) -> f32 {
        self.normal_impulse.length()
    }
}
