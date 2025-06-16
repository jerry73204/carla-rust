//! Collision detection sensor.

use crate::{
    client::ActorId,
    geom::{Transform, Vector3D},
    sensor::SensorData,
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
    /// Create CollisionData from carla-cxx CollisionData
    pub fn from_cxx(cxx_data: carla_cxx::sensor::CollisionData) -> Self {
        Self {
            // TODO: Extract proper metadata from carla-cxx CollisionEvent structure
            // This requires adding timestamp, transform, and sensor_id fields to carla-cxx CollisionEvent
            timestamp: todo!("CollisionData::from_cxx timestamp extraction not yet implemented - missing FFI metadata"),
            transform: todo!("CollisionData::from_cxx transform extraction not yet implemented - missing FFI metadata"),
            sensor_id: todo!("CollisionData::from_cxx sensor_id extraction not yet implemented - missing FFI metadata"),
            other_actor_id: cxx_data.other_actor_id,
            normal_impulse: Vector3D::new(
                cxx_data.normal_impulse.x as f32,
                cxx_data.normal_impulse.y as f32,
                cxx_data.normal_impulse.z as f32,
            ),
        }
    }

    /// Get the magnitude of the collision impulse.
    pub fn impulse_magnitude(&self) -> f32 {
        self.normal_impulse.length()
    }

    /// Check if this was a severe collision based on impulse magnitude.
    pub fn is_severe_collision(&self, threshold: f32) -> bool {
        self.impulse_magnitude() > threshold
    }
}
