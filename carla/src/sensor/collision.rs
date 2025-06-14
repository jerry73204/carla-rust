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
