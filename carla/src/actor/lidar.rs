//! LiDAR sensor implementation.

use crate::{
    actor::{ActorId, Sensor},
    error::CarlaResult,
    geom::{Transform, Vector3D},
    sensor_data::{LiDARData, LiDARPoint, LiDARStatistics, SemanticLiDARData},
    traits::{ActorT, SensorT},
};

/// LiDAR sensor for point cloud data.
#[derive(Debug)]
pub struct LiDAR(Sensor);

impl LiDAR {
    /// Try to create a LiDAR from a generic Sensor.
    /// Returns None if the sensor is not a LiDAR.
    pub fn from_sensor(sensor: Sensor) -> Option<Self> {
        let type_id = sensor.type_id();
        if type_id.contains("lidar") || type_id.contains("velodyne") {
            Some(LiDAR(sensor))
        } else {
            None
        }
    }

    /// Convert back to generic Sensor.
    pub fn into_sensor(self) -> Sensor {
        self.0
    }

    /// Get reference to the underlying sensor.
    pub fn as_sensor(&self) -> &Sensor {
        &self.0
    }

    /// Get the last LiDAR data from the sensor with full metadata.
    pub fn get_last_lidar_data(&self) -> Option<LiDARData> {
        let cxx_data = self.0.inner().get_last_lidar_data_full();
        if cxx_data.points.is_empty() {
            None
        } else {
            Some(LiDARData::from_cxx(cxx_data))
        }
    }

    /// Get the last LiDAR points only (without metadata) for performance.
    pub fn get_last_lidar_points(&self) -> Vec<LiDARPoint> {
        let cxx_data = self.0.inner().get_last_lidar_data();
        cxx_data
            .points
            .iter()
            .map(|p| LiDARPoint::new(p.x, p.y, p.z, p.intensity))
            .collect()
    }

    /// Get the last Semantic LiDAR data from the sensor.
    pub fn get_last_semantic_lidar_data(&self) -> Option<SemanticLiDARData> {
        let cxx_data = self.0.inner().get_last_semantic_lidar_data();
        if cxx_data.detections.is_empty() {
            None
        } else {
            Some(SemanticLiDARData::from_cxx(cxx_data))
        }
    }

    /// Get LiDAR statistics from the last measurement.
    pub fn get_statistics(&self) -> Option<LiDARStatistics> {
        self.get_last_lidar_data().map(|data| data.get_statistics())
    }

    /// Filter points by distance range from the last measurement.
    pub fn filter_by_distance(&self, min_distance: f32, max_distance: f32) -> Vec<LiDARPoint> {
        if let Some(data) = self.get_last_lidar_data() {
            data.filter_by_distance(min_distance, max_distance)
        } else {
            Vec::new()
        }
    }

    /// Filter points by intensity range from the last measurement.
    pub fn filter_by_intensity(&self, min_intensity: f32, max_intensity: f32) -> Vec<LiDARPoint> {
        if let Some(data) = self.get_last_lidar_data() {
            data.filter_by_intensity(min_intensity, max_intensity)
        } else {
            Vec::new()
        }
    }

    /// Filter points by height (Z coordinate) range from the last measurement.
    pub fn filter_by_height(&self, min_height: f32, max_height: f32) -> Vec<LiDARPoint> {
        if let Some(data) = self.get_last_lidar_data() {
            data.filter_by_height(min_height, max_height)
        } else {
            Vec::new()
        }
    }
}

impl ActorT for LiDAR {
    fn id(&self) -> ActorId {
        self.0.id()
    }

    fn type_id(&self) -> String {
        self.0.type_id()
    }

    fn transform(&self) -> Transform {
        self.0.transform()
    }

    fn set_transform(&self, transform: &Transform) -> CarlaResult<()> {
        self.0.set_transform(transform)
    }

    fn velocity(&self) -> Vector3D {
        self.0.velocity()
    }

    fn angular_velocity(&self) -> Vector3D {
        self.0.angular_velocity()
    }

    fn acceleration(&self) -> Vector3D {
        self.0.acceleration()
    }

    fn is_alive(&self) -> bool {
        self.0.is_alive()
    }

    fn set_simulate_physics(&self, enabled: bool) -> CarlaResult<()> {
        self.0.set_simulate_physics(enabled)
    }

    fn add_impulse(&self, impulse: &Vector3D) -> CarlaResult<()> {
        self.0.add_impulse(impulse)
    }

    fn add_force(&self, force: &Vector3D) -> CarlaResult<()> {
        self.0.add_force(force)
    }

    fn add_torque(&self, torque: &Vector3D) -> CarlaResult<()> {
        self.0.add_torque(torque)
    }

    fn bounding_box(&self) -> crate::geom::BoundingBox {
        self.0.bounding_box()
    }
}

impl SensorT for LiDAR {
    fn start_listening(&self) {
        self.0.start_listening()
    }

    fn stop_listening(&self) {
        self.0.stop_listening()
    }

    fn is_listening(&self) -> bool {
        self.0.is_listening()
    }

    fn has_new_data(&self) -> bool {
        self.0.has_new_data()
    }

    fn get_attribute(&self, name: &str) -> Option<String> {
        self.0.get_attribute(name)
    }

    fn enable_recording(&self, filename: &str) -> CarlaResult<()> {
        self.0.enable_recording(filename)
    }

    fn disable_recording(&self) -> CarlaResult<()> {
        self.0.disable_recording()
    }
}
