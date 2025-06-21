//! LiDAR sensor implementation.

use crate::{
    actor::{ActorFfi, Sensor, SensorFfi},
    sensor_data::{LiDARData, LiDARPoint, LiDARStatistics, SemanticLiDARData},
};

/// LiDAR sensor for point cloud data.
#[derive(Debug)]
pub struct LiDAR(pub(crate) Sensor);

impl LiDAR {
    /// Get the last LiDAR data from the sensor with full metadata.
    pub fn last_lidar_data(&self) -> Option<LiDARData> {
        let cxx_data = self.as_sensor_ffi().get_last_lidar_data_full();
        if cxx_data.points.is_empty() {
            None
        } else {
            Some(LiDARData::from_cxx(cxx_data))
        }
    }

    /// Get the last LiDAR points only (without metadata) for performance.
    pub fn last_lidar_points(&self) -> Vec<LiDARPoint> {
        let cxx_data = self.as_sensor_ffi().get_last_lidar_data();
        cxx_data
            .points
            .iter()
            .map(|p| LiDARPoint::new(p.x, p.y, p.z, p.intensity))
            .collect()
    }

    /// Get the last Semantic LiDAR data from the sensor.
    pub fn last_semantic_lidar_data(&self) -> Option<SemanticLiDARData> {
        let cxx_data = self.as_sensor_ffi().get_last_semantic_lidar_data();
        if cxx_data.detections.is_empty() {
            None
        } else {
            Some(SemanticLiDARData::from_cxx(cxx_data))
        }
    }

    /// Get LiDAR statistics from the last measurement.
    pub fn statistics(&self) -> Option<LiDARStatistics> {
        self.last_lidar_data().map(|data| data.statistics())
    }

    /// Filter points by distance range from the last measurement.
    pub fn filter_by_distance(&self, min_distance: f32, max_distance: f32) -> Vec<LiDARPoint> {
        if let Some(data) = self.last_lidar_data() {
            data.filter_by_distance(min_distance, max_distance)
        } else {
            Vec::new()
        }
    }

    /// Filter points by intensity range from the last measurement.
    pub fn filter_by_intensity(&self, min_intensity: f32, max_intensity: f32) -> Vec<LiDARPoint> {
        if let Some(data) = self.last_lidar_data() {
            data.filter_by_intensity(min_intensity, max_intensity)
        } else {
            Vec::new()
        }
    }

    /// Filter points by height (Z coordinate) range from the last measurement.
    pub fn filter_by_height(&self, min_height: f32, max_height: f32) -> Vec<LiDARPoint> {
        if let Some(data) = self.last_lidar_data() {
            data.filter_by_height(min_height, max_height)
        } else {
            Vec::new()
        }
    }
}

impl SensorFfi for LiDAR {
    fn as_sensor_ffi(&self) -> &carla_sys::SensorWrapper {
        self.0.as_sensor_ffi()
    }
}

// Implement conversion traits using the macro
crate::impl_sensor_conversions!(LiDAR, is_lidar);
