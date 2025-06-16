//! LiDAR sensor implementation.

use crate::{
    client::Sensor,
    error::{CarlaResult, SensorError},
    sensor::LiDARData,
    traits::ActorT,
};

/// LiDAR sensor that captures 3D point clouds.
pub struct LiDARSensor {
    sensor: Sensor,
}

impl LiDARSensor {
    /// Create a LiDAR sensor from a sensor.
    pub fn from_sensor(sensor: Sensor) -> CarlaResult<Self> {
        let type_id = sensor.get_type_id();
        if !type_id.contains("lidar") && !type_id.contains("velodyne") {
            return Err(crate::error::CarlaError::Actor(
                crate::error::ActorError::InvalidType {
                    expected: "LiDAR".to_string(),
                    actual: type_id,
                },
            ));
        }

        Ok(Self { sensor })
    }

    /// Get the underlying sensor.
    pub fn sensor(&self) -> &Sensor {
        &self.sensor
    }

    /// Check if the LiDAR is currently listening for data.
    pub fn is_listening(&self) -> bool {
        self.sensor.is_listening()
    }

    /// Start listening for LiDAR data.
    pub fn listen<F>(&mut self, callback: F) -> Result<(), SensorError>
    where
        F: Fn(Vec<u8>) + Send + 'static,
    {
        self.sensor.listen(callback)
    }

    /// Stop listening for LiDAR data.
    pub fn stop(&mut self) {
        self.sensor.stop();
    }

    /// Check if new LiDAR data is available.
    pub fn has_new_data(&self) -> bool {
        self.sensor.inner.has_new_data()
    }

    /// Get the last captured LiDAR data with full metadata.
    ///
    /// Returns None if no data is available or if the sensor is not listening.
    pub fn get_lidar_data(&self) -> Option<LiDARData> {
        let cxx_data = self.sensor.inner.get_last_lidar_data_full();
        if cxx_data.points.is_empty() {
            None
        } else {
            Some(LiDARData::from_cxx(cxx_data))
        }
    }

    /// Get the last captured LiDAR points only (without metadata).
    ///
    /// This is a faster method that only returns the point cloud data.
    pub fn get_lidar_points(&self) -> Vec<crate::sensor::LiDARPoint> {
        let cxx_data = self.sensor.inner.get_last_lidar_data();
        cxx_data
            .points
            .iter()
            .map(|p| crate::sensor::LiDARPoint::new(p.x, p.y, p.z, p.intensity))
            .collect()
    }

    /// Get LiDAR statistics from the last measurement.
    pub fn get_statistics(&self) -> Option<crate::sensor::LiDARStatistics> {
        self.get_lidar_data().map(|data| data.get_statistics())
    }

    /// Filter points by distance range from the last measurement.
    pub fn filter_by_distance(
        &self,
        min_distance: f32,
        max_distance: f32,
    ) -> Vec<crate::sensor::LiDARPoint> {
        if let Some(data) = self.get_lidar_data() {
            data.filter_by_distance(min_distance, max_distance)
        } else {
            Vec::new()
        }
    }

    /// Filter points by intensity range from the last measurement.
    pub fn filter_by_intensity(
        &self,
        min_intensity: f32,
        max_intensity: f32,
    ) -> Vec<crate::sensor::LiDARPoint> {
        if let Some(data) = self.get_lidar_data() {
            data.filter_by_intensity(min_intensity, max_intensity)
        } else {
            Vec::new()
        }
    }

    /// Filter points by height (Z coordinate) range from the last measurement.
    pub fn filter_by_height(
        &self,
        min_height: f32,
        max_height: f32,
    ) -> Vec<crate::sensor::LiDARPoint> {
        if let Some(data) = self.get_lidar_data() {
            data.filter_by_height(min_height, max_height)
        } else {
            Vec::new()
        }
    }
}

impl std::fmt::Debug for LiDARSensor {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("LiDARSensor")
            .field("sensor_id", &self.sensor.id())
            .field("is_listening", &self.is_listening())
            .finish()
    }
}

impl ActorT for LiDARSensor {
    fn get_id(&self) -> crate::client::ActorId {
        self.sensor.get_id()
    }

    fn get_type_id(&self) -> String {
        self.sensor.get_type_id()
    }

    fn get_transform(&self) -> crate::geom::Transform {
        self.sensor.get_transform()
    }

    fn set_transform(&self, transform: &crate::geom::Transform) -> CarlaResult<()> {
        self.sensor.set_transform(transform)
    }

    fn get_velocity(&self) -> crate::geom::Vector3D {
        self.sensor.get_velocity()
    }

    fn get_angular_velocity(&self) -> crate::geom::Vector3D {
        self.sensor.get_angular_velocity()
    }

    fn get_acceleration(&self) -> crate::geom::Vector3D {
        self.sensor.get_acceleration()
    }

    fn is_alive(&self) -> bool {
        self.sensor.is_alive()
    }

    fn destroy(&self) -> CarlaResult<()> {
        self.sensor.destroy()
    }

    fn set_simulate_physics(&self, enabled: bool) -> CarlaResult<()> {
        self.sensor.set_simulate_physics(enabled)
    }

    fn add_impulse(&self, impulse: &crate::geom::Vector3D) -> CarlaResult<()> {
        self.sensor.add_impulse(impulse)
    }

    fn add_force(&self, force: &crate::geom::Vector3D) -> CarlaResult<()> {
        self.sensor.add_force(force)
    }

    fn add_torque(&self, torque: &crate::geom::Vector3D) -> CarlaResult<()> {
        self.sensor.add_torque(torque)
    }
}
