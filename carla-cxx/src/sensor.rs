//! Sensor actor implementation for CARLA.

use crate::ffi::{self, Actor, Sensor};
use anyhow::Result;
use cxx::SharedPtr;
use std::sync::{Arc, Mutex};

/// High-level wrapper for CARLA Sensor
pub struct SensorWrapper {
    inner: SharedPtr<Sensor>,
    callback_data: Arc<Mutex<Option<Box<dyn FnMut(SensorData)>>>>,
    polling_active: Arc<Mutex<bool>>,
}

impl SensorWrapper {
    /// Create a SensorWrapper from an Actor (performs cast)
    pub fn from_actor(actor: &Actor) -> Option<Self> {
        let sensor_ptr = ffi::Actor_CastToSensor(actor);
        if sensor_ptr.is_null() {
            None
        } else {
            Some(Self {
                inner: sensor_ptr,
                callback_data: Arc::new(Mutex::new(None)),
                polling_active: Arc::new(Mutex::new(false)),
            })
        }
    }

    /// Start listening for sensor data with a callback
    /// Uses a blocking approach since CXX types are not Send-safe
    pub fn listen<F>(&mut self, callback: F) -> Result<()>
    where
        F: FnMut(SensorData) + 'static,
    {
        // Start the sensor listening (this sets up the C++ callback)
        ffi::Sensor_Listen(&self.inner);

        // Store callback and start polling
        {
            let mut callback_guard = self.callback_data.lock().unwrap();
            *callback_guard = Some(Box::new(callback));
        }

        // Set polling as active
        {
            let mut polling_guard = self.polling_active.lock().unwrap();
            *polling_guard = true;
        }

        Ok(())
    }

    /// Poll for sensor data and invoke callback if new data is available
    /// This should be called regularly from the main thread
    pub fn poll(&self) -> Result<bool> {
        if !*self.polling_active.lock().unwrap() {
            return Ok(false);
        }

        if ffi::Sensor_HasNewData(&self.inner) {
            if let Some(data) = Self::poll_sensor_data(&self.inner) {
                if let Ok(mut callback_guard) = self.callback_data.try_lock() {
                    if let Some(ref mut callback) = *callback_guard {
                        callback(data);
                        return Ok(true);
                    }
                }
            }
        }
        Ok(false)
    }

    /// Poll sensor data from the FFI layer
    fn poll_sensor_data(sensor: &SharedPtr<Sensor>) -> Option<SensorData> {
        // Try to get image data first
        let image_info = ffi::Sensor_GetLastImageData(sensor);
        if image_info.width > 0 && image_info.height > 0 {
            let mut buffer = vec![0u8; image_info.data_size];
            if ffi::Sensor_GetImageDataBuffer(sensor, &mut buffer) {
                return Some(SensorData::Image(ImageData {
                    width: image_info.width,
                    height: image_info.height,
                    data: buffer,
                    fov: image_info.fov,
                }));
            }
        }

        // Try LiDAR data
        let lidar_points = ffi::Sensor_GetLastLiDARData(sensor);
        if !lidar_points.is_empty() {
            let points: Vec<LiDARPoint> = lidar_points
                .into_iter()
                .map(|p| LiDARPoint {
                    x: p.x,
                    y: p.y,
                    z: p.z,
                    intensity: p.intensity,
                })
                .collect();
            return Some(SensorData::LiDAR(LiDARData::new(points)));
        }

        // Try Radar data
        let radar_detections = ffi::Sensor_GetLastRadarData(sensor);
        if !radar_detections.is_empty() {
            let detections: Vec<RadarDetection> = radar_detections
                .into_iter()
                .map(|d| RadarDetection {
                    velocity: d.velocity,
                    azimuth: d.azimuth,
                    altitude: d.altitude,
                    depth: d.depth,
                })
                .collect();
            return Some(SensorData::Radar(RadarData { detections }));
        }

        // Try IMU data
        let imu_data = ffi::Sensor_GetLastIMUData(sensor);
        if imu_data.accelerometer_x != 0.0
            || imu_data.accelerometer_y != 0.0
            || imu_data.accelerometer_z != 0.0
        {
            return Some(SensorData::IMU(IMUData {
                accelerometer: (
                    imu_data.accelerometer_x,
                    imu_data.accelerometer_y,
                    imu_data.accelerometer_z,
                ),
                gyroscope: (
                    imu_data.gyroscope_x,
                    imu_data.gyroscope_y,
                    imu_data.gyroscope_z,
                ),
                compass: imu_data.compass,
            }));
        }

        // Try GNSS data
        let gnss_data = ffi::Sensor_GetLastGNSSData(sensor);
        if gnss_data.latitude != 0.0 || gnss_data.longitude != 0.0 {
            return Some(SensorData::GNSS(GNSSData {
                latitude: gnss_data.latitude,
                longitude: gnss_data.longitude,
                altitude: gnss_data.altitude,
            }));
        }

        None
    }

    /// Stop listening for sensor data
    pub fn stop(&self) {
        // Stop the polling thread
        {
            let mut polling_guard = self.polling_active.lock().unwrap();
            *polling_guard = false;
        }

        // Stop the sensor
        ffi::Sensor_Stop(&self.inner);
    }

    /// Check if the sensor is currently listening
    pub fn is_listening(&self) -> bool {
        ffi::Sensor_IsListening(&self.inner)
    }

    /// Get the latest sensor data without setting up a callback
    /// Returns None if no data is available
    pub fn get_latest_data(&self) -> Option<SensorData> {
        Self::poll_sensor_data(&self.inner)
    }

    /// Check if new sensor data is available
    pub fn has_new_data(&self) -> bool {
        ffi::Sensor_HasNewData(&self.inner)
    }
}

impl Drop for SensorWrapper {
    fn drop(&mut self) {
        self.stop();
    }
}

/// Sensor data types
#[derive(Debug, Clone)]
pub enum SensorData {
    /// RGB camera image data
    Image(ImageData),
    /// LiDAR point cloud data
    LiDAR(LiDARData),
    /// Radar detection data
    Radar(RadarData),
    /// IMU sensor data
    IMU(IMUData),
    /// GNSS/GPS data
    GNSS(GNSSData),
    /// Raw binary data
    Raw(Vec<u8>),
}

/// RGB camera image data
#[derive(Debug, Clone)]
pub struct ImageData {
    /// Image width in pixels
    pub width: u32,
    /// Image height in pixels
    pub height: u32,
    /// Raw pixel data (BGRA format)
    pub data: Vec<u8>,
    /// Field of view in degrees
    pub fov: f32,
}

/// LiDAR point cloud data
#[derive(Debug, Clone)]
pub struct LiDARData {
    /// Number of points
    pub point_count: u32,
    /// Point cloud data (x, y, z, intensity for each point)
    pub points: Vec<LiDARPoint>,
}

/// Individual LiDAR point
#[derive(Debug, Clone, Copy)]
pub struct LiDARPoint {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub intensity: f32,
}

/// Radar detection data
#[derive(Debug, Clone)]
pub struct RadarData {
    /// Detection points
    pub detections: Vec<RadarDetection>,
}

/// Individual radar detection
#[derive(Debug, Clone, Copy)]
pub struct RadarDetection {
    /// Velocity in m/s
    pub velocity: f32,
    /// Azimuth angle in radians
    pub azimuth: f32,
    /// Altitude angle in radians
    pub altitude: f32,
    /// Depth in meters
    pub depth: f32,
}

/// IMU sensor data
#[derive(Debug, Clone, Copy)]
pub struct IMUData {
    /// Accelerometer readings (m/s²)
    pub accelerometer: (f64, f64, f64),
    /// Gyroscope readings (rad/s)
    pub gyroscope: (f64, f64, f64),
    /// Compass heading in radians
    pub compass: f64,
}

/// GNSS/GPS sensor data
#[derive(Debug, Clone, Copy)]
pub struct GNSSData {
    /// Latitude in degrees
    pub latitude: f64,
    /// Longitude in degrees
    pub longitude: f64,
    /// Altitude in meters
    pub altitude: f64,
}

impl ImageData {
    /// Create a new ImageData
    pub fn new(width: u32, height: u32, data: Vec<u8>, fov: f32) -> Self {
        Self {
            width,
            height,
            data,
            fov,
        }
    }

    /// Get pixel data as RGBA (converting from BGRA)
    pub fn as_rgba(&self) -> Vec<u8> {
        let mut rgba_data = Vec::with_capacity(self.data.len());
        for chunk in self.data.chunks_exact(4) {
            // Convert BGRA to RGBA
            rgba_data.push(chunk[2]); // R
            rgba_data.push(chunk[1]); // G
            rgba_data.push(chunk[0]); // B
            rgba_data.push(chunk[3]); // A
        }
        rgba_data
    }
}

impl LiDARData {
    /// Create a new LiDARData
    pub fn new(points: Vec<LiDARPoint>) -> Self {
        let point_count = points.len() as u32;
        Self {
            point_count,
            points,
        }
    }

    /// Filter points by distance
    pub fn filter_by_distance(&self, max_distance: f32) -> Self {
        let filtered_points: Vec<_> = self
            .points
            .iter()
            .filter(|point| {
                let distance = (point.x * point.x + point.y * point.y + point.z * point.z).sqrt();
                distance <= max_distance
            })
            .copied()
            .collect();

        Self::new(filtered_points)
    }

    /// Filter points by intensity threshold
    pub fn filter_by_intensity(&self, min_intensity: f32) -> Self {
        let filtered_points: Vec<_> = self
            .points
            .iter()
            .filter(|point| point.intensity >= min_intensity)
            .copied()
            .collect();

        Self::new(filtered_points)
    }

    /// Get points within a bounding box
    pub fn get_points_in_box(
        &self,
        min_x: f32,
        max_x: f32,
        min_y: f32,
        max_y: f32,
        min_z: f32,
        max_z: f32,
    ) -> Self {
        let filtered_points: Vec<_> = self
            .points
            .iter()
            .filter(|point| {
                point.x >= min_x
                    && point.x <= max_x
                    && point.y >= min_y
                    && point.y <= max_y
                    && point.z >= min_z
                    && point.z <= max_z
            })
            .copied()
            .collect();

        Self::new(filtered_points)
    }
}

impl RadarData {
    /// Create a new RadarData
    pub fn new(detections: Vec<RadarDetection>) -> Self {
        Self { detections }
    }

    /// Filter detections by velocity threshold
    pub fn filter_by_velocity(&self, min_velocity: f32) -> Self {
        let filtered_detections: Vec<_> = self
            .detections
            .iter()
            .filter(|detection| detection.velocity.abs() >= min_velocity)
            .copied()
            .collect();

        Self::new(filtered_detections)
    }

    /// Filter detections by distance
    pub fn filter_by_distance(&self, max_distance: f32) -> Self {
        let filtered_detections: Vec<_> = self
            .detections
            .iter()
            .filter(|detection| detection.depth <= max_distance)
            .copied()
            .collect();

        Self::new(filtered_detections)
    }
}

impl IMUData {
    /// Create a new IMUData
    pub fn new(accelerometer: (f64, f64, f64), gyroscope: (f64, f64, f64), compass: f64) -> Self {
        Self {
            accelerometer,
            gyroscope,
            compass,
        }
    }

    /// Get the magnitude of acceleration
    pub fn acceleration_magnitude(&self) -> f64 {
        let (x, y, z) = self.accelerometer;
        (x * x + y * y + z * z).sqrt()
    }

    /// Get the magnitude of angular velocity
    pub fn angular_velocity_magnitude(&self) -> f64 {
        let (x, y, z) = self.gyroscope;
        (x * x + y * y + z * z).sqrt()
    }
}

impl GNSSData {
    /// Create a new GNSSData
    pub fn new(latitude: f64, longitude: f64, altitude: f64) -> Self {
        Self {
            latitude,
            longitude,
            altitude,
        }
    }

    /// Calculate distance to another GNSS position (in meters, using Haversine formula)
    pub fn distance_to(&self, other: &GNSSData) -> f64 {
        const EARTH_RADIUS: f64 = 6371000.0; // Earth radius in meters

        let lat1_rad = self.latitude.to_radians();
        let lat2_rad = other.latitude.to_radians();
        let delta_lat = (other.latitude - self.latitude).to_radians();
        let delta_lon = (other.longitude - self.longitude).to_radians();

        let a = (delta_lat / 2.0).sin().powi(2)
            + lat1_rad.cos() * lat2_rad.cos() * (delta_lon / 2.0).sin().powi(2);
        let c = 2.0 * a.sqrt().atan2((1.0 - a).sqrt());

        EARTH_RADIUS * c
    }
}
