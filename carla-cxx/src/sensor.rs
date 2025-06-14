//! Sensor actor implementation for CARLA.

use crate::ffi::{self, Actor, Sensor};
use cxx::SharedPtr;
use std::sync::{Arc, Mutex};

/// High-level wrapper for CARLA Sensor
pub struct SensorWrapper {
    inner: SharedPtr<Sensor>,
    _callback_data: Arc<Mutex<Option<Box<dyn FnMut(&[u8]) + Send>>>>,
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
                _callback_data: Arc::new(Mutex::new(None)),
            })
        }
    }

    /// Start listening for sensor data with a callback
    /// Note: Callback functionality is TODO - CXX callbacks are complex
    pub fn listen<F>(&mut self, _callback: F)
    where
        F: FnMut(&[u8]) + Send + 'static,
    {
        // TODO: Implement sensor callbacks - requires more complex CXX bridge design
        // For now, this is a placeholder
        unimplemented!("Sensor callbacks need more sophisticated CXX bridge design");
    }

    /// Stop listening for sensor data
    pub fn stop(&self) {
        ffi::Sensor_Stop(&self.inner);
    }

    /// Check if the sensor is currently listening
    pub fn is_listening(&self) -> bool {
        ffi::Sensor_IsListening(&self.inner)
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
}
