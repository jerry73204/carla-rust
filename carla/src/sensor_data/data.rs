//! Base sensor data types and traits.

use crate::{geom::Transform, time::Timestamp};
use ndarray::Array3;

/// Base trait for all sensor data types.
pub trait SensorData: Send + Sync + std::fmt::Debug {
    /// Get the sensor data timestamp.
    fn timestamp(&self) -> Timestamp;

    /// Get the transform of the sensor when this data was captured.
    fn transform(&self) -> Transform;

    /// Get the frame number.
    fn frame(&self) -> u64 {
        self.timestamp().frame()
    }

    /// Get the sensor ID that produced this data.
    fn sensor_id(&self) -> u32;

    /// Get the size of the data in bytes.
    fn size(&self) -> usize;
}

/// Image sensor data (RGB, depth, semantic segmentation, etc.).
#[derive(Debug, Clone)]
pub struct ImageData {
    /// Image timestamp
    pub timestamp: Timestamp,
    /// Sensor transform when captured
    pub transform: Transform,
    /// Sensor ID
    pub sensor_id: u32,
    /// Image width in pixels
    pub width: u32,
    /// Image height in pixels
    pub height: u32,
    /// Field of view in degrees
    pub fov: f32,
    /// Raw image data (BGRA format)
    pub data: Vec<u8>,
}

impl ImageData {
    /// Create ImageData from carla-sys ImageData
    pub fn from_cxx(cxx_data: carla_sys::ImageData) -> Self {
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
            width: cxx_data.width,
            height: cxx_data.height,
            fov: cxx_data.fov,
            data: cxx_data.data,
        }
    }
}

impl SensorData for ImageData {
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
        self.data.len()
    }
}

impl ImageData {
    /// Convert raw BGRA data to RGB array.
    pub fn to_rgb_array(&self) -> Array3<u8> {
        let mut rgb_data = Array3::zeros((self.height as usize, self.width as usize, 3));

        for y in 0..self.height as usize {
            for x in 0..self.width as usize {
                let idx = (y * self.width as usize + x) * 4; // BGRA = 4 bytes per pixel
                if idx + 2 < self.data.len() {
                    rgb_data[[y, x, 0]] = self.data[idx + 2]; // R
                    rgb_data[[y, x, 1]] = self.data[idx + 1]; // G
                    rgb_data[[y, x, 2]] = self.data[idx + 0]; // B
                }
            }
        }

        rgb_data
    }

    /// Convert to grayscale array.
    pub fn to_grayscale_array(&self) -> ndarray::Array2<u8> {
        let mut gray_data = ndarray::Array2::zeros((self.height as usize, self.width as usize));

        for y in 0..self.height as usize {
            for x in 0..self.width as usize {
                let idx = (y * self.width as usize + x) * 4; // BGRA = 4 bytes per pixel
                if idx + 2 < self.data.len() {
                    let r = self.data[idx + 2] as f32;
                    let g = self.data[idx + 1] as f32;
                    let b = self.data[idx + 0] as f32;
                    // Standard luminance formula
                    let gray = (0.299 * r + 0.587 * g + 0.114 * b) as u8;
                    gray_data[[y, x]] = gray;
                }
            }
        }

        gray_data
    }

    /// Save image to file (requires image crate feature).
    pub fn save_to_disk(&self, path: &str) -> Result<(), Box<dyn std::error::Error>> {
        todo!()
    }
}
