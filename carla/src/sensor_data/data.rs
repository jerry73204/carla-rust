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
    pub fn save_to_disk(&self, _path: &str) -> Result<(), Box<dyn std::error::Error>> {
        todo!()
    }
}

/// Image format support detection
pub mod image_io {
    /// Check if PNG format is supported
    pub fn has_png_support() -> bool {
        // In Rust, we would use the 'image' crate which supports PNG by default
        cfg!(feature = "png")
    }

    /// Check if JPEG format is supported
    pub fn has_jpeg_support() -> bool {
        // In Rust, we would use the 'image' crate which supports JPEG by default
        cfg!(feature = "jpeg")
    }

    /// Check if TIFF format is supported
    pub fn has_tiff_support() -> bool {
        // In Rust, we would use the 'image' crate which supports TIFF optionally
        cfg!(feature = "tiff")
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        geom::{Location, Rotation, Transform},
        time::Timestamp,
    };

    // Helper function to create test image data
    fn create_test_image(width: u32, height: u32) -> ImageData {
        let data_size = (width * height * 4) as usize; // BGRA = 4 bytes per pixel
        ImageData {
            timestamp: Timestamp::new(100, 10.0, 0.016, 1234567890.0),
            transform: Transform::new(Location::new(0.0, 0.0, 0.0), Rotation::new(0.0, 0.0, 0.0)),
            sensor_id: 1,
            width,
            height,
            fov: 90.0,
            data: vec![0u8; data_size],
        }
    }

    #[test]
    fn test_image_format_support() {
        // Corresponds to C++ TEST(image, support)
        println!("PNG support = {}", image_io::has_png_support());
        println!("JPEG support = {}", image_io::has_jpeg_support());
        println!("TIFF support = {}", image_io::has_tiff_support());

        // These tests just verify the functions exist and return bool
        let _ = image_io::has_png_support();
        let _ = image_io::has_jpeg_support();
        let _ = image_io::has_tiff_support();
    }

    #[test]
    fn test_image_data_creation() {
        let img = create_test_image(640, 480);

        assert_eq!(img.width, 640);
        assert_eq!(img.height, 480);
        assert_eq!(img.fov, 90.0);
        assert_eq!(img.sensor_id, 1);
        assert_eq!(img.data.len(), 640 * 480 * 4);
        assert_eq!(img.size(), 640 * 480 * 4);
    }

    #[test]
    fn test_rgb_conversion() {
        let mut img = create_test_image(2, 2);

        // Set specific BGRA values
        // Pixel (0,0): B=10, G=20, R=30, A=255
        img.data[0] = 10; // B
        img.data[1] = 20; // G
        img.data[2] = 30; // R
        img.data[3] = 255; // A

        // Pixel (0,1): B=40, G=50, R=60, A=255
        img.data[4] = 40; // B
        img.data[5] = 50; // G
        img.data[6] = 60; // R
        img.data[7] = 255; // A

        let rgb_array = img.to_rgb_array();

        assert_eq!(rgb_array[[0, 0, 0]], 30); // R
        assert_eq!(rgb_array[[0, 0, 1]], 20); // G
        assert_eq!(rgb_array[[0, 0, 2]], 10); // B

        assert_eq!(rgb_array[[0, 1, 0]], 60); // R
        assert_eq!(rgb_array[[0, 1, 1]], 50); // G
        assert_eq!(rgb_array[[0, 1, 2]], 40); // B
    }

    #[test]
    fn test_grayscale_conversion() {
        let mut img = create_test_image(2, 2);

        // Set specific BGRA values
        // Pixel (0,0): B=100, G=150, R=200, A=255
        img.data[0] = 100; // B
        img.data[1] = 150; // G
        img.data[2] = 200; // R
        img.data[3] = 255; // A

        let gray_array = img.to_grayscale_array();

        // Expected: 0.299 * 200 + 0.587 * 150 + 0.114 * 100 = 159.25 ≈ 159
        assert!((gray_array[[0, 0]] as i32 - 159).abs() <= 1);
    }

    #[test]
    fn test_sensor_data_trait() {
        let img = create_test_image(100, 100);

        // Test SensorData trait methods
        assert_eq!(img.timestamp().frame(), 100);
        assert_eq!(img.frame(), 100);
        assert_eq!(img.sensor_id(), 1);
        assert_eq!(img.size(), 100 * 100 * 4);

        let transform = img.transform();
        assert_eq!(transform.location.x, 0.0);
        assert_eq!(transform.location.y, 0.0);
        assert_eq!(transform.location.z, 0.0);
    }
}
