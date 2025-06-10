use crate::sensor::{SensorData, SensorDataBase};
use anyhow::{anyhow, Result};
use carla_sys::*;
use nalgebra::Vector2;
use ndarray::ArrayView2;
use std::{ptr, slice};

/// Represents a single optical flow vector.
#[derive(Clone, Debug)]
pub struct OpticalFlowPixel {
    /// Horizontal flow component (x-direction)
    pub x: f32,
    /// Vertical flow component (y-direction)
    pub y: f32,
}

impl OpticalFlowPixel {
    /// Get the magnitude of the optical flow vector.
    pub fn magnitude(&self) -> f32 {
        (self.x * self.x + self.y * self.y).sqrt()
    }

    /// Get the direction of the optical flow vector in radians.
    pub fn direction(&self) -> f32 {
        self.y.atan2(self.x)
    }

    /// Get the direction of the optical flow vector in degrees.
    pub fn direction_degrees(&self) -> f32 {
        self.direction().to_degrees()
    }

    /// Convert to nalgebra Vector2.
    pub fn to_vector2(&self) -> Vector2<f32> {
        Vector2::new(self.x, self.y)
    }

    /// Create from nalgebra Vector2.
    pub fn from_vector2(vec: Vector2<f32>) -> Self {
        Self { x: vec.x, y: vec.y }
    }
}

/// Represents optical flow data from a camera sensor.
/// Optical flow captures the motion of objects in the image between frames.
#[derive(Clone, Debug)]
pub struct OpticalFlow {
    inner: *mut carla_optical_flow_data_t,
}

impl OpticalFlow {
    /// Create an OpticalFlow from a raw C pointer.
    /// 
    /// # Safety
    /// The pointer must be valid and not null.
    pub(crate) fn from_raw_ptr(ptr: *mut carla_optical_flow_data_t) -> Result<Self> {
        if ptr.is_null() {
            return Err(anyhow!("Null optical flow data pointer"));
        }
        Ok(Self { inner: ptr })
    }

    /// Get the width of the optical flow image.
    pub fn width(&self) -> usize {
        unsafe { carla_optical_flow_get_width(self.inner) }
    }

    /// Get the height of the optical flow image.
    pub fn height(&self) -> usize {
        unsafe { carla_optical_flow_get_height(self.inner) }
    }

    /// Get the field of view angle of the camera.
    pub fn fov_angle(&self) -> f32 {
        unsafe { carla_optical_flow_get_fov_angle(self.inner) }
    }

    /// Get the optical flow data as a slice of OpticalFlowPixel.
    pub fn as_slice(&self) -> &[OpticalFlowPixel] {
        let len = self.len();
        let data = unsafe { carla_optical_flow_get_data(self.inner) };
        
        if data.is_null() || len == 0 {
            &[]
        } else {
            // Convert from C struct array to Rust struct slice
            let c_pixels = unsafe { slice::from_raw_parts(data, len) };
            // This is safe because OpticalFlowPixel has the same memory layout as carla_optical_flow_pixel_t
            unsafe { slice::from_raw_parts(c_pixels.as_ptr() as *const OpticalFlowPixel, len) }
        }
    }

    /// Get the optical flow data as a 2D array view.
    /// Returns ArrayView2 with shape (height, width).
    pub fn as_array(&self) -> ArrayView2<'_, OpticalFlowPixel> {
        let height = self.height();
        let width = self.width();
        ArrayView2::from_shape((height, width), self.as_slice()).unwrap()
    }

    /// Get the total number of pixels in the optical flow image.
    pub fn len(&self) -> usize {
        unsafe { carla_optical_flow_get_size(self.inner) }
    }

    /// Check if the optical flow data is empty.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Get a specific pixel's optical flow vector.
    pub fn get_pixel(&self, x: usize, y: usize) -> Option<OpticalFlowPixel> {
        if x >= self.width() || y >= self.height() {
            return None;
        }
        
        let index = y * self.width() + x;
        self.as_slice().get(index).cloned()
    }

    /// Analyze the optical flow and return statistics.
    /// Returns (mean_magnitude, max_magnitude, dominant_direction_radians).
    pub fn analyze_optical_flow(&self) -> (f32, f32, f32) {
        let mut mean_magnitude = 0.0;
        let mut max_magnitude = 0.0;
        let mut dominant_direction = 0.0;
        
        unsafe {
            carla_image_analyze_optical_flow(
                self.inner,
                &mut mean_magnitude,
                &mut max_magnitude,
                &mut dominant_direction,
            );
        }
        
        (mean_magnitude, max_magnitude, dominant_direction)
    }

    /// Get the magnitude of optical flow at each pixel as a vector.
    pub fn get_magnitude_field(&self) -> Vec<f32> {
        self.as_slice()
            .iter()
            .map(|pixel| pixel.magnitude())
            .collect()
    }

    /// Get the direction of optical flow at each pixel as a vector (in radians).
    pub fn get_direction_field(&self) -> Vec<f32> {
        self.as_slice()
            .iter()
            .map(|pixel| pixel.direction())
            .collect()
    }

    /// Filter optical flow vectors by magnitude threshold.
    /// Returns pixel coordinates and flow vectors for pixels above the threshold.
    pub fn filter_by_magnitude(&self, threshold: f32) -> Vec<(usize, usize, OpticalFlowPixel)> {
        let mut result = Vec::new();
        let width = self.width();
        
        for (index, pixel) in self.as_slice().iter().enumerate() {
            if pixel.magnitude() >= threshold {
                let x = index % width;
                let y = index / width;
                result.push((x, y, pixel.clone()));
            }
        }
        
        result
    }

    /// Calculate optical flow statistics for a specific region.
    /// Returns (mean_magnitude, max_magnitude, pixel_count) for the region.
    pub fn analyze_region(&self, x_min: usize, y_min: usize, x_max: usize, y_max: usize) -> (f32, f32, usize) {
        let width = self.width();
        let height = self.height();
        
        let x_max = x_max.min(width);
        let y_max = y_max.min(height);
        
        if x_min >= x_max || y_min >= y_max {
            return (0.0, 0.0, 0);
        }
        
        let mut total_magnitude = 0.0;
        let mut max_magnitude = 0.0;
        let mut pixel_count = 0;
        
        for y in y_min..y_max {
            for x in x_min..x_max {
                if let Some(pixel) = self.get_pixel(x, y) {
                    let magnitude = pixel.magnitude();
                    total_magnitude += magnitude;
                    max_magnitude = max_magnitude.max(magnitude);
                    pixel_count += 1;
                }
            }
        }
        
        let mean_magnitude = if pixel_count > 0 {
            total_magnitude / (pixel_count as f32)
        } else {
            0.0
        };
        
        (mean_magnitude, max_magnitude, pixel_count)
    }

    /// Detect motion hotspots by finding regions with high optical flow magnitude.
    /// Returns a list of (center_x, center_y, average_magnitude) for detected hotspots.
    pub fn detect_motion_hotspots(&self, window_size: usize, magnitude_threshold: f32) -> Vec<(usize, usize, f32)> {
        let width = self.width();
        let height = self.height();
        let half_window = window_size / 2;
        let mut hotspots = Vec::new();
        
        for y in half_window..(height - half_window) {
            for x in half_window..(width - half_window) {
                let (mean_magnitude, _, _) = self.analyze_region(
                    x - half_window,
                    y - half_window,
                    x + half_window + 1,
                    y + half_window + 1,
                );
                
                if mean_magnitude >= magnitude_threshold {
                    hotspots.push((x, y, mean_magnitude));
                }
            }
        }
        
        hotspots
    }

    /// Convert optical flow to HSV color representation for visualization.
    /// Direction maps to hue, magnitude maps to value, saturation is constant.
    pub fn to_hsv_visualization(&self) -> Vec<(f32, f32, f32)> {
        let (_, max_magnitude, _) = self.analyze_optical_flow();
        
        self.as_slice()
            .iter()
            .map(|pixel| {
                let magnitude = pixel.magnitude();
                let direction = pixel.direction();
                
                // Normalize direction to [0, 1] for hue
                let hue = (direction + std::f32::consts::PI) / (2.0 * std::f32::consts::PI);
                let saturation = 1.0; // Full saturation for vibrant colors
                let value = if max_magnitude > 0.0 {
                    magnitude / max_magnitude
                } else {
                    0.0
                };
                
                (hue, saturation, value)
            })
            .collect()
    }
}

impl SensorDataBase for OpticalFlow {
    fn raw_ptr(&self) -> *mut carla_sensor_data_t {
        self.inner as *mut carla_sensor_data_t
    }
}

impl TryFrom<SensorData> for OpticalFlow {
    type Error = SensorData;

    fn try_from(value: SensorData) -> std::result::Result<Self, Self::Error> {
        // Check if the sensor data is actually optical flow data
        let data_type = value.data_type();
        if data_type == carla_sensor_data_type_t_CARLA_SENSOR_DATA_OPTICAL_FLOW {
            let optical_flow_ptr = unsafe { carla_sensor_data_as_optical_flow(value.inner) };
            if !optical_flow_ptr.is_null() {
                return Ok(OpticalFlow { inner: optical_flow_ptr });
            }
        }
        Err(value)
    }
}

impl Drop for OpticalFlow {
    fn drop(&mut self) {
        // Note: Optical flow data is managed by the sensor data lifecycle
        // We don't need to free it separately
        self.inner = ptr::null_mut();
    }
}

// SAFETY: OpticalFlow wraps a thread-safe C API
unsafe impl Send for OpticalFlow {}
unsafe impl Sync for OpticalFlow {}

// Ensure OpticalFlowPixel has the same memory layout as carla_optical_flow_pixel_t
#[cfg(test)]
mod tests {
    use super::*;
    use std::mem;

    #[test]
    fn test_optical_flow_pixel_layout() {
        // Ensure the Rust struct has the same layout as the C struct
        assert_eq!(mem::size_of::<OpticalFlowPixel>(), mem::size_of::<carla_optical_flow_pixel_t>());
        assert_eq!(mem::align_of::<OpticalFlowPixel>(), mem::align_of::<carla_optical_flow_pixel_t>());
    }

    #[test]
    fn test_optical_flow_pixel_operations() {
        let pixel = OpticalFlowPixel { x: 3.0, y: 4.0 };
        
        // Test magnitude calculation
        assert_eq!(pixel.magnitude(), 5.0);
        
        // Test direction calculation
        let direction = pixel.direction();
        assert!((direction - (4.0_f32).atan2(3.0)).abs() < 1e-6);
        
        // Test conversion to/from Vector2
        let vec = pixel.to_vector2();
        let pixel2 = OpticalFlowPixel::from_vector2(vec);
        assert_eq!(pixel.x, pixel2.x);
        assert_eq!(pixel.y, pixel2.y);
    }
}