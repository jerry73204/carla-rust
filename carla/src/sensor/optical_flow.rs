//! Optical Flow camera sensor implementations.

use crate::{
    geom::{Transform, Vector2D},
    sensor::SensorData,
    time::Timestamp,
};

/// Optical Flow camera sensor data.
#[derive(Debug, Clone)]
pub struct OpticalFlowData {
    /// Sensor timestamp
    pub timestamp: Timestamp,
    /// Sensor transform when captured
    pub transform: Transform,
    /// Sensor ID
    pub sensor_id: u32,
    /// Image width in pixels
    pub width: u32,
    /// Image height in pixels
    pub height: u32,
    /// Horizontal field of view in degrees
    pub fov: f32,
    /// Motion vectors for each pixel
    pub flow_vectors: Vec<OpticalFlowVector>,
}

/// Individual optical flow vector for a pixel.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OpticalFlowVector {
    /// X pixel coordinate
    pub x: u32,
    /// Y pixel coordinate
    pub y: u32,
    /// Motion vector in X direction (pixels per frame)
    pub flow_x: f32,
    /// Motion vector in Y direction (pixels per frame)
    pub flow_y: f32,
}

impl OpticalFlowVector {
    /// Create a new optical flow vector.
    pub fn new(x: u32, y: u32, flow_x: f32, flow_y: f32) -> Self {
        Self {
            x,
            y,
            flow_x,
            flow_y,
        }
    }

    /// Get the pixel coordinates as a tuple.
    pub fn pixel_coords(&self) -> (u32, u32) {
        (self.x, self.y)
    }

    /// Get the flow vector as a Vector2D.
    pub fn flow_vector(&self) -> Vector2D {
        Vector2D::new(self.flow_x, self.flow_y)
    }

    /// Get the magnitude of the flow vector.
    pub fn magnitude(&self) -> f32 {
        (self.flow_x * self.flow_x + self.flow_y * self.flow_y).sqrt()
    }

    /// Get the angle of the flow vector in radians.
    pub fn angle(&self) -> f32 {
        self.flow_y.atan2(self.flow_x)
    }

    /// Check if this pixel has significant motion (above threshold).
    pub fn has_motion(&self, threshold: f32) -> bool {
        self.magnitude() > threshold
    }
}

impl SensorData for OpticalFlowData {
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
        self.flow_vectors.len() * std::mem::size_of::<OpticalFlowVector>()
    }
}

impl OpticalFlowData {
    /// Create a new OpticalFlowData.
    pub fn new(
        timestamp: Timestamp,
        transform: Transform,
        sensor_id: u32,
        width: u32,
        height: u32,
        fov: f32,
        flow_vectors: Vec<OpticalFlowVector>,
    ) -> Self {
        Self {
            timestamp,
            transform,
            sensor_id,
            width,
            height,
            fov,
            flow_vectors,
        }
    }

    /// Get image dimensions as a tuple.
    pub fn dimensions(&self) -> (u32, u32) {
        (self.width, self.height)
    }

    /// Get the total number of flow vectors.
    pub fn vector_count(&self) -> usize {
        self.flow_vectors.len()
    }

    /// Filter vectors by motion magnitude.
    pub fn filter_by_magnitude(
        &self,
        min_magnitude: f32,
        max_magnitude: f32,
    ) -> Vec<OpticalFlowVector> {
        self.flow_vectors
            .iter()
            .filter(|v| {
                let mag = v.magnitude();
                mag >= min_magnitude && mag <= max_magnitude
            })
            .copied()
            .collect()
    }

    /// Get vectors with significant motion (above threshold).
    pub fn get_moving_pixels(&self, threshold: f32) -> Vec<OpticalFlowVector> {
        self.flow_vectors
            .iter()
            .filter(|v| v.has_motion(threshold))
            .copied()
            .collect()
    }

    /// Filter vectors by spatial region.
    pub fn filter_by_region(&self, x1: u32, y1: u32, x2: u32, y2: u32) -> Vec<OpticalFlowVector> {
        let min_x = x1.min(x2);
        let max_x = x1.max(x2);
        let min_y = y1.min(y2);
        let max_y = y1.max(y2);

        self.flow_vectors
            .iter()
            .filter(|v| v.x >= min_x && v.x <= max_x && v.y >= min_y && v.y <= max_y)
            .copied()
            .collect()
    }

    /// Generate a flow magnitude image (grayscale).
    pub fn generate_magnitude_image(&self) -> Vec<u8> {
        let mut image = vec![0u8; (self.width * self.height) as usize];

        // Find max magnitude for normalization
        let max_magnitude: f32 = self
            .flow_vectors
            .iter()
            .map(|v| v.magnitude())
            .fold(0.0f32, f32::max);

        if max_magnitude > 0.0 {
            for vector in &self.flow_vectors {
                let idx = (vector.y * self.width + vector.x) as usize;
                if idx < image.len() {
                    let normalized = (vector.magnitude() / max_magnitude * 255.0) as u8;
                    image[idx] = normalized;
                }
            }
        }

        image
    }

    /// Generate a flow direction image using HSV color encoding.
    /// Hue represents direction, Saturation represents magnitude.
    pub fn generate_direction_image(&self) -> Vec<[u8; 3]> {
        let mut image = vec![[0u8; 3]; (self.width * self.height) as usize];

        // Find max magnitude for normalization
        let max_magnitude: f32 = self
            .flow_vectors
            .iter()
            .map(|v| v.magnitude())
            .fold(0.0f32, f32::max);

        if max_magnitude > 0.0 {
            for vector in &self.flow_vectors {
                let idx = (vector.y * self.width + vector.x) as usize;
                if idx < image.len() {
                    let angle = vector.angle();
                    let magnitude = vector.magnitude();

                    // Convert angle to hue (0-360 degrees)
                    let hue = ((angle + std::f32::consts::PI) / (2.0 * std::f32::consts::PI)
                        * 360.0)
                        % 360.0;
                    // Normalize magnitude to saturation (0-1)
                    let saturation = (magnitude / max_magnitude).min(1.0);
                    let value = 1.0; // Full brightness

                    // Convert HSV to RGB
                    let rgb = hsv_to_rgb(hue, saturation, value);
                    image[idx] = [
                        (rgb.0 * 255.0) as u8,
                        (rgb.1 * 255.0) as u8,
                        (rgb.2 * 255.0) as u8,
                    ];
                }
            }
        }

        image
    }

    /// Calculate optical flow statistics.
    pub fn get_statistics(&self) -> OpticalFlowStatistics {
        if self.flow_vectors.is_empty() {
            return OpticalFlowStatistics::default();
        }

        let mut total_magnitude = 0.0f32;
        let mut max_magnitude = 0.0f32;
        let mut min_magnitude = f32::INFINITY;
        let mut moving_pixels = 0;
        let motion_threshold = 0.5; // pixels per frame

        for vector in &self.flow_vectors {
            let magnitude = vector.magnitude();
            total_magnitude += magnitude;
            max_magnitude = max_magnitude.max(magnitude);
            min_magnitude = min_magnitude.min(magnitude);

            if magnitude > motion_threshold {
                moving_pixels += 1;
            }
        }

        let avg_magnitude = total_magnitude / self.flow_vectors.len() as f32;
        let motion_percentage = (moving_pixels as f32 / self.flow_vectors.len() as f32) * 100.0;

        OpticalFlowStatistics {
            total_vectors: self.flow_vectors.len(),
            avg_magnitude,
            max_magnitude,
            min_magnitude,
            moving_pixels,
            motion_percentage,
        }
    }

    /// Estimate overall scene motion (global flow).
    pub fn estimate_global_motion(&self) -> Vector2D {
        if self.flow_vectors.is_empty() {
            return Vector2D::new(0.0, 0.0);
        }

        let mut total_flow_x = 0.0;
        let mut total_flow_y = 0.0;

        for vector in &self.flow_vectors {
            total_flow_x += vector.flow_x;
            total_flow_y += vector.flow_y;
        }

        Vector2D::new(
            total_flow_x / self.flow_vectors.len() as f32,
            total_flow_y / self.flow_vectors.len() as f32,
        )
    }

    /// Detect potential collisions based on expanding flow patterns.
    pub fn detect_expansion_regions(&self, expansion_threshold: f32) -> Vec<ExpansionRegion> {
        let mut regions = Vec::new();
        let center_x = self.width as f32 / 2.0;
        let center_y = self.height as f32 / 2.0;

        // Simple expansion detection: vectors pointing away from center with high magnitude
        for vector in &self.flow_vectors {
            let dx = vector.x as f32 - center_x;
            let dy = vector.y as f32 - center_y;
            let distance_from_center = (dx * dx + dy * dy).sqrt();

            if distance_from_center > 0.0 {
                // Calculate if flow is pointing away from center
                let normalized_radial_x = dx / distance_from_center;
                let normalized_radial_y = dy / distance_from_center;

                let radial_flow =
                    vector.flow_x * normalized_radial_x + vector.flow_y * normalized_radial_y;

                if radial_flow > expansion_threshold {
                    regions.push(ExpansionRegion {
                        center_x: vector.x,
                        center_y: vector.y,
                        expansion_strength: radial_flow,
                        magnitude: vector.magnitude(),
                    });
                }
            }
        }

        regions
    }
}

/// Optical flow sensor statistics.
#[derive(Debug, Clone, PartialEq)]
pub struct OpticalFlowStatistics {
    /// Total number of flow vectors
    pub total_vectors: usize,
    /// Average flow magnitude
    pub avg_magnitude: f32,
    /// Maximum flow magnitude
    pub max_magnitude: f32,
    /// Minimum flow magnitude
    pub min_magnitude: f32,
    /// Number of pixels with significant motion
    pub moving_pixels: usize,
    /// Percentage of pixels with motion
    pub motion_percentage: f32,
}

impl Default for OpticalFlowStatistics {
    fn default() -> Self {
        Self {
            total_vectors: 0,
            avg_magnitude: 0.0,
            max_magnitude: 0.0,
            min_magnitude: 0.0,
            moving_pixels: 0,
            motion_percentage: 0.0,
        }
    }
}

/// Region showing potential collision or expansion pattern.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ExpansionRegion {
    /// Center X coordinate of expansion
    pub center_x: u32,
    /// Center Y coordinate of expansion
    pub center_y: u32,
    /// Strength of the expansion pattern
    pub expansion_strength: f32,
    /// Magnitude of the flow at this point
    pub magnitude: f32,
}

/// Convert HSV color to RGB.
fn hsv_to_rgb(h: f32, s: f32, v: f32) -> (f32, f32, f32) {
    let c = v * s;
    let x = c * (1.0 - ((h / 60.0) % 2.0 - 1.0).abs());
    let m = v - c;

    let (r_prime, g_prime, b_prime) = if h < 60.0 {
        (c, x, 0.0)
    } else if h < 120.0 {
        (x, c, 0.0)
    } else if h < 180.0 {
        (0.0, c, x)
    } else if h < 240.0 {
        (0.0, x, c)
    } else if h < 300.0 {
        (x, 0.0, c)
    } else {
        (c, 0.0, x)
    };

    (r_prime + m, g_prime + m, b_prime + m)
}
