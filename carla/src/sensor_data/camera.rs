//! Camera sensor implementations.

use crate::{
    geom::Transform,
    sensor_data::{ImageData, SensorData},
    time::Timestamp,
};

/// RGB camera sensor data.
pub type RGBImageData = ImageData;

/// Depth camera sensor data.
#[derive(Debug, Clone)]
pub struct DepthImageData {
    /// Base image data
    pub image: ImageData,
}

impl DepthImageData {
    /// Create DepthImageData from ImageData
    pub fn new(image: ImageData) -> Self {
        Self { image }
    }
}

impl SensorData for DepthImageData {
    fn timestamp(&self) -> Timestamp {
        self.image.timestamp()
    }
    fn transform(&self) -> Transform {
        self.image.transform()
    }
    fn sensor_id(&self) -> u32 {
        self.image.sensor_id()
    }
    fn size(&self) -> usize {
        self.image.size()
    }
}

impl DepthImageData {
    /// Convert depth data to actual distance values in meters.
    pub fn to_depth_array(&self) -> ndarray::Array2<f32> {
        let mut depth_data =
            ndarray::Array2::zeros((self.image.height as usize, self.image.width as usize));

        for y in 0..self.image.height as usize {
            for x in 0..self.image.width as usize {
                let idx = (y * self.image.width as usize + x) * 4;
                if idx + 2 < self.image.data.len() {
                    let r = self.image.data[idx + 2] as f32;
                    let g = self.image.data[idx + 1] as f32;
                    let b = self.image.data[idx + 0] as f32;

                    // CARLA depth encoding: depth = (R + G * 256 + B * 256 * 256) / (256^3 - 1) * 1000
                    let normalized = (r + g * 256.0 + b * 65536.0) / 16777215.0;
                    let depth_meters = normalized * 1000.0; // Convert to meters
                    depth_data[[y, x]] = depth_meters;
                }
            }
        }

        depth_data
    }

    /// Get depth value at specific pixel coordinates.
    pub fn get_depth_at_pixel(&self, x: u32, y: u32) -> Option<f32> {
        if x >= self.image.width || y >= self.image.height {
            return None;
        }

        let idx = (y as usize * self.image.width as usize + x as usize) * 4;
        if idx + 2 < self.image.data.len() {
            let r = self.image.data[idx + 2] as f32;
            let g = self.image.data[idx + 1] as f32;
            let b = self.image.data[idx + 0] as f32;

            let normalized = (r + g * 256.0 + b * 65536.0) / 16777215.0;
            Some(normalized * 1000.0)
        } else {
            None
        }
    }
}

/// Semantic segmentation camera sensor data.
#[derive(Debug, Clone)]
pub struct SemanticSegmentationImageData {
    /// Base image data
    pub image: ImageData,
}

impl SemanticSegmentationImageData {
    /// Create SemanticSegmentationImageData from ImageData
    pub fn new(image: ImageData) -> Self {
        Self { image }
    }
}

impl SensorData for SemanticSegmentationImageData {
    fn timestamp(&self) -> Timestamp {
        self.image.timestamp()
    }
    fn transform(&self) -> Transform {
        self.image.transform()
    }
    fn sensor_id(&self) -> u32 {
        self.image.sensor_id()
    }
    fn size(&self) -> usize {
        self.image.size()
    }
}

impl SemanticSegmentationImageData {
    /// Convert to semantic label array.
    pub fn to_semantic_array(&self) -> ndarray::Array2<u8> {
        let mut semantic_data =
            ndarray::Array2::zeros((self.image.height as usize, self.image.width as usize));

        for y in 0..self.image.height as usize {
            for x in 0..self.image.width as usize {
                let idx = (y * self.image.width as usize + x) * 4;
                if idx < self.image.data.len() {
                    // In semantic segmentation, the red channel contains the class ID
                    semantic_data[[y, x]] = self.image.data[idx + 2];
                }
            }
        }

        semantic_data
    }

    /// Get semantic label at specific pixel coordinates.
    pub fn get_label_at_pixel(&self, x: u32, y: u32) -> Option<u8> {
        if x >= self.image.width || y >= self.image.height {
            return None;
        }

        let idx = (y as usize * self.image.width as usize + x as usize) * 4;
        if idx + 2 < self.image.data.len() {
            Some(self.image.data[idx + 2])
        } else {
            None
        }
    }

    /// Count pixels for each semantic class.
    pub fn count_classes(&self) -> std::collections::HashMap<u8, usize> {
        let mut counts = std::collections::HashMap::new();

        for y in 0..self.image.height as usize {
            for x in 0..self.image.width as usize {
                let idx = (y * self.image.width as usize + x) * 4;
                if idx + 2 < self.image.data.len() {
                    let class_id = self.image.data[idx + 2];
                    *counts.entry(class_id).or_insert(0) += 1;
                }
            }
        }

        counts
    }
}

/// Instance segmentation camera sensor data.
#[derive(Debug, Clone)]
pub struct InstanceSegmentationImageData {
    /// Base image data
    pub image: ImageData,
}

impl InstanceSegmentationImageData {
    /// Create InstanceSegmentationImageData from ImageData
    pub fn new(image: ImageData) -> Self {
        Self { image }
    }
}

impl SensorData for InstanceSegmentationImageData {
    fn timestamp(&self) -> Timestamp {
        self.image.timestamp()
    }
    fn transform(&self) -> Transform {
        self.image.transform()
    }
    fn sensor_id(&self) -> u32 {
        self.image.sensor_id()
    }
    fn size(&self) -> usize {
        self.image.size()
    }
}

impl InstanceSegmentationImageData {
    /// Convert to instance ID array.
    pub fn to_instance_array(&self) -> ndarray::Array2<u32> {
        let mut instance_data =
            ndarray::Array2::zeros((self.image.height as usize, self.image.width as usize));

        for y in 0..self.image.height as usize {
            for x in 0..self.image.width as usize {
                let idx = (y * self.image.width as usize + x) * 4;
                if idx + 2 < self.image.data.len() {
                    let r = self.image.data[idx + 2] as u32;
                    let g = self.image.data[idx + 1] as u32;
                    let b = self.image.data[idx + 0] as u32;

                    // Instance ID encoding: ID = R + G * 256 + B * 256^2
                    let instance_id = r + g * 256 + b * 65536;
                    instance_data[[y, x]] = instance_id;
                }
            }
        }

        instance_data
    }

    /// Get instance ID at specific pixel coordinates.
    pub fn get_instance_at_pixel(&self, x: u32, y: u32) -> Option<u32> {
        if x >= self.image.width || y >= self.image.height {
            return None;
        }

        let idx = (y as usize * self.image.width as usize + x as usize) * 4;
        if idx + 2 < self.image.data.len() {
            let r = self.image.data[idx + 2] as u32;
            let g = self.image.data[idx + 1] as u32;
            let b = self.image.data[idx + 0] as u32;

            Some(r + g * 256 + b * 65536)
        } else {
            None
        }
    }
}
