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

/// CityScapes palette for semantic segmentation
pub struct CityScapesPalette;

impl CityScapesPalette {
    /// Get the number of semantic tags
    pub fn get_number_of_tags() -> usize {
        23 // CARLA uses 23 different semantic tags
    }

    /// Get color for a specific semantic tag
    pub fn get_color(tag: u8) -> [u8; 3] {
        // Colors based on CARLA's CityScapes palette
        match tag {
            0 => [0, 0, 0],        // Unlabeled
            1 => [70, 70, 70],     // Building
            2 => [100, 40, 40],    // Fence
            3 => [55, 90, 80],     // Other
            4 => [220, 20, 60],    // Pedestrian
            5 => [153, 153, 153],  // Pole
            6 => [157, 234, 50],   // RoadLine
            7 => [128, 64, 128],   // Road
            8 => [244, 35, 232],   // SideWalk
            9 => [107, 142, 35],   // Vegetation
            10 => [0, 0, 142],     // Vehicles
            11 => [102, 102, 156], // Wall
            12 => [220, 220, 0],   // TrafficSign
            13 => [70, 130, 180],  // Sky
            14 => [81, 0, 81],     // Ground
            15 => [150, 100, 100], // Bridge
            16 => [230, 150, 140], // RailTrack
            17 => [180, 165, 180], // GuardRail
            18 => [250, 170, 30],  // TrafficLight
            19 => [110, 190, 160], // Static
            20 => [170, 120, 50],  // Dynamic
            21 => [45, 60, 150],   // Water
            22 => [145, 170, 100], // Terrain
            _ => [0, 0, 0],        // Default to unlabeled
        }
    }

    /// Get semantic class name
    pub fn get_tag_name(tag: u8) -> &'static str {
        match tag {
            0 => "Unlabeled",
            1 => "Building",
            2 => "Fence",
            3 => "Other",
            4 => "Pedestrian",
            5 => "Pole",
            6 => "RoadLine",
            7 => "Road",
            8 => "SideWalk",
            9 => "Vegetation",
            10 => "Vehicles",
            11 => "Wall",
            12 => "TrafficSign",
            13 => "Sky",
            14 => "Ground",
            15 => "Bridge",
            16 => "RailTrack",
            17 => "GuardRail",
            18 => "TrafficLight",
            19 => "Static",
            20 => "Dynamic",
            21 => "Water",
            22 => "Terrain",
            _ => "Unknown",
        }
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        geom::{Location, Rotation, Transform},
        time::Timestamp,
    };
    use approx::assert_relative_eq;

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
    #[cfg(not(debug_assertions))] // Only in release mode (performance)
    fn test_depth_conversion() {
        // Corresponds to C++ TEST(image, depth)
        let width = 256;
        let height = 256;
        let mut img = create_test_image(width, height);

        // Test all possible depth values
        let mut idx = 0;
        for z in 0..256 {
            for y in 0..256 {
                for x in 0..4 {
                    // Only test first 4 pixels of each row
                    if idx + 3 < img.data.len() {
                        // Encode depth as BGRA
                        let r = x as u8;
                        let g = y as u8;
                        let b = z as u8;
                        img.data[idx] = b; // B
                        img.data[idx + 1] = g; // G
                        img.data[idx + 2] = r; // R
                        img.data[idx + 3] = 255; // A

                        idx += 4;
                    }
                }
                // Skip remaining pixels in row
                idx += (width as usize - 4) * 4;
            }
        }

        let depth_img = DepthImageData::new(img);
        let depth_array = depth_img.to_depth_array();

        // Verify a few sample depth values
        for test_idx in 0..10 {
            let x = test_idx % 4;
            let y = test_idx * 25; // Sample every 25th row
            let z = y / 256;

            if y < height && x < 4 {
                let expected_normalized =
                    (x as f32 + y as f32 * 256.0 + z as f32 * 65536.0) / 16777215.0;
                let expected_depth = expected_normalized * 1000.0;
                let actual_depth = depth_array[[y as usize, x as usize]];

                assert_relative_eq!(actual_depth, expected_depth, epsilon = 0.1);
            }
        }
    }

    #[test]
    fn test_depth_basic_operations() {
        let mut img = create_test_image(2, 2);

        // Set a specific depth value at pixel (0, 0)
        // Test a simple depth value: let's use R=10, G=20, B=30
        img.data[0] = 30; // B
        img.data[1] = 20; // G
        img.data[2] = 10; // R
        img.data[3] = 255; // A

        let depth_img = DepthImageData::new(img);

        // Test get_depth_at_pixel
        let depth = depth_img.get_depth_at_pixel(0, 0).unwrap();

        // Expected: (10 + 20*256 + 30*65536) / 16777215.0 * 1000.0
        let expected = (10.0 + 20.0 * 256.0 + 30.0 * 65536.0) / 16777215.0 * 1000.0;
        assert_relative_eq!(depth, expected, epsilon = 0.1);

        // Test out of bounds
        assert!(depth_img.get_depth_at_pixel(10, 10).is_none());
    }

    #[test]
    fn test_semantic_segmentation() {
        // Corresponds to C++ TEST(image, semantic_segmentation)
        let width = CityScapesPalette::get_number_of_tags() as u32;
        let height = 1;
        let mut img = create_test_image(width, height);

        // Set each pixel to a different semantic class
        for tag in 0..width {
            let idx = (tag * 4) as usize;
            img.data[idx] = 0; // B
            img.data[idx + 1] = 0; // G
            img.data[idx + 2] = tag as u8; // R (semantic class ID)
            img.data[idx + 3] = 255; // A
        }

        let semantic_img = SemanticSegmentationImageData::new(img);
        let semantic_array = semantic_img.to_semantic_array();

        // Verify each semantic class
        for tag in 0..width {
            let class_id = semantic_array[[0, tag as usize]];
            assert_eq!(class_id, tag as u8);

            // Verify the color mapping
            let color = CityScapesPalette::get_color(tag as u8);
            assert_eq!(color.len(), 3); // RGB

            // Verify tag name
            let name = CityScapesPalette::get_tag_name(tag as u8);
            assert!(!name.is_empty());
        }

        // Test count_classes
        let class_counts = semantic_img.count_classes();
        assert_eq!(class_counts.len(), width as usize);
        for tag in 0..width {
            assert_eq!(*class_counts.get(&(tag as u8)).unwrap(), 1);
        }
    }

    #[test]
    fn test_semantic_segmentation_operations() {
        let mut img = create_test_image(3, 3);

        // Set different semantic classes
        // Row 0: Road (7), Road (7), Sidewalk (8)
        // Row 1: Vehicle (10), Pedestrian (4), Road (7)
        // Row 2: Building (1), Building (1), Sky (13)

        let classes = [
            7, 7, 8, // Row 0
            10, 4, 7, // Row 1
            1, 1, 13, // Row 2
        ];

        for (i, &class_id) in classes.iter().enumerate() {
            let idx = i * 4;
            img.data[idx + 2] = class_id; // R channel
        }

        let semantic_img = SemanticSegmentationImageData::new(img);

        // Test get_label_at_pixel
        assert_eq!(semantic_img.get_label_at_pixel(0, 0).unwrap(), 7); // Road
        assert_eq!(semantic_img.get_label_at_pixel(1, 1).unwrap(), 4); // Pedestrian
        assert_eq!(semantic_img.get_label_at_pixel(2, 2).unwrap(), 13); // Sky

        // Test count_classes
        let counts = semantic_img.count_classes();
        assert_eq!(*counts.get(&7).unwrap(), 3); // 3 Road pixels
        assert_eq!(*counts.get(&1).unwrap(), 2); // 2 Building pixels
        assert_eq!(*counts.get(&4).unwrap(), 1); // 1 Pedestrian pixel
        assert_eq!(*counts.get(&8).unwrap(), 1); // 1 Sidewalk pixel
        assert_eq!(*counts.get(&10).unwrap(), 1); // 1 Vehicle pixel
        assert_eq!(*counts.get(&13).unwrap(), 1); // 1 Sky pixel
    }

    #[test]
    fn test_cityscapes_palette() {
        // Test all semantic tags have valid colors
        for tag in 0..CityScapesPalette::get_number_of_tags() {
            let color = CityScapesPalette::get_color(tag as u8);
            let name = CityScapesPalette::get_tag_name(tag as u8);

            // Verify color is valid RGB
            assert!(color[0] <= 255);
            assert!(color[1] <= 255);
            assert!(color[2] <= 255);

            // Verify name is not unknown for valid tags
            assert_ne!(name, "Unknown");
        }

        // Test invalid tag
        let invalid_tag = 255;
        let color = CityScapesPalette::get_color(invalid_tag);
        let name = CityScapesPalette::get_tag_name(invalid_tag);
        assert_eq!(color, [0, 0, 0]); // Should be black (unlabeled)
        assert_eq!(name, "Unknown");
    }

    #[test]
    fn test_instance_segmentation() {
        let mut img = create_test_image(2, 2);

        // Set specific instance IDs
        // Instance ID = R + G * 256 + B * 256^2
        // Pixel (0,0): Instance ID = 1000 = 232 + 3*256 + 0*65536
        img.data[0] = 0; // B
        img.data[1] = 3; // G
        img.data[2] = 232; // R

        // Pixel (1,0): Instance ID = 65537 = 1 + 0*256 + 1*65536
        img.data[4] = 1; // B
        img.data[5] = 0; // G
        img.data[6] = 1; // R

        let instance_img = InstanceSegmentationImageData::new(img);

        // Test to_instance_array
        let instance_array = instance_img.to_instance_array();
        assert_eq!(instance_array[[0, 0]], 1000);
        assert_eq!(instance_array[[0, 1]], 65537);

        // Test get_instance_at_pixel
        assert_eq!(instance_img.get_instance_at_pixel(0, 0).unwrap(), 1000);
        assert_eq!(instance_img.get_instance_at_pixel(1, 0).unwrap(), 65537);
        assert!(instance_img.get_instance_at_pixel(10, 10).is_none());
    }
}
