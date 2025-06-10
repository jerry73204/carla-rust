use crate::sensor::data::Image;
use anyhow::{anyhow, Result};
use carla_sys::*;
use nalgebra::{Point3, Vector3};
use ndarray::{Array2, Array3, ArrayView2, ArrayView3};
use std::{ptr, slice};

/// Advanced image processing and analysis capabilities for CARLA sensors.
/// Provides semantic segmentation, instance segmentation, depth analysis,
/// and point cloud conversion functionality.
pub struct ImageAnalyzer {
    inner: *mut carla_image_analyzer_t,
}

impl ImageAnalyzer {
    /// Create a new image analyzer.
    pub fn new() -> Result<Self> {
        let analyzer_ptr = unsafe { carla_image_analyzer_create() };
        if analyzer_ptr.is_null() {
            return Err(anyhow!("Failed to create image analyzer"));
        }
        Ok(Self { inner: analyzer_ptr })
    }

    /// Analyze semantic segmentation in an image.
    /// 
    /// # Arguments
    /// * `image` - Semantic segmentation image from CARLA
    /// 
    /// # Returns
    /// Semantic segmentation analysis results
    pub fn analyze_semantic_segmentation(&self, image: &Image) -> Result<SemanticSegmentationAnalysis> {
        let mut analysis = carla_semantic_analysis_result_t {
            class_counts: ptr::null_mut(),
            class_percentages: ptr::null_mut(),
            dominant_class: 0,
            num_classes: 0,
            total_pixels: 0,
        };

        let error = unsafe {
            carla_image_analyze_semantic_segmentation(
                self.inner,
                image.raw_ptr() as *mut carla_image_data_t,
                &mut analysis,
            )
        };

        if error != 0 {
            return Err(anyhow!("Failed to analyze semantic segmentation"));
        }

        let result = SemanticSegmentationAnalysis::from_c_analysis(analysis);

        // Free C allocated arrays
        unsafe {
            if !analysis.class_counts.is_null() {
                carla_free_array(analysis.class_counts as *mut std::ffi::c_void);
            }
            if !analysis.class_percentages.is_null() {
                carla_free_array(analysis.class_percentages as *mut std::ffi::c_void);
            }
        }

        Ok(result)
    }

    /// Analyze instance segmentation in an image.
    /// 
    /// # Arguments
    /// * `image` - Instance segmentation image from CARLA
    /// 
    /// # Returns
    /// Instance segmentation analysis results
    pub fn analyze_instance_segmentation(&self, image: &Image) -> Result<InstanceSegmentationAnalysis> {
        let mut analysis = carla_instance_analysis_result_t {
            instance_ids: ptr::null_mut(),
            instance_sizes: ptr::null_mut(),
            instance_classes: ptr::null_mut(),
            num_instances: 0,
            largest_instance_id: 0,
            total_pixels: 0,
        };

        let error = unsafe {
            carla_image_analyze_instance_segmentation(
                self.inner,
                image.raw_ptr() as *mut carla_image_data_t,
                &mut analysis,
            )
        };

        if error != 0 {
            return Err(anyhow!("Failed to analyze instance segmentation"));
        }

        let result = InstanceSegmentationAnalysis::from_c_analysis(analysis);

        // Free C allocated arrays
        unsafe {
            if !analysis.instance_ids.is_null() {
                carla_free_array(analysis.instance_ids as *mut std::ffi::c_void);
            }
            if !analysis.instance_sizes.is_null() {
                carla_free_array(analysis.instance_sizes as *mut std::ffi::c_void);
            }
            if !analysis.instance_classes.is_null() {
                carla_free_array(analysis.instance_classes as *mut std::ffi::c_void);
            }
        }

        Ok(result)
    }

    /// Analyze depth information in a depth image.
    /// 
    /// # Arguments
    /// * `depth_image` - Depth image from CARLA camera
    /// 
    /// # Returns
    /// Depth analysis results
    pub fn analyze_depth(&self, depth_image: &Image) -> Result<DepthAnalysis> {
        let mut analysis = carla_depth_analysis_result_t {
            min_depth: 0.0,
            max_depth: 0.0,
            mean_depth: 0.0,
            median_depth: 0.0,
            std_depth: 0.0,
            depth_histogram: ptr::null_mut(),
            histogram_bins: 0,
        };

        let error = unsafe {
            carla_image_analyze_depth(
                self.inner,
                depth_image.raw_ptr() as *mut carla_image_data_t,
                &mut analysis,
            )
        };

        if error != 0 {
            return Err(anyhow!("Failed to analyze depth image"));
        }

        let result = DepthAnalysis::from_c_analysis(analysis);

        // Free C allocated histogram
        unsafe {
            if !analysis.depth_histogram.is_null() {
                carla_free_array(analysis.depth_histogram as *mut std::ffi::c_void);
            }
        }

        Ok(result)
    }

    /// Convert a depth image to a 3D point cloud.
    /// 
    /// # Arguments
    /// * `depth_image` - Depth image from CARLA camera
    /// * `rgb_image` - Optional RGB image for colored point cloud
    /// * `camera_intrinsics` - Camera intrinsic parameters
    /// 
    /// # Returns
    /// 3D point cloud data
    pub fn depth_to_point_cloud(
        &self,
        depth_image: &Image,
        rgb_image: Option<&Image>,
        camera_intrinsics: &CameraIntrinsics,
    ) -> Result<PointCloud> {
        let rgb_ptr = rgb_image
            .map(|img| img.raw_ptr() as *mut carla_image_data_t)
            .unwrap_or(ptr::null_mut());

        let c_intrinsics = camera_intrinsics.to_c_intrinsics();
        let mut point_cloud = carla_point_cloud_t {
            points: ptr::null_mut(),
            colors: ptr::null_mut(),
            num_points: 0,
            has_colors: rgb_image.is_some(),
        };

        let error = unsafe {
            carla_image_depth_to_point_cloud(
                self.inner,
                depth_image.raw_ptr() as *mut carla_image_data_t,
                rgb_ptr,
                &c_intrinsics,
                &mut point_cloud,
            )
        };

        if error != 0 {
            return Err(anyhow!("Failed to convert depth to point cloud"));
        }

        let result = PointCloud::from_c_point_cloud(point_cloud);

        // Free C allocated arrays
        unsafe {
            if !point_cloud.points.is_null() {
                carla_free_array(point_cloud.points as *mut std::ffi::c_void);
            }
            if !point_cloud.colors.is_null() {
                carla_free_array(point_cloud.colors as *mut std::ffi::c_void);
            }
        }

        Ok(result)
    }

    /// Extract objects from semantic segmentation mask.
    /// 
    /// # Arguments
    /// * `semantic_image` - Semantic segmentation image
    /// * `class_id` - Semantic class ID to extract
    /// 
    /// # Returns
    /// Binary mask of the specified class
    pub fn extract_semantic_class(
        &self,
        semantic_image: &Image,
        class_id: u8,
    ) -> Result<Array2<bool>> {
        let width = semantic_image.width();
        let height = semantic_image.height();
        let mut mask_data = vec![false; width * height];

        let error = unsafe {
            carla_image_extract_semantic_class(
                self.inner,
                semantic_image.raw_ptr() as *mut carla_image_data_t,
                class_id,
                mask_data.as_mut_ptr(),
            )
        };

        if error != 0 {
            return Err(anyhow!("Failed to extract semantic class"));
        }

        Array2::from_shape_vec((height, width), mask_data)
            .map_err(|e| anyhow!("Failed to create array: {}", e))
    }

    /// Calculate image quality metrics.
    /// 
    /// # Arguments
    /// * `image` - RGB image to analyze
    /// 
    /// # Returns
    /// Image quality metrics
    pub fn calculate_image_quality(&self, image: &Image) -> Result<ImageQualityMetrics> {
        let mut metrics = carla_image_quality_metrics_t {
            brightness: 0.0,
            contrast: 0.0,
            sharpness: 0.0,
            noise_level: 0.0,
            saturation: 0.0,
            entropy: 0.0,
        };

        let error = unsafe {
            carla_image_calculate_quality_metrics(
                self.inner,
                image.raw_ptr() as *mut carla_image_data_t,
                &mut metrics,
            )
        };

        if error != 0 {
            return Err(anyhow!("Failed to calculate image quality metrics"));
        }

        Ok(ImageQualityMetrics::from_c_metrics(metrics))
    }

    /// Detect edges in an image using Canny edge detection.
    /// 
    /// # Arguments
    /// * `image` - Input image
    /// * `low_threshold` - Low threshold for edge detection
    /// * `high_threshold` - High threshold for edge detection
    /// 
    /// # Returns
    /// Binary edge map
    pub fn detect_edges(
        &self,
        image: &Image,
        low_threshold: f32,
        high_threshold: f32,
    ) -> Result<Array2<bool>> {
        let width = image.width();
        let height = image.height();
        let mut edge_data = vec![false; width * height];

        let error = unsafe {
            carla_image_detect_edges(
                self.inner,
                image.raw_ptr() as *mut carla_image_data_t,
                low_threshold,
                high_threshold,
                edge_data.as_mut_ptr(),
            )
        };

        if error != 0 {
            return Err(anyhow!("Failed to detect edges"));
        }

        Array2::from_shape_vec((height, width), edge_data)
            .map_err(|e| anyhow!("Failed to create array: {}", e))
    }

    /// Apply histogram equalization to improve image contrast.
    /// 
    /// # Arguments
    /// * `image` - Input image
    /// 
    /// # Returns
    /// Histogram equalized image
    pub fn histogram_equalization(&self, image: &Image) -> Result<Array3<u8>> {
        let width = image.width();
        let height = image.height();
        let channels = 3; // RGB
        let mut output_data = vec![0u8; width * height * channels];

        let error = unsafe {
            carla_image_histogram_equalization(
                self.inner,
                image.raw_ptr() as *mut carla_image_data_t,
                output_data.as_mut_ptr(),
            )
        };

        if error != 0 {
            return Err(anyhow!("Failed to apply histogram equalization"));
        }

        Array3::from_shape_vec((height, width, channels), output_data)
            .map_err(|e| anyhow!("Failed to create array: {}", e))
    }
}

impl Drop for ImageAnalyzer {
    fn drop(&mut self) {
        if !self.inner.is_null() {
            unsafe {
                carla_image_analyzer_destroy(self.inner);
            }
            self.inner = ptr::null_mut();
        }
    }
}

/// Results of semantic segmentation analysis.
#[derive(Clone, Debug)]
pub struct SemanticSegmentationAnalysis {
    /// Count of pixels for each semantic class
    pub class_counts: Vec<u32>,
    /// Percentage of image covered by each class
    pub class_percentages: Vec<f32>,
    /// ID of the dominant (most common) class
    pub dominant_class: u8,
    /// Total number of semantic classes found
    pub num_classes: usize,
    /// Total number of pixels in the image
    pub total_pixels: usize,
}

impl SemanticSegmentationAnalysis {
    pub(crate) fn from_c_analysis(analysis: carla_semantic_analysis_result_t) -> Self {
        let class_counts = if analysis.class_counts.is_null() {
            Vec::new()
        } else {
            unsafe {
                slice::from_raw_parts(analysis.class_counts, analysis.num_classes).to_vec()
            }
        };

        let class_percentages = if analysis.class_percentages.is_null() {
            Vec::new()
        } else {
            unsafe {
                slice::from_raw_parts(analysis.class_percentages, analysis.num_classes).to_vec()
            }
        };

        Self {
            class_counts,
            class_percentages,
            dominant_class: analysis.dominant_class,
            num_classes: analysis.num_classes,
            total_pixels: analysis.total_pixels,
        }
    }

    /// Get the percentage of the image covered by a specific class.
    pub fn get_class_percentage(&self, class_id: u8) -> Option<f32> {
        if (class_id as usize) < self.class_percentages.len() {
            Some(self.class_percentages[class_id as usize])
        } else {
            None
        }
    }

    /// Get the pixel count for a specific class.
    pub fn get_class_count(&self, class_id: u8) -> Option<u32> {
        if (class_id as usize) < self.class_counts.len() {
            Some(self.class_counts[class_id as usize])
        } else {
            None
        }
    }
}

/// Results of instance segmentation analysis.
#[derive(Clone, Debug)]
pub struct InstanceSegmentationAnalysis {
    /// Unique instance IDs found in the image
    pub instance_ids: Vec<u32>,
    /// Size (pixel count) of each instance
    pub instance_sizes: Vec<u32>,
    /// Semantic class ID for each instance
    pub instance_classes: Vec<u8>,
    /// Total number of instances found
    pub num_instances: usize,
    /// ID of the largest instance
    pub largest_instance_id: u32,
    /// Total number of pixels in the image
    pub total_pixels: usize,
}

impl InstanceSegmentationAnalysis {
    pub(crate) fn from_c_analysis(analysis: carla_instance_analysis_result_t) -> Self {
        let instance_ids = if analysis.instance_ids.is_null() {
            Vec::new()
        } else {
            unsafe {
                slice::from_raw_parts(analysis.instance_ids, analysis.num_instances).to_vec()
            }
        };

        let instance_sizes = if analysis.instance_sizes.is_null() {
            Vec::new()
        } else {
            unsafe {
                slice::from_raw_parts(analysis.instance_sizes, analysis.num_instances).to_vec()
            }
        };

        let instance_classes = if analysis.instance_classes.is_null() {
            Vec::new()
        } else {
            unsafe {
                slice::from_raw_parts(analysis.instance_classes, analysis.num_instances).to_vec()
            }
        };

        Self {
            instance_ids,
            instance_sizes,
            instance_classes,
            num_instances: analysis.num_instances,
            largest_instance_id: analysis.largest_instance_id,
            total_pixels: analysis.total_pixels,
        }
    }

    /// Get the size of a specific instance by ID.
    pub fn get_instance_size(&self, instance_id: u32) -> Option<u32> {
        self.instance_ids
            .iter()
            .position(|&id| id == instance_id)
            .map(|index| self.instance_sizes[index])
    }

    /// Get the semantic class of a specific instance by ID.
    pub fn get_instance_class(&self, instance_id: u32) -> Option<u8> {
        self.instance_ids
            .iter()
            .position(|&id| id == instance_id)
            .map(|index| self.instance_classes[index])
    }
}

/// Results of depth image analysis.
#[derive(Clone, Debug)]
pub struct DepthAnalysis {
    /// Minimum depth value in meters
    pub min_depth: f32,
    /// Maximum depth value in meters
    pub max_depth: f32,
    /// Mean depth value in meters
    pub mean_depth: f32,
    /// Median depth value in meters
    pub median_depth: f32,
    /// Standard deviation of depth values
    pub std_depth: f32,
    /// Depth histogram (depth distribution)
    pub depth_histogram: Vec<u32>,
}

impl DepthAnalysis {
    pub(crate) fn from_c_analysis(analysis: carla_depth_analysis_result_t) -> Self {
        let depth_histogram = if analysis.depth_histogram.is_null() {
            Vec::new()
        } else {
            unsafe {
                slice::from_raw_parts(analysis.depth_histogram, analysis.histogram_bins).to_vec()
            }
        };

        Self {
            min_depth: analysis.min_depth,
            max_depth: analysis.max_depth,
            mean_depth: analysis.mean_depth,
            median_depth: analysis.median_depth,
            std_depth: analysis.std_depth,
            depth_histogram,
        }
    }

    /// Get the depth range (max - min).
    pub fn depth_range(&self) -> f32 {
        self.max_depth - self.min_depth
    }
}

/// 3D point cloud data.
#[derive(Clone, Debug)]
pub struct PointCloud {
    /// 3D points in world coordinates
    pub points: Vec<Point3<f32>>,
    /// Optional RGB colors for each point
    pub colors: Option<Vec<[u8; 3]>>,
}

impl PointCloud {
    pub(crate) fn from_c_point_cloud(cloud: carla_point_cloud_t) -> Self {
        let points = if cloud.points.is_null() {
            Vec::new()
        } else {
            unsafe {
                slice::from_raw_parts(cloud.points, cloud.num_points)
                    .iter()
                    .map(|p| Point3::new(p.x, p.y, p.z))
                    .collect()
            }
        };

        let colors = if cloud.has_colors && !cloud.colors.is_null() {
            let color_data = unsafe {
                slice::from_raw_parts(cloud.colors, cloud.num_points)
            };
            Some(
                color_data
                    .iter()
                    .map(|c| [c.r, c.g, c.b])
                    .collect()
            )
        } else {
            None
        };

        Self { points, colors }
    }

    /// Get the number of points in the cloud.
    pub fn len(&self) -> usize {
        self.points.len()
    }

    /// Check if the point cloud is empty.
    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    /// Check if the point cloud has color information.
    pub fn has_colors(&self) -> bool {
        self.colors.is_some()
    }
}

/// Camera intrinsic parameters for point cloud conversion.
#[derive(Clone, Debug)]
pub struct CameraIntrinsics {
    /// Focal length in X direction (pixels)
    pub fx: f32,
    /// Focal length in Y direction (pixels)
    pub fy: f32,
    /// Principal point X coordinate (pixels)
    pub cx: f32,
    /// Principal point Y coordinate (pixels)
    pub cy: f32,
    /// Image width in pixels
    pub width: u32,
    /// Image height in pixels
    pub height: u32,
}

impl CameraIntrinsics {
    pub(crate) fn to_c_intrinsics(&self) -> carla_camera_intrinsics_t {
        carla_camera_intrinsics_t {
            fx: self.fx,
            fy: self.fy,
            cx: self.cx,
            cy: self.cy,
            width: self.width,
            height: self.height,
        }
    }
}

/// Image quality metrics.
#[derive(Clone, Debug)]
pub struct ImageQualityMetrics {
    /// Average brightness (0.0 to 1.0)
    pub brightness: f32,
    /// Contrast measure (0.0 to 1.0)
    pub contrast: f32,
    /// Sharpness measure (higher is sharper)
    pub sharpness: f32,
    /// Estimated noise level (0.0 to 1.0)
    pub noise_level: f32,
    /// Color saturation (0.0 to 1.0)
    pub saturation: f32,
    /// Image entropy (information content)
    pub entropy: f32,
}

impl ImageQualityMetrics {
    pub(crate) fn from_c_metrics(metrics: carla_image_quality_metrics_t) -> Self {
        Self {
            brightness: metrics.brightness,
            contrast: metrics.contrast,
            sharpness: metrics.sharpness,
            noise_level: metrics.noise_level,
            saturation: metrics.saturation,
            entropy: metrics.entropy,
        }
    }
}

// SAFETY: ImageAnalyzer wraps a thread-safe C API
unsafe impl Send for ImageAnalyzer {}
unsafe impl Sync for ImageAnalyzer {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_camera_intrinsics_conversion() {
        let intrinsics = CameraIntrinsics {
            fx: 800.0,
            fy: 800.0,
            cx: 320.0,
            cy: 240.0,
            width: 640,
            height: 480,
        };

        let c_intrinsics = intrinsics.to_c_intrinsics();
        assert_eq!(c_intrinsics.fx, 800.0);
        assert_eq!(c_intrinsics.fy, 800.0);
        assert_eq!(c_intrinsics.cx, 320.0);
        assert_eq!(c_intrinsics.cy, 240.0);
        assert_eq!(c_intrinsics.width, 640);
        assert_eq!(c_intrinsics.height, 480);
    }

    #[test]
    fn test_point_cloud_properties() {
        let points = vec![
            Point3::new(1.0, 2.0, 3.0),
            Point3::new(4.0, 5.0, 6.0),
        ];
        let colors = Some(vec![[255, 0, 0], [0, 255, 0]]);

        let cloud = PointCloud { points, colors };

        assert_eq!(cloud.len(), 2);
        assert!(!cloud.is_empty());
        assert!(cloud.has_colors());
    }
}