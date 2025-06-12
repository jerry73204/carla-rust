//! Sensor data types for CARLA 0.10.0

use super::SensorData;
use crate::utils::check_carla_error;
use anyhow::{anyhow, Result};
use carla_sys::*;
use nalgebra::Vector3;
use ndarray::{Array2, Array3};

/// RGB/RGBA camera image data
#[derive(Debug)]
pub struct ImageData {
    width: u32,
    height: u32,
    fov: u32,
    raw_data: Vec<u8>,
}

impl ImageData {
    pub(crate) fn from_sensor_data(sensor_data: &SensorData) -> Result<Self> {
        let width = unsafe { carla_image_data_get_width(sensor_data.inner) };
        let height = unsafe { carla_image_data_get_height(sensor_data.inner) };
        let fov = unsafe { carla_image_data_get_fov(sensor_data.inner) };
        let data_size = unsafe { carla_image_data_get_raw_data_size(sensor_data.inner) };

        // Copy data safely into a Vec
        let mut raw_data = vec![0u8; data_size];
        let error = unsafe {
            carla_image_data_copy_to_buffer(sensor_data.inner, raw_data.as_mut_ptr(), data_size)
        };

        if error != carla_error_t_CARLA_ERROR_NONE {
            return Err(anyhow!("Failed to copy image data"));
        }

        Ok(Self {
            width,
            height,
            fov,
            raw_data,
        })
    }

    /// Get image width in pixels
    pub fn width(&self) -> u32 {
        self.width
    }

    /// Get image height in pixels
    pub fn height(&self) -> u32 {
        self.height
    }

    /// Get horizontal field of view
    pub fn fov(&self) -> u32 {
        self.fov
    }

    /// Get raw image data as bytes
    pub fn raw_data(&self) -> &[u8] {
        &self.raw_data
    }

    /// Convert to a 3D ndarray with shape (height, width, channels)
    pub fn to_array(&self) -> Result<Array3<u8>> {
        let channels = self.raw_data.len() / (self.width as usize * self.height as usize);
        if channels == 0 {
            return Err(anyhow!("Invalid image data size"));
        }

        Array3::from_shape_vec(
            (self.height as usize, self.width as usize, channels),
            self.raw_data.clone(),
        )
        .map_err(|e| anyhow!("Failed to create array: {}", e))
    }

    /// Convert BGRA image to RGB format
    pub fn to_rgb(&self, sensor_data: &SensorData) -> Result<Vec<u8>> {
        let mut rgb_data_ptr: *mut u8 = std::ptr::null_mut();
        let mut rgb_size: usize = 0;

        let error =
            unsafe { carla_image_bgra_to_rgb(sensor_data.inner, &mut rgb_data_ptr, &mut rgb_size) };
        check_carla_error(error)?;

        if rgb_data_ptr.is_null() {
            return Err(anyhow!("Failed to convert image to RGB"));
        }

        let rgb_slice = unsafe { std::slice::from_raw_parts(rgb_data_ptr, rgb_size) };
        let rgb_data = rgb_slice.to_vec();

        // Free the allocated memory
        unsafe { carla_image_free_data(rgb_data_ptr) };

        Ok(rgb_data)
    }

    /// Convert image to grayscale
    pub fn to_grayscale(&self, sensor_data: &SensorData) -> Result<Vec<u8>> {
        let mut gray_data_ptr: *mut u8 = std::ptr::null_mut();
        let mut gray_size: usize = 0;

        let error = unsafe {
            carla_image_bgra_to_grayscale(sensor_data.inner, &mut gray_data_ptr, &mut gray_size)
        };
        check_carla_error(error)?;

        if gray_data_ptr.is_null() {
            return Err(anyhow!("Failed to convert image to grayscale"));
        }

        let gray_slice = unsafe { std::slice::from_raw_parts(gray_data_ptr, gray_size) };
        let gray_data = gray_slice.to_vec();

        // Free the allocated memory
        unsafe { carla_image_free_data(gray_data_ptr) };

        Ok(gray_data)
    }

    /// Apply Gaussian blur to the image
    pub fn gaussian_blur(&self, sensor_data: &SensorData, sigma: f32) -> Result<Vec<u8>> {
        let mut output_data_ptr: *mut u8 = std::ptr::null_mut();
        let mut output_size: usize = 0;

        let error = unsafe {
            carla_image_gaussian_blur(
                sensor_data.inner,
                sigma,
                &mut output_data_ptr,
                &mut output_size,
            )
        };
        check_carla_error(error)?;

        if output_data_ptr.is_null() {
            return Err(anyhow!("Failed to apply Gaussian blur"));
        }

        let output_slice = unsafe { std::slice::from_raw_parts(output_data_ptr, output_size) };
        let output_data = output_slice.to_vec();

        // Free the allocated memory
        unsafe { carla_image_free_data(output_data_ptr) };

        Ok(output_data)
    }

    /// Apply edge detection to the image
    pub fn edge_detection(&self, sensor_data: &SensorData, threshold: f32) -> Result<Vec<u8>> {
        let mut output_data_ptr: *mut u8 = std::ptr::null_mut();
        let mut output_size: usize = 0;

        let error = unsafe {
            carla_image_edge_detection(
                sensor_data.inner,
                threshold,
                &mut output_data_ptr,
                &mut output_size,
            )
        };
        check_carla_error(error)?;

        if output_data_ptr.is_null() {
            return Err(anyhow!("Failed to apply edge detection"));
        }

        let output_slice = unsafe { std::slice::from_raw_parts(output_data_ptr, output_size) };
        let output_data = output_slice.to_vec();

        // Free the allocated memory
        unsafe { carla_image_free_data(output_data_ptr) };

        Ok(output_data)
    }

    /// Resize the image to new dimensions
    pub fn resize(
        &self,
        sensor_data: &SensorData,
        new_width: u32,
        new_height: u32,
    ) -> Result<Vec<u8>> {
        let mut output_data_ptr: *mut u8 = std::ptr::null_mut();
        let mut output_size: usize = 0;

        let error = unsafe {
            carla_image_resize(
                sensor_data.inner,
                new_width,
                new_height,
                &mut output_data_ptr,
                &mut output_size,
            )
        };
        check_carla_error(error)?;

        if output_data_ptr.is_null() {
            return Err(anyhow!("Failed to resize image"));
        }

        let output_slice = unsafe { std::slice::from_raw_parts(output_data_ptr, output_size) };
        let output_data = output_slice.to_vec();

        // Free the allocated memory
        unsafe { carla_image_free_data(output_data_ptr) };

        Ok(output_data)
    }

    /// Crop the image to a specific region
    pub fn crop(&self, sensor_data: &SensorData, roi: &ImageRegionOfInterest) -> Result<Vec<u8>> {
        let c_roi = carla_image_roi_t {
            x: roi.x,
            y: roi.y,
            width: roi.width,
            height: roi.height,
        };

        let mut output_data_ptr: *mut u8 = std::ptr::null_mut();
        let mut output_size: usize = 0;

        let error = unsafe {
            carla_image_crop(
                sensor_data.inner,
                &c_roi,
                &mut output_data_ptr,
                &mut output_size,
            )
        };
        check_carla_error(error)?;

        if output_data_ptr.is_null() {
            return Err(anyhow!("Failed to crop image"));
        }

        let output_slice = unsafe { std::slice::from_raw_parts(output_data_ptr, output_size) };
        let output_data = output_slice.to_vec();

        // Free the allocated memory
        unsafe { carla_image_free_data(output_data_ptr) };

        Ok(output_data)
    }
}

/// LiDAR point cloud data
#[derive(Debug)]
pub struct LidarData {
    points: Vec<LidarPoint>,
    horizontal_angle: u32,
    channels: u32,
}

impl LidarData {
    pub(crate) fn from_sensor_data(sensor_data: &SensorData) -> Result<Self> {
        let point_count = unsafe { carla_lidar_data_get_point_count(sensor_data.inner) };
        let horizontal_angle = unsafe { carla_lidar_data_get_horizontal_angle(sensor_data.inner) };
        let channels = unsafe { carla_lidar_data_get_channels(sensor_data.inner) };

        // Copy points safely
        let mut points_buffer = vec![
            carla_lidar_detection_t {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                intensity: 0.0
            };
            point_count
        ];
        let error = unsafe {
            carla_lidar_data_copy_points_to_buffer(
                sensor_data.inner,
                points_buffer.as_mut_ptr(),
                point_count,
            )
        };

        if error != carla_error_t_CARLA_ERROR_NONE {
            return Err(anyhow!("Failed to copy LiDAR points"));
        }

        let points = points_buffer
            .into_iter()
            .map(|p| LidarPoint {
                position: Vector3::new(p.x, p.y, p.z),
                intensity: p.intensity,
            })
            .collect();

        Ok(Self {
            points,
            horizontal_angle,
            channels,
        })
    }

    /// Get all LiDAR points
    pub fn points(&self) -> &[LidarPoint] {
        &self.points
    }

    /// Get horizontal angle
    pub fn horizontal_angle(&self) -> u32 {
        self.horizontal_angle
    }

    /// Get number of channels
    pub fn channels(&self) -> u32 {
        self.channels
    }

    /// Convert points to position array (N x 3)
    pub fn to_position_array(&self) -> Array2<f32> {
        let mut data = Vec::with_capacity(self.points.len() * 3);
        for point in &self.points {
            data.push(point.position.x);
            data.push(point.position.y);
            data.push(point.position.z);
        }
        Array2::from_shape_vec((self.points.len(), 3), data)
            .expect("Valid shape for position array")
    }

    /// Convert to full point cloud array (N x 4) with intensity
    pub fn to_array(&self) -> Array2<f32> {
        let mut data = Vec::with_capacity(self.points.len() * 4);
        for point in &self.points {
            data.push(point.position.x);
            data.push(point.position.y);
            data.push(point.position.z);
            data.push(point.intensity);
        }
        Array2::from_shape_vec((self.points.len(), 4), data)
            .expect("Valid shape for point cloud array")
    }
}

/// A single LiDAR detection point
#[derive(Debug, Clone, Copy)]
pub struct LidarPoint {
    pub position: Vector3<f32>,
    pub intensity: f32,
}

/// Semantic LiDAR point cloud data with object labels
#[derive(Debug)]
pub struct SemanticLidarData {
    points: Vec<SemanticLidarPoint>,
    horizontal_angle: u32,
    channels: u32,
}

impl SemanticLidarData {
    pub(crate) fn from_sensor_data(sensor_data: &SensorData) -> Result<Self> {
        let point_count = unsafe { carla_semantic_lidar_data_get_point_count(sensor_data.inner) };
        let horizontal_angle = unsafe { carla_lidar_data_get_horizontal_angle(sensor_data.inner) };
        let channels = unsafe { carla_lidar_data_get_channels(sensor_data.inner) };

        let points_ptr = unsafe { carla_semantic_lidar_data_get_points(sensor_data.inner) };
        if points_ptr.is_null() {
            return Err(anyhow!("Null semantic LiDAR points pointer"));
        }

        let points_slice = unsafe { std::slice::from_raw_parts(points_ptr, point_count) };
        let points = points_slice
            .iter()
            .map(|p| SemanticLidarPoint {
                position: Vector3::new(p.x, p.y, p.z),
                cos_inc_angle: p.cos_inc_angle,
                object_idx: p.object_idx,
                object_tag: p.object_tag,
            })
            .collect();

        Ok(Self {
            points,
            horizontal_angle,
            channels,
        })
    }

    /// Get all semantic LiDAR points
    pub fn points(&self) -> &[SemanticLidarPoint] {
        &self.points
    }

    /// Get horizontal angle
    pub fn horizontal_angle(&self) -> u32 {
        self.horizontal_angle
    }

    /// Get number of channels
    pub fn channels(&self) -> u32 {
        self.channels
    }
}

/// A single semantic LiDAR detection point with object information
#[derive(Debug, Clone, Copy)]
pub struct SemanticLidarPoint {
    pub position: Vector3<f32>,
    pub cos_inc_angle: f32,
    pub object_idx: u32,
    pub object_tag: u32,
}

/// Radar detection data
#[derive(Debug)]
pub struct RadarData {
    detections: Vec<RadarDetection>,
}

impl RadarData {
    pub(crate) fn from_sensor_data(sensor_data: &SensorData) -> Result<Self> {
        let detection_count = unsafe { carla_radar_data_get_detection_count(sensor_data.inner) };
        let detections_ptr = unsafe { carla_radar_data_get_detections(sensor_data.inner) };

        if detections_ptr.is_null() {
            return Err(anyhow!("Null radar detections pointer"));
        }

        let detections_slice =
            unsafe { std::slice::from_raw_parts(detections_ptr, detection_count) };
        let detections = detections_slice
            .iter()
            .map(|d| RadarDetection {
                velocity: d.velocity,
                azimuth: d.azimuth,
                altitude: d.altitude,
                depth: d.depth,
            })
            .collect();

        Ok(Self { detections })
    }

    /// Get all radar detections
    pub fn detections(&self) -> &[RadarDetection] {
        &self.detections
    }
}

/// A single radar detection
#[derive(Debug, Clone, Copy)]
pub struct RadarDetection {
    pub velocity: f32,
    pub azimuth: f32,
    pub altitude: f32,
    pub depth: f32,
}

/// IMU (Inertial Measurement Unit) sensor data
#[derive(Debug, Clone, Copy)]
pub struct ImuData {
    pub accelerometer: Vector3<f32>, // m/s²
    pub gyroscope: Vector3<f32>,     // rad/s
    pub compass: f32,                // radians
}

impl ImuData {
    pub(crate) fn from_sensor_data(sensor_data: &SensorData) -> Result<Self> {
        let accel = unsafe { carla_imu_data_get_accelerometer(sensor_data.inner) };
        let gyro = unsafe { carla_imu_data_get_gyroscope(sensor_data.inner) };
        let compass = unsafe { carla_imu_data_get_compass(sensor_data.inner) };

        Ok(Self {
            accelerometer: Vector3::new(accel.x, accel.y, accel.z),
            gyroscope: Vector3::new(gyro.x, gyro.y, gyro.z),
            compass,
        })
    }
}

/// GNSS (Global Navigation Satellite System) sensor data
#[derive(Debug, Clone, Copy)]
pub struct GnssData {
    pub latitude: f64,  // degrees
    pub longitude: f64, // degrees
    pub altitude: f64,  // meters
}

impl GnssData {
    pub(crate) fn from_sensor_data(sensor_data: &SensorData) -> Result<Self> {
        let latitude = unsafe { carla_gnss_data_get_latitude(sensor_data.inner) };
        let longitude = unsafe { carla_gnss_data_get_longitude(sensor_data.inner) };
        let altitude = unsafe { carla_gnss_data_get_altitude(sensor_data.inner) };

        Ok(Self {
            latitude,
            longitude,
            altitude,
        })
    }
}

/// Collision event data
#[derive(Debug)]
pub struct CollisionEvent {
    pub normal_impulse: Vector3<f32>,
    // Note: Actor references would need to be properly managed
    // For now, we'll just store IDs
    actor_id: Option<u32>,
    other_actor_id: Option<u32>,
}

impl CollisionEvent {
    pub(crate) fn from_sensor_data(sensor_data: &SensorData) -> Result<Self> {
        let normal_impulse = unsafe { carla_collision_data_get_normal_impulse(sensor_data.inner) };

        // Get actor pointers but we'll just extract IDs for safety
        let actor_ptr = unsafe { carla_collision_data_get_actor(sensor_data.inner) };
        let other_actor_ptr = unsafe { carla_collision_data_get_other_actor(sensor_data.inner) };

        let actor_id = if !actor_ptr.is_null() {
            Some(unsafe { carla_actor_get_id(actor_ptr) })
        } else {
            None
        };

        let other_actor_id = if !other_actor_ptr.is_null() {
            Some(unsafe { carla_actor_get_id(other_actor_ptr) })
        } else {
            None
        };

        Ok(Self {
            normal_impulse: Vector3::new(normal_impulse.x, normal_impulse.y, normal_impulse.z),
            actor_id,
            other_actor_id,
        })
    }

    /// Get the ID of the actor that experienced the collision
    pub fn actor_id(&self) -> Option<u32> {
        self.actor_id
    }

    /// Get the ID of the other actor involved in the collision
    pub fn other_actor_id(&self) -> Option<u32> {
        self.other_actor_id
    }
}

/// Lane invasion event data
#[derive(Debug)]
pub struct LaneInvasionEvent {
    actor_id: Option<u32>,
    crossed_lane_markings_count: usize,
}

impl LaneInvasionEvent {
    pub(crate) fn from_sensor_data(sensor_data: &SensorData) -> Result<Self> {
        let actor_ptr = unsafe { carla_lane_invasion_data_get_actor(sensor_data.inner) };
        let crossed_count =
            unsafe { carla_lane_invasion_data_get_crossed_lane_markings_count(sensor_data.inner) };

        let actor_id = if !actor_ptr.is_null() {
            Some(unsafe { carla_actor_get_id(actor_ptr) })
        } else {
            None
        };

        Ok(Self {
            actor_id,
            crossed_lane_markings_count: crossed_count,
        })
    }

    /// Get the ID of the actor that invaded the lane
    pub fn actor_id(&self) -> Option<u32> {
        self.actor_id
    }

    /// Get the number of crossed lane markings
    pub fn crossed_lane_markings_count(&self) -> usize {
        self.crossed_lane_markings_count
    }
}

/// Obstacle detection event data
#[derive(Debug)]
pub struct ObstacleDetectionEvent {
    actor_id: Option<u32>,
    other_actor_id: Option<u32>,
    pub distance: f32,
}

impl ObstacleDetectionEvent {
    pub(crate) fn from_sensor_data(sensor_data: &SensorData) -> Result<Self> {
        let actor_ptr = unsafe { carla_obstacle_detection_data_get_actor(sensor_data.inner) };
        let other_actor_ptr =
            unsafe { carla_obstacle_detection_data_get_other_actor(sensor_data.inner) };
        let distance = unsafe { carla_obstacle_detection_data_get_distance(sensor_data.inner) };

        let actor_id = if !actor_ptr.is_null() {
            Some(unsafe { carla_actor_get_id(actor_ptr) })
        } else {
            None
        };

        let other_actor_id = if !other_actor_ptr.is_null() {
            Some(unsafe { carla_actor_get_id(other_actor_ptr) })
        } else {
            None
        };

        Ok(Self {
            actor_id,
            other_actor_id,
            distance,
        })
    }

    /// Get the ID of the detecting actor
    pub fn actor_id(&self) -> Option<u32> {
        self.actor_id
    }

    /// Get the ID of the detected obstacle actor
    pub fn other_actor_id(&self) -> Option<u32> {
        self.other_actor_id
    }
}

/// DVS (Dynamic Vision Sensor) event array data (CARLA 0.10.0 feature)
#[derive(Debug)]
pub struct DvsEventArray {
    events: Vec<DvsEvent>,
    width: u32,
    height: u32,
    fov_angle: u64,
}

impl DvsEventArray {
    pub(crate) fn from_sensor_data(sensor_data: &SensorData) -> Result<Self> {
        let event_count = unsafe { carla_dvs_event_array_data_get_event_count(sensor_data.inner) };
        let width = unsafe { carla_dvs_event_array_data_get_width(sensor_data.inner) };
        let height = unsafe { carla_dvs_event_array_data_get_height(sensor_data.inner) };

        // Get the DVS event array data to access fov_angle
        let dvs_data = unsafe { carla_sensor_data_get_dvs_event_array(sensor_data.inner) };
        let fov_angle = dvs_data.fov_angle;

        let events_ptr = unsafe { carla_dvs_event_array_data_get_events(sensor_data.inner) };
        if events_ptr.is_null() && event_count > 0 {
            return Err(anyhow!("Null DVS events pointer"));
        }

        let events = if event_count > 0 {
            let events_slice = unsafe { std::slice::from_raw_parts(events_ptr, event_count) };
            events_slice
                .iter()
                .map(|e| DvsEvent {
                    x: e.x,
                    y: e.y,
                    timestamp: e.t,
                    polarity: e.pol,
                })
                .collect()
        } else {
            Vec::new()
        };

        Ok(Self {
            events,
            width,
            height,
            fov_angle,
        })
    }

    /// Get all DVS events
    pub fn events(&self) -> &[DvsEvent] {
        &self.events
    }

    /// Get sensor width
    pub fn width(&self) -> u32 {
        self.width
    }

    /// Get sensor height
    pub fn height(&self) -> u32 {
        self.height
    }

    /// Get field of view angle
    pub fn fov_angle(&self) -> u64 {
        self.fov_angle
    }

    /// Analyze DVS events to get statistics and activity information
    pub fn analyze_events(&self, sensor_data: &SensorData) -> Result<DvsAnalysis> {
        let mut analysis = carla_dvs_analysis_t {
            positive_events: 0,
            negative_events: 0,
            total_events: 0,
            event_rate: 0.0,
            active_region: carla_image_roi_t {
                x: 0,
                y: 0,
                width: 0,
                height: 0,
            },
            activity_density: 0.0,
        };

        let error = unsafe { carla_dvs_analyze_events(sensor_data.inner, &mut analysis) };
        check_carla_error(error)?;

        Ok(DvsAnalysis {
            positive_events: analysis.positive_events,
            negative_events: analysis.negative_events,
            total_events: analysis.total_events,
            event_rate: analysis.event_rate,
            active_region: ImageRegionOfInterest {
                x: analysis.active_region.x,
                y: analysis.active_region.y,
                width: analysis.active_region.width,
                height: analysis.active_region.height,
            },
            activity_density: analysis.activity_density,
        })
    }

    /// Filter events by polarity (positive or negative)
    pub fn filter_by_polarity(
        &self,
        sensor_data: &SensorData,
        positive_polarity: bool,
    ) -> Result<Vec<DvsEvent>> {
        let mut filtered_events_ptr: *mut carla_dvs_event_t = std::ptr::null_mut();
        let mut filtered_count: usize = 0;

        let error = unsafe {
            carla_dvs_filter_events_by_polarity(
                sensor_data.inner,
                positive_polarity,
                &mut filtered_events_ptr,
                &mut filtered_count,
            )
        };
        check_carla_error(error)?;

        if filtered_events_ptr.is_null() {
            return Ok(Vec::new());
        }

        let filtered_slice =
            unsafe { std::slice::from_raw_parts(filtered_events_ptr, filtered_count) };
        let events = filtered_slice
            .iter()
            .map(|e| DvsEvent {
                x: e.x,
                y: e.y,
                timestamp: e.t,
                polarity: e.pol,
            })
            .collect();

        // Free the allocated memory
        unsafe { carla_dvs_free_events(filtered_events_ptr) };

        Ok(events)
    }

    /// Filter events by time window
    pub fn filter_by_time(
        &self,
        sensor_data: &SensorData,
        start_time: i64,
        end_time: i64,
    ) -> Result<Vec<DvsEvent>> {
        let mut filtered_events_ptr: *mut carla_dvs_event_t = std::ptr::null_mut();
        let mut filtered_count: usize = 0;

        let error = unsafe {
            carla_dvs_filter_events_by_time(
                sensor_data.inner,
                start_time,
                end_time,
                &mut filtered_events_ptr,
                &mut filtered_count,
            )
        };
        check_carla_error(error)?;

        if filtered_events_ptr.is_null() {
            return Ok(Vec::new());
        }

        let filtered_slice =
            unsafe { std::slice::from_raw_parts(filtered_events_ptr, filtered_count) };
        let events = filtered_slice
            .iter()
            .map(|e| DvsEvent {
                x: e.x,
                y: e.y,
                timestamp: e.t,
                polarity: e.pol,
            })
            .collect();

        // Free the allocated memory
        unsafe { carla_dvs_free_events(filtered_events_ptr) };

        Ok(events)
    }

    /// Convert DVS events to image representation over a time window
    pub fn events_to_image(&self, sensor_data: &SensorData, time_window: f32) -> Result<Vec<u8>> {
        let mut image_data_ptr: *mut u8 = std::ptr::null_mut();
        let mut image_size: usize = 0;

        let error = unsafe {
            carla_dvs_events_to_image(
                sensor_data.inner,
                time_window,
                &mut image_data_ptr,
                &mut image_size,
            )
        };
        check_carla_error(error)?;

        if image_data_ptr.is_null() {
            return Err(anyhow!("Failed to generate DVS image representation"));
        }

        let image_slice = unsafe { std::slice::from_raw_parts(image_data_ptr, image_size) };
        let image_data = image_slice.to_vec();

        // Free the allocated memory
        unsafe { carla_image_free_data(image_data_ptr) };

        Ok(image_data)
    }
}

/// A single DVS event
#[derive(Debug, Clone, Copy)]
pub struct DvsEvent {
    pub x: u16,
    pub y: u16,
    pub timestamp: i64,
    pub polarity: bool, // true for positive, false for negative
}

/// Optical flow image data (CARLA 0.10.0 feature)
#[derive(Debug)]
pub struct OpticalFlowImage {
    pixels: Vec<OpticalFlowPixel>,
    width: u32,
    height: u32,
    fov: u32,
}

impl OpticalFlowImage {
    pub(crate) fn from_sensor_data(sensor_data: &SensorData) -> Result<Self> {
        let width = unsafe { carla_optical_flow_data_get_width(sensor_data.inner) };
        let height = unsafe { carla_optical_flow_data_get_height(sensor_data.inner) };
        let fov = unsafe { carla_image_data_get_fov(sensor_data.inner) };

        let pixels_ptr = unsafe { carla_optical_flow_data_get_pixels(sensor_data.inner) };
        if pixels_ptr.is_null() {
            return Err(anyhow!("Null optical flow pixels pointer"));
        }

        let pixel_count = (width * height) as usize;
        let pixels_slice = unsafe { std::slice::from_raw_parts(pixels_ptr, pixel_count) };
        let pixels = pixels_slice
            .iter()
            .map(|p| OpticalFlowPixel {
                flow_x: p.x,
                flow_y: p.y,
            })
            .collect();

        Ok(Self {
            pixels,
            width,
            height,
            fov,
        })
    }

    /// Get all optical flow pixels
    pub fn pixels(&self) -> &[OpticalFlowPixel] {
        &self.pixels
    }

    /// Get image width
    pub fn width(&self) -> u32 {
        self.width
    }

    /// Get image height
    pub fn height(&self) -> u32 {
        self.height
    }

    /// Get field of view
    pub fn fov(&self) -> u32 {
        self.fov
    }

    /// Convert to 2D flow field arrays (width x height for each flow component)
    pub fn to_flow_arrays(&self) -> (Array2<f32>, Array2<f32>) {
        let mut flow_x_data = Vec::with_capacity(self.pixels.len());
        let mut flow_y_data = Vec::with_capacity(self.pixels.len());

        for pixel in &self.pixels {
            flow_x_data.push(pixel.flow_x);
            flow_y_data.push(pixel.flow_y);
        }

        let flow_x =
            Array2::from_shape_vec((self.height as usize, self.width as usize), flow_x_data)
                .expect("Valid shape for flow_x array");

        let flow_y =
            Array2::from_shape_vec((self.height as usize, self.width as usize), flow_y_data)
                .expect("Valid shape for flow_y array");

        (flow_x, flow_y)
    }

    /// Analyze optical flow to get motion statistics and patterns
    pub fn analyze_optical_flow(&self, sensor_data: &SensorData) -> Result<OpticalFlowAnalysis> {
        let mut analysis = carla_optical_flow_analysis_t {
            average_flow: carla_vector3d_t {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            magnitude_avg: 0.0,
            magnitude_max: 0.0,
            moving_pixels: 0,
            motion_density: 0.0,
            dominant_direction: carla_vector3d_t {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        };

        let error = unsafe { carla_image_analyze_optical_flow(sensor_data.inner, &mut analysis) };
        check_carla_error(error)?;

        Ok(OpticalFlowAnalysis {
            average_flow: Vector3::new(
                analysis.average_flow.x,
                analysis.average_flow.y,
                analysis.average_flow.z,
            ),
            magnitude_avg: analysis.magnitude_avg,
            magnitude_max: analysis.magnitude_max,
            moving_pixels: analysis.moving_pixels,
            motion_density: analysis.motion_density,
            dominant_direction: Vector3::new(
                analysis.dominant_direction.x,
                analysis.dominant_direction.y,
                analysis.dominant_direction.z,
            ),
        })
    }

    /// Get optical flow magnitude as a 2D array
    pub fn get_magnitude_field(&self, sensor_data: &SensorData) -> Result<Array2<f32>> {
        let mut magnitude_data_ptr: *mut f32 = std::ptr::null_mut();
        let mut magnitude_size: usize = 0;

        let error = unsafe {
            carla_image_optical_flow_magnitude(
                sensor_data.inner,
                &mut magnitude_data_ptr,
                &mut magnitude_size,
            )
        };
        check_carla_error(error)?;

        if magnitude_data_ptr.is_null() {
            return Err(anyhow!("Failed to get optical flow magnitude data"));
        }

        let magnitude_slice =
            unsafe { std::slice::from_raw_parts(magnitude_data_ptr, magnitude_size) };
        let magnitude_data = magnitude_slice.to_vec();

        // Free the allocated memory
        unsafe { carla_optical_flow_free_data(magnitude_data_ptr) };

        Array2::from_shape_vec((self.height as usize, self.width as usize), magnitude_data)
            .map_err(|e| anyhow!("Failed to create magnitude array: {}", e))
    }

    /// Get optical flow direction as a 2D array (in radians)
    pub fn get_direction_field(&self, sensor_data: &SensorData) -> Result<Array2<f32>> {
        let mut direction_data_ptr: *mut f32 = std::ptr::null_mut();
        let mut direction_size: usize = 0;

        let error = unsafe {
            carla_image_optical_flow_direction(
                sensor_data.inner,
                &mut direction_data_ptr,
                &mut direction_size,
            )
        };
        check_carla_error(error)?;

        if direction_data_ptr.is_null() {
            return Err(anyhow!("Failed to get optical flow direction data"));
        }

        let direction_slice =
            unsafe { std::slice::from_raw_parts(direction_data_ptr, direction_size) };
        let direction_data = direction_slice.to_vec();

        // Free the allocated memory
        unsafe { carla_optical_flow_free_data(direction_data_ptr) };

        Array2::from_shape_vec((self.height as usize, self.width as usize), direction_data)
            .map_err(|e| anyhow!("Failed to create direction array: {}", e))
    }

    /// Detect motion regions in the optical flow field
    pub fn detect_motion_regions(
        &self,
        sensor_data: &SensorData,
        motion_threshold: f32,
    ) -> Result<Vec<ImageRegionOfInterest>> {
        let mut regions_ptr: *mut carla_image_roi_t = std::ptr::null_mut();
        let mut region_count: usize = 0;

        let error = unsafe {
            carla_image_detect_motion_regions(
                sensor_data.inner,
                motion_threshold,
                &mut regions_ptr,
                &mut region_count,
            )
        };
        check_carla_error(error)?;

        if regions_ptr.is_null() {
            return Ok(Vec::new());
        }

        let regions_slice = unsafe { std::slice::from_raw_parts(regions_ptr, region_count) };
        let regions = regions_slice
            .iter()
            .map(|r| ImageRegionOfInterest {
                x: r.x,
                y: r.y,
                width: r.width,
                height: r.height,
            })
            .collect();

        // Free the allocated memory
        unsafe { carla_image_free_array(regions_ptr as *mut _) };

        Ok(regions)
    }

    /// Calculate local flow vectors for computational efficiency
    pub fn calculate_local_vectors(&self) -> Array3<f32> {
        let mut data = Vec::with_capacity(self.pixels.len() * 3);
        for pixel in &self.pixels {
            data.push(pixel.flow_x);
            data.push(pixel.flow_y);
            data.push((pixel.flow_x * pixel.flow_x + pixel.flow_y * pixel.flow_y).sqrt());
            // magnitude
        }
        Array3::from_shape_vec((self.height as usize, self.width as usize, 3), data)
            .expect("Valid shape for local vectors array")
    }
}

/// A single optical flow pixel
#[derive(Debug, Clone, Copy)]
pub struct OpticalFlowPixel {
    pub flow_x: f32, // Horizontal flow component
    pub flow_y: f32, // Vertical flow component
}

/// Analysis result for DVS events
#[derive(Debug, Clone)]
pub struct DvsAnalysis {
    pub positive_events: u32,
    pub negative_events: u32,
    pub total_events: u32,
    pub event_rate: f32,
    pub active_region: ImageRegionOfInterest,
    pub activity_density: f32,
}

/// Analysis result for optical flow data
#[derive(Debug, Clone)]
pub struct OpticalFlowAnalysis {
    pub average_flow: Vector3<f32>,
    pub magnitude_avg: f32,
    pub magnitude_max: f32,
    pub moving_pixels: u32,
    pub motion_density: f32,
    pub dominant_direction: Vector3<f32>,
}

/// Region of Interest in an image
#[derive(Debug, Clone, Copy)]
pub struct ImageRegionOfInterest {
    pub x: u32,
    pub y: u32,
    pub width: u32,
    pub height: u32,
}

impl ImageRegionOfInterest {
    /// Create a new region of interest
    pub fn new(x: u32, y: u32, width: u32, height: u32) -> Self {
        Self {
            x,
            y,
            width,
            height,
        }
    }

    /// Get the area of the region
    pub fn area(&self) -> u32 {
        self.width * self.height
    }

    /// Check if a point is inside this region
    pub fn contains_point(&self, px: u32, py: u32) -> bool {
        px >= self.x && px < self.x + self.width && py >= self.y && py < self.y + self.height
    }

    /// Get the center point of the region
    pub fn center(&self) -> (u32, u32) {
        (self.x + self.width / 2, self.y + self.height / 2)
    }
}
