//! Sensor actor implementation for CARLA.

use crate::ffi::{self, Actor, Sensor};
use cxx::SharedPtr;

/// High-level wrapper for CARLA Sensor
pub struct SensorWrapper {
    inner: SharedPtr<Sensor>,
}

impl std::fmt::Debug for SensorWrapper {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SensorWrapper")
            .field("inner", &"SharedPtr<Sensor>")
            .finish()
    }
}

impl SensorWrapper {
    /// Create a SensorWrapper from an Actor (performs cast)
    pub fn from_actor(actor: &Actor) -> Option<Self> {
        let sensor_ptr = ffi::Actor_CastToSensor(actor);
        if sensor_ptr.is_null() {
            None
        } else {
            Some(Self { inner: sensor_ptr })
        }
    }

    /// Start the sensor listening
    pub fn listen(&self) {
        ffi::Sensor_Listen(&self.inner);
    }

    /// Stop the sensor
    pub fn stop(&self) {
        ffi::Sensor_Stop(&self.inner);
    }

    /// Check if sensor is listening
    pub fn is_listening(&self) -> bool {
        ffi::Sensor_IsListening(&self.inner)
    }

    /// Check if new data is available
    pub fn has_new_data(&self) -> bool {
        ffi::Sensor_HasNewData(&self.inner)
    }

    /// Check if this sensor is a camera
    pub fn is_camera(&self) -> bool {
        ffi::Sensor_IsCamera(&self.inner)
    }

    /// Get the camera type (RGB, Depth, etc.)
    pub fn get_camera_type(&self) -> CameraType {
        CameraType::from(ffi::Sensor_GetCameraType(&self.inner))
    }

    /// Get the last image data
    pub fn get_last_image_data(&self) -> ImageData {
        let simple_data = ffi::Sensor_GetLastImageData(&self.inner);
        ImageData::from(simple_data)
    }

    /// Get image data into provided buffer
    pub fn get_image_data_buffer(&self, buffer: &mut [u8]) -> bool {
        ffi::Sensor_GetImageDataBuffer(&self.inner, buffer)
    }

    /// Get the last LiDAR data (points only)
    pub fn get_last_lidar_data(&self) -> LiDARData {
        let points = ffi::Sensor_GetLastLiDARData(&self.inner);
        LiDARData::from(points)
    }

    /// Get the last LiDAR data with full metadata
    pub fn get_last_lidar_data_full(&self) -> LiDARDataFull {
        let data = ffi::bridge::Sensor_GetLastLiDARDataFull(&self.inner);
        LiDARDataFull::from(data)
    }

    /// Get the last radar data
    pub fn get_last_radar_data(&self) -> RadarData {
        let detections = ffi::Sensor_GetLastRadarData(&self.inner);
        RadarData::from(detections)
    }

    /// Get the last IMU data
    pub fn get_last_imu_data(&self) -> IMUData {
        let simple_data = ffi::Sensor_GetLastIMUData(&self.inner);
        IMUData::from(simple_data)
    }

    /// Get the last GNSS data
    pub fn get_last_gnss_data(&self) -> GNSSData {
        let simple_data = ffi::Sensor_GetLastGNSSData(&self.inner);
        GNSSData::from(simple_data)
    }

    /// Get the last collision data
    pub fn get_last_collision_data(&self) -> CollisionData {
        let simple_data = ffi::Sensor_GetLastCollisionData(&self.inner);
        CollisionData::from(simple_data)
    }

    /// Get the last lane invasion data
    pub fn get_last_lane_invasion_data(&self) -> LaneInvasionData {
        let markings = ffi::Sensor_GetLastLaneInvasionData(&self.inner);
        LaneInvasionData::from(markings)
    }

    // Advanced sensor data methods

    /// Get the last Dynamic Vision Sensor (DVS) data
    pub fn get_last_dvs_data(&self) -> DVSEventArray {
        let simple_data = ffi::bridge::Sensor_GetLastDVSData(&self.inner);
        DVSEventArray::from(simple_data)
    }

    /// Get the last Obstacle Detection event
    pub fn get_last_obstacle_detection_data(&self) -> ObstacleDetectionEvent {
        let simple_data = ffi::bridge::Sensor_GetLastObstacleDetectionData(&self.inner);
        ObstacleDetectionEvent::from(simple_data)
    }

    /// Get the last Semantic LiDAR data
    pub fn get_last_semantic_lidar_data(&self) -> SemanticLidarData {
        let simple_data = ffi::bridge::Sensor_GetLastSemanticLidarData(&self.inner);
        SemanticLidarData::from(simple_data)
    }

    /// Get the last RSS (Road Safety) response
    pub fn get_last_rss_data(&self) -> RssResponse {
        let simple_data = ffi::bridge::Sensor_GetLastRssData(&self.inner);
        RssResponse::from(simple_data)
    }

    /// Get the inner sensor for FFI calls
    pub fn get_inner_sensor(&self) -> &SharedPtr<Sensor> {
        &self.inner
    }
}

/// Camera sensor type
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CameraType {
    RGB = 0,
    Depth = 1,
    SemanticSegmentation = 2,
    InstanceSegmentation = 3,
    OpticalFlow = 4,
    DVS = 5,
}

impl From<u8> for CameraType {
    fn from(value: u8) -> Self {
        match value {
            0 => CameraType::RGB,
            1 => CameraType::Depth,
            2 => CameraType::SemanticSegmentation,
            3 => CameraType::InstanceSegmentation,
            4 => CameraType::OpticalFlow,
            5 => CameraType::DVS,
            _ => CameraType::RGB, // Default
        }
    }
}

/// Image sensor data
#[derive(Debug, Clone)]
pub struct ImageData {
    pub width: u32,
    pub height: u32,
    pub fov: f32,
    pub timestamp: crate::time::Timestamp,
    pub transform: crate::ffi::SimpleTransform,
    pub sensor_id: u32,
    pub data: Vec<u8>,
}

impl From<ffi::SimpleImageData> for ImageData {
    fn from(simple: ffi::SimpleImageData) -> Self {
        Self {
            width: simple.width,
            height: simple.height,
            fov: simple.fov,
            timestamp: crate::time::Timestamp::from(simple.timestamp),
            transform: simple.transform,
            sensor_id: simple.sensor_id,
            data: simple.data,
        }
    }
}

impl ImageData {
    /// Get the size of the image data in bytes
    pub fn data_size(&self) -> usize {
        self.data.len()
    }

    /// Check if the image data is empty
    pub fn is_empty(&self) -> bool {
        self.data.is_empty() || self.width == 0 || self.height == 0
    }

    /// Get pixel data at specific coordinates (BGRA format)
    pub fn get_pixel(&self, x: u32, y: u32) -> Option<[u8; 4]> {
        if x >= self.width || y >= self.height {
            return None;
        }

        let idx = ((y * self.width + x) * 4) as usize;
        if idx + 3 < self.data.len() {
            Some([
                self.data[idx],     // B
                self.data[idx + 1], // G
                self.data[idx + 2], // R
                self.data[idx + 3], // A
            ])
        } else {
            None
        }
    }
}

/// LiDAR sensor data
#[derive(Debug, Clone)]
pub struct LiDARData {
    pub points: Vec<LiDARPoint>,
}

impl From<Vec<ffi::SimpleLiDARPoint>> for LiDARData {
    fn from(points: Vec<ffi::SimpleLiDARPoint>) -> Self {
        let points = points
            .into_iter()
            .map(|p| LiDARPoint {
                x: p.x,
                y: p.y,
                z: p.z,
                intensity: p.intensity,
            })
            .collect();
        Self { points }
    }
}

/// LiDAR sensor data with full metadata
#[derive(Debug, Clone)]
pub struct LiDARDataFull {
    pub timestamp: crate::time::Timestamp,
    pub transform: crate::ffi::SimpleTransform,
    pub sensor_id: u32,
    pub channels: u32,
    pub horizontal_fov: f32,
    pub points_per_second: u32,
    pub points: Vec<LiDARPoint>,
}

impl From<ffi::bridge::SimpleLiDARData> for LiDARDataFull {
    fn from(simple: ffi::bridge::SimpleLiDARData) -> Self {
        let points = simple
            .points
            .into_iter()
            .map(|p| LiDARPoint {
                x: p.x,
                y: p.y,
                z: p.z,
                intensity: p.intensity,
            })
            .collect();

        Self {
            timestamp: crate::time::Timestamp::from(simple.timestamp),
            transform: simple.transform,
            sensor_id: simple.sensor_id,
            channels: simple.channels,
            horizontal_fov: simple.horizontal_fov,
            points_per_second: simple.points_per_second,
            points,
        }
    }
}

/// LiDAR point
#[derive(Debug, Clone, Copy)]
pub struct LiDARPoint {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub intensity: f32,
}

/// Radar sensor data
#[derive(Debug, Clone)]
pub struct RadarData {
    pub detections: Vec<RadarDetection>,
}

impl From<Vec<ffi::SimpleRadarDetection>> for RadarData {
    fn from(detections: Vec<ffi::SimpleRadarDetection>) -> Self {
        let detections = detections
            .into_iter()
            .map(|d| RadarDetection {
                velocity: d.velocity,
                azimuth: d.azimuth,
                altitude: d.altitude,
                depth: d.depth,
            })
            .collect();
        Self { detections }
    }
}

/// Radar detection
#[derive(Debug, Clone, Copy)]
pub struct RadarDetection {
    pub velocity: f32,
    pub azimuth: f32,
    pub altitude: f32,
    pub depth: f32,
}

/// IMU sensor data
#[derive(Debug, Clone, Copy)]
pub struct IMUData {
    pub accelerometer_x: f64,
    pub accelerometer_y: f64,
    pub accelerometer_z: f64,
    pub gyroscope_x: f64,
    pub gyroscope_y: f64,
    pub gyroscope_z: f64,
    pub compass: f64,
}

impl From<ffi::SimpleIMUData> for IMUData {
    fn from(simple: ffi::SimpleIMUData) -> Self {
        Self {
            accelerometer_x: simple.accelerometer_x,
            accelerometer_y: simple.accelerometer_y,
            accelerometer_z: simple.accelerometer_z,
            gyroscope_x: simple.gyroscope_x,
            gyroscope_y: simple.gyroscope_y,
            gyroscope_z: simple.gyroscope_z,
            compass: simple.compass,
        }
    }
}

/// GNSS sensor data
#[derive(Debug, Clone, Copy)]
pub struct GNSSData {
    pub latitude: f64,
    pub longitude: f64,
    pub altitude: f64,
}

impl From<ffi::SimpleGNSSData> for GNSSData {
    fn from(simple: ffi::SimpleGNSSData) -> Self {
        Self {
            latitude: simple.latitude,
            longitude: simple.longitude,
            altitude: simple.altitude,
        }
    }
}

/// Collision sensor data
#[derive(Debug, Clone, Copy)]
pub struct CollisionData {
    pub timestamp: crate::time::Timestamp,
    pub transform: crate::ffi::SimpleTransform,
    pub sensor_id: u32,
    pub other_actor_id: u32,
    pub normal_impulse: crate::ffi::bridge::SimpleVector3D,
}

impl CollisionData {
    pub fn impulse_magnitude(&self) -> f64 {
        (self.normal_impulse.x * self.normal_impulse.x
            + self.normal_impulse.y * self.normal_impulse.y
            + self.normal_impulse.z * self.normal_impulse.z)
            .sqrt()
    }
}

impl From<ffi::SimpleCollisionData> for CollisionData {
    fn from(simple: ffi::SimpleCollisionData) -> Self {
        Self {
            timestamp: crate::time::Timestamp::from(simple.timestamp),
            transform: simple.transform,
            sensor_id: simple.sensor_id,
            other_actor_id: simple.other_actor_id,
            normal_impulse: simple.normal_impulse,
        }
    }
}

/// Lane invasion sensor data
#[derive(Debug, Clone)]
pub struct LaneInvasionData {
    pub crossed_lane_markings: Vec<CrossedLaneMarking>,
}

impl From<Vec<ffi::SimpleCrossedLaneMarking>> for LaneInvasionData {
    fn from(markings: Vec<ffi::SimpleCrossedLaneMarking>) -> Self {
        let markings = markings.into_iter().map(CrossedLaneMarking::from).collect();
        Self {
            crossed_lane_markings: markings,
        }
    }
}

/// Crossed lane marking
#[derive(Debug, Clone)]
pub struct CrossedLaneMarking {
    pub lane_type: u32,
    pub color: u32,
    pub lane_change: u8,
}

impl From<ffi::SimpleCrossedLaneMarking> for CrossedLaneMarking {
    fn from(simple: ffi::SimpleCrossedLaneMarking) -> Self {
        Self {
            lane_type: simple.lane_type,
            color: simple.color,
            lane_change: simple.lane_change,
        }
    }
}

// Advanced sensor data types

/// Dynamic Vision Sensor (DVS) event
#[derive(Debug, Clone, Copy)]
pub struct DVSEvent {
    pub x: u16,    // X pixel coordinate
    pub y: u16,    // Y pixel coordinate
    pub t: i64,    // Timestamp in nanoseconds
    pub pol: bool, // Polarity (true=positive, false=negative)
}

impl From<ffi::bridge::SimpleDVSEvent> for DVSEvent {
    fn from(simple: ffi::bridge::SimpleDVSEvent) -> Self {
        Self {
            x: simple.x,
            y: simple.y,
            t: simple.t,
            pol: simple.pol,
        }
    }
}

/// Dynamic Vision Sensor (DVS) event array
#[derive(Debug, Clone)]
pub struct DVSEventArray {
    pub width: u32,            // Image width in pixels
    pub height: u32,           // Image height in pixels
    pub fov_angle: f32,        // Horizontal field of view
    pub events: Vec<DVSEvent>, // Array of DVS events
}

impl From<ffi::bridge::SimpleDVSEventArray> for DVSEventArray {
    fn from(simple: ffi::bridge::SimpleDVSEventArray) -> Self {
        let events = simple.events.into_iter().map(DVSEvent::from).collect();
        Self {
            width: simple.width,
            height: simple.height,
            fov_angle: simple.fov_angle,
            events,
        }
    }
}

/// Obstacle Detection event
#[derive(Debug, Clone, Copy)]
pub struct ObstacleDetectionEvent {
    pub self_actor_id: u32,  // Sensor's parent actor ID
    pub other_actor_id: u32, // Detected obstacle actor ID
    pub distance: f32,       // Distance to obstacle in meters
}

impl From<ffi::bridge::SimpleObstacleDetectionEvent> for ObstacleDetectionEvent {
    fn from(simple: ffi::bridge::SimpleObstacleDetectionEvent) -> Self {
        Self {
            self_actor_id: simple.self_actor_id,
            other_actor_id: simple.other_actor_id,
            distance: simple.distance,
        }
    }
}

/// Semantic LiDAR detection point
#[derive(Debug, Clone, Copy)]
pub struct SemanticLidarDetection {
    pub point: crate::ffi::bridge::SimpleLocation, // 3D point location (x, y, z)
    pub cos_inc_angle: f32,                        // Cosine of incidence angle
    pub object_idx: u32,                           // Object instance index
    pub object_tag: u32,                           // Semantic tag/label
}

impl From<ffi::bridge::SimpleSemanticLidarDetection> for SemanticLidarDetection {
    fn from(simple: ffi::bridge::SimpleSemanticLidarDetection) -> Self {
        Self {
            point: simple.point,
            cos_inc_angle: simple.cos_inc_angle,
            object_idx: simple.object_idx,
            object_tag: simple.object_tag,
        }
    }
}

/// Semantic LiDAR sensor data
#[derive(Debug, Clone)]
pub struct SemanticLidarData {
    pub horizontal_angle: f32, // Current horizontal rotation angle
    pub channel_count: u32,    // Number of laser channels
    pub detections: Vec<SemanticLidarDetection>, // Array of detections
}

impl From<ffi::bridge::SimpleSemanticLidarData> for SemanticLidarData {
    fn from(simple: ffi::bridge::SimpleSemanticLidarData) -> Self {
        let detections = simple
            .detections
            .into_iter()
            .map(SemanticLidarDetection::from)
            .collect();
        Self {
            horizontal_angle: simple.horizontal_angle,
            channel_count: simple.channel_count,
            detections,
        }
    }
}

/// RSS (Road Safety) response
#[derive(Debug, Clone)]
pub struct RssResponse {
    pub response_valid: bool,          // RSS calculation validity
    pub proper_response: String,       // RSS proper response (serialized)
    pub rss_state_snapshot: String,    // Current RSS state (serialized)
    pub situation_snapshot: String,    // Situation analysis (serialized)
    pub world_model: String,           // World model used (serialized)
    pub ego_dynamics_on_route: String, // Ego vehicle dynamics (serialized)
}

impl From<ffi::bridge::SimpleRssResponse> for RssResponse {
    fn from(simple: ffi::bridge::SimpleRssResponse) -> Self {
        Self {
            response_valid: simple.response_valid,
            proper_response: simple.proper_response,
            rss_state_snapshot: simple.rss_state_snapshot,
            situation_snapshot: simple.situation_snapshot,
            world_model: simple.world_model,
            ego_dynamics_on_route: simple.ego_dynamics_on_route,
        }
    }
}

/// Sensor data enum
#[derive(Debug, Clone)]
pub enum SensorData {
    Image(ImageData),
    LiDAR(LiDARData),
    Radar(RadarData),
    IMU(IMUData),
    GNSS(GNSSData),
    Collision(CollisionData),
    LaneInvasion(LaneInvasionData),
    // Advanced sensor types
    DVS(DVSEventArray),
    ObstacleDetection(ObstacleDetectionEvent),
    SemanticLiDAR(SemanticLidarData),
    RSS(RssResponse),
    Raw(Vec<u8>),
}
