//! Sensor actor implementation for CARLA.

use crate::ffi::{self, Actor, Sensor};
use cxx::SharedPtr;

/// High-level wrapper for CARLA Sensor
pub struct SensorWrapper {
    inner: SharedPtr<Sensor>,
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

    /// Get the last image data
    pub fn get_last_image_data(&self) -> ImageData {
        let simple_data = ffi::Sensor_GetLastImageData(&self.inner);
        ImageData::from(simple_data)
    }

    /// Get image data into provided buffer
    pub fn get_image_data_buffer(&self, buffer: &mut [u8]) -> bool {
        ffi::Sensor_GetImageDataBuffer(&self.inner, buffer)
    }

    /// Get the last LiDAR data
    pub fn get_last_lidar_data(&self) -> LiDARData {
        let points = ffi::Sensor_GetLastLiDARData(&self.inner);
        LiDARData::from(points)
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
}

/// Image sensor data
#[derive(Debug, Clone)]
pub struct ImageData {
    pub width: u32,
    pub height: u32,
    pub fov: f32,
    pub data_size: usize,
}

impl From<ffi::SimpleImageData> for ImageData {
    fn from(simple: ffi::SimpleImageData) -> Self {
        Self {
            width: simple.width,
            height: simple.height,
            fov: simple.fov,
            data_size: simple.data_size,
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
    pub other_actor_id: u32,
    pub normal_impulse: crate::ffi::SimpleVector3D,
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
    Raw(Vec<u8>),
}
