//! Sensor management and data types for CARLA 0.10.0

use crate::{geom::TransformExt, utils::check_carla_error};
use anyhow::{anyhow, Result};
use carla_sys::*;
use nalgebra::Isometry3;
use std::{
    ffi::c_void,
    sync::{Arc, Mutex},
};

mod data;

pub use data::*;

/// A sensor actor in the CARLA simulation.
pub struct Sensor {
    pub(crate) inner: *mut carla_sensor_t,
    // Keep track of callbacks to prevent them from being dropped
    _callback_data: Arc<Mutex<Option<Box<dyn FnMut(SensorData) + Send + 'static>>>>,
}

impl std::fmt::Debug for Sensor {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Sensor")
            .field("inner", &self.inner)
            .field("_callback_data", &"<callback>")
            .finish()
    }
}

impl Sensor {
    /// Create a Sensor from a raw C pointer.
    ///
    /// # Safety
    /// The pointer must be valid and not null, and must point to a sensor actor.
    pub(crate) fn from_raw_ptr(ptr: *mut carla_sensor_t) -> Result<Self> {
        if ptr.is_null() {
            return Err(anyhow!("Null sensor pointer"));
        }

        // Verify it's actually a sensor
        if !unsafe { carla_actor_is_sensor(ptr as *const carla_actor_t) } {
            return Err(anyhow!("Actor is not a sensor"));
        }

        Ok(Self {
            inner: ptr,
            _callback_data: Arc::new(Mutex::new(None)),
        })
    }

    /// Try to create a Sensor from an Actor.
    pub fn from_actor(actor: &crate::client::Actor) -> Option<Self> {
        let sensor_ptr = unsafe { carla_actor_as_sensor(actor.raw_ptr()) };
        if sensor_ptr.is_null() {
            None
        } else {
            Self::from_raw_ptr(sensor_ptr).ok()
        }
    }

    /// Start listening for sensor data with a callback.
    pub fn listen<F>(&mut self, callback: F) -> Result<()>
    where
        F: FnMut(SensorData) + Send + 'static,
    {
        // Store the callback in our mutex
        let callback_box = Box::new(callback);
        *self._callback_data.lock().unwrap() = Some(callback_box);

        // Get raw pointer to the callback data
        let callback_ptr = self._callback_data.as_ref() as *const _ as *mut c_void;

        let error =
            unsafe { carla_sensor_listen(self.inner, Some(sensor_callback_wrapper), callback_ptr) };
        check_carla_error(error)
    }

    /// Stop listening for sensor data.
    pub fn stop(&mut self) -> Result<()> {
        let error = unsafe { carla_sensor_stop(self.inner) };
        // Clear the callback
        *self._callback_data.lock().unwrap() = None;
        check_carla_error(error)
    }

    /// Check if the sensor is currently listening.
    pub fn is_listening(&self) -> bool {
        unsafe { carla_sensor_is_listening(self.inner) }
    }
}

// Note: Sensor doesn't implement Drop because it's a specialized Actor,
// and the underlying carla_actor_t will be freed when the Actor is dropped

// SAFETY: Sensor wraps a thread-safe C API
unsafe impl Send for Sensor {}
unsafe impl Sync for Sensor {}

/// C callback wrapper that forwards to Rust closures
extern "C" fn sensor_callback_wrapper(data: *mut carla_sensor_data_t, user_data: *mut c_void) {
    if data.is_null() || user_data.is_null() {
        return;
    }

    unsafe {
        // Reconstruct the Arc from the raw pointer
        let callback_data =
            &*(user_data as *const Arc<Mutex<Option<Box<dyn FnMut(SensorData) + Send + 'static>>>>);

        if let Ok(mut guard) = callback_data.lock() {
            if let Some(ref mut callback) = *guard {
                if let Ok(sensor_data) = SensorData::from_raw_ptr(data) {
                    callback(sensor_data);
                }
            }
        }
    }
}

/// Base sensor data that all sensor types share.
#[derive(Debug)]
pub struct SensorData {
    // We don't own this pointer - it's managed by CARLA
    pub(crate) inner: *const carla_sensor_data_t,
    pub(crate) data_type: SensorDataType,
}

impl SensorData {
    /// Create SensorData from a raw C pointer.
    ///
    /// # Safety
    /// The pointer must be valid and not null.
    pub(crate) fn from_raw_ptr(ptr: *const carla_sensor_data_t) -> Result<Self> {
        if ptr.is_null() {
            return Err(anyhow!("Null sensor data pointer"));
        }

        let data_type = unsafe { carla_sensor_data_get_type(ptr) };
        let data_type = SensorDataType::from_c_type(data_type);

        Ok(Self {
            inner: ptr,
            data_type,
        })
    }

    /// Get the type of sensor data.
    pub fn data_type(&self) -> SensorDataType {
        self.data_type
    }

    /// Get the frame number.
    pub fn frame(&self) -> u64 {
        unsafe { carla_sensor_data_get_frame(self.inner) }
    }

    /// Get the timestamp.
    pub fn timestamp(&self) -> f64 {
        unsafe { carla_sensor_data_get_timestamp(self.inner) }
    }

    /// Get the sensor transform.
    pub fn transform(&self) -> Isometry3<f32> {
        let c_transform = unsafe { carla_sensor_data_get_transform(self.inner) };
        crate::geom::Transform::from_c_transform(c_transform).to_na()
    }

    /// Try to convert to ImageData if this is image sensor data.
    pub fn try_into_image(&self) -> Option<ImageData> {
        match self.data_type {
            SensorDataType::Image => ImageData::from_sensor_data(self).ok(),
            _ => None,
        }
    }

    /// Try to convert to LidarData if this is LiDAR sensor data.
    pub fn try_into_lidar(&self) -> Option<LidarData> {
        match self.data_type {
            SensorDataType::Lidar => LidarData::from_sensor_data(self).ok(),
            _ => None,
        }
    }

    /// Try to convert to SemanticLidarData if this is semantic LiDAR sensor data.
    pub fn try_into_semantic_lidar(&self) -> Option<SemanticLidarData> {
        match self.data_type {
            SensorDataType::SemanticLidar => SemanticLidarData::from_sensor_data(self).ok(),
            _ => None,
        }
    }

    /// Try to convert to RadarData if this is radar sensor data.
    pub fn try_into_radar(&self) -> Option<RadarData> {
        match self.data_type {
            SensorDataType::Radar => RadarData::from_sensor_data(self).ok(),
            _ => None,
        }
    }

    /// Try to convert to ImuData if this is IMU sensor data.
    pub fn try_into_imu(&self) -> Option<ImuData> {
        match self.data_type {
            SensorDataType::Imu => ImuData::from_sensor_data(self).ok(),
            _ => None,
        }
    }

    /// Try to convert to GnssData if this is GNSS sensor data.
    pub fn try_into_gnss(&self) -> Option<GnssData> {
        match self.data_type {
            SensorDataType::Gnss => GnssData::from_sensor_data(self).ok(),
            _ => None,
        }
    }

    /// Try to convert to CollisionEvent if this is collision event data.
    pub fn try_into_collision(&self) -> Option<CollisionEvent> {
        match self.data_type {
            SensorDataType::Collision => CollisionEvent::from_sensor_data(self).ok(),
            _ => None,
        }
    }

    /// Try to convert to LaneInvasionEvent if this is lane invasion event data.
    pub fn try_into_lane_invasion(&self) -> Option<LaneInvasionEvent> {
        match self.data_type {
            SensorDataType::LaneInvasion => LaneInvasionEvent::from_sensor_data(self).ok(),
            _ => None,
        }
    }

    /// Try to convert to ObstacleDetectionEvent if this is obstacle detection event data.
    pub fn try_into_obstacle_detection(&self) -> Option<ObstacleDetectionEvent> {
        match self.data_type {
            SensorDataType::ObstacleDetection => {
                ObstacleDetectionEvent::from_sensor_data(self).ok()
            }
            _ => None,
        }
    }

    /// Try to convert to DvsEventArray if this is DVS event data.
    pub fn try_into_dvs_events(&self) -> Option<DvsEventArray> {
        match self.data_type {
            SensorDataType::DvsEventArray => DvsEventArray::from_sensor_data(self).ok(),
            _ => None,
        }
    }

    /// Try to convert to OpticalFlowImage if this is optical flow data.
    pub fn try_into_optical_flow(&self) -> Option<OpticalFlowImage> {
        match self.data_type {
            SensorDataType::OpticalFlowImage => OpticalFlowImage::from_sensor_data(self).ok(),
            _ => None,
        }
    }
}

/// Enumeration of sensor data types.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SensorDataType {
    Unknown,
    Image,
    Lidar,
    SemanticLidar,
    Radar,
    Imu,
    Gnss,
    Collision,
    LaneInvasion,
    ObstacleDetection,
    DvsEventArray,
    OpticalFlowImage,
    NormalsImage,
}

impl SensorDataType {
    pub(crate) fn from_c_type(c_type: carla_sensor_data_type_t) -> Self {
        match c_type {
            carla_sensor_data_type_t_CARLA_SENSOR_DATA_IMAGE => Self::Image,
            carla_sensor_data_type_t_CARLA_SENSOR_DATA_LIDAR => Self::Lidar,
            carla_sensor_data_type_t_CARLA_SENSOR_DATA_SEMANTIC_LIDAR => Self::SemanticLidar,
            carla_sensor_data_type_t_CARLA_SENSOR_DATA_RADAR => Self::Radar,
            carla_sensor_data_type_t_CARLA_SENSOR_DATA_IMU => Self::Imu,
            carla_sensor_data_type_t_CARLA_SENSOR_DATA_GNSS => Self::Gnss,
            carla_sensor_data_type_t_CARLA_SENSOR_DATA_COLLISION => Self::Collision,
            carla_sensor_data_type_t_CARLA_SENSOR_DATA_LANE_INVASION => Self::LaneInvasion,
            carla_sensor_data_type_t_CARLA_SENSOR_DATA_OBSTACLE_DETECTION => {
                Self::ObstacleDetection
            }
            carla_sensor_data_type_t_CARLA_SENSOR_DATA_DVS_EVENT_ARRAY => Self::DvsEventArray,
            carla_sensor_data_type_t_CARLA_SENSOR_DATA_OPTICAL_FLOW_IMAGE => Self::OpticalFlowImage,
            carla_sensor_data_type_t_CARLA_SENSOR_DATA_NORMALS_IMAGE => Self::NormalsImage,
            _ => Self::Unknown,
        }
    }
}
