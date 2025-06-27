//! FFI-safe callback types for sensor data.

use std::os::raw::c_void;

/// FFI-safe callback function type for sensor data.
///
/// This callback is invoked from C++ when sensor data is available.
///
/// # Parameters
/// - `sensor_id`: The ID of the sensor that generated the data
/// - `data`: Pointer to the serialized sensor data
/// - `size`: Size of the data in bytes
/// - `user_data`: User-provided context pointer
///
/// # Safety
/// The data pointer is only valid for the duration of the callback.
/// The callback must not store the pointer or access it after returning.
pub type SensorDataCallback =
    unsafe extern "C" fn(sensor_id: u32, data: *const u8, size: usize, user_data: *mut c_void);

/// Sensor data type identifier for deserialization.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SensorDataType {
    /// RGB, depth, or semantic segmentation camera
    Image = 0,
    /// LiDAR point cloud
    LiDAR = 1,
    /// Radar detections
    Radar = 2,
    /// IMU measurements
    IMU = 3,
    /// GNSS location
    GNSS = 4,
    /// Collision event
    Collision = 5,
    /// Lane invasion event
    LaneInvasion = 6,
    /// Obstacle detection
    ObstacleDetection = 7,
    /// Dynamic Vision Sensor events
    DVS = 8,
    /// Semantic LiDAR
    SemanticLiDAR = 9,
    /// RSS (Road Safety) data
    RSS = 10,
}

/// Header for serialized sensor data.
///
/// This header is prepended to all sensor data passed to callbacks.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct SensorDataHeader {
    /// Type of sensor data
    pub data_type: SensorDataType,
    /// Frame number
    pub frame: u64,
    /// Timestamp in seconds
    pub timestamp: f64,
    /// Transform of the sensor (location and rotation)
    pub transform: crate::ffi::SimpleTransform,
}

/// Registration result for callbacks.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct CallbackRegistration {
    /// Whether registration was successful
    pub success: bool,
    /// Unique handle for this callback registration
    pub handle: u64,
}
