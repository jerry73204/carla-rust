//! Sensor callback implementation with true callbacks.

use crate::{
    actor::{ActorExt, Sensor},
    error::{CarlaResult, SensorError},
    road::LaneMarkingType,
    sensor_data::{
        CollisionData, DVSData, DepthImageData, GNSSData, IMUData, ImageData,
        InstanceSegmentationImageData, LaneChange, LaneInvasionData, LaneMarkingColor,
        LaneMarkingInfo, LiDARData, ObstacleDetectionData, RGBImageData, RSSData, RadarData,
        SemanticLiDARData, SemanticSegmentationImageData,
    },
};
use carla_sys::{
    callback::{SensorDataHeader, SensorDataType},
    sensor_callback_ffi,
};
use std::{
    collections::HashMap,
    sync::{Arc, Mutex},
};

/// Type-erased sensor data callback.
pub type SensorCallback = Box<dyn Fn(CallbackSensorData) + Send + 'static>;

/// Sensor data variants that can be received in callbacks.
#[derive(Debug, Clone)]
pub enum CallbackSensorData {
    /// RGB camera data
    RGB(RGBImageData),
    /// Depth camera data
    Depth(DepthImageData),
    /// Semantic segmentation camera data
    SemanticSegmentation(SemanticSegmentationImageData),
    /// Instance segmentation camera data
    InstanceSegmentation(InstanceSegmentationImageData),
    /// LiDAR point cloud data
    LiDAR(LiDARData),
    /// Semantic LiDAR data
    SemanticLiDAR(SemanticLiDARData),
    /// Radar detection data
    Radar(RadarData),
    /// IMU measurement data
    IMU(IMUData),
    /// GNSS location data
    GNSS(GNSSData),
    /// Collision event data
    Collision(CollisionData),
    /// Lane invasion event data
    LaneInvasion(LaneInvasionData),
    /// Obstacle detection data
    ObstacleDetection(ObstacleDetectionData),
    /// DVS event array data
    DVS(DVSData),
    /// RSS data
    RSS(RSSData),
}

/// Manages sensor callbacks for a sensor.
pub struct SensorCallbackManager<'a> {
    /// Reference to the sensor
    sensor: &'a Sensor,
    /// Registered callbacks
    callbacks: Arc<Mutex<HashMap<u64, SensorCallback>>>,
    /// Active callback handles
    handles: Vec<u64>,
}

impl<'a> SensorCallbackManager<'a> {
    /// Create a new callback manager for a sensor.
    pub fn new(sensor: &'a Sensor) -> Self {
        Self {
            sensor,
            callbacks: Arc::new(Mutex::new(HashMap::new())),
            handles: Vec::new(),
        }
    }

    /// Register a callback for this sensor.
    ///
    /// The callback will be invoked whenever new sensor data is available.
    pub fn register_callback<F>(&mut self, callback: F) -> CarlaResult<u64>
    where
        F: Fn(CallbackSensorData) + Send + 'static,
    {
        // Create a boxed callback
        let boxed_callback = Box::new(callback);

        // Create callback context
        let context = CallbackContext {
            _sensor_id: self.sensor.id(),
            callbacks: Arc::clone(&self.callbacks),
        };

        // Box the context for FFI
        let context_ptr = Box::into_raw(Box::new(context)) as *mut u8;

        // Register the FFI callback
        let handle = unsafe {
            sensor_callback_ffi::Sensor_RegisterCallback(
                self.sensor.inner.get_inner_sensor(),
                sensor_data_callback_trampoline,
                context_ptr,
            )
        };

        if handle == 0 {
            // Registration failed, clean up
            unsafe {
                let _ = Box::from_raw(context_ptr as *mut CallbackContext);
            }
            return Err(SensorError::CallbackFailed(
                "Failed to register callback with sensor".to_string(),
            )
            .into());
        }

        // Store the callback
        {
            let mut callbacks = self.callbacks.lock().unwrap();
            callbacks.insert(handle, boxed_callback);
        }

        self.handles.push(handle);
        Ok(handle)
    }

    /// Unregister a callback.
    pub fn unregister_callback(&mut self, handle: u64) -> CarlaResult<()> {
        // Unregister from FFI
        let success = unsafe { sensor_callback_ffi::Sensor_UnregisterCallback(handle) };

        if !success {
            return Err(
                SensorError::CallbackFailed("Failed to unregister callback".to_string()).into(),
            );
        }

        // Remove from local storage
        {
            let mut callbacks = self.callbacks.lock().unwrap();
            callbacks.remove(&handle);
        }

        self.handles.retain(|&h| h != handle);
        Ok(())
    }

    /// Clear all callbacks for this sensor.
    pub fn clear_callbacks(&mut self) {
        // Clear FFI callbacks
        unsafe {
            sensor_callback_ffi::Sensor_ClearCallbacks(self.sensor.inner.get_inner_sensor());
        }

        // Clear local storage
        {
            let mut callbacks = self.callbacks.lock().unwrap();
            callbacks.clear();
        }

        self.handles.clear();
    }
}

/// Context passed to FFI callbacks.
struct CallbackContext {
    _sensor_id: u32,
    callbacks: Arc<Mutex<HashMap<u64, SensorCallback>>>,
}

/// FFI callback trampoline that deserializes data and calls Rust callbacks.
unsafe extern "C" fn sensor_data_callback_trampoline(
    _sensor_id: u32,
    data: *const u8,
    size: usize,
    user_data: *mut u8,
) {
    if data.is_null() || user_data.is_null() {
        return;
    }

    // Get the context
    let context = &*(user_data as *const CallbackContext);

    // Deserialize the sensor data
    let data_slice = std::slice::from_raw_parts(data, size);
    match deserialize_sensor_data(data_slice) {
        Ok(sensor_data) => {
            // Call all registered callbacks
            if let Ok(callbacks) = context.callbacks.lock() {
                for callback in callbacks.values() {
                    callback(sensor_data.clone());
                }
            }
        }
        Err(e) => {
            eprintln!("Failed to deserialize sensor data: {e}");
        }
    }
}

/// Deserialize sensor data from FFI buffer.
fn deserialize_sensor_data(data: &[u8]) -> CarlaResult<CallbackSensorData> {
    if data.len() < std::mem::size_of::<SensorDataHeader>() {
        return Err(SensorError::DataConversion(
            "Buffer too small for sensor data header".to_string(),
        )
        .into());
    }

    // Read header
    let header = unsafe { &*(data.as_ptr() as *const SensorDataHeader) };
    let payload = &data[std::mem::size_of::<SensorDataHeader>()..];

    // Create timestamp and transform from header
    let timestamp = crate::time::Timestamp::new(
        header.frame,
        header.timestamp,
        0.016, // Default delta
        header.timestamp,
    );

    // Extract transform from header
    let transform = crate::geom::Transform::from(header.transform);

    // Deserialize based on data type
    match header.data_type {
        SensorDataType::Image => deserialize_image_data(payload, timestamp, transform),
        SensorDataType::LiDAR => deserialize_lidar_data(payload, timestamp, transform),
        SensorDataType::Radar => deserialize_radar_data(payload, timestamp, transform),
        SensorDataType::IMU => deserialize_imu_data(payload, timestamp, transform),
        SensorDataType::GNSS => deserialize_gnss_data(payload, timestamp, transform),
        SensorDataType::Collision => deserialize_collision_data(payload, timestamp, transform),
        SensorDataType::LaneInvasion => {
            deserialize_lane_invasion_data(payload, timestamp, transform)
        }
        _ => Err(SensorError::DataConversion(format!(
            "Unknown sensor data type: {:?}",
            header.data_type
        ))
        .into()),
    }
}

/// Deserialize image data from payload.
fn deserialize_image_data(
    payload: &[u8],
    timestamp: crate::time::Timestamp,
    transform: crate::geom::Transform,
) -> CarlaResult<CallbackSensorData> {
    // Image format: raw BGRA data (width*height*4 bytes)
    // We need to infer dimensions or use default values since C++ doesn't serialize them

    if payload.is_empty() {
        return Err(SensorError::DataConversion("Image payload is empty".to_string()).into());
    }

    // Assume common image sizes - in practice this would come from sensor configuration
    let (width, height) = if payload.len() == 800 * 600 * 4 {
        (800, 600)
    } else if payload.len() == 1920 * 1080 * 4 {
        (1920, 1080)
    } else if payload.len() == 640 * 480 * 4 {
        (640, 480)
    } else {
        // Try to guess square image
        let pixel_count = payload.len() / 4;
        let side = (pixel_count as f64).sqrt() as u32;
        if (side * side * 4) as usize == payload.len() {
            (side, side)
        } else {
            // Default fallback
            (800, 600)
        }
    };

    let image = ImageData {
        timestamp,
        transform,
        sensor_id: 0, // Will be set by callback context
        width,
        height,
        fov: 90.0, // Default FOV
        data: payload.to_vec(),
    };

    // For now, return as RGB - in practice we'd need to detect the specific type
    Ok(CallbackSensorData::RGB(RGBImageData::new(image)))
}

/// Deserialize LiDAR data from payload.
fn deserialize_lidar_data(
    payload: &[u8],
    timestamp: crate::time::Timestamp,
    transform: crate::geom::Transform,
) -> CarlaResult<CallbackSensorData> {
    // LiDAR format: point_count(4) + points(count * 16 bytes per point)
    if payload.len() < 4 {
        return Err(SensorError::DataConversion("LiDAR payload too small".to_string()).into());
    }

    let point_count = u32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]]);
    let expected_size = 4 + (point_count * 16) as usize; // 4 floats per point

    if payload.len() < expected_size {
        return Err(SensorError::DataConversion(format!(
            "LiDAR data incomplete: expected {} bytes, got {}",
            expected_size,
            payload.len()
        ))
        .into());
    }

    let mut points = Vec::with_capacity(point_count as usize);
    let mut offset = 4;

    for _ in 0..point_count {
        let x = f32::from_le_bytes([
            payload[offset],
            payload[offset + 1],
            payload[offset + 2],
            payload[offset + 3],
        ]);
        let y = f32::from_le_bytes([
            payload[offset + 4],
            payload[offset + 5],
            payload[offset + 6],
            payload[offset + 7],
        ]);
        let z = f32::from_le_bytes([
            payload[offset + 8],
            payload[offset + 9],
            payload[offset + 10],
            payload[offset + 11],
        ]);
        let intensity = f32::from_le_bytes([
            payload[offset + 12],
            payload[offset + 13],
            payload[offset + 14],
            payload[offset + 15],
        ]);

        points.push(crate::sensor_data::LiDARPoint::new(x, y, z, intensity));
        offset += 16;
    }

    let lidar_data = LiDARData {
        timestamp,
        transform,
        sensor_id: 0,
        channels: 32,              // Default value
        horizontal_fov: 360.0,     // Default value
        points_per_second: 100000, // Default value
        points,
    };

    Ok(CallbackSensorData::LiDAR(lidar_data))
}

/// Deserialize radar data from payload.
fn deserialize_radar_data(
    payload: &[u8],
    timestamp: crate::time::Timestamp,
    transform: crate::geom::Transform,
) -> CarlaResult<CallbackSensorData> {
    // Radar format: detection_count(4) + detections(count * detection_size)
    if payload.len() < 4 {
        return Err(SensorError::DataConversion("Radar payload too small".to_string()).into());
    }

    let _detection_count = u32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]]);

    // For now, return basic radar data structure
    let radar_data = RadarData {
        timestamp,
        transform,
        sensor_id: 0,
        detections: Vec::new(), // TODO: Implement proper radar detection parsing from binary payload
                                // This requires understanding CARLA's binary serialization format for radar data
                                // and parsing the payload bytes into Vec<RadarDetection> structures.
                                // The SimpleRadarData structure shows the expected format with velocity, azimuth, altitude, depth.
    };

    Ok(CallbackSensorData::Radar(radar_data))
}

/// Deserialize IMU data from payload.
fn deserialize_imu_data(
    payload: &[u8],
    timestamp: crate::time::Timestamp,
    transform: crate::geom::Transform,
) -> CarlaResult<CallbackSensorData> {
    // IMU format: 7 floats (3 accel + 3 gyro + 1 compass)
    if payload.len() < 28 {
        return Err(SensorError::DataConversion("IMU payload too small".to_string()).into());
    }

    let accel_x = f32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]]);
    let accel_y = f32::from_le_bytes([payload[4], payload[5], payload[6], payload[7]]);
    let accel_z = f32::from_le_bytes([payload[8], payload[9], payload[10], payload[11]]);

    let gyro_x = f32::from_le_bytes([payload[12], payload[13], payload[14], payload[15]]);
    let gyro_y = f32::from_le_bytes([payload[16], payload[17], payload[18], payload[19]]);
    let gyro_z = f32::from_le_bytes([payload[20], payload[21], payload[22], payload[23]]);

    let compass = f32::from_le_bytes([payload[24], payload[25], payload[26], payload[27]]);

    let imu_data = IMUData {
        timestamp,
        transform,
        sensor_id: 0,
        accelerometer: [accel_x, accel_y, accel_z],
        gyroscope: [gyro_x, gyro_y, gyro_z],
        compass,
    };

    Ok(CallbackSensorData::IMU(imu_data))
}

/// Deserialize GNSS data from payload.
fn deserialize_gnss_data(
    payload: &[u8],
    timestamp: crate::time::Timestamp,
    transform: crate::geom::Transform,
) -> CarlaResult<CallbackSensorData> {
    // GNSS format: 3 doubles (lat, lon, alt)
    if payload.len() < 24 {
        return Err(SensorError::DataConversion("GNSS payload too small".to_string()).into());
    }

    let latitude = f64::from_le_bytes([
        payload[0], payload[1], payload[2], payload[3], payload[4], payload[5], payload[6],
        payload[7],
    ]);
    let longitude = f64::from_le_bytes([
        payload[8],
        payload[9],
        payload[10],
        payload[11],
        payload[12],
        payload[13],
        payload[14],
        payload[15],
    ]);
    let altitude = f64::from_le_bytes([
        payload[16],
        payload[17],
        payload[18],
        payload[19],
        payload[20],
        payload[21],
        payload[22],
        payload[23],
    ]);

    let gnss_data = GNSSData {
        timestamp,
        transform,
        sensor_id: 0,
        latitude,
        longitude,
        altitude,
    };

    Ok(CallbackSensorData::GNSS(gnss_data))
}

/// Deserialize collision data from payload.
fn deserialize_collision_data(
    payload: &[u8],
    timestamp: crate::time::Timestamp,
    transform: crate::geom::Transform,
) -> CarlaResult<CallbackSensorData> {
    // Collision format: normal_impulse(12) + other_actor_id(4)
    if payload.len() < 16 {
        return Err(SensorError::DataConversion("Collision payload too small".to_string()).into());
    }

    let normal_x = f32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]]);
    let normal_y = f32::from_le_bytes([payload[4], payload[5], payload[6], payload[7]]);
    let normal_z = f32::from_le_bytes([payload[8], payload[9], payload[10], payload[11]]);

    let other_actor_id = u32::from_le_bytes([payload[12], payload[13], payload[14], payload[15]]);

    let collision_data = CollisionData {
        timestamp,
        transform,
        sensor_id: 0,
        other_actor_id,
        normal_impulse: crate::geom::Vector3D::new(normal_x, normal_y, normal_z),
    };

    Ok(CallbackSensorData::Collision(collision_data))
}

/// Deserialize lane invasion data from payload.
fn deserialize_lane_invasion_data(
    payload: &[u8],
    timestamp: crate::time::Timestamp,
    transform: crate::geom::Transform,
) -> CarlaResult<CallbackSensorData> {
    // Lane invasion format: marking_count(4) + markings(count * 4)
    if payload.len() < 4 {
        return Err(
            SensorError::DataConversion("Lane invasion payload too small".to_string()).into(),
        );
    }

    let marking_count = u32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]]);
    let expected_size = 4 + (marking_count * 4) as usize;

    if payload.len() < expected_size {
        return Err(
            SensorError::DataConversion("Lane invasion data incomplete".to_string()).into(),
        );
    }

    // For now, create basic lane marking info with default values
    let mut crossed_lane_markings = Vec::new();
    let mut offset = 4;

    for _ in 0..marking_count {
        let marking_type = i32::from_le_bytes([
            payload[offset],
            payload[offset + 1],
            payload[offset + 2],
            payload[offset + 3],
        ]);
        // Create LaneMarkingInfo from parsed data
        // Note: This implementation assumes the binary format only contains marking type
        // Full implementation would require color and lane_change data from payload
        let lane_marking_info = LaneMarkingInfo {
            marking_type: LaneMarkingType::from_u32(marking_type as u32),
            color: LaneMarkingColor::Standard, // Default value - would need to parse from payload
            lane_change: LaneChange::None,     // Default value - would need to parse from payload
        };
        crossed_lane_markings.push(lane_marking_info);
        offset += 4;
    }

    let lane_invasion_data = LaneInvasionData {
        timestamp,
        transform,
        sensor_id: 0,
        crossed_lane_markings,
    };

    Ok(CallbackSensorData::LaneInvasion(lane_invasion_data))
}

impl<'a> Drop for SensorCallbackManager<'a> {
    fn drop(&mut self) {
        self.clear_callbacks();
    }
}
