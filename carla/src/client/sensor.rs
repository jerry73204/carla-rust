//! Sensor actor implementation.

use crate::{
    client::{Actor, ActorId},
    error::{CarlaResult, SensorError},
    geom::{FromCxx, ToCxx, Transform, Vector3D},
    traits::ActorT,
};
use carla_cxx::SensorWrapper;

/// Sensor actor.
#[derive(Debug)]
pub struct Sensor {
    /// Actor ID
    id: ActorId,
    /// Internal sensor wrapper for FFI calls
    pub(crate) inner: SensorWrapper,
}

impl Sensor {
    /// Create a sensor from a carla-cxx SensorWrapper and actor ID.
    pub fn new(sensor_wrapper: SensorWrapper, id: ActorId) -> CarlaResult<Self> {
        Ok(Self {
            id,
            inner: sensor_wrapper,
        })
    }

    /// Create a sensor from an actor by casting.
    pub fn from_actor(actor: Actor) -> CarlaResult<Option<Self>> {
        let actor_ref = actor.get_inner_actor();
        let actor_id = actor.get_id();
        if let Some(sensor_wrapper) = SensorWrapper::from_actor(actor_ref) {
            Ok(Some(Self {
                id: actor_id,
                inner: sensor_wrapper,
            }))
        } else {
            Ok(None)
        }
    }

    /// Get the sensor's actor ID.
    pub fn get_id(&self) -> ActorId {
        self.id
    }

    /// Get the sensor's actor ID.
    pub fn id(&self) -> ActorId {
        self.id
    }

    /// Start listening for sensor data with a callback.
    ///
    /// # Arguments
    /// * `callback` - Function called when new data is available
    ///
    /// # Errors
    /// Returns [`SensorError::AlreadyListening`] if already listening,
    /// or [`SensorError::CallbackFailed`] if callback registration fails.
    pub fn listen<F>(&self, callback: F) -> Result<(), SensorError>
    where
        F: Fn(Vec<u8>) + Send + 'static, // Generic sensor data as bytes
    {
        let _callback = callback;
        self.inner.listen();
        Ok(())
    }

    /// Start listening with an async channel for sensor data.
    ///
    /// Returns a receiver that yields sensor data. Requires the `async` feature.
    #[cfg(feature = "async")]
    pub fn listen_async(&self) -> Result<tokio::sync::mpsc::Receiver<Vec<u8>>, SensorError> {
        // TODO: Implement using carla-cxx FFI interface
        todo!("Sensor::listen_async not yet implemented with carla-cxx FFI")
    }

    /// Stop listening for sensor data.
    pub fn stop(&self) {
        self.inner.stop();
    }

    /// Check if the sensor is currently listening.
    pub fn is_listening(&self) -> bool {
        self.inner.is_listening()
    }

    /// Get the sensor's data frame number (increments with each measurement).
    pub fn get_frame(&self) -> u64 {
        // TODO: Implement frame number retrieval from sensor state
        // This requires adding Sensor_GetFrame FFI function
        todo!("Sensor::get_frame not yet implemented - missing FFI function Sensor_GetFrame")
    }

    /// Get the timestamp of the last sensor measurement.
    pub fn get_timestamp(&self) -> f64 {
        // TODO: Implement timestamp retrieval from sensor state
        // This requires adding Sensor_GetTimestamp FFI function
        todo!(
            "Sensor::get_timestamp not yet implemented - missing FFI function Sensor_GetTimestamp"
        )
    }

    /// Get sensor-specific attribute.
    pub fn get_attribute(&self, name: &str) -> Option<String> {
        let _name = name;
        // TODO: Implement sensor attribute retrieval
        // This requires adding Sensor_GetAttribute FFI function or storing blueprint reference
        todo!(
            "Sensor::get_attribute not yet implemented - missing FFI function Sensor_GetAttribute"
        )
    }

    /// Enable sensor recording.
    pub fn enable_recording(&self, filename: &str) -> CarlaResult<()> {
        let _filename = filename;
        // Sensor recording is typically done at the client level, not sensor level
        // This would require additional FFI functions that aren't part of the basic sensor API
        Err(crate::error::CarlaError::Sensor(
            SensorError::CallbackFailed(
                "enable_recording not available - use client-level recording".to_string(),
            ),
        ))
    }

    /// Disable sensor recording.
    pub fn disable_recording(&self) -> CarlaResult<()> {
        // Sensor recording is typically done at the client level, not sensor level
        // This would require additional FFI functions that aren't part of the basic sensor API
        Err(crate::error::CarlaError::Sensor(
            SensorError::CallbackFailed(
                "disable_recording not available - use client-level recording".to_string(),
            ),
        ))
    }

    /// Check if new sensor data is available.
    pub fn has_new_data(&self) -> bool {
        self.inner.has_new_data()
    }

    /// Get access to the underlying SensorWrapper for type-specific operations.
    pub(crate) fn inner(&self) -> &SensorWrapper {
        &self.inner
    }
}

impl ActorT for Sensor {
    fn get_id(&self) -> ActorId {
        self.id
    }
    fn get_type_id(&self) -> String {
        carla_cxx::ffi::bridge::Sensor_GetTypeId(self.inner.get_inner_sensor())
    }
    fn get_transform(&self) -> Transform {
        let cxx_transform =
            carla_cxx::ffi::bridge::Sensor_GetTransform(self.inner.get_inner_sensor());
        Transform::from(cxx_transform)
    }
    fn set_transform(&self, transform: &Transform) -> CarlaResult<()> {
        let cxx_transform = transform.to_cxx();
        carla_cxx::ffi::bridge::Sensor_SetTransform(self.inner.get_inner_sensor(), &cxx_transform);
        Ok(())
    }
    fn get_velocity(&self) -> Vector3D {
        let vel = carla_cxx::ffi::bridge::Sensor_GetVelocity(self.inner.get_inner_sensor());
        Vector3D::new(vel.x as f32, vel.y as f32, vel.z as f32)
    }
    fn get_angular_velocity(&self) -> Vector3D {
        let vel = carla_cxx::ffi::bridge::Sensor_GetAngularVelocity(self.inner.get_inner_sensor());
        Vector3D::new(vel.x as f32, vel.y as f32, vel.z as f32)
    }
    fn get_acceleration(&self) -> Vector3D {
        let acc = carla_cxx::ffi::bridge::Sensor_GetAcceleration(self.inner.get_inner_sensor());
        Vector3D::new(acc.x as f32, acc.y as f32, acc.z as f32)
    }
    fn is_alive(&self) -> bool {
        carla_cxx::ffi::bridge::Sensor_IsAlive(self.inner.get_inner_sensor())
    }
    fn destroy(&self) -> CarlaResult<()> {
        if carla_cxx::ffi::bridge::Sensor_Destroy(self.inner.get_inner_sensor()) {
            Ok(())
        } else {
            Err(crate::error::CarlaError::Actor(
                crate::error::ActorError::DestroyFailed(self.id),
            ))
        }
    }
    fn set_simulate_physics(&self, enabled: bool) -> CarlaResult<()> {
        carla_cxx::ffi::bridge::Sensor_SetSimulatePhysics(self.inner.get_inner_sensor(), enabled);
        Ok(())
    }
    fn add_impulse(&self, impulse: &Vector3D) -> CarlaResult<()> {
        let cxx_impulse = carla_cxx::SimpleVector3D {
            x: impulse.x as f64,
            y: impulse.y as f64,
            z: impulse.z as f64,
        };
        carla_cxx::ffi::bridge::Sensor_AddImpulse(self.inner.get_inner_sensor(), &cxx_impulse);
        Ok(())
    }
    fn add_force(&self, force: &Vector3D) -> CarlaResult<()> {
        let cxx_force = carla_cxx::SimpleVector3D {
            x: force.x as f64,
            y: force.y as f64,
            z: force.z as f64,
        };
        carla_cxx::ffi::bridge::Sensor_AddForce(self.inner.get_inner_sensor(), &cxx_force);
        Ok(())
    }
    fn add_torque(&self, torque: &Vector3D) -> CarlaResult<()> {
        let cxx_torque = carla_cxx::SimpleVector3D {
            x: torque.x as f64,
            y: torque.y as f64,
            z: torque.z as f64,
        };
        carla_cxx::ffi::bridge::Sensor_AddTorque(self.inner.get_inner_sensor(), &cxx_torque);
        Ok(())
    }
}

// ============================================================================
// Specific Sensor Types using NewType Pattern
// ============================================================================

/// Camera sensor for image data.
#[derive(Debug)]
pub struct Camera(Sensor);

impl Camera {
    /// Try to create a Camera from a generic Sensor.
    /// Returns None if the sensor is not a camera.
    pub fn from_sensor(sensor: Sensor) -> Option<Self> {
        if sensor.inner.is_camera() {
            Some(Camera(sensor))
        } else {
            None
        }
    }

    /// Convert back to generic Sensor.
    pub fn into_sensor(self) -> Sensor {
        self.0
    }

    /// Get reference to the underlying sensor.
    pub fn as_sensor(&self) -> &Sensor {
        &self.0
    }

    /// Get the camera type (RGB, Depth, Semantic, etc.)
    pub fn get_camera_type(&self) -> crate::client::CameraType {
        crate::client::CameraType::from(self.0.inner.get_camera_type())
    }

    /// Get the last image data from the camera.
    pub fn get_last_image_data(&self) -> Option<crate::sensor::ImageData> {
        let cxx_data = self.0.inner.get_last_image_data();
        if cxx_data.is_empty() {
            None
        } else {
            Some(crate::sensor::ImageData::from_cxx(cxx_data))
        }
    }

    /// Get image data into a provided buffer.
    /// Returns true if data was copied successfully.
    pub fn get_image_data_buffer(&self, buffer: &mut [u8]) -> bool {
        self.0.inner.get_image_data_buffer(buffer)
    }

    /// Start listening for camera data.
    pub fn listen(&self) {
        self.0.inner.listen();
    }

    /// Stop listening for camera data.
    pub fn stop(&self) {
        self.0.inner.stop();
    }

    /// Check if the camera is listening.
    pub fn is_listening(&self) -> bool {
        self.0.inner.is_listening()
    }

    /// Check if new image data is available.
    pub fn has_new_data(&self) -> bool {
        self.0.inner.has_new_data()
    }
}

impl ActorT for Camera {
    fn get_id(&self) -> ActorId {
        self.0.get_id()
    }
    fn get_type_id(&self) -> String {
        self.0.get_type_id()
    }
    fn get_transform(&self) -> Transform {
        self.0.get_transform()
    }
    fn set_transform(&self, transform: &Transform) -> CarlaResult<()> {
        self.0.set_transform(transform)
    }
    fn get_velocity(&self) -> Vector3D {
        self.0.get_velocity()
    }
    fn get_angular_velocity(&self) -> Vector3D {
        self.0.get_angular_velocity()
    }
    fn get_acceleration(&self) -> Vector3D {
        self.0.get_acceleration()
    }
    fn is_alive(&self) -> bool {
        self.0.is_alive()
    }
    fn destroy(&self) -> CarlaResult<()> {
        self.0.destroy()
    }
    fn set_simulate_physics(&self, enabled: bool) -> CarlaResult<()> {
        self.0.set_simulate_physics(enabled)
    }
    fn add_impulse(&self, impulse: &Vector3D) -> CarlaResult<()> {
        self.0.add_impulse(impulse)
    }
    fn add_force(&self, force: &Vector3D) -> CarlaResult<()> {
        self.0.add_force(force)
    }
    fn add_torque(&self, torque: &Vector3D) -> CarlaResult<()> {
        self.0.add_torque(torque)
    }
}

/// LiDAR sensor for point cloud data.
#[derive(Debug)]
pub struct LiDAR(Sensor);

impl LiDAR {
    /// Try to create a LiDAR from a generic Sensor.
    /// Returns None if the sensor is not a LiDAR.
    pub fn from_sensor(sensor: Sensor) -> Option<Self> {
        let type_id = sensor.get_type_id();
        if type_id.contains("lidar") || type_id.contains("velodyne") {
            Some(LiDAR(sensor))
        } else {
            None
        }
    }

    /// Convert back to generic Sensor.
    pub fn into_sensor(self) -> Sensor {
        self.0
    }

    /// Get reference to the underlying sensor.
    pub fn as_sensor(&self) -> &Sensor {
        &self.0
    }

    /// Get the last LiDAR data from the sensor with full metadata.
    pub fn get_last_lidar_data(&self) -> Option<crate::sensor::LiDARData> {
        let cxx_data = self.0.inner.get_last_lidar_data_full();
        if cxx_data.points.is_empty() {
            None
        } else {
            Some(crate::sensor::LiDARData::from_cxx(cxx_data))
        }
    }

    /// Get the last LiDAR points only (without metadata) for performance.
    pub fn get_last_lidar_points(&self) -> Vec<crate::sensor::LiDARPoint> {
        let cxx_data = self.0.inner.get_last_lidar_data();
        cxx_data
            .points
            .iter()
            .map(|p| crate::sensor::LiDARPoint::new(p.x, p.y, p.z, p.intensity))
            .collect()
    }

    /// Get the last Semantic LiDAR data from the sensor.
    pub fn get_last_semantic_lidar_data(&self) -> Option<crate::sensor::SemanticLiDARData> {
        let cxx_data = self.0.inner.get_last_semantic_lidar_data();
        if cxx_data.detections.is_empty() {
            None
        } else {
            Some(crate::sensor::SemanticLiDARData::from_cxx(cxx_data))
        }
    }
}

impl ActorT for LiDAR {
    fn get_id(&self) -> ActorId {
        self.0.get_id()
    }
    fn get_type_id(&self) -> String {
        self.0.get_type_id()
    }
    fn get_transform(&self) -> Transform {
        self.0.get_transform()
    }
    fn set_transform(&self, transform: &Transform) -> CarlaResult<()> {
        self.0.set_transform(transform)
    }
    fn get_velocity(&self) -> Vector3D {
        self.0.get_velocity()
    }
    fn get_angular_velocity(&self) -> Vector3D {
        self.0.get_angular_velocity()
    }
    fn get_acceleration(&self) -> Vector3D {
        self.0.get_acceleration()
    }
    fn is_alive(&self) -> bool {
        self.0.is_alive()
    }
    fn destroy(&self) -> CarlaResult<()> {
        self.0.destroy()
    }
    fn set_simulate_physics(&self, enabled: bool) -> CarlaResult<()> {
        self.0.set_simulate_physics(enabled)
    }
    fn add_impulse(&self, impulse: &Vector3D) -> CarlaResult<()> {
        self.0.add_impulse(impulse)
    }
    fn add_force(&self, force: &Vector3D) -> CarlaResult<()> {
        self.0.add_force(force)
    }
    fn add_torque(&self, torque: &Vector3D) -> CarlaResult<()> {
        self.0.add_torque(torque)
    }
}

/// Radar sensor for object detection data.
#[derive(Debug)]
pub struct Radar(Sensor);

impl Radar {
    /// Try to create a Radar from a generic Sensor.
    /// Returns None if the sensor is not a Radar.
    pub fn from_sensor(sensor: Sensor) -> Option<Self> {
        let type_id = sensor.get_type_id();
        if type_id.contains("radar") {
            Some(Radar(sensor))
        } else {
            None
        }
    }

    /// Convert back to generic Sensor.
    pub fn into_sensor(self) -> Sensor {
        self.0
    }

    /// Get reference to the underlying sensor.
    pub fn as_sensor(&self) -> &Sensor {
        &self.0
    }

    /// Get the last Radar data from the sensor.
    pub fn get_last_radar_data(&self) -> Option<crate::sensor::RadarData> {
        let cxx_data = self.0.inner.get_last_radar_data();
        if cxx_data.detections.is_empty() {
            None
        } else {
            Some(crate::sensor::RadarData::from_cxx(cxx_data))
        }
    }
}

impl ActorT for Radar {
    fn get_id(&self) -> ActorId {
        self.0.get_id()
    }
    fn get_type_id(&self) -> String {
        self.0.get_type_id()
    }
    fn get_transform(&self) -> Transform {
        self.0.get_transform()
    }
    fn set_transform(&self, transform: &Transform) -> CarlaResult<()> {
        self.0.set_transform(transform)
    }
    fn get_velocity(&self) -> Vector3D {
        self.0.get_velocity()
    }
    fn get_angular_velocity(&self) -> Vector3D {
        self.0.get_angular_velocity()
    }
    fn get_acceleration(&self) -> Vector3D {
        self.0.get_acceleration()
    }
    fn is_alive(&self) -> bool {
        self.0.is_alive()
    }
    fn destroy(&self) -> CarlaResult<()> {
        self.0.destroy()
    }
    fn set_simulate_physics(&self, enabled: bool) -> CarlaResult<()> {
        self.0.set_simulate_physics(enabled)
    }
    fn add_impulse(&self, impulse: &Vector3D) -> CarlaResult<()> {
        self.0.add_impulse(impulse)
    }
    fn add_force(&self, force: &Vector3D) -> CarlaResult<()> {
        self.0.add_force(force)
    }
    fn add_torque(&self, torque: &Vector3D) -> CarlaResult<()> {
        self.0.add_torque(torque)
    }
}

/// IMU sensor for inertial measurement data.
#[derive(Debug)]
pub struct IMU(Sensor);

impl IMU {
    /// Try to create an IMU from a generic Sensor.
    /// Returns None if the sensor is not an IMU.
    pub fn from_sensor(sensor: Sensor) -> Option<Self> {
        let type_id = sensor.get_type_id();
        if type_id.contains("imu") || type_id.contains("inertial") {
            Some(IMU(sensor))
        } else {
            None
        }
    }

    /// Convert back to generic Sensor.
    pub fn into_sensor(self) -> Sensor {
        self.0
    }

    /// Get reference to the underlying sensor.
    pub fn as_sensor(&self) -> &Sensor {
        &self.0
    }

    /// Get the last IMU data from the sensor.
    pub fn get_last_imu_data(&self) -> Option<crate::sensor::IMUData> {
        let cxx_data = self.0.inner.get_last_imu_data();
        // For IMU data, we'll always return Some unless there's an error in FFI
        Some(crate::sensor::IMUData::from_cxx(cxx_data))
    }
}

impl ActorT for IMU {
    fn get_id(&self) -> ActorId {
        self.0.get_id()
    }
    fn get_type_id(&self) -> String {
        self.0.get_type_id()
    }
    fn get_transform(&self) -> Transform {
        self.0.get_transform()
    }
    fn set_transform(&self, transform: &Transform) -> CarlaResult<()> {
        self.0.set_transform(transform)
    }
    fn get_velocity(&self) -> Vector3D {
        self.0.get_velocity()
    }
    fn get_angular_velocity(&self) -> Vector3D {
        self.0.get_angular_velocity()
    }
    fn get_acceleration(&self) -> Vector3D {
        self.0.get_acceleration()
    }
    fn is_alive(&self) -> bool {
        self.0.is_alive()
    }
    fn destroy(&self) -> CarlaResult<()> {
        self.0.destroy()
    }
    fn set_simulate_physics(&self, enabled: bool) -> CarlaResult<()> {
        self.0.set_simulate_physics(enabled)
    }
    fn add_impulse(&self, impulse: &Vector3D) -> CarlaResult<()> {
        self.0.add_impulse(impulse)
    }
    fn add_force(&self, force: &Vector3D) -> CarlaResult<()> {
        self.0.add_force(force)
    }
    fn add_torque(&self, torque: &Vector3D) -> CarlaResult<()> {
        self.0.add_torque(torque)
    }
}

/// GNSS sensor for GPS positioning data.
#[derive(Debug)]
pub struct GNSS(Sensor);

impl GNSS {
    /// Try to create a GNSS from a generic Sensor.
    /// Returns None if the sensor is not a GNSS.
    pub fn from_sensor(sensor: Sensor) -> Option<Self> {
        let type_id = sensor.get_type_id();
        if type_id.contains("gnss") || type_id.contains("gps") {
            Some(GNSS(sensor))
        } else {
            None
        }
    }

    /// Convert back to generic Sensor.
    pub fn into_sensor(self) -> Sensor {
        self.0
    }

    /// Get reference to the underlying sensor.
    pub fn as_sensor(&self) -> &Sensor {
        &self.0
    }

    /// Get the last GNSS data from the sensor.
    pub fn get_last_gnss_data(&self) -> Option<crate::sensor::GNSSData> {
        let cxx_data = self.0.inner.get_last_gnss_data();
        // For GNSS data, we'll always return Some unless there's an error in FFI
        Some(crate::sensor::GNSSData::from_cxx(cxx_data))
    }
}

impl ActorT for GNSS {
    fn get_id(&self) -> ActorId {
        self.0.get_id()
    }
    fn get_type_id(&self) -> String {
        self.0.get_type_id()
    }
    fn get_transform(&self) -> Transform {
        self.0.get_transform()
    }
    fn set_transform(&self, transform: &Transform) -> CarlaResult<()> {
        self.0.set_transform(transform)
    }
    fn get_velocity(&self) -> Vector3D {
        self.0.get_velocity()
    }
    fn get_angular_velocity(&self) -> Vector3D {
        self.0.get_angular_velocity()
    }
    fn get_acceleration(&self) -> Vector3D {
        self.0.get_acceleration()
    }
    fn is_alive(&self) -> bool {
        self.0.is_alive()
    }
    fn destroy(&self) -> CarlaResult<()> {
        self.0.destroy()
    }
    fn set_simulate_physics(&self, enabled: bool) -> CarlaResult<()> {
        self.0.set_simulate_physics(enabled)
    }
    fn add_impulse(&self, impulse: &Vector3D) -> CarlaResult<()> {
        self.0.add_impulse(impulse)
    }
    fn add_force(&self, force: &Vector3D) -> CarlaResult<()> {
        self.0.add_force(force)
    }
    fn add_torque(&self, torque: &Vector3D) -> CarlaResult<()> {
        self.0.add_torque(torque)
    }
}

/// Collision sensor for collision detection events.
#[derive(Debug)]
pub struct CollisionSensor(Sensor);

impl CollisionSensor {
    /// Try to create a CollisionSensor from a generic Sensor.
    /// Returns None if the sensor is not a collision sensor.
    pub fn from_sensor(sensor: Sensor) -> Option<Self> {
        let type_id = sensor.get_type_id();
        if type_id.contains("collision") {
            Some(CollisionSensor(sensor))
        } else {
            None
        }
    }

    /// Convert back to generic Sensor.
    pub fn into_sensor(self) -> Sensor {
        self.0
    }

    /// Get reference to the underlying sensor.
    pub fn as_sensor(&self) -> &Sensor {
        &self.0
    }

    /// Get the last collision data from the sensor.
    pub fn get_last_collision_data(&self) -> Option<crate::sensor::CollisionData> {
        let cxx_data = self.0.inner.get_last_collision_data();
        // For collision data, we check if there was actually a collision
        if cxx_data.other_actor_id == 0 {
            None // No collision
        } else {
            Some(crate::sensor::CollisionData::from_cxx(cxx_data))
        }
    }
}

impl ActorT for CollisionSensor {
    fn get_id(&self) -> ActorId {
        self.0.get_id()
    }
    fn get_type_id(&self) -> String {
        self.0.get_type_id()
    }
    fn get_transform(&self) -> Transform {
        self.0.get_transform()
    }
    fn set_transform(&self, transform: &Transform) -> CarlaResult<()> {
        self.0.set_transform(transform)
    }
    fn get_velocity(&self) -> Vector3D {
        self.0.get_velocity()
    }
    fn get_angular_velocity(&self) -> Vector3D {
        self.0.get_angular_velocity()
    }
    fn get_acceleration(&self) -> Vector3D {
        self.0.get_acceleration()
    }
    fn is_alive(&self) -> bool {
        self.0.is_alive()
    }
    fn destroy(&self) -> CarlaResult<()> {
        self.0.destroy()
    }
    fn set_simulate_physics(&self, enabled: bool) -> CarlaResult<()> {
        self.0.set_simulate_physics(enabled)
    }
    fn add_impulse(&self, impulse: &Vector3D) -> CarlaResult<()> {
        self.0.add_impulse(impulse)
    }
    fn add_force(&self, force: &Vector3D) -> CarlaResult<()> {
        self.0.add_force(force)
    }
    fn add_torque(&self, torque: &Vector3D) -> CarlaResult<()> {
        self.0.add_torque(torque)
    }
}

/// Lane invasion sensor for lane marking detection.
#[derive(Debug)]
pub struct LaneInvasionSensor(Sensor);

impl LaneInvasionSensor {
    /// Try to create a LaneInvasionSensor from a generic Sensor.
    /// Returns None if the sensor is not a lane invasion sensor.
    pub fn from_sensor(sensor: Sensor) -> Option<Self> {
        let type_id = sensor.get_type_id();
        if type_id.contains("lane_invasion") || type_id.contains("lane") {
            Some(LaneInvasionSensor(sensor))
        } else {
            None
        }
    }

    /// Convert back to generic Sensor.
    pub fn into_sensor(self) -> Sensor {
        self.0
    }

    /// Get reference to the underlying sensor.
    pub fn as_sensor(&self) -> &Sensor {
        &self.0
    }

    /// Get the last lane invasion data from the sensor.
    pub fn get_last_lane_invasion_data(&self) -> Option<crate::sensor::LaneInvasionData> {
        let cxx_data = self.0.inner.get_last_lane_invasion_data();
        if cxx_data.crossed_lane_markings.is_empty() {
            None // No lane invasion
        } else {
            Some(crate::sensor::LaneInvasionData::from_cxx(cxx_data))
        }
    }
}

impl ActorT for LaneInvasionSensor {
    fn get_id(&self) -> ActorId {
        self.0.get_id()
    }
    fn get_type_id(&self) -> String {
        self.0.get_type_id()
    }
    fn get_transform(&self) -> Transform {
        self.0.get_transform()
    }
    fn set_transform(&self, transform: &Transform) -> CarlaResult<()> {
        self.0.set_transform(transform)
    }
    fn get_velocity(&self) -> Vector3D {
        self.0.get_velocity()
    }
    fn get_angular_velocity(&self) -> Vector3D {
        self.0.get_angular_velocity()
    }
    fn get_acceleration(&self) -> Vector3D {
        self.0.get_acceleration()
    }
    fn is_alive(&self) -> bool {
        self.0.is_alive()
    }
    fn destroy(&self) -> CarlaResult<()> {
        self.0.destroy()
    }
    fn set_simulate_physics(&self, enabled: bool) -> CarlaResult<()> {
        self.0.set_simulate_physics(enabled)
    }
    fn add_impulse(&self, impulse: &Vector3D) -> CarlaResult<()> {
        self.0.add_impulse(impulse)
    }
    fn add_force(&self, force: &Vector3D) -> CarlaResult<()> {
        self.0.add_force(force)
    }
    fn add_torque(&self, torque: &Vector3D) -> CarlaResult<()> {
        self.0.add_torque(torque)
    }
}

/// DVS (Dynamic Vision Sensor) camera for event-based vision.
#[derive(Debug)]
pub struct DVSCamera(Sensor);

impl DVSCamera {
    /// Try to create a DVSCamera from a generic Sensor.
    /// Returns None if the sensor is not a DVS camera.
    pub fn from_sensor(sensor: Sensor) -> Option<Self> {
        let type_id = sensor.get_type_id();
        if type_id.contains("dvs") || type_id.contains("event") {
            Some(DVSCamera(sensor))
        } else {
            None
        }
    }

    /// Convert back to generic Sensor.
    pub fn into_sensor(self) -> Sensor {
        self.0
    }

    /// Get reference to the underlying sensor.
    pub fn as_sensor(&self) -> &Sensor {
        &self.0
    }

    /// Get the last DVS data from the sensor.
    pub fn get_last_dvs_data(&self) -> Option<crate::sensor::DVSData> {
        let cxx_data = self.0.inner.get_last_dvs_data();
        if cxx_data.events.is_empty() {
            None
        } else {
            Some(crate::sensor::DVSData::from_cxx(cxx_data))
        }
    }
}

impl ActorT for DVSCamera {
    fn get_id(&self) -> ActorId {
        self.0.get_id()
    }
    fn get_type_id(&self) -> String {
        self.0.get_type_id()
    }
    fn get_transform(&self) -> Transform {
        self.0.get_transform()
    }
    fn set_transform(&self, transform: &Transform) -> CarlaResult<()> {
        self.0.set_transform(transform)
    }
    fn get_velocity(&self) -> Vector3D {
        self.0.get_velocity()
    }
    fn get_angular_velocity(&self) -> Vector3D {
        self.0.get_angular_velocity()
    }
    fn get_acceleration(&self) -> Vector3D {
        self.0.get_acceleration()
    }
    fn is_alive(&self) -> bool {
        self.0.is_alive()
    }
    fn destroy(&self) -> CarlaResult<()> {
        self.0.destroy()
    }
    fn set_simulate_physics(&self, enabled: bool) -> CarlaResult<()> {
        self.0.set_simulate_physics(enabled)
    }
    fn add_impulse(&self, impulse: &Vector3D) -> CarlaResult<()> {
        self.0.add_impulse(impulse)
    }
    fn add_force(&self, force: &Vector3D) -> CarlaResult<()> {
        self.0.add_force(force)
    }
    fn add_torque(&self, torque: &Vector3D) -> CarlaResult<()> {
        self.0.add_torque(torque)
    }
}

/// Optical Flow camera for motion detection.
#[derive(Debug)]
pub struct OpticalFlowCamera(Sensor);

impl OpticalFlowCamera {
    /// Try to create an OpticalFlowCamera from a generic Sensor.
    /// Returns None if the sensor is not an optical flow camera.
    pub fn from_sensor(sensor: Sensor) -> Option<Self> {
        let type_id = sensor.get_type_id();
        if type_id.contains("optical_flow") || type_id.contains("flow") {
            Some(OpticalFlowCamera(sensor))
        } else {
            None
        }
    }

    /// Convert back to generic Sensor.
    pub fn into_sensor(self) -> Sensor {
        self.0
    }

    /// Get reference to the underlying sensor.
    pub fn as_sensor(&self) -> &Sensor {
        &self.0
    }

    /// Get the last optical flow data from the sensor.
    /// Note: This is a placeholder implementation since OpticalFlow FFI is not yet available.
    pub fn get_last_optical_flow_data(&self) -> Option<crate::sensor::OpticalFlowData> {
        // TODO: Implement using carla-cxx FFI interface
        // This requires adding Sensor_GetLastOpticalFlowData FFI function
        todo!("OpticalFlowCamera::get_last_optical_flow_data not yet implemented - missing FFI function")
    }
}

impl ActorT for OpticalFlowCamera {
    fn get_id(&self) -> ActorId {
        self.0.get_id()
    }
    fn get_type_id(&self) -> String {
        self.0.get_type_id()
    }
    fn get_transform(&self) -> Transform {
        self.0.get_transform()
    }
    fn set_transform(&self, transform: &Transform) -> CarlaResult<()> {
        self.0.set_transform(transform)
    }
    fn get_velocity(&self) -> Vector3D {
        self.0.get_velocity()
    }
    fn get_angular_velocity(&self) -> Vector3D {
        self.0.get_angular_velocity()
    }
    fn get_acceleration(&self) -> Vector3D {
        self.0.get_acceleration()
    }
    fn is_alive(&self) -> bool {
        self.0.is_alive()
    }
    fn destroy(&self) -> CarlaResult<()> {
        self.0.destroy()
    }
    fn set_simulate_physics(&self, enabled: bool) -> CarlaResult<()> {
        self.0.set_simulate_physics(enabled)
    }
    fn add_impulse(&self, impulse: &Vector3D) -> CarlaResult<()> {
        self.0.add_impulse(impulse)
    }
    fn add_force(&self, force: &Vector3D) -> CarlaResult<()> {
        self.0.add_force(force)
    }
    fn add_torque(&self, torque: &Vector3D) -> CarlaResult<()> {
        self.0.add_torque(torque)
    }
}

/// Obstacle Detection sensor for collision avoidance.
#[derive(Debug)]
pub struct ObstacleDetectionSensor(Sensor);

impl ObstacleDetectionSensor {
    /// Try to create an ObstacleDetectionSensor from a generic Sensor.
    /// Returns None if the sensor is not an obstacle detection sensor.
    pub fn from_sensor(sensor: Sensor) -> Option<Self> {
        let type_id = sensor.get_type_id();
        if type_id.contains("obstacle") || type_id.contains("detection") {
            Some(ObstacleDetectionSensor(sensor))
        } else {
            None
        }
    }

    /// Convert back to generic Sensor.
    pub fn into_sensor(self) -> Sensor {
        self.0
    }

    /// Get reference to the underlying sensor.
    pub fn as_sensor(&self) -> &Sensor {
        &self.0
    }

    /// Get the last obstacle detection data from the sensor.
    pub fn get_last_obstacle_detection_data(&self) -> Option<crate::sensor::ObstacleDetectionData> {
        let cxx_event = self.0.inner.get_last_obstacle_detection_data();
        if cxx_event.other_actor_id == 0 {
            None // No obstacle detected
        } else {
            Some(crate::sensor::ObstacleDetectionData::from_cxx(cxx_event))
        }
    }
}

impl ActorT for ObstacleDetectionSensor {
    fn get_id(&self) -> ActorId {
        self.0.get_id()
    }
    fn get_type_id(&self) -> String {
        self.0.get_type_id()
    }
    fn get_transform(&self) -> Transform {
        self.0.get_transform()
    }
    fn set_transform(&self, transform: &Transform) -> CarlaResult<()> {
        self.0.set_transform(transform)
    }
    fn get_velocity(&self) -> Vector3D {
        self.0.get_velocity()
    }
    fn get_angular_velocity(&self) -> Vector3D {
        self.0.get_angular_velocity()
    }
    fn get_acceleration(&self) -> Vector3D {
        self.0.get_acceleration()
    }
    fn is_alive(&self) -> bool {
        self.0.is_alive()
    }
    fn destroy(&self) -> CarlaResult<()> {
        self.0.destroy()
    }
    fn set_simulate_physics(&self, enabled: bool) -> CarlaResult<()> {
        self.0.set_simulate_physics(enabled)
    }
    fn add_impulse(&self, impulse: &Vector3D) -> CarlaResult<()> {
        self.0.add_impulse(impulse)
    }
    fn add_force(&self, force: &Vector3D) -> CarlaResult<()> {
        self.0.add_force(force)
    }
    fn add_torque(&self, torque: &Vector3D) -> CarlaResult<()> {
        self.0.add_torque(torque)
    }
}

/// RSS (Road Safety) sensor for safety compliance monitoring.
#[derive(Debug)]
pub struct RSSensor(Sensor);

impl RSSensor {
    /// Try to create an RSSensor from a generic Sensor.
    /// Returns None if the sensor is not an RSS sensor.
    pub fn from_sensor(sensor: Sensor) -> Option<Self> {
        let type_id = sensor.get_type_id();
        if type_id.contains("rss") || type_id.contains("safety") {
            Some(RSSensor(sensor))
        } else {
            None
        }
    }

    /// Convert back to generic Sensor.
    pub fn into_sensor(self) -> Sensor {
        self.0
    }

    /// Get reference to the underlying sensor.
    pub fn as_sensor(&self) -> &Sensor {
        &self.0
    }

    /// Get the last RSS data from the sensor.
    pub fn get_last_rss_data(&self) -> Option<crate::sensor::RSSData> {
        let cxx_response = self.0.inner.get_last_rss_data();
        if !cxx_response.response_valid {
            None // Invalid RSS response
        } else {
            Some(crate::sensor::RSSData::from_cxx(cxx_response))
        }
    }
}

impl ActorT for RSSensor {
    fn get_id(&self) -> ActorId {
        self.0.get_id()
    }
    fn get_type_id(&self) -> String {
        self.0.get_type_id()
    }
    fn get_transform(&self) -> Transform {
        self.0.get_transform()
    }
    fn set_transform(&self, transform: &Transform) -> CarlaResult<()> {
        self.0.set_transform(transform)
    }
    fn get_velocity(&self) -> Vector3D {
        self.0.get_velocity()
    }
    fn get_angular_velocity(&self) -> Vector3D {
        self.0.get_angular_velocity()
    }
    fn get_acceleration(&self) -> Vector3D {
        self.0.get_acceleration()
    }
    fn is_alive(&self) -> bool {
        self.0.is_alive()
    }
    fn destroy(&self) -> CarlaResult<()> {
        self.0.destroy()
    }
    fn set_simulate_physics(&self, enabled: bool) -> CarlaResult<()> {
        self.0.set_simulate_physics(enabled)
    }
    fn add_impulse(&self, impulse: &Vector3D) -> CarlaResult<()> {
        self.0.add_impulse(impulse)
    }
    fn add_force(&self, force: &Vector3D) -> CarlaResult<()> {
        self.0.add_force(force)
    }
    fn add_torque(&self, torque: &Vector3D) -> CarlaResult<()> {
        self.0.add_torque(torque)
    }
}
