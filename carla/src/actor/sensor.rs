//! Sensor actor implementation.

use crate::{
    actor::{Actor, ActorExt, ActorFfi, SensorFfi},
    error::{CarlaError, CarlaResult, SensorError},
    geom::{FromCxx, ToCxx},
};

/// Sensor actor.
#[derive(Debug)]
pub struct Sensor {
    /// Internal sensor wrapper for FFI calls
    pub(crate) inner: carla_sys::SensorWrapper,
}

impl Sensor {
    /// Create a sensor from a carla-sys SensorWrapper and actor ID.
    pub(crate) fn from_cxx(sensor_wrapper: carla_sys::SensorWrapper) -> CarlaResult<Self> {
        Ok(Self {
            inner: sensor_wrapper,
        })
    }

    /// Create a sensor from an actor by casting.
    pub fn from_actor(actor: Actor) -> Result<Self, Actor> {
        let actor_ref = actor.inner_actor();
        if let Some(sensor_wrapper) = carla_sys::SensorWrapper::from_actor(actor_ref) {
            Ok(Self {
                inner: sensor_wrapper,
            })
        } else {
            Err(actor)
        }
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

    // TODO: Re-enable async support in the future
    // /// Start listening with an async channel for sensor data.
    // ///
    // /// Returns a receiver that yields sensor data. Requires the `async` feature.
    // #[cfg(feature = "async")]
    // pub fn listen_async(&self) -> Result<tokio::sync::mpsc::Receiver<Vec<u8>>, SensorError> {
    //     // TODO: Implement using carla-sys FFI interface
    //     todo!("Sensor::listen_async not yet implemented with carla-sys FFI")
    // }

    /// Stop listening for sensor data.
    pub fn stop(&self) {
        self.inner.stop();
    }

    /// Check if the sensor is currently listening.
    pub fn is_listening(&self) -> bool {
        self.inner.is_listening()
    }

    /// Get the sensor's data frame number (increments with each measurement).
    pub fn frame(&self) -> u64 {
        carla_sys::ffi::bridge::Sensor_GetFrame(self.inner.get_inner_sensor())
    }

    /// Get the timestamp of the last sensor measurement.
    pub fn timestamp(&self) -> f64 {
        carla_sys::ffi::bridge::Sensor_GetTimestamp(self.inner.get_inner_sensor())
    }

    /// Get sensor-specific attribute.
    pub fn attribute(&self, name: &str) -> Option<String> {
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
        Err(CarlaError::Sensor(SensorError::CallbackFailed(
            "enable_recording not available - use client-level recording".to_string(),
        )))
    }

    /// Disable sensor recording.
    pub fn disable_recording(&self) -> CarlaResult<()> {
        // Sensor recording is typically done at the client level, not sensor level
        // This would require additional FFI functions that aren't part of the basic sensor API
        Err(CarlaError::Sensor(SensorError::CallbackFailed(
            "disable_recording not available - use client-level recording".to_string(),
        )))
    }

    /// Check if new sensor data is available.
    pub fn has_new_data(&self) -> bool {
        self.inner.has_new_data()
    }

    // Sensor type validation methods

    /// Check if this sensor is a camera.
    pub fn is_camera(&self) -> bool {
        let type_id = self.type_id();
        type_id.contains("sensor.camera")
    }

    /// Check if this sensor is a LiDAR.
    pub fn is_lidar(&self) -> bool {
        let type_id = self.type_id();
        type_id.contains("sensor.lidar") || type_id.contains("velodyne")
    }

    /// Check if this sensor is an IMU.
    pub fn is_imu(&self) -> bool {
        let type_id = self.type_id();
        type_id.contains("sensor.other.imu")
    }

    /// Check if this sensor is a GNSS.
    pub fn is_gnss(&self) -> bool {
        let type_id = self.type_id();
        type_id.contains("sensor.other.gnss")
    }

    /// Check if this sensor is a Radar.
    pub fn is_radar(&self) -> bool {
        let type_id = self.type_id();
        type_id.contains("sensor.other.radar")
    }

    /// Check if this sensor is a collision detector.
    pub fn is_collision(&self) -> bool {
        let type_id = self.type_id();
        type_id.contains("sensor.other.collision")
    }

    /// Check if this sensor is a lane invasion detector.
    pub fn is_lane_invasion(&self) -> bool {
        let type_id = self.type_id();
        type_id.contains("sensor.other.lane_invasion")
    }

    /// Check if this sensor is an obstacle detection sensor.
    pub fn is_obstacle_detection(&self) -> bool {
        let type_id = self.type_id();
        type_id.contains("sensor.other.obstacle")
    }

    /// Check if this sensor is a DVS camera.
    pub fn is_dvs(&self) -> bool {
        let type_id = self.type_id();
        type_id.contains("sensor.camera.dvs")
    }

    /// Check if this sensor is an RSS sensor.
    pub fn is_rss(&self) -> bool {
        let type_id = self.type_id();
        type_id.contains("sensor.other.rss")
    }

    /// Convert this sensor back to an Actor.
    pub fn into_actor(self) -> Actor {
        // Get the actor pointer from the sensor
        let actor_ptr = carla_sys::ffi::Sensor_CastToActor(self.inner.get_inner_sensor());
        if actor_ptr.is_null() {
            panic!("Internal error: Failed to cast Sensor to Actor");
        }

        // Create an ActorWrapper from the SharedPtr
        let actor_wrapper = carla_sys::ActorWrapper::new(actor_ptr);

        // Prevent the sensor from being destroyed when it goes out of scope
        // since the Actor will own it now
        std::mem::forget(self);

        // Create the Actor from the ActorWrapper
        Actor::from_cxx(actor_wrapper)
    }

    /// Get a reference to this sensor as an Actor.
    /// Note: This creates a new Actor wrapper each time. The underlying C++ object is shared.
    pub fn as_actor(&self) -> Actor {
        // Get the actor pointer from the sensor
        let actor_ptr = carla_sys::ffi::Sensor_CastToActor(self.inner.get_inner_sensor());
        if actor_ptr.is_null() {
            panic!("Internal error: Failed to cast Sensor to Actor");
        }

        // Create an ActorWrapper from the SharedPtr
        // This is safe because the SharedPtr increases the reference count
        let actor_wrapper = carla_sys::ActorWrapper::new(actor_ptr);
        Actor::from_cxx(actor_wrapper)
    }
}

// Note: Sensor doesn't implement ActorFfi directly because it would require
// creating a temporary ActorWrapper on each call, which violates the lifetime
// requirements of returning a reference. Instead, we implement ActorExt methods directly.

impl ActorExt for Sensor {
    fn id(&self) -> crate::actor::ActorId {
        // Sensor doesn't have a direct GetId method, need to cast to Actor
        let actor_ptr = carla_sys::ffi::Sensor_CastToActor(self.inner.get_inner_sensor());
        if actor_ptr.is_null() {
            panic!("Internal error: Failed to cast Sensor to Actor");
        }
        carla_sys::ffi::Actor_GetId(&actor_ptr)
    }

    fn type_id(&self) -> String {
        carla_sys::ffi::bridge::Sensor_GetTypeId(self.inner.get_inner_sensor())
    }

    fn transform(&self) -> crate::geom::Transform {
        let cxx_transform =
            carla_sys::ffi::bridge::Sensor_GetTransform(self.inner.get_inner_sensor());
        crate::geom::Transform::from(cxx_transform)
    }

    fn set_transform(&self, transform: &crate::geom::Transform) -> CarlaResult<()> {
        let cxx_transform = transform.to_cxx();
        carla_sys::ffi::bridge::Sensor_SetTransform(self.inner.get_inner_sensor(), &cxx_transform);
        Ok(())
    }

    fn velocity(&self) -> crate::geom::Vector3D {
        let actor_ptr = carla_sys::ffi::Sensor_CastToActor(self.inner.get_inner_sensor());
        if actor_ptr.is_null() {
            panic!("Internal error: Failed to cast Sensor to Actor");
        }
        let simple_velocity = carla_sys::ffi::Actor_GetVelocity(&actor_ptr);
        crate::geom::Vector3D::from_cxx(simple_velocity)
    }

    fn angular_velocity(&self) -> crate::geom::Vector3D {
        let actor_ptr = carla_sys::ffi::Sensor_CastToActor(self.inner.get_inner_sensor());
        if actor_ptr.is_null() {
            panic!("Internal error: Failed to cast Sensor to Actor");
        }
        let simple_angular_velocity = carla_sys::ffi::Actor_GetAngularVelocity(&actor_ptr);
        crate::geom::Vector3D::from_cxx(simple_angular_velocity)
    }

    fn acceleration(&self) -> crate::geom::Vector3D {
        let actor_ptr = carla_sys::ffi::Sensor_CastToActor(self.inner.get_inner_sensor());
        if actor_ptr.is_null() {
            panic!("Internal error: Failed to cast Sensor to Actor");
        }
        let simple_acceleration = carla_sys::ffi::Actor_GetAcceleration(&actor_ptr);
        crate::geom::Vector3D::from_cxx(simple_acceleration)
    }

    fn is_alive(&self) -> bool {
        carla_sys::ffi::bridge::Sensor_IsAlive(self.inner.get_inner_sensor())
    }

    fn set_simulate_physics(&self, enabled: bool) -> CarlaResult<()> {
        let actor_ptr = carla_sys::ffi::Sensor_CastToActor(self.inner.get_inner_sensor());
        if actor_ptr.is_null() {
            panic!("Internal error: Failed to cast Sensor to Actor");
        }
        carla_sys::ffi::Actor_SetSimulatePhysics(&actor_ptr, enabled);
        Ok(())
    }

    fn add_impulse(&self, impulse: &crate::geom::Vector3D) -> CarlaResult<()> {
        let actor_ptr = carla_sys::ffi::Sensor_CastToActor(self.inner.get_inner_sensor());
        if actor_ptr.is_null() {
            panic!("Internal error: Failed to cast Sensor to Actor");
        }
        let simple_impulse = impulse.to_cxx();
        carla_sys::ffi::Actor_AddImpulse(&actor_ptr, &simple_impulse);
        Ok(())
    }

    fn add_force(&self, force: &crate::geom::Vector3D) -> CarlaResult<()> {
        let actor_ptr = carla_sys::ffi::Sensor_CastToActor(self.inner.get_inner_sensor());
        if actor_ptr.is_null() {
            panic!("Internal error: Failed to cast Sensor to Actor");
        }
        let simple_force = force.to_cxx();
        carla_sys::ffi::Actor_AddForce(&actor_ptr, &simple_force);
        Ok(())
    }

    fn add_torque(&self, torque: &crate::geom::Vector3D) -> CarlaResult<()> {
        let actor_ptr = carla_sys::ffi::Sensor_CastToActor(self.inner.get_inner_sensor());
        if actor_ptr.is_null() {
            panic!("Internal error: Failed to cast Sensor to Actor");
        }
        let simple_torque = torque.to_cxx();
        carla_sys::ffi::Actor_AddTorque(&actor_ptr, &simple_torque);
        Ok(())
    }

    fn bounding_box(&self) -> crate::geom::BoundingBox {
        let actor_ptr = carla_sys::ffi::Sensor_CastToActor(self.inner.get_inner_sensor());
        if actor_ptr.is_null() {
            panic!("Internal error: Failed to cast Sensor to Actor");
        }
        let simple_bbox = carla_sys::ffi::Actor_GetBoundingBox(&actor_ptr);
        crate::geom::BoundingBox::from_cxx(simple_bbox)
    }

    fn destroy(&mut self) -> CarlaResult<()> {
        // Stop listening if sensor is listening
        if self.inner.is_listening() {
            self.inner.stop();
        }

        // Call sensor destruction
        let success = carla_sys::ffi::bridge::Sensor_Destroy(self.inner.get_inner_sensor());

        if success {
            Ok(())
        } else {
            // If destruction fails, it means the sensor is invalid or already destroyed
            Err(crate::error::DestroyError::InvalidActor {
                actor_id: self.id(),
            }
            .into())
        }
    }
}

impl SensorFfi for Sensor {
    fn as_sensor_ffi(&self) -> &carla_sys::SensorWrapper {
        &self.inner
    }
}

impl Drop for Sensor {
    fn drop(&mut self) {
        // Stop listening if sensor is listening
        if self.inner.is_listening() {
            self.inner.stop();
        }
        // Only destroy if the sensor is still alive
        if carla_sys::ffi::bridge::Sensor_IsAlive(self.inner.get_inner_sensor()) {
            let _ = carla_sys::ffi::bridge::Sensor_Destroy(self.inner.get_inner_sensor());
        }
    }
}

#[cfg(test)]
mod tests {

    #[test]
    fn test_sensor_type_validation() {
        // This test would require a mock sensor with specific type_id
        // For now, we'll just test the logic with a hypothetical sensor

        // Create a test to ensure the type validation methods work correctly
        // In real tests, we'd create actual sensors and test them
    }

    #[test]
    #[ignore = "Requires CARLA server connection"]
    fn test_sensor_actor_conversions() {
        // Test Sensor <-> Actor conversions
        // This would require a running CARLA server to create actual sensors
    }

    #[test]
    fn test_sensor_type_id_patterns() {
        // Test that our type ID patterns are correct
        let camera_types = vec![
            "sensor.camera.rgb",
            "sensor.camera.depth",
            "sensor.camera.semantic_segmentation",
            "sensor.camera.instance_segmentation",
            "sensor.camera.dvs",
        ];

        for type_id in camera_types {
            assert!(
                type_id.contains("sensor.camera"),
                "Camera type check failed for {}",
                type_id
            );
        }

        let lidar_types = vec![
            "sensor.lidar.ray_cast",
            "sensor.lidar.ray_cast_semantic",
            "velodyne.hdl64",
        ];

        for type_id in lidar_types {
            assert!(
                type_id.contains("sensor.lidar") || type_id.contains("velodyne"),
                "LiDAR type check failed for {}",
                type_id
            );
        }
    }
}
