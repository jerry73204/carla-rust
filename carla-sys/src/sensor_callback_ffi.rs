//! Manual FFI declarations for sensor callbacks.
//!
//! This module provides FFI bindings for sensor callbacks that cannot be
//! expressed through CXX due to its limitations with function pointers.

use crate::ffi::Sensor;

/// FFI-safe callback function type for sensor data.
pub type SensorDataCallback =
    unsafe extern "C" fn(sensor_id: u32, data: *const u8, size: usize, user_data: *mut u8);

// Manual FFI declarations for sensor callback functions
extern "C" {
    /// Register a callback for sensor data.
    ///
    /// # Safety
    /// - The sensor reference must be valid
    /// - The callback function must be valid for the lifetime of the registration
    /// - The user_data pointer must remain valid while the callback is registered
    pub fn Sensor_RegisterCallback(
        sensor: &Sensor,
        callback: SensorDataCallback,
        user_data: *mut u8,
    ) -> u64;

    /// Unregister a callback by handle.
    ///
    /// # Safety
    /// The handle must be a valid callback handle returned by Sensor_RegisterCallback.
    pub fn Sensor_UnregisterCallback(handle: u64) -> bool;

    /// Clear all callbacks for a sensor.
    ///
    /// # Safety
    /// The sensor reference must be valid.
    pub fn Sensor_ClearCallbacks(sensor: &Sensor);
}
