//! ROS2 integration for CARLA simulator.
//!
//! This module provides simplified client-side ROS2 integration functionality for CARLA,
//! focusing on sensor control which is the primary client-side ROS2 feature.
//! The ROS2 system in CARLA 0.10.0 provides native integration without requiring external bridges.
//!
//! For more complex ROS2 features like custom publishers/subscribers, use the official
//! CARLA ROS2 bridge or access the ROS2 functionality directly from Python/C++.

use crate::ffi::{self, Sensor};

/// Vehicle control command for ROS2 subscribers.
///
/// This represents the command structure that can be received
/// via ROS2 for controlling vehicles. This is mainly used for documentation
/// and type safety when working with ROS2 vehicle control topics.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VehicleControlCommand {
    /// Throttle input [0.0, 1.0]
    pub throttle: f32,
    /// Steering input [-1.0, 1.0] (left negative, right positive)
    pub steer: f32,
    /// Brake input [0.0, 1.0]
    pub brake: f32,
    /// Hand brake engaged
    pub hand_brake: bool,
    /// Reverse gear engaged
    pub reverse: bool,
    /// Gear selection (-1 for reverse, 0 for neutral, 1+ for forward gears)
    pub gear: i32,
    /// Manual gear shifting mode
    pub manual_gear_shift: bool,
}

impl Default for VehicleControlCommand {
    fn default() -> Self {
        Self {
            throttle: 0.0,
            steer: 0.0,
            brake: 0.0,
            hand_brake: false,
            reverse: false,
            gear: 1,
            manual_gear_shift: false,
        }
    }
}

/// Extension trait for sensors to add ROS2 functionality.
pub trait SensorROS2Ext {
    /// Enable ROS2 publishing for this sensor.
    ///
    /// When enabled, the sensor will publish its data to the appropriate
    /// ROS2 topic based on its type and the actor's ROS2 name.
    fn enable_for_ros(&self);

    /// Disable ROS2 publishing for this sensor.
    fn disable_for_ros(&self);

    /// Check if ROS2 publishing is enabled for this sensor.
    fn is_enabled_for_ros(&self) -> bool;
}

impl SensorROS2Ext for Sensor {
    fn enable_for_ros(&self) {
        ffi::bridge::Sensor_EnableForROS(self);
    }

    fn disable_for_ros(&self) {
        ffi::bridge::Sensor_DisableForROS(self);
    }

    fn is_enabled_for_ros(&self) -> bool {
        ffi::bridge::Sensor_IsEnabledForROS(self)
    }
}

/// ROS2 utility functions and helpers.
pub mod ros2_utils {
    use super::*;

    /// Enable ROS2 for multiple sensors at once.
    ///
    /// # Arguments
    /// * `sensors` - List of sensors to enable for ROS2
    pub fn enable_sensors_for_ros2(sensors: &[&Sensor]) {
        for sensor in sensors {
            sensor.enable_for_ros();
        }
    }

    /// Disable ROS2 for multiple sensors at once.
    ///
    /// # Arguments
    /// * `sensors` - List of sensors to disable for ROS2
    pub fn disable_sensors_for_ros2(sensors: &[&Sensor]) {
        for sensor in sensors {
            sensor.disable_for_ros();
        }
    }

    /// Generate a standard ROS2 topic name for a sensor.
    ///
    /// This follows the CARLA ROS2 naming convention:
    /// `/carla/{actor_name}/{sensor_name}`
    ///
    /// # Arguments
    /// * `actor_name` - Name of the actor/vehicle
    /// * `sensor_name` - Name/type of the sensor
    ///
    /// # Returns
    /// Standard ROS2 topic name
    pub fn generate_topic_name(actor_name: &str, sensor_name: &str) -> String {
        format!("/carla/{}/{}", actor_name, sensor_name)
    }

    /// Common ROS2 sensor names used in CARLA.
    pub mod sensor_names {
        /// RGB camera sensor topic suffix
        pub const RGB_CAMERA: &str = "rgb_camera";
        /// Depth camera sensor topic suffix
        pub const DEPTH_CAMERA: &str = "depth_camera";
        /// Semantic segmentation camera topic suffix
        pub const SEMANTIC_CAMERA: &str = "semantic_camera";
        /// LiDAR sensor topic suffix
        pub const LIDAR: &str = "lidar";
        /// Radar sensor topic suffix
        pub const RADAR: &str = "radar";
        /// IMU sensor topic suffix
        pub const IMU: &str = "imu";
        /// GNSS sensor topic suffix
        pub const GNSS: &str = "gnss";
        /// Collision detector topic suffix
        pub const COLLISION: &str = "collision";
        /// Lane invasion detector topic suffix
        pub const LANE_INVASION: &str = "lane_invasion";
    }
}
