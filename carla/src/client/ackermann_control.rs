use crate::{
    client::{ActorBase, Vehicle},
    rpc::{AckermannControllerSettings, VehicleAckermannControl},
    utils::check_carla_error,
};
use anyhow::{anyhow, Result};
use carla_sys::*;
use std::ptr;

/// Enhanced vehicle control using the Ackermann steering model.
/// This provides more realistic vehicle dynamics compared to basic control.
/// 
/// The Ackermann steering model accounts for:
/// - Proper steering geometry with inner/outer wheel angles
/// - Vehicle wheelbase and track width
/// - Speed-dependent steering response
/// - Advanced PID control for smooth operation
#[derive(Clone, Debug)]
pub struct AckermannController {
    inner: *mut carla_ackermann_controller_t,
    vehicle_ptr: *mut carla_vehicle_t,
}

impl AckermannController {
    /// Create a new Ackermann controller for a vehicle.
    /// 
    /// # Arguments
    /// * `vehicle` - The vehicle to control
    /// * `settings` - PID controller settings
    /// 
    /// # Returns
    /// A new Ackermann controller instance
    pub fn new(vehicle: &Vehicle, settings: &AckermannControllerSettings) -> Result<Self> {
        let c_settings = settings.to_c_settings();
        let controller_ptr = unsafe {
            carla_ackermann_controller_create(vehicle.raw_ptr(), &c_settings)
        };
        
        if controller_ptr.is_null() {
            return Err(anyhow!("Failed to create Ackermann controller"));
        }
        
        Ok(Self {
            inner: controller_ptr,
            vehicle_ptr: vehicle.raw_ptr(),
        })
    }

    /// Apply Ackermann control to the vehicle.
    /// 
    /// # Arguments
    /// * `control` - Ackermann control parameters
    /// 
    /// # Returns
    /// Result indicating success or failure
    pub fn apply_control(&mut self, control: &VehicleAckermannControl) -> Result<()> {
        let c_control = control.to_c_control();
        let error = unsafe {
            carla_ackermann_controller_apply_control(self.inner, &c_control)
        };
        check_carla_error(error)
    }

    /// Update the PID controller settings.
    /// 
    /// # Arguments
    /// * `settings` - New PID controller settings
    /// 
    /// # Returns
    /// Result indicating success or failure
    pub fn update_settings(&mut self, settings: &AckermannControllerSettings) -> Result<()> {
        let c_settings = settings.to_c_settings();
        let error = unsafe {
            carla_ackermann_controller_update_settings(self.inner, &c_settings)
        };
        check_carla_error(error)
    }

    /// Get the current PID controller settings.
    /// 
    /// # Returns
    /// Current controller settings
    pub fn get_settings(&self) -> AckermannControllerSettings {
        let c_settings = unsafe {
            carla_ackermann_controller_get_settings(self.inner)
        };
        AckermannControllerSettings::from_c_settings(c_settings)
    }

    /// Get the current vehicle state for Ackermann control.
    /// 
    /// # Returns
    /// Current vehicle state including wheel angles and speeds
    pub fn get_vehicle_state(&self) -> AckermannVehicleState {
        let c_state = unsafe {
            carla_ackermann_controller_get_vehicle_state(self.inner)
        };
        AckermannVehicleState::from_c_state(c_state)
    }

    /// Set the target speed for the controller.
    /// 
    /// # Arguments
    /// * `speed` - Target speed in m/s
    /// 
    /// # Returns
    /// Result indicating success or failure
    pub fn set_target_speed(&mut self, speed: f32) -> Result<()> {
        let error = unsafe {
            carla_ackermann_controller_set_target_speed(self.inner, speed)
        };
        check_carla_error(error)
    }

    /// Set the target steering angle.
    /// 
    /// # Arguments
    /// * `angle` - Target steering angle in radians
    /// 
    /// # Returns
    /// Result indicating success or failure
    pub fn set_target_steering(&mut self, angle: f32) -> Result<()> {
        let error = unsafe {
            carla_ackermann_controller_set_target_steering(self.inner, angle)
        };
        check_carla_error(error)
    }

    /// Enable or disable automatic speed control.
    /// When enabled, the controller will maintain the target speed.
    /// 
    /// # Arguments
    /// * `enabled` - Whether to enable automatic speed control
    /// 
    /// # Returns
    /// Result indicating success or failure
    pub fn set_auto_speed_control(&mut self, enabled: bool) -> Result<()> {
        let error = unsafe {
            carla_ackermann_controller_set_auto_speed_control(self.inner, enabled)
        };
        check_carla_error(error)
    }

    /// Get the current control output from the Ackermann controller.
    /// This shows what control values the controller is currently applying.
    /// 
    /// # Returns
    /// Current control output
    pub fn get_control_output(&self) -> VehicleAckermannControl {
        let c_control = unsafe {
            carla_ackermann_controller_get_control_output(self.inner)
        };
        VehicleAckermannControl::from_c_control(c_control)
    }

    /// Reset the PID controller internal state.
    /// This clears integral and derivative terms, useful when starting control.
    /// 
    /// # Returns
    /// Result indicating success or failure
    pub fn reset_controller(&mut self) -> Result<()> {
        let error = unsafe {
            carla_ackermann_controller_reset(self.inner)
        };
        check_carla_error(error)
    }

    /// Get controller performance metrics.
    /// 
    /// # Returns
    /// Performance metrics including errors and control statistics
    pub fn get_metrics(&self) -> AckermannControllerMetrics {
        let c_metrics = unsafe {
            carla_ackermann_controller_get_metrics(self.inner)
        };
        AckermannControllerMetrics::from_c_metrics(c_metrics)
    }

    /// Calculate optimal steering angles for the current vehicle configuration.
    /// This uses the Ackermann steering geometry to compute proper wheel angles.
    /// 
    /// # Arguments
    /// * `steering_angle` - Front wheel steering angle in radians
    /// 
    /// # Returns
    /// Individual wheel angles (front_left, front_right, rear_left, rear_right)
    pub fn calculate_wheel_angles(&self, steering_angle: f32) -> (f32, f32, f32, f32) {
        let mut front_left = 0.0;
        let mut front_right = 0.0;
        let mut rear_left = 0.0;
        let mut rear_right = 0.0;
        
        unsafe {
            carla_ackermann_calculate_wheel_angles(
                self.inner,
                steering_angle,
                &mut front_left,
                &mut front_right,
                &mut rear_left,
                &mut rear_right,
            );
        }
        
        (front_left, front_right, rear_left, rear_right)
    }

    /// Get the vehicle's wheelbase (distance between front and rear axles).
    /// 
    /// # Returns
    /// Wheelbase in meters
    pub fn get_wheelbase(&self) -> f32 {
        unsafe { carla_ackermann_controller_get_wheelbase(self.inner) }
    }

    /// Get the vehicle's track width (distance between left and right wheels).
    /// 
    /// # Returns
    /// Track width in meters
    pub fn get_track_width(&self) -> f32 {
        unsafe { carla_ackermann_controller_get_track_width(self.inner) }
    }
}

impl Drop for AckermannController {
    fn drop(&mut self) {
        if !self.inner.is_null() {
            unsafe {
                carla_ackermann_controller_destroy(self.inner);
            }
            self.inner = ptr::null_mut();
        }
    }
}

/// Current state of a vehicle for Ackermann control.
#[derive(Clone, Debug)]
pub struct AckermannVehicleState {
    /// Current vehicle speed in m/s
    pub speed: f32,
    /// Current steering angle in radians
    pub steering_angle: f32,
    /// Individual wheel speeds in m/s (front_left, front_right, rear_left, rear_right)
    pub wheel_speeds: (f32, f32, f32, f32),
    /// Individual wheel angles in radians (front_left, front_right, rear_left, rear_right)
    pub wheel_angles: (f32, f32, f32, f32),
    /// Vehicle heading in radians
    pub heading: f32,
    /// Angular velocity in rad/s
    pub angular_velocity: f32,
}

impl AckermannVehicleState {
    pub(crate) fn from_c_state(state: carla_ackermann_vehicle_state_t) -> Self {
        Self {
            speed: state.speed,
            steering_angle: state.steering_angle,
            wheel_speeds: (
                state.wheel_speeds[0],
                state.wheel_speeds[1],
                state.wheel_speeds[2],
                state.wheel_speeds[3],
            ),
            wheel_angles: (
                state.wheel_angles[0],
                state.wheel_angles[1],
                state.wheel_angles[2],
                state.wheel_angles[3],
            ),
            heading: state.heading,
            angular_velocity: state.angular_velocity,
        }
    }
}

/// Performance metrics for the Ackermann controller.
#[derive(Clone, Debug)]
pub struct AckermannControllerMetrics {
    /// Current speed error (target - actual) in m/s
    pub speed_error: f32,
    /// Current steering error (target - actual) in radians
    pub steering_error: f32,
    /// Integral of speed error over time
    pub speed_integral: f32,
    /// Integral of steering error over time
    pub steering_integral: f32,
    /// Derivative of speed error
    pub speed_derivative: f32,
    /// Derivative of steering error
    pub steering_derivative: f32,
    /// Control loop frequency in Hz
    pub control_frequency: f32,
    /// Time since last control update in seconds
    pub last_update_time: f64,
}

impl AckermannControllerMetrics {
    pub(crate) fn from_c_metrics(metrics: carla_ackermann_controller_metrics_t) -> Self {
        Self {
            speed_error: metrics.speed_error,
            steering_error: metrics.steering_error,
            speed_integral: metrics.speed_integral,
            steering_integral: metrics.steering_integral,
            speed_derivative: metrics.speed_derivative,
            steering_derivative: metrics.steering_derivative,
            control_frequency: metrics.control_frequency,
            last_update_time: metrics.last_update_time,
        }
    }
}

/// Utility functions for Ackermann steering calculations.
pub struct AckermannUtils;

impl AckermannUtils {
    /// Calculate the turning radius for a given steering angle and wheelbase.
    /// 
    /// # Arguments
    /// * `steering_angle` - Front wheel steering angle in radians
    /// * `wheelbase` - Vehicle wheelbase in meters
    /// 
    /// # Returns
    /// Turning radius in meters
    pub fn calculate_turning_radius(steering_angle: f32, wheelbase: f32) -> f32 {
        if steering_angle.abs() < f32::EPSILON {
            f32::INFINITY
        } else {
            wheelbase / steering_angle.tan()
        }
    }

    /// Calculate the required steering angle for a desired turning radius.
    /// 
    /// # Arguments
    /// * `turning_radius` - Desired turning radius in meters
    /// * `wheelbase` - Vehicle wheelbase in meters
    /// 
    /// # Returns
    /// Required steering angle in radians
    pub fn calculate_steering_angle(turning_radius: f32, wheelbase: f32) -> f32 {
        if turning_radius.is_infinite() {
            0.0
        } else {
            (wheelbase / turning_radius).atan()
        }
    }

    /// Calculate the speed of the inside and outside wheels during a turn.
    /// 
    /// # Arguments
    /// * `vehicle_speed` - Vehicle center speed in m/s
    /// * `steering_angle` - Front wheel steering angle in radians
    /// * `wheelbase` - Vehicle wheelbase in meters
    /// * `track_width` - Vehicle track width in meters
    /// 
    /// # Returns
    /// (inside_wheel_speed, outside_wheel_speed) in m/s
    pub fn calculate_wheel_speeds(
        vehicle_speed: f32,
        steering_angle: f32,
        wheelbase: f32,
        track_width: f32,
    ) -> (f32, f32) {
        if steering_angle.abs() < f32::EPSILON {
            // Straight line motion
            return (vehicle_speed, vehicle_speed);
        }

        let turning_radius = Self::calculate_turning_radius(steering_angle, wheelbase);
        let half_track = track_width / 2.0;

        let inside_radius = turning_radius - half_track;
        let outside_radius = turning_radius + half_track;

        let angular_velocity = vehicle_speed / turning_radius;
        let inside_speed = angular_velocity * inside_radius;
        let outside_speed = angular_velocity * outside_radius;

        (inside_speed, outside_speed)
    }

    /// Validate Ackermann controller settings.
    /// 
    /// # Arguments
    /// * `settings` - Controller settings to validate
    /// 
    /// # Returns
    /// True if settings are valid, false otherwise
    pub fn validate_settings(settings: &AckermannControllerSettings) -> bool {
        // Check for reasonable PID gains
        settings.speed_kp >= 0.0
            && settings.speed_ki >= 0.0
            && settings.speed_kd >= 0.0
            && settings.accel_kp >= 0.0
            && settings.accel_ki >= 0.0
            && settings.accel_kd >= 0.0
            && settings.speed_kp < 1000.0
            && settings.speed_ki < 1000.0
            && settings.speed_kd < 1000.0
            && settings.accel_kp < 1000.0
            && settings.accel_ki < 1000.0
            && settings.accel_kd < 1000.0
    }
}

// SAFETY: AckermannController wraps a thread-safe C API
unsafe impl Send for AckermannController {}
unsafe impl Sync for AckermannController {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_turning_radius_calculation() {
        let wheelbase = 2.5; // meters
        let steering_angle = 0.5; // radians

        let radius = AckermannUtils::calculate_turning_radius(steering_angle, wheelbase);
        assert!(radius > 0.0);

        // Test reverse calculation
        let calculated_angle = AckermannUtils::calculate_steering_angle(radius, wheelbase);
        assert!((calculated_angle - steering_angle).abs() < 1e-6);
    }

    #[test]
    fn test_wheel_speed_calculation() {
        let vehicle_speed = 10.0; // m/s
        let steering_angle = 0.3; // radians
        let wheelbase = 2.5; // meters
        let track_width = 1.8; // meters

        let (inside_speed, outside_speed) =
            AckermannUtils::calculate_wheel_speeds(vehicle_speed, steering_angle, wheelbase, track_width);

        // Inside wheel should be slower than outside wheel
        assert!(inside_speed < outside_speed);
        assert!(inside_speed > 0.0);
        assert!(outside_speed > 0.0);
    }

    #[test]
    fn test_settings_validation() {
        let valid_settings = AckermannControllerSettings {
            speed_kp: 1.0,
            speed_ki: 0.1,
            speed_kd: 0.05,
            accel_kp: 0.5,
            accel_ki: 0.05,
            accel_kd: 0.01,
        };
        assert!(AckermannUtils::validate_settings(&valid_settings));

        let invalid_settings = AckermannControllerSettings {
            speed_kp: -1.0, // Negative gain
            speed_ki: 0.1,
            speed_kd: 0.05,
            accel_kp: 0.5,
            accel_ki: 0.05,
            accel_kd: 0.01,
        };
        assert!(!AckermannUtils::validate_settings(&invalid_settings));
    }
}