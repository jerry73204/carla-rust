//! PID controllers for vehicle control.
//!
//! Provides two types of PID controllers:
//! - [`PIDLongitudinalController`] - Controls throttle and brake for speed regulation
//! - [`PIDLateralController`] - Controls steering for path following

use std::collections::VecDeque;

/// PID controller parameters.
///
/// Defines the gains for a PID (Proportional-Integral-Derivative) controller.
/// These parameters determine how the controller responds to errors:
///
/// - **K_P (Proportional)**: Responds to current error magnitude. Higher values
///   produce stronger correction but may cause overshooting.
/// - **K_I (Integral)**: Responds to accumulated error over time. Eliminates
///   steady-state error but may cause oscillation if too high.
/// - **K_D (Derivative)**: Responds to rate of error change. Provides damping
///   and reduces overshoot.
///
/// # Tuning Guidelines
///
/// 1. Start with K_I = K_D = 0, increase K_P until system oscillates
/// 2. Add K_D to dampen oscillations
/// 3. Add K_I if steady-state error remains
///
/// # Examples
///
/// ```
/// use carla::agents::navigation::pid::PIDParams;
///
/// // Aggressive controller (fast response, may overshoot)
/// let aggressive = PIDParams {
///     k_p: 2.0,
///     k_i: 0.5,
///     k_d: 0.1,
/// };
///
/// // Conservative controller (slow response, stable)
/// let conservative = PIDParams {
///     k_p: 0.5,
///     k_i: 0.1,
///     k_d: 0.05,
/// };
/// ```
#[derive(Debug, Clone, Copy)]
pub struct PIDParams {
    /// Proportional gain (responds to current error)
    pub k_p: f32,
    /// Integral gain (responds to accumulated error)
    pub k_i: f32,
    /// Derivative gain (responds to rate of change)
    pub k_d: f32,
}

impl Default for PIDParams {
    fn default() -> Self {
        Self {
            k_p: 1.0,
            k_i: 0.0,
            k_d: 0.0,
        }
    }
}

/// PID controller for longitudinal control (throttle/brake).
///
/// Controls vehicle speed by computing appropriate throttle or brake values
/// based on the error between target and current speed.
///
/// # Examples
/// ```
/// use carla::agents::navigation::pid::{PIDLongitudinalController, PIDParams};
///
/// let params = PIDParams {
///     k_p: 1.0,
///     k_i: 0.0,
///     k_d: 0.0,
/// };
/// let mut controller = PIDLongitudinalController::new(params, 1.0 / 20.0);
///
/// // Control loop
/// let target_speed = 30.0; // km/h
/// let current_speed = 25.0;
/// let acceleration = controller.run_step(target_speed, current_speed);
/// ```
#[derive(Debug)]
pub struct PIDLongitudinalController {
    params: PIDParams,
    dt: f32,
    error_buffer: VecDeque<f32>,
    max_buffer_size: usize,
}

impl PIDLongitudinalController {
    /// Creates a new longitudinal PID controller.
    ///
    /// # Arguments
    /// * `params` - PID parameters (K_P, K_I, K_D)
    /// * `dt` - Simulation time step in seconds
    pub fn new(params: PIDParams, dt: f32) -> Self {
        Self {
            params,
            dt,
            error_buffer: VecDeque::with_capacity(10),
            max_buffer_size: 10,
        }
    }

    /// Updates PID parameters.
    pub fn change_parameters(&mut self, params: PIDParams) {
        self.params = params;
    }

    /// Executes one step of the controller.
    ///
    /// # Arguments
    /// * `target_speed` - Desired speed in km/h
    /// * `current_speed` - Current vehicle speed in km/h
    ///
    /// # Returns
    /// Acceleration command (positive = throttle, negative = brake)
    pub fn run_step(&mut self, target_speed: f32, current_speed: f32) -> f32 {
        let error = target_speed - current_speed;

        // Add error to buffer
        self.error_buffer.push_back(error);
        if self.error_buffer.len() > self.max_buffer_size {
            self.error_buffer.pop_front();
        }

        // Compute integral (sum of errors)
        let integral: f32 = self.error_buffer.iter().sum();

        // Compute derivative (change in error)
        let derivative = if self.error_buffer.len() >= 2 {
            let len = self.error_buffer.len();
            self.error_buffer[len - 1] - self.error_buffer[len - 2]
        } else {
            0.0
        };

        // PID formula
        self.params.k_p * error
            + self.params.k_i * integral * self.dt
            + self.params.k_d * derivative / self.dt
    }
}

/// PID controller for lateral control (steering).
///
/// Controls vehicle steering by computing the appropriate steering angle
/// based on the cross-track error (lateral deviation from target waypoint).
///
/// # Examples
/// ```
/// use carla::agents::navigation::pid::{PIDLateralController, PIDParams};
///
/// let params = PIDParams {
///     k_p: 1.95,
///     k_i: 1.4,
///     k_d: 0.01,
/// };
/// let mut controller = PIDLateralController::new(params, 1.0 / 20.0);
///
/// // Control loop
/// let waypoint_x = 10.0;
/// let waypoint_y = 0.0;
/// let vehicle_x = 9.5;
/// let vehicle_y = 0.5;
/// let vehicle_yaw = 0.0; // degrees
///
/// let steering = controller.run_step(waypoint_x, waypoint_y, vehicle_x, vehicle_y, vehicle_yaw);
/// ```
#[derive(Debug)]
pub struct PIDLateralController {
    params: PIDParams,
    dt: f32,
    error_buffer: VecDeque<f32>,
    max_buffer_size: usize,
}

impl PIDLateralController {
    /// Creates a new lateral PID controller.
    ///
    /// # Arguments
    /// * `params` - PID parameters (K_P, K_I, K_D)
    /// * `dt` - Simulation time step in seconds
    pub fn new(params: PIDParams, dt: f32) -> Self {
        Self {
            params,
            dt,
            error_buffer: VecDeque::with_capacity(10),
            max_buffer_size: 10,
        }
    }

    /// Updates PID parameters.
    pub fn change_parameters(&mut self, params: PIDParams) {
        self.params = params;
    }

    /// Executes one step of the controller.
    ///
    /// # Arguments
    /// * `waypoint_x` - Target waypoint X coordinate
    /// * `waypoint_y` - Target waypoint Y coordinate
    /// * `vehicle_x` - Vehicle X coordinate
    /// * `vehicle_y` - Vehicle Y coordinate
    /// * `vehicle_yaw` - Vehicle yaw angle in degrees
    ///
    /// # Returns
    /// Steering command in range [-1.0, 1.0]
    pub fn run_step(
        &mut self,
        waypoint_x: f32,
        waypoint_y: f32,
        vehicle_x: f32,
        vehicle_y: f32,
        vehicle_yaw: f32,
    ) -> f32 {
        use std::f32::consts::PI;

        // Convert yaw to radians
        let yaw_rad = vehicle_yaw * PI / 180.0;

        // Compute forward vector
        let forward_x = yaw_rad.cos();
        let forward_y = yaw_rad.sin();

        // Compute vector to waypoint
        let waypoint_vec_x = waypoint_x - vehicle_x;
        let waypoint_vec_y = waypoint_y - vehicle_y;

        // Compute angle error (matches Python implementation)
        // Python uses: _dot = math.acos(np.clip(np.dot(w_vec, v_vec) / (wv_linalg), -1.0, 1.0))
        let waypoint_vec_len =
            (waypoint_vec_x * waypoint_vec_x + waypoint_vec_y * waypoint_vec_y).sqrt();
        let forward_vec_len = (forward_x * forward_x + forward_y * forward_y).sqrt();

        let wv_linalg = waypoint_vec_len * forward_vec_len;
        let error = if wv_linalg < 1e-6 {
            // If vectors are too small, no steering needed
            0.0
        } else {
            // Compute dot product and normalize
            let dot_product = waypoint_vec_x * forward_x + waypoint_vec_y * forward_y;
            let normalized_dot = (dot_product / wv_linalg).clamp(-1.0, 1.0);

            // Compute angle in radians
            let mut angle = normalized_dot.acos();

            // Use cross product to determine sign (left vs right)
            // cross = v_vec × w_vec, check z component
            let cross_z = forward_x * waypoint_vec_y - forward_y * waypoint_vec_x;
            if cross_z < 0.0 {
                angle *= -1.0;
            }

            angle
        };

        // Add error to buffer
        self.error_buffer.push_back(error);
        if self.error_buffer.len() > self.max_buffer_size {
            self.error_buffer.pop_front();
        }

        // Compute integral (sum of errors)
        let integral: f32 = self.error_buffer.iter().sum();

        // Compute derivative (change in error)
        let derivative = if self.error_buffer.len() >= 2 {
            let len = self.error_buffer.len();
            self.error_buffer[len - 1] - self.error_buffer[len - 2]
        } else {
            0.0
        };

        // PID formula
        let steering = self.params.k_p * error
            + self.params.k_i * integral * self.dt
            + self.params.k_d * derivative / self.dt;

        // Clamp to [-1.0, 1.0]
        steering.clamp(-1.0, 1.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_longitudinal_controller_proportional() {
        let params = PIDParams {
            k_p: 1.0,
            k_i: 0.0,
            k_d: 0.0,
        };
        let mut controller = PIDLongitudinalController::new(params, 0.05);

        // Target 30 km/h, current 20 km/h -> error = 10
        let acceleration = controller.run_step(30.0, 20.0);
        assert!((acceleration - 10.0).abs() < 0.001);

        // Target 20 km/h, current 30 km/h -> error = -10
        let acceleration = controller.run_step(20.0, 30.0);
        assert!((acceleration + 10.0).abs() < 0.001);
    }

    #[test]
    fn test_longitudinal_controller_integral() {
        let params = PIDParams {
            k_p: 0.0,
            k_i: 1.0,
            k_d: 0.0,
        };
        let dt = 0.05;
        let mut controller = PIDLongitudinalController::new(params, dt);

        // First step: error = 10
        let accel1 = controller.run_step(30.0, 20.0);
        assert!((accel1 - 10.0 * dt).abs() < 0.001);

        // Second step: error = 10, integral accumulates
        let accel2 = controller.run_step(30.0, 20.0);
        assert!((accel2 - 20.0 * dt).abs() < 0.001);
    }

    #[test]
    fn test_lateral_controller_aligned() {
        let params = PIDParams {
            k_p: 1.0,
            k_i: 0.0,
            k_d: 0.0,
        };
        let mut controller = PIDLateralController::new(params, 0.05);

        // Vehicle at (0, 0) facing east (0°), waypoint at (10, 0)
        // Should have zero cross-track error
        let steering = controller.run_step(10.0, 0.0, 0.0, 0.0, 0.0);
        assert!(steering.abs() < 0.001);
    }

    #[test]
    fn test_lateral_controller_left_deviation() {
        let params = PIDParams {
            k_p: 1.0,
            k_i: 0.0,
            k_d: 0.0,
        };
        let mut controller = PIDLateralController::new(params, 0.05);

        // Vehicle at (0, 0) facing east (0°), waypoint at (10, 5)
        // Waypoint is to the left, should steer left (positive)
        let steering = controller.run_step(10.0, 5.0, 0.0, 0.0, 0.0);
        assert!(steering > 0.0);
    }

    #[test]
    fn test_lateral_controller_right_deviation() {
        let params = PIDParams {
            k_p: 1.0,
            k_i: 0.0,
            k_d: 0.0,
        };
        let mut controller = PIDLateralController::new(params, 0.05);

        // Vehicle at (0, 0) facing east (0°), waypoint at (10, -5)
        // Waypoint is to the right, should steer right (negative)
        let steering = controller.run_step(10.0, -5.0, 0.0, 0.0, 0.0);
        assert!(steering < 0.0);
    }

    #[test]
    fn test_lateral_controller_clamping() {
        let params = PIDParams {
            k_p: 100.0, // Very high gain to force clamping
            k_i: 0.0,
            k_d: 0.0,
        };
        let mut controller = PIDLateralController::new(params, 0.05);

        // Large deviation should clamp output
        let steering = controller.run_step(10.0, 100.0, 0.0, 0.0, 0.0);
        assert!((steering - 1.0).abs() < 0.001); // Should be clamped to 1.0

        let steering = controller.run_step(10.0, -100.0, 0.0, 0.0, 0.0);
        assert!((steering + 1.0).abs() < 0.001); // Should be clamped to -1.0
    }

    #[test]
    fn test_change_parameters() {
        let params1 = PIDParams {
            k_p: 1.0,
            k_i: 0.0,
            k_d: 0.0,
        };
        let mut controller = PIDLongitudinalController::new(params1, 0.05);

        let accel1 = controller.run_step(30.0, 20.0);
        assert!((accel1 - 10.0).abs() < 0.001);

        // Change parameters
        let params2 = PIDParams {
            k_p: 2.0,
            k_i: 0.0,
            k_d: 0.0,
        };
        controller.change_parameters(params2);

        // Clear buffer for clean test
        controller.error_buffer.clear();
        let accel2 = controller.run_step(30.0, 20.0);
        assert!((accel2 - 20.0).abs() < 0.001); // Should be doubled
    }
}
