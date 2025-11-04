//! High-level vehicle PID controller combining lateral and longitudinal control.

use super::pid::{PIDLateralController, PIDLongitudinalController, PIDParams};
use crate::{agents::tools::get_speed, client::ActorBase, geom::Location, rpc::VehicleControl};

/// Combined PID controller for vehicle control.
///
/// Manages both lateral (steering) and longitudinal (speed) control using
/// separate PID controllers. Implements smooth steering changes and proper
/// throttle/brake selection based on acceleration sign.
///
/// # Examples
/// ```no_run
/// use carla::{
///     agents::navigation::{controller::VehiclePIDController, pid::PIDParams},
///     client::Vehicle,
///     geom::Location,
/// };
///
/// # fn example(vehicle: &Vehicle) -> anyhow::Result<()> {
/// let lateral_params = PIDParams {
///     k_p: 1.95,
///     k_i: 1.4,
///     k_d: 0.01,
/// };
/// let longitudinal_params = PIDParams {
///     k_p: 1.0,
///     k_i: 0.0,
///     k_d: 0.0,
/// };
///
/// let mut controller = VehiclePIDController::new(
///     lateral_params,
///     longitudinal_params,
///     0.0,        // offset
///     0.75,       // max_throttle
///     0.3,        // max_brake
///     0.8,        // max_steering
///     1.0 / 20.0, // dt
/// );
///
/// // Control loop
/// let target_speed = 30.0; // km/h
/// let waypoint_location = Location {
///     x: 100.0,
///     y: 50.0,
///     z: 0.3,
/// };
/// let control = controller.run_step(vehicle, target_speed, &waypoint_location)?;
/// # Ok(())
/// # }
/// ```
#[derive(Debug)]
pub struct VehiclePIDController {
    lateral_controller: PIDLateralController,
    longitudinal_controller: PIDLongitudinalController,
    offset: f32,
    max_throttle: f32,
    max_brake: f32,
    max_steering: f32,
    past_steering: f32,
}

impl VehiclePIDController {
    /// Creates a new vehicle PID controller.
    ///
    /// # Arguments
    /// * `lateral_params` - PID parameters for steering control
    /// * `longitudinal_params` - PID parameters for speed control
    /// * `offset` - Lateral offset from waypoint center (meters, positive = right)
    /// * `max_throttle` - Maximum throttle value [0.0, 1.0]
    /// * `max_brake` - Maximum brake value [0.0, 1.0]
    /// * `max_steering` - Maximum steering value [0.0, 1.0]
    /// * `dt` - Simulation time step in seconds
    pub fn new(
        lateral_params: PIDParams,
        longitudinal_params: PIDParams,
        offset: f32,
        max_throttle: f32,
        max_brake: f32,
        max_steering: f32,
        dt: f32,
    ) -> Self {
        Self {
            lateral_controller: PIDLateralController::new(lateral_params, dt),
            longitudinal_controller: PIDLongitudinalController::new(longitudinal_params, dt),
            offset,
            max_throttle,
            max_brake,
            max_steering,
            past_steering: 0.0,
        }
    }

    /// Updates the lateral PID parameters.
    pub fn change_lateral_pid(&mut self, params: PIDParams) {
        self.lateral_controller.change_parameters(params);
    }

    /// Updates the longitudinal PID parameters.
    pub fn change_longitudinal_pid(&mut self, params: PIDParams) {
        self.longitudinal_controller.change_parameters(params);
    }

    /// Sets the lateral offset from waypoint center.
    ///
    /// Positive values shift the target to the right, negative to the left.
    pub fn set_offset(&mut self, offset: f32) {
        self.offset = offset;
    }

    /// Executes one control step.
    ///
    /// Computes steering, throttle, and brake values based on the vehicle's
    /// current state and the target waypoint.
    ///
    /// # Arguments
    /// * `vehicle` - The vehicle to control
    /// * `target_speed` - Desired speed in km/h
    /// * `waypoint` - Target waypoint location
    ///
    /// # Returns
    /// VehicleControl with computed steering, throttle, and brake values
    pub fn run_step<T: ActorBase>(
        &mut self,
        vehicle: &T,
        target_speed: f32,
        waypoint: &Location,
    ) -> anyhow::Result<VehicleControl> {
        // Get vehicle state
        let vehicle_transform = vehicle.transform();
        let current_speed = get_speed(vehicle);

        // Apply lateral offset to waypoint
        let waypoint_adjusted = if self.offset.abs() > 0.001 {
            use std::f32::consts::PI;
            let yaw_rad = vehicle_transform.rotation.yaw * PI / 180.0;
            // Perpendicular vector (right direction)
            let right_x = -yaw_rad.sin();
            let right_y = yaw_rad.cos();

            Location {
                x: waypoint.x + right_x * self.offset,
                y: waypoint.y + right_y * self.offset,
                z: waypoint.z,
            }
        } else {
            *waypoint
        };

        // Compute steering
        let mut steering = self.lateral_controller.run_step(
            waypoint_adjusted.x,
            waypoint_adjusted.y,
            vehicle_transform.location.x,
            vehicle_transform.location.y,
            vehicle_transform.rotation.yaw,
        );

        // Smooth steering changes (max ±0.1 per frame)
        let steering_diff = steering - self.past_steering;
        if steering_diff.abs() > 0.1 {
            steering = self.past_steering + 0.1 * steering_diff.signum();
        }
        self.past_steering = steering;

        // Clamp steering to max_steering
        steering = steering.clamp(-self.max_steering, self.max_steering);

        // Compute acceleration
        let acceleration = self
            .longitudinal_controller
            .run_step(target_speed, current_speed);

        // Convert acceleration to throttle/brake
        let (throttle, brake) = if acceleration >= 0.0 {
            // Positive acceleration -> use throttle
            let throttle = acceleration.min(self.max_throttle);
            (throttle, 0.0)
        } else {
            // Negative acceleration -> use brake
            let brake = (-acceleration).min(self.max_brake);
            (0.0, brake)
        };

        Ok(VehicleControl {
            throttle,
            steer: steering,
            brake,
            hand_brake: false,
            reverse: false,
            manual_gear_shift: false,
            gear: 0,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_controller_creation() {
        let lateral_params = PIDParams {
            k_p: 1.95,
            k_i: 1.4,
            k_d: 0.01,
        };
        let longitudinal_params = PIDParams {
            k_p: 1.0,
            k_i: 0.0,
            k_d: 0.0,
        };

        let controller = VehiclePIDController::new(
            lateral_params,
            longitudinal_params,
            0.0,
            0.75,
            0.3,
            0.8,
            1.0 / 20.0,
        );

        assert_eq!(controller.offset, 0.0);
        assert_eq!(controller.max_throttle, 0.75);
        assert_eq!(controller.max_brake, 0.3);
        assert_eq!(controller.max_steering, 0.8);
    }

    #[test]
    fn test_parameter_changes() {
        let lateral_params = PIDParams {
            k_p: 1.0,
            k_i: 0.0,
            k_d: 0.0,
        };
        let longitudinal_params = PIDParams {
            k_p: 1.0,
            k_i: 0.0,
            k_d: 0.0,
        };

        let mut controller = VehiclePIDController::new(
            lateral_params,
            longitudinal_params,
            0.0,
            0.75,
            0.3,
            0.8,
            1.0 / 20.0,
        );

        // Test parameter changes
        let new_lateral = PIDParams {
            k_p: 2.0,
            k_i: 0.5,
            k_d: 0.1,
        };
        controller.change_lateral_pid(new_lateral);

        let new_longitudinal = PIDParams {
            k_p: 1.5,
            k_i: 0.0,
            k_d: 0.2,
        };
        controller.change_longitudinal_pid(new_longitudinal);

        // Test offset
        controller.set_offset(1.5);
        assert_eq!(controller.offset, 1.5);
    }
}
