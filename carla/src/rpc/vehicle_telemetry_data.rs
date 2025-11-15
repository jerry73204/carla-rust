/// Telemetry data for a single vehicle wheel.
///
/// **Available in CARLA 0.9.16+ only**
///
/// Provides detailed physics information about a wheel including friction, slip, forces, and torque.
/// This data is useful for advanced vehicle dynamics analysis and control.
///
/// Corresponds to [`carla.WheelTelemetryData`](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.WheelTelemetryData) in the Python API.
#[cfg(carla_0916)]
#[derive(Debug, Clone, PartialEq)]
pub struct WheelTelemetryData {
    /// Tire friction coefficient
    pub tire_friction: f32,
    /// Lateral slip angle
    pub lat_slip: f32,
    /// Longitudinal slip ratio
    pub long_slip: f32,
    /// Wheel angular velocity (rad/s)
    pub omega: f32,
    /// Tire load (vertical force on tire)
    pub tire_load: f32,
    /// Normalized tire load (0-1 range)
    pub normalized_tire_load: f32,
    /// Torque applied to wheel
    pub torque: f32,
    /// Longitudinal force on tire
    pub long_force: f32,
    /// Lateral force on tire
    pub lat_force: f32,
    /// Normalized longitudinal force
    pub normalized_long_force: f32,
    /// Normalized lateral force
    pub normalized_lat_force: f32,
}

#[cfg(carla_0916)]
impl From<carla_sys::carla::rpc::WheelTelemetryData> for WheelTelemetryData {
    fn from(value: carla_sys::carla::rpc::WheelTelemetryData) -> Self {
        Self {
            tire_friction: value.tire_friction,
            lat_slip: value.lat_slip,
            long_slip: value.long_slip,
            omega: value.omega,
            tire_load: value.tire_load,
            normalized_tire_load: value.normalized_tire_load,
            torque: value.torque,
            long_force: value.long_force,
            lat_force: value.lat_force,
            normalized_long_force: value.normalized_long_force,
            normalized_lat_force: value.normalized_lat_force,
        }
    }
}

/// Complete vehicle telemetry data including all wheels.
///
/// **Available in CARLA 0.9.16+ only**
///
/// Provides detailed physics and control state information for a vehicle, including
/// speed, control inputs (steering, throttle, brake), engine state, and per-wheel
/// telemetry. This is useful for analyzing vehicle behavior, tuning physics parameters,
/// and implementing advanced control systems.
///
/// Corresponds to [`carla.VehicleTelemetryData`](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.VehicleTelemetryData) in the Python API.
///
/// # Examples
///
/// ```no_run
/// use carla::client::{ActorBase, Client};
///
/// let client = Client::default();
/// let world = client.world();
///
/// # let bp_lib = world.blueprint_library();
/// # let vehicle_bp = bp_lib.filter("vehicle.tesla.model3").get(0).unwrap();
/// # let spawn_points = world.map().recommended_spawn_points();
/// # let actor = world.spawn_actor(&vehicle_bp, spawn_points.get(0).unwrap()).unwrap();
/// let vehicle: carla::client::Vehicle = actor.try_into().unwrap();
///
/// // Get telemetry data
/// let telemetry = vehicle.telemetry_data();
/// println!("Speed: {} m/s", telemetry.speed);
/// println!("Engine RPM: {}", telemetry.engine_rpm);
/// println!("Current gear: {}", telemetry.gear);
///
/// // Check wheel data
/// for (i, wheel) in telemetry.wheels.iter().enumerate() {
///     println!(
///         "Wheel {}: slip={:.2}, torque={:.2}",
///         i, wheel.long_slip, wheel.torque
///     );
/// }
/// ```
#[cfg(carla_0916)]
#[derive(Debug, Clone, PartialEq)]
pub struct VehicleTelemetryData {
    /// Vehicle speed (m/s)
    pub speed: f32,
    /// Steering angle (-1 to 1)
    pub steer: f32,
    /// Throttle position (0 to 1)
    pub throttle: f32,
    /// Brake position (0 to 1)
    pub brake: f32,
    /// Engine RPM
    pub engine_rpm: f32,
    /// Current gear (-1=reverse, 0=neutral, 1+=forward)
    pub gear: i32,
    /// Aerodynamic drag force
    pub drag: f32,
    /// Per-wheel telemetry data
    pub wheels: Vec<WheelTelemetryData>,
}

#[cfg(carla_0916)]
impl From<&carla_sys::carla_rust::rpc::FfiVehicleTelemetryData> for VehicleTelemetryData {
    fn from(value: &carla_sys::carla_rust::rpc::FfiVehicleTelemetryData) -> Self {
        // Get wheels via wheels() method
        let wheels_ref = value.wheels();
        let mut wheels_vec = Vec::new();

        // Iterate through C++ vector
        for wheel in wheels_ref.iter() {
            wheels_vec.push(WheelTelemetryData {
                tire_friction: wheel.tire_friction,
                lat_slip: wheel.lat_slip,
                long_slip: wheel.long_slip,
                omega: wheel.omega,
                tire_load: wheel.tire_load,
                normalized_tire_load: wheel.normalized_tire_load,
                torque: wheel.torque,
                long_force: wheel.long_force,
                lat_force: wheel.lat_force,
                normalized_long_force: wheel.normalized_long_force,
                normalized_lat_force: wheel.normalized_lat_force,
            });
        }

        Self {
            speed: value.speed(),
            steer: value.steer(),
            throttle: value.throttle(),
            brake: value.brake(),
            engine_rpm: value.engine_rpm(),
            gear: value.gear(),
            drag: value.drag(),
            wheels: wheels_vec,
        }
    }
}
