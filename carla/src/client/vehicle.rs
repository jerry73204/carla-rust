// SAFETY: This module uses unwrap_unchecked() for performance on methods guaranteed
// to never return null. See UNWRAP_REPLACEMENTS.md for detailed C++ code audit.

use super::{Actor, ActorBase};
#[cfg(carla_0916)]
use crate::rpc::VehicleTelemetryData;
use crate::rpc::{
    AckermannControllerSettings, TrafficLightState, VehicleAckermannControl, VehicleControl,
    VehicleDoor, VehicleFailureState, VehicleLightState, VehiclePhysicsControl,
    VehicleWheelLocation,
};
use autocxx::prelude::*;
use carla_sys::{
    carla::traffic_manager::constants::Networking::TM_DEFAULT_PORT,
    carla_rust::client::{FfiActor, FfiVehicle},
};
use cxx::SharedPtr;
use derivative::Derivative;
use static_assertions::assert_impl_all;

/// Represents a vehicle in the simulation.
///
/// [`Vehicle`] provides methods for:
/// - Vehicle control (throttle, steering, braking)
/// - Autopilot configuration
/// - Physics parameter tuning
/// - Light and door control
/// - Ackermann steering configuration
///
/// Corresponds to [`carla.Vehicle`](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.Vehicle) in the Python API
///
/// # Examples
///
/// ```no_run
/// use carla::client::Client;
///
/// let client = Client::default();
/// let mut world = client.world();
///
/// # let bp_lib = world.blueprint_library();
/// # let vehicle_bp = bp_lib.filter("vehicle.*").get(0).unwrap();
/// # let spawn_points = world.map().recommended_spawn_points();
/// # let actor = world.spawn_actor(&vehicle_bp, &spawn_points.get(0).unwrap()).unwrap();
/// let vehicle: carla::client::Vehicle = actor.try_into().unwrap();
///
/// // Control the vehicle
/// let mut control = vehicle.control();
/// control.throttle = 0.7;
/// control.steer = -0.3;
/// vehicle.apply_control(&control);
///
/// // Enable autopilot
/// vehicle.set_autopilot(true);
/// ```
#[derive(Clone, Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct Vehicle {
    #[derivative(Debug = "ignore")]
    inner: SharedPtr<FfiVehicle>,
}

impl Vehicle {
    /// Enables or disables autopilot using the default Traffic Manager port.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// # use carla::client::Client;
    /// # let client = Client::default();
    /// # let mut world = client.world();
    /// # let bp_lib = world.blueprint_library();
    /// # let vehicle_bp = bp_lib.filter("vehicle.*").get(0).unwrap();
    /// # let spawn_points = world.map().recommended_spawn_points();
    /// # let actor = world.spawn_actor(&vehicle_bp, &spawn_points.get(0).unwrap()).unwrap();
    /// # let vehicle: carla::client::Vehicle = actor.try_into().unwrap();
    /// vehicle.set_autopilot(true);
    /// ```
    pub fn set_autopilot(&self, enabled: bool) {
        self.set_autopilot_opt(enabled, TM_DEFAULT_PORT)
    }

    /// Enables or disables autopilot with a specific Traffic Manager port.
    pub fn set_autopilot_opt(&self, enabled: bool, tm_port: u16) {
        self.inner.SetAutopilot(enabled, tm_port);
    }

    /// Enables or disables debug telemetry display.
    pub fn show_debug_telemetry(&self, enabled: bool) {
        self.inner.ShowDebugTelemetry(enabled);
    }

    /// Applies vehicle control (throttle, steering, braking, etc.).
    ///
    /// # Examples
    ///
    /// ```no_run
    /// # use carla::client::Client;
    /// # let client = Client::default();
    /// # let mut world = client.world();
    /// # let bp_lib = world.blueprint_library();
    /// # let vehicle_bp = bp_lib.filter("vehicle.*").get(0).unwrap();
    /// # let spawn_points = world.map().recommended_spawn_points();
    /// # let actor = world.spawn_actor(&vehicle_bp, &spawn_points.get(0).unwrap()).unwrap();
    /// # let vehicle: carla::client::Vehicle = actor.try_into().unwrap();
    /// let mut control = vehicle.control();
    /// control.throttle = 1.0; // Full throttle
    /// control.steer = 0.5; // Steer right
    /// control.brake = 0.0; // No braking
    /// vehicle.apply_control(&control);
    /// ```
    pub fn apply_control(&self, control: &VehicleControl) {
        self.inner.ApplyControl(control);
    }

    /// Returns the current vehicle control state.
    pub fn control(&self) -> VehicleControl {
        self.inner.GetControl()
    }

    /// Applies physics control parameters (mass, drag, torque curve, etc.).
    ///
    /// Use this to tune vehicle handling characteristics.
    pub fn apply_physics_control(&self, control: &VehiclePhysicsControl) {
        let control = control.to_cxx();
        self.inner.ApplyPhysicsControl(&control);
    }

    /// Returns the current physics control parameters.
    pub fn physics_control(&self) -> VehiclePhysicsControl {
        VehiclePhysicsControl::from_cxx(&self.inner.GetPhysicsControl().within_unique_ptr())
    }

    /// Applies Ackermann steering control (used for bicycle-like steering models).
    pub fn apply_ackermann_control(&self, control: &VehicleAckermannControl) {
        self.inner.ApplyAckermannControl(control);
    }

    /// Configures Ackermann controller settings.
    pub fn apply_ackermann_controller_settings(&self, settings: &AckermannControllerSettings) {
        self.inner.ApplyAckermannControllerSettings(settings)
    }

    /// Returns the current Ackermann controller settings.
    pub fn ackermann_controller_settings(&self) -> AckermannControllerSettings {
        self.inner.GetAckermannControllerSettings()
    }

    /// Opens a vehicle door.
    pub fn open_door(&self, door: VehicleDoor) {
        self.inner.OpenDoor(door);
    }

    /// Closes a vehicle door.
    pub fn close_door(&self, door: VehicleDoor) {
        self.inner.CloseDoor(door);
    }

    /// Sets the vehicle light state (headlights, brake lights, turn signals, etc.).
    pub fn set_light_state(&self, light_state: &VehicleLightState) {
        self.inner.SetLightState(light_state);
    }

    /// Sets the steering angle for a specific wheel (in degrees).
    pub fn set_wheel_steer_direction(&self, wheel_location: VehicleWheelLocation, degrees: f32) {
        self.inner.SetWheelSteerDirection(wheel_location, degrees);
    }

    /// Returns the steering angle for a specific wheel (in degrees).
    pub fn wheel_steer_angle(&self, wheel_location: VehicleWheelLocation) -> f32 {
        self.inner.GetWheelSteerAngle(wheel_location)
    }

    /// Returns the current light state.
    pub fn light_state(&self) -> VehicleLightState {
        self.inner.GetLightState()
    }

    /// Returns the state of the traffic light affecting this vehicle.
    pub fn traffic_light_state(&self) -> TrafficLightState {
        self.inner.GetTrafficLightState()
    }

    /// Returns whether the vehicle is currently at a traffic light.
    pub fn is_at_traffic_light(&self) -> bool {
        self.inner.IsAtTrafficLight()
    }

    /// Returns the vehicle's current failure state.
    ///
    /// Available since CARLA 0.9.14+
    ///
    /// # Returns
    ///
    /// The failure state (None, Rollover, Engine, or TirePuncture)
    pub fn failure_state(&self) -> VehicleFailureState {
        self.inner.GetFailureState()
    }

    /// Returns detailed vehicle telemetry data including wheel physics.
    ///
    /// **Available only in CARLA 0.9.16+**
    ///
    /// Provides comprehensive physics data including speed, steering, engine RPM,
    /// gear, drag, and per-wheel telemetry (friction, slip, forces, torque).
    ///
    /// # Examples
    ///
    /// ```no_run
    /// # use carla::client::Client;
    /// # let client = Client::default();
    /// # let mut world = client.world();
    /// # let bp_lib = world.blueprint_library();
    /// # let vehicle_bp = bp_lib.filter("vehicle.*").get(0).unwrap();
    /// # let spawn_points = world.map().recommended_spawn_points();
    /// # let actor = world.spawn_actor(&vehicle_bp, &spawn_points.get(0).unwrap()).unwrap();
    /// # let vehicle: carla::client::Vehicle = actor.try_into().unwrap();
    /// let telemetry = vehicle.telemetry_data();
    /// println!("Speed: {} m/s", telemetry.speed);
    /// println!("Engine RPM: {}", telemetry.engine_rpm);
    /// println!("Gear: {}", telemetry.gear);
    /// for (i, wheel) in telemetry.wheels.iter().enumerate() {
    ///     println!(
    ///         "Wheel {}: friction={}, slip={}",
    ///         i, wheel.tire_friction, wheel.lat_slip
    ///     );
    /// }
    /// ```
    #[cfg(carla_0916)]
    pub fn telemetry_data(&self) -> VehicleTelemetryData {
        (&*self.inner.GetTelemetryData().within_unique_ptr()).into()
    }

    /// Sets the pitch angle for a specific wheel (in degrees).
    ///
    /// **Available only in CARLA 0.9.16+**
    #[cfg(carla_0916)]
    pub fn set_wheel_pitch_angle(&self, wheel_location: VehicleWheelLocation, degrees: f32) {
        self.inner.SetWheelPitchAngle(wheel_location, degrees);
    }

    /// Returns the pitch angle for a specific wheel (in degrees).
    ///
    /// **Available only in CARLA 0.9.16+**
    #[cfg(carla_0916)]
    pub fn wheel_pitch_angle(&self, wheel_location: VehicleWheelLocation) -> f32 {
        self.inner.GetWheelPitchAngle(wheel_location)
    }

    // TODO: Implement vehicle_bone_world_transforms() when autocxx can handle std::vector<Transform>
    // /// Returns world transforms for all vehicle bones.
    // ///
    // /// **Available only in CARLA 0.9.16+**
    // ///
    // /// Useful for advanced vehicle visualization and skeleton tracking.
    // #[cfg(carla_0916)]
    // pub fn vehicle_bone_world_transforms(&self) -> Vec<crate::geom::Transform> {
    //     use crate::geom::{Location, Rotation, Transform};
    //
    //     let transforms_vec = self.inner.GetVehicleBoneWorldTransforms();
    //     let mut result = Vec::new();
    //
    //     unsafe {
    //         for i in 0..transforms_vec.size() {
    //             let t = transforms_vec.index(i);
    //             result.push(Transform {
    //                 location: Location {
    //                     x: t.location.x,
    //                     y: t.location.y,
    //                     z: t.location.z,
    //                 },
    //                 rotation: Rotation {
    //                     pitch: t.rotation.pitch,
    //                     yaw: t.rotation.yaw,
    //                     roll: t.rotation.roll,
    //                 },
    //             });
    //         }
    //     }
    //
    //     result
    // }

    /// Restores PhysX physics simulation (after using Chrono or CarSim).
    ///
    /// **Available only in CARLA 0.9.16+**
    #[cfg(carla_0916)]
    pub fn restore_phys_x_physics(&self) {
        self.inner.RestorePhysXPhysics();
    }

    /// Enables CarSim physics simulation with the given configuration file.
    pub fn enable_car_sim(&self, simfile_path: &str) {
        self.inner.EnableCarSim(simfile_path)
    }

    /// Enables or disables using the CarSim road for physics.
    pub fn use_car_sim_road(&self, enabled: bool) {
        self.inner.UseCarSimRoad(enabled);
    }

    /// Enables Chrono physics engine for high-fidelity vehicle simulation.
    ///
    /// Chrono provides more accurate tire and suspension modeling than the default physics.
    pub fn enable_chrono_physics(
        &self,
        max_substeps: u64,
        max_substep_delta_time: f32,
        vehicle_json: &str,
        powertrain_json: &str,
        tire_json: &str,
        base_json_path: &str,
    ) {
        self.inner.EnableChronoPhysics(
            max_substeps,
            max_substep_delta_time,
            vehicle_json,
            powertrain_json,
            tire_json,
            base_json_path,
        );
    }

    /// Converts this vehicle into a generic [`Actor`].
    pub fn into_actor(self) -> Actor {
        let ptr = self.inner.to_actor();
        unsafe { Actor::from_cxx(ptr).unwrap_unchecked() }
    }

    pub(crate) fn from_cxx(ptr: SharedPtr<FfiVehicle>) -> Option<Self> {
        if ptr.is_null() {
            None
        } else {
            Some(Self { inner: ptr })
        }
    }
}

impl ActorBase for Vehicle {
    fn cxx_actor(&self) -> SharedPtr<FfiActor> {
        self.inner.to_actor()
    }
}

impl TryFrom<Actor> for Vehicle {
    type Error = Actor;

    fn try_from(value: Actor) -> Result<Self, Self::Error> {
        let ptr = value.inner.to_vehicle();
        Self::from_cxx(ptr).ok_or(value)
    }
}

assert_impl_all!(Vehicle: Send, Sync);
