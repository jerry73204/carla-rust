use crate::geom::{FfiVector2D, Location, Vector2D};
use autocxx::prelude::*;
use carla_sys::carla_rust::rpc::FfiVehiclePhysicsControl;

#[cfg(carla_version_0100)]
use crate::{geom::Vector3D, rpc::WheelPhysicsControl, utils::IteratorExt};

#[cfg(not(carla_version_0100))]
use crate::utils::IteratorExt;

#[cfg(not(carla_version_0100))]
use carla_sys::carla::rpc::{GearPhysicsControl, WheelPhysicsControl};

// =============================================================================
// CARLA 0.10.0 VehiclePhysicsControl
// =============================================================================

#[cfg(carla_version_0100)]
/// Vehicle physics control parameters.
///
/// This type contains all the physical parameters that define how a vehicle behaves,
/// including engine characteristics, transmission, mass properties, aerodynamics,
/// and wheel physics. These parameters can be queried from a vehicle and modified
/// to create custom vehicle physics.
///
/// Corresponds to `carla.VehiclePhysicsControl` in the Python API.
#[derive(Debug, Clone)]
pub struct VehiclePhysicsControl {
    /// Engine torque curve as pairs of (RPM, Torque in Nm)
    pub torque_curve: Vec<Vector2D>,
    /// Maximum engine torque (Nm)
    pub max_torque: f32,
    /// Maximum RPM of the engine
    pub max_rpm: f32,
    /// Idle RPM of the engine
    pub idle_rpm: f32,
    /// Brake effect multiplier
    pub brake_effect: f32,
    /// Moment of inertia for rev up
    pub rev_up_moi: f32,
    /// Rev down rate
    pub rev_down_rate: f32,
    /// Differential type (0 = undefined)
    pub differential_type: u8,
    /// Front/rear torque split (0.0 = rear, 1.0 = front, 0.5 = equal)
    pub front_rear_split: f32,
    /// If true, the vehicle will have automatic gear changes
    pub use_automatic_gears: bool,
    /// Time it takes to change gears (seconds)
    pub gear_change_time: f32,
    /// Final drive ratio
    pub final_ratio: f32,
    /// List of forward gear ratios
    pub forward_gear_ratios: Vec<f32>,
    /// List of reverse gear ratios
    pub reverse_gear_ratios: Vec<f32>,
    /// RPM at which upshift occurs
    pub change_up_rpm: f32,
    /// RPM at which downshift occurs
    pub change_down_rpm: f32,
    /// Transmission efficiency (0.0 to 1.0)
    pub transmission_efficiency: f32,
    /// Vehicle mass in kilograms
    pub mass: f32,
    /// Drag coefficient (lower = more aerodynamic)
    pub drag_coefficient: f32,
    /// Center of mass location relative to vehicle origin
    pub center_of_mass: Location,
    /// Chassis width in cm
    pub chassis_width: f32,
    /// Chassis height in cm
    pub chassis_height: f32,
    /// Downforce coefficient
    pub downforce_coefficient: f32,
    /// Drag area
    pub drag_area: f32,
    /// Inertia tensor scale
    pub inertia_tensor_scale: Vector3D,
    /// Sleep threshold
    pub sleep_threshold: f32,
    /// Sleep slope limit
    pub sleep_slope_limit: f32,
    /// Steering curve as pairs of (Speed in km/h, Steering angle)
    pub steering_curve: Vec<Vector2D>,
    /// List of wheel physics parameters (one per wheel)
    pub wheels: Vec<WheelPhysicsControl>,
    /// If true, uses sweep-based wheel collision detection (more accurate but slower)
    pub use_sweep_wheel_collision: bool,
}

#[cfg(carla_version_0100)]
impl VehiclePhysicsControl {
    pub fn to_cxx(&self) -> UniquePtr<FfiVehiclePhysicsControl> {
        let torque_curve = self
            .torque_curve
            .iter()
            .map(|v| unsafe {
                std::mem::transmute::<crate::geom::FfiVector2D, carla_sys::carla::geom::Vector2D>(
                    v.into_ffi(),
                )
            })
            .collect_cxx_vector();
        let steering_curve = self
            .steering_curve
            .iter()
            .map(|v| unsafe {
                std::mem::transmute::<crate::geom::FfiVector2D, carla_sys::carla::geom::Vector2D>(
                    v.into_ffi(),
                )
            })
            .collect_cxx_vector();
        let forward_gear_ratios = self
            .forward_gear_ratios
            .iter()
            .cloned()
            .collect_cxx_vector();
        let reverse_gear_ratios = self
            .reverse_gear_ratios
            .iter()
            .cloned()
            .collect_cxx_vector();

        let mut ctrl = FfiVehiclePhysicsControl::new().within_unique_ptr();
        ctrl.pin_mut().set_torque_curve(&torque_curve);
        ctrl.pin_mut().set_max_torque(self.max_torque);
        ctrl.pin_mut().set_max_rpm(self.max_rpm);
        ctrl.pin_mut().set_idle_rpm(self.idle_rpm);
        ctrl.pin_mut().set_brake_effect(self.brake_effect);
        ctrl.pin_mut().set_rev_up_moi(self.rev_up_moi);
        ctrl.pin_mut().set_rev_down_rate(self.rev_down_rate);
        ctrl.pin_mut().set_differential_type(self.differential_type);
        ctrl.pin_mut().set_front_rear_split(self.front_rear_split);
        ctrl.pin_mut()
            .set_use_automatic_gears(self.use_automatic_gears);
        ctrl.pin_mut().set_gear_change_time(self.gear_change_time);
        ctrl.pin_mut().set_final_ratio(self.final_ratio);
        ctrl.pin_mut().set_forward_gear_ratios(&forward_gear_ratios);
        ctrl.pin_mut().set_reverse_gear_ratios(&reverse_gear_ratios);
        ctrl.pin_mut().set_change_up_rpm(self.change_up_rpm);
        ctrl.pin_mut().set_change_down_rpm(self.change_down_rpm);
        ctrl.pin_mut()
            .set_transmission_efficiency(self.transmission_efficiency);
        ctrl.pin_mut().set_mass(self.mass);
        ctrl.pin_mut().set_drag_coefficient(self.drag_coefficient);
        ctrl.pin_mut()
            .set_center_of_mass(&self.center_of_mass.into_ffi());
        ctrl.pin_mut().set_chassis_width(self.chassis_width);
        ctrl.pin_mut().set_chassis_height(self.chassis_height);
        ctrl.pin_mut()
            .set_downforce_coefficient(self.downforce_coefficient);
        ctrl.pin_mut().set_drag_area(self.drag_area);
        ctrl.pin_mut()
            .set_inertia_tensor_scale(&self.inertia_tensor_scale.into_ffi());
        ctrl.pin_mut().set_sleep_threshold(self.sleep_threshold);
        ctrl.pin_mut().set_sleep_slope_limit(self.sleep_slope_limit);
        ctrl.pin_mut().set_steering_curve(&steering_curve);
        ctrl.pin_mut()
            .set_use_sweep_wheel_collision(self.use_sweep_wheel_collision);

        // Add wheels
        ctrl.pin_mut().clear_wheels();
        for wheel in &self.wheels {
            let wheel_cxx = wheel.to_cxx();
            ctrl.pin_mut().push_wheel(&wheel_cxx);
        }

        ctrl
    }

    pub fn from_cxx(from: &FfiVehiclePhysicsControl) -> Self {
        let convert_vec2d = |v: &carla_sys::carla::geom::Vector2D| unsafe {
            Vector2D::from_ffi(std::mem::transmute::<
                carla_sys::carla::geom::Vector2D,
                FfiVector2D,
            >(v.clone()))
        };

        let num_wheels = from.wheels_size();
        let wheels: Vec<WheelPhysicsControl> = (0..num_wheels)
            .map(|i| {
                let ffi_wheel = from.wheel_at(i);
                WheelPhysicsControl::from_cxx(&ffi_wheel)
            })
            .collect();

        Self {
            torque_curve: from.torque_curve().iter().map(convert_vec2d).collect(),
            max_torque: from.max_torque(),
            max_rpm: from.max_rpm(),
            idle_rpm: from.idle_rpm(),
            brake_effect: from.brake_effect(),
            rev_up_moi: from.rev_up_moi(),
            rev_down_rate: from.rev_down_rate(),
            differential_type: from.differential_type(),
            front_rear_split: from.front_rear_split(),
            use_automatic_gears: from.use_automatic_gears(),
            gear_change_time: from.gear_change_time(),
            final_ratio: from.final_ratio(),
            forward_gear_ratios: from.forward_gear_ratios().iter().cloned().collect(),
            reverse_gear_ratios: from.reverse_gear_ratios().iter().cloned().collect(),
            change_up_rpm: from.change_up_rpm(),
            change_down_rpm: from.change_down_rpm(),
            transmission_efficiency: from.transmission_efficiency(),
            mass: from.mass(),
            drag_coefficient: from.drag_coefficient(),
            center_of_mass: Location::from_ffi(from.center_of_mass().clone()),
            chassis_width: from.chassis_width(),
            chassis_height: from.chassis_height(),
            downforce_coefficient: from.downforce_coefficient(),
            drag_area: from.drag_area(),
            inertia_tensor_scale: Vector3D::from_ffi(from.inertia_tensor_scale()),
            sleep_threshold: from.sleep_threshold(),
            sleep_slope_limit: from.sleep_slope_limit(),
            steering_curve: from.steering_curve().iter().map(convert_vec2d).collect(),
            wheels,
            use_sweep_wheel_collision: from.use_sweep_wheel_collision(),
        }
    }
}

// =============================================================================
// CARLA 0.9.x VehiclePhysicsControl
// =============================================================================

#[cfg(not(carla_version_0100))]
/// Vehicle physics control parameters.
///
/// This type contains all the physical parameters that define how a vehicle behaves,
/// including engine characteristics, transmission, mass properties, aerodynamics,
/// and wheel physics. These parameters can be queried from a vehicle and modified
/// to create custom vehicle physics.
///
/// Corresponds to [`carla.VehiclePhysicsControl`] in the Python API.
#[cfg_attr(carla_version_0916, doc = "")]
#[cfg_attr(
    carla_version_0916,
    doc = " [`carla.VehiclePhysicsControl`]: https://carla.readthedocs.io/en/0.9.16/python_api/#carla.VehiclePhysicsControl"
)]
#[cfg_attr(carla_version_0915, doc = "")]
#[cfg_attr(
    carla_version_0915,
    doc = " [`carla.VehiclePhysicsControl`]: https://carla.readthedocs.io/en/0.9.15/python_api/#carla.VehiclePhysicsControl"
)]
#[cfg_attr(carla_version_0914, doc = "")]
#[cfg_attr(
    carla_version_0914,
    doc = " [`carla.VehiclePhysicsControl`]: https://carla.readthedocs.io/en/0.9.14/python_api/#carla.VehiclePhysicsControl"
)]
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
/// // Get current physics control
/// let mut physics = vehicle.physics_control();
///
/// // Modify physics parameters
/// physics.mass = 1500.0; // Set mass to 1500 kg
/// physics.drag_coefficient = 0.3; // Reduce drag
/// physics.max_rpm = 6000.0; // Increase max RPM
///
/// // Apply modified physics
/// vehicle.apply_physics_control(&physics);
/// ```
#[derive(Debug, Clone)]
pub struct VehiclePhysicsControl {
    /// Engine torque curve as pairs of (RPM, Torque in Nm)
    pub torque_curve: Vec<Vector2D>,
    /// Maximum RPM of the engine
    pub max_rpm: f32,
    /// Moment of inertia of the engine (kg-m^2)
    pub moi: f32,
    /// Damping rate when throttle is at maximum
    pub damping_rate_full_throttle: f32,
    /// Damping rate with no throttle and clutch engaged
    pub damping_rate_zero_throttle_clutch_engaged: f32,
    /// Damping rate with no throttle and clutch disengaged
    pub damping_rate_zero_throttle_clutch_disengaged: f32,
    /// If true, the vehicle will have automatic transmission
    pub use_gear_autobox: bool,
    /// Time it takes to switch gears (seconds)
    pub gear_switch_time: f32,
    /// Clutch strength (higher = faster gear changes)
    pub clutch_strength: f32,
    /// Final drive ratio
    pub final_ratio: f32,
    /// List of gear ratios for forward gears
    pub forward_gears: Vec<GearPhysicsControl>,
    /// Vehicle mass in kilograms
    pub mass: f32,
    /// Drag coefficient (lower = more aerodynamic)
    pub drag_coefficient: f32,
    /// Center of mass location relative to vehicle origin
    pub center_of_mass: Location,
    /// Steering curve as pairs of (Speed in km/h, Steering angle)
    pub steering_curve: Vec<Vector2D>,
    /// List of wheel physics parameters (one per wheel)
    pub wheels: Vec<WheelPhysicsControl>,
    /// If true, uses sweep-based wheel collision detection (more accurate but slower)
    pub use_sweep_wheel_collision: bool,
}

#[cfg(not(carla_version_0100))]
impl VehiclePhysicsControl {
    pub fn to_cxx(&self) -> UniquePtr<FfiVehiclePhysicsControl> {
        let Self {
            ref torque_curve,
            max_rpm,
            moi,
            damping_rate_full_throttle,
            damping_rate_zero_throttle_clutch_engaged,
            damping_rate_zero_throttle_clutch_disengaged,
            use_gear_autobox,
            gear_switch_time,
            clutch_strength,
            final_ratio,
            ref forward_gears,
            mass,
            drag_coefficient,
            ref center_of_mass,
            ref steering_curve,
            ref wheels,
            use_sweep_wheel_collision,
        } = *self;

        // SAFETY: Our Vector2D and carla::geom::Vector2D have identical layout
        // Convert Vec<Vector2D> into CxxVector<carla::geom::Vector2D>
        let torque_curve = torque_curve
            .iter()
            .map(|v| unsafe {
                // Convert FfiVector2D to carla::geom::Vector2D by reinterpret cast
                std::mem::transmute::<crate::geom::FfiVector2D, carla_sys::carla::geom::Vector2D>(
                    v.into_ffi(),
                )
            })
            .collect_cxx_vector();
        let steering_curve = steering_curve
            .iter()
            .map(|v| unsafe {
                std::mem::transmute::<crate::geom::FfiVector2D, carla_sys::carla::geom::Vector2D>(
                    v.into_ffi(),
                )
            })
            .collect_cxx_vector();
        let mut center_of_mass = Box::pin(center_of_mass.into_ffi());
        let mut forward_gears = forward_gears.iter().cloned().collect_cxx_vector();
        let mut wheels = wheels.iter().cloned().collect_cxx_vector();

        FfiVehiclePhysicsControl::new1(
            &torque_curve,
            max_rpm,
            moi,
            damping_rate_full_throttle,
            damping_rate_zero_throttle_clutch_engaged,
            damping_rate_zero_throttle_clutch_disengaged,
            use_gear_autobox,
            gear_switch_time,
            clutch_strength,
            final_ratio,
            forward_gears.pin_mut(),
            mass,
            drag_coefficient,
            center_of_mass.as_mut(),
            &steering_curve,
            wheels.pin_mut(),
            use_sweep_wheel_collision,
        )
        .within_unique_ptr()
    }

    pub fn from_cxx(from: &FfiVehiclePhysicsControl) -> Self {
        Self {
            torque_curve: from
                .torque_curve()
                .iter()
                .map(|v| unsafe {
                    Vector2D::from_ffi(std::mem::transmute::<
                        carla_sys::carla::geom::Vector2D,
                        FfiVector2D,
                    >(v.clone()))
                })
                .collect(),
            max_rpm: from.max_rpm(),
            moi: from.moi(),
            damping_rate_full_throttle: from.damping_rate_full_throttle(),
            damping_rate_zero_throttle_clutch_engaged: from
                .damping_rate_zero_throttle_clutch_engaged(),
            damping_rate_zero_throttle_clutch_disengaged: from
                .damping_rate_zero_throttle_clutch_disengaged(),
            use_gear_autobox: from.use_gear_autobox(),
            gear_switch_time: from.gear_switch_time(),
            clutch_strength: from.clutch_strength(),
            final_ratio: from.final_ratio(),
            forward_gears: from.forward_gears().iter().cloned().collect(),
            mass: from.mass(),
            drag_coefficient: from.drag_coefficient(),
            center_of_mass: Location::from_ffi(from.center_of_mass().clone()),
            steering_curve: from
                .steering_curve()
                .iter()
                .map(|v| unsafe {
                    Vector2D::from_ffi(std::mem::transmute::<
                        carla_sys::carla::geom::Vector2D,
                        FfiVector2D,
                    >(v.clone()))
                })
                .collect(),
            wheels: from.wheels().iter().cloned().collect(),
            use_sweep_wheel_collision: from.use_sweep_wheel_collision(),
        }
    }
}
