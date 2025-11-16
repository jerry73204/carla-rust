use crate::{
    geom::{FfiVector2D, Location, Vector2D},
    utils::IteratorExt,
};
use autocxx::prelude::*;
use carla_sys::{
    carla::rpc::{GearPhysicsControl, WheelPhysicsControl},
    carla_rust::rpc::FfiVehiclePhysicsControl,
};

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
    /// Moment of inertia of the engine (kg·m²)
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
