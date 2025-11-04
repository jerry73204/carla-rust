use crate::{
    geom::{FfiVector2D, Location, Vector2D},
    utils::IteratorExt,
};
use autocxx::prelude::*;
use carla_sys::{
    carla::rpc::{GearPhysicsControl, WheelPhysicsControl},
    carla_rust::rpc::FfiVehiclePhysicsControl,
};
use nalgebra::Vector2;

#[derive(Debug, Clone)]
pub struct VehiclePhysicsControl {
    pub torque_curve: Vec<Vector2<f32>>,
    pub max_rpm: f32,
    pub moi: f32,
    pub damping_rate_full_throttle: f32,
    pub damping_rate_zero_throttle_clutch_engaged: f32,
    pub damping_rate_zero_throttle_clutch_disengaged: f32,
    pub use_gear_autobox: bool,
    pub gear_switch_time: f32,
    pub clutch_strength: f32,
    pub final_ratio: f32,
    pub forward_gears: Vec<GearPhysicsControl>,
    pub mass: f32,
    pub drag_coefficient: f32,
    pub center_of_mass: Location,
    pub steering_curve: Vec<Vector2<f32>>,
    pub wheels: Vec<WheelPhysicsControl>,
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
        // We need to collect native Vector2<f32> into CxxVector<carla::geom::Vector2D>
        let torque_curve = torque_curve
            .iter()
            .map(|v| unsafe {
                let ffi_vec = Vector2D::from_na(v);
                // Convert FfiVector2D to carla::geom::Vector2D by reinterpret cast
                std::mem::transmute::<crate::geom::FfiVector2D, carla_sys::carla::geom::Vector2D>(
                    ffi_vec.into_ffi(),
                )
            })
            .collect_cxx_vector();
        let steering_curve = steering_curve
            .iter()
            .map(|v| unsafe {
                let ffi_vec = Vector2D::from_na(v);
                std::mem::transmute::<crate::geom::FfiVector2D, carla_sys::carla::geom::Vector2D>(
                    ffi_vec.into_ffi(),
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
                    .to_na()
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
                    .to_na()
                })
                .collect(),
            wheels: from.wheels().iter().cloned().collect(),
            use_sweep_wheel_collision: from.use_sweep_wheel_collision(),
        }
    }
}
