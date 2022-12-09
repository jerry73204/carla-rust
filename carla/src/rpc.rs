use crate::{
    geom::{Location, LocationExt, Vector2D, Vector2DExt},
    utils::IteratorExt,
};
use autocxx::prelude::*;
use carla_sys::carla_rust::rpc::{FfiEpisodeSettings, FfiVehiclePhysicsControl};
use nalgebra::{Translation3, Vector2};

pub use carla_sys::{
    carla::rpc::{
        AttachmentType, GearPhysicsControl, OpendriveGenerationParameters, TrafficLightState,
        VehicleControl, VehicleDoor, VehicleLightState_LightState, VehicleWheelLocation,
        WheelPhysicsControl,
    },
    carla_rust::rpc::FfiActorId as ActorId,
};

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
    pub center_of_mass: Translation3<f32>,
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

        let torque_curve = torque_curve
            .iter()
            .map(Vector2D::from_na)
            .collect_cxx_vector();
        let steering_curve = steering_curve
            .iter()
            .map(Vector2D::from_na)
            .collect_cxx_vector();
        let mut center_of_mass = Box::pin(Location::from_na(center_of_mass));
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
            torque_curve: from.torque_curve().iter().map(|v| v.to_na()).collect(),
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
            center_of_mass: from.center_of_mass().to_na(),
            steering_curve: from.steering_curve().iter().map(|v| v.to_na()).collect(),
            wheels: from.wheels().iter().cloned().collect(),
            use_sweep_wheel_collision: from.use_sweep_wheel_collision(),
        }
    }
}

#[derive(Debug, Clone)]
pub struct EpisodeSettings {
    pub synchronous_mode: bool,
    pub no_rendering_mode: bool,
    pub fixed_delta_seconds: Option<f64>,
    pub substepping: bool,
    pub max_substep_delta_time: f64,
    pub max_substeps: u64,
    pub max_culling_distance: f32,
    pub deterministic_ragdolls: bool,
    pub tile_stream_distance: f32,
    pub actor_active_distance: f32,
}

impl EpisodeSettings {
    pub(crate) fn from_cxx(from: &FfiEpisodeSettings) -> Self {
        let fixed_delta_seconds = from.fixed_delta_seconds();
        let fixed_delta_seconds = if fixed_delta_seconds.is_nan() {
            None
        } else {
            Some(fixed_delta_seconds)
        };

        Self {
            synchronous_mode: from.synchronous_mode(),
            no_rendering_mode: from.no_rendering_mode(),
            fixed_delta_seconds,
            substepping: from.substepping(),
            max_substep_delta_time: from.max_substep_delta_time(),
            max_substeps: from.max_substeps().0 as u64,
            max_culling_distance: from.max_culling_distance(),
            deterministic_ragdolls: from.deterministic_ragdolls(),
            tile_stream_distance: from.tile_stream_distance(),
            actor_active_distance: from.actor_active_distance(),
        }
    }

    pub(crate) fn to_cxx(&self) -> UniquePtr<FfiEpisodeSettings> {
        let Self {
            synchronous_mode,
            no_rendering_mode,
            fixed_delta_seconds,
            substepping,
            max_substep_delta_time,
            max_substeps,
            max_culling_distance,
            deterministic_ragdolls,
            tile_stream_distance,
            actor_active_distance,
        } = *self;
        let fixed_delta_seconds = fixed_delta_seconds.unwrap_or(0.0);

        FfiEpisodeSettings::new2(
            synchronous_mode,
            no_rendering_mode,
            fixed_delta_seconds,
            substepping,
            max_substep_delta_time,
            c_int(max_substeps as std::os::raw::c_int),
            max_culling_distance,
            deterministic_ragdolls,
            tile_stream_distance,
            actor_active_distance,
        )
        .within_unique_ptr()
    }
}

impl Default for EpisodeSettings {
    fn default() -> Self {
        let ffi = FfiEpisodeSettings::new().within_unique_ptr();
        Self::from_cxx(&ffi)
    }
}
