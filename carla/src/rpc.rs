use autocxx::prelude::*;
pub use carla_sys::carla::rpc::{
    AttachmentType, OpendriveGenerationParameters, TrafficLightState, VehicleControl, VehicleDoor,
    VehicleLightState_LightState, VehicleWheelLocation,
};
use carla_sys::carla_rust::rpc::FfiEpisodeSettings;

pub use carla_sys::carla_rust::rpc::FfiActorId as ActorId;

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
