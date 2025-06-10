use carla_sys::*;

/// A collection of configuration options, corresponding to
/// `carla.WorldSettings` in Python API.
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
    pub(crate) fn from_c_settings(settings: carla_episode_settings_t) -> Self {
        let fixed_delta_seconds = if settings.fixed_delta_seconds.is_nan() {
            None
        } else {
            Some(settings.fixed_delta_seconds)
        };

        Self {
            synchronous_mode: settings.synchronous_mode,
            no_rendering_mode: settings.no_rendering_mode,
            fixed_delta_seconds,
            substepping: settings.substepping,
            max_substep_delta_time: settings.max_substep_delta_time,
            max_substeps: settings.max_substeps as u64,
            max_culling_distance: settings.max_culling_distance,
            deterministic_ragdolls: settings.deterministic_ragdolls,
            tile_stream_distance: settings.tile_stream_distance,
            actor_active_distance: settings.actor_active_distance,
        }
    }

    pub(crate) fn to_c_settings(&self) -> carla_episode_settings_t {
        carla_episode_settings_t {
            synchronous_mode: self.synchronous_mode,
            no_rendering_mode: self.no_rendering_mode,
            fixed_delta_seconds: self.fixed_delta_seconds.unwrap_or(0.0),
            substepping: self.substepping,
            max_substep_delta_time: self.max_substep_delta_time,
            max_substeps: self.max_substeps as u32,
            max_culling_distance: self.max_culling_distance,
            deterministic_ragdolls: self.deterministic_ragdolls,
            tile_stream_distance: self.tile_stream_distance,
            actor_active_distance: self.actor_active_distance,
        }
    }
}

impl Default for EpisodeSettings {
    fn default() -> Self {
        let c_settings = unsafe { carla_episode_settings_default() };
        Self::from_c_settings(c_settings)
    }
}
