use autocxx::prelude::*;
use carla_sys::carla_rust::rpc::FfiEpisodeSettings;

/// Simulation configuration settings, Corresponds to [`carla.WorldSettings`](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.WorldSettings) in the Python API.
///
/// These settings control how the simulation runs, including timing, rendering,
/// and physics behavior.
///
/// # Examples
///
/// ```
/// use carla::rpc::EpisodeSettings;
///
/// let mut settings = EpisodeSettings::default();
///
/// // Enable synchronous mode (client controls simulation ticks)
/// settings.synchronous_mode = true;
///
/// // Set fixed time step (20 FPS = 0.05 seconds per frame)
/// settings.fixed_delta_seconds = Some(0.05);
///
/// // Enable physics substepping for stability
/// settings.substepping = true;
/// settings.max_substeps = 10;
/// settings.max_substep_delta_time = 0.01;
/// ```
#[derive(Debug, Clone)]
pub struct EpisodeSettings {
    /// If true, the server waits for client tick commands (synchronous mode).
    /// If false, the simulation runs freely (asynchronous mode).
    pub synchronous_mode: bool,

    /// If true, disables rendering for performance (sensors still work).
    pub no_rendering_mode: bool,

    /// Fixed time step in seconds. `None` means variable time step.
    /// Common values: 0.05 (20 FPS), 0.033 (30 FPS), 0.0167 (60 FPS).
    pub fixed_delta_seconds: Option<f64>,

    /// Enable physics substepping for more accurate simulation.
    pub substepping: bool,

    /// Maximum time per physics substep (seconds).
    pub max_substep_delta_time: f64,

    /// Maximum number of physics substeps per frame.
    pub max_substeps: u64,

    /// Maximum distance for object culling (meters, 0.0 = disabled).
    pub max_culling_distance: f32,

    /// Enable deterministic ragdoll physics (reproducible simulations).
    pub deterministic_ragdolls: bool,

    /// Distance for tile streaming in large maps (meters).
    pub tile_stream_distance: f32,

    /// Distance at which actors become active (meters).
    pub actor_active_distance: f32,

    /// Whether the spectator acts as ego for tile loading in large maps.
    /// Default: `true`. Available in CARLA 0.9.15+.
    #[cfg(carla_0915)]
    pub spectator_as_ego: bool,
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
            #[cfg(carla_0915)]
            spectator_as_ego: from.spectator_as_ego(),
        }
    }

    pub(crate) fn to_cxx(&self) -> UniquePtr<FfiEpisodeSettings> {
        let fixed_delta_seconds = self.fixed_delta_seconds.unwrap_or(0.0);

        #[allow(unused_mut)]
        let mut settings = FfiEpisodeSettings::new2(
            self.synchronous_mode,
            self.no_rendering_mode,
            fixed_delta_seconds,
            self.substepping,
            self.max_substep_delta_time,
            c_int(self.max_substeps as std::os::raw::c_int),
            self.max_culling_distance,
            self.deterministic_ragdolls,
            self.tile_stream_distance,
            self.actor_active_distance,
        )
        .within_unique_ptr();

        #[cfg(carla_0915)]
        settings
            .pin_mut()
            .set_spectator_as_ego(self.spectator_as_ego);

        settings
    }
}

impl Default for EpisodeSettings {
    fn default() -> Self {
        let ffi = FfiEpisodeSettings::new().within_unique_ptr();
        Self::from_cxx(&ffi)
    }
}
