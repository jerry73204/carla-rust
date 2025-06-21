//! World management and simulation control.

use crate::{
    actor::{Actor, ActorId, ActorList},
    client::{ActorBlueprint, BlueprintLibrary},
    error::CarlaResult,
    geom::Transform,
    road::Map,
    time::Timestamp,
    traits::ActorT,
};
use carla_sys::WorldWrapper;
use std::{sync::Arc, time::Duration};

/// Represents the simulation world.
#[derive(Debug, Clone)]
pub struct World {
    /// Internal handle to carla-sys World
    inner: Arc<WorldWrapper>,
}

impl World {
    /// Create a new World from a WorldWrapper
    pub(crate) fn from_cxx(inner: WorldWrapper) -> Self {
        Self {
            inner: Arc::new(inner),
        }
    }

    /// Get the world ID.
    pub fn id(&self) -> u64 {
        self.inner.get_id()
    }

    /// Get the map for this world.
    pub fn map(&self) -> CarlaResult<Map> {
        let map_wrapper = self.inner.get_map();
        Ok(Map::new(map_wrapper))
    }

    /// Get the blueprint library.
    pub fn blueprint_library(&self) -> CarlaResult<BlueprintLibrary> {
        let blueprint_library_wrapper = self.inner.get_blueprint_library();
        Ok(BlueprintLibrary::from_cxx(blueprint_library_wrapper))
    }

    /// Get the spectator actor (main camera).
    pub fn spectator(&self) -> CarlaResult<Actor> {
        self.inner
            .get_spectator()
            .map(|actor_wrapper| Actor::from_cxx(actor_wrapper))
            .ok_or_else(|| {
                crate::error::CarlaError::Actor(
                    crate::error::ActorError::NotFound(0), // Spectator doesn't have a specific ID
                )
            })
    }

    /// Get weather parameters.
    pub fn weather(&self) -> CarlaResult<WeatherParameters> {
        let simple_weather = self.inner.get_weather();
        Ok(WeatherParameters::from(simple_weather))
    }

    /// Set weather parameters.
    pub fn set_weather(&self, weather: &WeatherParameters) -> CarlaResult<()> {
        let simple_weather = weather.into();
        self.inner.set_weather(&simple_weather);
        Ok(())
    }

    /// Get a snapshot of the current world state.
    pub fn snapshot(&self) -> CarlaResult<WorldSnapshot> {
        let cxx_timestamp = self.inner.get_snapshot();
        let timestamp = crate::time::Timestamp::from(cxx_timestamp);

        // Get actor snapshots
        let actor_list = self.actors()?;
        let mut actor_snapshots = Vec::new();

        for actor in &actor_list {
            let snapshot = ActorSnapshot {
                id: actor.id(),
                transform: actor.transform(),
                velocity: actor.velocity(),
                angular_velocity: actor.angular_velocity(),
                acceleration: actor.acceleration(),
            };
            actor_snapshots.push(snapshot);
        }

        Ok(WorldSnapshot {
            timestamp,
            actors: actor_snapshots,
        })
    }

    /// Get an actor by ID.
    pub fn actor(&self, actor_id: ActorId) -> CarlaResult<Option<Actor>> {
        Ok(self
            .inner
            .get_actor(actor_id)
            .map(|actor_wrapper| Actor::from_cxx(actor_wrapper)))
    }

    /// Get all actors in the world.
    pub fn actors(&self) -> CarlaResult<ActorList> {
        let simple_actor_list = self.inner.get_actors();
        Ok(ActorList::new(simple_actor_list, Arc::clone(&self.inner)))
    }

    /// Get actors filtered by type.
    pub fn actors_by_type(&self, actor_type: &str) -> CarlaResult<Vec<Actor>> {
        let actor_list = self.actors()?;
        Ok(actor_list.find_by_type(actor_type))
    }

    /// Spawn an actor.
    pub fn spawn_actor(
        &self,
        blueprint: &ActorBlueprint,
        transform: &Transform,
        parent: Option<&Actor>,
    ) -> CarlaResult<Actor> {
        let simple_transform: carla_sys::SimpleTransform = transform.into();
        let parent_actor = parent.map(|p| p.get_inner_actor());

        match self
            .inner
            .spawn_actor(blueprint.inner(), &simple_transform, parent_actor)
        {
            Ok(actor_wrapper) => Ok(Actor::from_cxx(actor_wrapper)),
            Err(_e) => Err(crate::error::SpawnError::LocationOccupied {
                location: transform.location.clone(),
            }
            .into()),
        }
    }

    /// Try to spawn an actor (returns None if location is occupied).
    pub fn try_spawn_actor(
        &self,
        blueprint: &ActorBlueprint,
        transform: &Transform,
        parent: Option<&Actor>,
    ) -> CarlaResult<Option<Actor>> {
        let simple_transform: carla_sys::SimpleTransform = transform.into();
        let parent_actor = parent.map(|p| p.get_inner_actor());

        match self
            .inner
            .try_spawn_actor(blueprint.inner(), &simple_transform, parent_actor)
        {
            Some(actor_wrapper) => Ok(Some(Actor::from_cxx(actor_wrapper))),
            None => Ok(None),
        }
    }

    /// Tick the world (advance simulation by one step).
    pub fn tick(&self) -> CarlaResult<WorldSnapshot> {
        let timeout = Duration::from_secs(10); // Default timeout
        let _frame_id = self.inner.tick(timeout);

        // Return current snapshot after tick
        self.snapshot()
    }

    /// Wait for a tick with timeout.
    pub fn wait_for_tick(&self, timeout: Option<Duration>) -> CarlaResult<WorldSnapshot> {
        let timeout = timeout.unwrap_or_else(|| Duration::from_secs(10));
        let _frame_id = self.inner.tick(timeout);

        // Return current snapshot after tick
        self.snapshot()
    }

    /// Enable/disable synchronous mode.
    pub fn set_synchronous_mode(&self, enabled: bool) -> CarlaResult<()> {
        let mut settings = self.inner.get_settings_raw();
        settings.synchronous_mode = enabled;
        let timeout = Duration::from_secs(10);
        self.inner.apply_settings_raw(&settings, timeout);
        Ok(())
    }

    /// Get simulation settings.
    pub fn settings(&self) -> CarlaResult<WorldSettings> {
        let simple_settings = self.inner.get_settings_raw();
        Ok(WorldSettings::from(simple_settings))
    }

    /// Apply simulation settings.
    pub fn apply_settings(&self, settings: &WorldSettings) -> CarlaResult<()> {
        let simple_settings = settings.into();
        let timeout = Duration::from_secs(10);
        self.inner.apply_settings_raw(&simple_settings, timeout);
        Ok(())
    }

    /// Get all traffic lights in the world.
    pub fn traffic_lights(&self) -> CarlaResult<Vec<crate::actor::TrafficLight>> {
        let actors = self.actors()?;
        Ok(actors
            .find_by_type("traffic.traffic_light")
            .into_iter()
            .filter_map(|actor| match actor.to_traffic_light() {
                Ok(traffic_light) => Some(traffic_light),
                Err(_) => None,
            })
            .collect())
    }

    /// Get traffic lights that affect a specific waypoint.
    ///
    /// Returns all traffic lights within the specified distance that affect
    /// the given waypoint.
    pub fn traffic_lights_from_waypoint(
        &self,
        waypoint: &crate::road::Waypoint,
        distance: f64,
    ) -> CarlaResult<ActorList> {
        let simple_actor_list = self
            .inner
            .get_traffic_lights_from_waypoint(waypoint.inner().get_waypoint(), distance);
        Ok(ActorList::new(simple_actor_list, Arc::clone(&self.inner)))
    }

    /// Get all traffic lights in a junction.
    pub fn traffic_lights_in_junction(&self, junction_id: i32) -> CarlaResult<ActorList> {
        let simple_actor_list = self.inner.get_traffic_lights_in_junction(junction_id);
        Ok(ActorList::new(simple_actor_list, Arc::clone(&self.inner)))
    }

    /// Get reference to the internal WorldWrapper for advanced operations
    pub fn inner(&self) -> &WorldWrapper {
        &self.inner
    }
}

/// World snapshot containing all actor states at a given time.
#[derive(Debug, Clone)]
pub struct WorldSnapshot {
    /// Snapshot timestamp
    pub timestamp: Timestamp,
    /// Actor snapshots
    pub actors: Vec<ActorSnapshot>,
}

/// Snapshot of an actor's state.
#[derive(Debug, Clone)]
pub struct ActorSnapshot {
    /// Actor ID
    pub id: ActorId,
    /// Actor transform
    pub transform: Transform,
    /// Actor velocity
    pub velocity: crate::geom::Vector3D,
    /// Actor angular velocity
    pub angular_velocity: crate::geom::Vector3D,
    /// Actor acceleration
    pub acceleration: crate::geom::Vector3D,
}

/// Weather parameters for the simulation.
#[derive(Debug, Clone, PartialEq)]
pub struct WeatherParameters {
    /// Cloud amount [0.0, 100.0]
    pub cloudiness: f32,
    /// Precipitation amount [0.0, 100.0]
    pub precipitation: f32,
    /// Precipitation deposits [0.0, 100.0]
    pub precipitation_deposits: f32,
    /// Wind intensity [0.0, 100.0]
    pub wind_intensity: f32,
    /// Sun azimuth angle in degrees [0.0, 360.0]
    pub sun_azimuth_angle: f32,
    /// Sun altitude angle in degrees [-90.0, 90.0]
    pub sun_altitude_angle: f32,
    /// Fog density [0.0, 100.0]
    pub fog_density: f32,
    /// Fog distance in meters
    pub fog_distance: f32,
    /// Fog falloff
    pub fog_falloff: f32,
    /// Wetness [0.0, 100.0]
    pub wetness: f32,
    /// Scattering intensity [0.0, 100.0]
    pub scattering_intensity: f32,
    /// Mie scattering scale [0.0, 100.0]
    pub mie_scattering_scale: f32,
    /// Rayleigh scattering scale [0.0, 100.0]
    pub rayleigh_scattering_scale: f32,
    /// Dust storm intensity [0.0, 100.0]
    pub dust_storm: f32,
}

impl Default for WeatherParameters {
    fn default() -> Self {
        Self {
            cloudiness: 0.0,
            precipitation: 0.0,
            precipitation_deposits: 0.0,
            wind_intensity: 0.0,
            sun_azimuth_angle: 0.0,
            sun_altitude_angle: 45.0,
            fog_density: 0.0,
            fog_distance: 0.0,
            fog_falloff: 0.2,
            wetness: 0.0,
            scattering_intensity: 1.0,
            mie_scattering_scale: 0.03,
            rayleigh_scattering_scale: 0.0331,
            dust_storm: 0.0,
        }
    }
}

impl From<carla_sys::ffi::bridge::SimpleWeatherParameters> for WeatherParameters {
    fn from(simple: carla_sys::ffi::bridge::SimpleWeatherParameters) -> Self {
        Self {
            cloudiness: simple.cloudiness,
            precipitation: simple.precipitation,
            precipitation_deposits: simple.precipitation_deposits,
            wind_intensity: simple.wind_intensity,
            sun_azimuth_angle: simple.sun_azimuth_angle,
            sun_altitude_angle: simple.sun_altitude_angle,
            fog_density: simple.fog_density,
            fog_distance: simple.fog_distance,
            fog_falloff: simple.fog_falloff,
            wetness: simple.wetness,
            scattering_intensity: simple.scattering_intensity,
            mie_scattering_scale: simple.mie_scattering_scale,
            rayleigh_scattering_scale: simple.rayleigh_scattering_scale,
            dust_storm: simple.dust_storm,
        }
    }
}

impl From<&WeatherParameters> for carla_sys::ffi::bridge::SimpleWeatherParameters {
    fn from(weather: &WeatherParameters) -> Self {
        Self {
            cloudiness: weather.cloudiness,
            precipitation: weather.precipitation,
            precipitation_deposits: weather.precipitation_deposits,
            wind_intensity: weather.wind_intensity,
            sun_azimuth_angle: weather.sun_azimuth_angle,
            sun_altitude_angle: weather.sun_altitude_angle,
            fog_density: weather.fog_density,
            fog_distance: weather.fog_distance,
            fog_falloff: weather.fog_falloff,
            wetness: weather.wetness,
            scattering_intensity: weather.scattering_intensity,
            mie_scattering_scale: weather.mie_scattering_scale,
            rayleigh_scattering_scale: weather.rayleigh_scattering_scale,
            dust_storm: weather.dust_storm,
        }
    }
}

/// World simulation settings.
#[derive(Debug, Clone, PartialEq)]
pub struct WorldSettings {
    /// Synchronous mode enabled
    pub synchronous_mode: bool,
    /// Fixed delta time for synchronous mode
    pub fixed_delta_seconds: Option<f64>,
    /// Substepping enabled
    pub substepping: bool,
    /// Maximum substep delta time
    pub max_substep_delta_time: f64,
    /// Maximum substeps
    pub max_substeps: i32,
    /// Max culling distance
    pub max_culling_distance: f32,
    /// Deterministic ragdolls
    pub deterministic_ragdolls: bool,
    /// Tile stream distance
    pub tile_stream_distance: f32,
    /// Actor active distance
    pub actor_active_distance: f32,
    /// No rendering mode enabled
    pub no_rendering_mode: bool,
    /// Spectator as ego vehicle
    pub spectator_as_ego: bool,
}

impl Default for WorldSettings {
    fn default() -> Self {
        Self {
            synchronous_mode: false,
            fixed_delta_seconds: None,
            substepping: true,
            max_substep_delta_time: 0.01,
            max_substeps: 10,
            max_culling_distance: 0.0,
            deterministic_ragdolls: true,
            tile_stream_distance: 3000.0,
            actor_active_distance: 2000.0,
            no_rendering_mode: false,
            spectator_as_ego: false,
        }
    }
}

impl From<carla_sys::ffi::bridge::SimpleEpisodeSettings> for WorldSettings {
    fn from(simple: carla_sys::ffi::bridge::SimpleEpisodeSettings) -> Self {
        Self {
            synchronous_mode: simple.synchronous_mode,
            fixed_delta_seconds: if simple.fixed_delta_seconds <= 0.0 {
                None
            } else {
                Some(simple.fixed_delta_seconds)
            },
            substepping: simple.substepping,
            max_substep_delta_time: simple.max_substep_delta_time,
            max_substeps: simple.max_substeps,
            max_culling_distance: simple.max_culling_distance,
            deterministic_ragdolls: simple.deterministic_ragdolls,
            tile_stream_distance: simple.tile_stream_distance,
            actor_active_distance: simple.actor_active_distance,
            no_rendering_mode: simple.no_rendering_mode,
            spectator_as_ego: simple.spectator_as_ego,
        }
    }
}

impl From<&WorldSettings> for carla_sys::ffi::bridge::SimpleEpisodeSettings {
    fn from(settings: &WorldSettings) -> Self {
        Self {
            synchronous_mode: settings.synchronous_mode,
            fixed_delta_seconds: settings.fixed_delta_seconds.unwrap_or(0.0),
            substepping: settings.substepping,
            max_substep_delta_time: settings.max_substep_delta_time,
            max_substeps: settings.max_substeps,
            max_culling_distance: settings.max_culling_distance,
            deterministic_ragdolls: settings.deterministic_ragdolls,
            tile_stream_distance: settings.tile_stream_distance,
            actor_active_distance: settings.actor_active_distance,
            no_rendering_mode: settings.no_rendering_mode,
            spectator_as_ego: settings.spectator_as_ego,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        geom::{Location, Rotation},
        time::Timestamp,
    };

    #[test]
    fn test_weather_parameters_default() {
        let weather = WeatherParameters::default();
        assert_eq!(weather.cloudiness, 0.0);
        assert_eq!(weather.precipitation, 0.0);
        assert_eq!(weather.sun_altitude_angle, 45.0);
        assert_eq!(weather.fog_falloff, 0.2);
        assert_eq!(weather.scattering_intensity, 1.0);
        assert_eq!(weather.dust_storm, 0.0);
    }

    #[test]
    fn test_weather_parameters_conversion() {
        let weather = WeatherParameters {
            cloudiness: 50.0,
            precipitation: 30.0,
            dust_storm: 10.0,
            ..Default::default()
        };

        let simple_weather: carla_sys::ffi::bridge::SimpleWeatherParameters = (&weather).into();
        assert_eq!(simple_weather.cloudiness, 50.0);
        assert_eq!(simple_weather.precipitation, 30.0);
        assert_eq!(simple_weather.dust_storm, 10.0);

        let weather_back = WeatherParameters::from(simple_weather);
        assert_eq!(weather_back, weather);
    }

    #[test]
    fn test_world_settings_default() {
        let settings = WorldSettings::default();
        assert!(!settings.synchronous_mode);
        assert_eq!(settings.fixed_delta_seconds, None);
        assert!(settings.substepping);
        assert_eq!(settings.max_substep_delta_time, 0.01);
        assert!(!settings.no_rendering_mode);
        assert!(!settings.spectator_as_ego);
    }

    #[test]
    fn test_world_settings_conversion() {
        let settings = WorldSettings {
            synchronous_mode: true,
            fixed_delta_seconds: Some(0.05),
            no_rendering_mode: true,
            spectator_as_ego: true,
            ..Default::default()
        };

        let simple_settings: carla_sys::ffi::bridge::SimpleEpisodeSettings = (&settings).into();
        assert!(simple_settings.synchronous_mode);
        assert_eq!(simple_settings.fixed_delta_seconds, 0.05);
        assert!(simple_settings.no_rendering_mode);
        assert!(simple_settings.spectator_as_ego);

        let settings_back = WorldSettings::from(simple_settings);
        assert_eq!(settings_back, settings);
    }

    #[test]
    fn test_actor_snapshot_creation() {
        let snapshot = ActorSnapshot {
            id: 42,
            transform: Transform::new(
                Location::new(1.0, 2.0, 3.0),
                Rotation::new(10.0, 20.0, 30.0),
            ),
            velocity: crate::geom::Vector3D::new(5.0, 0.0, 0.0),
            angular_velocity: crate::geom::Vector3D::new(0.0, 0.0, 1.0),
            acceleration: crate::geom::Vector3D::new(0.0, 0.0, -9.8),
        };

        assert_eq!(snapshot.id, 42);
        assert_eq!(snapshot.transform.location.x, 1.0);
        assert_eq!(snapshot.velocity.x, 5.0);
        assert_eq!(snapshot.angular_velocity.z, 1.0);
        assert_eq!(snapshot.acceleration.z, -9.8);
    }

    #[test]
    fn test_world_snapshot_creation() {
        let timestamp = Timestamp::new(100, 5.0, 0.016, 1234567890.0);
        let snapshot = WorldSnapshot {
            timestamp,
            actors: vec![
                ActorSnapshot {
                    id: 1,
                    transform: Transform::default(),
                    velocity: crate::geom::Vector3D::default(),
                    angular_velocity: crate::geom::Vector3D::default(),
                    acceleration: crate::geom::Vector3D::default(),
                },
                ActorSnapshot {
                    id: 2,
                    transform: Transform::default(),
                    velocity: crate::geom::Vector3D::default(),
                    angular_velocity: crate::geom::Vector3D::default(),
                    acceleration: crate::geom::Vector3D::default(),
                },
            ],
        };

        assert_eq!(snapshot.timestamp.frame(), 100);
        assert_eq!(snapshot.actors.len(), 2);
        assert_eq!(snapshot.actors[0].id, 1);
        assert_eq!(snapshot.actors[1].id, 2);
    }
}
