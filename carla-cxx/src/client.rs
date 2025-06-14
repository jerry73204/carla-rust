//! High-level wrapper types for CARLA client functionality.

use crate::ffi::{self, Actor, ActorBlueprint, BlueprintLibrary, Client, SimpleTransform, World};
use anyhow::Result;
use cxx::{SharedPtr, UniquePtr};
use std::time::Duration;

/// High-level wrapper for CARLA Client
pub struct ClientWrapper {
    inner: UniquePtr<Client>,
}

impl ClientWrapper {
    /// Create a new CARLA client connection
    pub fn new(host: &str, port: u16) -> Result<Self> {
        let inner = ffi::create_client(host, port, 0);
        if inner.is_null() {
            anyhow::bail!("Failed to create CARLA client");
        }
        Ok(Self { inner })
    }

    /// Get reference to the inner Client for FFI operations
    pub fn get_client(&self) -> &Client {
        &self.inner
    }

    /// Get the server version string
    pub fn get_server_version(&self) -> String {
        ffi::Client_GetServerVersion(&self.inner)
    }

    /// Set client timeout duration
    pub fn set_timeout(&mut self, timeout: Duration) {
        ffi::Client_SetTimeout(self.inner.pin_mut(), timeout.as_secs_f64());
    }

    /// Get current client timeout duration
    pub fn get_timeout(&mut self) -> Duration {
        Duration::from_secs_f64(ffi::Client_GetTimeout(self.inner.pin_mut()))
    }

    /// Get the simulation world
    pub fn get_world(&self) -> WorldWrapper {
        let world_ptr = ffi::Client_GetWorld(&self.inner);
        WorldWrapper { inner: world_ptr }
    }

    // Recording methods

    /// Start recording simulation data to a file
    pub fn start_recorder(&self, filename: &str, additional_data: bool) -> String {
        ffi::Client_StartRecorder(&self.inner, filename, additional_data)
    }

    /// Stop the current recording session
    pub fn stop_recorder(&self) {
        ffi::Client_StopRecorder(&self.inner);
    }

    /// Show information about a recording file
    pub fn show_recorder_file_info(&self, filename: &str, show_all: bool) -> String {
        ffi::Client_ShowRecorderFileInfo(&self.inner, filename, show_all)
    }

    /// Show collision analysis from a recording file
    /// type1 and type2 are actor type filters: b'a' for all, b'v' for vehicles, b'w' for walkers
    pub fn show_recorder_collisions(&self, filename: &str, type1: u8, type2: u8) -> String {
        ffi::Client_ShowRecorderCollisions(&self.inner, filename, type1, type2)
    }

    /// Show blocked actors analysis from a recording file
    pub fn show_recorder_actors_blocked(
        &self,
        filename: &str,
        min_time: f64,
        min_distance: f64,
    ) -> String {
        ffi::Client_ShowRecorderActorsBlocked(&self.inner, filename, min_time, min_distance)
    }

    // Playback methods

    /// Replay a recording file
    pub fn replay_file(
        &self,
        filename: &str,
        start_time: f64,
        duration: f64,
        follow_id: u32,
        replay_sensors: bool,
    ) -> String {
        ffi::Client_ReplayFile(
            &self.inner,
            filename,
            start_time,
            duration,
            follow_id,
            replay_sensors,
        )
    }

    /// Stop the current replay session
    pub fn stop_replayer(&self, keep_actors: bool) {
        ffi::Client_StopReplayer(&self.inner, keep_actors);
    }

    /// Set the replay speed multiplier
    pub fn set_replayer_time_factor(&self, time_factor: f64) {
        ffi::Client_SetReplayerTimeFactor(&self.inner, time_factor);
    }

    /// Set whether to ignore hero vehicles during replay
    pub fn set_replayer_ignore_hero(&self, ignore_hero: bool) {
        ffi::Client_SetReplayerIgnoreHero(&self.inner, ignore_hero);
    }

    /// Set whether to ignore spectator during replay
    pub fn set_replayer_ignore_spectator(&self, ignore_spectator: bool) {
        ffi::Client_SetReplayerIgnoreSpectator(&self.inner, ignore_spectator);
    }

    // Batch operation methods

    /// Apply multiple commands asynchronously without waiting for results
    pub fn apply_batch(
        &self,
        commands: &[crate::ffi::bridge::SimpleBatchCommand],
        do_tick_cue: bool,
    ) {
        ffi::bridge::Client_ApplyBatch(&self.inner, commands, do_tick_cue);
    }

    /// Apply multiple commands synchronously and return results
    pub fn apply_batch_sync(
        &self,
        commands: &[crate::ffi::bridge::SimpleBatchCommand],
        do_tick_cue: bool,
    ) -> Vec<crate::ffi::bridge::SimpleBatchResponse> {
        ffi::bridge::Client_ApplyBatchSync(&self.inner, commands, do_tick_cue)
    }
}

/// High-level wrapper for CARLA World
pub struct WorldWrapper {
    inner: SharedPtr<World>,
}

impl WorldWrapper {
    /// Get the world ID
    pub fn get_id(&self) -> u64 {
        ffi::World_GetId(&self.inner)
    }

    /// Get the blueprint library for spawning actors
    pub fn get_blueprint_library(&self) -> BlueprintLibraryWrapper {
        let library_ptr = ffi::World_GetBlueprintLibrary(&self.inner);
        BlueprintLibraryWrapper { inner: library_ptr }
    }

    /// Get the spectator actor (camera)
    pub fn get_spectator(&self) -> Option<ActorWrapper> {
        let actor_ptr = ffi::World_GetSpectator(&self.inner);
        if actor_ptr.is_null() {
            None
        } else {
            Some(ActorWrapper { inner: actor_ptr })
        }
    }

    /// Advance the simulation by one tick
    pub fn tick(&self, timeout: Duration) -> u64 {
        ffi::World_Tick(&self.inner, timeout.as_secs_f64())
    }

    /// Get the current world snapshot with timestamp
    pub fn get_snapshot(&self) -> crate::time::Timestamp {
        let simple_timestamp = ffi::World_GetSnapshot(&self.inner);
        crate::time::Timestamp::from(simple_timestamp)
    }

    /// Spawn an actor in the world (panics if failed)
    pub fn spawn_actor(
        &self,
        blueprint: &ActorBlueprint,
        transform: &SimpleTransform,
        parent: Option<&Actor>,
    ) -> Result<ActorWrapper> {
        let parent_ptr = parent
            .map(|p| p as *const Actor)
            .unwrap_or(std::ptr::null());
        let actor_ptr =
            unsafe { ffi::World_SpawnActor(&self.inner, blueprint, transform, parent_ptr) };
        if actor_ptr.is_null() {
            anyhow::bail!("Failed to spawn actor");
        }
        Ok(ActorWrapper { inner: actor_ptr })
    }

    /// Try to spawn an actor in the world (returns None if failed)
    pub fn try_spawn_actor(
        &self,
        blueprint: &ActorBlueprint,
        transform: &SimpleTransform,
        parent: Option<&Actor>,
    ) -> Option<ActorWrapper> {
        let parent_ptr = parent
            .map(|p| p as *const Actor)
            .unwrap_or(std::ptr::null());
        let actor_ptr =
            unsafe { ffi::World_TrySpawnActor(&self.inner, blueprint, transform, parent_ptr) };
        if actor_ptr.is_null() {
            None
        } else {
            Some(ActorWrapper { inner: actor_ptr })
        }
    }

    /// Get the map associated with this world
    pub fn get_map(&self) -> crate::map::MapWrapper {
        let map_ptr = ffi::World_GetMap(&self.inner);
        crate::map::MapWrapper::new(map_ptr)
    }

    /// Get the current episode settings (raw FFI version)
    pub fn get_settings_raw(&self) -> crate::ffi::bridge::SimpleEpisodeSettings {
        ffi::World_GetSettings(&self.inner)
    }

    /// Apply new episode settings to the world (raw FFI version)
    pub fn apply_settings_raw(
        &self,
        settings: &crate::ffi::bridge::SimpleEpisodeSettings,
        timeout: Duration,
    ) -> u64 {
        ffi::World_ApplySettings(&self.inner, settings, timeout.as_secs_f64())
    }

    /// Get the current weather parameters
    pub fn get_weather(&self) -> crate::ffi::bridge::SimpleWeatherParameters {
        ffi::World_GetWeather(&self.inner)
    }

    /// Set the weather parameters
    pub fn set_weather(&self, weather: &crate::ffi::bridge::SimpleWeatherParameters) {
        ffi::World_SetWeather(&self.inner, weather);
    }

    /// Check if weather simulation is enabled
    pub fn is_weather_enabled(&self) -> bool {
        ffi::World_IsWeatherEnabled(&self.inner)
    }

    // World interaction methods

    /// Cast a ray between two points and get all intersection points
    pub fn cast_ray(
        &self,
        start_location: &crate::ffi::SimpleLocation,
        end_location: &crate::ffi::SimpleLocation,
    ) -> Vec<crate::ffi::bridge::SimpleLabelledPoint> {
        ffi::World_CastRay(&self.inner, start_location, end_location)
    }

    /// Project a point along a direction and get the first intersection
    pub fn project_point(
        &self,
        location: &crate::ffi::SimpleLocation,
        direction: &crate::ffi::SimpleVector3D,
        search_distance: f32,
    ) -> crate::ffi::bridge::SimpleOptionalLabelledPoint {
        ffi::World_ProjectPoint(&self.inner, location, direction, search_distance)
    }

    /// Project a point to the ground and get the intersection
    pub fn ground_projection(
        &self,
        location: &crate::ffi::SimpleLocation,
        search_distance: f32,
    ) -> crate::ffi::bridge::SimpleOptionalLabelledPoint {
        ffi::World_GroundProjection(&self.inner, location, search_distance)
    }

    /// Get traffic lights near a waypoint within the specified distance
    pub fn get_traffic_lights_from_waypoint(
        &self,
        waypoint: &crate::ffi::Waypoint,
        distance: f64,
    ) -> crate::ffi::bridge::SimpleActorList {
        ffi::World_GetTrafficLightsFromWaypoint(&self.inner, waypoint, distance)
    }

    /// Get traffic lights in a specific junction
    pub fn get_traffic_lights_in_junction(
        &self,
        junction_id: i32,
    ) -> crate::ffi::bridge::SimpleActorList {
        ffi::World_GetTrafficLightsInJunction(&self.inner, junction_id)
    }

    /// Get a random location from the navigation mesh
    pub fn get_random_location_from_navigation(
        &self,
    ) -> crate::ffi::bridge::SimpleOptionalLocation {
        ffi::World_GetRandomLocationFromNavigation(&self.inner)
    }

    /// Set the percentage of pedestrians that cross at traffic lights
    pub fn set_pedestrians_cross_factor(&self, percentage: f32) {
        ffi::World_SetPedestriansCrossFactor(&self.inner, percentage);
    }

    /// Get all actors in the world
    pub fn get_actors(&self) -> crate::ffi::bridge::SimpleActorList {
        ffi::World_GetActors(&self.inner)
    }

    /// Get specific actors by their IDs
    pub fn get_actors_by_ids(&self, actor_ids: &[u32]) -> crate::ffi::bridge::SimpleActorList {
        ffi::World_GetActorsByIds(&self.inner, actor_ids)
    }

    /// Get a specific actor by its ID
    pub fn get_actor(&self, actor_id: u32) -> Option<ActorWrapper> {
        let actor_ptr = ffi::World_GetActor(&self.inner, actor_id);
        if actor_ptr.is_null() {
            None
        } else {
            Some(ActorWrapper { inner: actor_ptr })
        }
    }

    // Debug drawing methods

    /// Draw a debug point in the world
    pub fn draw_debug_point(&self, point: &crate::ffi::bridge::SimpleDebugPoint) {
        ffi::bridge::World_DrawDebugPoint(&self.inner, point);
    }

    /// Draw a debug line in the world
    pub fn draw_debug_line(&self, line: &crate::ffi::bridge::SimpleDebugLine) {
        ffi::bridge::World_DrawDebugLine(&self.inner, line);
    }

    /// Draw a debug arrow in the world
    pub fn draw_debug_arrow(&self, arrow: &crate::ffi::bridge::SimpleDebugArrow) {
        ffi::bridge::World_DrawDebugArrow(&self.inner, arrow);
    }

    /// Draw a debug box in the world
    pub fn draw_debug_box(&self, box_shape: &crate::ffi::bridge::SimpleDebugBox) {
        ffi::bridge::World_DrawDebugBox(&self.inner, box_shape);
    }

    /// Draw a debug string in the world
    pub fn draw_debug_string(&self, string: &crate::ffi::bridge::SimpleDebugString) {
        ffi::bridge::World_DrawDebugString(&self.inner, string);
    }
}

/// High-level wrapper for CARLA Actor
pub struct ActorWrapper {
    inner: SharedPtr<Actor>,
}

impl ActorWrapper {
    /// Get reference to the inner Actor for casting
    pub fn get_actor(&self) -> &Actor {
        &self.inner
    }

    /// Get the actor's unique ID
    pub fn get_id(&self) -> u32 {
        ffi::Actor_GetId(&self.inner)
    }

    /// Get the actor's type ID string
    pub fn get_type_id(&self) -> String {
        ffi::Actor_GetTypeId(&self.inner)
    }

    /// Get the actor's display ID string
    pub fn get_display_id(&self) -> String {
        ffi::Actor_GetDisplayId(&self.inner)
    }

    /// Get the actor's current location
    pub fn get_location(&self) -> crate::ffi::SimpleLocation {
        ffi::Actor_GetLocation(&self.inner)
    }

    /// Get the actor's current transform
    pub fn get_transform(&self) -> SimpleTransform {
        ffi::Actor_GetTransform(&self.inner)
    }

    /// Set the actor's location
    pub fn set_location(&self, location: &crate::ffi::SimpleLocation) {
        ffi::Actor_SetLocation(&self.inner, location);
    }

    /// Set the actor's transform
    pub fn set_transform(&self, transform: &SimpleTransform) {
        ffi::Actor_SetTransform(&self.inner, transform);
    }

    /// Destroy the actor
    pub fn destroy(&self) -> bool {
        ffi::Actor_Destroy(&self.inner)
    }

    /// Check if the actor is still alive
    pub fn is_alive(&self) -> bool {
        ffi::Actor_IsAlive(&self.inner)
    }
}

/// High-level wrapper for CARLA BlueprintLibrary
pub struct BlueprintLibraryWrapper {
    inner: SharedPtr<BlueprintLibrary>,
}

impl BlueprintLibraryWrapper {
    /// Find a blueprint by ID
    pub fn find(&self, id: &str) -> Option<SharedPtr<ActorBlueprint>> {
        let blueprint_ptr = ffi::BlueprintLibrary_Find(&self.inner, id);
        if blueprint_ptr.is_null() {
            None
        } else {
            Some(blueprint_ptr)
        }
    }

    // TODO: Implement filter method - CXX doesn't support Vec<SharedPtr<T>>
    // pub fn filter(&self, wildcard_pattern: &str) -> Vec<SharedPtr<ActorBlueprint>> {
    //     ffi::BlueprintLibrary_Filter(&self.inner, wildcard_pattern)
    // }

    /// Get the number of blueprints in the library
    pub fn size(&self) -> usize {
        ffi::BlueprintLibrary_Size(&self.inner)
    }
}
