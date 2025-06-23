//! High-level wrapper types for CARLA client functionality.

use crate::ffi::{self, Actor, BlueprintLibrary, Client, SimpleTransform, World};
use anyhow::Result;
use cxx::{SharedPtr, UniquePtr};
use std::time::Duration;

/// High-level wrapper for CARLA Client
pub struct ClientWrapper {
    inner: UniquePtr<Client>,
}

impl std::fmt::Debug for ClientWrapper {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ClientWrapper")
            .field("inner", &"<UniquePtr<Client>>")
            .finish()
    }
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
    pub fn get_world(&self) -> Result<WorldWrapper> {
        let world_ptr = ffi::Client_GetWorld(&self.inner);
        if world_ptr.is_null() {
            anyhow::bail!("Failed to get world from client");
        }
        Ok(WorldWrapper { inner: world_ptr })
    }

    // World/Map management methods

    /// Get list of available maps
    pub fn get_available_maps(&self) -> Vec<String> {
        ffi::Client_GetAvailableMaps(&self.inner)
    }

    /// Load a new world/map
    pub fn load_world(&self, map_name: &str) -> Result<WorldWrapper> {
        let world_ptr = ffi::Client_LoadWorld(&self.inner, map_name);
        if world_ptr.is_null() {
            anyhow::bail!("Failed to load world: {}", map_name);
        }
        Ok(WorldWrapper { inner: world_ptr })
    }

    /// Reload the current world
    pub fn reload_world(&self, reset_settings: bool) -> Result<WorldWrapper> {
        let world_ptr = ffi::Client_ReloadWorld(&self.inner, reset_settings);
        if world_ptr.is_null() {
            anyhow::bail!("Failed to reload world");
        }
        Ok(WorldWrapper { inner: world_ptr })
    }

    /// Generate OpenDRIVE world from string
    pub fn generate_opendrive_world(&self, opendrive: &str) -> Result<WorldWrapper> {
        let world_ptr = ffi::Client_GenerateOpenDriveWorld(&self.inner, opendrive);
        if world_ptr.is_null() {
            anyhow::bail!("Failed to generate OpenDRIVE world");
        }
        Ok(WorldWrapper { inner: world_ptr })
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

impl std::fmt::Debug for WorldWrapper {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("WorldWrapper")
            .field("inner", &"<SharedPtr<World>>")
            .finish()
    }
}

impl WorldWrapper {
    /// Get the world ID
    pub fn get_id(&self) -> u64 {
        ffi::World_GetId(&self.inner)
    }

    /// Get the blueprint library for spawning actors
    pub fn get_blueprint_library(&self) -> Result<BlueprintLibraryWrapper> {
        let library_ptr = ffi::World_GetBlueprintLibrary(&self.inner);
        if library_ptr.is_null() {
            anyhow::bail!("Failed to get blueprint library");
        }
        Ok(BlueprintLibraryWrapper { inner: library_ptr })
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
        blueprint: &crate::actor_blueprint::ActorBlueprintWrapper,
        transform: &SimpleTransform,
        parent: Option<&Actor>,
    ) -> Result<ActorWrapper> {
        let parent_ptr = parent
            .map(|p| p as *const Actor)
            .unwrap_or(std::ptr::null());
        let actor_ptr = unsafe {
            ffi::World_SpawnActor(&self.inner, blueprint.get_inner(), transform, parent_ptr)
        };
        if actor_ptr.is_null() {
            anyhow::bail!("Failed to spawn actor");
        }
        Ok(ActorWrapper { inner: actor_ptr })
    }

    /// Try to spawn an actor in the world (returns None if failed)
    pub fn try_spawn_actor(
        &self,
        blueprint: &crate::actor_blueprint::ActorBlueprintWrapper,
        transform: &SimpleTransform,
        parent: Option<&Actor>,
    ) -> Option<ActorWrapper> {
        let parent_ptr = parent
            .map(|p| p as *const Actor)
            .unwrap_or(std::ptr::null());
        let actor_ptr = unsafe {
            ffi::World_TrySpawnActor(&self.inner, blueprint.get_inner(), transform, parent_ptr)
        };
        if actor_ptr.is_null() {
            None
        } else {
            Some(ActorWrapper { inner: actor_ptr })
        }
    }

    /// Get the map associated with this world
    pub fn get_map(&self) -> Result<crate::map::MapWrapper> {
        let map_ptr = ffi::World_GetMap(&self.inner);
        if map_ptr.is_null() {
            anyhow::bail!("Failed to get map from world");
        }
        Ok(crate::map::MapWrapper::new(map_ptr))
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

    /// Get actors filtered by type using native CARLA filtering
    pub fn get_actors_filtered_by_type(
        &self,
        wildcard_pattern: &str,
    ) -> crate::ffi::bridge::SimpleActorList {
        ffi::World_GetActorsFilteredByType(&self.inner, wildcard_pattern)
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

    // Advanced world features

    /// Load specific map layers dynamically
    pub fn load_level_layer(&self, map_layers: u8) {
        ffi::World_LoadLevelLayer(&self.inner, map_layers);
    }

    /// Unload specific map layers to save memory
    pub fn unload_level_layer(&self, map_layers: u8) {
        ffi::World_UnloadLevelLayer(&self.inner, map_layers);
    }

    /// Get bounding boxes of all level elements with specific tag
    pub fn get_level_bbs(
        &self,
        queried_tag: u8,
    ) -> Vec<crate::ffi::bridge::SimpleLevelBoundingBox> {
        ffi::World_GetLevelBBs(&self.inner, queried_tag)
    }

    /// Get environment objects with specific tag
    pub fn get_environment_objects(
        &self,
        queried_tag: u8,
    ) -> Vec<crate::ffi::bridge::SimpleEnvironmentObject> {
        ffi::World_GetEnvironmentObjects(&self.inner, queried_tag)
    }

    /// Enable or disable specific environment objects
    pub fn enable_environment_objects(&self, env_objects_ids: &[u64], enable: bool) {
        ffi::World_EnableEnvironmentObjects(&self.inner, env_objects_ids, enable);
    }

    /// Reset all traffic lights to their initial state
    pub fn reset_all_traffic_lights(&self) {
        ffi::World_ResetAllTrafficLights(&self.inner);
    }

    /// Freeze or unfreeze all traffic lights
    pub fn freeze_all_traffic_lights(&self, frozen: bool) {
        ffi::World_FreezeAllTrafficLights(&self.inner, frozen);
    }

    /// Get light states of all vehicles
    pub fn get_vehicles_light_states(&self) -> Vec<crate::ffi::bridge::SimpleBatchCommand> {
        ffi::World_GetVehiclesLightStates(&self.inner)
    }

    /// Apply color texture to a named object
    pub fn apply_color_texture_to_object(
        &self,
        object_name: &str,
        texture: &crate::ffi::bridge::SimpleTextureColor,
        material_type: u8,
    ) {
        ffi::World_ApplyColorTextureToObject(&self.inner, object_name, texture, material_type);
    }

    /// Apply HDR float color texture to a named object
    pub fn apply_float_color_texture_to_object(
        &self,
        object_name: &str,
        texture: &crate::ffi::bridge::SimpleTextureFloatColor,
        material_type: u8,
    ) {
        ffi::World_ApplyFloatColorTextureToObject(&self.inner, object_name, texture, material_type);
    }

    /// Get names of all objects that can have textures applied
    pub fn get_names_of_all_objects(&self) -> Vec<String> {
        ffi::World_GetNamesOfAllObjects(&self.inner)
    }

    /// Set the random seed for pedestrian navigation
    pub fn set_pedestrians_seed(&self, seed: u32) {
        ffi::World_SetPedestriansSeed(&self.inner, seed);
    }

    /// Get the light manager for controlling lights in the world
    pub fn get_light_manager(&self) -> crate::light_manager::LightManagerWrapper {
        let light_manager_ptr = ffi::bridge::World_GetLightManager(&self.inner);
        crate::light_manager::LightManagerWrapper::new(light_manager_ptr)
    }
}

/// High-level wrapper for CARLA Actor
pub struct ActorWrapper {
    inner: SharedPtr<Actor>,
}

impl std::fmt::Debug for ActorWrapper {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ActorWrapper")
            .field("inner", &"<SharedPtr<Actor>>")
            .finish()
    }
}

impl Clone for ActorWrapper {
    fn clone(&self) -> Self {
        Self {
            inner: self.inner.clone(),
        }
    }
}

impl ActorWrapper {
    /// Create a new ActorWrapper from a SharedPtr<Actor>
    pub fn new(inner: SharedPtr<Actor>) -> Self {
        Self { inner }
    }

    /// Get reference to the inner Actor for casting
    pub fn get_actor(&self) -> &Actor {
        &self.inner
    }

    /// Get the SharedPtr<Actor> for casting operations
    pub fn get_shared_ptr(&self) -> SharedPtr<Actor> {
        self.inner.clone()
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

impl std::fmt::Debug for BlueprintLibraryWrapper {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("BlueprintLibraryWrapper")
            .field("inner", &"<SharedPtr<BlueprintLibrary>>")
            .finish()
    }
}

impl BlueprintLibraryWrapper {
    /// Find a blueprint by ID
    pub fn find(&self, id: &str) -> Option<crate::actor_blueprint::ActorBlueprintWrapper> {
        let blueprint_ptr = ffi::BlueprintLibrary_Find(&self.inner, id);
        if blueprint_ptr.is_null() {
            None
        } else {
            Some(crate::actor_blueprint::ActorBlueprintWrapper::new(
                blueprint_ptr,
            ))
        }
    }

    /// Get all blueprints in the library
    pub fn get_all(&self) -> Vec<crate::actor_blueprint::ActorBlueprintWrapper> {
        let blueprint_list = ffi::BlueprintLibrary_GetAll(&self.inner);
        blueprint_list
            .blueprint_ids
            .into_iter()
            .filter_map(|id| self.find(&id))
            .collect()
    }

    /// Filter blueprints by tags (all tags must match)
    pub fn filter_by_tags(
        &self,
        tags: &[&str],
    ) -> Vec<crate::actor_blueprint::ActorBlueprintWrapper> {
        let tags_vec: Vec<String> = tags.iter().map(|&s| s.to_string()).collect();
        let blueprint_list = ffi::BlueprintLibrary_FilterByTags(&self.inner, tags_vec);
        blueprint_list
            .blueprint_ids
            .into_iter()
            .filter_map(|id| self.find(&id))
            .collect()
    }

    /// Filter blueprints by attribute name and value
    pub fn filter_by_attribute(
        &self,
        attribute_name: &str,
        attribute_value: &str,
    ) -> Vec<crate::actor_blueprint::ActorBlueprintWrapper> {
        let blueprint_list =
            ffi::BlueprintLibrary_FilterByAttribute(&self.inner, attribute_name, attribute_value);
        blueprint_list
            .blueprint_ids
            .into_iter()
            .filter_map(|id| self.find(&id))
            .collect()
    }

    /// Search blueprints by ID substring
    pub fn search(&self, search_term: &str) -> Vec<crate::actor_blueprint::ActorBlueprintWrapper> {
        let blueprint_list = ffi::BlueprintLibrary_Search(&self.inner, search_term);
        blueprint_list
            .blueprint_ids
            .into_iter()
            .filter_map(|id| self.find(&id))
            .collect()
    }

    /// Get the number of blueprints in the library
    pub fn size(&self) -> usize {
        ffi::BlueprintLibrary_Size(&self.inner)
    }

    /// Filter blueprints by wildcard pattern
    pub fn filter(
        &self,
        wildcard_pattern: &str,
    ) -> Vec<crate::actor_blueprint::ActorBlueprintWrapper> {
        let blueprint_list = ffi::BlueprintLibrary_Filter(&self.inner, wildcard_pattern);
        blueprint_list
            .blueprint_ids
            .into_iter()
            .filter_map(|id| self.find(&id))
            .collect()
    }
}
