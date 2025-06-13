use crate::geom::{Transform, TransformExt};
use anyhow::{anyhow, Result};
use carla_sys::*;
use nalgebra::Translation3;
use std::{ptr, time::Duration};

const DEFAULT_TICK_TIMEOUT: Duration = Duration::from_secs(60);

/// The world contains the map and assets of a simulation,
/// corresponding to `carla.World` in Python API.
#[derive(Debug)]
pub struct World {
    pub(crate) inner: *mut carla_world_t,
}

impl World {
    /// Create a World from a raw C pointer.
    ///
    /// # Safety
    /// The pointer must be valid and not null.
    pub(crate) fn from_raw_ptr(ptr: *mut carla_world_t) -> Result<Self> {
        if ptr.is_null() {
            return Err(anyhow!("Null world pointer"));
        }
        Ok(Self { inner: ptr })
    }

    pub fn id(&self) -> u64 {
        unsafe { carla_world_get_id(self.inner) }
    }

    pub fn map(&self) -> Result<super::Map> {
        let map_ptr = unsafe { carla_world_get_map(self.inner) };
        if map_ptr.is_null() {
            return Err(anyhow!("Failed to get map from world"));
        }
        super::Map::from_raw_ptr(map_ptr)
    }

    pub fn light_manager(&self) -> Result<()> {
        // Note: Light Manager is deprecated in CARLA 0.10.0
        Err(anyhow!("Light Manager API is deprecated in CARLA 0.10.0"))
    }

    pub fn load_level_layer(&self, map_layers: u32) -> Result<()> {
        // Note: Level layer loading needs to be implemented in C wrapper
        Err(anyhow!(
            "Level layer loading not yet implemented in C wrapper"
        ))
    }

    pub fn unload_level_layer(&self, map_layers: u32) -> Result<()> {
        // Note: Level layer unloading needs to be implemented in C wrapper
        Err(anyhow!(
            "Level layer unloading not yet implemented in C wrapper"
        ))
    }

    pub fn blueprint_library(&self) -> Result<super::BlueprintLibrary> {
        let bp_lib_ptr = unsafe { carla_world_get_blueprint_library(self.inner) };
        if bp_lib_ptr.is_null() {
            return Err(anyhow!("Failed to get blueprint library"));
        }
        super::BlueprintLibrary::from_raw_ptr(bp_lib_ptr)
    }

    pub fn vehicle_light_states(&self) -> Result<()> {
        // Note: Vehicle light states need to be implemented in C wrapper
        Err(anyhow!(
            "Vehicle light states not yet implemented in C wrapper"
        ))
    }

    pub fn random_location_from_navigation(&self) -> Result<Translation3<f32>> {
        let transform_list_ptr =
            unsafe { carla_world_get_random_location_from_navigation(self.inner) };
        if transform_list_ptr.is_null() {
            return Err(anyhow!("Failed to get random location from navigation"));
        }

        let size = unsafe { carla_transform_list_size(transform_list_ptr) };
        if size == 0 {
            unsafe { carla_transform_list_free(transform_list_ptr as *mut _) };
            return Err(anyhow!("No random location available"));
        }

        let transform = unsafe { carla_transform_list_get(transform_list_ptr, 0) };
        unsafe { carla_transform_list_free(transform_list_ptr as *mut _) };

        Ok(Translation3::new(
            transform.location.x,
            transform.location.y,
            transform.location.z,
        ))
    }

    pub fn spectator(&self) -> Result<super::Actor> {
        let actor_ptr = unsafe { carla_world_get_spectator(self.inner) };
        if actor_ptr.is_null() {
            return Err(anyhow!("Failed to get spectator actor"));
        }
        super::Actor::from_raw_ptr(actor_ptr)
    }

    // TODO: Implement settings when EpisodeSettings is available
    /*
    pub fn settings(&self) -> EpisodeSettings {
        // C wrapper function needed
        unimplemented!("Episode settings not implemented in C wrapper yet")
    }
    */

    pub fn snapshot(&self) -> Result<carla_world_snapshot_t> {
        let snapshot = unsafe { carla_world_get_snapshot(self.inner) };
        Ok(snapshot)
    }

    // TODO: Implement when C wrapper provides this function
    /*
    pub fn names_of_all_objects(&self) -> Vec<String> {
        // C wrapper function needed
        Vec::new()
    }
    */

    pub fn actor(&self, actor_id: u32) -> Option<super::Actor> {
        let actor_ptr = unsafe { carla_world_get_actor(self.inner, actor_id) };
        if actor_ptr.is_null() {
            None
        } else {
            super::Actor::from_raw_ptr(actor_ptr).ok()
        }
    }

    // TODO: Uncomment when ActorList module is migrated
    /*
    pub fn actors(&self) -> Result<ActorList> {
        let actor_list_ptr = unsafe { carla_world_get_actors(self.inner) };
        if actor_list_ptr.is_null() {
            return Err(anyhow!("Failed to get actors from world"));
        }
        ActorList::from_raw_ptr(actor_list_ptr)
    }
    */

    // TODO: Uncomment when ActorList module is migrated
    /*
    pub fn actors_by_ids(&self, ids: &[u32]) -> Result<ActorList> {
        let actor_list_ptr = unsafe {
            carla_world_get_actors_by_id(self.inner, ids.as_ptr(), ids.len())
        };
        if actor_list_ptr.is_null() {
            return Err(anyhow!("Failed to get actors by IDs"));
        }
        ActorList::from_raw_ptr(actor_list_ptr)
    }
    */

    // TODO: Uncomment when Waypoint and Actor modules are migrated
    /*
    pub fn traffic_lights_from_waypoint(&self, waypoint: &Waypoint, distance: f64) -> ActorVec {
        // C wrapper function needed
        unimplemented!("Traffic lights from waypoint not implemented in C wrapper yet")
    }

    pub fn traffic_lights_in_junction(&self, junc_id: i32) -> ActorVec {
        // C wrapper function needed
        unimplemented!("Traffic lights in junction not implemented in C wrapper yet")
    }
    */

    // TODO: Uncomment when EpisodeSettings is migrated
    /*
    pub fn apply_settings(&mut self, settings: &EpisodeSettings, timeout: Duration) -> u64 {
        // C wrapper function needed
        unimplemented!("Apply settings not implemented in C wrapper yet")
    }
    */

    pub fn spawn_actor(
        &mut self,
        blueprint: &super::ActorBlueprint,
        transform: &Transform,
    ) -> Result<super::Actor> {
        self.spawn_actor_opt(blueprint, transform, None)
    }

    pub fn spawn_actor_opt(
        &mut self,
        blueprint: &super::ActorBlueprint,
        transform: &Transform,
        parent: Option<&super::Actor>,
    ) -> Result<super::Actor> {
        let parent_ptr = parent
            .map(|parent| parent.raw_ptr())
            .unwrap_or(ptr::null_mut());

        let c_transform = transform.to_c_transform();

        let spawn_result = unsafe {
            carla_world_spawn_actor(self.inner, blueprint.raw_ptr(), &c_transform, parent_ptr)
        };

        if spawn_result.error != carla_error_t_CARLA_ERROR_NONE {
            return Err(anyhow!(
                "Failed to spawn actor: error code {}",
                spawn_result.error
            ));
        }

        if spawn_result.actor.is_null() {
            return Err(anyhow!("Failed to spawn actor - null actor returned"));
        }

        super::Actor::from_raw_ptr(spawn_result.actor)
    }

    pub fn try_spawn_actor(
        &mut self,
        blueprint: &super::ActorBlueprint,
        transform: &Transform,
    ) -> Result<Option<super::Actor>> {
        self.try_spawn_actor_opt(blueprint, transform, None)
    }

    pub fn try_spawn_actor_opt(
        &mut self,
        blueprint: &super::ActorBlueprint,
        transform: &Transform,
        parent: Option<&super::Actor>,
    ) -> Result<Option<super::Actor>> {
        let parent_ptr = parent
            .map(|parent| parent.raw_ptr())
            .unwrap_or(ptr::null_mut());

        let c_transform = transform.to_c_transform();

        let spawn_result = unsafe {
            carla_world_try_spawn_actor(self.inner, blueprint.raw_ptr(), &c_transform, parent_ptr)
        };

        match spawn_result.error {
            carla_error_t_CARLA_ERROR_NONE => {
                if spawn_result.actor.is_null() {
                    Ok(None)
                } else {
                    Ok(Some(super::Actor::from_raw_ptr(spawn_result.actor)?))
                }
            }
            _ => {
                // For try_spawn, we return None instead of error for some cases
                Ok(None)
            }
        }
    }

    pub fn wait_for_tick(&self) -> Result<carla_world_snapshot_t> {
        loop {
            if let Some(snapshot) = self.wait_for_tick_or_timeout(DEFAULT_TICK_TIMEOUT)? {
                return Ok(snapshot);
            }
        }
    }

    // TODO: Uncomment when ActorBuilder is migrated
    /*
    pub fn actor_builder(&mut self, key: &str) -> Result<ActorBuilder<'_>> {
        ActorBuilder::new(self, key)
    }
    */

    #[must_use]
    pub fn wait_for_tick_or_timeout(
        &self,
        timeout: Duration,
    ) -> Result<Option<carla_world_snapshot_t>> {
        unsafe {
            carla_world_wait_for_tick(self.inner, timeout.as_millis() as u64);
        }
        // Return the current snapshot after waiting
        Ok(Some(self.snapshot()?))
    }

    pub fn tick_or_timeout(&mut self, timeout: Duration) -> u64 {
        unsafe { carla_world_tick(self.inner, timeout.as_millis() as u64) }
    }

    pub fn tick(&mut self) -> u64 {
        self.tick_or_timeout(DEFAULT_TICK_TIMEOUT)
    }

    // TODO: Implement when C wrapper provides these functions
    /*
    pub fn set_pedestrians_cross_factor(&mut self, percentage: f32) {
        // C wrapper function needed
        unimplemented!("Set pedestrians cross factor not implemented in C wrapper yet")
    }

    pub fn set_pedestrians_seed(&mut self, seed: usize) {
        // C wrapper function needed
        unimplemented!("Set pedestrians seed not implemented in C wrapper yet")
    }
    */

    // TODO: Uncomment when Landmark and Actor modules are migrated
    /*
    pub fn traffic_sign_at(&self, landmark: &Landmark) -> Option<Actor> {
        // C wrapper function needed
        unimplemented!("Traffic sign at landmark not implemented in C wrapper yet")
    }

    pub fn traffic_light_at(&self, landmark: &Landmark) -> Option<Actor> {
        // C wrapper function needed
        unimplemented!("Traffic light at landmark not implemented in C wrapper yet")
    }

    pub fn traffic_light_from_open_drive(&self, sign_id: &str) -> Option<Actor> {
        // C wrapper function needed
        unimplemented!("Traffic light from OpenDRIVE not implemented in C wrapper yet")
    }
    */

    // TODO: Implement when C wrapper provides these functions
    /*
    pub fn freeze_all_traffic_lights(&mut self, frozen: bool) {
        // C wrapper function needed
        unimplemented!("Freeze all traffic lights not implemented in C wrapper yet")
    }

    pub fn reset_all_traffic_lights(&mut self) {
        // C wrapper function needed
        unimplemented!("Reset all traffic lights not implemented in C wrapper yet")
    }
    */

    // TODO: Uncomment when BoundingBoxList is migrated
    /*
    pub fn level_bounding_boxes(&self, queried_tag: u8) -> BoundingBoxList {
        // C wrapper function needed
        unimplemented!("Level bounding boxes not implemented in C wrapper yet")
    }
    */

    pub fn weather(&self) -> carla_weather_parameters_t {
        unsafe { carla_world_get_weather(self.inner) }
    }

    pub fn set_weather(&mut self, weather: &carla_weather_parameters_t) {
        unsafe { carla_world_set_weather(self.inner, weather) };
    }

    // TODO: Uncomment when EnvironmentObjectList is migrated
    /*
    pub fn environment_objects(&self, queried_tag: u8) -> EnvironmentObjectList {
        // C wrapper function needed
        unimplemented!("Environment objects not implemented in C wrapper yet")
    }

    pub fn enable_environment_objects(&self, ids: &[u64], enable: bool) {
        // C wrapper function needed
        unimplemented!("Enable environment objects not implemented in C wrapper yet")
    }
    */

    // TODO: Uncomment when LabelledPoint is migrated
    /*
    pub fn project_point(
        &self,
        location: &Translation3<f32>,
        direction: &Vector3<f32>,
        search_distance: f32,
    ) -> Option<LabelledPoint> {
        // C wrapper function needed
        unimplemented!("Project point not implemented in C wrapper yet")
    }

    pub fn ground_projection(
        &self,
        location: &Translation3<f32>,
        search_distance: f32,
    ) -> Option<LabelledPoint> {
        // C wrapper function needed
        unimplemented!("Ground projection not implemented in C wrapper yet")
    }
    */

    // TODO: Uncomment when LabelledPointList is migrated
    /*
    pub fn cast_ray(
        &self,
        start: &Translation3<f32>,
        end: &Translation3<f32>,
    ) -> LabelledPointList {
        // C wrapper function needed
        unimplemented!("Cast ray not implemented in C wrapper yet")
    }
    */
}

impl Drop for World {
    fn drop(&mut self) {
        if !self.inner.is_null() {
            unsafe {
                carla_world_free(self.inner);
            }
            self.inner = ptr::null_mut();
        }
    }
}

// SAFETY: World wraps a thread-safe C API
unsafe impl Send for World {}
unsafe impl Sync for World {}
