use super::{
    Actor, ActorBase, ActorBlueprint, ActorBuilder, ActorList, ActorVec, BlueprintLibrary,
    BoundingBoxList, EnvironmentObjectList, LabelledPointList, Landmark, LightManager, Map,
    Waypoint, WorldSnapshot,
};
use crate::{
    geom::{Location, LocationExt, Transform, TransformExt, Vector3D, Vector3DExt},
    road::JuncId,
    rpc::{
        ActorId, AttachmentType, EpisodeSettings, LabelledPoint, MapLayer, VehicleLightStateList,
        WeatherParameters,
    },
    utils::{check_carla_error, rust_string_to_c, c_string_to_rust, ArrayConversionExt},
};
use anyhow::{anyhow, Result};
use carla_sys::*;
use nalgebra::{Isometry3, Translation3, Vector3};
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

    pub fn map(&self) -> Result<Map> {
        let map_ptr = unsafe { carla_world_get_map(self.inner) };
        if map_ptr.is_null() {
            return Err(anyhow!("Failed to get map from world"));
        }
        Map::from_raw_ptr(map_ptr)
    }

    pub fn light_manager(&self) -> Result<LightManager> {
        // Note: Light Manager is deprecated in CARLA 0.10.0
        Err(anyhow!("Light Manager API is deprecated in CARLA 0.10.0"))
    }

    pub fn load_level_layer(&self, map_layers: MapLayer) -> Result<()> {
        // Note: Level layer loading needs to be implemented in C wrapper
        Err(anyhow!("Level layer loading not yet implemented in C wrapper"))
    }

    pub fn unload_level_layer(&self, map_layers: MapLayer) -> Result<()> {
        // Note: Level layer unloading needs to be implemented in C wrapper
        Err(anyhow!("Level layer unloading not yet implemented in C wrapper"))
    }

    pub fn blueprint_library(&self) -> Result<BlueprintLibrary> {
        let bp_lib_ptr = unsafe { carla_world_get_blueprint_library(self.inner) };
        if bp_lib_ptr.is_null() {
            return Err(anyhow!("Failed to get blueprint library"));
        }
        BlueprintLibrary::from_raw_ptr(bp_lib_ptr)
    }

    pub fn vehicle_light_states(&self) -> Result<VehicleLightStateList> {
        // Note: Vehicle light states need to be implemented in C wrapper
        Err(anyhow!("Vehicle light states not yet implemented in C wrapper"))
    }

    pub fn random_location_from_navigation(&self) -> Result<Translation3<f32>> {
        let transform_list_ptr = unsafe { carla_world_get_random_location_from_navigation(self.inner) };
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

    pub fn spectator(&self) -> Result<Actor> {
        let actor_ptr = unsafe { carla_world_get_spectator(self.inner) };
        if actor_ptr.is_null() {
            return Err(anyhow!("Failed to get spectator actor"));
        }
        Actor::from_raw_ptr(actor_ptr)
    }

    pub fn settings(&self) -> EpisodeSettings {
        let ptr = self.inner.GetSettings().within_unique_ptr();
        EpisodeSettings::from_cxx(&ptr)
    }

    pub fn snapshot(&self) -> Result<WorldSnapshot> {
        let snapshot = unsafe { carla_world_get_snapshot(self.inner) };
        WorldSnapshot::from_c_snapshot(snapshot)
    }

    pub fn names_of_all_objects(&self) -> Vec<String> {
        self.inner
            .GetNamesOfAllObjects()
            .iter()
            .map(|s| s.to_string())
            .collect()
    }

    pub fn actor(&self, actor_id: ActorId) -> Option<Actor> {
        let actor_ptr = unsafe { carla_world_get_actor(self.inner, actor_id) };
        if actor_ptr.is_null() {
            None
        } else {
            Actor::from_raw_ptr(actor_ptr).ok()
        }
    }

    pub fn actors(&self) -> Result<ActorList> {
        let actor_list_ptr = unsafe { carla_world_get_actors(self.inner) };
        if actor_list_ptr.is_null() {
            return Err(anyhow!("Failed to get actors from world"));
        }
        ActorList::from_raw_ptr(actor_list_ptr)
    }

    pub fn actors_by_ids(&self, ids: &[ActorId]) -> Result<ActorList> {
        let actor_list_ptr = unsafe {
            carla_world_get_actors_by_id(self.inner, ids.as_ptr(), ids.len())
        };
        if actor_list_ptr.is_null() {
            return Err(anyhow!("Failed to get actors by IDs"));
        }
        ActorList::from_raw_ptr(actor_list_ptr)
    }

    pub fn traffic_lights_from_waypoint(&self, waypoint: &Waypoint, distance: f64) -> ActorVec {
        let ptr = self
            .inner
            .GetTrafficLightsFromWaypoint(&waypoint.inner, distance)
            .within_unique_ptr();
        ActorVec::from_cxx(ptr).unwrap()
    }

    pub fn traffic_lights_in_junction(&self, junc_id: JuncId) -> ActorVec {
        let ptr = self
            .inner
            .GetTrafficLightsInJunction(junc_id)
            .within_unique_ptr();
        ActorVec::from_cxx(ptr).unwrap()
    }

    pub fn apply_settings(&mut self, settings: &EpisodeSettings, timeout: Duration) -> u64 {
        let settings = settings.to_cxx();
        let millis = timeout.as_millis() as usize;
        self.inner.pin_mut().ApplySettings(&settings, millis)
    }

    pub fn spawn_actor(
        &mut self,
        blueprint: &ActorBlueprint,
        transform: &Isometry3<f32>,
    ) -> Result<Actor> {
        self.spawn_actor_opt::<Actor, _>(blueprint, transform, None, None)
    }

    pub fn spawn_actor_opt<A, T>(
        &mut self,
        blueprint: &ActorBlueprint,
        transform: &Isometry3<f32>,
        parent: Option<&A>,
        attachment_type: T,
    ) -> Result<Actor>
    where
        A: ActorBase,
        T: Into<Option<AttachmentType>>,
    {
        let parent_ptr = parent
            .map(|parent| parent.raw_ptr())
            .unwrap_or(ptr::null_mut());
        
        let c_transform = Transform::from_na(transform).to_c_transform();
        
        let spawn_result = unsafe {
            carla_world_spawn_actor(
                self.inner,
                blueprint.raw_ptr(),
                &c_transform,
                parent_ptr,
            )
        };
        
        check_carla_error(spawn_result.error)?;
        
        if spawn_result.actor.is_null() {
            return Err(anyhow!("Failed to spawn actor - null actor returned"));
        }
        
        Actor::from_raw_ptr(spawn_result.actor)
    }
    
    pub fn try_spawn_actor(
        &mut self,
        blueprint: &ActorBlueprint,
        transform: &Isometry3<f32>,
    ) -> Result<Option<Actor>> {
        self.try_spawn_actor_opt::<Actor, _>(blueprint, transform, None, None)
    }
    
    pub fn try_spawn_actor_opt<A, T>(
        &mut self,
        blueprint: &ActorBlueprint,
        transform: &Isometry3<f32>,
        parent: Option<&A>,
        attachment_type: T,
    ) -> Result<Option<Actor>>
    where
        A: ActorBase,
        T: Into<Option<AttachmentType>>,
    {
        let parent_ptr = parent
            .map(|parent| parent.raw_ptr())
            .unwrap_or(ptr::null_mut());
        
        let c_transform = Transform::from_na(transform).to_c_transform();
        
        let spawn_result = unsafe {
            carla_world_try_spawn_actor(
                self.inner,
                blueprint.raw_ptr(),
                &c_transform,
                parent_ptr,
            )
        };
        
        match spawn_result.error {
            carla_error_t_CARLA_ERROR_NONE => {
                if spawn_result.actor.is_null() {
                    Ok(None)
                } else {
                    Ok(Some(Actor::from_raw_ptr(spawn_result.actor)?))
                }
            }
            _ => {
                // For try_spawn, we return None instead of error for some cases
                Ok(None)
            }
        }
    }

    pub fn wait_for_tick(&self) -> Result<WorldSnapshot> {
        loop {
            if let Some(snapshot) = self.wait_for_tick_or_timeout(DEFAULT_TICK_TIMEOUT)? {
                return Ok(snapshot);
            }
        }
    }

    pub fn actor_builder(&mut self, key: &str) -> Result<ActorBuilder<'_>> {
        ActorBuilder::new(self, key)
    }

    #[must_use]
    pub fn wait_for_tick_or_timeout(&self, timeout: Duration) -> Result<Option<WorldSnapshot>> {
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

    pub fn set_pedestrians_cross_factor(&mut self, percentage: f32) {
        self.inner.pin_mut().SetPedestriansCrossFactor(percentage);
    }

    pub fn set_pedestrians_seed(&mut self, seed: usize) {
        let seed = c_uint(seed as std::os::raw::c_uint);
        self.inner.pin_mut().SetPedestriansSeed(seed);
    }

    pub fn traffic_sign_at(&self, landmark: &Landmark) -> Option<Actor> {
        let ptr = self.inner.GetTrafficSign(landmark.inner.as_ref().unwrap());
        Actor::from_cxx(ptr)
    }

    pub fn traffic_light_at(&self, landmark: &Landmark) -> Option<Actor> {
        let ptr = self.inner.GetTrafficLight(landmark.inner.as_ref().unwrap());
        Actor::from_cxx(ptr)
    }

    pub fn traffic_light_from_open_drive(&self, sign_id: &str) -> Option<Actor> {
        let_cxx_string!(sign_id = sign_id);
        let ptr = self.inner.GetTrafficLightFromOpenDRIVE(&sign_id);
        Actor::from_cxx(ptr)
    }

    pub fn freeze_all_traffic_lights(&mut self, frozen: bool) {
        self.inner.pin_mut().FreezeAllTrafficLights(frozen);
    }

    pub fn reset_all_traffic_lights(&mut self) {
        self.inner.pin_mut().ResetAllTrafficLights();
    }

    pub fn level_bounding_boxes(&self, queried_tag: u8) -> BoundingBoxList {
        let ptr = self.inner.GetLevelBBs(queried_tag);
        BoundingBoxList::from_cxx(ptr).unwrap()
    }

    pub fn weather(&self) -> WeatherParameters {
        let c_weather = unsafe { carla_world_get_weather(self.inner) };
        WeatherParameters::from_c_weather(c_weather)
    }

    pub fn set_weather(&mut self, weather: &WeatherParameters) {
        let c_weather = weather.to_c_weather();
        unsafe { carla_world_set_weather(self.inner, &c_weather) };
    }

    pub fn environment_objects(&self, queried_tag: u8) -> EnvironmentObjectList {
        let ptr = self
            .inner
            .GetEnvironmentObjects(queried_tag)
            .within_unique_ptr();
        EnvironmentObjectList::from_cxx(ptr).unwrap()
    }

    pub fn enable_environment_objects(&self, ids: &[u64], enable: bool) {
        let ptr = ids.as_ptr();
        let len = ids.len();
        unsafe {
            self.inner.EnableEnvironmentObjects(ptr, len, enable);
        }
    }

    pub fn project_point(
        &self,
        location: &Translation3<f32>,
        direction: &Vector3<f32>,
        search_distance: f32,
    ) -> Option<LabelledPoint> {
        let ptr = self.inner.ProjectPoint(
            Location::from_na_translation(location),
            Vector3D::from_na(direction),
            search_distance,
        );

        if ptr.is_null() {
            None
        } else {
            Some((*ptr).clone())
        }
    }

    pub fn ground_projection(
        &self,
        location: &Translation3<f32>,
        search_distance: f32,
    ) -> Option<LabelledPoint> {
        let ptr = self
            .inner
            .GroundProjection(Location::from_na_translation(location), search_distance);

        if ptr.is_null() {
            None
        } else {
            Some((*ptr).clone())
        }
    }

    pub fn cast_ray(
        &self,
        start: &Translation3<f32>,
        end: &Translation3<f32>,
    ) -> LabelledPointList {
        let ptr = self
            .inner
            .CastRay(
                Location::from_na_translation(start),
                Location::from_na_translation(end),
            )
            .within_unique_ptr();
        LabelledPointList::from_cxx(ptr).unwrap()
    }

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
