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
    utils::CxxVectorExt,
};
use anyhow::{anyhow, Result};
use autocxx::prelude::*;
use carla_sys::carla_rust::client::{FfiActor, FfiWorld};
use cxx::{let_cxx_string, CxxVector, UniquePtr};
use derivative::Derivative;
use nalgebra::{Isometry3, Translation3, Vector3};
use static_assertions::assert_impl_all;
use std::{ptr, time::Duration};

const DEFAULT_TICK_TIMEOUT: Duration = Duration::from_secs(60);

/// The world contains the map and assets of a simulation,
/// corresponding to `carla.World` in Python API.
#[derive(Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct World {
    #[derivative(Debug = "ignore")]
    pub(crate) inner: UniquePtr<FfiWorld>,
}

impl World {
    pub fn id(&self) -> u64 {
        self.inner.GetId()
    }

    pub fn map(&self) -> Map {
        let ptr = self.inner.GetMap();
        Map::from_cxx(ptr).unwrap()
    }

    pub fn light_manager(&self) -> LightManager {
        LightManager::from_cxx(self.inner.GetLightManager()).unwrap()
    }

    pub fn load_level_layer(&self, map_layers: MapLayer) {
        self.inner.LoadLevelLayer(map_layers as u16);
    }

    pub fn unload_level_layer(&self, map_layers: MapLayer) {
        self.inner.UnloadLevelLayer(map_layers as u16);
    }

    pub fn blueprint_library(&self) -> BlueprintLibrary {
        let ptr = self.inner.GetBlueprintLibrary();
        BlueprintLibrary::from_cxx(ptr).unwrap()
    }

    pub fn vehicle_light_states(&self) -> VehicleLightStateList {
        let ptr = self.inner.GetVehiclesLightStates().within_unique_ptr();
        VehicleLightStateList::from_cxx(ptr).unwrap()
    }

    pub fn random_location_from_navigation(&self) -> Translation3<f32> {
        self.inner
            .GetRandomLocationFromNavigation()
            .to_na_translation()
    }

    pub fn spectator(&self) -> Actor {
        let actor = self.inner.GetSpectator();
        Actor::from_cxx(actor).unwrap()
    }

    pub fn settings(&self) -> EpisodeSettings {
        let ptr = self.inner.GetSettings().within_unique_ptr();
        EpisodeSettings::from_cxx(&ptr)
    }

    pub fn snapshot(&self) -> WorldSnapshot {
        let ptr = self.inner.GetSnapshot();
        WorldSnapshot::from_cxx(ptr).unwrap()
    }

    pub fn names_of_all_objects(&self) -> Vec<String> {
        self.inner
            .GetNamesOfAllObjects()
            .iter()
            .map(|s| s.to_string())
            .collect()
    }

    pub fn actor(&self, actor_id: ActorId) -> Option<Actor> {
        let ptr = self.inner.GetActor(actor_id);
        Actor::from_cxx(ptr)
    }

    pub fn actors(&self) -> ActorList {
        let ptr = self.inner.GetActors();
        ActorList::from_cxx(ptr).unwrap()
    }

    pub fn actors_by_ids(&self, ids: &[ActorId]) -> ActorList {
        let mut vec = CxxVector::new_typed();
        ids.iter().cloned().for_each(|id| {
            vec.pin_mut().push(id);
        });

        let ptr = self.inner.GetActorsByIds(&vec);
        ActorList::from_cxx(ptr).unwrap()
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
        let parent = parent.map(|parent| parent.cxx_actor());
        let attachment_type = attachment_type.into().unwrap_or(AttachmentType::Rigid);

        unsafe {
            let parent_ptr: *const FfiActor = parent
                .as_ref()
                .and_then(|parent| parent.as_ref())
                .map(|ref_| ref_ as *const _)
                .unwrap_or(ptr::null());
            let ffi_transform = Transform::from_na(transform);
            let actor = self.inner.pin_mut().TrySpawnActor(
                &blueprint.inner,
                &ffi_transform,
                parent_ptr,
                attachment_type,
            );
            Actor::from_cxx(actor)
                .ok_or_else(|| anyhow!("Unable to spawn an actor with transform {}", transform))
        }
    }

    pub fn wait_for_tick(&self) -> WorldSnapshot {
        loop {
            if let Some(snapshot) = self.wait_for_tick_or_timeout(DEFAULT_TICK_TIMEOUT) {
                return snapshot;
            }
        }
    }

    pub fn actor_builder(&mut self, key: &str) -> Result<ActorBuilder<'_>> {
        ActorBuilder::new(self, key)
    }

    #[must_use]
    pub fn wait_for_tick_or_timeout(&self, timeout: Duration) -> Option<WorldSnapshot> {
        let ptr = self.inner.WaitForTick(timeout.as_millis() as usize);
        WorldSnapshot::from_cxx(ptr)
    }

    pub fn tick_or_timeuot(&mut self, timeout: Duration) -> u64 {
        self.inner.pin_mut().Tick(timeout.as_millis() as usize)
    }

    pub fn tick(&mut self) -> u64 {
        self.tick_or_timeuot(DEFAULT_TICK_TIMEOUT)
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
        self.inner.GetWeather()
    }

    pub fn set_weather(&mut self, weather: &WeatherParameters) {
        self.inner.pin_mut().SetWeather(weather)
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

    pub(crate) fn from_cxx(ptr: UniquePtr<FfiWorld>) -> Option<World> {
        if ptr.is_null() {
            None
        } else {
            Some(Self { inner: ptr })
        }
    }
}

impl Clone for World {
    fn clone(&self) -> Self {
        Self::from_cxx(self.inner.clone().within_unique_ptr()).unwrap()
    }
}

assert_impl_all!(World: Sync, Send);
