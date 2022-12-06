use super::{
    Actor, ActorBase, ActorBlueprint, ActorBuilder, ActorList, BlueprintLibrary, Landmark, Map,
    WorldSnapshot,
};
use crate::{
    geom::{Transform, TransformExt},
    rpc::{ActorId, AttachmentType, EpisodeSettings},
};
use anyhow::{anyhow, Result};
use autocxx::prelude::*;
use carla_sys::carla_rust::{
    client::{FfiActor, FfiWorld},
    new_vector_uint32_t,
};
use cxx::{let_cxx_string, UniquePtr};
use derivative::Derivative;
use nalgebra::Isometry3;
use static_assertions::assert_impl_all;
use std::{ptr, time::Duration};

const DEFAULT_TICK_TIMEOUT: Duration = Duration::from_secs(60);

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

    pub fn blueprint_library(&self) -> BlueprintLibrary {
        let ptr = self.inner.GetBlueprintLibrary();
        BlueprintLibrary::from_cxx(ptr).unwrap()
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
        let mut vec = new_vector_uint32_t();
        ids.iter().cloned().for_each(|id| {
            vec.pin_mut().push(id);
        });

        let ptr = self.inner.GetActorsByIds(&vec);
        ActorList::from_cxx(ptr).unwrap()
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
                parent_ptr as *mut _,
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

    pub fn traffic_sign(&self, landmark: &Landmark) -> Option<Actor> {
        let ptr = self.inner.GetTrafficSign(landmark.inner.as_ref().unwrap());
        Actor::from_cxx(ptr)
    }

    pub fn traffic_light(&self, landmark: &Landmark) -> Option<Actor> {
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

    // pub fn level_bounding_boxes(&self) -> Vec<>

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

assert_impl_all!(World: Send);
