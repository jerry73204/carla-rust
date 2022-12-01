use crate::{geom::Transform, rpc::AttachmentType};
use autocxx::prelude::*;
use carla_sys::carla_rust::client::{FfiActor, FfiWorld};
use cxx::UniquePtr;
use nalgebra::Isometry3;
use std::{ptr, time::Duration};

use super::{Actor, ActorBlueprint, BlueprintLibrary, Map, WorldSnapshot};

pub struct World {
    pub(crate) inner: UniquePtr<FfiWorld>,
}

impl World {
    pub fn id(&self) -> u64 {
        self.inner.GetId()
    }

    pub fn map(&self) -> Map {
        Map {
            inner: self.inner.GetMap().within_unique_ptr(),
        }
    }

    pub fn blueprint_library(&self) -> BlueprintLibrary {
        let ptr = self.inner.GetBlueprintLibrary().within_unique_ptr();
        BlueprintLibrary::from_cxx(ptr)
    }

    pub fn spectator(&self) -> Actor {
        let actor = self.inner.GetSpectator();
        Actor::from_cxx(actor)
    }

    pub fn try_spawn_actor(
        &mut self,
        blueprint: &ActorBlueprint,
        transform: &Isometry3<f32>,
        parent: Option<&Actor>,
    ) -> Option<Actor> {
        self.try_spawn_actor_opt(blueprint, transform, parent, AttachmentType::Rigid)
    }

    pub fn try_spawn_actor_opt(
        &mut self,
        blueprint: &ActorBlueprint,
        transform: &Isometry3<f32>,
        parent: Option<&Actor>,
        attachment_type: AttachmentType,
    ) -> Option<Actor> {
        unsafe {
            let parent_ptr: *const FfiActor = parent
                .and_then(|parent| parent.inner.as_ref())
                .map(|ref_| ref_ as *const _)
                .unwrap_or(ptr::null());
            let transform = Transform::from_na(transform);
            let actor = self.inner.pin_mut().TrySpawnActor(
                &blueprint.inner,
                &transform.inner,
                parent_ptr as *mut _,
                attachment_type,
            );
            if actor.is_null() {
                None
            } else {
                Some(Actor { inner: actor })
            }
        }
    }

    pub fn wait_for_tick(&self, dur: Duration) -> WorldSnapshot {
        let ptr = self
            .inner
            .WaitForTick(dur.as_millis() as usize)
            .within_unique_ptr();
        WorldSnapshot::from_cxx(ptr)
    }

    pub fn tick(&mut self, dur: Duration) -> u64 {
        self.inner.pin_mut().Tick(dur.as_millis() as usize)
    }

    pub fn set_pedestrians_cross_factor(&mut self, percentage: f32) {
        self.inner.pin_mut().SetPedestriansCrossFactor(percentage);
    }

    pub fn set_pedestrians_seed(&mut self, seed: usize) {
        let seed = c_uint(seed as std::os::raw::c_uint);
        self.inner.pin_mut().SetPedestriansSeed(seed);
    }

    pub fn reset_all_traffic_lights(&mut self) {
        self.inner.pin_mut().ResetAllTrafficLights();
    }

    pub(crate) fn from_cxx(from: UniquePtr<FfiWorld>) -> World {
        Self { inner: from }
    }
}
