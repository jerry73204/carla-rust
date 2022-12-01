use super::{Actor, ActorBase, ActorBlueprint, BlueprintLibrary, Map, WorldSnapshot};
use crate::{geom::Transform, rpc::AttachmentType};
use autocxx::prelude::*;
use carla_sys::carla_rust::client::{FfiActor, FfiWorld};
use cxx::UniquePtr;
use nalgebra::Isometry3;
use std::{ptr, time::Duration};

#[repr(transparent)]
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
        let ptr = self.inner.GetBlueprintLibrary();
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
    ) -> Option<Actor> {
        self.try_spawn_actor_opt::<Actor>(blueprint, transform, Default::default())
    }

    pub fn try_spawn_actor_opt<A>(
        &mut self,
        blueprint: &ActorBlueprint,
        transform: &Isometry3<f32>,
        options: TrySpawnActorOptions<'_, A>,
    ) -> Option<Actor>
    where
        A: ActorBase,
    {
        let TrySpawnActorOptions {
            parent,
            attachment_type,
        } = options;

        unsafe {
            let parent = parent.map(|parent| parent.cxx_actor());
            let parent_ptr: *const FfiActor = parent
                .as_ref()
                .and_then(|parent| parent.as_ref())
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

    pub fn wait_for_tick(&self) -> WorldSnapshot {
        const TIMEOUT: Duration = Duration::from_secs(60);
        loop {
            if let Some(snapshot) = self.wait_for_tick_or_timeout(TIMEOUT) {
                return snapshot;
            }
        }
    }

    #[must_use]
    pub fn wait_for_tick_or_timeout(&self, timeout: Duration) -> Option<WorldSnapshot> {
        let ptr = self.inner.WaitForTick(timeout.as_millis() as usize);
        if ptr.is_null() {
            None
        } else {
            Some(WorldSnapshot::from_cxx(ptr))
        }
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

#[derive(Clone)]
pub struct TrySpawnActorOptions<'a, A> {
    pub parent: Option<&'a A>,
    pub attachment_type: AttachmentType,
}

impl<'a, A> Default for TrySpawnActorOptions<'a, A> {
    fn default() -> Self {
        Self {
            parent: None,
            attachment_type: AttachmentType::Rigid,
        }
    }
}
