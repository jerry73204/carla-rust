use std::ptr;

use crate::{AttachmentType, Transform};
use autocxx::prelude::*;
use carla_sys::carla_rust::client::{FfiActor, FfiWorld};
use cxx::UniquePtr;
use nalgebra::Isometry3;

use crate::{Actor, ActorBlueprint, BlueprintLibrary, Map};

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
        let actor = self.inner.GetSpectator().within_unique_ptr();
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

    pub(crate) fn from_cxx(from: UniquePtr<FfiWorld>) -> World {
        Self { inner: from }
    }
}