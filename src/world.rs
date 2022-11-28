use std::ptr;

use cxx::UniquePtr;
use nalgebra::Isometry3;

use crate::{actor::Actor, blueprint_library::BlueprintLibrary, ffi, map::Map, ActorBlueprint};

pub struct World {
    pub(crate) inner: UniquePtr<ffi::World>,
}

impl World {
    pub fn id(&self) -> u64 {
        self.inner.get_id()
    }

    pub fn map(&self) -> Map {
        Map {
            inner: ffi::world_get_map(&self.inner),
        }
    }

    pub fn blueprint_library(&self) -> BlueprintLibrary {
        BlueprintLibrary {
            inner: ffi::world_get_blueprint_library(&self.inner),
        }
    }

    pub fn spectator(&self) -> Actor {
        Actor {
            inner: ffi::world_get_spectator(&self.inner),
        }
    }

    pub fn try_spawn_actor(
        &mut self,
        blueprint: &ActorBlueprint,
        transform: &Isometry3<f32>,
        parent: Option<&Actor>,
    ) -> Option<Actor> {
        unsafe {
            let parent_ptr: *const ffi::SharedActor = parent
                .and_then(|parent| parent.inner.as_ref())
                .map(|ref_| ref_ as *const _)
                .unwrap_or(ptr::null());
            let transform = ffi::Transform::from_na(transform);
            let actor = ffi::world_try_spawn_actor(
                self.inner.pin_mut(),
                &blueprint.inner,
                &transform,
                parent_ptr,
            );
            if actor.is_null() {
                None
            } else {
                Some(Actor { inner: actor })
            }
        }
    }
}
