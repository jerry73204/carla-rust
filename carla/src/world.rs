use autocxx::prelude::*;
use carla_sys::carla_rust::client::FfiWorld;
use cxx::UniquePtr;

use crate::Map;

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

    // pub fn blueprint_library(&self) -> BlueprintLibrary {
    //     BlueprintLibrary {
    //         inner: ffi::world_get_blueprint_library(&self.inner),
    //     }
    // }

    // pub fn spectator(&self) -> Actor {
    //     Actor {
    //         inner: ffi::world_get_spectator(&self.inner),
    //     }
    // }

    // pub fn try_spawn_actor(
    //     &mut self,
    //     blueprint: &ActorBlueprint,
    //     transform: &Isometry3<f32>,
    //     parent: Option<&Actor>,
    // ) -> Option<Actor> {
    //     unsafe {
    //         let parent_ptr: *const ffi::SharedActor = parent
    //             .and_then(|parent| parent.inner.as_ref())
    //             .map(|ref_| ref_ as *const _)
    //             .unwrap_or(ptr::null());
    //         let transform = ffi::Transform::from_na(transform);
    //         let actor = ffi::world_try_spawn_actor(
    //             self.inner.pin_mut(),
    //             &blueprint.inner,
    //             &transform,
    //             parent_ptr,
    //         );
    //         if actor.is_null() {
    //             None
    //         } else {
    //             Some(Actor { inner: actor })
    //         }
    //     }
    // }
}
