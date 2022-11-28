use cxx::UniquePtr;

use crate::{actor::Actor, blueprint_library::BlueprintLibrary, ffi, map::Map};

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
}
