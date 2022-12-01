use super::ActorBlueprint;
use autocxx::prelude::*;
use carla_sys::carla_rust::client::{copy_actor_blueprint, FfiBlueprintLibrary};
use cxx::{let_cxx_string, UniquePtr};

#[repr(transparent)]
pub struct BlueprintLibrary {
    pub(crate) inner: UniquePtr<FfiBlueprintLibrary>,
}

impl BlueprintLibrary {
    pub(crate) fn from_cxx(ptr: UniquePtr<FfiBlueprintLibrary>) -> Self {
        Self { inner: ptr }
    }

    pub fn filter(&self, pattern: &str) -> Self {
        let_cxx_string!(pattern = pattern);
        let ptr = self.inner.filter(&pattern).within_unique_ptr();
        Self::from_cxx(ptr)
    }

    pub fn find(&self, key: &str) -> Option<ActorBlueprint> {
        let_cxx_string!(key = key);
        unsafe {
            let actor_bp = self.inner.find(&key).as_ref()?;
            let actor_bp = copy_actor_blueprint(actor_bp).within_unique_ptr();
            Some(ActorBlueprint::from_cxx(actor_bp))
        }
    }

    pub fn get(&self, index: usize) -> Option<ActorBlueprint> {
        unsafe {
            let actor_bp = self.inner.at(index).as_ref()?;
            let actor_bp = copy_actor_blueprint(actor_bp).within_unique_ptr();
            Some(ActorBlueprint::from_cxx(actor_bp))
        }
    }

    pub fn len(&self) -> usize {
        self.inner.size()
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.inner.is_empty()
    }
}
