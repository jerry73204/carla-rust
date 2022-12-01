use super::ActorBlueprint;
use autocxx::prelude::*;
use carla_sys::carla_rust::client::{copy_actor_blueprint, FfiBlueprintLibrary};
use cxx::{let_cxx_string, SharedPtr};

#[derive(Clone)]
#[repr(transparent)]
pub struct BlueprintLibrary {
    inner: SharedPtr<FfiBlueprintLibrary>,
}

impl BlueprintLibrary {
    pub(crate) fn from_cxx(ptr: SharedPtr<FfiBlueprintLibrary>) -> Option<Self> {
        if ptr.is_null() {
            None
        } else {
            Some(Self { inner: ptr })
        }
    }

    pub fn filter(&self, pattern: &str) -> Self {
        let_cxx_string!(pattern = pattern);
        let ptr = self.inner.filter(&pattern);
        Self::from_cxx(ptr).unwrap()
    }

    pub fn find(&self, key: &str) -> Option<ActorBlueprint> {
        let_cxx_string!(key = key);
        unsafe {
            let actor_bp = self.inner.find(&key).as_ref()?;
            let actor_bp = copy_actor_blueprint(actor_bp).within_unique_ptr();
            Some(ActorBlueprint::from_cxx(actor_bp).unwrap())
        }
    }

    pub fn get(&self, index: usize) -> Option<ActorBlueprint> {
        unsafe {
            let actor_bp = self.inner.at(index).as_ref()?;
            let actor_bp = copy_actor_blueprint(actor_bp).within_unique_ptr();
            Some(ActorBlueprint::from_cxx(actor_bp).unwrap())
        }
    }

    pub fn iter(&self) -> impl Iterator<Item = ActorBlueprint> + '_ {
        (0..self.len()).map(|idx| self.get(idx).unwrap())
    }

    pub fn len(&self) -> usize {
        self.inner.size()
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.inner.is_empty()
    }
}
