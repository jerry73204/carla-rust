// SAFETY: This module uses unwrap_unchecked() for performance on methods guaranteed
// to never return null. See UNWRAP_REPLACEMENTS.md for detailed C++ code audit.

use super::ActorBlueprint;
use autocxx::prelude::*;
use carla_sys::carla_rust::client::{copy_actor_blueprint, FfiBlueprintLibrary};
use cxx::{let_cxx_string, SharedPtr};
use derivative::Derivative;
use static_assertions::assert_impl_all;

/// Provides blueprints used to spawn actors, corresponding to
/// `carla.BlueprintLibrary` in Python API.
#[derive(Clone, Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct BlueprintLibrary {
    #[derivative(Debug = "ignore")]
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
        unsafe { Self::from_cxx(ptr).unwrap_unchecked() }
    }

    pub fn find(&self, key: &str) -> Option<ActorBlueprint> {
        let_cxx_string!(key = key);
        unsafe {
            let actor_bp = self.inner.find(&key).as_ref()?;
            let actor_bp = copy_actor_blueprint(actor_bp).within_unique_ptr();
            Some(ActorBlueprint::from_cxx(actor_bp).unwrap_unchecked())
        }
    }

    pub fn get(&self, index: usize) -> Option<ActorBlueprint> {
        unsafe {
            let actor_bp = self.inner.at(index).as_ref()?;
            let actor_bp = copy_actor_blueprint(actor_bp).within_unique_ptr();
            Some(ActorBlueprint::from_cxx(actor_bp).unwrap_unchecked())
        }
    }

    pub fn iter(&self) -> impl Iterator<Item = ActorBlueprint> + '_ {
        // SAFETY: Index is bounds-checked by (0..self.len()), so get() always returns Some
        (0..self.len()).map(|idx| unsafe { self.get(idx).unwrap_unchecked() })
    }

    pub fn len(&self) -> usize {
        self.inner.size()
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.inner.is_empty()
    }
}

assert_impl_all!(BlueprintLibrary: Send, Sync);
