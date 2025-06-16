//! Blueprint library containing all available actor blueprints.

use crate::{client::ActorBlueprint, error::CarlaResult};

/// Blueprint library containing all available actor blueprints.
#[derive(Debug)]
pub struct BlueprintLibrary {
    /// Internal handle to carla-cxx BlueprintLibrary
    inner: carla_cxx::BlueprintLibraryWrapper,
}

impl BlueprintLibrary {
    /// Create a new BlueprintLibrary from a carla-cxx BlueprintLibraryWrapper.
    pub fn from_cxx(inner: carla_cxx::BlueprintLibraryWrapper) -> Self {
        Self { inner }
    }

    /// Find a blueprint by ID.
    pub fn find(&self, blueprint_id: &str) -> CarlaResult<Option<ActorBlueprint>> {
        match self.inner.find(blueprint_id) {
            Some(blueprint_wrapper) => Ok(Some(ActorBlueprint::new(blueprint_wrapper))),
            None => Ok(None),
        }
    }

    /// Filter blueprints by wildcard pattern.
    pub fn filter(&self, wildcard_pattern: &str) -> CarlaResult<Self> {
        todo!("BlueprintLibrary::filter - missing FFI for wildcard pattern matching")
    }

    // TODO: The method is not supported in C++ library. Remove it and related FFI items.
    // /// Filter blueprints by tags.
    // pub fn filter_by_tags(&self, tags: &[&str]) -> CarlaResult<Vec<ActorBlueprint>> {
    //     let blueprints = self.inner.filter_by_tags(tags);
    //     Ok(blueprints.into_iter().map(ActorBlueprint::new).collect())
    // }

    /// Get blueprints that have a specific attribute.
    pub fn filter_by_attribute(
        &self,
        attribute_name: &str,
        attribute_value: Option<&str>,
    ) -> CarlaResult<Self> {
        todo!()
    }

    /// Get the number of blueprints in the library.
    pub fn len(&self) -> usize {
        self.inner.size()
    }

    // TODO: return an iterator of ActorBlueprint
    pub fn iter(&self) {
        todo!()
    }
}
