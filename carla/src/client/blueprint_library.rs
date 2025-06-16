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
            Some(blueprint_wrapper) => Ok(Some(ActorBlueprint::from_cxx(blueprint_wrapper))),
            None => Ok(None),
        }
    }

    /// Filter blueprints by wildcard pattern.
    pub fn filter(&self, wildcard_pattern: &str) -> CarlaResult<Vec<ActorBlueprint>> {
        let blueprints = self.inner.filter(wildcard_pattern);
        Ok(blueprints.into_iter().map(ActorBlueprint::from_cxx).collect())
    }

    /// Get blueprints that have a specific attribute.
    pub fn filter_by_attribute(
        &self,
        attribute_name: &str,
        attribute_value: &str,
    ) -> CarlaResult<Vec<ActorBlueprint>> {
        let blueprints = self
            .inner
            .filter_by_attribute(attribute_name, attribute_value);
        Ok(blueprints.into_iter().map(ActorBlueprint::from_cxx).collect())
    }

    /// Get the number of blueprints in the library.
    pub fn len(&self) -> usize {
        self.inner.size()
    }

    /// Get an iterator over all blueprints in the library.
    pub fn iter(&self) -> impl Iterator<Item = ActorBlueprint> {
        self.inner.get_all().into_iter().map(ActorBlueprint::from_cxx)
    }
}
