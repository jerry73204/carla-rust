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
    pub fn new(inner: carla_cxx::BlueprintLibraryWrapper) -> Self {
        Self { inner }
    }

    /// Find a blueprint by ID.
    pub fn find(&self, blueprint_id: &str) -> CarlaResult<Option<ActorBlueprint>> {
        match self.inner.find(blueprint_id) {
            Some(blueprint_ptr) => {
                // TODO: Need ActorBlueprint wrapper integration
                let _blueprint_ptr = blueprint_ptr;
                todo!("ActorBlueprint wrapper from carla-cxx SharedPtr not yet implemented")
            }
            None => Ok(None),
        }
    }

    /// Filter blueprints by wildcard pattern.
    pub fn filter(&self, wildcard_pattern: &str) -> CarlaResult<Vec<ActorBlueprint>> {
        // TODO: Implement using carla-cxx FFI interface - CXX doesn't support Vec<SharedPtr<T>> return types
        let _wildcard_pattern = wildcard_pattern;
        todo!(
            "BlueprintLibrary::filter - missing FFI for Vec<SharedPtr<ActorBlueprint>> return type"
        )
    }

    /// Get all blueprints.
    pub fn get_all(&self) -> CarlaResult<Vec<ActorBlueprint>> {
        // TODO: Implement using carla-cxx FFI interface - missing FFI function
        todo!("BlueprintLibrary::get_all - missing FFI function")
    }

    /// Filter blueprints by tags.
    pub fn filter_by_tags(&self, tags: &[&str]) -> CarlaResult<Vec<ActorBlueprint>> {
        // TODO: Implement using carla-cxx FFI interface - missing FFI function
        let _tags = tags;
        todo!("BlueprintLibrary::filter_by_tags - missing FFI function")
    }

    /// Get all vehicle blueprints.
    pub fn get_vehicles(&self) -> CarlaResult<Vec<ActorBlueprint>> {
        self.filter("vehicle.*")
    }

    /// Get all walker blueprints.
    pub fn get_walkers(&self) -> CarlaResult<Vec<ActorBlueprint>> {
        self.filter("walker.*")
    }

    /// Get all sensor blueprints.
    pub fn get_sensors(&self) -> CarlaResult<Vec<ActorBlueprint>> {
        self.filter("sensor.*")
    }

    /// Get all static object blueprints.
    pub fn get_static_objects(&self) -> CarlaResult<Vec<ActorBlueprint>> {
        self.filter("static.*")
    }

    /// Get blueprints that have a specific attribute.
    pub fn filter_by_attribute(
        &self,
        attribute_name: &str,
        attribute_value: Option<&str>,
    ) -> CarlaResult<Vec<ActorBlueprint>> {
        // TODO: Implement using carla-cxx FFI interface - missing FFI function
        let _attribute_name = attribute_name;
        let _attribute_value = attribute_value;
        todo!("BlueprintLibrary::filter_by_attribute - missing FFI function")
    }

    /// Search blueprints by name substring.
    pub fn search(&self, search_term: &str) -> CarlaResult<Vec<ActorBlueprint>> {
        // TODO: Implement using carla-cxx FFI interface - missing FFI function
        let _search_term = search_term;
        todo!("BlueprintLibrary::search - missing FFI function")
    }

    /// Get the number of blueprints in the library.
    pub fn size(&self) -> usize {
        self.inner.size()
    }

    /// Check if a blueprint with the given ID exists.
    pub fn contains(&self, blueprint_id: &str) -> bool {
        self.inner.find(blueprint_id).is_some()
    }
}
