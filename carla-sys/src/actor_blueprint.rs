//! Actor blueprint wrapper for CARLA.

use crate::ffi::{self, ActorBlueprint};
use cxx::SharedPtr;

/// High-level wrapper for CARLA ActorBlueprint
#[derive(Clone)]
pub struct ActorBlueprintWrapper {
    inner: SharedPtr<ActorBlueprint>,
}

impl std::fmt::Debug for ActorBlueprintWrapper {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ActorBlueprintWrapper")
            .field("id", &self.get_id())
            .field("tags", &self.get_tags())
            .finish()
    }
}

impl ActorBlueprintWrapper {
    /// Create a new ActorBlueprintWrapper from a SharedPtr<ActorBlueprint>
    pub fn new(inner: SharedPtr<ActorBlueprint>) -> Self {
        Self { inner }
    }

    /// Get the blueprint's ID
    pub fn get_id(&self) -> String {
        ffi::ActorBlueprint_GetId(&*self.inner)
    }

    /// Get all tags associated with this blueprint
    pub fn get_tags(&self) -> Vec<String> {
        ffi::ActorBlueprint_GetTags(&*self.inner)
    }

    /// Check if the blueprint matches the given wildcard pattern
    pub fn match_tags(&self, wildcard_pattern: &str) -> bool {
        ffi::ActorBlueprint_MatchTags(&*self.inner, wildcard_pattern)
    }

    /// Check if the blueprint contains a specific tag
    pub fn contains_tag(&self, tag: &str) -> bool {
        ffi::ActorBlueprint_ContainsTag(&*self.inner, tag)
    }

    /// Check if the blueprint has a specific attribute
    pub fn contains_attribute(&self, id: &str) -> bool {
        ffi::ActorBlueprint_ContainsAttribute(&*self.inner, id)
    }

    /// Set an attribute value on the blueprint
    pub fn set_attribute(&self, id: &str, value: &str) {
        ffi::ActorBlueprint_SetAttribute(&*self.inner, id, value);
    }

    /// Get reference to the inner ActorBlueprint for FFI operations
    pub fn get_inner(&self) -> &ActorBlueprint {
        &self.inner
    }

    /// Get the inner SharedPtr for operations that need it
    pub fn get_shared_ptr(&self) -> &SharedPtr<ActorBlueprint> {
        &self.inner
    }

    /// Get all attribute IDs for this blueprint
    pub fn get_attribute_ids(&self) -> Vec<String> {
        ffi::ActorBlueprint_GetAttributeIds(&*self.inner)
    }

    /// Get the value of a specific attribute
    pub fn get_attribute(&self, id: &str) -> Option<String> {
        let value = ffi::ActorBlueprint_GetAttribute(&*self.inner, id);
        if value.is_empty() {
            None
        } else {
            Some(value)
        }
    }

    /// Get the type of a specific attribute
    pub fn get_attribute_type(&self, id: &str) -> u8 {
        ffi::ActorBlueprint_GetAttributeType(&*self.inner, id)
    }

    /// Check if a specific attribute is modifiable
    pub fn is_attribute_modifiable(&self, id: &str) -> bool {
        ffi::ActorBlueprint_IsAttributeModifiable(&*self.inner, id)
    }

    /// Get recommended values for a specific attribute
    pub fn get_attribute_recommended_values(&self, id: &str) -> Vec<String> {
        ffi::ActorBlueprint_GetAttributeRecommendedValues(&*self.inner, id)
    }

    /// Get the number of attributes in this blueprint
    pub fn get_attribute_count(&self) -> usize {
        ffi::ActorBlueprint_GetAttributeCount(&*self.inner)
    }
}
