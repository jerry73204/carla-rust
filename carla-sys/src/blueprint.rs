//! Extension traits for CARLA ActorBlueprint functionality.

use crate::ffi::{self, ActorBlueprint};

/// Extension methods for ActorBlueprint
pub trait ActorBlueprintExt {
    /// Get the blueprint's ID
    fn get_id(&self) -> String;

    /// Get all tags associated with this blueprint
    fn get_tags(&self) -> Vec<String>;

    /// Check if the blueprint matches the given wildcard pattern
    fn match_tags(&self, wildcard_pattern: &str) -> bool;

    /// Check if the blueprint contains a specific tag
    fn contains_tag(&self, tag: &str) -> bool;

    /// Check if the blueprint has a specific attribute
    fn contains_attribute(&self, id: &str) -> bool;

    /// Set an attribute value on the blueprint
    fn set_attribute(&mut self, id: &str, value: &str);
}

impl ActorBlueprintExt for ActorBlueprint {
    fn get_id(&self) -> String {
        ffi::ActorBlueprint_GetId(self)
    }

    fn get_tags(&self) -> Vec<String> {
        ffi::ActorBlueprint_GetTags(self)
    }

    fn match_tags(&self, wildcard_pattern: &str) -> bool {
        ffi::ActorBlueprint_MatchTags(self, wildcard_pattern)
    }

    fn contains_tag(&self, tag: &str) -> bool {
        ffi::ActorBlueprint_ContainsTag(self, tag)
    }

    fn contains_attribute(&self, id: &str) -> bool {
        ffi::ActorBlueprint_ContainsAttribute(self, id)
    }

    fn set_attribute(&mut self, id: &str, value: &str) {
        ffi::ActorBlueprint_SetAttribute(self, id, value);
    }
}
