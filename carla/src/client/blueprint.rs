//! Actor blueprint for spawning.

use crate::error::{CarlaResult, SpawnError};
use carla_cxx::ActorBlueprintWrapper;
use std::collections::HashMap;

/// Actor blueprint for spawning.
#[derive(Debug, Clone)]
pub struct ActorBlueprint {
    /// Internal wrapper from carla-cxx
    inner: ActorBlueprintWrapper,
    /// Cached attributes (populated on demand)
    attributes_cache: Option<HashMap<String, ActorAttribute>>,
}

impl ActorBlueprint {
    /// Create a blueprint from a carla-cxx ActorBlueprintWrapper.
    pub fn new(inner: ActorBlueprintWrapper) -> Self {
        Self {
            inner,
            attributes_cache: None,
        }
    }

    /// Get the blueprint ID.
    pub fn get_id(&self) -> String {
        self.inner.get_id()
    }

    /// Get the blueprint tags.
    pub fn get_tags(&self) -> Vec<String> {
        self.inner.get_tags()
    }

    /// Get reference to the inner carla-cxx ActorBlueprint wrapper
    pub fn get_inner(&self) -> &ActorBlueprintWrapper {
        &self.inner
    }

    /// Set an attribute value.
    pub fn set_attribute(&mut self, key: &str, value: &str) -> CarlaResult<()> {
        if self.inner.contains_attribute(key) {
            self.inner.set_attribute(key, value);
            // Clear cache since attribute changed
            self.attributes_cache = None;
            Ok(())
        } else {
            Err(SpawnError::AttributeError {
                attribute: key.to_string(),
                reason: "Attribute not found".to_string(),
            }
            .into())
        }
    }

    /// Check if blueprint has a specific attribute.
    pub fn contains_attribute(&self, key: &str) -> bool {
        self.inner.contains_attribute(key)
    }

    /// Check if blueprint has a specific tag.
    pub fn has_tag(&self, tag: &str) -> bool {
        self.inner.contains_tag(tag)
    }

    /// Check if blueprint matches a wildcard pattern (for tags).
    pub fn matches_pattern(&self, pattern: &str) -> bool {
        self.inner.match_tags(pattern)
    }
}

/// Actor attribute for blueprint configuration.
#[derive(Debug, Clone)]
pub struct ActorAttribute {
    /// Attribute ID
    pub id: String,
    /// Attribute type
    pub attribute_type: ActorAttributeType,
    /// Current value
    pub value: String,
    /// Recommended values
    pub recommended_values: Vec<String>,
    /// Whether attribute is modifiable
    pub is_modifiable: bool,
}

impl ActorAttribute {
    /// Create a new attribute.
    pub fn new(
        id: String,
        attribute_type: ActorAttributeType,
        value: String,
        is_modifiable: bool,
    ) -> Self {
        Self {
            id,
            attribute_type,
            value,
            recommended_values: Vec::new(),
            is_modifiable,
        }
    }

    /// Set the attribute value.
    pub fn set_value(&mut self, value: &str) -> CarlaResult<()> {
        if !self.is_modifiable {
            return Err(SpawnError::AttributeError {
                attribute: self.id.clone(),
                reason: "Attribute is not modifiable".to_string(),
            }
            .into());
        }

        // TODO: Add type validation based on attribute_type
        todo!();
        self.value = value.to_string();
        Ok(())
    }

    /// Add a recommended value.
    pub fn add_recommended_value(&mut self, value: String) {
        if !self.recommended_values.contains(&value) {
            self.recommended_values.push(value);
        }
    }

    /// Check if a value is in the recommended values list.
    pub fn is_recommended_value(&self, value: &str) -> bool {
        self.recommended_values.contains(&value.to_string())
    }

    /// Validate value against type and constraints.
    pub fn validate_value(&self, value: &str) -> CarlaResult<()> {
        match self.attribute_type {
            ActorAttributeType::Bool => {
                if !["true", "false", "True", "False", "1", "0"].contains(&value) {
                    return Err(SpawnError::AttributeError {
                        attribute: self.id.clone(),
                        reason: format!("Invalid boolean value: {}", value),
                    }
                    .into());
                }
            }
            ActorAttributeType::Int => {
                if value.parse::<i64>().is_err() {
                    return Err(SpawnError::AttributeError {
                        attribute: self.id.clone(),
                        reason: format!("Invalid integer value: {}", value),
                    }
                    .into());
                }
            }
            ActorAttributeType::Float => {
                if value.parse::<f64>().is_err() {
                    return Err(SpawnError::AttributeError {
                        attribute: self.id.clone(),
                        reason: format!("Invalid float value: {}", value),
                    }
                    .into());
                }
            }
            ActorAttributeType::String => {
                // String values are always valid
            }
            ActorAttributeType::RGBColor => {
                // TODO: Validate RGB color format (e.g., "255,128,64")
                todo!();
                let parts: Vec<&str> = value.split(',').collect();
                if parts.len() != 3 {
                    return Err(SpawnError::AttributeError {
                        attribute: self.id.clone(),
                        reason: format!("Invalid RGB color format: {}", value),
                    }
                    .into());
                }
                for part in parts {
                    if part.trim().parse::<u8>().is_err() {
                        return Err(SpawnError::AttributeError {
                            attribute: self.id.clone(),
                            reason: format!("Invalid RGB component: {}", part),
                        }
                        .into());
                    }
                }
            }
        }
        Ok(())
    }
}

/// Actor attribute types.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ActorAttributeType {
    /// Boolean attribute
    Bool,
    /// Integer attribute
    Int,
    /// Float attribute
    Float,
    /// String attribute
    String,
    /// RGB color attribute
    RGBColor,
}
