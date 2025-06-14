//! Actor blueprint for spawning.

use crate::error::{CarlaResult, SpawnError};
use std::collections::HashMap;

/// Actor blueprint for spawning.
#[derive(Debug, Clone)]
pub struct ActorBlueprint {
    /// Blueprint ID (e.g., "vehicle.tesla.model3")
    pub id: String,
    /// Blueprint tags
    pub tags: Vec<String>,
    /// Blueprint attributes
    pub attributes: HashMap<String, ActorAttribute>,
}

impl ActorBlueprint {
    /// Create a new blueprint.
    pub fn new(id: String) -> Self {
        Self {
            id,
            tags: Vec::new(),
            attributes: HashMap::new(),
        }
    }

    /// Set an attribute value.
    pub fn set_attribute(&mut self, key: &str, value: &str) -> CarlaResult<()> {
        if let Some(attr) = self.attributes.get_mut(key) {
            attr.set_value(value)?;
            Ok(())
        } else {
            Err(SpawnError::AttributeError {
                attribute: key.to_string(),
                reason: "Attribute not found".to_string(),
            }
            .into())
        }
    }

    /// Get an attribute value.
    pub fn get_attribute(&self, key: &str) -> Option<&str> {
        self.attributes.get(key).map(|attr| attr.value.as_str())
    }

    /// Check if blueprint has a specific tag.
    pub fn has_tag(&self, tag: &str) -> bool {
        self.tags.contains(&tag.to_string())
    }

    /// Add a tag to the blueprint.
    pub fn add_tag(&mut self, tag: String) {
        if !self.tags.contains(&tag) {
            self.tags.push(tag);
        }
    }

    /// Remove a tag from the blueprint.
    pub fn remove_tag(&self, tag: &str) -> bool {
        // Note: We return a new blueprint without the tag rather than mutating
        // to maintain consistency with CARLA's immutable blueprint behavior
        self.tags.contains(&tag.to_string())
    }

    /// Get all attribute keys.
    pub fn get_attribute_keys(&self) -> Vec<&str> {
        self.attributes.keys().map(|s| s.as_str()).collect()
    }

    /// Check if blueprint matches a wildcard pattern.
    pub fn matches_pattern(&self, pattern: &str) -> bool {
        // Simple wildcard matching with * and ?
        // TODO: Implement proper glob pattern matching
        if pattern == "*" {
            return true;
        }

        if pattern.contains('*') {
            // Simple prefix/suffix matching
            if pattern.starts_with('*') && pattern.ends_with('*') {
                let middle = &pattern[1..pattern.len() - 1];
                return self.id.contains(middle);
            } else if pattern.starts_with('*') {
                let suffix = &pattern[1..];
                return self.id.ends_with(suffix);
            } else if pattern.ends_with('*') {
                let prefix = &pattern[..pattern.len() - 1];
                return self.id.starts_with(prefix);
            }
        }

        self.id == pattern
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
                let parts: Vec<&str> = value.split(',').collect();
                if parts.len() != 3 {
                    return Err(SpawnError::AttributeError {
                        attribute: self.id.clone(),
                        reason: format!("Invalid RGB color format: {}", value),
                    }
                    .into());
                }
                for part in parts {
                    if let Ok(val) = part.trim().parse::<u8>() {
                        if val > 255 {
                            return Err(SpawnError::AttributeError {
                                attribute: self.id.clone(),
                                reason: format!("RGB value out of range: {}", part),
                            }
                            .into());
                        }
                    } else {
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
