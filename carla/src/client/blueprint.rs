//! Actor blueprint for spawning.

use crate::error::CarlaResult;
use carla_sys::ActorBlueprintWrapper;

/// Actor blueprint for spawning.
#[derive(Debug, Clone)]
pub struct ActorBlueprint {
    /// Internal wrapper from carla-sys
    inner: ActorBlueprintWrapper,
}

impl ActorBlueprint {
    /// Create a blueprint from a carla-sys ActorBlueprintWrapper.
    pub fn from_cxx(inner: ActorBlueprintWrapper) -> Self {
        Self { inner }
    }

    /// Get the blueprint ID.
    pub fn id(&self) -> String {
        self.inner.get_id()
    }

    /// Get the blueprint tags.
    pub fn tags(&self) -> Vec<String> {
        self.inner.get_tags()
    }

    /// Get reference to the inner carla-sys ActorBlueprint wrapper
    pub(crate) fn inner(&self) -> &ActorBlueprintWrapper {
        &self.inner
    }

    /// Get an entry to an attribute if the corresponding key exists.
    /// The entry can be used to get or set the value.
    pub fn attribute_entry(&mut self, key: &str) -> Option<ActorAttribute<'_>> {
        if self.contains_attribute(key) {
            Some(ActorAttribute::new(self, key.to_string()))
        } else {
            None
        }
    }

    /// Get the value of the attribute. Convenience method.
    pub fn attribute(&self, key: &str) -> Option<AttributeValue> {
        let value_str = self.inner.get_attribute(key)?;
        let attr_type = ActorAttributeType::from(self.inner.get_attribute_type(key));

        // Parse the string value based on the attribute type
        match attr_type {
            ActorAttributeType::Bool => value_str.parse::<bool>().ok().map(AttributeValue::Bool),
            ActorAttributeType::Int => value_str.parse::<i64>().ok().map(AttributeValue::Int),
            ActorAttributeType::Float => value_str.parse::<f64>().ok().map(AttributeValue::Float),
            ActorAttributeType::String => Some(AttributeValue::String(value_str)),
            ActorAttributeType::RGBColor => {
                // Parse RGB color from string format "r,g,b"
                let parts: Vec<&str> = value_str.split(',').collect();
                if parts.len() == 3 {
                    let r = parts[0].parse::<u8>().ok()?;
                    let g = parts[1].parse::<u8>().ok()?;
                    let b = parts[2].parse::<u8>().ok()?;
                    Some(AttributeValue::RGBColor(RGBColor::new(r, g, b)))
                } else {
                    None
                }
            }
        }
    }

    /// Set the value of the attribute. Convenience method.
    pub fn set_attribute<V>(&mut self, key: &str, value: V) -> CarlaResult<()>
    where
        V: Into<AttributeValue>,
    {
        if !self.inner.is_attribute_modifiable(key) {
            return Err(crate::error::CarlaError::Actor(
                crate::error::ActorError::AttributeNotModifiable(key.to_string()),
            ));
        }

        let attr_value = value.into();
        let value_str = match attr_value {
            AttributeValue::Bool(b) => b.to_string(),
            AttributeValue::Int(i) => i.to_string(),
            AttributeValue::Float(f) => f.to_string(),
            AttributeValue::String(s) => s,
            AttributeValue::RGBColor(color) => format!("{},{},{}", color.r, color.g, color.b),
        };

        self.inner.set_attribute(key, &value_str);
        Ok(())
    }

    /// Check if blueprint has a specific attribute.
    pub fn contains_attribute(&self, key: &str) -> bool {
        self.inner.contains_attribute(key)
    }

    /// Get all available attribute IDs for this blueprint.
    pub fn attribute_ids(&self) -> Vec<String> {
        self.inner.get_attribute_ids()
    }

    /// Get the number of attributes in this blueprint.
    pub fn attribute_count(&self) -> usize {
        self.inner.get_attribute_count()
    }

    /// Check if blueprint has a specific tag.
    pub fn has_tag(&self, tag: &str) -> bool {
        self.inner.contains_tag(tag)
    }

    /// Check if blueprint matches a wildcard pattern (for tags).
    pub fn matches_tags(&self, pattern: &str) -> bool {
        self.inner.match_tags(pattern)
    }
}

/// Actor attribute for blueprint configuration.
///
/// Provides access to an attribute of an actor blueprint, allowing
/// reading and modification of the attribute value.
#[derive(Debug)]
pub struct ActorAttribute<'a> {
    /// Reference to the parent blueprint
    blueprint: &'a mut ActorBlueprint,
    /// Attribute ID/key
    attribute_id: String,
}

impl<'a> ActorAttribute<'a> {
    /// Create a new ActorAttribute reference
    pub fn new(blueprint: &'a mut ActorBlueprint, attribute_id: String) -> Self {
        Self {
            blueprint,
            attribute_id,
        }
    }

    /// Get the attribute ID
    pub fn id(&self) -> &str {
        &self.attribute_id
    }

    /// Check if this attribute can be modified
    pub fn is_modifiable(&self) -> bool {
        self.blueprint
            .inner
            .is_attribute_modifiable(&self.attribute_id)
    }

    /// Get recommended values for this attribute
    pub fn recommended_values(&self) -> Vec<String> {
        self.blueprint
            .inner
            .get_attribute_recommended_values(&self.attribute_id)
    }

    /// Get the type of this attribute
    pub fn ty(&self) -> ActorAttributeType {
        let type_value = self.blueprint.inner.get_attribute_type(&self.attribute_id);
        ActorAttributeType::from(type_value)
    }

    /// Get the current value of this attribute
    pub fn get(&self) -> Option<AttributeValue> {
        let value_str = self.blueprint.inner.get_attribute(&self.attribute_id)?;
        let attr_type = self.ty();

        // Parse the string value based on the attribute type
        match attr_type {
            ActorAttributeType::Bool => value_str.parse::<bool>().ok().map(AttributeValue::Bool),
            ActorAttributeType::Int => value_str.parse::<i64>().ok().map(AttributeValue::Int),
            ActorAttributeType::Float => value_str.parse::<f64>().ok().map(AttributeValue::Float),
            ActorAttributeType::String => Some(AttributeValue::String(value_str)),
            ActorAttributeType::RGBColor => {
                // Parse RGB color from string format "r,g,b"
                let parts: Vec<&str> = value_str.split(',').collect();
                if parts.len() == 3 {
                    let r = parts[0].parse::<u8>().ok()?;
                    let g = parts[1].parse::<u8>().ok()?;
                    let b = parts[2].parse::<u8>().ok()?;
                    Some(AttributeValue::RGBColor(RGBColor::new(r, g, b)))
                } else {
                    None
                }
            }
        }
    }

    /// Set the value of this attribute
    pub fn set<V>(&mut self, value: V) -> CarlaResult<()>
    where
        V: Into<AttributeValue>,
    {
        if !self.is_modifiable() {
            return Err(crate::error::CarlaError::Actor(
                crate::error::ActorError::AttributeNotModifiable(self.attribute_id.clone()),
            ));
        }

        let attr_value = value.into();
        let value_str = match attr_value {
            AttributeValue::Bool(b) => b.to_string(),
            AttributeValue::Int(i) => i.to_string(),
            AttributeValue::Float(f) => f.to_string(),
            AttributeValue::String(s) => s,
            AttributeValue::RGBColor(color) => format!("{},{},{}", color.r, color.g, color.b),
        };

        self.blueprint
            .inner
            .set_attribute(&self.attribute_id, &value_str);
        Ok(())
    }
}

/// RGB color type for CARLA attributes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RGBColor {
    /// Red component (0-255)
    pub r: u8,
    /// Green component (0-255)  
    pub g: u8,
    /// Blue component (0-255)
    pub b: u8,
}

impl RGBColor {
    /// Create a new RGB color.
    pub fn new(r: u8, g: u8, b: u8) -> Self {
        Self { r, g, b }
    }
}

impl From<(u8, u8, u8)> for RGBColor {
    fn from((r, g, b): (u8, u8, u8)) -> Self {
        Self::new(r, g, b)
    }
}

impl From<carla_sys::SimpleColor> for RGBColor {
    fn from(color: carla_sys::SimpleColor) -> Self {
        Self::new(color.r, color.g, color.b)
    }
}

impl From<RGBColor> for carla_sys::SimpleColor {
    fn from(color: RGBColor) -> Self {
        Self {
            r: color.r,
            g: color.g,
            b: color.b,
        }
    }
}

/// Attribute value that can be assigned to an actor blueprint attribute.
#[derive(Debug, Clone, PartialEq)]
pub enum AttributeValue {
    /// Boolean value
    Bool(bool),
    /// Integer value
    Int(i64),
    /// Float value
    Float(f64),
    /// String value
    String(String),
    /// RGB color value
    RGBColor(RGBColor),
}

impl From<String> for AttributeValue {
    fn from(v: String) -> Self {
        Self::String(v)
    }
}

impl From<&str> for AttributeValue {
    fn from(v: &str) -> Self {
        Self::String(v.to_string())
    }
}

impl From<f64> for AttributeValue {
    fn from(v: f64) -> Self {
        Self::Float(v)
    }
}

impl From<i64> for AttributeValue {
    fn from(v: i64) -> Self {
        Self::Int(v)
    }
}

impl From<bool> for AttributeValue {
    fn from(v: bool) -> Self {
        Self::Bool(v)
    }
}

impl From<RGBColor> for AttributeValue {
    fn from(v: RGBColor) -> Self {
        Self::RGBColor(v)
    }
}

impl From<(u8, u8, u8)> for AttributeValue {
    fn from(v: (u8, u8, u8)) -> Self {
        Self::RGBColor(RGBColor::from(v))
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

impl From<u8> for ActorAttributeType {
    fn from(value: u8) -> Self {
        match value {
            0 => Self::Bool,
            1 => Self::Int,
            2 => Self::Float,
            3 => Self::String,
            4 => Self::RGBColor,
            _ => Self::String, // Default to String for unknown types
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_rgb_color_creation() {
        let color = RGBColor::new(255, 128, 64);
        assert_eq!(color.r, 255);
        assert_eq!(color.g, 128);
        assert_eq!(color.b, 64);
    }

    #[test]
    fn test_rgb_color_from_tuple() {
        let color = RGBColor::from((200, 100, 50));
        assert_eq!(color.r, 200);
        assert_eq!(color.g, 100);
        assert_eq!(color.b, 50);
    }

    #[test]
    fn test_rgb_color_common_colors() {
        let black = RGBColor::new(0, 0, 0);
        let white = RGBColor::new(255, 255, 255);
        let red = RGBColor::new(255, 0, 0);
        let green = RGBColor::new(0, 255, 0);
        let blue = RGBColor::new(0, 0, 255);

        assert_eq!(black.r + black.g + black.b, 0);
        assert_eq!(white.r as u16 + white.g as u16 + white.b as u16, 765);
        assert!(red.r > red.g && red.r > red.b);
        assert!(green.g > green.r && green.g > green.b);
        assert!(blue.b > blue.r && blue.b > blue.g);
    }

    #[test]
    fn test_attribute_value_variants() {
        let bool_attr = AttributeValue::Bool(true);
        let int_attr = AttributeValue::Int(42);
        let float_attr = AttributeValue::Float(std::f64::consts::PI);
        let string_attr = AttributeValue::String("test".to_string());
        let color_attr = AttributeValue::RGBColor(RGBColor::new(255, 128, 0));

        match bool_attr {
            AttributeValue::Bool(val) => assert!(val),
            _ => panic!("Expected Bool variant"),
        }

        match int_attr {
            AttributeValue::Int(val) => assert_eq!(val, 42),
            _ => panic!("Expected Int variant"),
        }

        match float_attr {
            AttributeValue::Float(val) => assert!((val - std::f64::consts::PI).abs() < 0.001),
            _ => panic!("Expected Float variant"),
        }

        match string_attr {
            AttributeValue::String(val) => assert_eq!(val, "test"),
            _ => panic!("Expected String variant"),
        }

        match color_attr {
            AttributeValue::RGBColor(color) => {
                assert_eq!(color.r, 255);
                assert_eq!(color.g, 128);
                assert_eq!(color.b, 0);
            }
            _ => panic!("Expected RGBColor variant"),
        }
    }

    #[test]
    fn test_attribute_value_from_conversions() {
        let bool_val: AttributeValue = true.into();
        let int_val: AttributeValue = 123.into();
        let float_val: AttributeValue = std::f64::consts::E.into();
        let string_val: AttributeValue = "hello".to_string().into();
        let color_val: AttributeValue = (255, 255, 0).into();

        assert!(matches!(bool_val, AttributeValue::Bool(true)));
        assert!(matches!(int_val, AttributeValue::Int(123)));
        assert!(
            matches!(float_val, AttributeValue::Float(f) if (f - std::f64::consts::E).abs() < 0.001)
        );
        assert!(matches!(string_val, AttributeValue::String(s) if s == "hello"));
        assert!(
            matches!(color_val, AttributeValue::RGBColor(c) if c.r == 255 && c.g == 255 && c.b == 0)
        );
    }

    #[test]
    fn test_actor_attribute_type_enum() {
        let types = [
            ActorAttributeType::Bool,
            ActorAttributeType::Int,
            ActorAttributeType::Float,
            ActorAttributeType::String,
            ActorAttributeType::RGBColor,
        ];

        assert_eq!(types.len(), 5);

        // Test Debug output
        assert_eq!(format!("{:?}", ActorAttributeType::Bool), "Bool");
        assert_eq!(format!("{:?}", ActorAttributeType::RGBColor), "RGBColor");
    }

    #[test]
    fn test_actor_attribute_type_from_u8() {
        assert_eq!(ActorAttributeType::from(0), ActorAttributeType::Bool);
        assert_eq!(ActorAttributeType::from(1), ActorAttributeType::Int);
        assert_eq!(ActorAttributeType::from(2), ActorAttributeType::Float);
        assert_eq!(ActorAttributeType::from(3), ActorAttributeType::String);
        assert_eq!(ActorAttributeType::from(4), ActorAttributeType::RGBColor);

        // Test unknown values default to String
        assert_eq!(ActorAttributeType::from(99), ActorAttributeType::String);
        assert_eq!(ActorAttributeType::from(255), ActorAttributeType::String);
    }

    #[test]
    fn test_actor_attribute_type_equality() {
        assert_eq!(ActorAttributeType::Bool, ActorAttributeType::Bool);
        assert_ne!(ActorAttributeType::Bool, ActorAttributeType::Int);
        assert_ne!(ActorAttributeType::Float, ActorAttributeType::String);
    }

    #[test]
    fn test_rgb_color_display() {
        let color = RGBColor::new(128, 64, 32);
        let debug_string = format!("{color:?}");

        assert!(debug_string.contains("128"));
        assert!(debug_string.contains("64"));
        assert!(debug_string.contains("32"));
    }

    #[test]
    fn test_attribute_value_debug() {
        let values = vec![
            AttributeValue::Bool(false),
            AttributeValue::Int(-42),
            AttributeValue::Float(0.0),
            AttributeValue::String("debug_test".to_string()),
            AttributeValue::RGBColor(RGBColor::new(1, 2, 3)),
        ];

        for value in values {
            let debug_string = format!("{value:?}");
            assert!(!debug_string.is_empty());
        }
    }

    #[test]
    fn test_rgb_color_edge_cases() {
        // Test boundary values
        let min_color = RGBColor::new(0, 0, 0);
        let max_color = RGBColor::new(255, 255, 255);

        assert_eq!(min_color.r, 0);
        assert_eq!(max_color.r, 255);

        // Test that values are stored as provided
        let test_color = RGBColor::new(1, 254, 127);
        assert_eq!(test_color.r, 1);
        assert_eq!(test_color.g, 254);
        assert_eq!(test_color.b, 127);
    }

    // Note: Testing actual blueprint functionality would require FFI mocks
    // These tests focus on the data structures and type conversions
}
