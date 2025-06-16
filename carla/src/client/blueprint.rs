//! Actor blueprint for spawning.

use crate::error::CarlaResult;
use carla_cxx::ActorBlueprintWrapper;
use std::marker::PhantomData;

/// Actor blueprint for spawning.
#[derive(Debug, Clone)]
pub struct ActorBlueprint {
    /// Internal wrapper from carla-cxx
    inner: ActorBlueprintWrapper,
}

impl ActorBlueprint {
    /// Create a blueprint from a carla-cxx ActorBlueprintWrapper.
    pub fn new(inner: ActorBlueprintWrapper) -> Self {
        Self { inner }
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

    /// TODO: Implement this method. It returns an entry to an
    /// attribute if the corresponding key exists. The entry can be used to get or set the value.
    pub fn attribute_entry(&mut self, key: &str) -> Option<ActorAttribute<'_>> {
        todo!()
    }

    /// TODO: Implement this method. It returns the value of the attribute. The method is designed for convenience.
    pub fn get_attribute(&self, key: &str) -> Option<AttributeValue> {
        todo!()
    }

    /// TODO: Implement this method. It returns the value of the attribute. The method is designed for convenience.
    pub fn set_attribute<V>(&mut self, key: &str, value: V) -> CarlaResult<()>
    where
        V: Into<AttributeValue>,
    {
        todo!()
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
    pub fn matches_tags(&self, pattern: &str) -> bool {
        self.inner.match_tags(pattern)
    }

    pub fn attribute_iter() {
        todo!("return an iterator of attribute entries")
    }

    pub fn len(&self) -> usize {
        todo!("Return the number of attributes. Rename this method to an informative name")
    }
}

// TODO: Re-design this struct API.
/// Actor attribute for blueprint configuration.
#[derive(Debug, Clone)]
pub struct ActorAttribute<'a> {
    _phanton: PhantomData<&'a ()>, // Placeholder value. Please keep a reference to the parent ActorBlueprint
}

// TODO: Re-design the API.
impl<'a> ActorAttribute<'a> {
    pub fn id(&self) -> &str {
        todo!()
    }

    pub fn is_modifiable(&self) -> bool {
        todo!()
    }
    pub fn recommended_values(&self) -> Vec<String> {
        todo!()
    }

    pub fn ty(&self) -> ActorAttributeType {
        todo!()
    }

    pub fn get(&self) -> AttributeValue {
        todo!()
    }

    // TODO: The input argument performs automatic type conversion
    pub fn set<V>(&mut self, value: V) -> CarlaResult<()>
    where
        V: Into<AttributeValue>,
    {
        // Valid only when is_modifiable() is true.
        todo!()
    }
}

// TODO: implement the enum. Each variant corresponds to an attribute type and holds a value of that type.
pub enum AttributeValue {
    Bool(bool),
    Int(i64),
    Float(f64),
    String(String),
    RGBColor(()), // TODO: fill in a proper type and create a "impl From<XXX> for AttributeValue" for this variant.
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
