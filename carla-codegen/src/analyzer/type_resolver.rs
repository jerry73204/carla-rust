//! Type resolution and mapping from Python to Rust types

use crate::error::{CodegenError, Result};
use std::collections::HashMap;
use tracing::debug;

/// Type information after resolution
#[derive(Debug, Clone, PartialEq)]
pub enum RustType {
    /// Primitive types
    Primitive(String),
    /// Option type
    Option(Box<RustType>),
    /// Vector type
    Vec(Box<RustType>),
    /// HashMap type
    HashMap(Box<RustType>, Box<RustType>),
    /// Custom CARLA type
    Custom(String),
    /// Reference to another type
    Reference(String),
    /// Mutable reference
    MutReference(String),
    /// String slice
    Str,
}

impl RustType {
    /// Convert to Rust type string
    pub fn to_rust_string(&self) -> String {
        match self {
            RustType::Primitive(name) => name.clone(),
            RustType::Option(inner) => format!("Option<{}>", inner.to_rust_string()),
            RustType::Vec(inner) => format!("Vec<{}>", inner.to_rust_string()),
            RustType::HashMap(key, value) => {
                format!(
                    "HashMap<{}, {}>",
                    key.to_rust_string(),
                    value.to_rust_string()
                )
            }
            RustType::Custom(name) => name.clone(),
            RustType::Reference(name) => format!("&{}", name),
            RustType::MutReference(name) => format!("&mut {}", name),
            RustType::Str => "&str".to_string(),
        }
    }
}

/// Type resolver for Python to Rust type conversion
pub struct TypeResolver {
    /// Type mappings from Python to Rust
    type_mappings: HashMap<String, RustType>,
    /// Known CARLA types
    carla_types: HashMap<String, String>,
    /// Custom type mappings from config
    custom_mappings: HashMap<String, String>,
}

impl TypeResolver {
    /// Create a new type resolver with default mappings
    pub fn new() -> Self {
        let mut resolver = Self {
            type_mappings: HashMap::new(),
            carla_types: HashMap::new(),
            custom_mappings: HashMap::new(),
        };

        resolver.init_default_mappings();
        resolver
    }

    /// Create a type resolver with custom mappings
    pub fn with_custom_mappings(custom_mappings: HashMap<String, String>) -> Self {
        let mut resolver = Self::new();
        resolver.custom_mappings = custom_mappings;
        resolver
    }

    /// Initialize default type mappings
    fn init_default_mappings(&mut self) {
        // Primitive types
        self.type_mappings
            .insert("int".to_string(), RustType::Primitive("i32".to_string()));
        self.type_mappings
            .insert("float".to_string(), RustType::Primitive("f32".to_string()));
        self.type_mappings
            .insert("double".to_string(), RustType::Primitive("f64".to_string()));
        self.type_mappings
            .insert("bool".to_string(), RustType::Primitive("bool".to_string()));
        self.type_mappings
            .insert("str".to_string(), RustType::Primitive("String".to_string()));
        self.type_mappings.insert(
            "string".to_string(),
            RustType::Primitive("String".to_string()),
        );

        // Special types
        self.type_mappings
            .insert("uint16".to_string(), RustType::Primitive("u16".to_string()));
        self.type_mappings
            .insert("uint32".to_string(), RustType::Primitive("u32".to_string()));
        self.type_mappings
            .insert("uint64".to_string(), RustType::Primitive("u64".to_string()));
        self.type_mappings
            .insert("uchar".to_string(), RustType::Primitive("u8".to_string()));

        // CARLA types
        self.carla_types.insert(
            "carla.Vector2D".to_string(),
            "crate::geom::Vector2D".to_string(),
        );
        self.carla_types.insert(
            "carla.Vector3D".to_string(),
            "crate::geom::Vector3D".to_string(),
        );
        self.carla_types.insert(
            "carla.Location".to_string(),
            "crate::geom::Location".to_string(),
        );
        self.carla_types.insert(
            "carla.Rotation".to_string(),
            "crate::geom::Rotation".to_string(),
        );
        self.carla_types.insert(
            "carla.Transform".to_string(),
            "crate::geom::Transform".to_string(),
        );
        self.carla_types.insert(
            "carla.BoundingBox".to_string(),
            "crate::geom::BoundingBox".to_string(),
        );
        self.carla_types.insert(
            "carla.GeoLocation".to_string(),
            "crate::geom::GeoLocation".to_string(),
        );

        self.carla_types
            .insert("carla.Actor".to_string(), "crate::actor::Actor".to_string());
        self.carla_types.insert(
            "carla.Vehicle".to_string(),
            "crate::actor::Vehicle".to_string(),
        );
        self.carla_types.insert(
            "carla.Walker".to_string(),
            "crate::actor::Walker".to_string(),
        );
        self.carla_types.insert(
            "carla.TrafficLight".to_string(),
            "crate::actor::TrafficLight".to_string(),
        );
        self.carla_types.insert(
            "carla.TrafficSign".to_string(),
            "crate::actor::TrafficSign".to_string(),
        );

        self.carla_types.insert(
            "carla.World".to_string(),
            "crate::client::World".to_string(),
        );
        self.carla_types.insert(
            "carla.Client".to_string(),
            "crate::client::Client".to_string(),
        );
        self.carla_types.insert(
            "carla.TrafficManager".to_string(),
            "crate::traffic_manager::TrafficManager".to_string(),
        );
    }

    /// Resolve a Python type to a Rust type
    pub fn resolve_type(&self, python_type: &str) -> Result<RustType> {
        debug!("Resolving type: {}", python_type);

        // Check custom mappings first
        if let Some(rust_type) = self.custom_mappings.get(python_type) {
            return Ok(RustType::Custom(rust_type.clone()));
        }

        // Handle None/null
        if python_type == "None" || python_type.is_empty() {
            return Ok(RustType::Primitive("()".to_string()));
        }

        // Handle list types
        if python_type.starts_with("list(") && python_type.ends_with(")") {
            let inner = &python_type[5..python_type.len() - 1];
            let inner_type = self.resolve_type(inner)?;
            return Ok(RustType::Vec(Box::new(inner_type)));
        }

        // Handle list with brackets
        if python_type.starts_with("list[") && python_type.ends_with("]") {
            let inner = &python_type[5..python_type.len() - 1];
            let inner_type = self.resolve_type(inner)?;
            return Ok(RustType::Vec(Box::new(inner_type)));
        }

        // Handle dict types
        if python_type.starts_with("dict") {
            // Simple dict becomes HashMap<String, String>
            return Ok(RustType::HashMap(
                Box::new(RustType::Primitive("String".to_string())),
                Box::new(RustType::Primitive("String".to_string())),
            ));
        }

        // Handle CARLA types
        if let Some(rust_type) = self.carla_types.get(python_type) {
            return Ok(RustType::Custom(rust_type.clone()));
        }

        // Handle primitive types
        if let Some(rust_type) = self.type_mappings.get(python_type) {
            return Ok(rust_type.clone());
        }

        // Handle tuple of lists (e.g., "list(list(float))")
        if python_type == "list(list(float))" {
            return Ok(RustType::Vec(Box::new(RustType::Vec(Box::new(
                RustType::Primitive("f32".to_string()),
            )))));
        }

        // Default: treat as custom type
        Err(CodegenError::UnknownType(python_type.to_string()))
    }

    /// Add a custom type mapping
    pub fn add_type_mapping(&mut self, python_type: String, rust_type: RustType) {
        self.type_mappings.insert(python_type, rust_type);
    }

    /// Add a CARLA type mapping
    pub fn add_carla_type(&mut self, python_type: String, rust_path: String) {
        self.carla_types.insert(python_type, rust_path);
    }
}

impl Default for TypeResolver {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_resolve_primitive_types() {
        let resolver = TypeResolver::new();

        assert_eq!(
            resolver.resolve_type("int").unwrap(),
            RustType::Primitive("i32".to_string())
        );
        assert_eq!(
            resolver.resolve_type("float").unwrap(),
            RustType::Primitive("f32".to_string())
        );
        assert_eq!(
            resolver.resolve_type("bool").unwrap(),
            RustType::Primitive("bool".to_string())
        );
        assert_eq!(
            resolver.resolve_type("str").unwrap(),
            RustType::Primitive("String".to_string())
        );
    }

    #[test]
    fn test_resolve_list_types() {
        let resolver = TypeResolver::new();

        let list_int = resolver.resolve_type("list(int)").unwrap();
        assert_eq!(
            list_int,
            RustType::Vec(Box::new(RustType::Primitive("i32".to_string())))
        );
        assert_eq!(list_int.to_rust_string(), "Vec<i32>");

        let list_float = resolver.resolve_type("list[float]").unwrap();
        assert_eq!(
            list_float,
            RustType::Vec(Box::new(RustType::Primitive("f32".to_string())))
        );
    }

    #[test]
    fn test_resolve_carla_types() {
        let resolver = TypeResolver::new();

        assert_eq!(
            resolver.resolve_type("carla.Vector3D").unwrap(),
            RustType::Custom("crate::geom::Vector3D".to_string())
        );
        assert_eq!(
            resolver.resolve_type("carla.Actor").unwrap(),
            RustType::Custom("crate::actor::Actor".to_string())
        );
    }
}
