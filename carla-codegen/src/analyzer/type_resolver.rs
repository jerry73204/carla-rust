//! Type resolution and mapping from Python to Rust types

use crate::error::{CodegenError, Result, SourceContext};
use std::{collections::HashMap, path::PathBuf};
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
    /// Union of multiple types
    Union(Vec<RustType>),
    /// Tuple type
    Tuple(Vec<RustType>),
}

impl RustType {
    /// Convert to Rust type string
    pub fn to_rust_string(&self) -> String {
        match self {
            RustType::Primitive(name) => name.clone(),
            RustType::Option(inner) => {
                let inner_str = inner.to_rust_string();
                format!("Option<{inner_str}>")
            }
            RustType::Vec(inner) => {
                let inner_str = inner.to_rust_string();
                format!("Vec<{inner_str}>")
            }
            RustType::HashMap(key, value) => {
                let key_str = key.to_rust_string();
                let value_str = value.to_rust_string();
                format!("HashMap<{key_str}, {value_str}>")
            }
            RustType::Custom(name) => name.clone(),
            RustType::Reference(name) => format!("&{name}"),
            RustType::MutReference(name) => format!("&mut {name}"),
            RustType::Str => "&str".to_string(),
            RustType::Union(types) => {
                // For display purposes, show as "Type1 | Type2 | Type3"
                types
                    .iter()
                    .map(|t| t.to_rust_string())
                    .collect::<Vec<_>>()
                    .join(" | ")
            }
            RustType::Tuple(types) => {
                // Generate tuple type like (Type1, Type2, Type3)
                let type_strs: Vec<String> = types.iter().map(|t| t.to_rust_string()).collect();
                format!("({})", type_strs.join(", "))
            }
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
            .insert("uint".to_string(), RustType::Primitive("u32".to_string()));
        self.type_mappings
            .insert("float".to_string(), RustType::Primitive("f32".to_string()));
        self.type_mappings
            .insert("double".to_string(), RustType::Primitive("f64".to_string()));
        self.type_mappings
            .insert("bool".to_string(), RustType::Primitive("bool".to_string()));
        self.type_mappings.insert(
            "boolean".to_string(),
            RustType::Primitive("bool".to_string()),
        );
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

        // Callback and function types
        self.type_mappings.insert(
            "callback".to_string(),
            RustType::Primitive("u64".to_string()), // Callback ID
        );
        self.type_mappings.insert(
            "function".to_string(),
            RustType::Custom("Box<dyn Fn() + Send + Sync>".to_string()),
        );
        self.type_mappings.insert(
            "callable".to_string(),
            RustType::Custom("Box<dyn Fn() + Send + Sync>".to_string()),
        );

        // Texture types
        self.type_mappings.insert(
            "TextureColor".to_string(),
            RustType::Custom("crate::sensor::TextureColor".to_string()),
        );
        self.type_mappings.insert(
            "TextureFloatColor".to_string(),
            RustType::Custom("crate::sensor::TextureFloatColor".to_string()),
        );

        // Other special types
        self.type_mappings.insert(
            "tuple".to_string(),
            RustType::Custom("(String, String)".to_string()), // Generic tuple fallback
        );
        self.type_mappings.insert(
            "set(int)".to_string(),
            RustType::Vec(Box::new(RustType::Primitive("i32".to_string()))), // Set as Vec for simplicity
        );

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
        // Create a minimal context for backward compatibility
        let context = SourceContext::new_class(PathBuf::new(), String::new())
            .with_python_type(python_type.to_string());
        self.resolve_type_with_context(python_type, context)
    }

    /// Resolve a Python type to a Rust type with source context
    pub fn resolve_type_with_context(
        &self,
        python_type: &str,
        context: SourceContext,
    ) -> Result<RustType> {
        debug!("Resolving type: {}", python_type);

        // Add a check for the problematic type
        if python_type == "string" {
            eprintln!("WARNING: Found 'string' type - this should be 'str'");
        }

        // Check custom mappings first
        if let Some(rust_type) = self.custom_mappings.get(python_type) {
            return Ok(RustType::Custom(rust_type.clone()));
        }

        // Handle None/null
        if python_type == "None" || python_type.is_empty() {
            return Ok(RustType::Primitive("()".to_string()));
        }

        // Handle tuple types (e.g., "[name, world, actor, relative]")
        if python_type.starts_with('[') && python_type.ends_with(']') {
            return self.parse_tuple_type(python_type);
        }

        // Handle list types
        if python_type.starts_with("list(") && python_type.ends_with(")") {
            let inner = &python_type[5..python_type.len() - 1];
            // Check if inner is a tuple type (starts with '[' and ends with ']')
            if inner.starts_with('[') && inner.ends_with(']') {
                let tuple_type = self.parse_tuple_type(inner)?;
                return Ok(RustType::Vec(Box::new(tuple_type)));
            }
            let inner_type = self.resolve_type(inner)?;
            return Ok(RustType::Vec(Box::new(inner_type)));
        }

        // Handle list with brackets
        if python_type.starts_with("list[") && python_type.ends_with("]") {
            let inner = &python_type[5..python_type.len() - 1];
            let inner_type = self.resolve_type(inner)?;
            return Ok(RustType::Vec(Box::new(inner_type)));
        }

        // Handle array types (e.g., "array(carla.BoundingBox)")
        if python_type.starts_with("array(") && python_type.ends_with(")") {
            let inner = &python_type[6..python_type.len() - 1];
            let inner_type = self.resolve_type(inner)?;
            return Ok(RustType::Vec(Box::new(inner_type)));
        }

        // Handle dict types
        if python_type == "dict" || python_type.starts_with("dict(") {
            // Simple dict becomes HashMap<String, String>
            return Ok(RustType::HashMap(
                Box::new(RustType::Primitive("String".to_string())),
                Box::new(RustType::Primitive("String".to_string())),
            ));
        }

        // Handle special compound types
        if python_type == "any carla Command" {
            return Ok(RustType::Custom(
                "Box<dyn crate::command::Command>".to_string(),
            ));
        }

        // Handle "carla.Actor or int" type as special case for backwards compatibility
        if python_type == "carla.Actor or int" {
            return Ok(RustType::Custom("crate::actor::ActorOrId".to_string()));
        }

        // Handle union types with " or " separator
        if python_type.contains(" or ") {
            let parts: Vec<&str> = python_type.split(" or ").map(|s| s.trim()).collect();
            if parts.len() > 1 {
                let mut types = Vec::new();
                for part in parts {
                    types.push(self.resolve_type(part)?);
                }
                return Ok(RustType::Union(types));
            }
        }

        // Handle union types with " / " separator
        if python_type.contains(" / ") {
            let parts: Vec<&str> = python_type.split(" / ").map(|s| s.trim()).collect();
            if parts.len() > 1 {
                let mut types = Vec::new();
                for part in parts {
                    types.push(self.resolve_type(part)?);
                }
                return Ok(RustType::Union(types));
            }
        }

        // Handle "single char" type
        if python_type == "single char" {
            return Ok(RustType::Primitive("char".to_string()));
        }

        // Handle "string" return type
        if python_type == "string" {
            return Ok(RustType::Primitive("String".to_string()));
        }

        // Handle "bytes" type
        if python_type == "bytes" {
            return Ok(RustType::Vec(Box::new(RustType::Primitive(
                "u8".to_string(),
            ))));
        }

        // Handle tuple types with parentheses (e.g., "tuple(carla.Waypoint)")
        if python_type.starts_with("tuple(") && python_type.ends_with(")") {
            let inner = &python_type[6..python_type.len() - 1];

            // Check if it's a multi-element tuple
            if inner.contains(',') {
                // Split by comma and resolve each type
                let elements: Vec<&str> = inner.split(',').map(|s| s.trim()).collect();
                let mut tuple_types = Vec::new();
                for element in elements {
                    tuple_types.push(self.resolve_type(element)?);
                }
                return Ok(RustType::Tuple(tuple_types));
            } else {
                // Single element tuple - just use the type directly
                let inner_type = self.resolve_type(inner)?;
                return Ok(inner_type);
            }
        }

        // Handle CARLA types
        if let Some(rust_type) = self.carla_types.get(python_type) {
            return Ok(RustType::Custom(rust_type.clone()));
        }

        // Handle any type starting with "carla."
        if python_type.starts_with("carla.") {
            let type_name = python_type.strip_prefix("carla.").unwrap_or(python_type);
            return Ok(RustType::Custom(format!("crate::{type_name}")));
        }

        // Handle module.Type pattern (e.g., "command.Response")
        if python_type.contains('.') && !python_type.starts_with("carla.") {
            // Split into module and type
            let parts: Vec<&str> = python_type.split('.').collect();
            if parts.len() == 2 {
                let module = parts[0];
                let type_name = parts[1];
                return Ok(RustType::Custom(format!("crate::{module}::{type_name}")));
            }
        }

        // Handle primitive types
        if let Some(rust_type) = self.type_mappings.get(python_type) {
            return Ok(rust_type.clone());
        }

        // Handle bare 'list' type (without inner type)
        if python_type == "list" {
            // Default to list of dynamic type
            return Ok(RustType::Vec(Box::new(RustType::Custom(
                "Box<dyn std::any::Any>".to_string(),
            ))));
        }

        // Handle tuple of lists (e.g., "list(list(float))")
        if python_type == "list(list(float))" {
            return Ok(RustType::Vec(Box::new(RustType::Vec(Box::new(
                RustType::Primitive("f32".to_string()),
            )))));
        }

        // If nothing else matches, check if it might be a CARLA type without the prefix
        // (e.g., "WeatherParameters" instead of "carla.WeatherParameters")
        let type_name_pascal = python_type.to_string();
        if type_name_pascal
            .chars()
            .next()
            .map(|c| c.is_uppercase())
            .unwrap_or(false)
        {
            // Likely a CARLA type without prefix
            return Ok(RustType::Custom(format!("crate::{type_name_pascal}")));
        }

        // Default: return unknown type error with context
        Err(CodegenError::unknown_type(python_type.to_string(), context))
    }

    /// Add a custom type mapping
    pub fn add_type_mapping(&mut self, python_type: String, rust_type: RustType) {
        self.type_mappings.insert(python_type, rust_type);
    }

    /// Add a CARLA type mapping
    pub fn add_carla_type(&mut self, python_type: String, rust_path: String) {
        self.carla_types.insert(python_type, rust_path);
    }

    /// Parse a tuple type like "[name, world, actor, relative]"
    fn parse_tuple_type(&self, tuple_str: &str) -> Result<RustType> {
        // Remove the brackets
        let inner = tuple_str.trim_start_matches('[').trim_end_matches(']');

        // Split by comma and parse each element
        let elements: Vec<&str> = inner.split(',').map(|s| s.trim()).collect();

        if elements.is_empty() {
            return Ok(RustType::Tuple(vec![]));
        }

        // For the specific case of bone transforms, we know the structure
        if elements == vec!["name", "world", "actor", "relative"] {
            // This represents a bone transform tuple
            return Ok(RustType::Tuple(vec![
                RustType::Primitive("String".to_string()), // name
                RustType::Custom("crate::geom::Transform".to_string()), // world transform
                RustType::Custom("crate::geom::Transform".to_string()), // actor transform
                RustType::Custom("crate::geom::Transform".to_string()), // relative transform
            ]));
        }

        // For other tuples, try to resolve each element type
        let mut tuple_types = Vec::new();
        for element in elements {
            // Try to resolve as a type, otherwise treat as a field name
            match self.resolve_type(element) {
                Ok(rust_type) => tuple_types.push(rust_type),
                Err(_) => {
                    // If it's not a known type, it's likely a field name
                    // We'll need context to determine the actual type
                    // For now, use a generic approach
                    match element {
                        "name" | "id" => {
                            tuple_types.push(RustType::Primitive("String".to_string()))
                        }
                        "x" | "y" | "z" | "pitch" | "yaw" | "roll" => {
                            tuple_types.push(RustType::Primitive("f32".to_string()))
                        }
                        _ => tuple_types.push(RustType::Custom("serde_json::Value".to_string())),
                    }
                }
            }
        }

        Ok(RustType::Tuple(tuple_types))
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

    #[test]
    fn test_resolve_module_types() {
        let resolver = TypeResolver::new();

        // Test module.Type pattern
        assert_eq!(
            resolver.resolve_type("command.Response").unwrap(),
            RustType::Custom("crate::command::Response".to_string())
        );

        // Test list of module.Type
        let list_type = resolver.resolve_type("list(command.Response)").unwrap();
        assert_eq!(list_type.to_rust_string(), "Vec<crate::command::Response>");
    }

    #[test]
    fn test_resolve_tuple_types() {
        let resolver = TypeResolver::new();

        // Test simple tuple
        let tuple_type = resolver
            .resolve_type("[name, world, actor, relative]")
            .unwrap();
        assert_eq!(
            tuple_type.to_rust_string(),
            "(String, crate::geom::Transform, crate::geom::Transform, crate::geom::Transform)"
        );

        // Test list of tuples
        let list_tuple = resolver
            .resolve_type("list([name, world, actor, relative])")
            .unwrap();
        assert_eq!(
            list_tuple.to_rust_string(),
            "Vec<(String, crate::geom::Transform, crate::geom::Transform, crate::geom::Transform)>"
        );
    }

    #[test]
    fn test_carla_specific_types() {
        let resolver = TypeResolver::new();

        // Test the specific bone transforms type from CARLA
        let bone_type = resolver
            .resolve_type("list([name,world, actor, relative])")
            .unwrap();
        assert_eq!(
            bone_type.to_rust_string(),
            "Vec<(String, crate::geom::Transform, crate::geom::Transform, crate::geom::Transform)>"
        );

        // Test other CARLA types we added
        assert_eq!(
            resolver.resolve_type("boolean").unwrap(),
            RustType::Primitive("bool".to_string())
        );
        assert_eq!(
            resolver.resolve_type("uint").unwrap(),
            RustType::Primitive("u32".to_string())
        );
        assert_eq!(
            resolver.resolve_type("function").unwrap(),
            RustType::Custom("Box<dyn Fn() + Send + Sync>".to_string())
        );
    }

    #[test]
    fn test_nested_complex_types() {
        let resolver = TypeResolver::new();

        // Test nested lists
        let nested_list = resolver.resolve_type("list(list(float))").unwrap();
        assert_eq!(nested_list.to_rust_string(), "Vec<Vec<f32>>");

        // Test list with union types
        let union_list = resolver.resolve_type("list(carla.Actor or int)").unwrap();
        assert_eq!(union_list.to_rust_string(), "Vec<crate::actor::ActorOrId>");
    }
}
