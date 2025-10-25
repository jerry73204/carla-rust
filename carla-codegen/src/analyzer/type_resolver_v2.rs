//! Version 2 of the type resolver with improved architecture

use super::{RustType, TypeContext, TypeInferrer, TypeRegistry, TypeValidator};
use crate::{error::Result, parser::yaml_schema::Parameter};
use std::collections::HashMap;
use tracing::{debug, warn};

/// Result of type resolution
#[derive(Debug, Clone)]
pub struct ResolvedType {
    /// The resolved Rust type
    pub rust_type: RustType,
    /// Imports needed for this type
    pub imports_needed: Vec<String>,
    /// Whether this type needs boxing (for recursion)
    pub needs_boxing: bool,
    /// Type alias to use if any
    pub type_alias: Option<String>,
}

/// Unified type resolver with context support
pub struct TypeResolverV2 {
    /// Type registry
    registry: TypeRegistry,
    /// Type inferrer
    inferrer: TypeInferrer,
    /// Type validator
    validator: TypeValidator,
    /// Custom type mappings from config
    custom_mappings: HashMap<String, String>,
}

impl TypeResolverV2 {
    /// Create a new type resolver
    pub fn new() -> Self {
        let registry = TypeRegistry::new();
        let inferrer = TypeInferrer::new(TypeRegistry::new());
        let validator = TypeValidator::new(TypeRegistry::new());

        Self {
            registry,
            inferrer,
            validator,
            custom_mappings: HashMap::new(),
        }
    }

    /// Create with custom mappings
    pub fn with_custom_mappings(mut self, mappings: HashMap<String, String>) -> Self {
        self.custom_mappings = mappings;
        self
    }

    /// Main entry point for type resolution
    pub fn resolve(&self, python_type: &str, context: TypeContext) -> Result<ResolvedType> {
        debug!(
            "Resolving type '{}' in context {:?}",
            python_type, context.location
        );

        // Phase 1: Parse string to AST
        let rust_type = self.parse_type(python_type, &context)?;

        // Phase 2: Validate type
        let mut validated_type = self.validate_type(rust_type, &context)?;

        // Run type validator and auto-fix issues
        if self.validator.fix_type(&mut validated_type) {
            debug!("Fixed type issues in {:?}", validated_type);
        }

        // Phase 3: Enhance type with context
        let enhanced_type = self.enhance_type(validated_type, &context)?;

        // Phase 4: Collect metadata
        let resolved = self.finalize_type(enhanced_type, &context)?;

        // Phase 5: Final validation
        let issues = self
            .validator
            .validate_resolved_type(&resolved, &format!("{:?}", context.location));
        if !issues.is_empty() {
            for issue in &issues {
                warn!("Type validation issue: {:?}", issue);
            }
        }

        Ok(resolved)
    }

    /// Resolve a parameter type with inference
    pub fn resolve_parameter(
        &self,
        param: &Parameter,
        context: TypeContext,
    ) -> Result<ResolvedType> {
        // If type is specified, use it
        if let Some(ref param_type) = param.param_type {
            return self.resolve(param_type, context);
        }

        // Otherwise, try to infer
        if let Some(inferred_type) = self.inferrer.infer_parameter_type(param, &context) {
            debug!(
                "Inferred type for parameter '{}': {:?}",
                param.param_name, inferred_type
            );
            return self.finalize_type(inferred_type, &context);
        }

        // Default fallback
        warn!(
            "No type specified or inferred for parameter '{}'",
            param.param_name
        );
        self.finalize_type(RustType::Custom("String".to_string()), &context)
    }

    /// Phase 1: Parse type string to RustType AST
    fn parse_type(&self, python_type: &str, context: &TypeContext) -> Result<RustType> {
        // Check custom mappings first
        if let Some(mapped) = self.custom_mappings.get(python_type) {
            return Ok(RustType::Custom(mapped.clone()));
        }

        // Handle None type
        if python_type == "None" || python_type.is_empty() {
            return Ok(RustType::Primitive("()".to_string()));
        }

        // Handle list types
        if python_type.starts_with("list(") && python_type.ends_with(')') {
            let inner = &python_type[5..python_type.len() - 1];

            // Special case: list([name, world, actor, relative]) -> list of tuples
            if inner.starts_with('[') && inner.ends_with(']') {
                // This is a list of tuples with specific field names
                // For now, we'll treat it as Vec<(String, crate::carla::Transform, crate::carla::Actor, crate::carla::Transform)>
                // TODO: Parse the actual field names and infer types
                return Ok(RustType::Vec(Box::new(RustType::Tuple(vec![
                    RustType::Custom("String".to_string()),
                    RustType::Custom("crate::carla::Transform".to_string()),
                    RustType::Custom("crate::carla::Actor".to_string()),
                    RustType::Custom("crate::carla::Transform".to_string()),
                ]))));
            }

            let inner_type = self.parse_type(inner, &context.clone().inside_generic())?;
            return Ok(RustType::Vec(Box::new(inner_type)));
        }

        // Handle dict types
        if python_type.starts_with("dict[") && python_type.ends_with(']') {
            let inner = &python_type[5..python_type.len() - 1];
            let parts: Vec<&str> = inner.split(',').map(|s| s.trim()).collect();
            if parts.len() == 2 {
                let key_type = self.parse_type(parts[0], &context.clone().inside_generic())?;
                let value_type = self.parse_type(parts[1], &context.clone().inside_generic())?;
                return Ok(RustType::HashMap(Box::new(key_type), Box::new(value_type)));
            }
        }

        // Handle union types
        if python_type.contains(" / ") || python_type.contains(" or ") {
            let parts: Vec<&str> = if python_type.contains(" / ") {
                python_type.split(" / ").map(|s| s.trim()).collect()
            } else {
                python_type.split(" or ").map(|s| s.trim()).collect()
            };
            let mut types = Vec::new();
            for part in parts {
                types.push(self.parse_type(part, context)?);
            }
            return Ok(RustType::Union(types));
        }

        // Handle tuple types
        if python_type.starts_with("tuple(") && python_type.ends_with(')') {
            let inner = &python_type[6..python_type.len() - 1];
            if inner.contains(',') {
                let parts: Vec<&str> = inner.split(',').map(|s| s.trim()).collect();
                let mut types = Vec::new();
                for part in parts {
                    types.push(self.parse_type(part, context)?);
                }
                return Ok(RustType::Tuple(types));
            }
        }

        // Handle CARLA types
        if python_type.starts_with("carla.") {
            let type_name = python_type.strip_prefix("carla.").unwrap();
            return Ok(RustType::Custom(format!("crate::carla::{type_name}")));
        }

        // Handle primitive types
        let rust_type = match python_type {
            "bool" | "boolean" => RustType::Primitive("bool".to_string()),
            "int" => RustType::Primitive("i32".to_string()),
            "uint" => RustType::Primitive("u32".to_string()),
            "uint8" => RustType::Primitive("u8".to_string()),
            "uint16" => RustType::Primitive("u16".to_string()),
            "uint32" => RustType::Primitive("u32".to_string()),
            "uint64" => RustType::Primitive("u64".to_string()),
            "int8" => RustType::Primitive("i8".to_string()),
            "int16" => RustType::Primitive("i16".to_string()),
            "int32" => RustType::Primitive("i32".to_string()),
            "int64" => RustType::Primitive("i64".to_string()),
            "float" => RustType::Primitive("f32".to_string()),
            "double" => RustType::Primitive("f64".to_string()),
            "str" | "string" => RustType::Custom("String".to_string()),
            "bytes" => RustType::Vec(Box::new(RustType::Primitive("u8".to_string()))),
            "list" => RustType::Vec(Box::new(RustType::Custom(
                "crate::carla::Actor".to_string(),
            ))),
            "dict" => RustType::HashMap(
                Box::new(RustType::Custom("String".to_string())),
                Box::new(RustType::Custom("String".to_string())),
            ),
            _ => {
                // Check if it's a known type in registry
                if let Some(info) = self.registry.lookup(python_type) {
                    RustType::Custom(info.rust_path.clone())
                } else {
                    // Unknown type - might be a CARLA type without prefix
                    if python_type
                        .chars()
                        .next()
                        .map(|c| c.is_uppercase())
                        .unwrap_or(false)
                    {
                        RustType::Custom(format!("crate::carla::{python_type}"))
                    } else {
                        RustType::Custom(python_type.to_string())
                    }
                }
            }
        };

        Ok(rust_type)
    }

    /// Phase 2: Validate the parsed type
    fn validate_type(&self, rust_type: RustType, context: &TypeContext) -> Result<RustType> {
        match &rust_type {
            RustType::Vec(inner) => {
                let validated_inner =
                    self.validate_type((**inner).clone(), &context.clone().inside_generic())?;
                Ok(RustType::Vec(Box::new(validated_inner)))
            }
            RustType::Option(inner) => {
                let validated_inner =
                    self.validate_type((**inner).clone(), &context.clone().inside_generic())?;
                Ok(RustType::Option(Box::new(validated_inner)))
            }
            RustType::HashMap(key, value) => {
                let validated_key =
                    self.validate_type((**key).clone(), &context.clone().inside_generic())?;
                let validated_value =
                    self.validate_type((**value).clone(), &context.clone().inside_generic())?;
                Ok(RustType::HashMap(
                    Box::new(validated_key),
                    Box::new(validated_value),
                ))
            }
            RustType::Custom(type_name) => {
                // Validate custom types exist or can be resolved
                if type_name.starts_with("crate::") || self.registry.lookup(type_name).is_some() {
                    Ok(rust_type)
                } else if self.registry.is_standard_type(type_name) {
                    Ok(rust_type)
                } else {
                    warn!("Unknown type '{}' in {:?}", type_name, context.location);
                    Ok(rust_type)
                }
            }
            _ => Ok(rust_type),
        }
    }

    /// Phase 3: Enhance type with module paths and context
    fn enhance_type(&self, rust_type: RustType, context: &TypeContext) -> Result<RustType> {
        match rust_type {
            RustType::Custom(type_name) => {
                // Don't modify if already has module path
                if type_name.contains("::") {
                    return Ok(RustType::Custom(type_name));
                }

                // Don't modify standard types
                if self.registry.is_standard_type(&type_name) {
                    return Ok(RustType::Custom(type_name));
                }

                // Check for CARLA types that need module path
                if let Some(info) = self.registry.lookup(&type_name) {
                    return Ok(RustType::Custom(info.rust_path.clone()));
                }

                // If it looks like a CARLA type, add module path
                if type_name
                    .chars()
                    .next()
                    .map(|c| c.is_uppercase())
                    .unwrap_or(false)
                {
                    return Ok(RustType::Custom(format!("crate::carla::{type_name}")));
                }

                Ok(RustType::Custom(type_name))
            }
            RustType::Vec(inner) => {
                let enhanced_inner =
                    self.enhance_type(*inner, &context.clone().inside_generic())?;
                Ok(RustType::Vec(Box::new(enhanced_inner)))
            }
            RustType::Option(inner) => {
                let enhanced_inner =
                    self.enhance_type(*inner, &context.clone().inside_generic())?;
                Ok(RustType::Option(Box::new(enhanced_inner)))
            }
            RustType::HashMap(key, value) => {
                let enhanced_key = self.enhance_type(*key, &context.clone().inside_generic())?;
                let enhanced_value =
                    self.enhance_type(*value, &context.clone().inside_generic())?;
                Ok(RustType::HashMap(
                    Box::new(enhanced_key),
                    Box::new(enhanced_value),
                ))
            }
            _ => Ok(rust_type),
        }
    }

    /// Phase 4: Finalize with metadata
    fn finalize_type(&self, rust_type: RustType, context: &TypeContext) -> Result<ResolvedType> {
        let mut imports = Vec::new();
        let mut needs_boxing = false;
        let mut type_alias = None;

        // Check for recursive types
        if let RustType::Custom(ref type_name) = rust_type {
            if type_name.ends_with(&context.containing_class) {
                needs_boxing = true;
            }
        }

        // Collect imports
        self.collect_imports_for_type(&rust_type, &mut imports);

        // Check for type aliases
        if let RustType::Custom(ref type_name) = rust_type {
            let alias = self.registry.resolve_alias(type_name);
            if &alias != type_name {
                type_alias = Some(alias);
            }
        }

        Ok(ResolvedType {
            rust_type,
            imports_needed: imports,
            needs_boxing,
            type_alias,
        })
    }

    /// Collect imports needed for a type
    fn collect_imports_for_type(&self, rust_type: &RustType, imports: &mut Vec<String>) {
        match rust_type {
            RustType::Custom(type_name) => {
                if type_name == "HashMap" {
                    imports.push("use std::collections::HashMap;".to_string());
                } else if type_name == "HashSet" {
                    imports.push("use std::collections::HashSet;".to_string());
                }
            }
            RustType::HashMap(key, value) => {
                imports.push("use std::collections::HashMap;".to_string());
                self.collect_imports_for_type(key, imports);
                self.collect_imports_for_type(value, imports);
            }
            RustType::Vec(inner) | RustType::Option(inner) => {
                self.collect_imports_for_type(inner, imports);
            }
            RustType::Union(types) | RustType::Tuple(types) => {
                for t in types {
                    self.collect_imports_for_type(t, imports);
                }
            }
            _ => {}
        }
    }
}

impl Default for TypeResolverV2 {
    fn default() -> Self {
        Self::new()
    }
}
