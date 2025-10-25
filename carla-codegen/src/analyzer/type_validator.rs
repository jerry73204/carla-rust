//! Type validation layer for pre-generation checks

use super::{ResolvedType, RustType, TypeRegistry};
use std::collections::{HashMap, HashSet};
use tracing::warn;

/// Type validation issues
#[derive(Debug, Clone)]
pub enum ValidationIssue {
    /// Bare generic type without parameters
    BareGeneric { type_name: String, location: String },
    /// Invalid module path
    InvalidModulePath { path: String, location: String },
    /// Unresolved type reference
    UnresolvedType { type_name: String, location: String },
    /// Invalid trait derivation
    InvalidTrait { trait_name: String, reason: String },
    /// Circular dependency detected
    CircularDependency { types: Vec<String> },
    /// Invalid type name (e.g., Python type leaked)
    InvalidTypeName { name: String, suggestion: String },
}

/// Type validator for pre-generation validation
pub struct TypeValidator {
    /// Type registry for lookups
    registry: TypeRegistry,
    /// Known module paths
    valid_modules: HashSet<String>,
    /// Python to Rust type corrections
    type_corrections: HashMap<String, String>,
}

impl TypeValidator {
    /// Create a new type validator
    pub fn new(registry: TypeRegistry) -> Self {
        let mut validator = Self {
            registry,
            valid_modules: HashSet::new(),
            type_corrections: HashMap::new(),
        };

        // Initialize valid module paths
        validator.valid_modules.insert("crate".to_string());
        validator.valid_modules.insert("crate::carla".to_string());
        validator.valid_modules.insert("crate::error".to_string());
        validator.valid_modules.insert("std".to_string());
        validator
            .valid_modules
            .insert("std::collections".to_string());

        // Initialize type corrections
        validator
            .type_corrections
            .insert("dict".to_string(), "HashMap".to_string());
        validator
            .type_corrections
            .insert("boolean".to_string(), "bool".to_string());
        validator
            .type_corrections
            .insert("uint".to_string(), "u32".to_string());
        validator
            .type_corrections
            .insert("uint8".to_string(), "u8".to_string());
        validator
            .type_corrections
            .insert("uint16".to_string(), "u16".to_string());
        validator
            .type_corrections
            .insert("uint32".to_string(), "u32".to_string());
        validator
            .type_corrections
            .insert("uint64".to_string(), "u64".to_string());
        validator
            .type_corrections
            .insert("int".to_string(), "i32".to_string());
        validator
            .type_corrections
            .insert("int8".to_string(), "i8".to_string());
        validator
            .type_corrections
            .insert("int16".to_string(), "i16".to_string());
        validator
            .type_corrections
            .insert("int32".to_string(), "i32".to_string());
        validator
            .type_corrections
            .insert("int64".to_string(), "i64".to_string());
        validator
            .type_corrections
            .insert("float".to_string(), "f32".to_string());
        validator
            .type_corrections
            .insert("double".to_string(), "f64".to_string());

        validator
    }

    /// Validate a resolved type
    pub fn validate_resolved_type(
        &self,
        resolved: &ResolvedType,
        location: &str,
    ) -> Vec<ValidationIssue> {
        let mut issues = Vec::new();

        // Validate the RustType
        self.validate_rust_type(&resolved.rust_type, location, &mut issues);

        // Check imports are valid
        for import in &resolved.imports_needed {
            if !self.is_valid_import(import) {
                warn!("Suspicious import statement: {}", import);
            }
        }

        issues
    }

    /// Validate a RustType
    fn validate_rust_type(
        &self,
        rust_type: &RustType,
        location: &str,
        issues: &mut Vec<ValidationIssue>,
    ) {
        match rust_type {
            RustType::Vec(inner) => {
                // Validate inner type
                self.validate_rust_type(inner, &format!("{location} -> Vec"), issues);
            }
            RustType::Option(inner) => {
                // Validate inner type
                self.validate_rust_type(inner, &format!("{location} -> Option"), issues);
            }
            RustType::HashMap(key, value) => {
                // Validate key and value types
                self.validate_rust_type(key, &format!("{location} -> HashMap key"), issues);
                self.validate_rust_type(value, &format!("{location} -> HashMap value"), issues);
            }
            RustType::Custom(type_name) => {
                // Check for Python type names that leaked through
                if let Some(correction) = self.type_corrections.get(type_name) {
                    issues.push(ValidationIssue::InvalidTypeName {
                        name: type_name.clone(),
                        suggestion: correction.clone(),
                    });
                    return;
                }

                // Check for bare generics
                if type_name == "Vec" || type_name == "HashMap" || type_name == "Option" {
                    issues.push(ValidationIssue::BareGeneric {
                        type_name: type_name.clone(),
                        location: location.to_string(),
                    });
                    return;
                }

                // Check module paths
                if type_name.contains("::") {
                    let parts: Vec<&str> = type_name.split("::").collect();
                    if parts.len() >= 2 {
                        let module = parts[..parts.len() - 1].join("::");
                        if !self.is_valid_module_path(&module) {
                            issues.push(ValidationIssue::InvalidModulePath {
                                path: module,
                                location: location.to_string(),
                            });
                        }
                    }
                } else if !self.is_known_simple_type(type_name) {
                    // Check if it's a known type without module path
                    if type_name
                        .chars()
                        .next()
                        .map(|c| c.is_uppercase())
                        .unwrap_or(false)
                        && !self.registry.is_standard_type(type_name)
                    {
                        // Likely a CARLA type missing module path
                        issues.push(ValidationIssue::UnresolvedType {
                            type_name: type_name.clone(),
                            location: location.to_string(),
                        });
                    }
                }
            }
            RustType::Union(types) => {
                // Union types should have been resolved to specific types
                warn!("Union type still present at validation: {:?}", types);
                for t in types {
                    self.validate_rust_type(t, &format!("{location} -> Union"), issues);
                }
            }
            RustType::Tuple(types) => {
                // Validate each type in tuple
                for (i, t) in types.iter().enumerate() {
                    self.validate_rust_type(t, &format!("{location} -> Tuple[{i}]"), issues);
                }
            }
            _ => {
                // Primitive types are always valid
            }
        }
    }

    /// Check if an import statement is valid
    fn is_valid_import(&self, import: &str) -> bool {
        import.starts_with("use ")
            && (import.contains("std::")
                || import.contains("crate::")
                || import.contains("super::"))
    }

    /// Check if a module path is valid
    fn is_valid_module_path(&self, path: &str) -> bool {
        self.valid_modules.contains(path)
            || path.starts_with("std::")
            || path.starts_with("crate::carla::")
            || path.starts_with("crate::error")
    }

    /// Check if a type is a known simple type
    fn is_known_simple_type(&self, type_name: &str) -> bool {
        matches!(
            type_name,
            "String"
                | "str"
                | "&str"
                | "bool"
                | "char"
                | "i8"
                | "i16"
                | "i32"
                | "i64"
                | "i128"
                | "isize"
                | "u8"
                | "u16"
                | "u32"
                | "u64"
                | "u128"
                | "usize"
                | "f32"
                | "f64"
        )
    }

    /// Validate trait derivations for a struct
    pub fn validate_derives(
        &self,
        derives: &[String],
        fields: &[(String, RustType)],
    ) -> Vec<ValidationIssue> {
        let mut issues = Vec::new();

        // Check Copy trait
        if derives.contains(&"Copy".to_string()) {
            for (field_name, field_type) in fields {
                if !self.is_copyable_type(field_type) {
                    issues.push(ValidationIssue::InvalidTrait {
                        trait_name: "Copy".to_string(),
                        reason: format!("field '{field_name}' is not Copy"),
                    });
                    break;
                }
            }
        }

        // Check Hash trait
        if derives.contains(&"Hash".to_string()) {
            for (field_name, field_type) in fields {
                if field_type.to_rust_string().contains("f32")
                    || field_type.to_rust_string().contains("f64")
                {
                    issues.push(ValidationIssue::InvalidTrait {
                        trait_name: "Hash".to_string(),
                        reason: format!("field '{field_name}' contains floating point type"),
                    });
                    break;
                }
            }
        }

        issues
    }

    /// Check if a type is copyable
    fn is_copyable_type(&self, rust_type: &RustType) -> bool {
        match rust_type {
            RustType::Primitive(_) => true,
            RustType::Custom(name) => {
                // String is a known type but not Copy
                if name == "String" {
                    return false;
                }
                self.registry.is_copyable(name)
                    || (self.is_known_simple_type(name) && name != "String")
            }
            RustType::Option(inner) => self.is_copyable_type(inner),
            RustType::Reference(_) => true,
            _ => false,
        }
    }

    /// Check for circular dependencies
    pub fn check_circular_dependencies(
        &self,
        types: &HashMap<String, Vec<String>>, // type_name -> dependencies
    ) -> Vec<ValidationIssue> {
        let mut issues = Vec::new();
        let mut visited = HashSet::new();
        let mut path = Vec::new();

        for type_name in types.keys() {
            if self.has_circular_dependency(type_name, types, &mut visited, &mut path) {
                issues.push(ValidationIssue::CircularDependency {
                    types: path.clone(),
                });
            }
            visited.clear();
            path.clear();
        }

        issues
    }

    /// Check if a type has circular dependencies
    fn has_circular_dependency(
        &self,
        type_name: &str,
        types: &HashMap<String, Vec<String>>,
        visited: &mut HashSet<String>,
        path: &mut Vec<String>,
    ) -> bool {
        if path.contains(&type_name.to_string()) {
            path.push(type_name.to_string());
            return true;
        }

        if visited.contains(type_name) {
            return false;
        }

        visited.insert(type_name.to_string());
        path.push(type_name.to_string());

        if let Some(deps) = types.get(type_name) {
            for dep in deps {
                if self.has_circular_dependency(dep, types, visited, path) {
                    return true;
                }
            }
        }

        path.pop();
        false
    }

    /// Fix validation issues where possible
    pub fn fix_type(&self, rust_type: &mut RustType) -> bool {
        match rust_type {
            RustType::Custom(name) => {
                // Fix Python type names
                if let Some(correction) = self.type_corrections.get(name) {
                    *name = correction.clone();
                    return true;
                }

                // Add module paths to unqualified CARLA types
                if name
                    .chars()
                    .next()
                    .map(|c| c.is_uppercase())
                    .unwrap_or(false)
                    && !name.contains("::")
                    && !self.is_known_simple_type(name)
                {
                    *name = format!("crate::carla::{name}");
                    return true;
                }

                false
            }
            RustType::Vec(inner) => self.fix_type(inner),
            RustType::Option(inner) => self.fix_type(inner),
            RustType::HashMap(key, value) => {
                let fixed_key = self.fix_type(key);
                let fixed_value = self.fix_type(value);
                fixed_key || fixed_value
            }
            RustType::Union(types) | RustType::Tuple(types) => {
                let mut any_fixed = false;
                for t in types {
                    if self.fix_type(t) {
                        any_fixed = true;
                    }
                }
                any_fixed
            }
            _ => false,
        }
    }
}

impl Default for TypeValidator {
    fn default() -> Self {
        Self::new(TypeRegistry::new())
    }
}
