//! Generator context for code generation

use crate::{
    analyzer::{
        determine_self_type_with_config, AnalysisResult, InheritanceResolver, SelfType,
        TypeResolver,
    },
    config::Config,
    parser::yaml_schema::{Method, Module, Parameter},
};
use convert_case::{Case, Casing};
use serde::Serialize;

/// Context for code generation containing all necessary data
pub struct GeneratorContext<'a> {
    pub module: &'a Module,
    pub type_resolver: &'a TypeResolver,
    pub inheritance_resolver: &'a InheritanceResolver,
    pub config: &'a Config,
}

impl<'a> GeneratorContext<'a> {
    /// Create a new generator context
    pub fn new(
        module: &'a Module,
        type_resolver: &'a TypeResolver,
        inheritance_resolver: &'a InheritanceResolver,
        config: &'a Config,
    ) -> Self {
        Self {
            module,
            type_resolver,
            inheritance_resolver,
            config,
        }
    }

    /// Convert a Python method name to Rust naming convention
    pub fn rust_method_name(&self, python_name: &str) -> String {
        let mut name = python_name.to_string();

        // Handle special Python methods
        match python_name {
            "__init__" => return "new".to_string(),
            "__str__" => return "to_string".to_string(),
            "__repr__" => return "fmt".to_string(),
            "__eq__" => return "eq".to_string(),
            "__ne__" => return "ne".to_string(),
            "__hash__" => return "hash".to_string(),
            "__len__" => return "len".to_string(),
            "__iter__" => return "iter".to_string(),
            "__contains__" => return "contains".to_string(),
            "__getitem__" => return "get".to_string(),
            "__setitem__" => return "set".to_string(),
            _ => {}
        }

        // Remove prefixes based on config
        for prefix in &self.config.naming.remove_prefix {
            if name.starts_with(prefix) {
                name = name.strip_prefix(prefix).unwrap().to_string();
            }
        }

        // Convert to snake_case
        name.to_case(Case::Snake)
    }

    /// Convert a Python class name to Rust naming convention
    pub fn rust_class_name(&self, python_name: &str) -> String {
        let clean_name = python_name.strip_prefix("carla.").unwrap_or(python_name);
        clean_name.to_case(Case::Pascal)
    }

    /// Convert a Python parameter to Rust parameter
    pub fn rust_parameter(&self, param: &Parameter) -> crate::Result<RustParameter> {
        let rust_type = if param.param_name == "self" {
            "&self".to_string()
        } else if let Some(param_type) = &param.param_type {
            self.type_resolver
                .resolve_type(param_type)?
                .to_rust_string()
        } else {
            // Default to String if no type specified
            "String".to_string()
        };

        // Convert default value to string representation
        let default_string = param.default.as_ref().map(|val| {
            match val {
                serde_yaml::Value::String(s) => s.clone(),
                serde_yaml::Value::Number(n) => n.to_string(),
                serde_yaml::Value::Bool(b) => b.to_string(),
                serde_yaml::Value::Sequence(seq) => {
                    // Convert array to string representation (e.g., [1, 2, 3])
                    format!(
                        "[{}]",
                        seq.iter()
                            .map(|v| match v {
                                serde_yaml::Value::String(s) => format!("\"{s}\""),
                                serde_yaml::Value::Number(n) => n.to_string(),
                                serde_yaml::Value::Bool(b) => b.to_string(),
                                _ => format!("{v:?}"),
                            })
                            .collect::<Vec<_>>()
                            .join(", ")
                    )
                }
                _ => format!("{val:?}"),
            }
        });

        Ok(RustParameter {
            name: param.param_name.clone(),
            rust_type,
            doc: param.doc.clone(),
            default: default_string,
            units: param.param_units.clone(),
        })
    }

    /// Convert a method to Rust method info
    pub fn rust_method(&self, method: &Method, class_name: &str) -> crate::Result<RustMethod> {
        let rust_name = self.rust_method_name(&method.def_name);

        // Convert parameters (excluding self)
        let mut rust_params = Vec::new();
        for param in &method.params {
            // Skip 'self' parameter
            if param.param_name == "self" {
                continue;
            }
            rust_params.push(self.rust_parameter(param)?);
        }

        // Determine self type
        let self_type = if method.is_static {
            None
        } else {
            Some(determine_self_type_with_config(
                method,
                class_name,
                self.config,
            ))
        };

        // Check for return type override
        let return_type = if let Some(override_type) = self
            .config
            .get_return_type_override(class_name, &method.def_name)
        {
            override_type.clone()
        } else if let Some(ret) = &method.return_type {
            if ret == "None" || ret.is_empty() {
                if self.config.method_signatures.use_result_return_types {
                    self.config.method_signatures.error_type.clone()
                } else {
                    "()".to_string()
                }
            } else {
                match self.type_resolver.resolve_type(ret) {
                    Ok(rust_type) => {
                        let resolved_type = rust_type.to_rust_string();
                        if self.config.method_signatures.use_result_return_types {
                            format!("Result<{resolved_type}>")
                        } else {
                            resolved_type
                        }
                    }
                    Err(e) => {
                        tracing::warn!("Failed to resolve return type '{}': {}", ret, e);
                        // Fall back to error type
                        if self.config.method_signatures.use_result_return_types {
                            self.config.method_signatures.error_type.clone()
                        } else {
                            "()".to_string()
                        }
                    }
                }
            }
        } else if self.config.method_signatures.use_result_return_types {
            self.config.method_signatures.error_type.clone()
        } else {
            "()".to_string()
        };

        // Generate FFI function name
        let method_pascal = method.def_name.to_case(Case::Pascal);
        let ffi_function = format!("{class_name}_{method_pascal}");

        Ok(RustMethod {
            name: rust_name,
            self_type,
            params: rust_params,
            return_type,
            doc: method.doc.clone(),
            warning: method.warning.clone(),
            note: method.note.clone(),
            raises: method.raises.clone(),
            is_static: method.is_static,
            return_units: method.return_units.clone(),
            ffi_function,
            has_ffi_impl: true, // Assume FFI implementation needed
        })
    }

    /// Check if a method should have a builder pattern
    pub fn needs_builder(&self, method: &Method) -> bool {
        let optional_params = method
            .params
            .iter()
            .filter(|p| p.param_name != "self" && p.default.is_some())
            .count();

        optional_params >= self.config.builder_threshold
    }
}

/// Rust parameter information
#[derive(Debug, Clone, Serialize)]
pub struct RustParameter {
    pub name: String,
    pub rust_type: String,
    pub doc: Option<String>,
    pub default: Option<String>,
    pub units: Option<String>,
}

/// Rust method information
#[derive(Debug, Clone, Serialize)]
pub struct RustMethod {
    pub name: String,
    pub self_type: Option<SelfType>,
    pub params: Vec<RustParameter>,
    pub return_type: String,
    pub doc: Option<String>,
    pub warning: Option<String>,
    pub note: Option<String>,
    pub raises: Option<String>,
    pub is_static: bool,
    pub return_units: Option<String>,
    pub ffi_function: String,
    pub has_ffi_impl: bool,
}

/// Rust field information
#[derive(Debug, Clone, Serialize)]
pub struct RustField {
    pub name: String,
    pub rust_type: String,
    pub doc: Option<String>,
    pub units: Option<String>,
    pub warning: Option<String>,
    pub note: Option<String>,
}

/// Data for struct template
#[derive(Debug, Clone, Serialize)]
pub struct StructData {
    pub name: String,
    pub doc: Option<String>,
    pub fields: Vec<RustField>,
    pub has_ffi_inner: bool,
    pub ffi_type: String,
    pub derives: Vec<String>,
    #[serde(skip)]
    pub special_methods_analysis: Option<AnalysisResult>,
}

/// Data for impl template
#[derive(Debug, Clone, Serialize)]
pub struct ImplData {
    pub name: String,
    pub methods: Vec<RustMethod>,
}

/// Data for module template
#[derive(Debug, Clone, Serialize)]
pub struct ModuleData {
    pub module_doc: Option<String>,
    pub use_statements: Vec<String>,
    pub structs: Vec<StructInfo>,
}

/// Struct information for module template
#[derive(Debug, Clone, Serialize)]
pub struct StructInfo {
    pub rendered: String,
    pub impl_rendered: String,
    pub has_impl: bool,
}
