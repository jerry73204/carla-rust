//! FFI code generation for carla-sys
//!
//! This module generates:
//! - FFI function declarations for use in CXX bridge
//! - C++ bridge implementation code
//! - Rust FFI wrapper functions

use crate::{
    analyzer::TypeResolver,
    config::Config,
    error::Result,
    parser::yaml_schema::{Class, Method, Module, Parameter},
};
use convert_case::{Case, Casing};

/// FFI code generator for carla-sys
pub struct FfiGenerator<'a> {
    config: &'a Config,
    type_resolver: &'a TypeResolver,
}

impl<'a> FfiGenerator<'a> {
    /// Create a new FFI generator
    pub fn new(config: &'a Config, type_resolver: &'a TypeResolver) -> Self {
        Self {
            config,
            type_resolver,
        }
    }

    /// Generate FFI declarations for a module
    pub fn generate_ffi_module(&self, module: &Module) -> Result<FfiModule> {
        let mut ffi_module = FfiModule {
            name: module.module_name.clone(),
            classes: Vec::new(),
        };

        for class in &module.classes {
            if self.config.should_generate_class(&class.class_name) {
                let ffi_class = self.generate_ffi_class(class)?;
                ffi_module.classes.push(ffi_class);
            }
        }

        Ok(ffi_module)
    }

    /// Generate FFI declarations for a class
    fn generate_ffi_class(&self, class: &Class) -> Result<FfiClass> {
        let class_name = &class.class_name;
        let mut ffi_class = FfiClass {
            name: class_name.clone(),
            rust_name: to_rust_type_name(class_name),
            methods: Vec::new(),
        };

        // Generate constructor if present
        if let Some(init_method) = class.methods.iter().find(|m| m.def_name == "__init__") {
            let ffi_method = self.generate_ffi_constructor(class_name, init_method)?;
            ffi_class.methods.push(ffi_method);
        }

        // Generate other methods
        for method in &class.methods {
            if method.def_name == "__init__" {
                continue;
            }

            if self.config.should_generate_method(&method.def_name) {
                let ffi_method = self.generate_ffi_method(class_name, method)?;
                ffi_class.methods.push(ffi_method);
            }
        }

        Ok(ffi_class)
    }

    /// Generate FFI constructor
    fn generate_ffi_constructor(&self, class_name: &str, method: &Method) -> Result<FfiMethod> {
        let ffi_name = format!("{class_name}_new");
        let return_type = format!("SharedPtr<{class_name}>");

        Ok(FfiMethod {
            name: ffi_name,
            rust_name: "new".to_string(),
            is_constructor: true,
            is_static: true,
            parameters: self.convert_parameters(&method.params)?,
            return_type: Some(return_type),
            cpp_implementation: None,
        })
    }

    /// Generate FFI method
    fn generate_ffi_method(&self, class_name: &str, method: &Method) -> Result<FfiMethod> {
        let method_name = &method.def_name;
        let rust_name = to_rust_method_name(method_name);
        let ffi_name = format!("{class_name}_{method_name}");

        let mut parameters = Vec::new();

        // Add self parameter for non-static methods
        if !method.is_static {
            parameters.push(FfiParameter {
                name: "self".to_string(),
                cpp_type: format!("const {class_name}*"),
                rust_type: format!("&{class_name}"),
                is_const: true,
                is_ref: true,
            });
        }

        // Add method parameters
        parameters.extend(self.convert_parameters(&method.params)?);

        // Convert return type
        let return_type = method
            .return_type
            .as_ref()
            .map(|t| self.convert_return_type(t))
            .transpose()?;

        Ok(FfiMethod {
            name: ffi_name,
            rust_name,
            is_constructor: false,
            is_static: method.is_static,
            parameters,
            return_type,
            cpp_implementation: None,
        })
    }

    /// Convert method parameters to FFI parameters
    fn convert_parameters(&self, params: &[Parameter]) -> Result<Vec<FfiParameter>> {
        let mut ffi_params = Vec::new();

        for param in params {
            if let Some(ref param_type) = param.param_type {
                let rust_type = self.type_resolver.resolve_type(param_type)?;
                let cpp_type = Self::python_to_cpp_type(param_type)?;

                // Convert Python types to Rust FFI types
                let ffi_rust_type = match param_type.as_str() {
                    "float" => "f64".to_string(),
                    "int" => "i32".to_string(),
                    "str" => "&str".to_string(),
                    "bool" => "bool".to_string(),
                    _ => rust_type.to_rust_string(),
                };

                ffi_params.push(FfiParameter {
                    name: param.param_name.clone(),
                    cpp_type: cpp_type.clone(),
                    rust_type: ffi_rust_type,
                    is_const: self.is_const_param(&cpp_type),
                    is_ref: self.is_ref_param(&cpp_type),
                });
            }
        }

        Ok(ffi_params)
    }

    /// Convert Python return type to C++ type
    fn convert_return_type(&self, python_type: &str) -> Result<String> {
        Self::python_to_cpp_type(python_type)
    }

    /// Map Python types to C++ types
    fn python_to_cpp_type(python_type: &str) -> Result<String> {
        let cpp_type = match python_type {
            "int" => "int32_t".to_string(),
            "float" => "double".to_string(),
            "str" => "std::string".to_string(),
            "bool" => "bool".to_string(),
            "list" => "std::vector<void*>".to_string(), // Generic for now
            "dict" => "std::unordered_map<std::string, void*>".to_string(), // Generic for now
            _ if python_type.starts_with("carla.") => {
                // CARLA types
                let type_name = python_type.strip_prefix("carla.").unwrap_or(python_type);
                format!("carla::client::{type_name}")
            }
            _ if python_type.starts_with("list[") => {
                // Handle list types
                let inner = python_type
                    .trim_start_matches("list[")
                    .trim_end_matches(']');
                let inner_cpp = Self::python_to_cpp_type(inner)?;
                format!("std::vector<{inner_cpp}>")
            }
            _ => python_type.to_string(), // Keep as-is for unknown types
        };

        Ok(cpp_type)
    }

    /// Check if parameter should be const
    fn is_const_param(&self, cpp_type: &str) -> bool {
        !cpp_type.contains("*") || cpp_type.starts_with("const ")
    }

    /// Check if parameter should be passed by reference
    fn is_ref_param(&self, cpp_type: &str) -> bool {
        cpp_type.contains("&") || cpp_type.starts_with("std::")
    }

    /// Generate C++ bridge implementation
    pub fn generate_cpp_bridge(&self, modules: &[FfiModule]) -> Result<String> {
        let mut cpp_code = String::new();

        // Header
        cpp_code.push_str("// Generated C++ bridge implementation\n");
        cpp_code.push_str("#include \"carla_sys_bridge.h\"\n");
        cpp_code.push_str("#include <carla/client/Client.h>\n");
        cpp_code.push_str("#include <carla/client/World.h>\n");
        cpp_code.push_str("#include <carla/client/Actor.h>\n\n");

        cpp_code.push_str("namespace carla_sys {\n\n");

        for module in modules {
            for class in &module.classes {
                cpp_code.push_str(&self.generate_class_cpp_bridge(class)?);
            }
        }

        cpp_code.push_str("\n} // namespace carla_sys\n");

        Ok(cpp_code)
    }

    /// Generate C++ bridge for a class
    fn generate_class_cpp_bridge(&self, class: &FfiClass) -> Result<String> {
        let mut cpp = String::new();

        for method in &class.methods {
            cpp.push_str(&self.generate_method_cpp_bridge(&class.name, method)?);
            cpp.push_str("\n\n");
        }

        Ok(cpp)
    }

    /// Generate C++ bridge for a method
    fn generate_method_cpp_bridge(&self, class_name: &str, method: &FfiMethod) -> Result<String> {
        if self.config.stub_mode {
            // Generate stub implementation
            self.generate_stub_implementation(class_name, method)
        } else {
            // Generate real FFI implementation
            self.generate_ffi_implementation(class_name, method)
        }
    }

    /// Generate stub implementation for docs-only mode
    fn generate_stub_implementation(
        &self,
        _class_name: &str,
        method: &FfiMethod,
    ) -> Result<String> {
        let return_type = method.return_type.as_deref().unwrap_or("void");

        let params = method
            .parameters
            .iter()
            .map(|p| format!("{} {}", p.cpp_type, p.name))
            .collect::<Vec<_>>()
            .join(", ");

        Ok(format!(
            "{} {}({}) {{\n    throw std::runtime_error(\"Stub implementation - not available in docs-only mode\");\n}}",
            return_type,
            method.name,
            params
        ))
    }

    /// Generate real FFI implementation
    fn generate_ffi_implementation(&self, class_name: &str, method: &FfiMethod) -> Result<String> {
        // This would generate the actual C++ implementation
        // For now, return a TODO
        let return_type = method.return_type.as_deref().unwrap_or("void");

        let params = method
            .parameters
            .iter()
            .map(|p| format!("{} {}", p.cpp_type, p.name))
            .collect::<Vec<_>>()
            .join(", ");

        Ok(format!(
            "{} {}({}) {{\n    // TODO: Implement FFI bridge for {}::{}\n    throw std::runtime_error(\"Not implemented\");\n}}",
            return_type,
            method.name,
            params,
            class_name,
            method.rust_name
        ))
    }
}

/// FFI module representation
#[derive(Debug, Clone)]
pub struct FfiModule {
    pub name: String,
    pub classes: Vec<FfiClass>,
}

/// FFI class representation
#[derive(Debug, Clone)]
pub struct FfiClass {
    pub name: String,
    pub rust_name: String,
    pub methods: Vec<FfiMethod>,
}

/// FFI method representation
#[derive(Debug, Clone)]
pub struct FfiMethod {
    pub name: String,
    pub rust_name: String,
    pub is_constructor: bool,
    pub is_static: bool,
    pub parameters: Vec<FfiParameter>,
    pub return_type: Option<String>,
    pub cpp_implementation: Option<String>,
}

/// FFI parameter representation
#[derive(Debug, Clone)]
pub struct FfiParameter {
    pub name: String,
    pub cpp_type: String,
    pub rust_type: String,
    pub is_const: bool,
    pub is_ref: bool,
}

/// Convert class name to Rust type name
fn to_rust_type_name(name: &str) -> String {
    name.to_case(Case::Pascal)
}

/// Convert method name to Rust method name
fn to_rust_method_name(name: &str) -> String {
    if name == "__init__" {
        "new".to_string()
    } else {
        name.to_case(Case::Snake)
    }
}
