//! YAML structure validation

use crate::{
    error::{CodegenError, Result},
    parser::yaml_schema::*,
};
use tracing::warn;

/// Validate a module structure
pub fn validate_module(module: &Module) -> Result<()> {
    if module.module_name.is_empty() {
        return Err(CodegenError::InvalidStructure(
            "Module name cannot be empty".to_string(),
        ));
    }

    for class in &module.classes {
        validate_class(class)?;
    }

    Ok(())
}

/// Validate a class structure
pub fn validate_class(class: &Class) -> Result<()> {
    if class.class_name.is_empty() {
        return Err(CodegenError::InvalidStructure(
            "Class name cannot be empty".to_string(),
        ));
    }

    // Validate parent reference if present
    if let Some(parent) = &class.parent {
        if parent.is_empty() {
            return Err(CodegenError::InvalidStructure(format!(
                "Empty parent reference in class {}",
                class.class_name
            )));
        }
    }

    // Validate instance variables
    for var in &class.instance_variables {
        validate_instance_variable(var, &class.class_name)?;
    }

    // Validate methods
    for method in &class.methods {
        validate_method(method, &class.class_name)?;
    }

    Ok(())
}

/// Validate an instance variable
pub fn validate_instance_variable(var: &InstanceVariable, class_name: &str) -> Result<()> {
    if var.var_name.is_empty() {
        return Err(CodegenError::InvalidStructure(format!(
            "Instance variable name cannot be empty in class {class_name}"
        )));
    }

    if let Some(ref var_type) = var.var_type {
        if var_type.is_empty() {
            return Err(CodegenError::InvalidStructure(format!(
                "Instance variable '{}' has empty type in class {class_name}",
                var.var_name
            )));
        }
    }
    // Note: Variables without type are allowed (e.g., enum variants)

    Ok(())
}

/// Validate a method
pub fn validate_method(method: &Method, class_name: &str) -> Result<()> {
    if method.def_name.is_empty() {
        return Err(CodegenError::InvalidStructure(format!(
            "Method name cannot be empty in class {class_name}"
        )));
    }

    // Validate parameters
    for param in &method.params {
        validate_parameter(param, &method.def_name, class_name)?;
    }

    // Methods without return type are valid - they return None in Python, () in Rust
    // No warning needed

    Ok(())
}

/// Check if a method is a special Python method where parameter types can be inferred
fn is_special_method_with_implicit_types(method_name: &str, param_name: &str) -> bool {
    match method_name {
        // Comparison methods where 'other' is typically the same type as self
        "__eq__" | "__ne__" | "__lt__" | "__le__" | "__gt__" | "__ge__" => param_name == "other",
        // Arithmetic operators where 'other' is typically the same type as self
        "__add__" | "__sub__" | "__mul__" | "__div__" | "__mod__" | "__pow__" => {
            param_name == "other" || param_name == "rhs"
        }
        // Reverse arithmetic operators
        "__radd__" | "__rsub__" | "__rmul__" | "__rdiv__" | "__rmod__" | "__rpow__" => {
            param_name == "other" || param_name == "lhs"
        }
        // In-place operators
        "__iadd__" | "__isub__" | "__imul__" | "__idiv__" | "__imod__" | "__ipow__" => {
            param_name == "other" || param_name == "rhs"
        }
        // Bitwise operators
        "__and__" | "__or__" | "__xor__" | "__lshift__" | "__rshift__" => {
            param_name == "other" || param_name == "rhs"
        }
        // Contains method where 'item' type depends on container
        "__contains__" => param_name == "item",
        // Index/slice methods
        "__getitem__" | "__setitem__" | "__delitem__" => {
            param_name == "key" || param_name == "index" || param_name == "value"
        }
        _ => false,
    }
}

/// Validate a parameter
pub fn validate_parameter(param: &Parameter, method_name: &str, class_name: &str) -> Result<()> {
    if param.param_name.is_empty() {
        return Err(CodegenError::InvalidStructure(format!(
            "Parameter name cannot be empty in method '{method_name}' of class '{class_name}'"
        )));
    }

    // Warn about parameters without type (except 'self' and special method parameters)
    if param.param_type.is_none()
        && param.param_name != "self"
        && !is_special_method_with_implicit_types(method_name, &param.param_name)
    {
        warn!(
            "Parameter '{}' in method '{}' of class '{}' has no type specified",
            param.param_name, method_name, class_name
        );
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_validate_valid_module() {
        let module = Module {
            module_name: "carla".to_string(),
            doc: Some("Test module".to_string()),
            classes: vec![Class {
                class_name: "TestClass".to_string(),
                parent: None,
                doc: Some("Test class".to_string()),
                instance_variables: vec![InstanceVariable {
                    var_name: "x".to_string(),
                    var_type: Some("float".to_string()),
                    doc: Some("X coordinate".to_string()),
                    var_units: None,
                    note: None,
                    warning: None,
                }],
                methods: vec![Method {
                    def_name: "test_method".to_string(),
                    return_type: Some("int".to_string()),
                    return_units: None,
                    params: vec![Parameter {
                        param_name: "self".to_string(),
                        param_type: None,
                        default: None,
                        param_units: None,
                        doc: None,
                    }],
                    doc: Some("Test method".to_string()),
                    note: None,
                    warning: None,
                    raises: None,
                    is_static: false,
                }],
            }],
        };

        assert!(validate_module(&module).is_ok());
    }

    #[test]
    fn test_validate_empty_module_name() {
        let module = Module {
            module_name: "".to_string(),
            doc: None,
            classes: vec![],
        };

        assert!(matches!(
            validate_module(&module),
            Err(CodegenError::InvalidStructure(_))
        ));
    }
}
