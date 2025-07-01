//! Module normalization - infer missing types and fix common patterns

#[allow(unused_imports)]
use crate::parser::yaml_schema::{Class, Method, Module, Parameter};
use tracing::debug;

/// Normalize modules by inferring missing types and fixing patterns
pub fn normalize_modules(modules: &mut [Module]) {
    for module in modules {
        normalize_module(module);
    }
}

/// Normalize a single module
pub fn normalize_module(module: &mut Module) {
    for class in &mut module.classes {
        normalize_class(class);
    }
}

/// Normalize a class
pub fn normalize_class(class: &mut Class) {
    for method in &mut class.methods {
        normalize_method(method, &class.class_name);
    }
}

/// Normalize a method - infer missing parameter types for special methods
pub fn normalize_method(method: &mut Method, class_name: &str) {
    // Skip if not a special method
    if !method.def_name.starts_with("__") {
        return;
    }

    for param in &mut method.params {
        // Skip self parameter
        if param.param_name == "self" {
            continue;
        }

        // If parameter already has a type, skip
        if let Some(ref existing_type) = param.param_type {
            debug!(
                "Parameter '{}' in method '{}' already has type '{}', skipping inference",
                param.param_name, method.def_name, existing_type
            );
            continue;
        }

        // Infer type based on method and parameter name
        if let Some(inferred_type) =
            infer_parameter_type(&method.def_name, &param.param_name, class_name)
        {
            debug!(
                "Inferred type '{}' for parameter '{}' in method '{}' of class '{}'",
                inferred_type, param.param_name, method.def_name, class_name
            );
            param.param_type = Some(inferred_type);
        }
    }
}

/// Infer parameter type based on special method patterns
fn infer_parameter_type(method_name: &str, param_name: &str, class_name: &str) -> Option<String> {
    match method_name {
        // Comparison methods - 'other' is same type as self
        // NOTE: We only infer if no explicit type is provided
        "__eq__" | "__ne__" | "__lt__" | "__le__" | "__gt__" | "__ge__" => {
            if param_name == "other" {
                Some(class_name.to_string())
            } else {
                None
            }
        }

        // Arithmetic operators - 'other' or 'rhs' is same type as self
        "__add__" | "__sub__" | "__mul__" | "__div__" | "__truediv__" | "__floordiv__"
        | "__mod__" | "__pow__" => {
            if param_name == "other" || param_name == "rhs" {
                Some(class_name.to_string())
            } else {
                None
            }
        }

        // Reverse arithmetic operators - 'other' or 'lhs' is same type as self
        "__radd__" | "__rsub__" | "__rmul__" | "__rdiv__" | "__rtruediv__" | "__rfloordiv__"
        | "__rmod__" | "__rpow__" => {
            if param_name == "other" || param_name == "lhs" {
                Some(class_name.to_string())
            } else {
                None
            }
        }

        // In-place operators - 'other' or 'rhs' is same type as self
        "__iadd__" | "__isub__" | "__imul__" | "__idiv__" | "__itruediv__" | "__ifloordiv__"
        | "__imod__" | "__ipow__" => {
            if param_name == "other" || param_name == "rhs" {
                Some(class_name.to_string())
            } else {
                None
            }
        }

        // Bitwise operators - 'other' or 'rhs' is same type as self
        "__and__" | "__or__" | "__xor__" | "__lshift__" | "__rshift__" => {
            if param_name == "other" || param_name == "rhs" {
                Some(class_name.to_string())
            } else {
                None
            }
        }

        // Bitwise in-place operators
        "__iand__" | "__ior__" | "__ixor__" | "__ilshift__" | "__irshift__" => {
            if param_name == "other" || param_name == "rhs" {
                Some(class_name.to_string())
            } else {
                None
            }
        }

        // Index operations - common patterns
        "__getitem__" => {
            if param_name == "key" || param_name == "index" {
                // Default to int for index, but this could be customized
                Some("int".to_string())
            } else {
                None
            }
        }

        "__setitem__" => {
            match param_name {
                "key" | "index" => Some("int".to_string()),
                "value" => None, // Value type varies by container
                _ => None,
            }
        }

        "__delitem__" => {
            if param_name == "key" || param_name == "index" {
                Some("int".to_string())
            } else {
                None
            }
        }

        // Contains - item type varies by container
        "__contains__" => {
            // Can't infer item type generically
            None
        }

        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_infer_comparison_operators() {
        let mut method = Method {
            def_name: "__eq__".to_string(),
            params: vec![
                Parameter {
                    param_name: "self".to_string(),
                    param_type: None,
                    default: None,
                    param_units: None,
                    doc: None,
                },
                Parameter {
                    param_name: "other".to_string(),
                    param_type: None,
                    default: None,
                    param_units: None,
                    doc: None,
                },
            ],
            return_type: Some("bool".to_string()),
            return_units: None,
            doc: None,
            note: None,
            warning: None,
            raises: None,
            is_static: false,
        };

        normalize_method(&mut method, "carla.Location");

        assert_eq!(
            method.params[1].param_type,
            Some("carla.Location".to_string())
        );
    }

    #[test]
    fn test_infer_arithmetic_operators() {
        let mut method = Method {
            def_name: "__add__".to_string(),
            params: vec![
                Parameter {
                    param_name: "self".to_string(),
                    param_type: None,
                    default: None,
                    param_units: None,
                    doc: None,
                },
                Parameter {
                    param_name: "other".to_string(),
                    param_type: None,
                    default: None,
                    param_units: None,
                    doc: None,
                },
            ],
            return_type: Some("carla.Vector3D".to_string()),
            return_units: None,
            doc: None,
            note: None,
            warning: None,
            raises: None,
            is_static: false,
        };

        normalize_method(&mut method, "carla.Vector3D");

        assert_eq!(
            method.params[1].param_type,
            Some("carla.Vector3D".to_string())
        );
    }

    #[test]
    fn test_no_inference_for_non_special_methods() {
        let mut method = Method {
            def_name: "distance".to_string(),
            params: vec![
                Parameter {
                    param_name: "self".to_string(),
                    param_type: None,
                    default: None,
                    param_units: None,
                    doc: None,
                },
                Parameter {
                    param_name: "other".to_string(),
                    param_type: None,
                    default: None,
                    param_units: None,
                    doc: None,
                },
            ],
            return_type: Some("float".to_string()),
            return_units: None,
            doc: None,
            note: None,
            warning: None,
            raises: None,
            is_static: false,
        };

        normalize_method(&mut method, "carla.Location");

        // Should not infer type for non-special methods
        assert_eq!(method.params[1].param_type, None);
    }
}
