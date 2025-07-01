//! Enum detection for CARLA classes

use crate::parser::yaml_schema::Class;
use tracing::debug;

/// Detect if a class represents an enum
pub fn is_enum_class(class: &Class) -> bool {
    // Check if all instance variables have no type (enum variants)
    if class.instance_variables.is_empty() {
        return false;
    }

    // Check if any instance variable has a type
    let has_typed_fields = class
        .instance_variables
        .iter()
        .any(|var| var.var_type.is_some());

    debug!(
        "Class {} has_typed_fields: {}, num_vars: {}",
        class.class_name,
        has_typed_fields,
        class.instance_variables.len()
    );

    for var in &class.instance_variables {
        debug!("  Variable {}: type={:?}", var.var_name, var.var_type);
    }

    // If no typed fields and has instance variables, it's likely an enum
    !has_typed_fields && !class.instance_variables.is_empty() && class.methods.is_empty()
}

/// Get enum variants from a class
pub fn get_enum_variants(class: &Class) -> Vec<&str> {
    if !is_enum_class(class) {
        return Vec::new();
    }

    class
        .instance_variables
        .iter()
        .map(|var| var.var_name.as_str())
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::parser::yaml_schema::InstanceVariable;

    #[test]
    fn test_enum_detection() {
        let enum_class = Class {
            class_name: "VehicleWheelLocation".to_string(),
            parent: None,
            doc: Some("Wheel locations".to_string()),
            instance_variables: vec![
                InstanceVariable {
                    var_name: "FL_Wheel".to_string(),
                    var_type: None,
                    doc: Some("Front left".to_string()),
                    var_units: None,
                    note: None,
                    warning: None,
                },
                InstanceVariable {
                    var_name: "FR_Wheel".to_string(),
                    var_type: None,
                    doc: Some("Front right".to_string()),
                    var_units: None,
                    note: None,
                    warning: None,
                },
            ],
            methods: vec![],
        };

        assert!(is_enum_class(&enum_class));

        let variants = get_enum_variants(&enum_class);
        assert_eq!(variants, vec!["FL_Wheel", "FR_Wheel"]);
    }

    #[test]
    fn test_struct_detection() {
        let struct_class = Class {
            class_name: "Vector3D".to_string(),
            parent: None,
            doc: Some("3D vector".to_string()),
            instance_variables: vec![InstanceVariable {
                var_name: "x".to_string(),
                var_type: Some("float".to_string()),
                doc: Some("X coord".to_string()),
                var_units: None,
                note: None,
                warning: None,
            }],
            methods: vec![],
        };

        assert!(!is_enum_class(&struct_class));
    }

    #[test]
    fn test_class_with_methods_not_enum() {
        let class_with_methods = Class {
            class_name: "Actor".to_string(),
            parent: None,
            doc: Some("Actor class".to_string()),
            instance_variables: vec![InstanceVariable {
                var_name: "id".to_string(),
                var_type: Some("int".to_string()),
                doc: Some("ID".to_string()),
                var_units: None,
                note: None,
                warning: None,
            }],
            methods: vec![crate::parser::yaml_schema::Method {
                def_name: "get_id".to_string(),
                return_type: Some("int".to_string()),
                params: vec![],
                doc: None,
                warning: None,
                note: None,
                raises: None,
                is_static: false,
                return_units: None,
            }],
        };

        assert!(!is_enum_class(&class_with_methods));
    }
}
