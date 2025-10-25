//! Tests for TypeValidator

#[cfg(test)]
mod tests {
    use crate::analyzer::{ResolvedType, RustType, TypeRegistry, TypeValidator, ValidationIssue};

    #[test]
    fn test_python_type_corrections() {
        let validator = TypeValidator::new(TypeRegistry::new());

        // Test dict -> HashMap
        let mut rust_type = RustType::Custom("dict".to_string());
        assert!(validator.fix_type(&mut rust_type));
        assert_eq!(rust_type, RustType::Custom("HashMap".to_string()));

        // Test boolean -> bool
        let mut rust_type = RustType::Custom("boolean".to_string());
        assert!(validator.fix_type(&mut rust_type));
        assert_eq!(rust_type, RustType::Custom("bool".to_string()));

        // Test uint -> u32
        let mut rust_type = RustType::Custom("uint".to_string());
        assert!(validator.fix_type(&mut rust_type));
        assert_eq!(rust_type, RustType::Custom("u32".to_string()));

        // Test uint16 -> u16
        let mut rust_type = RustType::Custom("uint16".to_string());
        assert!(validator.fix_type(&mut rust_type));
        assert_eq!(rust_type, RustType::Custom("u16".to_string()));
    }

    #[test]
    fn test_bare_generic_detection() {
        let validator = TypeValidator::new(TypeRegistry::new());

        let resolved = ResolvedType {
            rust_type: RustType::Custom("Vec".to_string()),
            imports_needed: vec![],
            needs_boxing: false,
            type_alias: None,
        };

        let issues = validator.validate_resolved_type(&resolved, "test_location");
        assert_eq!(issues.len(), 1);
        match &issues[0] {
            ValidationIssue::BareGeneric { type_name, .. } => {
                assert_eq!(type_name, "Vec");
            }
            _ => panic!("Expected BareGeneric issue"),
        }
    }

    #[test]
    fn test_unqualified_carla_types() {
        let validator = TypeValidator::new(TypeRegistry::new());

        // Test unqualified CARLA type gets module path
        let mut rust_type = RustType::Custom("Actor".to_string());
        assert!(validator.fix_type(&mut rust_type));
        assert_eq!(
            rust_type,
            RustType::Custom("crate::carla::Actor".to_string())
        );

        // Test unqualified Transform
        let mut rust_type = RustType::Custom("Transform".to_string());
        assert!(validator.fix_type(&mut rust_type));
        assert_eq!(
            rust_type,
            RustType::Custom("crate::carla::Transform".to_string())
        );

        // Test that String is not modified
        let mut rust_type = RustType::Custom("String".to_string());
        assert!(!validator.fix_type(&mut rust_type));
        assert_eq!(rust_type, RustType::Custom("String".to_string()));
    }

    #[test]
    fn test_nested_type_fixes() {
        let validator = TypeValidator::new(TypeRegistry::new());

        // Test Vec<dict> -> Vec<HashMap>
        let mut rust_type = RustType::Vec(Box::new(RustType::Custom("dict".to_string())));
        assert!(validator.fix_type(&mut rust_type));
        match rust_type {
            RustType::Vec(inner) => {
                assert_eq!(*inner, RustType::Custom("HashMap".to_string()));
            }
            _ => panic!("Expected Vec type"),
        }

        // Test Option<Actor> -> Option<crate::carla::Actor>
        let mut rust_type = RustType::Option(Box::new(RustType::Custom("Actor".to_string())));
        assert!(validator.fix_type(&mut rust_type));
        match rust_type {
            RustType::Option(inner) => {
                assert_eq!(*inner, RustType::Custom("crate::carla::Actor".to_string()));
            }
            _ => panic!("Expected Option type"),
        }
    }

    #[test]
    fn test_copy_trait_validation() {
        let validator = TypeValidator::new(TypeRegistry::new());

        // Test struct with copyable fields
        let copyable_fields = vec![
            ("x".to_string(), RustType::Primitive("f32".to_string())),
            ("y".to_string(), RustType::Primitive("f32".to_string())),
            ("z".to_string(), RustType::Primitive("f32".to_string())),
        ];

        let issues = validator.validate_derives(&["Copy".to_string()], &copyable_fields);
        assert!(issues.is_empty());

        // Test struct with non-copyable field
        let non_copyable_fields = vec![
            ("name".to_string(), RustType::Custom("String".to_string())),
            ("id".to_string(), RustType::Primitive("i32".to_string())),
        ];

        let issues = validator.validate_derives(&["Copy".to_string()], &non_copyable_fields);
        assert_eq!(issues.len(), 1);
        match &issues[0] {
            ValidationIssue::InvalidTrait { trait_name, reason } => {
                assert_eq!(trait_name, "Copy");
                assert!(reason.contains("name"));
            }
            _ => panic!("Expected InvalidTrait issue"),
        }
    }

    #[test]
    fn test_hash_trait_validation() {
        let validator = TypeValidator::new(TypeRegistry::new());

        // Test struct with hashable fields
        let hashable_fields = vec![
            ("id".to_string(), RustType::Primitive("i32".to_string())),
            ("name".to_string(), RustType::Custom("String".to_string())),
        ];

        let issues = validator.validate_derives(&["Hash".to_string()], &hashable_fields);
        assert!(issues.is_empty());

        // Test struct with float field (not hashable)
        let float_fields = vec![
            ("x".to_string(), RustType::Primitive("f32".to_string())),
            ("y".to_string(), RustType::Primitive("f32".to_string())),
        ];

        let issues = validator.validate_derives(&["Hash".to_string()], &float_fields);
        assert_eq!(issues.len(), 1);
        match &issues[0] {
            ValidationIssue::InvalidTrait { trait_name, .. } => {
                assert_eq!(trait_name, "Hash");
            }
            _ => panic!("Expected InvalidTrait issue"),
        }
    }

    #[test]
    fn test_circular_dependency_detection() {
        let validator = TypeValidator::new(TypeRegistry::new());
        let mut deps = std::collections::HashMap::new();

        // Create circular dependency: A -> B -> C -> A
        deps.insert("A".to_string(), vec!["B".to_string()]);
        deps.insert("B".to_string(), vec!["C".to_string()]);
        deps.insert("C".to_string(), vec!["A".to_string()]);

        let issues = validator.check_circular_dependencies(&deps);
        assert!(!issues.is_empty());
        match &issues[0] {
            ValidationIssue::CircularDependency { types } => {
                assert!(types.contains(&"A".to_string()));
                assert!(types.contains(&"B".to_string()));
                assert!(types.contains(&"C".to_string()));
            }
            _ => panic!("Expected CircularDependency issue"),
        }
    }
}
