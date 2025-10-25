//! Tests for TypeResolverV2

#[cfg(test)]
mod tests {
    use crate::analyzer::{RustType, TypeContext, TypeLocation, TypeResolverV2};
    use std::path::PathBuf;

    fn test_context() -> TypeContext {
        TypeContext {
            location: TypeLocation::Field {
                field_name: "test".to_string(),
            },
            containing_class: "TestClass".to_string(),
            containing_module: "carla".to_string(),
            inside_generic: false,
            source_file: PathBuf::from("test.yml"),
        }
    }

    #[test]
    fn test_primitive_types() {
        let resolver = TypeResolverV2::new();
        let ctx = test_context();

        // Test boolean variants
        let result = resolver.resolve("bool", ctx.clone()).unwrap();
        assert_eq!(result.rust_type, RustType::Primitive("bool".to_string()));

        let result = resolver.resolve("boolean", ctx.clone()).unwrap();
        assert_eq!(result.rust_type, RustType::Primitive("bool".to_string()));

        // Test integer variants
        let result = resolver.resolve("int", ctx.clone()).unwrap();
        assert_eq!(result.rust_type, RustType::Primitive("i32".to_string()));

        let result = resolver.resolve("uint", ctx.clone()).unwrap();
        assert_eq!(result.rust_type, RustType::Primitive("u32".to_string()));

        let result = resolver.resolve("uint16", ctx.clone()).unwrap();
        assert_eq!(result.rust_type, RustType::Primitive("u16".to_string()));

        // Test float types
        let result = resolver.resolve("float", ctx.clone()).unwrap();
        assert_eq!(result.rust_type, RustType::Primitive("f32".to_string()));

        let result = resolver.resolve("double", ctx.clone()).unwrap();
        assert_eq!(result.rust_type, RustType::Primitive("f64".to_string()));
    }

    #[test]
    fn test_carla_types() {
        let resolver = TypeResolverV2::new();
        let ctx = test_context();

        // Test CARLA type with prefix
        let result = resolver.resolve("carla.Actor", ctx.clone()).unwrap();
        assert_eq!(
            result.rust_type,
            RustType::Custom("crate::carla::Actor".to_string())
        );

        // Test CARLA type without prefix (should add module path)
        let result = resolver.resolve("Transform", ctx.clone()).unwrap();
        assert_eq!(
            result.rust_type,
            RustType::Custom("crate::carla::Transform".to_string())
        );
    }

    #[test]
    fn test_collection_types() {
        let resolver = TypeResolverV2::new();
        let ctx = test_context();

        // Test list type
        let result = resolver.resolve("list(carla.Actor)", ctx.clone()).unwrap();
        match result.rust_type {
            RustType::Vec(inner) => {
                assert_eq!(*inner, RustType::Custom("crate::carla::Actor".to_string()));
            }
            _ => panic!("Expected Vec type"),
        }

        // Test dict type
        let result = resolver.resolve("dict[str, float]", ctx.clone()).unwrap();
        match result.rust_type {
            RustType::HashMap(key, value) => {
                assert_eq!(*key, RustType::Custom("String".to_string()));
                assert_eq!(*value, RustType::Primitive("f32".to_string()));
            }
            _ => panic!("Expected HashMap type"),
        }

        // Test bare dict (should become HashMap<String, String>)
        let result = resolver.resolve("dict", ctx.clone()).unwrap();
        match result.rust_type {
            RustType::HashMap(key, value) => {
                assert_eq!(*key, RustType::Custom("String".to_string()));
                assert_eq!(*value, RustType::Custom("String".to_string()));
            }
            _ => panic!("Expected HashMap type"),
        }
    }

    #[test]
    fn test_union_types() {
        let resolver = TypeResolverV2::new();
        let ctx = test_context();

        // Test union with "or"
        let result = resolver.resolve("carla.Actor or int", ctx.clone()).unwrap();
        match result.rust_type {
            RustType::Union(types) => {
                assert_eq!(types.len(), 2);
                assert_eq!(
                    types[0],
                    RustType::Custom("crate::carla::Actor".to_string())
                );
                assert_eq!(types[1], RustType::Primitive("i32".to_string()));
            }
            _ => panic!("Expected Union type"),
        }

        // Test union with "/"
        let result = resolver.resolve("bool / int / float", ctx.clone()).unwrap();
        match result.rust_type {
            RustType::Union(types) => {
                assert_eq!(types.len(), 3);
                assert_eq!(types[0], RustType::Primitive("bool".to_string()));
                assert_eq!(types[1], RustType::Primitive("i32".to_string()));
                assert_eq!(types[2], RustType::Primitive("f32".to_string()));
            }
            _ => panic!("Expected Union type"),
        }
    }

    #[test]
    fn test_special_list_types() {
        let resolver = TypeResolverV2::new();
        let ctx = test_context();

        // Test list with bracketed tuple notation
        let result = resolver
            .resolve("list([name,world, actor, relative])", ctx.clone())
            .unwrap();
        match result.rust_type {
            RustType::Vec(inner) => match inner.as_ref() {
                RustType::Tuple(types) => {
                    assert_eq!(types.len(), 4);
                    assert_eq!(types[0], RustType::Custom("String".to_string()));
                    assert_eq!(
                        types[1],
                        RustType::Custom("crate::carla::Transform".to_string())
                    );
                    assert_eq!(
                        types[2],
                        RustType::Custom("crate::carla::Actor".to_string())
                    );
                    assert_eq!(
                        types[3],
                        RustType::Custom("crate::carla::Transform".to_string())
                    );
                }
                _ => panic!("Expected inner type to be Tuple"),
            },
            _ => panic!("Expected Vec type"),
        }
    }

    #[test]
    fn test_type_validation() {
        let resolver = TypeResolverV2::new();
        let ctx = test_context();

        // Test that validation fixes Python types
        let result = resolver.resolve("dict", ctx.clone()).unwrap();
        assert!(result.rust_type.to_rust_string().contains("HashMap"));

        // Test that bare Vec gets fixed
        let result = resolver.resolve("list", ctx.clone()).unwrap();
        match result.rust_type {
            RustType::Vec(inner) => {
                assert_eq!(*inner, RustType::Custom("crate::carla::Actor".to_string()));
            }
            _ => panic!("Expected Vec type"),
        }
    }

    #[test]
    fn test_import_collection() {
        let resolver = TypeResolverV2::new();
        let ctx = test_context();

        // Test HashMap import collection
        let result = resolver.resolve("dict[str, int]", ctx.clone()).unwrap();
        assert!(result
            .imports_needed
            .contains(&"use std::collections::HashMap;".to_string()));

        // Test no imports for primitive types
        let result = resolver.resolve("bool", ctx.clone()).unwrap();
        assert!(result.imports_needed.is_empty());
    }

    #[test]
    fn test_recursive_type_detection() {
        let resolver = TypeResolverV2::new();
        let mut ctx = test_context();
        ctx.containing_class = "Actor".to_string();
        ctx.location = TypeLocation::Field {
            field_name: "parent".to_string(),
        };

        // Test that self-referential types are detected
        let result = resolver.resolve("carla.Actor", ctx.clone()).unwrap();
        assert_eq!(
            result.rust_type,
            RustType::Custom("crate::carla::Actor".to_string())
        );
        // Note: Boxing decision is made later in the generation phase
    }
}
