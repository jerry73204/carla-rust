//! Integration tests for carla-codegen

use carla_codegen::{
    analyzer::{DependencyGraph, InheritanceResolver, TypeResolver},
    config::Config,
    parser::YamlParser,
};
use tempfile::TempDir;

#[test]
fn test_basic_yaml_parsing() {
    let yaml_content = r#"
- module_name: carla
  classes:
  - class_name: Vector3D
    doc: >
      Helper class to perform 3D operations.
    instance_variables:
    - var_name: x
      type: float
      doc: >
        X-axis value.
    - var_name: y
      type: float
      doc: >
        Y-axis value.
    - var_name: z
      type: float
      doc: >
        Z-axis value.
    methods:
    - def_name: __init__
      params:
      - param_name: x
        type: float
        default: 0.0
      - param_name: y
        type: float
        default: 0.0
      - param_name: z
        type: float
        default: 0.0
    - def_name: length
      return: float
      doc: >
        Computes the length of the vector.
"#;

    let dir = TempDir::new().unwrap();
    let file_path = dir.path().join("test.yml");
    std::fs::write(&file_path, yaml_content).unwrap();

    let mut parser = YamlParser::new();
    parser.parse_file(&file_path).unwrap();

    let modules = parser.modules();
    assert_eq!(modules.len(), 1);
    assert_eq!(modules[0].module_name, "carla");
    assert_eq!(modules[0].classes.len(), 1);
    assert_eq!(modules[0].classes[0].class_name, "Vector3D");
    assert_eq!(modules[0].classes[0].instance_variables.len(), 3);
    assert_eq!(modules[0].classes[0].methods.len(), 2);
}

#[test]
fn test_type_resolver() {
    let resolver = TypeResolver::new();

    // Test primitive types
    assert_eq!(
        resolver.resolve_type("int").unwrap().to_rust_string(),
        "i32"
    );
    assert_eq!(
        resolver.resolve_type("float").unwrap().to_rust_string(),
        "f32"
    );
    assert_eq!(
        resolver.resolve_type("bool").unwrap().to_rust_string(),
        "bool"
    );
    assert_eq!(
        resolver.resolve_type("str").unwrap().to_rust_string(),
        "String"
    );

    // Test list types
    assert_eq!(
        resolver.resolve_type("list(int)").unwrap().to_rust_string(),
        "Vec<i32>"
    );
    assert_eq!(
        resolver
            .resolve_type("list[float]")
            .unwrap()
            .to_rust_string(),
        "Vec<f32>"
    );

    // Test CARLA types
    assert_eq!(
        resolver
            .resolve_type("carla.Vector3D")
            .unwrap()
            .to_rust_string(),
        "crate::carla::Vector3D"
    );
}

#[test]
fn test_inheritance_resolution() {
    let yaml_content = r#"
- module_name: carla
  classes:
  - class_name: Actor
    doc: Base class for all actors
  - class_name: Vehicle
    parent: carla.Actor
    doc: Vehicle actor
  - class_name: Walker
    parent: carla.Actor
    doc: Walker actor
"#;

    let dir = TempDir::new().unwrap();
    let file_path = dir.path().join("test.yml");
    std::fs::write(&file_path, yaml_content).unwrap();

    let mut parser = YamlParser::new();
    parser.parse_file(&file_path).unwrap();

    let modules = parser.modules();

    let mut inheritance = InheritanceResolver::new();
    inheritance.build_from_modules(modules);

    assert!(inheritance.inherits_from("Vehicle", "Actor"));
    assert!(inheritance.inherits_from("Walker", "Actor"));
    assert!(!inheritance.inherits_from("Actor", "Vehicle"));

    assert_eq!(inheritance.get_root_classes(), vec!["Actor"]);
    assert!(inheritance
        .get_leaf_classes()
        .contains(&"Vehicle".to_string()));
    assert!(inheritance
        .get_leaf_classes()
        .contains(&"Walker".to_string()));
}

#[test]
fn test_dependency_graph() {
    let yaml_content = r#"
- module_name: carla
  classes:
  - class_name: Transform
    doc: Transform class
  - class_name: Actor
    doc: Base class for all actors
    methods:
    - def_name: get_transform
      return: carla.Transform
  - class_name: Vehicle
    parent: carla.Actor
    doc: Vehicle actor
"#;

    let dir = TempDir::new().unwrap();
    let file_path = dir.path().join("test.yml");
    std::fs::write(&file_path, yaml_content).unwrap();

    let mut parser = YamlParser::new();
    parser.parse_file(&file_path).unwrap();

    let modules = parser.modules();

    let mut dep_graph = DependencyGraph::new();
    dep_graph.build_from_modules(modules).unwrap();

    // Check that Actor depends on Transform
    let actor_deps = dep_graph.get_dependencies("Actor").unwrap();
    assert!(actor_deps.contains("Transform"));

    // Check topological sort
    let sorted = dep_graph.topological_sort().unwrap();

    // Debug: print the sorted order
    eprintln!("Topological sort order: {sorted:?}");

    let transform_idx = sorted.iter().position(|x| x == "Transform").unwrap();
    let actor_idx = sorted.iter().position(|x| x == "Actor").unwrap();
    let vehicle_idx = sorted.iter().position(|x| x == "Vehicle").unwrap();

    assert!(
        transform_idx < actor_idx,
        "Transform should come before Actor"
    );
    assert!(actor_idx < vehicle_idx, "Actor should come before Vehicle");
}

#[test]
fn test_config_loading() {
    let config = Config::default();
    assert_eq!(config.builder_threshold, 2);
    assert_eq!(config.module_structure, "nested");
    assert_eq!(config.naming.method_case, "snake_case");
    assert!(config.documentation.include_warnings);
}
