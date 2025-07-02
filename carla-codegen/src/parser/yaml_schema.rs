//! YAML schema definitions for CARLA Python API documentation

use serde::{Deserialize, Serialize};

/// Root structure of a YAML file
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct YamlRoot {
    #[serde(flatten)]
    pub modules: Vec<Module>,
}

/// Module definition in YAML
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Module {
    pub module_name: String,
    #[serde(default)]
    pub doc: Option<String>,
    #[serde(default)]
    pub classes: Vec<Class>,
    /// Source file path (not serialized, added during parsing)
    #[serde(skip)]
    pub source_file: Option<std::path::PathBuf>,
}

/// Class definition in YAML
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Class {
    pub class_name: String,
    #[serde(default)]
    pub parent: Option<String>,
    #[serde(default)]
    pub doc: Option<String>,
    #[serde(default)]
    pub instance_variables: Vec<InstanceVariable>,
    #[serde(default)]
    pub methods: Vec<Method>,
}

/// Instance variable definition
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InstanceVariable {
    pub var_name: String,
    #[serde(rename = "type")]
    #[serde(default)]
    pub var_type: Option<String>,
    #[serde(default)]
    pub doc: Option<String>,
    #[serde(default)]
    pub var_units: Option<String>,
    #[serde(default)]
    pub note: Option<String>,
    #[serde(default)]
    pub warning: Option<String>,
}

/// Method definition
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Method {
    pub def_name: String,
    #[serde(rename = "return", default)]
    pub return_type: Option<String>,
    #[serde(default)]
    pub return_units: Option<String>,
    #[serde(default)]
    pub params: Vec<Parameter>,
    #[serde(default)]
    pub doc: Option<String>,
    #[serde(default)]
    pub note: Option<String>,
    #[serde(default)]
    pub warning: Option<String>,
    #[serde(default)]
    pub raises: Option<String>,
    #[serde(rename = "static", default)]
    pub is_static: bool,
}

/// Method parameter definition
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Parameter {
    pub param_name: String,
    #[serde(rename = "type", default)]
    pub param_type: Option<String>,
    #[serde(default)]
    pub default: Option<serde_yaml::Value>,
    #[serde(default)]
    pub param_units: Option<String>,
    #[serde(default)]
    pub doc: Option<String>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_simple_class() {
        let yaml = r#"
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
    methods:
    - def_name: __init__
      params:
      - param_name: x
        type: float
        default: 0.0
"#;

        let modules: Vec<Module> = serde_yaml::from_str(yaml).unwrap();
        assert_eq!(modules.len(), 1);
        assert_eq!(modules[0].module_name, "carla");
        assert_eq!(modules[0].classes.len(), 1);
        assert_eq!(modules[0].classes[0].class_name, "Vector3D");
    }
}
