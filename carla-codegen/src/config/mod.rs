//! Configuration for code generation

use serde::{Deserialize, Serialize};
use std::path::{Path, PathBuf};

/// Main configuration struct
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Config {
    /// Output directory for generated code
    #[serde(default = "default_output_dir")]
    pub output_dir: PathBuf,

    /// YAML input directory
    #[serde(default)]
    pub yaml_dir: Option<PathBuf>,

    /// Generate builder patterns for methods with >N optional params
    #[serde(default = "default_builder_threshold")]
    pub builder_threshold: usize,

    /// Generate async versions of methods
    #[serde(default)]
    pub generate_async: bool,

    /// Module organization ("nested" or "flat")
    #[serde(default = "default_module_structure")]
    pub module_structure: String,

    /// Type mapping configuration
    #[serde(default)]
    pub type_mapping: TypeMappingConfig,

    /// Filter configuration
    #[serde(default)]
    pub filters: FilterConfig,

    /// Naming convention configuration
    #[serde(default)]
    pub naming: NamingConfig,

    /// Documentation generation options
    #[serde(default)]
    pub documentation: DocumentationConfig,

    /// Generate stub implementations instead of real FFI calls
    #[serde(default)]
    pub stub_mode: bool,
}

/// Type mapping configuration
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct TypeMappingConfig {
    /// Custom type mappings
    #[serde(default)]
    pub mappings: std::collections::HashMap<String, String>,

    /// Special type handling
    #[serde(default)]
    pub special: std::collections::HashMap<String, String>,
}

/// Filter configuration
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct FilterConfig {
    /// Include specific classes
    #[serde(default)]
    pub include_classes: Vec<String>,

    /// Exclude specific classes
    #[serde(default)]
    pub exclude_classes: Vec<String>,

    /// Include specific methods
    #[serde(default)]
    pub include_methods: Vec<String>,

    /// Exclude specific methods
    #[serde(default)]
    pub exclude_methods: Vec<String>,

    /// Skip generation for specific modules
    #[serde(default)]
    pub skip_modules: Vec<String>,
}

/// Naming convention configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NamingConfig {
    /// Method naming convention
    #[serde(default = "default_method_case")]
    pub method_case: String,

    /// Type naming convention
    #[serde(default = "default_type_case")]
    pub type_case: String,

    /// Module naming convention
    #[serde(default = "default_module_case")]
    pub module_case: String,

    /// Prefixes to remove from method names
    #[serde(default)]
    pub remove_prefix: Vec<String>,

    /// Suffix for actor types
    #[serde(default = "default_actor_suffix")]
    pub actor_suffix: String,
}

/// Documentation generation configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DocumentationConfig {
    /// Include warning sections
    #[serde(default = "default_true")]
    pub include_warnings: bool,

    /// Include note sections
    #[serde(default = "default_true")]
    pub include_notes: bool,

    /// Include Python examples
    #[serde(default)]
    pub include_python_examples: bool,

    /// Generate doc tests
    #[serde(default = "default_true")]
    pub generate_doc_tests: bool,
}

impl Config {
    /// Create config from file
    pub fn from_file<P: AsRef<Path>>(path: P) -> crate::Result<Self> {
        let content = std::fs::read_to_string(path)?;
        let config: Config = toml::from_str(&content)
            .map_err(|e| crate::error::CodegenError::Config(e.to_string()))?;
        Ok(config)
    }

    /// Create a builder for configuration
    pub fn builder() -> ConfigBuilder {
        ConfigBuilder::default()
    }

    /// Add a custom type mapping
    pub fn add_type_mapping(&mut self, python_type: String, rust_type: String) {
        self.type_mapping.mappings.insert(python_type, rust_type);
    }

    /// Set the builder threshold
    pub fn set_builder_threshold(&mut self, threshold: usize) {
        self.builder_threshold = threshold;
    }

    /// Exclude a class from generation
    pub fn exclude_class(&mut self, class_name: String) {
        if !self.filters.exclude_classes.contains(&class_name) {
            self.filters.exclude_classes.push(class_name);
        }
    }

    /// Include only specific classes
    pub fn include_class(&mut self, class_name: String) {
        if !self.filters.include_classes.contains(&class_name) {
            self.filters.include_classes.push(class_name);
        }
    }

    /// Check if a class should be generated
    pub fn should_generate_class(&self, class_name: &str) -> bool {
        // If include list is not empty, class must be in it
        if !self.filters.include_classes.is_empty() {
            return self
                .filters
                .include_classes
                .contains(&class_name.to_string());
        }

        // Otherwise, check it's not in exclude list
        !self
            .filters
            .exclude_classes
            .contains(&class_name.to_string())
    }

    /// Check if a method should be generated
    pub fn should_generate_method(&self, method_name: &str) -> bool {
        // If include list is not empty, method must be in it
        if !self.filters.include_methods.is_empty() {
            return self
                .filters
                .include_methods
                .contains(&method_name.to_string());
        }

        // Otherwise, check it's not in exclude list
        !self
            .filters
            .exclude_methods
            .contains(&method_name.to_string())
    }

    /// Check if a module should be generated
    pub fn should_generate_module(&self, module_name: &str) -> bool {
        !self.filters.skip_modules.contains(&module_name.to_string())
    }

    /// Set stub mode
    pub fn set_stub_mode(&mut self, stub: bool) {
        self.stub_mode = stub;
    }
}

impl Default for Config {
    fn default() -> Self {
        Self {
            output_dir: default_output_dir(),
            yaml_dir: None,
            builder_threshold: default_builder_threshold(),
            generate_async: false,
            module_structure: default_module_structure(),
            type_mapping: TypeMappingConfig::default(),
            filters: FilterConfig::default(),
            naming: NamingConfig::default(),
            documentation: DocumentationConfig::default(),
            stub_mode: false,
        }
    }
}

impl Default for NamingConfig {
    fn default() -> Self {
        Self {
            method_case: default_method_case(),
            type_case: default_type_case(),
            module_case: default_module_case(),
            remove_prefix: vec!["get_".to_string(), "set_".to_string()],
            actor_suffix: default_actor_suffix(),
        }
    }
}

impl Default for DocumentationConfig {
    fn default() -> Self {
        Self {
            include_warnings: default_true(),
            include_notes: default_true(),
            include_python_examples: false,
            generate_doc_tests: default_true(),
        }
    }
}

/// Configuration builder
#[derive(Default)]
pub struct ConfigBuilder {
    config: Config,
}

impl ConfigBuilder {
    /// Set YAML directory
    pub fn yaml_dir<P: Into<PathBuf>>(mut self, dir: P) -> Self {
        self.config.yaml_dir = Some(dir.into());
        self
    }

    /// Set output directory
    pub fn output_dir<P: Into<PathBuf>>(mut self, dir: P) -> Self {
        self.config.output_dir = dir.into();
        self
    }

    /// Set configuration file
    pub fn config_file<P: AsRef<Path>>(self, path: P) -> crate::Result<Self> {
        Ok(Self {
            config: Config::from_file(path)?,
        })
    }

    /// Set builder threshold
    pub fn builder_threshold(mut self, threshold: usize) -> Self {
        self.config.builder_threshold = threshold;
        self
    }

    /// Enable async generation
    pub fn generate_async(mut self, enable: bool) -> Self {
        self.config.generate_async = enable;
        self
    }

    /// Set module structure
    pub fn module_structure(mut self, structure: String) -> Self {
        self.config.module_structure = structure;
        self
    }

    /// Add type mapping
    pub fn add_type_mapping(mut self, python_type: String, rust_type: String) -> Self {
        self.config
            .type_mapping
            .mappings
            .insert(python_type, rust_type);
        self
    }

    /// Exclude class
    pub fn exclude_class(mut self, class_name: String) -> Self {
        self.config.exclude_class(class_name);
        self
    }

    /// Include class
    pub fn include_class(mut self, class_name: String) -> Self {
        self.config.include_class(class_name);
        self
    }

    /// Skip module
    pub fn skip_module(mut self, module_name: String) -> Self {
        if !self.config.filters.skip_modules.contains(&module_name) {
            self.config.filters.skip_modules.push(module_name);
        }
        self
    }

    /// Set stub mode
    pub fn stub_mode(mut self, stub: bool) -> Self {
        self.config.stub_mode = stub;
        self
    }

    /// Build the configuration
    pub fn build(self) -> Config {
        self.config
    }
}

// Default value functions
fn default_output_dir() -> PathBuf {
    PathBuf::from("generated")
}

fn default_builder_threshold() -> usize {
    2
}

fn default_module_structure() -> String {
    "nested".to_string()
}

fn default_method_case() -> String {
    "snake_case".to_string()
}

fn default_type_case() -> String {
    "PascalCase".to_string()
}

fn default_module_case() -> String {
    "snake_case".to_string()
}

fn default_actor_suffix() -> String {
    "Actor".to_string()
}

fn default_true() -> bool {
    true
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default_config() {
        let config = Config::default();
        assert_eq!(config.output_dir, PathBuf::from("generated"));
        assert_eq!(config.builder_threshold, 2);
        assert_eq!(config.module_structure, "nested");
        assert!(!config.generate_async);
    }

    #[test]
    fn test_config_builder() {
        let config = Config::builder()
            .output_dir("out")
            .builder_threshold(3)
            .generate_async(true)
            .add_type_mapping("long".to_string(), "i64".to_string())
            .exclude_class("DeprecatedClass".to_string())
            .build();

        assert_eq!(config.output_dir, PathBuf::from("out"));
        assert_eq!(config.builder_threshold, 3);
        assert!(config.generate_async);
        assert_eq!(
            config.type_mapping.mappings.get("long"),
            Some(&"i64".to_string())
        );
        assert!(config
            .filters
            .exclude_classes
            .contains(&"DeprecatedClass".to_string()));
    }

    #[test]
    fn test_should_generate_class() {
        let mut config = Config::default();

        // By default, all classes should be generated
        assert!(config.should_generate_class("Actor"));

        // Exclude a class
        config.exclude_class("BadActor".to_string());
        assert!(!config.should_generate_class("BadActor"));
        assert!(config.should_generate_class("GoodActor"));

        // Include only specific classes
        config.include_class("Actor".to_string());
        config.include_class("Vehicle".to_string());
        assert!(config.should_generate_class("Actor"));
        assert!(config.should_generate_class("Vehicle"));
        assert!(!config.should_generate_class("Walker"));
    }

    #[test]
    fn test_should_generate_method() {
        let mut config = Config::default();

        // By default, all methods should be generated
        assert!(config.should_generate_method("get_location"));

        // Exclude a method
        config
            .filters
            .exclude_methods
            .push("deprecated_method".to_string());
        assert!(!config.should_generate_method("deprecated_method"));
        assert!(config.should_generate_method("good_method"));
    }

    #[test]
    fn test_should_generate_module() {
        let mut config = Config::default();

        // By default, all modules should be generated
        assert!(config.should_generate_module("actor"));

        // Skip a module
        config.filters.skip_modules.push("osm2odr".to_string());
        assert!(!config.should_generate_module("osm2odr"));
        assert!(config.should_generate_module("world"));
    }
}
