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

    /// Generate FFI code for carla-sys
    #[serde(default)]
    pub generate_ffi: bool,

    /// Code formatting configuration
    #[serde(default)]
    pub formatting: FormattingConfig,

    /// Method signature configuration
    #[serde(default)]
    pub method_signatures: MethodSignatureConfig,

    /// Argument handling configuration
    #[serde(default)]
    pub argument_handling: ArgumentHandlingConfig,

    /// Per-class configuration overrides
    #[serde(default)]
    pub class_overrides: std::collections::HashMap<String, ClassConfig>,

    /// Per-method configuration overrides
    #[serde(default)]
    pub method_overrides: std::collections::HashMap<String, MethodConfig>,
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

/// Code formatting configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FormattingConfig {
    /// Enable rustfmt formatting
    #[serde(default = "default_true")]
    pub enable_rustfmt: bool,

    /// Rustfmt configuration file path
    #[serde(default)]
    pub rustfmt_config: Option<PathBuf>,

    /// Line width for formatting
    #[serde(default = "default_line_width")]
    pub line_width: usize,

    /// Tab width for formatting
    #[serde(default = "default_tab_width")]
    pub tab_width: usize,

    /// Use tabs instead of spaces
    #[serde(default)]
    pub use_tabs: bool,

    /// Force trailing commas
    #[serde(default)]
    pub force_trailing_commas: bool,
}

/// Method signature configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MethodSignatureConfig {
    /// Default self parameter type (self, ref, mut_ref)
    #[serde(default = "default_self_type")]
    pub default_self_type: String,

    /// Use auto-detection for self parameter types
    #[serde(default = "default_true")]
    pub auto_detect_self_type: bool,

    /// Methods that should always take ownership (self)
    #[serde(default)]
    pub owned_self_patterns: Vec<String>,

    /// Methods that should always take mutable reference (&mut self)
    #[serde(default)]
    pub mut_self_patterns: Vec<String>,

    /// Methods that should always take immutable reference (&self)
    #[serde(default)]
    pub ref_self_patterns: Vec<String>,

    /// Generate async versions of methods
    #[serde(default)]
    pub generate_async_variants: bool,

    /// Generate Result<T> return types
    #[serde(default = "default_true")]
    pub use_result_return_types: bool,

    /// Error type for Result<T, E>
    #[serde(default = "default_error_type")]
    pub error_type: String,
}

/// Argument handling configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ArgumentHandlingConfig {
    /// How to handle optional arguments (option, builder, overload)
    #[serde(default = "default_optional_args_style")]
    pub optional_args_style: String,

    /// How to handle **kwargs (builder, map, ignore)
    #[serde(default = "default_kwargs_style")]
    pub kwargs_style: String,

    /// Generate builder patterns for methods with many parameters
    #[serde(default = "default_true")]
    pub enable_builder_patterns: bool,

    /// Minimum number of optional parameters to trigger builder pattern
    #[serde(default = "default_builder_threshold")]
    pub builder_threshold: usize,

    /// Generate overloaded methods for optional parameters
    #[serde(default)]
    pub generate_overloads: bool,

    /// Maximum number of overloads to generate
    #[serde(default = "default_max_overloads")]
    pub max_overloads: usize,

    /// Default values for parameters
    #[serde(default)]
    pub default_values: std::collections::HashMap<String, String>,
}

/// Per-class configuration
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ClassConfig {
    /// Override method signature settings for this class
    #[serde(default)]
    pub method_signatures: Option<MethodSignatureConfig>,

    /// Override argument handling for this class
    #[serde(default)]
    pub argument_handling: Option<ArgumentHandlingConfig>,

    /// Custom derive attributes for the struct
    #[serde(default)]
    pub custom_derives: Vec<String>,

    /// Additional traits to implement
    #[serde(default)]
    pub implement_traits: Vec<String>,

    /// Whether to generate Clone impl
    #[serde(default)]
    pub generate_clone: Option<bool>,

    /// Whether to generate Debug impl
    #[serde(default)]
    pub generate_debug: Option<bool>,

    /// Custom documentation for the class
    #[serde(default)]
    pub custom_doc: Option<String>,
}

/// Per-method configuration
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct MethodConfig {
    /// Override self parameter type for this method
    #[serde(default)]
    pub self_type: Option<String>,

    /// Override return type for this method
    #[serde(default)]
    pub return_type: Option<String>,

    /// Override argument handling for this method
    #[serde(default)]
    pub argument_handling: Option<ArgumentHandlingConfig>,

    /// Generate async version of this method
    #[serde(default)]
    pub generate_async: Option<bool>,

    /// Skip generation of this method
    #[serde(default)]
    pub skip: bool,

    /// Custom implementation body
    #[serde(default)]
    pub custom_impl: Option<String>,

    /// Custom documentation for the method
    #[serde(default)]
    pub custom_doc: Option<String>,

    /// Parameter overrides
    #[serde(default)]
    pub parameter_overrides: std::collections::HashMap<String, ParameterConfig>,
}

/// Per-parameter configuration
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ParameterConfig {
    /// Override parameter type
    #[serde(default)]
    pub param_type: Option<String>,

    /// Override parameter name
    #[serde(default)]
    pub param_name: Option<String>,

    /// Override default value
    #[serde(default)]
    pub default_value: Option<String>,

    /// Make parameter optional
    #[serde(default)]
    pub optional: Option<bool>,

    /// Custom documentation for the parameter
    #[serde(default)]
    pub custom_doc: Option<String>,
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

    /// Get method signature configuration for a specific class/method
    pub fn get_method_signature_config(
        &self,
        class_name: &str,
        _method_name: &str,
    ) -> &MethodSignatureConfig {
        // Check for class-specific override
        if let Some(class_config) = self.class_overrides.get(class_name) {
            if let Some(ref method_sigs) = class_config.method_signatures {
                return method_sigs;
            }
        }

        // Use global configuration
        &self.method_signatures
    }

    /// Get argument handling configuration for a specific class/method
    pub fn get_argument_handling_config(
        &self,
        class_name: &str,
        method_name: &str,
    ) -> &ArgumentHandlingConfig {
        // Check for method-specific override first
        let method_key = format!("{class_name}::{method_name}");
        if let Some(method_config) = self.method_overrides.get(&method_key) {
            if let Some(ref arg_handling) = method_config.argument_handling {
                return arg_handling;
            }
        }

        // Check for class-specific override
        if let Some(class_config) = self.class_overrides.get(class_name) {
            if let Some(ref arg_handling) = class_config.argument_handling {
                return arg_handling;
            }
        }

        // Use global configuration
        &self.argument_handling
    }

    /// Check if a method should be skipped
    pub fn should_skip_method(&self, class_name: &str, method_name: &str) -> bool {
        let method_key = format!("{class_name}::{method_name}");
        if let Some(method_config) = self.method_overrides.get(&method_key) {
            return method_config.skip;
        }
        false
    }

    /// Get custom implementation for a method
    pub fn get_custom_implementation(
        &self,
        class_name: &str,
        method_name: &str,
    ) -> Option<&String> {
        let method_key = format!("{class_name}::{method_name}");
        self.method_overrides
            .get(&method_key)
            .and_then(|config| config.custom_impl.as_ref())
    }

    /// Get self type override for a method
    pub fn get_self_type_override(&self, class_name: &str, method_name: &str) -> Option<&String> {
        let method_key = format!("{class_name}::{method_name}");
        self.method_overrides
            .get(&method_key)
            .and_then(|config| config.self_type.as_ref())
    }

    /// Get return type override for a method
    pub fn get_return_type_override(&self, class_name: &str, method_name: &str) -> Option<&String> {
        let method_key = format!("{class_name}::{method_name}");
        self.method_overrides
            .get(&method_key)
            .and_then(|config| config.return_type.as_ref())
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
            generate_ffi: false,
            formatting: FormattingConfig::default(),
            method_signatures: MethodSignatureConfig::default(),
            argument_handling: ArgumentHandlingConfig::default(),
            class_overrides: std::collections::HashMap::new(),
            method_overrides: std::collections::HashMap::new(),
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

impl Default for FormattingConfig {
    fn default() -> Self {
        Self {
            enable_rustfmt: default_true(),
            rustfmt_config: None,
            line_width: default_line_width(),
            tab_width: default_tab_width(),
            use_tabs: false,
            force_trailing_commas: false,
        }
    }
}

impl Default for MethodSignatureConfig {
    fn default() -> Self {
        Self {
            default_self_type: default_self_type(),
            auto_detect_self_type: default_true(),
            owned_self_patterns: vec![
                "destroy".to_string(),
                "delete".to_string(),
                "consume".to_string(),
                "take".to_string(),
            ],
            mut_self_patterns: vec![
                "set_*".to_string(),
                "add_*".to_string(),
                "remove_*".to_string(),
                "clear_*".to_string(),
                "reset_*".to_string(),
                "apply_*".to_string(),
                "enable_*".to_string(),
                "disable_*".to_string(),
            ],
            ref_self_patterns: vec![
                "get_*".to_string(),
                "is_*".to_string(),
                "has_*".to_string(),
                "list_*".to_string(),
            ],
            generate_async_variants: false,
            use_result_return_types: default_true(),
            error_type: default_error_type(),
        }
    }
}

impl Default for ArgumentHandlingConfig {
    fn default() -> Self {
        Self {
            optional_args_style: default_optional_args_style(),
            kwargs_style: default_kwargs_style(),
            enable_builder_patterns: default_true(),
            builder_threshold: default_builder_threshold(),
            generate_overloads: false,
            max_overloads: default_max_overloads(),
            default_values: std::collections::HashMap::new(),
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

fn default_line_width() -> usize {
    100
}

fn default_tab_width() -> usize {
    4
}

fn default_self_type() -> String {
    "ref".to_string()
}

fn default_error_type() -> String {
    "Result<()>".to_string()
}

fn default_optional_args_style() -> String {
    "option".to_string()
}

fn default_kwargs_style() -> String {
    "builder".to_string()
}

fn default_max_overloads() -> usize {
    5
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
