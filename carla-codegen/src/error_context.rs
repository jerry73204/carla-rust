//! Enhanced error types with source context tracking

use std::{fmt, path::PathBuf};

/// Source location context for errors
#[derive(Debug, Clone)]
pub struct SourceContext {
    /// YAML file path
    pub yaml_file: PathBuf,
    /// Class name being processed
    pub class_name: String,
    /// Method name if applicable
    pub method_name: Option<String>,
    /// Parameter name if applicable
    pub param_name: Option<String>,
    /// Python type that caused the issue
    pub python_type: Option<String>,
}

impl SourceContext {
    /// Create a new source context for a class
    pub fn new_class(yaml_file: PathBuf, class_name: String) -> Self {
        Self {
            yaml_file,
            class_name,
            method_name: None,
            param_name: None,
            python_type: None,
        }
    }

    /// Add method context
    pub fn with_method(mut self, method_name: String) -> Self {
        self.method_name = Some(method_name);
        self
    }

    /// Add parameter context
    pub fn with_param(mut self, param_name: String) -> Self {
        self.param_name = Some(param_name);
        self
    }

    /// Add Python type context
    pub fn with_python_type(mut self, python_type: String) -> Self {
        self.python_type = Some(python_type);
        self
    }
}

impl fmt::Display for SourceContext {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{} -> {}", self.yaml_file.display(), self.class_name)?;

        if let Some(method) = &self.method_name {
            write!(f, ".{method}")?;
        }

        if let Some(param) = &self.param_name {
            write!(f, " parameter '{param}'")?;
        }

        if let Some(py_type) = &self.python_type {
            write!(f, " (type: {py_type})")?;
        }

        Ok(())
    }
}

/// Generated code context for errors
#[derive(Debug, Clone)]
pub struct CodeContext {
    /// The generated Rust code snippet that failed
    pub code_snippet: String,
    /// Target file path
    pub target_file: PathBuf,
    /// What was being generated
    pub construct: String,
}

impl CodeContext {
    /// Create a new code context
    pub fn new(code_snippet: String, target_file: PathBuf, construct: String) -> Self {
        Self {
            code_snippet,
            target_file,
            construct,
        }
    }
}

/// Pipeline processing stage
#[derive(Debug, Clone, Copy)]
pub enum Stage {
    YamlParsing,
    TypeResolution,
    AstGeneration,
    CodeFormatting,
    FileOutput,
}

impl fmt::Display for Stage {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Stage::YamlParsing => write!(f, "YAML parsing"),
            Stage::TypeResolution => write!(f, "Type resolution"),
            Stage::AstGeneration => write!(f, "AST generation"),
            Stage::CodeFormatting => write!(f, "Code formatting"),
            Stage::FileOutput => write!(f, "File output"),
        }
    }
}

/// Error configuration for customizing error reporting
#[derive(Debug, Clone, Copy)]
pub struct ErrorConfig {
    /// Show generated code context in errors (default: true)
    pub show_generated_code: bool,
    /// Show YAML source context (default: true)
    pub show_source_context: bool,
    /// Continue processing after errors (default: false)
    pub continue_on_error: bool,
    /// Maximum number of errors before stopping (default: 10)
    pub max_errors: usize,
    /// Use colored output (default: auto-detect TTY)
    pub use_colors: bool,
}

impl Default for ErrorConfig {
    fn default() -> Self {
        Self {
            show_generated_code: true,
            show_source_context: true,
            continue_on_error: false,
            max_errors: 10,
            use_colors: atty::is(atty::Stream::Stderr),
        }
    }
}

/// Error collector for batch processing
#[derive(Debug, Default)]
pub struct ErrorCollector {
    errors: Vec<crate::error::CodegenError>,
    config: ErrorConfig,
}

impl ErrorCollector {
    /// Create a new error collector with default config
    pub fn new() -> Self {
        Self::default()
    }

    /// Create a new error collector with custom config
    pub fn with_config(config: ErrorConfig) -> Self {
        Self {
            errors: Vec::new(),
            config,
        }
    }

    /// Add an error to the collector
    pub fn add_error(
        &mut self,
        error: crate::error::CodegenError,
    ) -> Result<(), crate::error::CodegenError> {
        self.errors.push(error.clone());

        if !self.config.continue_on_error || self.errors.len() >= self.config.max_errors {
            Err(error)
        } else {
            Ok(())
        }
    }

    /// Check if there are any errors
    pub fn has_errors(&self) -> bool {
        !self.errors.is_empty()
    }

    /// Get the number of errors
    pub fn error_count(&self) -> usize {
        self.errors.len()
    }

    /// Get all collected errors
    pub fn errors(&self) -> &[crate::error::CodegenError] {
        &self.errors
    }

    /// Take all errors, consuming the collector
    pub fn take_errors(self) -> Vec<crate::error::CodegenError> {
        self.errors
    }
}
