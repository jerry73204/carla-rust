//! Error types for the carla-codegen crate

use thiserror::Error;

/// Main error type for carla-codegen
#[derive(Debug, Error)]
pub enum CodegenError {
    /// YAML parsing error
    #[error("Failed to parse YAML: {0}")]
    YamlParse(#[from] serde_yaml::Error),

    /// File I/O error
    #[error("File I/O error: {0}")]
    Io(#[from] std::io::Error),

    /// Type not found in mappings
    #[error("Type '{0}' not found in type mappings")]
    UnknownType(String),

    /// Circular dependency detected
    #[error("Circular dependency detected: {0}")]
    CircularDependency(String),

    /// Invalid configuration
    #[error("Invalid configuration: {0}")]
    Config(String),

    /// Invalid YAML structure
    #[error("Invalid YAML structure: {0}")]
    InvalidStructure(String),

    /// Type conversion error
    #[error("Type conversion error: {0}")]
    TypeConversion(String),

    /// AST generation error
    #[error("AST generation error: {0}")]
    AstError(String),

    /// Syn parsing error
    #[error("Syn parsing error: {0}")]
    SynError(#[from] syn::Error),

    /// Formatter error
    #[error("Code formatting error: {0}")]
    FormatterError(String),
}

/// Result type alias for carla-codegen
pub type Result<T> = std::result::Result<T, CodegenError>;

// Implement From conversions for AST errors
impl From<crate::ast::AstError> for CodegenError {
    fn from(err: crate::ast::AstError) -> Self {
        CodegenError::AstError(err.to_string())
    }
}

impl From<crate::ast::formatter::FormatterError> for CodegenError {
    fn from(err: crate::ast::formatter::FormatterError) -> Self {
        CodegenError::FormatterError(err.to_string())
    }
}
