//! Error types for the carla-codegen crate

use std::path::PathBuf;
use thiserror::Error;

pub use crate::error_context::{CodeContext, ErrorCollector, ErrorConfig, SourceContext, Stage};

/// Main error type for carla-codegen
#[derive(Debug, Error, Clone)]
pub enum CodegenError {
    /// YAML parsing error with context
    #[error("YAML parsing failed: {message}")]
    YamlParse {
        message: String,
        file: PathBuf,
        yaml_error: Option<String>, // Store error as String for Clone
    },

    /// File I/O error
    #[error("File I/O error: {0}")]
    Io(String), // Store as String for Clone

    /// Type not found in mappings with context
    #[error("Type '{python_type}' not found in type mappings")]
    UnknownType {
        python_type: String,
        context: Box<SourceContext>, // Box large context
    },

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

    /// AST generation error with context
    #[error("Code generation failed: {message}")]
    GenerationError {
        message: String,
        context: Box<SourceContext>, // Box large context
        stage: Stage,
    },

    /// Syn parsing error with context
    #[error("Syn parsing error: {message}")]
    SynError {
        message: String,
        context: Box<SourceContext>, // Box large context
        generated_code: Option<Box<CodeContext>>, // Box large context
        syn_error: Option<String>, // Store error as String for Clone
    },

    /// Formatter error
    #[error("Code formatting error: {0}")]
    FormatterError(String),
}

/// Result type alias for carla-codegen
pub type Result<T> = std::result::Result<T, CodegenError>;

// Helper implementations for CodegenError
impl CodegenError {
    /// Create an UnknownType error with context
    pub fn unknown_type(python_type: String, context: SourceContext) -> Self {
        Self::UnknownType {
            python_type,
            context: Box::new(context),
        }
    }

    /// Create a SynError with context
    pub fn syn_error(
        syn_error: syn::Error,
        context: SourceContext,
        generated_code: Option<CodeContext>,
    ) -> Self {
        Self::SynError {
            message: syn_error.to_string(),
            context: Box::new(context),
            generated_code: generated_code.map(Box::new),
            syn_error: Some(syn_error.to_string()),
        }
    }

    /// Create a GenerationError with context
    pub fn generation_error(message: String, context: SourceContext, stage: Stage) -> Self {
        Self::GenerationError {
            message,
            context: Box::new(context),
            stage,
        }
    }

    /// Create a YamlParse error
    pub fn yaml_parse(error: serde_yaml::Error, file: PathBuf) -> Self {
        Self::YamlParse {
            message: error.to_string(),
            file,
            yaml_error: Some(error.to_string()),
        }
    }

    /// Format the error with full context
    pub fn format_with_context(&self, config: &ErrorConfig) -> String {
        use std::fmt::Write;
        let mut output = String::new();

        match self {
            Self::UnknownType {
                python_type,
                context,
            } => {
                writeln!(
                    &mut output,
                    "Error: Type resolution failed for '{python_type}'"
                )
                .unwrap();
                if config.show_source_context {
                    writeln!(&mut output, "  Source: {context}").unwrap();
                    writeln!(&mut output, "  Stage: Type resolution").unwrap();
                    writeln!(&mut output).unwrap();
                    writeln!(
                        &mut output,
                        "  Help: Add a type mapping for '{python_type}' in your configuration:"
                    )
                    .unwrap();
                    writeln!(&mut output, "        [type_mapping.mappings]").unwrap();
                    writeln!(
                        &mut output,
                        "        \"{python_type}\" = \"Box<dyn Fn() + Send + Sync>\""
                    )
                    .unwrap();
                }
            }
            Self::SynError {
                message,
                context,
                generated_code,
                ..
            } => {
                writeln!(&mut output, "Error: Syn parsing error: {message}").unwrap();
                if config.show_source_context {
                    writeln!(&mut output, "  Source: {context}").unwrap();
                    writeln!(&mut output, "  Stage: AST generation").unwrap();
                }
                if config.show_generated_code {
                    if let Some(code_ctx) = generated_code {
                        writeln!(&mut output).unwrap();
                        writeln!(&mut output, "  Generated code that failed:").unwrap();
                        for line in code_ctx.code_snippet.lines() {
                            writeln!(&mut output, "  {line}").unwrap();
                        }
                        writeln!(&mut output).unwrap();
                        writeln!(
                            &mut output,
                            "  Target file: {}",
                            code_ctx.target_file.display()
                        )
                        .unwrap();
                        writeln!(&mut output, "  Context: {}", code_ctx.construct).unwrap();
                    }
                }
            }
            Self::GenerationError {
                message,
                context,
                stage,
            } => {
                writeln!(&mut output, "Error: {message}").unwrap();
                if config.show_source_context {
                    writeln!(&mut output, "  Source: {context}").unwrap();
                    writeln!(&mut output, "  Stage: {stage}").unwrap();
                }
            }
            Self::YamlParse { message, file, .. } => {
                writeln!(&mut output, "Error: YAML parsing failed: {message}").unwrap();
                writeln!(&mut output, "  File: {}", file.display()).unwrap();
            }
            _ => {
                // Default formatting for other error types
                writeln!(&mut output, "Error: {self}").unwrap();
            }
        }

        output
    }
}

// Implement From conversions for standard errors
impl From<std::io::Error> for CodegenError {
    fn from(err: std::io::Error) -> Self {
        CodegenError::Io(err.to_string())
    }
}

impl From<serde_yaml::Error> for CodegenError {
    fn from(err: serde_yaml::Error) -> Self {
        CodegenError::YamlParse {
            message: err.to_string(),
            file: PathBuf::new(), // Will be set by caller
            yaml_error: Some(err.to_string()),
        }
    }
}

// Implement From conversions for AST errors
impl From<crate::ast::AstError> for CodegenError {
    fn from(err: crate::ast::AstError) -> Self {
        // For now, convert to simple string error
        // TODO: Enhance AstError to include context
        CodegenError::GenerationError {
            message: err.to_string(),
            context: Box::new(SourceContext::new_class(PathBuf::new(), String::new())),
            stage: Stage::AstGeneration,
        }
    }
}

impl From<crate::ast::formatter::FormatterError> for CodegenError {
    fn from(err: crate::ast::formatter::FormatterError) -> Self {
        CodegenError::FormatterError(err.to_string())
    }
}

impl From<syn::Error> for CodegenError {
    fn from(err: syn::Error) -> Self {
        // Create a minimal context for syn errors
        let context = SourceContext::new_class(PathBuf::new(), String::new());
        CodegenError::syn_error(err, context, None)
    }
}
