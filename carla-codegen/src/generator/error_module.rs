//! Error module generation for carla-codegen

use crate::error::Result;
use std::path::Path;

/// Generates the error module for the output
pub struct ErrorModuleGenerator {
    /// Whether to use thiserror crate
    use_thiserror: bool,
    /// The error type name
    error_type_name: String,
    /// Target crate type (carla-sys or carla)
    target_crate: TargetCrate,
}

/// Target crate type for error generation
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TargetCrate {
    /// Minimal errors for carla-sys FFI layer
    CarlaSys,
    /// Rich errors for carla high-level API
    Carla,
    /// Generic for testing
    Generic,
}

impl ErrorModuleGenerator {
    /// Create a new error module generator
    pub fn new(target_crate: TargetCrate) -> Self {
        Self {
            use_thiserror: true,
            error_type_name: "CarlaError".to_string(),
            target_crate,
        }
    }

    /// Set whether to use thiserror
    pub fn with_thiserror(mut self, use_thiserror: bool) -> Self {
        self.use_thiserror = use_thiserror;
        self
    }

    /// Set the error type name
    pub fn with_error_type_name(mut self, name: String) -> Self {
        self.error_type_name = name;
        self
    }

    /// Generate the error module content
    pub fn generate(&self) -> String {
        let mut content = String::new();

        // Header
        content.push_str("//! Error types for CARLA bindings\n");
        content.push_str("//!\n");
        content
            .push_str("//! This module provides error types used throughout the generated code.\n");
        content.push_str(
            "//! All fallible operations return `Result<T>` using the error type defined here.\n\n",
        );

        // Imports
        if self.use_thiserror {
            content.push_str("use thiserror::Error;\n\n");
        } else {
            content.push_str("use std::error::Error;\n");
            content.push_str("use std::fmt;\n\n");
        }

        // Error enum
        content.push_str("/// Main error type for CARLA operations\n");
        if self.use_thiserror {
            content.push_str("#[derive(Debug, Error, Clone)]\n");
        } else {
            content.push_str("#[derive(Debug, Clone)]\n");
        }
        content.push_str(&format!("pub enum {} {{\n", self.error_type_name));

        // Add variants based on target crate
        match self.target_crate {
            TargetCrate::CarlaSys => {
                self.add_minimal_variants(&mut content);
            }
            TargetCrate::Carla => {
                self.add_rich_variants(&mut content);
            }
            TargetCrate::Generic => {
                self.add_generic_variants(&mut content);
            }
        }

        content.push_str("}\n\n");

        // Manual Error implementation if not using thiserror
        if !self.use_thiserror {
            self.add_manual_error_impl(&mut content);
        }

        // Result type alias
        content.push_str("/// Result type alias for CARLA operations\n");
        content.push_str(&format!(
            "pub type Result<T> = std::result::Result<T, {}>;\n",
            self.error_type_name
        ));

        // Helper methods
        content.push_str(&format!("\nimpl {} {{\n", self.error_type_name));
        content.push_str("    /// Create a not implemented error\n");
        content.push_str("    pub fn not_implemented(feature: impl Into<String>) -> Self {\n");
        content.push_str("        Self::NotImplemented(feature.into())\n");
        content.push_str("    }\n");
        content.push_str("}\n");

        content
    }

    /// Add minimal error variants for carla-sys
    fn add_minimal_variants(&self, content: &mut String) {
        if self.use_thiserror {
            content.push_str("    /// Feature not yet implemented\n");
            content.push_str("    #[error(\"Not implemented: {0}\")]\n");
            content.push_str("    NotImplemented(String),\n\n");

            content.push_str("    /// FFI error from C++ layer\n");
            content.push_str("    #[error(\"FFI error: {0}\")]\n");
            content.push_str("    FfiError(String),\n\n");

            content.push_str("    /// Invalid parameter provided\n");
            content.push_str("    #[error(\"Invalid parameter: {0}\")]\n");
            content.push_str("    InvalidParameter(String),\n");
        } else {
            content.push_str("    /// Feature not yet implemented\n");
            content.push_str("    NotImplemented(String),\n");
            content.push_str("    /// FFI error from C++ layer\n");
            content.push_str("    FfiError(String),\n");
            content.push_str("    /// Invalid parameter provided\n");
            content.push_str("    InvalidParameter(String),\n");
        }
    }

    /// Add rich error variants for carla crate
    fn add_rich_variants(&self, content: &mut String) {
        self.add_minimal_variants(content);

        if self.use_thiserror {
            content.push_str("\n    /// Connection to server failed\n");
            content.push_str("    #[error(\"Connection failed: {0}\")]\n");
            content.push_str("    ConnectionFailed(String),\n\n");

            content.push_str("    /// Actor not found\n");
            content.push_str("    #[error(\"Actor not found with ID: {0}\")]\n");
            content.push_str("    ActorNotFound(u32),\n\n");

            content.push_str("    /// Invalid blueprint\n");
            content.push_str("    #[error(\"Invalid blueprint: {0}\")]\n");
            content.push_str("    InvalidBlueprint(String),\n\n");

            content.push_str("    /// Simulation error\n");
            content.push_str("    #[error(\"Simulation error: {0}\")]\n");
            content.push_str("    SimulationError(String),\n");
        } else {
            content.push_str("    /// Connection to server failed\n");
            content.push_str("    ConnectionFailed(String),\n");
            content.push_str("    /// Actor not found\n");
            content.push_str("    ActorNotFound(u32),\n");
            content.push_str("    /// Invalid blueprint\n");
            content.push_str("    InvalidBlueprint(String),\n");
            content.push_str("    /// Simulation error\n");
            content.push_str("    SimulationError(String),\n");
        }
    }

    /// Add generic error variants for testing
    fn add_generic_variants(&self, content: &mut String) {
        if self.use_thiserror {
            content.push_str("    /// Feature not yet implemented\n");
            content.push_str("    #[error(\"Not implemented: {0}\")]\n");
            content.push_str("    NotImplemented(String),\n\n");

            content.push_str("    /// Generic error\n");
            content.push_str("    #[error(\"Error: {0}\")]\n");
            content.push_str("    Generic(String),\n");
        } else {
            content.push_str("    /// Feature not yet implemented\n");
            content.push_str("    NotImplemented(String),\n");
            content.push_str("    /// Generic error\n");
            content.push_str("    Generic(String),\n");
        }
    }

    /// Add manual Error trait implementation
    fn add_manual_error_impl(&self, content: &mut String) {
        content.push_str(&format!(
            "impl fmt::Display for {} {{\n",
            self.error_type_name
        ));
        content.push_str("    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {\n");
        content.push_str("        match self {\n");
        content.push_str(
            "            Self::NotImplemented(msg) => write!(f, \"Not implemented: {}\", msg),\n",
        );

        if self.target_crate == TargetCrate::CarlaSys {
            content.push_str(
                "            Self::FfiError(msg) => write!(f, \"FFI error: {}\", msg),\n",
            );
            content.push_str("            Self::InvalidParameter(msg) => write!(f, \"Invalid parameter: {}\", msg),\n");
        } else if self.target_crate == TargetCrate::Carla {
            content.push_str(
                "            Self::FfiError(msg) => write!(f, \"FFI error: {}\", msg),\n",
            );
            content.push_str("            Self::InvalidParameter(msg) => write!(f, \"Invalid parameter: {}\", msg),\n");
            content.push_str("            Self::ConnectionFailed(msg) => write!(f, \"Connection failed: {}\", msg),\n");
            content.push_str("            Self::ActorNotFound(id) => write!(f, \"Actor not found with ID: {}\", id),\n");
            content.push_str("            Self::InvalidBlueprint(msg) => write!(f, \"Invalid blueprint: {}\", msg),\n");
            content.push_str("            Self::SimulationError(msg) => write!(f, \"Simulation error: {}\", msg),\n");
        } else if self.target_crate == TargetCrate::Generic {
            content.push_str("            Self::Generic(msg) => write!(f, \"Error: {}\", msg),\n");
        }

        content.push_str("        }\n");
        content.push_str("    }\n");
        content.push_str("}\n\n");

        content.push_str(&format!("impl Error for {} {{}}\n\n", self.error_type_name));
    }

    /// Generate the error module file
    pub fn generate_file(&self, output_dir: &Path) -> Result<()> {
        let error_file = output_dir.join("error.rs");
        let content = self.generate();
        std::fs::write(error_file, content)?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_minimal_error_generation() {
        let generator = ErrorModuleGenerator::new(TargetCrate::CarlaSys);
        let content = generator.generate();

        assert!(content.contains("pub enum CarlaError"));
        assert!(content.contains("NotImplemented(String)"));
        assert!(content.contains("FfiError(String)"));
        assert!(content.contains("pub type Result<T>"));
    }

    #[test]
    fn test_rich_error_generation() {
        let generator = ErrorModuleGenerator::new(TargetCrate::Carla);
        let content = generator.generate();

        assert!(content.contains("ConnectionFailed(String)"));
        assert!(content.contains("ActorNotFound(u32)"));
        assert!(content.contains("SimulationError(String)"));
    }

    #[test]
    fn test_manual_error_impl() {
        let generator = ErrorModuleGenerator::new(TargetCrate::Generic).with_thiserror(false);
        let content = generator.generate();

        assert!(content.contains("impl fmt::Display"));
        assert!(content.contains("impl Error for CarlaError"));
    }
}
