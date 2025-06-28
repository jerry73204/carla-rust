//! CARLA Python API to Rust Code Generator
//!
//! This crate provides functionality to parse CARLA's Python API YAML documentation
//! and generate corresponding Rust type definitions.

pub mod analyzer;
pub mod ast;
pub mod config;
pub mod error;
pub mod generator;
pub mod parser;

pub use error::{CodegenError, Result};

use std::path::Path;
use tracing::info;

/// Main code generator struct
pub struct Generator {
    pub config: config::Config,
    parser: parser::YamlParser,
    generator: Option<generator::CodeGenerator>,
}

impl Generator {
    /// Create a new generator with the given configuration
    pub fn new(config: config::Config) -> Self {
        Self {
            config,
            parser: parser::YamlParser::new(),
            generator: None,
        }
    }

    /// Add a YAML directory to parse
    pub fn add_yaml_dir<P: AsRef<Path>>(&mut self, dir: P) -> Result<&mut Self> {
        self.parser.parse_directory(dir)?;
        Ok(self)
    }

    /// Add a single YAML file to parse
    pub fn add_yaml_file<P: AsRef<Path>>(&mut self, file: P) -> Result<&mut Self> {
        self.parser.parse_file(file)?;
        Ok(self)
    }

    /// Set the output directory
    pub fn set_output_dir<P: AsRef<Path>>(&mut self, dir: P) -> Result<&mut Self> {
        self.config.output_dir = dir.as_ref().to_path_buf();
        Ok(self)
    }

    /// Generate Rust code from parsed YAML files
    pub fn generate(&mut self) -> Result<()> {
        info!("Starting code generation");

        // Get parsed modules - we need to clone here since merge_modules consumes self
        let modules = std::mem::take(&mut self.parser).merge_modules();

        if modules.is_empty() {
            return Err(CodegenError::Config("No modules to generate".to_string()));
        }

        // Create code generator if not already created
        if self.generator.is_none() {
            self.generator = Some(generator::CodeGenerator::new(self.config.clone())?);
        }

        // Generate code
        let generator = self.generator.as_mut().unwrap();
        generator.generate(&modules, &self.config.output_dir)?;

        info!("Code generation completed");
        Ok(())
    }

    /// Generate code to string for testing
    #[cfg(test)]
    pub fn generate_to_string(&mut self) -> Result<String> {
        // For now, generate to a temporary directory and read back
        let temp_dir = tempfile::tempdir()?;
        self.set_output_dir(temp_dir.path())?;
        self.generate()?;

        // Read generated files
        let mut content = String::new();
        for entry in std::fs::read_dir(temp_dir.path())? {
            let entry = entry?;
            if entry.path().extension().and_then(|s| s.to_str()) == Some("rs") {
                content.push_str(&std::fs::read_to_string(entry.path())?);
                content.push('\n');
            }
        }

        Ok(content)
    }
}

/// Generate code with default configuration
pub fn generate_default() -> Result<()> {
    let config = config::Config::default();
    let mut generator = Generator::new(config);
    generator.generate()
}
