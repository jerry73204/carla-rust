//! YAML parser for CARLA Python API documentation

pub mod validator;
pub mod yaml_schema;

use crate::error::Result;
use std::{fs, path::Path};
use tracing::{debug, info};
use walkdir::WalkDir;

pub use yaml_schema::*;

/// Parser for YAML files
pub struct YamlParser {
    modules: Vec<Module>,
}

impl YamlParser {
    /// Create a new parser
    pub fn new() -> Self {
        Self {
            modules: Vec::new(),
        }
    }

    /// Parse a single YAML file
    pub fn parse_file<P: AsRef<Path>>(&mut self, path: P) -> Result<()> {
        let path = path.as_ref();
        info!("Parsing YAML file: {}", path.display());

        let content = fs::read_to_string(path)?;
        let modules: Vec<Module> = serde_yaml::from_str(&content)?;

        debug!("Found {} modules in {}", modules.len(), path.display());

        // Validate each module
        for module in &modules {
            validator::validate_module(module)?;
        }

        self.modules.extend(modules);
        Ok(())
    }

    /// Parse all YAML files in a directory
    pub fn parse_directory<P: AsRef<Path>>(&mut self, dir: P) -> Result<()> {
        let dir = dir.as_ref();
        info!("Parsing YAML files in directory: {}", dir.display());

        for entry in WalkDir::new(dir).into_iter().filter_map(|e| e.ok()) {
            let path = entry.path();
            if path.is_file() && path.extension().and_then(|s| s.to_str()) == Some("yml") {
                self.parse_file(path)?;
            }
        }

        Ok(())
    }

    /// Get all parsed modules
    pub fn modules(&self) -> &[Module] {
        &self.modules
    }

    /// Merge modules with the same name
    pub fn merge_modules(self) -> Vec<Module> {
        use std::collections::HashMap;

        let mut merged: HashMap<String, Module> = HashMap::new();

        for module in self.modules {
            merged
                .entry(module.module_name.clone())
                .and_modify(|existing| {
                    // Merge classes from modules with the same name
                    existing.classes.extend(module.classes.clone());
                })
                .or_insert(module);
        }

        merged.into_values().collect()
    }
}

impl Default for YamlParser {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs::write;
    use tempfile::TempDir;

    #[test]
    fn test_parse_single_file() {
        let dir = TempDir::new().unwrap();
        let file_path = dir.path().join("test.yml");

        let yaml_content = r#"
- module_name: carla
  classes:
  - class_name: TestClass
    doc: Test documentation
"#;

        write(&file_path, yaml_content).unwrap();

        let mut parser = YamlParser::new();
        parser.parse_file(&file_path).unwrap();

        assert_eq!(parser.modules().len(), 1);
        assert_eq!(parser.modules()[0].module_name, "carla");
    }
}
