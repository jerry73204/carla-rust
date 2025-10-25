//! Improved import collection from AST

use crate::analyzer::{ResolvedType, RustType};
use std::collections::{BTreeSet, HashMap};

/// Import information
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub struct Import {
    /// The import statement
    pub statement: String,
    /// Import category for grouping
    pub category: ImportCategory,
}

/// Categories for import grouping
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub enum ImportCategory {
    /// Standard library imports
    Std,
    /// External crate imports
    External,
    /// Internal crate imports
    Internal,
    /// Re-exports
    Reexport,
}

/// Collects and organizes imports
pub struct ImportCollector {
    /// Collected imports
    imports: BTreeSet<Import>,
    /// Import aliases
    aliases: HashMap<String, String>,
}

impl ImportCollector {
    /// Create a new import collector
    pub fn new() -> Self {
        Self {
            imports: BTreeSet::new(),
            aliases: HashMap::new(),
        }
    }

    /// Add imports from a resolved type
    pub fn add_from_resolved(&mut self, resolved: &ResolvedType) {
        // Add imports from the resolved type metadata
        for import in &resolved.imports_needed {
            self.add_import(import);
        }

        // Also collect from the RustType itself
        self.add_from_rust_type(&resolved.rust_type);
    }

    /// Add imports from a RustType
    pub fn add_from_rust_type(&mut self, rust_type: &RustType) {
        match rust_type {
            RustType::Custom(type_name) => {
                self.add_imports_for_custom_type(type_name);
            }
            RustType::Vec(inner) | RustType::Option(inner) => {
                self.add_from_rust_type(inner);
            }
            RustType::HashMap(key, value) => {
                eprintln!("DEBUG: Adding HashMap import for RustType::HashMap");
                self.add_import("use std::collections::HashMap;");
                self.add_from_rust_type(key);
                self.add_from_rust_type(value);
            }
            RustType::Union(types) | RustType::Tuple(types) => {
                for t in types {
                    self.add_from_rust_type(t);
                }
            }
            _ => {}
        }
    }

    /// Add imports for a custom type string
    fn add_imports_for_custom_type(&mut self, type_name: &str) {
        // Extract base type from generics
        let base_type = if let Some(pos) = type_name.find('<') {
            &type_name[..pos]
        } else {
            type_name
        };

        match base_type {
            "HashMap" => self.add_import("use std::collections::HashMap;"),
            "HashSet" => self.add_import("use std::collections::HashSet;"),
            "BTreeMap" => self.add_import("use std::collections::BTreeMap;"),
            "BTreeSet" => self.add_import("use std::collections::BTreeSet;"),
            "Arc" => self.add_import("use std::sync::Arc;"),
            "Mutex" => self.add_import("use std::sync::Mutex;"),
            "RwLock" => self.add_import("use std::sync::RwLock;"),
            "Rc" => self.add_import("use std::rc::Rc;"),
            "RefCell" => self.add_import("use std::cell::RefCell;"),
            "serde_json::Value" => self.add_import("use serde_json;"),
            _ => {
                // Check for nested generics
                if type_name.contains('<') {
                    self.parse_and_add_generic_imports(type_name);
                }
            }
        }
    }

    /// Parse generic type and add imports
    fn parse_and_add_generic_imports(&mut self, type_str: &str) {
        // Simple parser for nested generics
        let mut depth = 0;
        let mut current = String::new();

        for ch in type_str.chars() {
            match ch {
                '<' => {
                    if depth == 0 && !current.is_empty() {
                        self.add_imports_for_custom_type(&current);
                        current.clear();
                    }
                    depth += 1;
                }
                '>' => {
                    depth -= 1;
                    if depth == 0 && !current.is_empty() {
                        self.add_imports_for_custom_type(&current);
                        current.clear();
                    }
                }
                ',' if depth == 1 => {
                    if !current.trim().is_empty() {
                        self.add_imports_for_custom_type(current.trim());
                    }
                    current.clear();
                }
                ' ' if current.is_empty() => {
                    // Skip leading spaces
                }
                _ => {
                    current.push(ch);
                }
            }
        }

        if !current.trim().is_empty() {
            self.add_imports_for_custom_type(current.trim());
        }
    }

    /// Add a single import
    pub fn add_import(&mut self, import_stmt: &str) {
        let category = self.categorize_import(import_stmt);
        self.imports.insert(Import {
            statement: import_stmt.to_string(),
            category,
        });
    }

    /// Categorize an import statement
    fn categorize_import(&self, import_stmt: &str) -> ImportCategory {
        if import_stmt.contains("use std::") {
            ImportCategory::Std
        } else if import_stmt.contains("use crate::") {
            ImportCategory::Internal
        } else if import_stmt.starts_with("pub use") {
            ImportCategory::Reexport
        } else {
            ImportCategory::External
        }
    }

    /// Get organized imports as strings
    pub fn get_organized_imports(&self) -> Vec<String> {
        let mut result = Vec::new();
        let mut last_category = None;

        for import in &self.imports {
            // Add blank line between categories
            if let Some(last) = last_category {
                if last != import.category {
                    result.push(String::new());
                }
            }

            result.push(import.statement.clone());
            last_category = Some(import.category.clone());
        }

        result
    }

    /// Get all imports as a single block
    pub fn get_import_block(&self) -> String {
        self.get_organized_imports().join("\n")
    }

    /// Check if any imports are needed
    pub fn has_imports(&self) -> bool {
        !self.imports.is_empty()
    }

    /// Clear all imports
    pub fn clear(&mut self) {
        self.imports.clear();
        self.aliases.clear();
    }
}

impl Default for ImportCollector {
    fn default() -> Self {
        Self::new()
    }
}
