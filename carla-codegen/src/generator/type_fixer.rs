//! Post-processing type fixes for generated code

use tracing::debug;

/// Fix common type issues in generated code
pub struct TypeFixer;

impl TypeFixer {
    /// Fix bare Vec without generic parameter
    pub fn fix_bare_vec(type_str: &str) -> String {
        if type_str == "Vec" {
            debug!("Fixing bare Vec to Vec<crate::carla::Actor>");
            "Vec<crate::carla::Actor>".to_string()
        } else {
            type_str.to_string()
        }
    }

    /// Fix missing imports by analyzing type usage
    pub fn get_required_imports(type_str: &str) -> Vec<String> {
        let mut imports = Vec::new();

        if type_str.contains("HashMap") {
            imports.push("use std::collections::HashMap;".to_string());
        }
        if type_str.contains("HashSet") {
            imports.push("use std::collections::HashSet;".to_string());
        }
        if type_str.contains("BTreeMap") {
            imports.push("use std::collections::BTreeMap;".to_string());
        }
        if type_str.contains("BTreeSet") {
            imports.push("use std::collections::BTreeSet;".to_string());
        }

        // Check for types that need to be imported from the same module
        if type_str.contains("Actor") && !type_str.contains("::") {
            // Actor is used without module path - needs import
            imports.push("use crate::carla::Actor;".to_string());
        }
        if type_str.contains("World") && !type_str.contains("::") {
            imports.push("use crate::carla::World;".to_string());
        }
        if type_str.contains("Command") && !type_str.contains("::") {
            imports.push("use crate::carla::Command;".to_string());
        }

        imports
    }

    /// Fix WeatherParameters module path issue
    pub fn fix_module_paths(type_str: &str) -> String {
        // Fix WeatherParameters being looked up in crate root
        if type_str == "WeatherParameters" {
            "crate::carla::WeatherParameters".to_string()
        } else {
            type_str.to_string()
        }
    }

    /// Apply all fixes to a type string
    pub fn fix_type_string(type_str: &str) -> String {
        let mut fixed = Self::fix_bare_vec(type_str);
        fixed = Self::fix_module_paths(&fixed);
        fixed
    }
}
