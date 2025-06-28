//! Dependency analysis for class hierarchies and type relationships

use crate::{
    error::{CodegenError, Result},
    parser::yaml_schema::{Class, Module},
};
use indexmap::IndexSet;
use std::collections::{HashMap, HashSet};
use tracing::debug;

/// Dependency graph for classes
pub struct DependencyGraph {
    /// Map from class name to its dependencies
    dependencies: HashMap<String, IndexSet<String>>,
    /// Map from class name to its dependents (reverse dependencies)
    dependents: HashMap<String, IndexSet<String>>,
    /// Map from class name to its parent
    inheritance: HashMap<String, String>,
}

impl DependencyGraph {
    /// Create a new dependency graph
    pub fn new() -> Self {
        Self {
            dependencies: HashMap::new(),
            dependents: HashMap::new(),
            inheritance: HashMap::new(),
        }
    }

    /// Build dependency graph from modules
    pub fn build_from_modules(&mut self, modules: &[Module]) -> Result<()> {
        // First pass: collect all class names and inheritance
        let mut all_classes = HashSet::new();

        for module in modules {
            for class in &module.classes {
                all_classes.insert(class.class_name.clone());

                // Ensure every class has an entry in dependencies map
                self.dependencies
                    .entry(class.class_name.clone())
                    .or_insert_with(IndexSet::new);

                // Track inheritance
                if let Some(parent) = &class.parent {
                    // Normalize parent name by removing "carla." prefix
                    let normalized_parent =
                        parent.strip_prefix("carla.").unwrap_or(parent).to_string();
                    self.inheritance
                        .insert(class.class_name.clone(), normalized_parent.clone());
                    self.add_dependency(&class.class_name, &normalized_parent);
                }
            }
        }

        // Second pass: analyze type dependencies
        for module in modules {
            for class in &module.classes {
                self.analyze_class_dependencies(class, &all_classes)?;
            }
        }

        // Check for circular dependencies
        self.check_circular_dependencies()?;

        Ok(())
    }

    /// Analyze dependencies for a single class
    fn analyze_class_dependencies(
        &mut self,
        class: &Class,
        known_classes: &HashSet<String>,
    ) -> Result<()> {
        // Check instance variable types
        for var in &class.instance_variables {
            if let Some(ref var_type) = var.var_type {
                if let Some(dep_class) = extract_class_from_type(var_type) {
                    if known_classes.contains(&dep_class) && dep_class != class.class_name {
                        self.add_dependency(&class.class_name, &dep_class);
                    }
                }
            }
        }

        // Check method parameter and return types
        for method in &class.methods {
            // Return type
            if let Some(return_type) = &method.return_type {
                if let Some(dep_class) = extract_class_from_type(return_type) {
                    if known_classes.contains(&dep_class) && dep_class != class.class_name {
                        self.add_dependency(&class.class_name, &dep_class);
                    }
                }
            }

            // Parameter types
            for param in &method.params {
                if let Some(param_type) = &param.param_type {
                    if let Some(dep_class) = extract_class_from_type(param_type) {
                        if known_classes.contains(&dep_class) && dep_class != class.class_name {
                            self.add_dependency(&class.class_name, &dep_class);
                        }
                    }
                }
            }
        }

        Ok(())
    }

    /// Add a dependency relationship
    fn add_dependency(&mut self, class: &str, dependency: &str) {
        // Normalize dependency name by removing "carla." prefix
        let normalized_dep = dependency.strip_prefix("carla.").unwrap_or(dependency);

        debug!("Adding dependency: {} depends on {}", class, normalized_dep);

        self.dependencies
            .entry(class.to_string())
            .or_insert_with(IndexSet::new)
            .insert(normalized_dep.to_string());

        self.dependents
            .entry(normalized_dep.to_string())
            .or_insert_with(IndexSet::new)
            .insert(class.to_string());
    }

    /// Check for circular dependencies
    fn check_circular_dependencies(&self) -> Result<()> {
        let mut visited = HashSet::new();
        let mut stack = HashSet::new();

        for class in self.dependencies.keys() {
            if !visited.contains(class) {
                self.dfs_check_circular(class, &mut visited, &mut stack)?;
            }
        }

        Ok(())
    }

    /// DFS helper for circular dependency check
    fn dfs_check_circular(
        &self,
        class: &str,
        visited: &mut HashSet<String>,
        stack: &mut HashSet<String>,
    ) -> Result<()> {
        visited.insert(class.to_string());
        stack.insert(class.to_string());

        if let Some(deps) = self.dependencies.get(class) {
            for dep in deps {
                if !visited.contains(dep) {
                    self.dfs_check_circular(dep, visited, stack)?;
                } else if stack.contains(dep) {
                    return Err(CodegenError::CircularDependency(format!(
                        "Circular dependency detected: {} -> {}",
                        class, dep
                    )));
                }
            }
        }

        stack.remove(class);
        Ok(())
    }

    /// Get topologically sorted class list
    pub fn topological_sort(&self) -> Result<Vec<String>> {
        let mut result = Vec::new();
        let mut visited = HashSet::new();
        let mut temp_visited = HashSet::new();

        // Get all classes
        let mut all_classes: HashSet<String> = self.dependencies.keys().cloned().collect();
        all_classes.extend(self.dependents.keys().cloned());

        for class in &all_classes {
            if !visited.contains(class) {
                self.topological_sort_dfs(class, &mut visited, &mut temp_visited, &mut result)?;
            }
        }

        Ok(result)
    }

    /// DFS helper for topological sort
    fn topological_sort_dfs(
        &self,
        class: &str,
        visited: &mut HashSet<String>,
        temp_visited: &mut HashSet<String>,
        result: &mut Vec<String>,
    ) -> Result<()> {
        if temp_visited.contains(class) {
            return Err(CodegenError::CircularDependency(format!(
                "Circular dependency detected at: {}",
                class
            )));
        }

        if !visited.contains(class) {
            temp_visited.insert(class.to_string());

            if let Some(deps) = self.dependencies.get(class) {
                for dep in deps {
                    self.topological_sort_dfs(dep, visited, temp_visited, result)?;
                }
            }

            temp_visited.remove(class);
            visited.insert(class.to_string());
            result.push(class.to_string());
        }

        Ok(())
    }

    /// Get direct dependencies of a class
    pub fn get_dependencies(&self, class: &str) -> Option<&IndexSet<String>> {
        self.dependencies.get(class)
    }

    /// Get classes that depend on this class
    pub fn get_dependents(&self, class: &str) -> Option<&IndexSet<String>> {
        self.dependents.get(class)
    }

    /// Get parent class if any
    pub fn get_parent(&self, class: &str) -> Option<&String> {
        self.inheritance.get(class)
    }
}

/// Extract class name from a type string
fn extract_class_from_type(type_str: &str) -> Option<String> {
    // Handle carla.ClassName format
    if type_str.starts_with("carla.") {
        let class_name = type_str.strip_prefix("carla.")?;
        // Handle generic types like list(carla.Actor)
        if let Some(end) = class_name.find(&['(', '[', ' '][..]) {
            return Some(class_name[..end].to_string());
        }
        return Some(class_name.to_string());
    }

    // Handle list(carla.ClassName) format
    if type_str.starts_with("list(carla.") && type_str.ends_with(")") {
        let inner = &type_str[5..type_str.len() - 1];
        return extract_class_from_type(inner);
    }

    None
}

impl Default for DependencyGraph {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_extract_class_from_type() {
        assert_eq!(
            extract_class_from_type("carla.Actor"),
            Some("Actor".to_string())
        );
        assert_eq!(
            extract_class_from_type("carla.Vehicle"),
            Some("Vehicle".to_string())
        );
        assert_eq!(
            extract_class_from_type("list(carla.Actor)"),
            Some("Actor".to_string())
        );
        assert_eq!(extract_class_from_type("int"), None);
    }

    #[test]
    fn test_simple_dependency_graph() {
        let modules = vec![Module {
            module_name: "carla".to_string(),
            doc: None,
            classes: vec![
                Class {
                    class_name: "Actor".to_string(),
                    parent: None,
                    doc: None,
                    instance_variables: vec![],
                    methods: vec![],
                },
                Class {
                    class_name: "Vehicle".to_string(),
                    parent: Some("carla.Actor".to_string()),
                    doc: None,
                    instance_variables: vec![],
                    methods: vec![],
                },
            ],
        }];

        let mut graph = DependencyGraph::new();
        graph.build_from_modules(&modules).unwrap();

        // Check inheritance (parent is normalized without "carla." prefix)
        assert_eq!(graph.get_parent("Vehicle"), Some(&"Actor".to_string()));

        // Check topological sort includes all classes
        let sorted = graph.topological_sort().unwrap();
        assert!(sorted.contains(&"Actor".to_string()));
        assert!(sorted.contains(&"Vehicle".to_string()));

        // Vehicle depends on Actor through inheritance
        let actor_idx = sorted.iter().position(|x| x == "Actor").unwrap();
        let vehicle_idx = sorted.iter().position(|x| x == "Vehicle").unwrap();

        assert!(
            actor_idx < vehicle_idx,
            "Actor should come before Vehicle in topological sort"
        );
    }
}
