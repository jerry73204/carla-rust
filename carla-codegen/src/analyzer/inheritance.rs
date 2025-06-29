//! Class inheritance analysis and resolution

use crate::parser::yaml_schema::Module;
use std::collections::HashMap;
use tracing::debug;

/// Inheritance information for a class
#[derive(Debug, Clone)]
pub struct InheritanceInfo {
    /// Direct parent class name (if any)
    pub parent: Option<String>,
    /// All ancestor classes (including parent)
    pub ancestors: Vec<String>,
    /// Direct child classes
    pub children: Vec<String>,
    /// All descendant classes (including children)
    pub descendants: Vec<String>,
}

/// Inheritance resolver for class hierarchies
pub struct InheritanceResolver {
    /// Map from class name to its inheritance info
    inheritance_map: HashMap<String, InheritanceInfo>,
}

impl InheritanceResolver {
    /// Create a new inheritance resolver
    pub fn new() -> Self {
        Self {
            inheritance_map: HashMap::new(),
        }
    }

    /// Build inheritance information from modules
    pub fn build_from_modules(&mut self, modules: &[Module]) {
        // First pass: collect direct parent-child relationships
        let mut parent_map = HashMap::new();
        let mut children_map: HashMap<String, Vec<String>> = HashMap::new();

        for module in modules {
            for class in &module.classes {
                let class_name = &class.class_name;

                // Initialize entry
                self.inheritance_map
                    .entry(class_name.clone())
                    .or_insert(InheritanceInfo {
                        parent: None,
                        ancestors: Vec::new(),
                        children: Vec::new(),
                        descendants: Vec::new(),
                    });

                if let Some(parent) = &class.parent {
                    // Clean parent name (remove "carla." prefix if present)
                    let clean_parent = parent.strip_prefix("carla.").unwrap_or(parent);

                    parent_map.insert(class_name.clone(), clean_parent.to_string());
                    children_map
                        .entry(clean_parent.to_string())
                        .or_default()
                        .push(class_name.clone());

                    // Update inheritance info
                    if let Some(info) = self.inheritance_map.get_mut(class_name) {
                        info.parent = Some(clean_parent.to_string());
                    }
                }
            }
        }

        // Second pass: build ancestor chains
        for class in parent_map.keys() {
            let ancestors = self.build_ancestor_chain(class, &parent_map);
            if let Some(info) = self.inheritance_map.get_mut(class) {
                info.ancestors = ancestors;
            }
        }

        // Third pass: update children and descendants
        for (parent, children) in &children_map {
            let descendants = self.build_descendant_list(parent, &children_map);
            if let Some(info) = self.inheritance_map.get_mut(parent) {
                info.children = children.clone();
                info.descendants = descendants;
            }
        }

        debug!(
            "Built inheritance map for {} classes",
            self.inheritance_map.len()
        );
    }

    /// Build the complete ancestor chain for a class
    fn build_ancestor_chain(
        &self,
        class: &str,
        parent_map: &HashMap<String, String>,
    ) -> Vec<String> {
        let mut ancestors = Vec::new();
        let mut current = class;

        // Follow parent chain, with cycle detection
        let mut visited = std::collections::HashSet::new();
        visited.insert(current.to_string());

        while let Some(parent) = parent_map.get(current) {
            if visited.contains(parent) {
                // Cycle detected, stop here
                break;
            }
            ancestors.push(parent.clone());
            visited.insert(parent.clone());
            current = parent;
        }

        ancestors
    }

    /// Build the complete descendant list for a class
    fn build_descendant_list(
        &self,
        class: &str,
        children_map: &HashMap<String, Vec<String>>,
    ) -> Vec<String> {
        let mut descendants = Vec::new();
        let mut to_visit = vec![class.to_string()];
        let mut visited = std::collections::HashSet::new();

        while let Some(current) = to_visit.pop() {
            if visited.contains(&current) {
                continue;
            }
            visited.insert(current.clone());

            if let Some(children) = children_map.get(&current) {
                for child in children {
                    if child != class && !descendants.contains(child) {
                        descendants.push(child.clone());
                        to_visit.push(child.clone());
                    }
                }
            }
        }

        descendants
    }

    /// Get inheritance info for a class
    pub fn get_info(&self, class: &str) -> Option<&InheritanceInfo> {
        self.inheritance_map.get(class)
    }

    /// Check if a class inherits from another
    pub fn inherits_from(&self, class: &str, ancestor: &str) -> bool {
        if let Some(info) = self.get_info(class) {
            info.ancestors.contains(&ancestor.to_string())
        } else {
            false
        }
    }

    /// Get all root classes (classes with no parent)
    pub fn get_root_classes(&self) -> Vec<String> {
        self.inheritance_map
            .iter()
            .filter(|(_, info)| info.parent.is_none())
            .map(|(name, _)| name.clone())
            .collect()
    }

    /// Get all leaf classes (classes with no children)
    pub fn get_leaf_classes(&self) -> Vec<String> {
        self.inheritance_map
            .iter()
            .filter(|(_, info)| info.children.is_empty())
            .map(|(name, _)| name.clone())
            .collect()
    }
}

impl Default for InheritanceResolver {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::parser::yaml_schema::Class;

    #[test]
    fn test_inheritance_resolver() {
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
                Class {
                    class_name: "Car".to_string(),
                    parent: Some("carla.Vehicle".to_string()),
                    doc: None,
                    instance_variables: vec![],
                    methods: vec![],
                },
            ],
        }];

        let mut resolver = InheritanceResolver::new();
        resolver.build_from_modules(&modules);

        // Test parent relationships
        assert_eq!(
            resolver.get_info("Vehicle").unwrap().parent,
            Some("Actor".to_string())
        );
        assert_eq!(
            resolver.get_info("Car").unwrap().parent,
            Some("Vehicle".to_string())
        );

        // Test ancestor chains
        let car_ancestors = &resolver.get_info("Car").unwrap().ancestors;
        assert_eq!(car_ancestors.len(), 2);
        assert!(car_ancestors.contains(&"Vehicle".to_string()));
        assert!(car_ancestors.contains(&"Actor".to_string()));

        // Test inheritance check
        assert!(resolver.inherits_from("Car", "Actor"));
        assert!(resolver.inherits_from("Car", "Vehicle"));
        assert!(!resolver.inherits_from("Actor", "Vehicle"));

        // Test root and leaf classes
        assert_eq!(resolver.get_root_classes(), vec!["Actor"]);
        assert_eq!(resolver.get_leaf_classes(), vec!["Car"]);
    }
}
