//! Analysis and mapping of Python special methods to Rust traits

use crate::parser::yaml_schema::Method;
use std::collections::HashMap;

/// Represents a Rust trait that should be implemented based on Python special methods
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct TraitMapping {
    /// The Rust trait name (e.g., "PartialEq", "Debug")
    pub trait_name: &'static str,
    /// Whether this trait should be derived or manually implemented
    pub should_derive: bool,
    /// Required Python methods for this trait
    pub required_methods: Vec<&'static str>,
    /// Optional Python methods that enhance the trait
    pub optional_methods: Vec<&'static str>,
    /// Additional traits that should also be implemented
    pub implies_traits: Vec<&'static str>,
}

/// Analyzes Python special methods and determines which Rust traits to implement
pub struct SpecialMethodAnalyzer {
    mappings: HashMap<&'static str, TraitMapping>,
}

impl SpecialMethodAnalyzer {
    /// Create a new analyzer with default mappings
    pub fn new() -> Self {
        let mut mappings = HashMap::new();

        // Equality traits
        mappings.insert(
            "__eq__",
            TraitMapping {
                trait_name: "PartialEq",
                should_derive: false,
                required_methods: vec!["__eq__"],
                optional_methods: vec!["__ne__"],
                implies_traits: vec!["Eq"], // If no __ne__ or it's consistent
            },
        );

        // Ordering traits
        mappings.insert(
            "__lt__",
            TraitMapping {
                trait_name: "PartialOrd",
                should_derive: false,
                required_methods: vec!["__lt__", "__le__", "__gt__", "__ge__"],
                optional_methods: vec![],
                implies_traits: vec!["Ord"], // If total ordering is possible
            },
        );

        // Hash trait
        mappings.insert(
            "__hash__",
            TraitMapping {
                trait_name: "Hash",
                should_derive: false,
                required_methods: vec!["__hash__"],
                optional_methods: vec![],
                implies_traits: vec![],
            },
        );

        // Display/Debug traits
        mappings.insert(
            "__str__",
            TraitMapping {
                trait_name: "Display",
                should_derive: false,
                required_methods: vec!["__str__"],
                optional_methods: vec![],
                implies_traits: vec![],
            },
        );

        mappings.insert(
            "__repr__",
            TraitMapping {
                trait_name: "Debug",
                should_derive: false,
                required_methods: vec!["__repr__"],
                optional_methods: vec![],
                implies_traits: vec![],
            },
        );

        // Iterator trait
        mappings.insert(
            "__iter__",
            TraitMapping {
                trait_name: "IntoIterator",
                should_derive: false,
                required_methods: vec!["__iter__"],
                optional_methods: vec!["__next__"],
                implies_traits: vec![],
            },
        );

        // Index traits
        mappings.insert(
            "__getitem__",
            TraitMapping {
                trait_name: "Index",
                should_derive: false,
                required_methods: vec!["__getitem__"],
                optional_methods: vec![],
                implies_traits: vec![],
            },
        );

        mappings.insert(
            "__setitem__",
            TraitMapping {
                trait_name: "IndexMut",
                should_derive: false,
                required_methods: vec!["__getitem__", "__setitem__"],
                optional_methods: vec![],
                implies_traits: vec!["Index"],
            },
        );

        // Container traits
        mappings.insert(
            "__len__",
            TraitMapping {
                trait_name: "Len",
                should_derive: false,
                required_methods: vec!["__len__"],
                optional_methods: vec![],
                implies_traits: vec![],
            },
        );

        mappings.insert(
            "__contains__",
            TraitMapping {
                trait_name: "Contains",
                should_derive: false,
                required_methods: vec!["__contains__"],
                optional_methods: vec![],
                implies_traits: vec![],
            },
        );

        // Conversion traits
        mappings.insert(
            "__bool__",
            TraitMapping {
                trait_name: "Into<bool>",
                should_derive: false,
                required_methods: vec!["__bool__"],
                optional_methods: vec![],
                implies_traits: vec![],
            },
        );

        mappings.insert(
            "__int__",
            TraitMapping {
                trait_name: "Into<i32>",
                should_derive: false,
                required_methods: vec!["__int__"],
                optional_methods: vec![],
                implies_traits: vec![],
            },
        );

        mappings.insert(
            "__float__",
            TraitMapping {
                trait_name: "Into<f64>",
                should_derive: false,
                required_methods: vec!["__float__"],
                optional_methods: vec![],
                implies_traits: vec![],
            },
        );

        // Arithmetic traits
        mappings.insert(
            "__add__",
            TraitMapping {
                trait_name: "Add",
                should_derive: false,
                required_methods: vec!["__add__"],
                optional_methods: vec!["__radd__"],
                implies_traits: vec![],
            },
        );

        mappings.insert(
            "__sub__",
            TraitMapping {
                trait_name: "Sub",
                should_derive: false,
                required_methods: vec!["__sub__"],
                optional_methods: vec!["__rsub__"],
                implies_traits: vec![],
            },
        );

        mappings.insert(
            "__mul__",
            TraitMapping {
                trait_name: "Mul",
                should_derive: false,
                required_methods: vec!["__mul__"],
                optional_methods: vec!["__rmul__"],
                implies_traits: vec![],
            },
        );

        mappings.insert(
            "__div__",
            TraitMapping {
                trait_name: "Div",
                should_derive: false,
                required_methods: vec!["__div__", "__truediv__"],
                optional_methods: vec!["__rdiv__", "__rtruediv__"],
                implies_traits: vec![],
            },
        );

        // Context manager (needs custom impl)
        mappings.insert(
            "__enter__",
            TraitMapping {
                trait_name: "ContextManager",
                should_derive: false,
                required_methods: vec!["__enter__", "__exit__"],
                optional_methods: vec![],
                implies_traits: vec![],
            },
        );

        Self { mappings }
    }

    /// Analyze a class's methods with full method information
    pub fn analyze_class_with_methods(&self, methods: &[Method]) -> AnalysisResult {
        let class_methods: Vec<String> = methods.iter().map(|m| m.def_name.clone()).collect();
        let mut result = self.analyze_class(&class_methods);

        // Check for union types in special methods
        for method in methods {
            if method.def_name == "__eq__" || method.def_name == "__ne__" {
                // Find the 'other' parameter
                if let Some(other_param) = method.params.iter().find(|p| p.param_name == "other") {
                    tracing::debug!("Found 'other' param in __eq__ method: {:?}", other_param);
                    if let Some(param_type) = &other_param.param_type {
                        tracing::debug!("Parameter type for 'other': {}", param_type);
                        // Check if it's a union type
                        if param_type.contains(" / ") || param_type.contains(" or ") {
                            // Store union type information for PartialEq generation
                            tracing::debug!("Detected union type: {}", param_type);
                            result.union_eq_types = Some(param_type.clone());
                        }
                    }
                }
            }
        }

        result
    }

    /// Analyze a class's methods and determine which traits to implement
    pub fn analyze_class(&self, class_methods: &[String]) -> AnalysisResult {
        let mut traits_to_implement = Vec::new();
        let mut traits_to_derive = Vec::new();
        let mut manual_implementations = HashMap::new();

        // Check each special method
        for method_name in class_methods {
            if let Some(mapping) = self.mappings.get(method_name.as_str()) {
                // Check if all required methods are present
                let has_all_required = mapping
                    .required_methods
                    .iter()
                    .all(|&req| class_methods.contains(&req.to_string()));

                if has_all_required {
                    if mapping.should_derive {
                        traits_to_derive.push(mapping.trait_name.to_string());
                    } else {
                        tracing::debug!(
                            "Adding manual implementation for trait: {}",
                            mapping.trait_name
                        );
                        traits_to_implement.push(mapping.trait_name.to_string());
                        manual_implementations.insert(
                            mapping.trait_name.to_string(),
                            SpecialMethodImpl {
                                trait_name: mapping.trait_name.to_string(),
                                methods: method_name.clone(),
                                additional_methods: mapping
                                    .optional_methods
                                    .iter()
                                    .filter(|&&m| class_methods.contains(&m.to_string()))
                                    .map(|&s| s.to_string())
                                    .collect(),
                            },
                        );
                    }

                    // Add implied traits
                    for implied in &mapping.implies_traits {
                        traits_to_implement.push(implied.to_string());
                    }
                }
            }
        }

        // Special handling for Eq trait (requires PartialEq)
        if traits_to_implement.contains(&"PartialEq".to_string())
            && !class_methods.contains(&"__ne__".to_string())
        {
            // If __eq__ exists but not __ne__, we can derive Eq
            traits_to_derive.push("Eq".to_string());
        }

        // Special handling for Ord trait (requires PartialOrd + Eq)
        if traits_to_implement.contains(&"PartialOrd".to_string())
            && traits_to_implement.contains(&"PartialEq".to_string())
        {
            // Check if we have total ordering
            let ordering_methods = ["__lt__", "__le__", "__gt__", "__ge__"];
            let has_all_ordering = ordering_methods
                .iter()
                .all(|&m| class_methods.contains(&m.to_string()));

            if has_all_ordering {
                traits_to_derive.push("Ord".to_string());
            }
        }

        // Clone and Copy detection based on class characteristics
        let should_derive_copy = self.should_derive_copy(class_methods);
        if should_derive_copy {
            traits_to_derive.push("Copy".to_string());
            traits_to_derive.push("Clone".to_string());
        } else {
            // Most classes should be Clone
            traits_to_derive.push("Clone".to_string());
        }

        AnalysisResult {
            traits_to_implement,
            traits_to_derive,
            manual_implementations,
            special_method_mappings: self.create_method_mappings(class_methods),
            union_eq_types: None,
        }
    }

    /// Determine if a class should derive Copy
    fn should_derive_copy(&self, class_methods: &[String]) -> bool {
        // Don't derive Copy if class has mutable methods or manages resources
        let has_mutable_methods = class_methods.iter().any(|m| {
            m.starts_with("set_")
                || m.starts_with("add_")
                || m.starts_with("remove_")
                || m == "__setitem__"
                || m == "__delitem__"
        });

        let has_resource_methods = class_methods.iter().any(|m| {
            m == "__del__" || m == "__enter__" || m == "__exit__" || m == "close" || m == "destroy"
        });

        !has_mutable_methods && !has_resource_methods
    }

    /// Create mappings from Python methods to Rust implementations
    fn create_method_mappings(&self, class_methods: &[String]) -> HashMap<String, String> {
        let mut mappings = HashMap::new();

        for method in class_methods {
            let rust_impl = match method.as_str() {
                "__eq__" => "eq",
                "__ne__" => "ne",
                "__lt__" => "lt",
                "__le__" => "le",
                "__gt__" => "gt",
                "__ge__" => "ge",
                "__hash__" => "hash",
                "__str__" => "fmt",  // for Display
                "__repr__" => "fmt", // for Debug
                "__len__" => "len",
                "__bool__" => "into",
                "__int__" => "into",
                "__float__" => "into",
                "__add__" => "add",
                "__sub__" => "sub",
                "__mul__" => "mul",
                "__div__" | "__truediv__" => "div",
                "__mod__" => "rem",
                "__neg__" => "neg",
                "__pos__" => "pos",
                "__abs__" => "abs",
                "__invert__" => "not",
                "__and__" => "bitand",
                "__or__" => "bitor",
                "__xor__" => "bitxor",
                "__lshift__" => "shl",
                "__rshift__" => "shr",
                "__getitem__" => "index",
                "__setitem__" => "index_mut",
                "__contains__" => "contains",
                "__iter__" => "into_iter",
                "__next__" => "next",
                _ => continue,
            };

            mappings.insert(method.clone(), rust_impl.to_string());
        }

        mappings
    }
}

/// Result of analyzing special methods for a class
#[derive(Debug, Clone)]
pub struct AnalysisResult {
    /// Traits that need manual implementation
    pub traits_to_implement: Vec<String>,
    /// Traits that can be derived
    pub traits_to_derive: Vec<String>,
    /// Manual trait implementations needed
    pub manual_implementations: HashMap<String, SpecialMethodImpl>,
    /// Mapping of Python method names to Rust trait method names
    pub special_method_mappings: HashMap<String, String>,
    /// Union types for __eq__ method (e.g., "bool / int / float")
    pub union_eq_types: Option<String>,
}

/// Information about a manual trait implementation
#[derive(Debug, Clone)]
pub struct SpecialMethodImpl {
    /// The trait name
    pub trait_name: String,
    /// The main Python method
    pub methods: String,
    /// Additional methods that enhance the implementation
    pub additional_methods: Vec<String>,
}

impl Default for SpecialMethodAnalyzer {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_equality_trait_detection() {
        let analyzer = SpecialMethodAnalyzer::new();
        let methods = vec!["__eq__".to_string(), "__ne__".to_string()];

        let result = analyzer.analyze_class(&methods);

        assert!(result
            .traits_to_implement
            .contains(&"PartialEq".to_string()));
        assert!(result.manual_implementations.contains_key("PartialEq"));
    }

    #[test]
    fn test_ordering_trait_detection() {
        let analyzer = SpecialMethodAnalyzer::new();
        let methods = vec![
            "__eq__".to_string(),
            "__lt__".to_string(),
            "__le__".to_string(),
            "__gt__".to_string(),
            "__ge__".to_string(),
        ];

        let result = analyzer.analyze_class(&methods);

        assert!(result
            .traits_to_implement
            .contains(&"PartialOrd".to_string()));
        assert!(result.traits_to_derive.contains(&"Ord".to_string()));
    }

    #[test]
    fn test_display_debug_detection() {
        let analyzer = SpecialMethodAnalyzer::new();
        let methods = vec!["__str__".to_string(), "__repr__".to_string()];

        let result = analyzer.analyze_class(&methods);

        assert!(result.traits_to_implement.contains(&"Display".to_string()));
        assert!(result.traits_to_implement.contains(&"Debug".to_string()));
    }

    #[test]
    fn test_copy_derivation() {
        let analyzer = SpecialMethodAnalyzer::new();

        // Immutable class should derive Copy
        let immutable_methods = vec!["__eq__".to_string(), "get_value".to_string()];
        let result = analyzer.analyze_class(&immutable_methods);
        assert!(result.traits_to_derive.contains(&"Copy".to_string()));

        // Mutable class should not derive Copy
        let mutable_methods = vec!["__eq__".to_string(), "set_value".to_string()];
        let result = analyzer.analyze_class(&mutable_methods);
        assert!(!result.traits_to_derive.contains(&"Copy".to_string()));
        assert!(result.traits_to_derive.contains(&"Clone".to_string()));
    }
}
