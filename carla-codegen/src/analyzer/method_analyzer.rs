//! Method analysis for determining self parameter types

use crate::{config::Config, parser::yaml_schema::Method};

/// Determine the self parameter type for a method
pub fn determine_self_type(method: &Method, class_name: &str) -> SelfType {
    determine_self_type_with_config(method, class_name, &Config::default())
}

/// Determine the self parameter type for a method with configuration
pub fn determine_self_type_with_config(
    method: &Method,
    class_name: &str,
    config: &Config,
) -> SelfType {
    // Check for explicit override first
    if let Some(override_type) = config.get_self_type_override(class_name, &method.def_name) {
        return match override_type.as_str() {
            "self" | "owned" => SelfType::Owned,
            "ref" | "&self" => SelfType::Ref,
            "mut_ref" | "&mut self" => SelfType::MutRef,
            _ => SelfType::Ref, // Default fallback
        };
    }

    let method_config = config.get_method_signature_config(class_name, &method.def_name);

    // If auto-detection is disabled, use the default
    if !method_config.auto_detect_self_type {
        return match method_config.default_self_type.as_str() {
            "self" | "owned" => SelfType::Owned,
            "mut_ref" | "&mut self" => SelfType::MutRef,
            _ => SelfType::Ref,
        };
    }
    let method_name = &method.def_name;

    // Check configured patterns
    for pattern in &method_config.owned_self_patterns {
        if matches_pattern(method_name, pattern) {
            return SelfType::Owned;
        }
    }

    for pattern in &method_config.mut_self_patterns {
        if matches_pattern(method_name, pattern) {
            return SelfType::MutRef;
        }
    }

    for pattern in &method_config.ref_self_patterns {
        if matches_pattern(method_name, pattern) {
            return SelfType::Ref;
        }
    }

    // Fallback to legacy detection
    if is_destructive_method(method_name) {
        return SelfType::Owned;
    }

    if is_mutation_method(method_name, method) {
        return SelfType::MutRef;
    }

    // Default to &self for getters and queries
    SelfType::Ref
}

/// Check if a method name matches a pattern (supports * wildcards)
fn matches_pattern(method_name: &str, pattern: &str) -> bool {
    if pattern.contains('*') {
        let parts: Vec<&str> = pattern.split('*').collect();
        if parts.len() == 2 {
            let (prefix, suffix) = (parts[0], parts[1]);
            method_name.starts_with(prefix) && method_name.ends_with(suffix)
        } else {
            // More complex patterns not supported yet
            method_name == pattern
        }
    } else {
        method_name == pattern
    }
}

/// Self parameter types
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize)]
pub enum SelfType {
    /// Takes ownership (self)
    Owned,
    /// Immutable reference (&self)
    Ref,
    /// Mutable reference (&mut self)
    MutRef,
}

impl SelfType {
    /// Convert to Rust syntax
    pub fn to_rust_string(self) -> &'static str {
        match self {
            SelfType::Owned => "self",
            SelfType::Ref => "&self",
            SelfType::MutRef => "&mut self",
        }
    }
}

/// Check if a method is destructive (consumes self)
fn is_destructive_method(method_name: &str) -> bool {
    matches!(
        method_name,
        "destroy" | "delete" | "remove" | "consume" | "take"
    )
}

/// Check if a method mutates the object
fn is_mutation_method(method_name: &str, method: &Method) -> bool {
    // Check method name patterns
    if method_name.starts_with("set_")
        || method_name.starts_with("add_")
        || method_name.starts_with("remove_")
        || method_name.starts_with("clear_")
        || method_name.starts_with("reset_")
        || method_name.starts_with("apply_")
        || method_name.starts_with("enable_")
        || method_name.starts_with("disable_")
        || method_name.starts_with("open_")
        || method_name.starts_with("close_")
        || method_name.starts_with("start_")
        || method_name.starts_with("stop_")
        || method_name.starts_with("freeze_")
        || method_name.starts_with("unfreeze_")
        || method_name.starts_with("show_")
        || method_name.starts_with("hide_")
        || method_name.starts_with("blend_")
        || method_name.starts_with("use_")
    {
        return true;
    }

    // Methods that return nothing usually mutate
    if method.return_type.is_none()
        || matches!(method.return_type.as_deref(), Some("None") | Some(""))
    {
        // Exception for getter-like methods
        if !method_name.starts_with("get_") && !method_name.starts_with("is_") {
            return true;
        }
    }

    false
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_destructive_methods() {
        assert_eq!(
            determine_self_type(
                &Method {
                    def_name: "destroy".to_string(),
                    return_type: Some("bool".to_string()),
                    return_units: None,
                    params: vec![],
                    doc: None,
                    note: None,
                    warning: None,
                    raises: None,
                    is_static: false,
                },
                "Actor"
            ),
            SelfType::Owned
        );
    }

    #[test]
    fn test_mutation_methods() {
        assert_eq!(
            determine_self_type(
                &Method {
                    def_name: "set_location".to_string(),
                    return_type: None,
                    return_units: None,
                    params: vec![],
                    doc: None,
                    note: None,
                    warning: None,
                    raises: None,
                    is_static: false,
                },
                "Actor"
            ),
            SelfType::MutRef
        );

        assert_eq!(
            determine_self_type(
                &Method {
                    def_name: "add_impulse".to_string(),
                    return_type: None,
                    return_units: None,
                    params: vec![],
                    doc: None,
                    note: None,
                    warning: None,
                    raises: None,
                    is_static: false,
                },
                "Actor"
            ),
            SelfType::MutRef
        );
    }

    #[test]
    fn test_getter_methods() {
        assert_eq!(
            determine_self_type(
                &Method {
                    def_name: "get_location".to_string(),
                    return_type: Some("carla.Location".to_string()),
                    return_units: None,
                    params: vec![],
                    doc: None,
                    note: None,
                    warning: None,
                    raises: None,
                    is_static: false,
                },
                "Actor"
            ),
            SelfType::Ref
        );

        assert_eq!(
            determine_self_type(
                &Method {
                    def_name: "is_alive".to_string(),
                    return_type: Some("bool".to_string()),
                    return_units: None,
                    params: vec![],
                    doc: None,
                    note: None,
                    warning: None,
                    raises: None,
                    is_static: false,
                },
                "Actor"
            ),
            SelfType::Ref
        );
    }
}
