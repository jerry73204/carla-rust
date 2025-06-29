//! AST generation modules for type-safe Rust code generation
//!
//! This module provides AST builders for generating Rust code using the `syn` crate,
//! replacing string-based template generation with type-safe AST construction.

pub mod doc_builder;
pub mod formatter;
pub mod impl_builder;
pub mod struct_builder;
pub mod trait_builder;

use quote::ToTokens;
use syn::Result as SynResult;

/// Core trait for AST builders that can generate Rust syntax nodes
pub trait AstBuilder {
    type Output: ToTokens;

    /// Build the AST node
    fn build(&self) -> SynResult<Self::Output>;

    /// Validate the builder configuration before building
    fn validate(&self) -> crate::Result<()> {
        Ok(())
    }
}

/// Context for AST generation containing shared state and utilities
#[derive(Debug, Clone)]
pub struct AstContext {
    /// Module name for the generated code
    pub module_name: String,
    /// Whether to generate documentation
    pub generate_docs: bool,
    /// Whether to generate async methods
    pub generate_async: bool,
    /// Default traits to derive
    pub default_derives: Vec<String>,
}

impl Default for AstContext {
    fn default() -> Self {
        Self {
            module_name: "generated".to_string(),
            generate_docs: true,
            generate_async: false,
            default_derives: vec!["Debug".to_string(), "Clone".to_string()],
        }
    }
}

/// Error types specific to AST generation
#[derive(Debug, thiserror::Error)]
pub enum AstError {
    #[error("Invalid Rust identifier: {0}")]
    InvalidIdentifier(String),

    #[error("Unsupported type: {0}")]
    UnsupportedType(String),

    #[error("Syntax error: {0}")]
    SyntaxError(String),

    #[error("Missing required field: {0}")]
    MissingField(String),
}

/// Utilities for working with Rust identifiers and keywords
pub mod utils {
    use convert_case::{Case, Casing};
    use syn::Ident;

    /// List of Rust keywords that need to be escaped
    const RUST_KEYWORDS: &[&str] = &[
        "abstract", "alignof", "as", "become", "box", "break", "const", "continue", "crate", "do",
        "else", "enum", "extern", "false", "final", "fn", "for", "if", "impl", "in", "let", "loop",
        "macro", "match", "mod", "move", "mut", "offsetof", "override", "priv", "proc", "pub",
        "pure", "ref", "return", "self", "Self", "sizeof", "static", "struct", "super", "trait",
        "true", "type", "typeof", "unsafe", "unsized", "use", "virtual", "where", "while", "yield",
        "async", "await", "dyn", "try",
    ];

    /// Convert a string to a valid Rust identifier
    pub fn to_rust_ident(name: &str) -> Ident {
        let clean_name = name.to_case(Case::Snake);

        // Escape keywords by adding underscore suffix
        let escaped_name = if RUST_KEYWORDS.contains(&clean_name.as_str()) {
            format!("{clean_name}_")
        } else {
            clean_name
        };

        syn::parse_str(&escaped_name).expect("Failed to parse identifier")
    }

    /// Convert a string to a valid Rust type name (PascalCase)
    pub fn to_rust_type_name(name: &str) -> Ident {
        let clean_name = name.to_case(Case::Pascal);
        syn::parse_str(&clean_name).expect("Failed to parse type name")
    }

    /// Check if a string is a valid Rust identifier
    pub fn is_valid_ident(name: &str) -> bool {
        syn::parse_str::<Ident>(name).is_ok()
    }
}

#[cfg(test)]
mod tests {
    use super::{utils::*, *};

    #[test]
    fn test_rust_ident_conversion() {
        assert_eq!(to_rust_ident("test_name").to_string(), "test_name");
        assert_eq!(to_rust_ident("TestName").to_string(), "test_name");
        assert_eq!(to_rust_ident("async").to_string(), "async_");
        assert_eq!(to_rust_ident("self").to_string(), "self_");
    }

    #[test]
    fn test_rust_type_name_conversion() {
        assert_eq!(to_rust_type_name("test_name").to_string(), "TestName");
        assert_eq!(to_rust_type_name("testName").to_string(), "TestName");
        assert_eq!(to_rust_type_name("TESTNAME").to_string(), "Testname");
    }

    #[test]
    fn test_valid_ident_check() {
        assert!(is_valid_ident("test_name"));
        assert!(is_valid_ident("TestName"));
        assert!(!is_valid_ident("123invalid"));
        assert!(!is_valid_ident("test-name"));
    }

    #[test]
    fn test_ast_context_default() {
        let ctx = AstContext::default();
        assert_eq!(ctx.module_name, "generated");
        assert!(ctx.generate_docs);
        assert!(!ctx.generate_async);
        assert_eq!(ctx.default_derives.len(), 2);
    }
}
