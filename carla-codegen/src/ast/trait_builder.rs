//! Trait derivation and implementation for CARLA types

use crate::{
    analyzer::type_resolver::RustType,
    ast::{AstBuilder, AstContext},
};
use std::collections::HashSet;
use syn::{parse_quote, Attribute, Ident, Result as SynResult};

/// Builder for automatic trait derivation
#[derive(Debug, Clone)]
pub struct TraitBuilder {
    /// Target type for trait derivation
    pub target_type: Ident,
    /// Field types to analyze for derivation
    pub field_types: Vec<RustType>,
    /// Context for generation
    pub context: AstContext,
    /// Explicitly requested derives
    pub explicit_derives: Vec<String>,
    /// Derives to exclude
    pub exclude_derives: HashSet<String>,
}

impl TraitBuilder {
    /// Create a new trait builder
    pub fn new(target_type: Ident, context: AstContext) -> Self {
        Self {
            target_type,
            field_types: Vec::new(),
            context,
            explicit_derives: Vec::new(),
            exclude_derives: HashSet::new(),
        }
    }

    /// Add field type for analysis
    pub fn add_field_type(mut self, field_type: RustType) -> Self {
        self.field_types.push(field_type);
        self
    }

    /// Add explicit derive
    pub fn add_derive(mut self, derive: String) -> Self {
        self.explicit_derives.push(derive);
        self
    }

    /// Exclude a derive
    pub fn exclude_derive(mut self, derive: String) -> Self {
        self.exclude_derives.insert(derive);
        self
    }

    /// Determine which traits can be automatically derived
    pub fn analyze_derivable_traits(&self) -> Vec<String> {
        let mut derives = Vec::new();

        // Always include explicit derives first
        for derive in &self.explicit_derives {
            if !self.exclude_derives.contains(derive) {
                derives.push(derive.clone());
            }
        }

        // Add default derives from context
        for derive in &self.context.default_derives {
            if !derives.contains(derive) && !self.exclude_derives.contains(derive) {
                derives.push(derive.clone());
            }
        }

        // Analyze field types for automatic derivation
        if self.can_derive_debug() && !derives.contains(&"Debug".to_string()) {
            derives.push("Debug".to_string());
        }

        if self.can_derive_clone() && !derives.contains(&"Clone".to_string()) {
            derives.push("Clone".to_string());
        }

        if self.can_derive_copy() && !derives.contains(&"Copy".to_string()) {
            derives.push("Copy".to_string());
        }

        if self.can_derive_partial_eq() && !derives.contains(&"PartialEq".to_string()) {
            derives.push("PartialEq".to_string());
        }

        if self.can_derive_eq()
            && !derives.contains(&"Eq".to_string())
            && derives.contains(&"PartialEq".to_string())
        {
            derives.push("Eq".to_string());
        }

        if self.can_derive_hash() && !derives.contains(&"Hash".to_string()) {
            derives.push("Hash".to_string());
        }

        if self.can_derive_default() && !derives.contains(&"Default".to_string()) {
            derives.push("Default".to_string());
        }

        // Remove excluded derives
        derives.retain(|d| !self.exclude_derives.contains(d));

        derives
    }

    /// Check if Debug can be derived
    fn can_derive_debug(&self) -> bool {
        // Debug can be derived for most types, but not for function pointers or certain complex types
        for field_type in &self.field_types {
            if !self.type_supports_debug(field_type) {
                return false;
            }
        }
        true
    }

    /// Check if Clone can be derived
    fn can_derive_clone(&self) -> bool {
        // Clone can be derived if all fields support Clone
        for field_type in &self.field_types {
            if !self.type_supports_clone(field_type) {
                return false;
            }
        }
        true
    }

    /// Check if Copy can be derived
    fn can_derive_copy(&self) -> bool {
        // Copy can only be derived if all fields are Copy
        // This is more restrictive than Clone
        for field_type in &self.field_types {
            if !self.type_supports_copy(field_type) {
                return false;
            }
        }
        true
    }

    /// Check if PartialEq can be derived
    fn can_derive_partial_eq(&self) -> bool {
        // PartialEq can be derived if all fields support PartialEq
        for field_type in &self.field_types {
            if !self.type_supports_partial_eq(field_type) {
                return false;
            }
        }
        true
    }

    /// Check if Eq can be derived (requires PartialEq)
    fn can_derive_eq(&self) -> bool {
        // Eq can be derived if all fields support Eq
        // Note: floating point types support PartialEq but not Eq
        for field_type in &self.field_types {
            if !self.type_supports_eq(field_type) {
                return false;
            }
        }
        true
    }

    /// Check if Hash can be derived
    fn can_derive_hash(&self) -> bool {
        // Hash can be derived if all fields support Hash
        // Note: floating point types don't support Hash
        for field_type in &self.field_types {
            if !self.type_supports_hash(field_type) {
                return false;
            }
        }
        true
    }

    /// Check if Default can be derived
    fn can_derive_default(&self) -> bool {
        // Default can be derived if all fields support Default
        for field_type in &self.field_types {
            if !self.type_supports_default(field_type) {
                return false;
            }
        }
        true
    }

    /// Check if a type supports Debug
    #[allow(clippy::only_used_in_recursion)]
    fn type_supports_debug(&self, rust_type: &RustType) -> bool {
        match rust_type {
            RustType::Primitive(name) => {
                // Most primitives support Debug
                !matches!(name.as_str(), "fn" | "*const" | "*mut")
            }
            RustType::Option(inner) => self.type_supports_debug(inner),
            RustType::Vec(inner) => self.type_supports_debug(inner),
            RustType::HashMap(key, value) => {
                self.type_supports_debug(key) && self.type_supports_debug(value)
            }
            RustType::Custom(_) => true, // Assume custom types can derive Debug
            RustType::Reference(_) => true,
            RustType::MutReference(_) => true,
            RustType::Str => true,
            RustType::Union(types) => {
                // Union types support Debug if all component types do
                types.iter().all(|t| self.type_supports_debug(t))
            }
            RustType::Tuple(types) => {
                // Tuples support Debug if all element types do
                types.iter().all(|t| self.type_supports_debug(t))
            }
        }
    }

    /// Check if a type supports Clone
    #[allow(clippy::only_used_in_recursion)]
    fn type_supports_clone(&self, rust_type: &RustType) -> bool {
        match rust_type {
            RustType::Primitive(name) => {
                // Most primitives support Clone, but not function pointers
                !matches!(name.as_str(), "fn" | "*const" | "*mut")
            }
            RustType::Option(inner) => self.type_supports_clone(inner),
            RustType::Vec(inner) => self.type_supports_clone(inner),
            RustType::HashMap(key, value) => {
                self.type_supports_clone(key) && self.type_supports_clone(value)
            }
            RustType::Custom(_) => true, // Assume custom types can derive Clone
            RustType::Reference(_) => false, // References themselves are Copy, but we mean the owned version
            RustType::MutReference(_) => false,
            RustType::Str => false, // &str is Copy, but String is Clone
            RustType::Union(types) => {
                // Union types support Clone if all component types do
                types.iter().all(|t| self.type_supports_clone(t))
            }
            RustType::Tuple(types) => {
                // Tuples support Clone if all element types do
                types.iter().all(|t| self.type_supports_clone(t))
            }
        }
    }

    /// Check if a type supports Copy
    #[allow(clippy::only_used_in_recursion)]
    fn type_supports_copy(&self, rust_type: &RustType) -> bool {
        match rust_type {
            RustType::Primitive(name) => {
                // Only simple primitives support Copy
                matches!(
                    name.as_str(),
                    "i8" | "i16"
                        | "i32"
                        | "i64"
                        | "i128"
                        | "isize"
                        | "u8"
                        | "u16"
                        | "u32"
                        | "u64"
                        | "u128"
                        | "usize"
                        | "f32"
                        | "f64"
                        | "bool"
                        | "char"
                )
            }
            RustType::Option(inner) => self.type_supports_copy(inner),
            RustType::Vec(_) => false,        // Vec is not Copy
            RustType::HashMap(_, _) => false, // HashMap is not Copy
            RustType::Custom(_) => false,     // Assume custom types are not Copy by default
            RustType::Reference(_) => true,
            RustType::MutReference(_) => false, // Mutable references are not Copy
            RustType::Str => true,              // &str is Copy
            RustType::Union(_) => false,        // Union types can't be Copy
            RustType::Tuple(types) => {
                // Tuples can be Copy if all element types are Copy
                types.iter().all(|t| self.type_supports_copy(t))
            }
        }
    }

    /// Check if a type supports PartialEq
    #[allow(clippy::only_used_in_recursion)]
    fn type_supports_partial_eq(&self, rust_type: &RustType) -> bool {
        match rust_type {
            RustType::Primitive(name) => {
                // Most primitives support PartialEq
                !matches!(name.as_str(), "fn" | "*const" | "*mut")
            }
            RustType::Option(inner) => self.type_supports_partial_eq(inner),
            RustType::Vec(inner) => self.type_supports_partial_eq(inner),
            RustType::HashMap(key, value) => {
                self.type_supports_partial_eq(key) && self.type_supports_partial_eq(value)
            }
            RustType::Custom(_) => true, // Assume custom types can derive PartialEq
            RustType::Reference(_) => true,
            RustType::MutReference(_) => true,
            RustType::Str => true,
            RustType::Union(types) => {
                // Union types support PartialEq if all component types do
                types.iter().all(|t| self.type_supports_partial_eq(t))
            }
            RustType::Tuple(types) => {
                // Tuples support PartialEq if all element types do
                types.iter().all(|t| self.type_supports_partial_eq(t))
            }
        }
    }

    /// Check if a type supports Eq (stricter than PartialEq)
    #[allow(clippy::only_used_in_recursion)]
    fn type_supports_eq(&self, rust_type: &RustType) -> bool {
        match rust_type {
            RustType::Primitive(name) => {
                // Floating point types don't support Eq due to NaN
                !matches!(name.as_str(), "f32" | "f64" | "fn" | "*const" | "*mut")
            }
            RustType::Option(inner) => self.type_supports_eq(inner),
            RustType::Vec(inner) => self.type_supports_eq(inner),
            RustType::HashMap(key, value) => {
                self.type_supports_eq(key) && self.type_supports_eq(value)
            }
            RustType::Custom(_) => true, // Assume custom types can derive Eq if they have PartialEq
            RustType::Reference(_) => true,
            RustType::MutReference(_) => true,
            RustType::Str => true,
            RustType::Union(types) => {
                // Union types support Eq if all component types do
                types.iter().all(|t| self.type_supports_eq(t))
            }
            RustType::Tuple(types) => {
                // Tuples support Eq if all element types do
                types.iter().all(|t| self.type_supports_eq(t))
            }
        }
    }

    /// Check if a type supports Hash
    #[allow(clippy::only_used_in_recursion)]
    fn type_supports_hash(&self, rust_type: &RustType) -> bool {
        match rust_type {
            RustType::Primitive(name) => {
                // Floating point types don't support Hash due to NaN
                !matches!(name.as_str(), "f32" | "f64" | "fn" | "*const" | "*mut")
            }
            RustType::Option(inner) => self.type_supports_hash(inner),
            RustType::Vec(inner) => self.type_supports_hash(inner),
            RustType::HashMap(_, _) => false, // HashMap itself doesn't implement Hash
            RustType::Custom(_) => true,      // Assume custom types can derive Hash
            RustType::Reference(_) => true,
            RustType::MutReference(_) => false, // Mutable references don't implement Hash
            RustType::Str => true,
            RustType::Union(types) => {
                // Union types support Hash if all component types do
                types.iter().all(|t| self.type_supports_hash(t))
            }
            RustType::Tuple(types) => {
                // Tuples support Hash if all element types do
                types.iter().all(|t| self.type_supports_hash(t))
            }
        }
    }

    /// Check if a type supports Default
    fn type_supports_default(&self, rust_type: &RustType) -> bool {
        match rust_type {
            RustType::Primitive(name) => {
                // Most primitives support Default
                !matches!(name.as_str(), "fn" | "*const" | "*mut")
            }
            RustType::Option(_) => true, // Option<T> always has Default (None)
            RustType::Vec(_) => true,    // Vec<T> always has Default (empty vec)
            RustType::HashMap(_, _) => true, // HashMap<K,V> always has Default (empty map)
            RustType::Custom(_) => false, // Custom types might not have Default
            RustType::Reference(_) => false, // References can't have Default
            RustType::MutReference(_) => false,
            RustType::Str => false,      // &str can't have Default
            RustType::Union(_) => false, // Union types can't have Default
            RustType::Tuple(_) => false, // Tuples don't have Default
        }
    }

    /// Generate derive attribute
    pub fn generate_derive_attribute(&self) -> Option<Attribute> {
        let derives = self.analyze_derivable_traits();

        if derives.is_empty() {
            return None;
        }

        let derive_idents: Result<Vec<Ident>, _> =
            derives.iter().map(|d| syn::parse_str(d)).collect();

        match derive_idents {
            Ok(idents) => Some(parse_quote!(#[derive(#(#idents),*)])),
            Err(_) => None,
        }
    }
}

impl AstBuilder for TraitBuilder {
    type Output = Attribute;

    fn build(&self) -> SynResult<Self::Output> {
        self.generate_derive_attribute()
            .ok_or_else(|| syn::Error::new_spanned(&self.target_type, "No derivable traits found"))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ast::utils::to_rust_type_name;
    use quote::ToTokens;

    #[test]
    fn test_basic_derives() {
        let context = AstContext::default();
        let type_name = to_rust_type_name("SimpleStruct");

        let builder = TraitBuilder::new(type_name, context)
            .add_field_type(RustType::Primitive("i32".to_string()))
            .add_field_type(RustType::Primitive("f32".to_string()));

        let derives = builder.analyze_derivable_traits();

        // Should have Debug and Clone from default context
        assert!(derives.contains(&"Debug".to_string()));
        assert!(derives.contains(&"Clone".to_string()));

        // Should have PartialEq but not Eq (due to f32)
        assert!(derives.contains(&"PartialEq".to_string()));
        assert!(!derives.contains(&"Eq".to_string()));

        // Should not have Hash (due to f32)
        assert!(!derives.contains(&"Hash".to_string()));
    }

    #[test]
    fn test_integer_only_struct() {
        let context = AstContext::default();
        let type_name = to_rust_type_name("IntegerStruct");

        let builder = TraitBuilder::new(type_name, context)
            .add_field_type(RustType::Primitive("i32".to_string()))
            .add_field_type(RustType::Primitive("u64".to_string()));

        let derives = builder.analyze_derivable_traits();

        // Should have all basic traits for integers
        assert!(derives.contains(&"Debug".to_string()));
        assert!(derives.contains(&"Clone".to_string()));
        assert!(derives.contains(&"PartialEq".to_string()));
        assert!(derives.contains(&"Eq".to_string()));
        assert!(derives.contains(&"Hash".to_string()));
    }

    #[test]
    fn test_copy_derivation() {
        let context = AstContext::default();
        let type_name = to_rust_type_name("CopyStruct");

        let builder = TraitBuilder::new(type_name, context)
            .add_field_type(RustType::Primitive("i32".to_string()))
            .add_field_type(RustType::Primitive("bool".to_string()));

        let derives = builder.analyze_derivable_traits();

        // Should be able to derive Copy for simple primitives
        assert!(derives.contains(&"Copy".to_string()));
    }

    #[test]
    fn test_vec_field_no_copy() {
        let context = AstContext::default();
        let type_name = to_rust_type_name("VecStruct");

        let builder = TraitBuilder::new(type_name, context).add_field_type(RustType::Vec(
            Box::new(RustType::Primitive("i32".to_string())),
        ));

        let derives = builder.analyze_derivable_traits();

        // Should not be able to derive Copy (Vec is not Copy)
        assert!(!derives.contains(&"Copy".to_string()));

        // But should still have Clone, Debug, etc.
        assert!(derives.contains(&"Clone".to_string()));
        assert!(derives.contains(&"Debug".to_string()));
    }

    #[test]
    fn test_explicit_derives() {
        let context = AstContext::default();
        let type_name = to_rust_type_name("ExplicitStruct");

        let builder = TraitBuilder::new(type_name, context)
            .add_derive("Serialize".to_string())
            .add_derive("Deserialize".to_string())
            .add_field_type(RustType::Primitive("String".to_string()));

        let derives = builder.analyze_derivable_traits();

        // Should include explicit derives
        assert!(derives.contains(&"Serialize".to_string()));
        assert!(derives.contains(&"Deserialize".to_string()));

        // Should also include default and analyzed derives
        assert!(derives.contains(&"Debug".to_string()));
        assert!(derives.contains(&"Clone".to_string()));
    }

    #[test]
    fn test_exclude_derives() {
        let context = AstContext::default();
        let type_name = to_rust_type_name("ExcludeStruct");

        let builder = TraitBuilder::new(type_name, context)
            .add_field_type(RustType::Primitive("i32".to_string()))
            .exclude_derive("Clone".to_string());

        let derives = builder.analyze_derivable_traits();

        // Should not include excluded derive
        assert!(!derives.contains(&"Clone".to_string()));

        // Should still include others
        assert!(derives.contains(&"Debug".to_string()));
    }

    #[test]
    fn test_derive_attribute_generation() {
        let context = AstContext::default();
        let type_name = to_rust_type_name("TestStruct");

        let builder = TraitBuilder::new(type_name, context)
            .add_field_type(RustType::Primitive("i32".to_string()));

        let attr = builder
            .generate_derive_attribute()
            .expect("Should generate derive");
        let tokens = attr.to_token_stream().to_string();

        assert!(tokens.contains("derive ("));
        assert!(tokens.contains("Debug"));
        assert!(tokens.contains("Clone"));
    }

    #[test]
    fn test_option_and_result_types() {
        let context = AstContext::default();
        let type_name = to_rust_type_name("OptionStruct");

        let builder = TraitBuilder::new(type_name, context)
            .add_field_type(RustType::Option(Box::new(RustType::Primitive(
                "i32".to_string(),
            ))))
            .add_field_type(RustType::Option(Box::new(RustType::Primitive(
                "f32".to_string(),
            ))));

        let derives = builder.analyze_derivable_traits();

        // Option<i32> and Option<f32> should allow most derives
        assert!(derives.contains(&"Debug".to_string()));
        assert!(derives.contains(&"Clone".to_string()));
        assert!(derives.contains(&"PartialEq".to_string()));

        // But f32 should prevent Eq and Hash
        assert!(!derives.contains(&"Eq".to_string()));
        assert!(!derives.contains(&"Hash".to_string()));
    }

    #[test]
    fn test_tuple_derive_analysis() {
        let context = AstContext::default();
        let type_name = to_rust_type_name("TupleStruct");

        // Test with tuple of Copy types
        let builder = TraitBuilder::new(type_name.clone(), context.clone()).add_field_type(
            RustType::Tuple(vec![
                RustType::Primitive("i32".to_string()),
                RustType::Primitive("u32".to_string()),
                RustType::Primitive("bool".to_string()),
            ]),
        );

        let derives = builder.analyze_derivable_traits();

        // Should be able to derive Copy for simple primitives tuple
        assert!(derives.contains(&"Copy".to_string()));
        assert!(derives.contains(&"Clone".to_string()));
        assert!(derives.contains(&"Debug".to_string()));
        assert!(derives.contains(&"PartialEq".to_string()));
        assert!(derives.contains(&"Eq".to_string()));
        assert!(derives.contains(&"Hash".to_string()));

        // Test with tuple containing non-Copy type
        let builder2 = TraitBuilder::new(type_name, context).add_field_type(RustType::Tuple(vec![
            RustType::Primitive("i32".to_string()),
            RustType::Vec(Box::new(RustType::Primitive("u8".to_string()))),
        ]));

        let derives2 = builder2.analyze_derivable_traits();

        // Should not be able to derive Copy (Vec is not Copy)
        assert!(!derives2.contains(&"Copy".to_string()));
        // But should still have Clone, Debug, etc.
        assert!(derives2.contains(&"Clone".to_string()));
        assert!(derives2.contains(&"Debug".to_string()));
        assert!(derives2.contains(&"PartialEq".to_string()));
    }
}
