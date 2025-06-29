//! Struct AST generation for CARLA types

use crate::{
    analyzer::type_resolver::RustType,
    ast::{AstBuilder, AstContext, AstError},
};
use syn::{
    parse_quote, Attribute, Field, Fields, FieldsNamed, Ident, ItemStruct, Result as SynResult,
    Type, Visibility,
};

/// Builder for generating Rust struct definitions
#[derive(Debug, Clone)]
pub struct StructBuilder {
    /// Struct name
    pub name: Ident,
    /// Documentation for the struct
    pub doc: Option<String>,
    /// Fields in the struct
    pub fields: Vec<StructField>,
    /// Traits to derive
    pub derives: Vec<String>,
    /// Visibility of the struct
    pub visibility: Visibility,
    /// Generation context
    pub context: AstContext,
}

/// Represents a field in a struct
#[derive(Debug, Clone)]
pub struct StructField {
    /// Field name
    pub name: Ident,
    /// Field type
    pub rust_type: RustType,
    /// Field documentation
    pub doc: Option<String>,
    /// Field visibility
    pub visibility: Visibility,
    /// Optional warning about the field
    pub warning: Option<String>,
    /// Units for numeric fields
    pub units: Option<String>,
}

impl StructBuilder {
    /// Create a new struct builder
    pub fn new(name: Ident, context: AstContext) -> Self {
        Self {
            name,
            doc: None,
            fields: Vec::new(),
            derives: context.default_derives.clone(),
            visibility: parse_quote!(pub),
            context,
        }
    }

    /// Set the documentation for the struct
    pub fn with_doc(mut self, doc: String) -> Self {
        self.doc = Some(doc);
        self
    }

    /// Add a field to the struct
    pub fn add_field(mut self, field: StructField) -> Self {
        self.fields.push(field);
        self
    }

    /// Set custom derives for the struct
    pub fn with_derives(mut self, derives: Vec<String>) -> Self {
        self.derives = derives;
        self
    }

    /// Set the visibility of the struct
    pub fn with_visibility(mut self, visibility: Visibility) -> Self {
        self.visibility = visibility;
        self
    }

    /// Generate documentation attributes
    fn generate_doc_attrs(&self) -> SynResult<Vec<Attribute>> {
        let mut attrs = Vec::new();

        if let Some(ref doc) = self.doc {
            // Split documentation into lines and create doc attributes
            for line in doc.lines() {
                let trimmed = line.trim();
                if !trimmed.is_empty() {
                    attrs.push(parse_quote!(#[doc = #trimmed]));
                }
            }
        }

        Ok(attrs)
    }

    /// Generate derive attributes
    fn generate_derive_attrs(&self) -> SynResult<Vec<Attribute>> {
        if self.derives.is_empty() {
            return Ok(vec![]);
        }

        let derive_list = self
            .derives
            .iter()
            .map(|d| {
                let ident: Ident = syn::parse_str(d)?;
                Ok(ident)
            })
            .collect::<SynResult<Vec<_>>>()?;

        Ok(vec![parse_quote!(#[derive(#(#derive_list),*)])])
    }

    /// Generate field definitions
    fn generate_fields(&self) -> SynResult<Fields> {
        let mut fields = Vec::new();

        for field in &self.fields {
            let name = &field.name;
            let ty = self.rust_type_to_syn_type(&field.rust_type)?;
            let vis = &field.visibility;

            // Generate field documentation
            let mut attrs: Vec<syn::Attribute> = Vec::new();
            if let Some(ref doc) = field.doc {
                attrs.push(parse_quote!(#[doc = #doc]));
            }

            if let Some(ref warning) = field.warning {
                attrs.push(parse_quote!(#[doc = ""]));
                attrs.push(parse_quote!(#[doc = "# Warning"]));
                attrs.push(parse_quote!(#[doc = #warning]));
            }

            if let Some(ref units) = field.units {
                attrs.push(parse_quote!(#[doc = ""]));
                let units_doc = format!("**Units**: {units}");
                attrs.push(parse_quote!(#[doc = #units_doc]));
            }

            let field: Field = parse_quote! {
                #(#attrs)*
                #vis #name: #ty
            };

            fields.push(field);
        }

        Ok(Fields::Named(FieldsNamed {
            brace_token: Default::default(),
            named: fields.into_iter().collect(),
        }))
    }

    /// Convert RustType to syn Type
    #[allow(clippy::only_used_in_recursion)]
    fn rust_type_to_syn_type(&self, rust_type: &RustType) -> SynResult<Type> {
        match rust_type {
            RustType::Primitive(name) => {
                let type_str = name.as_str();
                syn::parse_str(type_str)
            }
            RustType::Option(inner) => {
                let inner_type = self.rust_type_to_syn_type(inner)?;
                Ok(parse_quote!(Option<#inner_type>))
            }
            RustType::Vec(inner) => {
                let inner_type = self.rust_type_to_syn_type(inner)?;
                Ok(parse_quote!(Vec<#inner_type>))
            }
            RustType::HashMap(key, value) => {
                let key_type = self.rust_type_to_syn_type(key)?;
                let value_type = self.rust_type_to_syn_type(value)?;
                Ok(parse_quote!(std::collections::HashMap<#key_type, #value_type>))
            }
            RustType::Custom(path) => syn::parse_str(path),
            RustType::Reference(inner) => {
                let inner_type: Type = syn::parse_str(inner)?;
                Ok(parse_quote!(&#inner_type))
            }
            RustType::MutReference(inner) => {
                let inner_type: Type = syn::parse_str(inner)?;
                Ok(parse_quote!(&mut #inner_type))
            }
            RustType::Str => Ok(parse_quote!(&str)),
        }
    }
}

impl AstBuilder for StructBuilder {
    type Output = ItemStruct;

    fn build(&self) -> SynResult<Self::Output> {
        let name = &self.name;
        let vis = &self.visibility;

        // Generate all attributes
        let mut attrs = Vec::new();
        attrs.extend(self.generate_doc_attrs()?);
        attrs.extend(self.generate_derive_attrs()?);

        // Generate fields
        let fields = self.generate_fields()?;

        Ok(parse_quote! {
            #(#attrs)*
            #vis struct #name #fields
        })
    }

    fn validate(&self) -> crate::Result<()> {
        // Check that struct name is valid
        if self.name.to_string().is_empty() {
            return Err(AstError::MissingField("struct name".to_string()).into());
        }

        // Validate field names are unique
        let mut field_names = std::collections::HashSet::new();
        for field in &self.fields {
            let name = field.name.to_string();
            if !field_names.insert(name.clone()) {
                return Err(
                    AstError::InvalidIdentifier(format!("Duplicate field name: {name}")).into(),
                );
            }
        }

        Ok(())
    }
}

impl StructField {
    /// Create a new struct field
    pub fn new(name: Ident, rust_type: RustType) -> Self {
        Self {
            name,
            rust_type,
            doc: None,
            visibility: parse_quote!(pub),
            warning: None,
            units: None,
        }
    }

    /// Set the documentation for the field
    pub fn with_doc(mut self, doc: String) -> Self {
        self.doc = Some(doc);
        self
    }

    /// Set the visibility of the field
    pub fn with_visibility(mut self, visibility: Visibility) -> Self {
        self.visibility = visibility;
        self
    }

    /// Add a warning to the field
    pub fn with_warning(mut self, warning: String) -> Self {
        self.warning = Some(warning);
        self
    }

    /// Add units information to the field
    pub fn with_units(mut self, units: String) -> Self {
        self.units = Some(units);
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ast::utils::{to_rust_ident, to_rust_type_name};
    use quote::ToTokens;

    #[test]
    fn test_struct_builder_basic() {
        let context = AstContext::default();
        let name = to_rust_type_name("TestStruct");

        let builder = StructBuilder::new(name, context).with_doc("A test struct".to_string());

        let result = builder.build().expect("Failed to build struct");
        let tokens = result.to_token_stream().to_string();

        assert!(tokens.contains("struct TestStruct"));
        assert!(tokens.contains("doc = \"A test struct\""));
        assert!(tokens.contains("derive (Debug , Clone)"));
    }

    #[test]
    fn test_struct_with_fields() {
        let context = AstContext::default();
        let name = to_rust_type_name("Vector3D");

        let field1 = StructField::new(to_rust_ident("x"), RustType::Primitive("f32".to_string()))
            .with_doc("X coordinate".to_string())
            .with_units("meters".to_string());

        let field2 = StructField::new(to_rust_ident("y"), RustType::Primitive("f32".to_string()))
            .with_doc("Y coordinate".to_string());

        let builder = StructBuilder::new(name, context)
            .with_doc("3D vector".to_string())
            .add_field(field1)
            .add_field(field2);

        let result = builder.build().expect("Failed to build struct");
        let tokens = result.to_token_stream().to_string();

        assert!(tokens.contains("pub x : f32"));
        assert!(tokens.contains("pub y : f32"));
        assert!(tokens.contains("X coordinate"));
        assert!(tokens.contains("**Units**: meters"));
    }

    #[test]
    fn test_struct_validation() {
        let context = AstContext::default();
        let name = to_rust_type_name("TestStruct");

        // Test duplicate field names
        let field1 = StructField::new(
            to_rust_ident("test"),
            RustType::Primitive("i32".to_string()),
        );
        let field2 = StructField::new(
            to_rust_ident("test"), // Duplicate name
            RustType::Primitive("f32".to_string()),
        );

        let builder = StructBuilder::new(name, context)
            .add_field(field1)
            .add_field(field2);

        assert!(builder.validate().is_err());
    }

    #[test]
    fn test_complex_types() {
        let context = AstContext::default();
        let name = to_rust_type_name("ComplexStruct");

        let field1 = StructField::new(
            to_rust_ident("optional_value"),
            RustType::Option(Box::new(RustType::Primitive("i32".to_string()))),
        );

        let field2 = StructField::new(
            to_rust_ident("list_values"),
            RustType::Vec(Box::new(RustType::Primitive("f32".to_string()))),
        );

        let field3 = StructField::new(
            to_rust_ident("custom_type"),
            RustType::Custom("crate::geom::Location".to_string()),
        );

        let builder = StructBuilder::new(name, context)
            .add_field(field1)
            .add_field(field2)
            .add_field(field3);

        let result = builder.build().expect("Failed to build struct");
        let tokens = result.to_token_stream().to_string();

        assert!(tokens.contains("Option < i32 >"));
        assert!(tokens.contains("Vec < f32 >"));
        assert!(tokens.contains("crate :: geom :: Location"));
    }
}
