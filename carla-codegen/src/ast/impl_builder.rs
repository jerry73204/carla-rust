//! Implementation block AST generation for CARLA types

use crate::{
    analyzer::type_resolver::RustType,
    ast::{doc_builder::DocBuilder, AstBuilder, AstContext, AstError},
};
use syn::{
    parse_quote, Block, Ident, ImplItem, ImplItemFn, ItemImpl, Result as SynResult, ReturnType,
    Signature, Type, Visibility,
};

/// Builder for generating Rust impl blocks
#[derive(Debug, Clone)]
pub struct ImplBuilder {
    /// Type being implemented
    pub target_type: Ident,
    /// Methods in the impl block
    pub methods: Vec<MethodBuilder>,
    /// Generation context
    pub context: AstContext,
}

/// Builder for individual methods within impl blocks
#[derive(Debug, Clone)]
pub struct MethodBuilder {
    /// Method name
    pub name: Ident,
    /// Method visibility
    pub visibility: Visibility,
    /// Method parameters
    pub parameters: Vec<MethodParameter>,
    /// Return type
    pub return_type: Option<RustType>,
    /// Method documentation
    pub doc: Option<DocBuilder>,
    /// Whether method is async
    pub is_async: bool,
    /// Whether method is unsafe
    pub is_unsafe: bool,
    /// CARLA class name for FFI generation
    pub carla_class: String,
    /// Original method name from Python API
    pub python_name: String,
}

/// Represents a method parameter
#[derive(Debug, Clone)]
pub struct MethodParameter {
    /// Parameter name
    pub name: Ident,
    /// Parameter type
    pub rust_type: RustType,
    /// Parameter documentation
    pub doc: Option<String>,
    /// Default value (for optional parameters)
    pub default: Option<String>,
    /// Whether parameter is mutable
    pub is_mut: bool,
}

impl ImplBuilder {
    /// Create a new impl builder
    pub fn new(target_type: Ident, context: AstContext) -> Self {
        Self {
            target_type,
            methods: Vec::new(),
            context,
        }
    }

    /// Add a method to the impl block
    pub fn add_method(mut self, method: MethodBuilder) -> Self {
        self.methods.push(method);
        self
    }

    /// Generate impl items from method builders
    fn generate_impl_items(&self) -> SynResult<Vec<ImplItem>> {
        let mut items = Vec::new();

        for method in &self.methods {
            let impl_item = method.build()?;
            items.push(ImplItem::Fn(impl_item));
        }

        Ok(items)
    }
}

impl AstBuilder for ImplBuilder {
    type Output = ItemImpl;

    fn build(&self) -> SynResult<Self::Output> {
        let target_type = &self.target_type;
        let items = self.generate_impl_items()?;

        Ok(parse_quote! {
            impl #target_type {
                #(#items)*
            }
        })
    }

    fn validate(&self) -> crate::Result<()> {
        // Check that type name is valid
        if self.target_type.to_string().is_empty() {
            return Err(AstError::MissingField("target type".to_string()).into());
        }

        // Validate method names are unique
        let mut method_names = std::collections::HashSet::new();
        for method in &self.methods {
            let name = method.name.to_string();
            if !method_names.insert(name.clone()) {
                return Err(
                    AstError::InvalidIdentifier(format!("Duplicate method name: {name}")).into(),
                );
            }
        }

        // Validate each method
        for method in &self.methods {
            method.validate()?;
        }

        Ok(())
    }
}

impl MethodBuilder {
    /// Create a new method builder
    pub fn new(name: Ident, carla_class: String, python_name: String) -> Self {
        Self {
            name,
            visibility: parse_quote!(pub),
            parameters: Vec::new(),
            return_type: None,
            doc: None,
            is_async: false,
            is_unsafe: false,
            carla_class,
            python_name,
        }
    }

    /// Set method visibility
    pub fn with_visibility(mut self, visibility: Visibility) -> Self {
        self.visibility = visibility;
        self
    }

    /// Add a parameter to the method
    pub fn add_parameter(mut self, parameter: MethodParameter) -> Self {
        self.parameters.push(parameter);
        self
    }

    /// Set the return type
    pub fn with_return_type(mut self, return_type: RustType) -> Self {
        self.return_type = Some(return_type);
        self
    }

    /// Set method documentation
    pub fn with_doc(mut self, doc: DocBuilder) -> Self {
        self.doc = Some(doc);
        self
    }

    /// Mark method as async
    pub fn with_async(mut self, is_async: bool) -> Self {
        self.is_async = is_async;
        self
    }

    /// Mark method as unsafe
    pub fn with_unsafe(mut self, is_unsafe: bool) -> Self {
        self.is_unsafe = is_unsafe;
        self
    }

    /// Build method signature
    fn build_signature(&self) -> SynResult<Signature> {
        let name = &self.name;
        let mut inputs: syn::punctuated::Punctuated<syn::FnArg, syn::Token![,]> =
            syn::punctuated::Punctuated::new();

        // Add self parameter if not static
        inputs.push(parse_quote!(self));

        // Add other parameters
        for param in &self.parameters {
            let param_name = &param.name;
            let param_type = self.rust_type_to_syn_type(&param.rust_type)?;

            let fn_arg = if param.is_mut {
                parse_quote!(mut #param_name: #param_type)
            } else {
                parse_quote!(#param_name: #param_type)
            };

            inputs.push(fn_arg);
        }

        // Build return type
        let output = match &self.return_type {
            Some(rust_type) => {
                let ty = self.rust_type_to_syn_type(rust_type)?;
                ReturnType::Type(parse_quote!(->), Box::new(parse_quote!(crate::Result<#ty>)))
            }
            None => ReturnType::Type(parse_quote!(->), Box::new(parse_quote!(crate::Result<()>))),
        };

        Ok(Signature {
            constness: None,
            asyncness: if self.is_async {
                Some(parse_quote!(async))
            } else {
                None
            },
            unsafety: if self.is_unsafe {
                Some(parse_quote!(unsafe))
            } else {
                None
            },
            abi: None,
            fn_token: parse_quote!(fn),
            ident: name.clone(),
            generics: Default::default(),
            paren_token: Default::default(),
            inputs,
            variadic: None,
            output,
        })
    }

    /// Build method body with FFI todo placeholder
    fn build_body(&self) -> SynResult<Block> {
        let class_name = &self.carla_class;
        let method_name = &self.python_name;
        let ffi_function_name = format!("{class_name}_{method_name}");

        let todo_message =
            format!("{method_name} not yet implemented - missing FFI function {ffi_function_name}");

        Ok(parse_quote! {
            {
                // TODO: Implement using carla-sys FFI interface
                // This requires adding #ffi_function_name FFI function
                todo!(#todo_message)
            }
        })
    }

    /// Convert RustType to syn Type (same as in struct_builder, should be shared)
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

    /// Validate method configuration
    pub fn validate(&self) -> crate::Result<()> {
        // Check method name is valid
        if self.name.to_string().is_empty() {
            return Err(AstError::MissingField("method name".to_string()).into());
        }

        // Check class name is provided
        if self.carla_class.is_empty() {
            return Err(AstError::MissingField("CARLA class name".to_string()).into());
        }

        // Validate parameter names are unique
        let mut param_names = std::collections::HashSet::new();
        for param in &self.parameters {
            let name = param.name.to_string();
            if !param_names.insert(name.clone()) {
                return Err(AstError::InvalidIdentifier(format!(
                    "Duplicate parameter name: {name}"
                ))
                .into());
            }
        }

        Ok(())
    }
}

impl AstBuilder for MethodBuilder {
    type Output = ImplItemFn;

    fn build(&self) -> SynResult<Self::Output> {
        let vis = &self.visibility;
        let sig = self.build_signature()?;
        let block = self.build_body()?;

        // Build documentation attributes
        let attrs = if let Some(ref doc) = self.doc {
            doc.build()
        } else {
            // Generate default FFI todo documentation
            let doc = crate::ast::doc_builder::generate_ffi_todo_doc(
                &self.carla_class,
                &self.python_name,
            );
            doc.build()
        };

        Ok(ImplItemFn {
            attrs,
            vis: vis.clone(),
            defaultness: None,
            sig,
            block,
        })
    }
}

impl MethodParameter {
    /// Create a new method parameter
    pub fn new(name: Ident, rust_type: RustType) -> Self {
        Self {
            name,
            rust_type,
            doc: None,
            default: None,
            is_mut: false,
        }
    }

    /// Set parameter documentation
    pub fn with_doc(mut self, doc: String) -> Self {
        self.doc = Some(doc);
        self
    }

    /// Set default value
    pub fn with_default(mut self, default: String) -> Self {
        self.default = Some(default);
        self
    }

    /// Mark parameter as mutable
    pub fn with_mut(mut self, is_mut: bool) -> Self {
        self.is_mut = is_mut;
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ast::{
        doc_builder,
        utils::{to_rust_ident, to_rust_type_name},
    };
    use quote::ToTokens;

    #[test]
    fn test_impl_builder_basic() {
        let context = AstContext::default();
        let type_name = to_rust_type_name("Actor");

        let method = MethodBuilder::new(
            to_rust_ident("get_id"),
            "Actor".to_string(),
            "GetId".to_string(),
        )
        .with_return_type(RustType::Primitive("u32".to_string()));

        let builder = ImplBuilder::new(type_name, context).add_method(method);

        let result = builder.build().expect("Failed to build impl");
        let tokens = result.to_token_stream().to_string();

        assert!(tokens.contains("impl Actor"));
        assert!(tokens.contains("pub fn get_id"));
        assert!(tokens.contains("crate :: Result < u32 >"));
        assert!(tokens.contains("todo !"));
    }

    #[test]
    fn test_method_with_parameters() {
        let param1 =
            MethodParameter::new(to_rust_ident("x"), RustType::Primitive("f32".to_string()))
                .with_doc("X coordinate".to_string());

        let param2 =
            MethodParameter::new(to_rust_ident("y"), RustType::Primitive("f32".to_string()))
                .with_doc("Y coordinate".to_string());

        let method = MethodBuilder::new(
            to_rust_ident("set_location"),
            "Actor".to_string(),
            "SetLocation".to_string(),
        )
        .add_parameter(param1)
        .add_parameter(param2);

        let result = method.build().expect("Failed to build method");
        let tokens = result.to_token_stream().to_string();

        assert!(tokens.contains("fn set_location"));
        assert!(tokens.contains("x : f32"));
        assert!(tokens.contains("y : f32"));
        assert!(tokens.contains("Actor_SetLocation"));
    }

    #[test]
    fn test_method_with_documentation() {
        let doc = doc_builder::generate_method_doc(
            Some("Gets the actor's unique identifier"),
            &[],
            Some("The actor's ID"),
            None,
            None,
        );

        let method = MethodBuilder::new(
            to_rust_ident("id"),
            "Actor".to_string(),
            "GetId".to_string(),
        )
        .with_return_type(RustType::Primitive("u32".to_string()))
        .with_doc(doc);

        let result = method.build().expect("Failed to build method");
        let tokens = result.to_token_stream().to_string();

        assert!(tokens.contains("Gets the actor's unique identifier"));
        assert!(tokens.contains("# Returns"));
        assert!(tokens.contains("The actor's ID"));
    }

    #[test]
    fn test_async_method() {
        let method = MethodBuilder::new(
            to_rust_ident("async_operation"),
            "Actor".to_string(),
            "AsyncOperation".to_string(),
        )
        .with_async(true)
        .with_return_type(RustType::Primitive("String".to_string()));

        let result = method.build().expect("Failed to build async method");
        let tokens = result.to_token_stream().to_string();

        assert!(tokens.contains("async fn async_operation"));
        assert!(tokens.contains("crate :: Result < String >"));
    }

    #[test]
    fn test_impl_validation() {
        let context = AstContext::default();
        let type_name = to_rust_type_name("TestType");

        // Test duplicate method names
        let method1 = MethodBuilder::new(
            to_rust_ident("test_method"),
            "Test".to_string(),
            "TestMethod".to_string(),
        );
        let method2 = MethodBuilder::new(
            to_rust_ident("test_method"), // Duplicate name
            "Test".to_string(),
            "TestMethod2".to_string(),
        );

        let builder = ImplBuilder::new(type_name, context)
            .add_method(method1)
            .add_method(method2);

        assert!(builder.validate().is_err());
    }

    #[test]
    fn test_method_validation() {
        // Test duplicate parameter names
        let param1 = MethodParameter::new(
            to_rust_ident("test"),
            RustType::Primitive("i32".to_string()),
        );
        let param2 = MethodParameter::new(
            to_rust_ident("test"), // Duplicate name
            RustType::Primitive("f32".to_string()),
        );

        let method = MethodBuilder::new(
            to_rust_ident("test_method"),
            "Test".to_string(),
            "TestMethod".to_string(),
        )
        .add_parameter(param1)
        .add_parameter(param2);

        assert!(method.validate().is_err());
    }

    #[test]
    fn test_complex_parameter_types() {
        let param1 = MethodParameter::new(
            to_rust_ident("optional_value"),
            RustType::Option(Box::new(RustType::Primitive("i32".to_string()))),
        );

        let param2 = MethodParameter::new(
            to_rust_ident("location"),
            RustType::Reference("crate::geom::Location".to_string()),
        );

        let method = MethodBuilder::new(
            to_rust_ident("complex_method"),
            "Actor".to_string(),
            "ComplexMethod".to_string(),
        )
        .add_parameter(param1)
        .add_parameter(param2);

        let result = method.build().expect("Failed to build method");
        let tokens = result.to_token_stream().to_string();

        assert!(tokens.contains("optional_value : Option < i32 >"));
        assert!(tokens.contains("location : & crate :: geom :: Location"));
    }
}
