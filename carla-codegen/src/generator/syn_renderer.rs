//! Syn-based renderer for type-safe Rust code generation

use crate::{
    analyzer::{InheritanceResolver, TypeResolver},
    ast::{
        doc_builder::{generate_ffi_todo_doc, generate_method_doc, DocBuilder},
        formatter::{FormatConfig, RustFormatter},
        impl_builder::{ImplBuilder, MethodBuilder, MethodParameter},
        struct_builder::{StructBuilder, StructField},
        trait_builder::TraitBuilder,
        utils::{to_rust_ident, to_rust_type_name},
        AstBuilder, AstContext,
    },
    config::Config,
    generator::{
        context::{GeneratorContext, ImplData, ModuleData, StructData},
        renderer::TemplateRenderer,
    },
    parser::yaml_schema::{Class, Method, Module},
    Result,
};

use syn::{parse_quote, File, Item};
use tracing::debug;

/// Syn-based template renderer that generates type-safe Rust AST
#[derive(Debug)]
pub struct SynRenderer {
    /// Code formatter
    formatter: RustFormatter,
    /// AST generation context
    ast_context: AstContext,
}

impl SynRenderer {
    /// Create a new syn renderer
    pub fn new() -> Self {
        Self {
            formatter: RustFormatter::new(),
            ast_context: AstContext::default(),
        }
    }

    /// Create a new syn renderer with custom format configuration
    pub fn with_format_config(format_config: FormatConfig) -> Self {
        Self {
            formatter: RustFormatter::with_config(format_config),
            ast_context: AstContext::default(),
        }
    }

    /// Set AST generation context
    pub fn with_ast_context(mut self, context: AstContext) -> Self {
        self.ast_context = context;
        self
    }

    /// Generate a struct using AST builders
    #[allow(dead_code)]
    fn generate_struct_ast(&self, context: &GeneratorContext, class: &Class) -> Result<Item> {
        let struct_name = to_rust_type_name(&class.class_name);
        let mut builder = StructBuilder::new(struct_name, self.ast_context.clone());

        // Add documentation
        if let Some(ref doc) = class.doc {
            builder = builder.with_doc(doc.clone());
        }

        // Add fields
        for var in &class.instance_variables {
            // Skip variables without types (e.g., enum variants)
            let Some(ref var_type) = var.var_type else {
                continue;
            };

            let rust_type = context.type_resolver.resolve_type(var_type)?;
            let field_name = to_rust_ident(&var.var_name);

            let mut field =
                StructField::new(field_name, rust_type).with_visibility(parse_quote!(pub));

            if let Some(ref doc) = var.doc {
                field = field.with_doc(doc.clone());
            }

            if let Some(ref warning) = var.warning {
                field = field.with_warning(warning.clone());
            }

            if let Some(ref units) = var.var_units {
                field = field.with_units(units.clone());
            }

            builder = builder.add_field(field);
        }

        // Auto-generate trait derivation
        let trait_builder = TraitBuilder::new(
            to_rust_type_name(&class.class_name),
            self.ast_context.clone(),
        );

        // Add field types for analysis
        let mut trait_builder = trait_builder;
        for var in &class.instance_variables {
            if let Some(ref var_type) = var.var_type {
                if let Ok(rust_type) = context.type_resolver.resolve_type(var_type) {
                    trait_builder = trait_builder.add_field_type(rust_type);
                }
            }
        }

        // Apply automatic trait derivation
        let derives = trait_builder.analyze_derivable_traits();
        builder = builder.with_derives(derives);

        // Build the struct
        let struct_item = builder.build()?;
        Ok(Item::Struct(struct_item))
    }

    /// Generate implementation block using AST builders
    #[allow(dead_code)]
    fn generate_impl_ast(&self, context: &GeneratorContext, class: &Class) -> Result<Item> {
        let type_name = to_rust_type_name(&class.class_name);
        let mut builder = ImplBuilder::new(type_name, self.ast_context.clone());

        // Add methods
        for method in &class.methods {
            // Skip constructors for now
            if method.def_name == "__init__" {
                continue;
            }

            // Check if method should be generated
            if !context.config.should_generate_method(&method.def_name) {
                debug!("Skipping method: {}::{}", class.class_name, method.def_name);
                continue;
            }

            let method_ast = self.generate_method_ast(context, method, &class.class_name)?;
            builder = builder.add_method(method_ast);
        }

        // Build the impl block
        let impl_item = builder.build()?;
        Ok(Item::Impl(impl_item))
    }

    /// Generate a single method using AST builders
    #[allow(dead_code)]
    fn generate_method_ast(
        &self,
        context: &GeneratorContext,
        method: &Method,
        class_name: &str,
    ) -> Result<MethodBuilder> {
        let method_name = to_rust_ident(&context.rust_method_name(&method.def_name));
        let mut builder =
            MethodBuilder::new(method_name, class_name.to_string(), method.def_name.clone());

        // Add parameters
        for param in &method.params {
            // Skip 'self' parameter
            if param.param_name == "self" {
                continue;
            }

            let param_name = to_rust_ident(&param.param_name);
            let rust_type = if let Some(ref param_type) = param.param_type {
                context.type_resolver.resolve_type(param_type)?
            } else {
                // Default to String if no type specified
                crate::analyzer::type_resolver::RustType::Primitive("String".to_string())
            };

            let mut parameter = MethodParameter::new(param_name, rust_type);

            if let Some(ref doc) = param.doc {
                parameter = parameter.with_doc(doc.clone());
            }

            // Handle default values
            if let Some(ref default) = param.default {
                let default_str = match default {
                    serde_yaml::Value::String(s) => s.clone(),
                    serde_yaml::Value::Number(n) => n.to_string(),
                    serde_yaml::Value::Bool(b) => b.to_string(),
                    _ => format!("{:?}", default),
                };
                parameter = parameter.with_default(default_str);
            }

            builder = builder.add_parameter(parameter);
        }

        // Set return type
        if let Some(ref return_type) = method.return_type {
            let rust_return_type = context.type_resolver.resolve_type(return_type)?;
            builder = builder.with_return_type(rust_return_type);
        }

        // Generate documentation
        let param_docs: Vec<(&str, &str)> = method
            .params
            .iter()
            .filter(|p| p.param_name != "self")
            .map(|p| (p.param_name.as_str(), p.doc.as_deref().unwrap_or("")))
            .collect();

        let doc = generate_method_doc(
            method.doc.as_deref(),
            &param_docs,
            method.doc.as_deref(), // TODO: separate return documentation
            method.warning.as_deref(),
            method.note.as_deref(),
        );

        builder = builder.with_doc(doc);

        // Mark as async if configured
        if self.ast_context.generate_async {
            builder = builder.with_async(true);
        }

        Ok(builder)
    }

    /// Generate a complete module file
    #[allow(dead_code)]
    fn generate_module_file(&self, module: &Module, classes: &[Class]) -> Result<String> {
        let mut items = Vec::new();

        // Add module-level documentation
        let module_doc = crate::ast::doc_builder::generate_module_doc(
            &module.module_name,
            module.doc.as_deref(),
        );

        // Generate items for each class
        for class in classes {
            // Create a temporary context for this class
            let type_resolver = TypeResolver::new();
            let inheritance_resolver = InheritanceResolver::new();
            let config = Config::default();
            let context =
                GeneratorContext::new(module, &type_resolver, &inheritance_resolver, &config);

            // Generate struct
            let struct_item = self.generate_struct_ast(&context, class)?;
            items.push(struct_item);

            // Generate impl block if class has methods
            if !class.methods.is_empty() {
                let impl_item = self.generate_impl_ast(&context, class)?;
                items.push(impl_item);
            }
        }

        // Create the file
        let file = File {
            shebang: None,
            attrs: module_doc,
            items,
        };

        // Format and return
        self.formatter.format_file(&file).map_err(Into::into)
    }
}

impl TemplateRenderer for SynRenderer {
    fn render_struct(&self, data: &StructData) -> Result<String> {
        // Convert StructData to our AST builder format
        let struct_name = to_rust_type_name(&data.name);
        let mut builder = StructBuilder::new(struct_name, self.ast_context.clone());

        // Add documentation
        if let Some(ref doc) = data.doc {
            builder = builder.with_doc(doc.clone());
        }

        // Add fields
        for field in &data.fields {
            let field_name = to_rust_ident(&field.name);
            let rust_type =
                crate::analyzer::type_resolver::RustType::Custom(field.rust_type.clone());

            let mut struct_field =
                StructField::new(field_name, rust_type).with_visibility(parse_quote!(pub));

            if let Some(ref doc) = field.doc {
                struct_field = struct_field.with_doc(doc.clone());
            }

            if let Some(ref warning) = field.warning {
                struct_field = struct_field.with_warning(warning.clone());
            }

            if let Some(ref units) = field.units {
                struct_field = struct_field.with_units(units.clone());
            }

            builder = builder.add_field(struct_field);
        }

        // Build and format
        let struct_item = builder.build()?;
        self.formatter
            .format_item(&Item::Struct(struct_item))
            .map_err(Into::into)
    }

    fn render_impl(&self, data: &ImplData) -> Result<String> {
        let type_name = to_rust_type_name(&data.name);
        let mut builder = ImplBuilder::new(type_name, self.ast_context.clone());

        // Add methods
        for method in &data.methods {
            let method_name = to_rust_ident(&method.name);
            let mut method_builder =
                MethodBuilder::new(method_name, data.name.clone(), method.name.clone());

            // Add parameters
            for param in &method.params {
                let param_name = to_rust_ident(&param.name);
                let rust_type =
                    crate::analyzer::type_resolver::RustType::Custom(param.rust_type.clone());

                let mut parameter = MethodParameter::new(param_name, rust_type);

                if let Some(ref doc) = param.doc {
                    parameter = parameter.with_doc(doc.clone());
                }

                if let Some(ref default) = param.default {
                    parameter = parameter.with_default(default.clone());
                }

                method_builder = method_builder.add_parameter(parameter);
            }

            // Set return type if specified
            if !method.return_type.is_empty() && method.return_type != "()" {
                let rust_return_type =
                    crate::analyzer::type_resolver::RustType::Custom(method.return_type.clone());
                method_builder = method_builder.with_return_type(rust_return_type);
            }

            // Add documentation
            if let Some(ref doc) = method.doc {
                let doc_builder = DocBuilder::new(doc.clone());
                method_builder = method_builder.with_doc(doc_builder);
            } else {
                // Generate default FFI todo documentation
                let doc_builder = generate_ffi_todo_doc(&data.name, &method.name);
                method_builder = method_builder.with_doc(doc_builder);
            }

            builder = builder.add_method(method_builder);
        }

        // Build and format
        let impl_item = builder.build()?;
        self.formatter
            .format_item(&Item::Impl(impl_item))
            .map_err(Into::into)
    }

    fn render_module(&self, data: &ModuleData) -> Result<String> {
        let mut items = Vec::new();

        // Create module-level documentation
        let module_doc = if let Some(ref doc) = data.module_doc {
            crate::ast::doc_builder::generate_module_doc("module", Some(doc))
        } else {
            crate::ast::doc_builder::generate_module_doc("module", None)
        };

        // Add use statements as raw items
        for use_stmt in &data.use_statements {
            if let Ok(item) = syn::parse_str::<Item>(&format!("{};", use_stmt)) {
                items.push(item);
            }
        }

        // Add struct items (converted from rendered strings)
        for struct_info in &data.structs {
            // Parse the rendered struct back into an AST item
            if let Ok(item) = syn::parse_str::<Item>(&struct_info.rendered) {
                items.push(item);
            }

            // Add impl block if present
            if struct_info.has_impl {
                if let Ok(item) = syn::parse_str::<Item>(&struct_info.impl_rendered) {
                    items.push(item);
                }
            }
        }

        // Create the file
        let file = File {
            shebang: None,
            attrs: module_doc,
            items,
        };

        // Format and return
        self.formatter.format_file(&file).map_err(Into::into)
    }
}

impl Default for SynRenderer {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        analyzer::type_resolver::TypeResolver,
        config::Config,
        generator::context::RustField,
        parser::yaml_schema::{Class, InstanceVariable, Method, Parameter},
    };

    fn create_test_class() -> Class {
        Class {
            class_name: "TestActor".to_string(),
            parent: None,
            doc: Some("A test actor for unit testing".to_string()),
            instance_variables: vec![
                InstanceVariable {
                    var_name: "id".to_string(),
                    var_type: Some("int".to_string()),
                    doc: Some("Actor ID".to_string()),
                    var_units: None,
                    note: None,
                    warning: None,
                },
                InstanceVariable {
                    var_name: "location".to_string(),
                    var_type: Some("carla.Location".to_string()),
                    doc: Some("Actor location".to_string()),
                    var_units: Some("meters".to_string()),
                    note: None,
                    warning: Some("May be None if actor is destroyed".to_string()),
                },
            ],
            methods: vec![
                Method {
                    def_name: "get_id".to_string(),
                    return_type: Some("int".to_string()),
                    params: vec![Parameter {
                        param_name: "self".to_string(),
                        param_type: None,
                        default: None,
                        param_units: None,
                        doc: None,
                    }],
                    doc: Some("Gets the actor's unique identifier".to_string()),
                    warning: None,
                    note: None,
                    raises: None,
                    is_static: false,
                    return_units: None,
                },
                Method {
                    def_name: "set_location".to_string(),
                    return_type: None,
                    params: vec![
                        Parameter {
                            param_name: "self".to_string(),
                            param_type: None,
                            default: None,
                            param_units: None,
                            doc: None,
                        },
                        Parameter {
                            param_name: "location".to_string(),
                            param_type: Some("carla.Location".to_string()),
                            default: None,
                            param_units: Some("meters".to_string()),
                            doc: Some("New location for the actor".to_string()),
                        },
                    ],
                    doc: Some("Sets the actor's location".to_string()),
                    warning: Some("Actor must be alive".to_string()),
                    note: None,
                    raises: None,
                    is_static: false,
                    return_units: None,
                },
            ],
        }
    }

    #[test]
    fn test_generate_struct_ast() {
        let renderer = SynRenderer::new();
        let class = create_test_class();
        let module = crate::parser::yaml_schema::Module {
            module_name: "test".to_string(),
            doc: None,
            classes: vec![class.clone()],
        };

        let type_resolver = TypeResolver::new();
        let inheritance_resolver = InheritanceResolver::new();
        let config = Config::default();
        let context =
            GeneratorContext::new(&module, &type_resolver, &inheritance_resolver, &config);

        let result = renderer.generate_struct_ast(&context, &class);
        assert!(result.is_ok());

        if let Ok(Item::Struct(struct_item)) = result {
            assert_eq!(struct_item.ident.to_string(), "TestActor");
            assert!(!struct_item.fields.is_empty());
        } else {
            panic!("Expected struct item");
        }
    }

    #[test]
    fn test_generate_impl_ast() {
        let renderer = SynRenderer::new();
        let class = create_test_class();
        let module = crate::parser::yaml_schema::Module {
            module_name: "test".to_string(),
            doc: None,
            classes: vec![class.clone()],
        };

        let type_resolver = TypeResolver::new();
        let inheritance_resolver = InheritanceResolver::new();
        let config = Config::default();
        let context =
            GeneratorContext::new(&module, &type_resolver, &inheritance_resolver, &config);

        let result = renderer.generate_impl_ast(&context, &class);
        assert!(result.is_ok());

        if let Ok(Item::Impl(impl_item)) = result {
            assert!(!impl_item.items.is_empty());
        } else {
            panic!("Expected impl item");
        }
    }

    #[test]
    fn test_template_renderer_interface() {
        let renderer = SynRenderer::new();

        // Test struct rendering
        let struct_data = StructData {
            name: "TestStruct".to_string(),
            doc: Some("Test documentation".to_string()),
            fields: vec![RustField {
                name: "id".to_string(),
                rust_type: "u32".to_string(),
                doc: Some("ID field".to_string()),
                units: None,
                warning: None,
                note: None,
            }],
            has_ffi_inner: false,
            ffi_type: "TestStruct".to_string(),
        };

        let result = renderer.render_struct(&struct_data);
        assert!(result.is_ok());

        let rendered = result.unwrap();
        assert!(rendered.contains("struct TestStruct"));
        assert!(rendered.contains("pub id: u32"));
        assert!(rendered.contains("Test documentation"));
    }

    #[test]
    fn test_formatter_integration() {
        let renderer = SynRenderer::new();

        // Create a simple struct to format
        let struct_item: syn::ItemStruct = syn::parse_quote! {
            pub struct TestStruct {
                pub field: i32,
            }
        };

        let result = renderer
            .formatter
            .format_item(&syn::Item::Struct(struct_item));
        assert!(result.is_ok());

        let formatted = result.unwrap();
        assert!(formatted.contains("pub struct TestStruct"));
        assert!(formatted.contains("pub field: i32"));
    }
}
