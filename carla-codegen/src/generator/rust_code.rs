//! Rust code generation

use crate::{
    error::Result,
    generator::{
        context::{GeneratorContext, ImplData, ModuleData, RustField, StructData, StructInfo},
        renderer::TemplateRenderer,
    },
    parser::yaml_schema::Class,
};
use convert_case::{Case, Casing};
use std::path::Path;
use tracing::debug;

/// Rust code generator
pub struct RustCodeGenerator<'a, R: TemplateRenderer> {
    context: &'a GeneratorContext<'a>,
    renderer: &'a R,
}

impl<'a, R: TemplateRenderer> RustCodeGenerator<'a, R> {
    /// Create a new Rust code generator
    pub fn new(context: &'a GeneratorContext<'a>, renderer: &'a R) -> Self {
        Self { context, renderer }
    }

    /// Generate code for a module
    pub fn generate_module(&self, output_dir: &Path) -> Result<()> {
        let module_name = self.context.module.module_name.replace("carla.", "");

        if module_name.is_empty() || module_name == "carla" {
            // Generate classes in the root module
            for class in &self.context.module.classes {
                if self.context.config.should_generate_class(&class.class_name) {
                    self.generate_class_file(class, output_dir)?;
                }
            }
        } else {
            // Create subdirectory for module
            let module_dir = output_dir.join(&module_name);
            std::fs::create_dir_all(&module_dir)?;

            // Generate module file
            self.generate_module_file(&module_dir)?;

            // Generate class files
            for class in &self.context.module.classes {
                if self.context.config.should_generate_class(&class.class_name) {
                    self.generate_class_file(class, &module_dir)?;
                }
            }
        }

        Ok(())
    }

    /// Generate a module file
    fn generate_module_file(&self, module_dir: &Path) -> Result<()> {
        let mut structs = Vec::new();

        // Generate all classes
        for class in &self.context.module.classes {
            // Check if class should be generated
            if !self.context.config.should_generate_class(&class.class_name) {
                debug!("Skipping class: {}", class.class_name);
                continue;
            }

            let struct_data = self.generate_struct_data(class)?;
            let impl_data = self.generate_impl_data(class)?;

            let struct_rendered = self.renderer.render_struct(&struct_data)?;
            let impl_rendered = self.renderer.render_impl(&impl_data)?;

            structs.push(StructInfo {
                rendered: struct_rendered,
                impl_rendered,
                has_impl: !impl_data.methods.is_empty(),
            });
        }

        let module_data = ModuleData {
            module_doc: self.context.module.doc.clone(),
            use_statements: vec!["use crate::error::Result".to_string()],
            structs,
        };

        let content = self.renderer.render_module(&module_data)?;

        // Write to mod.rs
        std::fs::write(module_dir.join("mod.rs"), content)?;

        Ok(())
    }

    /// Generate a single class file
    fn generate_class_file(&self, class: &Class, output_dir: &Path) -> Result<()> {
        let class_snake = class.class_name.to_case(Case::Snake);
        let filename = format!("{class_snake}.rs");
        let filepath = output_dir.join(filename);

        debug!("Generating class file: {}", filepath.display());

        // Generate struct
        let struct_data = self.generate_struct_data(class)?;
        let struct_code = self.renderer.render_struct(&struct_data)?;

        // Generate impl block
        let impl_data = self.generate_impl_data(class)?;
        let impl_code = if impl_data.methods.is_empty() {
            String::new()
        } else {
            let impl_rendered = self.renderer.render_impl(&impl_data)?;
            format!("\n\n{impl_rendered}")
        };

        // Combine code
        let mut content = String::from("//! ");
        content.push_str(&self.context.rust_class_name(&class.class_name));
        content.push_str(" type\n\n");
        content.push_str("use crate::error::Result;\n\n");
        content.push_str(&struct_code);
        content.push_str(&impl_code);

        // Format with rustfmt
        let formatted = self.format_code(&content)?;

        std::fs::write(filepath, formatted)?;
        Ok(())
    }

    /// Generate struct data for templates
    fn generate_struct_data(&self, class: &Class) -> Result<StructData> {
        let rust_name = self.context.rust_class_name(&class.class_name);

        // Convert instance variables to fields
        let mut fields = Vec::new();
        for var in &class.instance_variables {
            // Skip variables without types (e.g., enum variants)
            let Some(ref var_type) = var.var_type else {
                continue;
            };

            let rust_type = self.context.type_resolver.resolve_type(var_type)?;

            fields.push(RustField {
                name: var.var_name.to_case(Case::Snake),
                rust_type: rust_type.to_rust_string(),
                doc: var.doc.clone(),
                units: var.var_units.clone(),
                warning: var.warning.clone(),
                note: var.note.clone(),
            });
        }

        // Determine if this is an FFI wrapper type
        let has_ffi_inner = !fields.is_empty() || !class.methods.is_empty();
        let ffi_type = if has_ffi_inner {
            format!("SharedPtr<ffi::{rust_name}>")
        } else {
            String::new()
        };

        Ok(StructData {
            name: rust_name,
            doc: class.doc.clone(),
            fields,
            has_ffi_inner,
            ffi_type,
        })
    }

    /// Generate impl data for templates
    fn generate_impl_data(&self, class: &Class) -> Result<ImplData> {
        let rust_name = self.context.rust_class_name(&class.class_name);

        // Convert methods
        let mut methods = Vec::new();
        for method in &class.methods {
            // Skip constructors for now
            if method.def_name == "__init__" {
                continue;
            }

            // Check if method should be generated
            if !self.context.config.should_generate_method(&method.def_name) {
                debug!("Skipping method: {}::{}", rust_name, method.def_name);
                continue;
            }

            let rust_method = self.context.rust_method(method, &rust_name)?;
            methods.push(rust_method);
        }

        Ok(ImplData {
            name: rust_name,
            methods,
        })
    }

    /// Format code using rustfmt
    fn format_code(&self, code: &str) -> Result<String> {
        // For now, just return the code as-is
        // In a real implementation, we would call rustfmt
        Ok(code.to_string())
    }
}
