//! Rust code generation

use crate::{
    analyzer::{
        is_enum_class, type_resolver::RustType, ResolvedType, SpecialMethodAnalyzer, TypeContext,
        TypeLocation, TypeResolverV2,
    },
    error::{CodegenError, Result},
    generator::{
        context::{GeneratorContext, ImplData, RustField, StructData},
        import_collector::ImportCollector,
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

    /// Check if a type is recursive and wrap in Box if needed
    fn fix_recursive_type(
        &self,
        rust_type: &RustType,
        current_class: &str,
        field_name: &str,
    ) -> RustType {
        match rust_type {
            RustType::Custom(type_name) => {
                // Check if the type references the current class
                if type_name.ends_with(&format!("::{current_class}")) {
                    debug!(
                        "Found recursive type in {}.{}: {}",
                        current_class, field_name, type_name
                    );
                    // Wrap in Option<Box<T>> for optional recursive references
                    if field_name == "parent" {
                        RustType::Option(Box::new(RustType::Custom(format!("Box<{type_name}>"))))
                    } else {
                        RustType::Custom(format!("Box<{type_name}>"))
                    }
                } else {
                    rust_type.clone()
                }
            }
            RustType::Vec(inner) => {
                // Check if vector contains recursive type
                let fixed_inner = self.fix_recursive_type(inner, current_class, field_name);
                RustType::Vec(Box::new(fixed_inner))
            }
            RustType::Option(inner) => {
                // Check if option contains recursive type
                let fixed_inner = self.fix_recursive_type(inner, current_class, field_name);
                RustType::Option(Box::new(fixed_inner))
            }
            _ => rust_type.clone(),
        }
    }

    /// Convert Python enum variant name to Rust convention
    fn to_rust_enum_variant(&self, python_name: &str) -> String {
        match python_name {
            // Special cases that need manual mapping
            "FL_Wheel" => "FlWheel".to_string(),
            "FR_Wheel" => "FrWheel".to_string(),
            "BL_Wheel" => "BlWheel".to_string(),
            "BR_Wheel" => "BrWheel".to_string(),
            "Front_Wheel" => "FrontWheel".to_string(),
            "Back_Wheel" => "BackWheel".to_string(),
            "AO_Roughness_Metallic_Emissive" => "AoRoughnessMetallicEmissive".to_string(),

            // General conversion rules
            name => {
                // Handle snake_case and mixed case
                if name.contains('_') {
                    // Split by underscore and capitalize each part
                    name.split('_')
                        .map(|part| {
                            let mut chars = part.chars();
                            match chars.next() {
                                None => String::new(),
                                Some(first) => {
                                    first.to_uppercase().collect::<String>()
                                        + chars.as_str().to_lowercase().as_str()
                                }
                            }
                        })
                        .collect::<String>()
                } else {
                    // Already in correct format or needs first letter capitalized
                    let mut chars = name.chars();
                    match chars.next() {
                        None => String::new(),
                        Some(first) => first.to_uppercase().collect::<String>() + chars.as_str(),
                    }
                }
            }
        }
    }

    /// Check if a type is copyable
    fn is_copyable_type(&self, rust_type_str: &str) -> bool {
        // Types that are NOT Copy
        if rust_type_str.contains("String")
            || rust_type_str.contains("Vec<")
            || rust_type_str.contains("HashMap<")
            || rust_type_str.contains("Box<")
            || rust_type_str.contains("dyn ")
            || rust_type_str == "serde_json::Value"
        {
            return false;
        }

        // Primitive types and simple types are Copy
        matches!(
            rust_type_str,
            "i8" | "i16"
                | "i32"
                | "i64"
                | "i128"
                | "u8"
                | "u16"
                | "u32"
                | "u64"
                | "u128"
                | "f32"
                | "f64"
                | "bool"
                | "char"
                | "&str"
        ) || rust_type_str.starts_with("Option<") && {
            // Option<T> is Copy if T is Copy
            let inner = &rust_type_str[7..rust_type_str.len() - 1];
            self.is_copyable_type(inner)
        }
    }

    /// Check if all fields in a struct are copyable
    fn can_derive_copy(&self, fields: &[RustField]) -> bool {
        fields.iter().all(|f| self.is_copyable_type(&f.rust_type))
    }

    /// Collect imports needed for the given resolved types
    fn collect_imports_from_resolved(&self, resolved_types: &[ResolvedType]) -> Vec<String> {
        let mut collector = ImportCollector::new();

        // Add imports from all resolved types
        for resolved in resolved_types {
            eprintln!("DEBUG: Processing resolved type: {:?}", resolved.rust_type);
            collector.add_from_resolved(resolved);
        }

        // Always add error type import
        collector.add_import("use crate::error::Result;");

        collector.get_organized_imports()
    }

    /// Collect imports needed for the given type strings (legacy)
    fn collect_imports(&self, types: &[String]) -> Vec<String> {
        let mut collector = ImportCollector::new();

        for type_str in types {
            // Debug print to see what types we're processing
            if type_str.contains("HashMap") {
                eprintln!("DEBUG: Found HashMap in type: {type_str}");
            }

            // Parse type string to detect imports
            if type_str.contains("HashMap") {
                collector.add_import("use std::collections::HashMap;");
            }
            if type_str.contains("HashSet") {
                collector.add_import("use std::collections::HashSet;");
            }
            if type_str.contains("BTreeMap") {
                collector.add_import("use std::collections::BTreeMap;");
            }
            if type_str.contains("BTreeSet") {
                collector.add_import("use std::collections::BTreeSet;");
            }
            if type_str.contains("Arc<") {
                collector.add_import("use std::sync::Arc;");
            }
            if type_str.contains("Mutex<") {
                collector.add_import("use std::sync::Mutex;");
            }
            if type_str.contains("serde_json::") {
                collector.add_import("use serde_json;");
            }
            // Check for unqualified type names that need imports
            if type_str == "Actor" || (type_str.contains("<Actor>") && !type_str.contains("::")) {
                collector.add_import("use super::Actor;");
            }
            if type_str == "World" || (type_str.contains("<World>") && !type_str.contains("::")) {
                collector.add_import("use super::World;");
            }
            if type_str == "WeatherParameters" {
                collector.add_import("use super::WeatherParameters;");
            }
        }

        // Add error type import if we have any methods
        collector.add_import("use crate::error::Result;");

        collector.get_organized_imports()
    }

    /// Generate code for a module
    pub fn generate_module(&self, output_dir: &Path) -> Result<()> {
        // Create directory structure based on module_name
        // For "carla", create output/carla/
        // For "carla.actor", create output/carla/actor/
        let module_path = self.context.module.module_name.replace('.', "/");
        let module_dir = output_dir.join(&module_path);
        std::fs::create_dir_all(&module_dir)?;

        // Generate module file
        eprintln!(
            "Generating module file for: {}",
            self.context.module.module_name
        );
        match self.generate_module_file(&module_dir) {
            Ok(()) => {}
            Err(e) => {
                eprintln!("Error generating module file: {e}");
                return Err(e);
            }
        }

        // Generate class files in the module directory
        for class in &self.context.module.classes {
            if self.context.config.should_generate_class(&class.class_name) {
                eprintln!("Generating class file for: {}", class.class_name);
                match self.generate_class_file(class, &module_dir) {
                    Ok(()) => {}
                    Err(e) => {
                        eprintln!("Error generating class {}: {}", class.class_name, e);
                        // Print class details for debugging
                        eprintln!("Failed class details:");
                        eprintln!("  - Name: {}", class.class_name);
                        eprintln!(
                            "  - Instance variables: {} vars",
                            class.instance_variables.len()
                        );
                        eprintln!("  - Methods: {} methods", class.methods.len());
                        if class.class_name == "World" {
                            eprintln!("  World class methods:");
                            for method in &class.methods {
                                eprintln!(
                                    "    - {}: return_type={:?}",
                                    method.def_name, method.return_type
                                );
                            }
                        }
                        return Err(e);
                    }
                }
            }
        }

        Ok(())
    }

    /// Generate a module file
    fn generate_module_file(&self, module_dir: &Path) -> Result<()> {
        let mut module_content = String::new();

        // Add module documentation
        if let Some(doc) = &self.context.module.doc {
            module_content.push_str("//! ");
            module_content.push_str(&doc.replace('\n', "\n//! "));
            module_content.push_str("\n\n");
        }

        // Generate exports for all classes
        let mut exports = Vec::new();
        for class in &self.context.module.classes {
            // Check if class should be generated
            if !self.context.config.should_generate_class(&class.class_name) {
                debug!("Skipping class: {}", class.class_name);
                continue;
            }

            let clean_name = class
                .class_name
                .strip_prefix("carla.")
                .unwrap_or(&class.class_name);
            let class_snake = clean_name.to_case(Case::Snake);
            let rust_name = self.context.rust_class_name(&class.class_name);

            // Add export
            exports.push(format!("mod {class_snake};"));
            exports.push(format!("pub use {class_snake}::{rust_name};"));
        }

        if exports.is_empty() {
            module_content.push_str("// No classes exported from this module\n");
        } else {
            module_content.push_str(&exports.join("\n"));
            module_content.push('\n');
        }

        // Add missing type definitions for the carla module
        if self.context.module.module_name == "carla" {
            // Write missing types to a separate file
            let missing_types_content = crate::generator::missing_types::generate_missing_types();
            let missing_types_file = module_dir.join("missing_types.rs");
            std::fs::write(&missing_types_file, missing_types_content)?;

            // Add to module exports
            module_content.push_str("\n// Additional type definitions\n");
            module_content.push_str("mod missing_types;\n");
            module_content.push_str("pub use missing_types::*;\n");
        }

        // Add Command trait re-export for command module
        if self.context.module.module_name == "command" {
            module_content.push_str("\n// Re-export Command trait from carla module\n");
            module_content.push_str("pub use crate::carla::Command;\n");
        }

        // Format with rustfmt
        let formatted = self.format_code(&module_content)?;

        // Write to mod.rs
        std::fs::write(module_dir.join("mod.rs"), formatted)?;

        Ok(())
    }

    /// Generate a single class file
    fn generate_class_file(&self, class: &Class, output_dir: &Path) -> Result<()> {
        let clean_name = class
            .class_name
            .strip_prefix("carla.")
            .unwrap_or(&class.class_name);
        let class_snake = clean_name.to_case(Case::Snake);
        let filename = format!("{class_snake}.rs");
        let filepath = output_dir.join(filename);

        debug!("Generating class file: {}", filepath.display());

        // Check if this is an enum
        let is_enum = is_enum_class(class);
        debug!("Class {} is_enum: {}", class.class_name, is_enum);

        // Log class structure for debugging
        if class.class_name.contains("ColorConverter") {
            eprintln!("DEBUG: ColorConverter class structure:");
            eprintln!("  - instance_variables: {:?}", class.instance_variables);
            eprintln!("  - methods: {} methods", class.methods.len());
            eprintln!("  - is_enum: {is_enum}");
        }

        if is_enum {
            self.generate_enum_file(class, filepath)?;
        } else {
            // Generate struct
            let (struct_data, struct_resolved_types) = self.generate_struct_data(class)?;
            let struct_code = self.renderer.render_struct(&struct_data)?;

            // Generate impl block
            let (impl_data, impl_resolved_types) = self.generate_impl_data(class)?;
            let impl_code = if impl_data.methods.is_empty() {
                String::new()
            } else {
                let impl_rendered = self.renderer.render_impl(&impl_data)?;
                format!("\n\n{impl_rendered}")
            };

            // Generate trait implementations based on special methods
            let trait_impls = if let Some(ref analysis) = struct_data.special_methods_analysis {
                let impls = self.generate_trait_implementations(class, analysis)?;
                debug!("Generated trait implementations length: {}", impls.len());
                if !impls.is_empty() {
                    debug!("Trait implementations content: {}", &impls);
                }
                impls
            } else {
                String::new()
            };

            // Collect all resolved types
            let mut all_resolved_types = struct_resolved_types;
            all_resolved_types.extend(impl_resolved_types);

            // If we don't have resolved types, fall back to string-based collection
            let imports = if all_resolved_types.is_empty() {
                eprintln!(
                    "DEBUG: Using string-based import collection for class {}",
                    class.class_name
                );
                // Fallback: collect all types used in fields and methods
                let mut all_types: Vec<String> = struct_data
                    .fields
                    .iter()
                    .map(|f| f.rust_type.clone())
                    .collect();

                // Add types from method signatures
                for method in &impl_data.methods {
                    // Add return type
                    all_types.push(method.return_type.clone());
                    // Also extract types from inside Result<T>
                    if method.return_type.starts_with("Result<")
                        && method.return_type.ends_with(">")
                    {
                        let inner = &method.return_type[7..method.return_type.len() - 1];
                        all_types.push(inner.to_string());
                        if inner.contains("HashMap") {
                            eprintln!("DEBUG: Extracted inner type with HashMap: {inner}");
                        }
                    }
                    // Debug specific method
                    if method.name == "get_vehicles_light_states" {
                        eprintln!(
                            "DEBUG: Found get_vehicles_light_states method with return type: {}",
                            method.return_type
                        );
                    }
                    // Add parameter types
                    for param in &method.params {
                        all_types.push(param.rust_type.clone());
                    }
                }

                {
                    eprintln!("DEBUG: All types passed to collect_imports: {all_types:?}");
                    self.collect_imports(&all_types)
                }
            } else {
                eprintln!(
                    "DEBUG: Using resolved-based import collection for class {}",
                    class.class_name
                );
                eprintln!("DEBUG: {} resolved types found", all_resolved_types.len());
                self.collect_imports_from_resolved(&all_resolved_types)
            };

            // Combine code
            let mut content = String::from("//! ");
            content.push_str(&self.context.rust_class_name(&class.class_name));
            content.push_str(" type\n\n");

            // Add imports
            for import in &imports {
                content.push_str(import);
                content.push('\n');
            }
            if !imports.is_empty() {
                content.push('\n');
            }

            content.push_str(&struct_code);
            content.push_str(&impl_code);
            if !trait_impls.is_empty() {
                content.push_str("\n\n");
                content.push_str(&trait_impls);
            }

            // Format with rustfmt
            let formatted = match self.format_code(&content) {
                Ok(formatted) => formatted,
                Err(e) => {
                    eprintln!("Error formatting class {}: {e}", class.class_name);
                    eprintln!("Content that failed to format:");
                    eprintln!("==== START CONTENT ====");
                    eprintln!("{content}");
                    eprintln!("==== END CONTENT ====");
                    return Err(e);
                }
            };

            // Debug: write unformatted content too
            let debug_path = filepath.with_extension("rs.debug");
            std::fs::write(&debug_path, &content)?;
            debug!("Wrote debug output to: {}", debug_path.display());

            std::fs::write(filepath, formatted)?;
        }
        Ok(())
    }

    /// Generate struct data for templates
    fn generate_struct_data(&self, class: &Class) -> Result<(StructData, Vec<ResolvedType>)> {
        // This should not be called for enums
        if is_enum_class(class) {
            return Err(CodegenError::InvalidStructure(format!(
                "generate_struct_data called for enum class: {}",
                class.class_name
            )));
        }

        let rust_name = self.context.rust_class_name(&class.class_name);
        let mut resolved_types = Vec::new();

        // Convert instance variables to fields
        let mut fields = Vec::new();
        for var in &class.instance_variables {
            // Skip variables without types (e.g., enum variants)
            let Some(ref var_type) = var.var_type else {
                continue;
            };

            // Create type context for field
            let type_context = TypeContext {
                location: TypeLocation::Field {
                    field_name: var.var_name.clone(),
                },
                containing_class: rust_name.clone(),
                containing_module: self.context.module.module_name.clone(),
                inside_generic: false,
                source_file: self
                    .context
                    .module
                    .source_file
                    .clone()
                    .unwrap_or_else(|| std::path::PathBuf::from("unknown.yml")),
            };

            // Use TypeResolverV2 for better type resolution
            let resolver_v2 = TypeResolverV2::new();
            let resolved = resolver_v2.resolve(var_type, type_context)?;

            // Fix recursive types
            let fixed_rust_type =
                self.fix_recursive_type(&resolved.rust_type, &rust_name, &var.var_name);

            // Handle union types specially
            let rust_type_string = match &fixed_rust_type {
                RustType::Union(types) => {
                    // Check for specific union patterns
                    if types.len() == 2 {
                        let type0 = types[0].to_rust_string();
                        let type1 = types[1].to_rust_string();
                        if (type0 == "crate::carla::Actor" && type1 == "i32")
                            || (type1 == "crate::carla::Actor" && type0 == "i32")
                        {
                            // Use ActorOrId enum
                            "crate::carla::ActorOrId".to_string()
                        } else {
                            // For other unions, we need to generate a custom enum
                            // For now, use the first type as a fallback
                            debug!(
                                "Warning: Unhandled union type {:?}, using first type",
                                fixed_rust_type
                            );
                            type0
                        }
                    } else {
                        // For other unions, use the first type
                        debug!(
                            "Warning: Complex union type {:?}, using first type",
                            fixed_rust_type
                        );
                        types
                            .first()
                            .map(|t| t.to_rust_string())
                            .unwrap_or_else(|| "String".to_string())
                    }
                }
                _ => fixed_rust_type.to_rust_string(),
            };

            // Add debug output for HashMap issue
            if var.var_name == "attributes" {
                eprintln!("DEBUG: Processing 'attributes' field");
                eprintln!("  Original type: {var_type}");
                eprintln!("  Resolved RustType: {:?}", resolved.rust_type);
                eprintln!("  Fixed RustType: {fixed_rust_type:?}");
                eprintln!("  Type string: {rust_type_string}");
            }

            // Store resolved type for import collection
            resolved_types.push(ResolvedType {
                rust_type: fixed_rust_type,
                imports_needed: resolved.imports_needed.clone(),
                needs_boxing: resolved.needs_boxing,
                type_alias: resolved.type_alias.clone(),
            });

            fields.push(RustField {
                name: var.var_name.to_case(Case::Snake),
                rust_type: rust_type_string,
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

        // Analyze special methods for trait derivations
        let analyzer = SpecialMethodAnalyzer::new();
        let special_methods_analysis = analyzer.analyze_class_with_methods(&class.methods);

        // Debug log union types
        if let Some(ref union_types) = special_methods_analysis.union_eq_types {
            debug!(
                "Found union types for {}: {}",
                class.class_name, union_types
            );
        }

        // Add derive attributes based on analysis
        let mut derives = special_methods_analysis.traits_to_derive.clone();

        // Always add Debug unless custom __repr__ is present
        let has_repr = class.methods.iter().any(|m| m.def_name == "__repr__");
        if !has_repr && !derives.contains(&"Debug".to_string()) {
            derives.push("Debug".to_string());
        }

        // Check if we can derive Copy
        if derives.contains(&"Copy".to_string()) && !self.can_derive_copy(&fields) {
            // Remove Copy if fields aren't copyable
            derives.retain(|d| d != "Copy");
            debug!(
                "Removed Copy derive from {} because it has non-copyable fields",
                rust_name
            );
        }

        // Always derive Clone if not already present
        if !derives.contains(&"Clone".to_string()) {
            derives.push("Clone".to_string());
        }

        // Filter out any empty strings from derives
        derives.retain(|d| !d.trim().is_empty());

        Ok((
            StructData {
                name: rust_name,
                doc: class.doc.clone(),
                fields,
                has_ffi_inner,
                ffi_type,
                derives,
                special_methods_analysis: Some(special_methods_analysis),
            },
            resolved_types,
        ))
    }

    /// Generate impl data for templates
    fn generate_impl_data(&self, class: &Class) -> Result<(ImplData, Vec<ResolvedType>)> {
        let rust_name = self.context.rust_class_name(&class.class_name);
        let mut resolved_types = Vec::new();

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

            // Check for per-method skip configuration
            if self
                .context
                .config
                .should_skip_method(&rust_name, &method.def_name)
            {
                debug!(
                    "Skipping method (per-method config): {}::{}",
                    rust_name, method.def_name
                );
                continue;
            }

            let rust_method = self.context.rust_method(method, &class.class_name)?;

            // Collect resolved types from method return type
            if let Some(return_type) = &method.return_type {
                let type_context = TypeContext {
                    location: TypeLocation::ReturnType {
                        method_name: method.def_name.clone(),
                    },
                    containing_class: rust_name.clone(),
                    containing_module: self.context.module.module_name.clone(),
                    inside_generic: false,
                    source_file: self
                        .context
                        .module
                        .source_file
                        .clone()
                        .unwrap_or_else(|| std::path::PathBuf::from("unknown.yml")),
                };

                let resolver_v2 = TypeResolverV2::new();
                if let Ok(resolved) = resolver_v2.resolve(return_type, type_context) {
                    resolved_types.push(resolved);
                }
            }

            // Collect resolved types from method parameters
            for param in &method.params {
                if let Some(param_type) = &param.param_type {
                    if param.param_name != "self" {
                        let type_context = TypeContext {
                            location: TypeLocation::Parameter {
                                method_name: method.def_name.clone(),
                                param_name: param.param_name.clone(),
                            },
                            containing_class: rust_name.clone(),
                            containing_module: self.context.module.module_name.clone(),
                            inside_generic: false,
                            source_file: self
                                .context
                                .module
                                .source_file
                                .clone()
                                .unwrap_or_else(|| std::path::PathBuf::from("unknown.yml")),
                        };

                        let resolver_v2 = TypeResolverV2::new();
                        if let Ok(resolved) = resolver_v2.resolve(param_type, type_context) {
                            resolved_types.push(resolved);
                        }
                    }
                }
            }

            methods.push(rust_method);
        }

        Ok((
            ImplData {
                name: rust_name,
                methods,
            },
            resolved_types,
        ))
    }

    /// Generate an enum file
    fn generate_enum_file(&self, class: &Class, filepath: std::path::PathBuf) -> Result<()> {
        debug!("Generating enum file for class: {}", class.class_name);
        debug!("Instance variables: {:?}", class.instance_variables);

        let rust_name = self.context.rust_class_name(&class.class_name);

        let mut content = String::from("//! ");
        content.push_str(&rust_name);
        content.push_str(" enum\n\n");

        // Add doc comment if present
        if let Some(doc) = &class.doc {
            content.push_str("/// ");
            content.push_str(&doc.replace('\n', "\n/// "));
            content.push('\n');
        }

        // Generate enum
        content.push_str("#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]\n");
        content.push_str("pub enum ");
        content.push_str(&rust_name);
        content.push_str(" {\n");

        // Generate variants
        for var in &class.instance_variables {
            if let Some(doc) = &var.doc {
                content.push_str("    /// ");
                content.push_str(&doc.replace('\n', "\n    /// "));
                content.push('\n');
            }
            content.push_str("    ");
            // Convert variant name to Rust convention
            let variant_name = self.to_rust_enum_variant(&var.var_name);
            content.push_str(&variant_name);
            content.push_str(",\n");
        }

        content.push_str("}\n");

        // Format with rustfmt
        let formatted = self.format_code(&content)?;

        std::fs::write(filepath, formatted)?;
        Ok(())
    }

    /// Format code using rustfmt
    fn format_code(&self, code: &str) -> Result<String> {
        // Check if formatting is enabled
        if !self.context.config.formatting.enable_rustfmt {
            return Ok(code.to_string());
        }

        use std::{
            io::Write,
            process::{Command, Stdio},
        };

        let mut cmd = Command::new("rustfmt");
        cmd.arg("--edition=2021")
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped());

        // Add custom rustfmt config if specified
        if let Some(ref config_path) = self.context.config.formatting.rustfmt_config {
            cmd.arg("--config-path").arg(config_path);
        }

        // Add inline configuration options
        if self.context.config.formatting.line_width != 100 {
            cmd.arg("--config").arg(format!(
                "max_width={}",
                self.context.config.formatting.line_width
            ));
        }

        if self.context.config.formatting.use_tabs {
            cmd.arg("--config").arg("hard_tabs=true");
        }

        if self.context.config.formatting.tab_width != 4 {
            cmd.arg("--config").arg(format!(
                "tab_spaces={}",
                self.context.config.formatting.tab_width
            ));
        }

        if self.context.config.formatting.force_trailing_commas {
            cmd.arg("--config").arg("trailing_comma=Always");
        }

        let rustfmt = cmd.spawn();

        match rustfmt {
            Ok(mut process) => {
                // Write code to rustfmt's stdin
                if let Some(stdin) = process.stdin.take() {
                    let mut stdin = stdin;
                    let _ = stdin.write_all(code.as_bytes());
                }

                // Read formatted output
                let output = process.wait_with_output()?;

                if output.status.success() {
                    Ok(String::from_utf8_lossy(&output.stdout).to_string())
                } else {
                    // If rustfmt fails, return original code
                    debug!(
                        "rustfmt failed: {}",
                        String::from_utf8_lossy(&output.stderr)
                    );
                    Ok(code.to_string())
                }
            }
            Err(e) => {
                // rustfmt not available, return code as-is
                debug!("rustfmt not found ({}), skipping formatting", e);
                Ok(code.to_string())
            }
        }
    }

    /// Generate trait implementations based on special methods analysis
    fn generate_trait_implementations(
        &self,
        class: &Class,
        analysis: &crate::analyzer::AnalysisResult,
    ) -> Result<String> {
        let mut implementations = Vec::new();
        let rust_name = self.context.rust_class_name(&class.class_name);

        debug!(
            "Manual implementations for {}: {:?}",
            rust_name,
            analysis.manual_implementations.keys().collect::<Vec<_>>()
        );
        debug!("Union eq types: {:?}", analysis.union_eq_types);

        // Generate implementations for each trait that needs manual implementation
        for trait_name in analysis.manual_implementations.keys() {
            match trait_name.as_str() {
                "PartialEq" => {
                    // Check if we have union types for eq
                    if let Some(union_types) = &analysis.union_eq_types {
                        // Generate multiple PartialEq implementations
                        implementations
                            .extend(self.generate_union_partial_eq_impls(&rust_name, union_types)?);
                    } else {
                        // Standard PartialEq implementation
                        if let Some(_eq_method) =
                            class.methods.iter().find(|m| m.def_name == "__eq__")
                        {
                            let impl_code = format!(
                                r#"impl PartialEq for {rust_name} {{
    fn eq(&self, other: &Self) -> bool {{
        // TODO: Implement using FFI function {rust_name}_eq
        todo!("{rust_name}::eq not yet implemented - missing FFI function {rust_name}_eq")
    }}
}}"#
                            );
                            implementations.push(impl_code);
                        }
                    }
                }
                "PartialOrd" => {
                    // Find comparison methods
                    let has_lt = class.methods.iter().any(|m| m.def_name == "__lt__");
                    let has_le = class.methods.iter().any(|m| m.def_name == "__le__");
                    let has_gt = class.methods.iter().any(|m| m.def_name == "__gt__");
                    let has_ge = class.methods.iter().any(|m| m.def_name == "__ge__");

                    if has_lt && has_le && has_gt && has_ge {
                        let impl_code = format!(
                            r#"impl PartialOrd for {rust_name} {{
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {{
        // TODO: Implement using FFI comparison functions
        todo!("{rust_name}::partial_cmp not yet implemented - missing FFI comparison functions")
    }}
}}"#
                        );
                        implementations.push(impl_code);
                    }
                }
                "Display" => {
                    // Find __str__ method
                    if class.methods.iter().any(|m| m.def_name == "__str__") {
                        let impl_code = format!(
                            r#"impl std::fmt::Display for {rust_name} {{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {{
        // TODO: Implement using FFI function {rust_name}_str
        todo!("{rust_name}::fmt for Display not yet implemented - missing FFI function {rust_name}_str")
    }}
}}"#
                        );
                        implementations.push(impl_code);
                    }
                }
                "Debug" => {
                    // Find __repr__ method
                    if class.methods.iter().any(|m| m.def_name == "__repr__") {
                        let impl_code = format!(
                            r#"impl std::fmt::Debug for {rust_name} {{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {{
        // TODO: Implement using FFI function {rust_name}_repr
        todo!("{rust_name}::fmt for Debug not yet implemented - missing FFI function {rust_name}_repr")
    }}
}}"#
                        );
                        implementations.push(impl_code);
                    }
                }
                "Hash" => {
                    // Find __hash__ method
                    if class.methods.iter().any(|m| m.def_name == "__hash__") {
                        let impl_code = format!(
                            r#"impl std::hash::Hash for {rust_name} {{
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {{
        // TODO: Implement using FFI function {rust_name}_hash
        todo!("{rust_name}::hash not yet implemented - missing FFI function {rust_name}_hash")
    }}
}}"#
                        );
                        implementations.push(impl_code);
                    }
                }
                "Add" | "Sub" | "Mul" | "Div" => {
                    // Handle arithmetic operators
                    let (method_name, trait_method) = match trait_name.as_str() {
                        "Add" => ("__add__", "add"),
                        "Sub" => ("__sub__", "sub"),
                        "Mul" => ("__mul__", "mul"),
                        "Div" => ("__div__", "div"),
                        _ => continue,
                    };

                    if class.methods.iter().any(|m| m.def_name == method_name) {
                        let impl_code = format!(
                            r#"impl std::ops::{trait_name} for {rust_name} {{
    type Output = Self;
    
    fn {trait_method}(self, rhs: Self) -> Self::Output {{
        // TODO: Implement using FFI function {rust_name}_{trait_method}
        todo!("{rust_name}::{trait_method} not yet implemented - missing FFI function {rust_name}_{trait_method}")
    }}
}}"#
                        );
                        implementations.push(impl_code);
                    }
                }
                _ => {
                    // Other traits not yet handled
                    debug!("Trait implementation for {} not yet supported", trait_name);
                }
            }
        }

        Ok(implementations.join("\n\n"))
    }

    /// Generate multiple PartialEq implementations for union types
    fn generate_union_partial_eq_impls(
        &self,
        rust_name: &str,
        union_types_str: &str,
    ) -> Result<Vec<String>> {
        let mut implementations = Vec::new();

        // Parse the union type string and resolve each type
        let type_parts: Vec<&str> = if union_types_str.contains(" / ") {
            union_types_str.split(" / ").map(|s| s.trim()).collect()
        } else if union_types_str.contains(" or ") {
            union_types_str.split(" or ").map(|s| s.trim()).collect()
        } else {
            vec![union_types_str]
        };

        // Generate PartialEq<T> for each type in the union
        let mut _has_self_comparison = false;
        for type_str in type_parts {
            match self.context.type_resolver.resolve_type(type_str) {
                Ok(rust_type) => {
                    let type_name = rust_type.to_rust_string();

                    // Check if this is a self-comparison
                    if type_name == format!("crate::carla::{rust_name}") {
                        _has_self_comparison = true;
                        continue; // Skip, we'll add the standard PartialEq later
                    }

                    let impl_code = format!(
                        r#"impl PartialEq<{type_name}> for {rust_name} {{
    fn eq(&self, other: &{type_name}) -> bool {{
        // TODO: Implement using FFI function {rust_name}_eq_{}
        todo!("{rust_name}::eq<{type_name}> not yet implemented - missing FFI function")
    }}
}}"#,
                        type_str.replace('.', "_").replace(' ', "")
                    );
                    implementations.push(impl_code);
                }
                Err(e) => {
                    tracing::warn!("Failed to resolve union type '{}': {}", type_str, e);
                }
            }
        }

        // Also generate the standard PartialEq for self-comparison
        let self_impl = format!(
            r#"impl PartialEq for {rust_name} {{
    fn eq(&self, other: &Self) -> bool {{
        // TODO: Implement using FFI function {rust_name}_eq
        todo!("{rust_name}::eq not yet implemented - missing FFI function {rust_name}_eq")
    }}
}}"#
        );
        implementations.push(self_impl);

        Ok(implementations)
    }
}
