//! Code generation module

pub mod context;
pub mod error_module;
pub mod ffi_generator;
pub mod import_collector;
pub mod missing_types;
pub mod renderer;
pub mod rust_code;
pub mod syn_renderer;
pub mod type_fixer;

use crate::{
    analyzer::{DependencyGraph, InheritanceResolver, TypeResolver},
    config::Config,
    error::Result,
    parser::yaml_schema::Module,
};
use std::path::Path;
use tracing::info;

pub use context::GeneratorContext;
pub use ffi_generator::{FfiGenerator, FfiModule};
pub use renderer::TemplateRenderer;
pub use rust_code::RustCodeGenerator;
pub use syn_renderer::SynRenderer;

/// Main code generator that orchestrates the generation process
pub struct CodeGenerator {
    config: Config,
    type_resolver: TypeResolver,
    inheritance_resolver: InheritanceResolver,
    dependency_graph: DependencyGraph,
    renderer: SynRenderer,
}

impl CodeGenerator {
    /// Create a new code generator
    pub fn new(config: Config) -> Result<Self> {
        let renderer = SynRenderer::new();

        // Create type resolver with custom mappings from config
        let type_resolver =
            TypeResolver::with_custom_mappings(config.type_mapping.mappings.clone());

        Ok(Self {
            config,
            type_resolver,
            inheritance_resolver: InheritanceResolver::new(),
            dependency_graph: DependencyGraph::new(),
            renderer,
        })
    }

    /// Generate code from parsed modules
    pub fn generate(&mut self, modules: &[Module], output_dir: &Path) -> Result<()> {
        info!("Starting code generation for {} modules", modules.len());

        // Build analysis data
        self.inheritance_resolver.build_from_modules(modules);
        self.dependency_graph.build_from_modules(modules)?;

        // Get generation order
        let class_order = self.dependency_graph.topological_sort()?;
        info!("Generated class order: {} classes", class_order.len());

        // Create output directory
        std::fs::create_dir_all(output_dir)?;

        // Generate error module first (if enabled)
        self.generate_error_module(output_dir)?;

        // Generate code based on configuration
        if self.config.generate_ffi
            || self
                .config
                .output_dir
                .to_string_lossy()
                .contains("carla-sys")
        {
            // Generate FFI code for carla-sys
            self.generate_ffi_code(modules, output_dir)?;
        } else {
            // Generate regular Rust code
            for module in modules {
                self.generate_module(module, output_dir)?;
            }

            // Generate mod.rs files for the module hierarchy
            self.generate_mod_files(modules, output_dir)?;
        }

        info!("Code generation completed successfully");
        Ok(())
    }

    /// Generate FFI code for carla-sys
    fn generate_ffi_code(&self, modules: &[Module], output_dir: &Path) -> Result<()> {
        info!("Generating FFI code for carla-sys");

        let ffi_gen = FfiGenerator::new(&self.config, &self.type_resolver);
        let mut ffi_modules = Vec::new();

        // Generate FFI modules
        for module in modules {
            if self
                .config
                .should_generate_module(&module.module_name.replace("carla.", ""))
            {
                let ffi_module = ffi_gen.generate_ffi_module(module)?;
                ffi_modules.push(ffi_module);
            }
        }

        // Generate Rust FFI declarations
        self.generate_ffi_rust_code(&ffi_modules, output_dir)?;

        // Generate C++ bridge implementation
        let cpp_bridge = ffi_gen.generate_cpp_bridge(&ffi_modules)?;
        let cpp_path = output_dir.join("carla_sys_bridge_generated.cpp");
        std::fs::write(cpp_path, cpp_bridge)?;

        // Generate CXX bridge declarations
        self.generate_cxx_bridge(&ffi_modules, output_dir)?;

        Ok(())
    }

    /// Generate Rust FFI declarations
    fn generate_ffi_rust_code(&self, ffi_modules: &[FfiModule], output_dir: &Path) -> Result<()> {
        let mut rust_code = String::new();

        rust_code.push_str("// Generated FFI declarations for CARLA API\n\n");
        rust_code.push_str("use cxx::SharedPtr;\n\n");

        // Generate type exports
        rust_code.push_str("// Re-export generated types\n");
        rust_code.push_str("pub mod types {\n");
        for module in ffi_modules {
            for class in &module.classes {
                rust_code.push_str(&format!("    pub struct {};\n", class.rust_name));
            }
        }
        rust_code.push_str("}\n\n");

        // Write to file
        let rust_path = output_dir.join("ffi_generated.rs");
        std::fs::write(rust_path, rust_code)?;

        Ok(())
    }

    /// Generate CXX bridge declarations
    fn generate_cxx_bridge(&self, ffi_modules: &[FfiModule], output_dir: &Path) -> Result<()> {
        let mut cxx_code = String::new();

        cxx_code.push_str("// Generated CXX bridge declarations\n\n");
        cxx_code.push_str("#[cxx::bridge(namespace = \"carla_sys\")]\n");
        cxx_code.push_str("mod ffi_generated {\n");

        // Extern "C++" block
        cxx_code.push_str("    unsafe extern \"C++\" {\n");
        cxx_code.push_str("        include!(\"carla_sys_bridge.h\");\n\n");

        // Add type declarations
        cxx_code.push_str("        // Generated type declarations\n");
        for module in ffi_modules {
            for class in &module.classes {
                cxx_code.push_str(&format!("        type {};\n", class.name));
            }
        }

        cxx_code.push_str("\n        // Generated function declarations\n");
        for module in ffi_modules {
            for class in &module.classes {
                for method in &class.methods {
                    // Generate function signature
                    let params = method
                        .parameters
                        .iter()
                        .map(|p| format!("{}: {}", p.name, p.rust_type))
                        .collect::<Vec<_>>()
                        .join(", ");

                    let return_type = method
                        .return_type
                        .as_ref()
                        .map(|t| {
                            // Convert C++ types to Rust types for CXX bridge
                            let rust_type = match t.as_str() {
                                "int32_t" => "i32".to_string(),
                                "double" => "f64".to_string(),
                                "bool" => "bool".to_string(),
                                "std::string" => "String".to_string(),
                                _ if t.starts_with("std::vector<") => {
                                    // Convert std::vector<T> to Vec<T>
                                    let inner =
                                        t.trim_start_matches("std::vector<").trim_end_matches('>');
                                    format!("Vec<{inner}>")
                                }
                                _ if t.starts_with("SharedPtr<") => t.to_string(),
                                _ if t.starts_with("carla::client::") => {
                                    // For CARLA types, we need to map them properly
                                    let type_name = t.strip_prefix("carla::client::").unwrap_or(t);
                                    type_name.to_string()
                                }
                                _ => t.to_string(),
                            };
                            format!(" -> {rust_type}")
                        })
                        .unwrap_or_default();

                    cxx_code.push_str(&format!(
                        "        fn {}({}){};\n",
                        method.name, params, return_type
                    ));
                }
            }
        }

        cxx_code.push_str("    }\n");
        cxx_code.push_str("}\n");

        // Write to file
        let cxx_path = output_dir.join("cxx_bridge_generated.rs");
        std::fs::write(cxx_path, cxx_code)?;

        Ok(())
    }

    /// Generate error module
    fn generate_error_module(&self, output_dir: &Path) -> Result<()> {
        use error_module::{ErrorModuleGenerator, TargetCrate};

        // Check if error module generation is enabled (default: true)
        let generate_error = self.config.error_generation.enabled;
        if !generate_error {
            info!("Skipping error module generation (disabled in config)");
            return Ok(());
        }

        info!("Generating error module");

        // Determine target crate type
        let target_crate =
            if self.config.generate_ffi || output_dir.to_string_lossy().contains("carla-sys") {
                TargetCrate::CarlaSys
            } else if output_dir.to_string_lossy().contains("carla") {
                TargetCrate::Carla
            } else {
                TargetCrate::Generic
            };

        let generator = ErrorModuleGenerator::new(target_crate)
            .with_thiserror(self.config.error_generation.use_thiserror)
            .with_error_type_name(self.config.error_generation.error_type_name.clone());

        generator.generate_file(output_dir)?;
        info!("Generated error.rs");

        Ok(())
    }

    /// Generate code for a single module
    fn generate_module(&self, module: &Module, output_dir: &Path) -> Result<()> {
        // Check if module should be generated
        let module_name = module.module_name.replace("carla.", "");
        if !self.config.should_generate_module(&module_name) {
            info!("Skipping module: {}", module.module_name);
            return Ok(());
        }

        info!("Generating code for module: {}", module.module_name);

        let context = GeneratorContext::new(
            module,
            &self.type_resolver,
            &self.inheritance_resolver,
            &self.config,
        );

        let rust_gen = RustCodeGenerator::new(&context, &self.renderer);
        match rust_gen.generate_module(output_dir) {
            Ok(()) => {}
            Err(e) => {
                eprintln!("Error generating module {}: {}", module.module_name, e);
                return Err(e);
            }
        }

        Ok(())
    }

    /// Generate mod.rs files for the module hierarchy
    fn generate_mod_files(&self, modules: &[Module], output_dir: &Path) -> Result<()> {
        use std::collections::{HashMap, HashSet};

        // Build module hierarchy
        let mut module_tree: HashMap<String, HashSet<String>> = HashMap::new();

        // Root level modules
        let mut root_modules = HashSet::new();

        for module in modules {
            let parts: Vec<&str> = module.module_name.split('.').collect();

            if parts.is_empty() {
                continue;
            }

            // Add to root modules
            root_modules.insert(parts[0].to_string());

            // Build the hierarchy
            for i in 0..parts.len() - 1 {
                let parent_path = parts[0..=i].join("/");
                let child_name = parts[i + 1];

                module_tree
                    .entry(parent_path)
                    .or_default()
                    .insert(child_name.to_string());
            }
        }

        // Generate root mod.rs
        let mut root_content = String::from("//! Generated CARLA API bindings\n\n");

        // Include error module if it exists
        if output_dir.join("error.rs").exists() {
            root_content.push_str("pub mod error;\n\n");
        }

        let mut sorted_roots: Vec<_> = root_modules.into_iter().collect();
        sorted_roots.sort();

        for module_name in sorted_roots {
            root_content.push_str(&format!("pub mod {module_name};\n"));
        }

        std::fs::write(output_dir.join("mod.rs"), root_content)?;

        // Generate intermediate mod.rs files
        for (parent_path, children) in module_tree {
            let parent_dir = output_dir.join(&parent_path);
            let mod_file = parent_dir.join("mod.rs");

            // Read existing content if file exists
            let existing_content = if mod_file.exists() {
                std::fs::read_to_string(&mod_file).unwrap_or_default()
            } else {
                String::new()
            };

            // Collect existing module declarations
            let mut all_modules = HashSet::new();
            for line in existing_content.lines() {
                if let Some(module) = line.strip_prefix("mod ").and_then(|s| s.strip_suffix(';')) {
                    all_modules.insert(module.to_string());
                } else if let Some(module) = line
                    .strip_prefix("pub mod ")
                    .and_then(|s| s.strip_suffix(';'))
                {
                    all_modules.insert(module.to_string());
                }
            }

            // Add new children
            all_modules.extend(children);

            // Generate new content preserving doc comments and exports
            let mut new_content = String::new();
            let mut in_header = true;
            let mut header_lines = Vec::new();

            for line in existing_content.lines() {
                if in_header && (line.starts_with("//!") || line.trim().is_empty()) {
                    header_lines.push(line);
                } else if line.starts_with("mod ") || line.starts_with("pub mod ") {
                    in_header = false;
                    // Skip old mod declarations, we'll regenerate them
                } else if line.starts_with("pub use ") {
                    in_header = false;
                    // Preserve pub use statements
                    if !new_content.is_empty() || !header_lines.is_empty() {
                        // Add header if we haven't already
                        if !header_lines.is_empty() {
                            new_content.push_str(&header_lines.join("\n"));
                            new_content.push_str("\n\n");
                            header_lines.clear();
                        }
                    }
                    new_content.push_str(line);
                    new_content.push('\n');
                }
            }

            // Add header if not already added
            if !header_lines.is_empty() {
                new_content = header_lines.join("\n") + "\n\n" + &new_content;
            }

            // Add module declarations
            let mut sorted_modules: Vec<_> = all_modules.into_iter().collect();
            sorted_modules.sort();

            // Find where to insert mod declarations (before pub use statements)
            let lines: Vec<&str> = new_content.lines().collect();
            let insert_pos = lines
                .iter()
                .position(|line| line.starts_with("pub use "))
                .unwrap_or(lines.len());

            let mut final_content = String::new();

            // Add lines before pub use
            for (i, line) in lines.iter().enumerate() {
                if i == insert_pos {
                    // Insert mod declarations
                    for module in &sorted_modules {
                        final_content.push_str(&format!("pub mod {module};\n"));
                    }
                    if !sorted_modules.is_empty() && insert_pos < lines.len() {
                        final_content.push('\n'); // Blank line before pub use
                    }
                }
                final_content.push_str(line);
                final_content.push('\n');
            }

            // If no pub use statements, add mod declarations at the end
            if insert_pos == lines.len() && !sorted_modules.is_empty() {
                for module in &sorted_modules {
                    final_content.push_str(&format!("pub mod {module};\n"));
                }
            }

            std::fs::write(mod_file, final_content)?;
        }

        Ok(())
    }
}
