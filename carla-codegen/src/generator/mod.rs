//! Code generation module

pub mod context;
pub mod renderer;
pub mod rust_code;
pub mod syn_renderer;

use crate::{
    analyzer::{DependencyGraph, InheritanceResolver, TypeResolver},
    config::Config,
    error::Result,
    parser::yaml_schema::Module,
};
use std::path::Path;
use tracing::info;

pub use context::GeneratorContext;
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

        // Generate code for each module
        for module in modules {
            self.generate_module(module, output_dir)?;
        }

        // Generate mod.rs file
        self.generate_mod_file(modules, output_dir)?;

        info!("Code generation completed successfully");
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
        rust_gen.generate_module(output_dir)?;

        Ok(())
    }

    /// Generate the main mod.rs file
    fn generate_mod_file(&self, modules: &[Module], output_dir: &Path) -> Result<()> {
        let mut content = String::from("//! Generated CARLA API bindings\n\n");

        // Add module declarations
        for module in modules {
            let module_name = module.module_name.replace("carla.", "");
            if !module_name.is_empty() && module_name != "carla" {
                content.push_str(&format!("pub mod {};\n", module_name));
            }
        }

        std::fs::write(output_dir.join("mod.rs"), content)?;
        Ok(())
    }
}
