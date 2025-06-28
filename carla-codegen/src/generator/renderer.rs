//! Template rendering interface

use crate::{
    error::Result,
    generator::context::{ImplData, ModuleData, StructData},
};

/// Common interface for template renderers
pub trait TemplateRenderer {
    /// Render a struct definition
    fn render_struct(&self, data: &StructData) -> Result<String>;

    /// Render an impl block
    fn render_impl(&self, data: &ImplData) -> Result<String>;

    /// Render a module file
    fn render_module(&self, data: &ModuleData) -> Result<String>;
}
