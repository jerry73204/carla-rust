//! Code analysis module for type resolution and dependency management

pub mod dependency;
pub mod inheritance;
pub mod type_resolver;

pub use dependency::DependencyGraph;
pub use inheritance::{InheritanceInfo, InheritanceResolver};
pub use type_resolver::{RustType, TypeResolver};
