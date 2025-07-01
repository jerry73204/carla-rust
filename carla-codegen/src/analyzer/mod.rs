//! Code analysis module for type resolution and dependency management

pub mod dependency;
pub mod enum_detector;
pub mod inheritance;
pub mod method_analyzer;
pub mod special_methods;
pub mod type_resolver;

pub use dependency::DependencyGraph;
pub use enum_detector::{get_enum_variants, is_enum_class};
pub use inheritance::{InheritanceInfo, InheritanceResolver};
pub use method_analyzer::{determine_self_type, determine_self_type_with_config, SelfType};
pub use special_methods::{AnalysisResult, SpecialMethodAnalyzer, SpecialMethodImpl};
pub use type_resolver::{RustType, TypeResolver};
