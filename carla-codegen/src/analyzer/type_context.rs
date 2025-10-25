//! Type resolution context information

use std::path::PathBuf;

/// Context information for type resolution
#[derive(Debug, Clone)]
pub struct TypeContext {
    /// Where the type appears
    pub location: TypeLocation,
    /// The containing class name
    pub containing_class: String,
    /// The containing module
    pub containing_module: String,
    /// Whether this type is inside a generic
    pub inside_generic: bool,
    /// Source file for error reporting
    pub source_file: PathBuf,
}

/// Where a type appears in the code
#[derive(Debug, Clone, PartialEq)]
pub enum TypeLocation {
    /// Method parameter
    Parameter {
        method_name: String,
        param_name: String,
    },
    /// Method return type
    ReturnType { method_name: String },
    /// Struct/class field
    Field { field_name: String },
    /// Inside a trait implementation
    TraitImpl { trait_name: String },
    /// Generic type parameter
    GenericParam,
    /// Type alias definition
    TypeAlias,
}

impl TypeContext {
    /// Create a new type context
    pub fn new(
        location: TypeLocation,
        containing_class: String,
        containing_module: String,
    ) -> Self {
        Self {
            location,
            containing_class,
            containing_module,
            inside_generic: false,
            source_file: PathBuf::new(),
        }
    }

    /// Set the source file
    pub fn with_source_file(mut self, file: PathBuf) -> Self {
        self.source_file = file;
        self
    }

    /// Mark as being inside a generic
    pub fn inside_generic(mut self) -> Self {
        self.inside_generic = true;
        self
    }

    /// Check if this is a parameter type
    pub fn is_parameter(&self) -> bool {
        matches!(self.location, TypeLocation::Parameter { .. })
    }

    /// Check if this is a return type
    pub fn is_return_type(&self) -> bool {
        matches!(self.location, TypeLocation::ReturnType { .. })
    }

    /// Get the method name if this is in a method context
    pub fn method_name(&self) -> Option<&str> {
        match &self.location {
            TypeLocation::Parameter { method_name, .. } => Some(method_name),
            TypeLocation::ReturnType { method_name } => Some(method_name),
            _ => None,
        }
    }

    /// Get the parameter name if this is a parameter
    pub fn param_name(&self) -> Option<&str> {
        match &self.location {
            TypeLocation::Parameter { param_name, .. } => Some(param_name),
            _ => None,
        }
    }
}
