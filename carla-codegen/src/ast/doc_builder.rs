//! Documentation generation utilities for AST builders

use std::fmt::Write;
use syn::{parse_quote, Attribute};

/// Builder for generating comprehensive Rust documentation
#[derive(Debug, Clone)]
pub struct DocBuilder {
    /// Main documentation text
    pub content: String,
    /// Optional warning text
    pub warning: Option<String>,
    /// Optional note text
    pub note: Option<String>,
    /// Code examples
    pub examples: Vec<String>,
    /// Whether to generate doc tests
    pub generate_tests: bool,
}

impl DocBuilder {
    /// Create a new documentation builder
    pub fn new(content: String) -> Self {
        Self {
            content,
            warning: None,
            note: None,
            examples: Vec::new(),
            generate_tests: true,
        }
    }

    /// Add a warning section
    pub fn with_warning(mut self, warning: String) -> Self {
        self.warning = Some(warning);
        self
    }

    /// Add a note section
    pub fn with_note(mut self, note: String) -> Self {
        self.note = Some(note);
        self
    }

    /// Add a code example
    pub fn add_example(mut self, example: String) -> Self {
        self.examples.push(example);
        self
    }

    /// Set whether to generate doc tests
    pub fn with_doc_tests(mut self, generate_tests: bool) -> Self {
        self.generate_tests = generate_tests;
        self
    }

    /// Build the complete documentation as attributes
    pub fn build(&self) -> Vec<Attribute> {
        let mut attrs = Vec::new();

        // Main documentation
        self.add_doc_lines(&mut attrs, &self.content);

        // Warning section
        if let Some(ref warning) = self.warning {
            attrs.push(parse_quote!(#[doc = ""]));
            attrs.push(parse_quote!(#[doc = "# Warning"]));
            attrs.push(parse_quote!(#[doc = ""]));
            self.add_doc_lines(&mut attrs, warning);
        }

        // Note section
        if let Some(ref note) = self.note {
            attrs.push(parse_quote!(#[doc = ""]));
            attrs.push(parse_quote!(#[doc = "# Note"]));
            attrs.push(parse_quote!(#[doc = ""]));
            self.add_doc_lines(&mut attrs, note);
        }

        // Examples
        if !self.examples.is_empty() {
            attrs.push(parse_quote!(#[doc = ""]));
            attrs.push(parse_quote!(#[doc = "# Examples"]));
            attrs.push(parse_quote!(#[doc = ""]));

            for (i, example) in self.examples.iter().enumerate() {
                if i > 0 {
                    attrs.push(parse_quote!(#[doc = ""]));
                }

                // Determine if this should be a doc test
                let code_block = if self.generate_tests && example.contains("carla") {
                    "```rust,no_run"
                } else if self.generate_tests {
                    "```rust"
                } else {
                    "```rust,ignore"
                };

                attrs.push(parse_quote!(#[doc = #code_block]));
                self.add_doc_lines(&mut attrs, example);
                attrs.push(parse_quote!(#[doc = "```"]));
            }
        }

        attrs
    }

    /// Helper to add multi-line documentation
    fn add_doc_lines(&self, attrs: &mut Vec<Attribute>, content: &str) {
        for line in content.lines() {
            let trimmed = line.trim();
            attrs.push(parse_quote!(#[doc = #trimmed]));
        }
    }
}

/// Generate FFI documentation with TODO placeholder
pub fn generate_ffi_todo_doc(class_name: &str, method_name: &str) -> DocBuilder {
    let content = format!(
        "TODO: Implement using carla-sys FFI interface\n\
         This requires adding {}_{} FFI function to carla-sys.",
        class_name, method_name
    );

    DocBuilder::new(content).with_note(
        "This method is not yet implemented and will panic with todo!() \
             when called. Implementation requires corresponding FFI bindings \
             in the carla-sys crate."
            .to_string(),
    )
}

/// Generate method documentation from Python API docs
pub fn generate_method_doc(
    doc: Option<&str>,
    params: &[(&str, &str)], // (name, doc) pairs
    return_doc: Option<&str>,
    warning: Option<&str>,
    note: Option<&str>,
) -> DocBuilder {
    let mut content = String::new();

    // Main documentation
    if let Some(doc) = doc {
        content.push_str(doc);
    } else {
        content.push_str("Method from CARLA Python API");
    }

    // Parameters section
    if !params.is_empty() {
        content.push_str("\n\n# Parameters\n\n");
        for (name, param_doc) in params {
            writeln!(content, "* `{}` - {}", name, param_doc).unwrap();
        }
    }

    // Returns section
    if let Some(return_doc) = return_doc {
        writeln!(content, "\n# Returns\n\n{}", return_doc).unwrap();
    }

    let mut builder = DocBuilder::new(content);

    if let Some(warning) = warning {
        builder = builder.with_warning(warning.to_string());
    }

    if let Some(note) = note {
        builder = builder.with_note(note.to_string());
    }

    builder
}

/// Generate struct field documentation
pub fn generate_field_doc(
    doc: Option<&str>,
    units: Option<&str>,
    warning: Option<&str>,
) -> Vec<Attribute> {
    let mut attrs = Vec::new();

    // Main documentation
    if let Some(doc) = doc {
        for line in doc.lines() {
            let trimmed = line.trim();
            if !trimmed.is_empty() {
                attrs.push(parse_quote!(#[doc = #trimmed]));
            }
        }
    }

    // Units information
    if let Some(units) = units {
        if !attrs.is_empty() {
            attrs.push(parse_quote!(#[doc = ""]));
        }
        let units_doc = format!("**Units**: {}", units);
        attrs.push(parse_quote!(#[doc = #units_doc]));
    }

    // Warning
    if let Some(warning) = warning {
        if !attrs.is_empty() {
            attrs.push(parse_quote!(#[doc = ""]));
        }
        attrs.push(parse_quote!(#[doc = "# Warning"]));
        attrs.push(parse_quote!(#[doc = ""]));
        for line in warning.lines() {
            let trimmed = line.trim();
            attrs.push(parse_quote!(#[doc = #trimmed]));
        }
    }

    attrs
}

/// Generate module-level documentation
pub fn generate_module_doc(module_name: &str, doc: Option<&str>) -> Vec<Attribute> {
    let mut attrs = Vec::new();

    let content = if let Some(doc) = doc {
        format!("# {}\n\n{}", module_name, doc)
    } else {
        format!(
            "# {}\n\nAuto-generated module from CARLA Python API",
            module_name
        )
    };

    attrs.push(parse_quote!(#[doc = ""]));

    for line in content.lines() {
        let trimmed = line.trim();
        attrs.push(parse_quote!(#[doc = #trimmed]));
    }

    attrs.push(parse_quote!(#[doc = ""]));
    attrs.push(
        parse_quote!(#[doc = "This module contains types and functions generated from CARLA's"]),
    );
    attrs.push(parse_quote!(#[doc = "Python API documentation. These complement the manually implemented"]));
    attrs.push(parse_quote!(#[doc = "types in the rest of the crate."]));

    attrs
}

#[cfg(test)]
mod tests {
    use super::*;
    use quote::ToTokens;

    #[test]
    fn test_basic_doc_builder() {
        let doc = DocBuilder::new("This is a test function".to_string());
        let attrs = doc.build();

        assert_eq!(attrs.len(), 1);
        let tokens = attrs[0].to_token_stream().to_string();
        assert!(tokens.contains("This is a test function"));
    }

    #[test]
    fn test_doc_with_warning() {
        let doc = DocBuilder::new("Main documentation".to_string())
            .with_warning("This is dangerous".to_string());
        let attrs = doc.build();

        let all_tokens = attrs
            .iter()
            .map(|attr| attr.to_token_stream().to_string())
            .collect::<Vec<_>>()
            .join(" ");

        assert!(all_tokens.contains("Main documentation"));
        assert!(all_tokens.contains("# Warning"));
        assert!(all_tokens.contains("This is dangerous"));
    }

    #[test]
    fn test_doc_with_example() {
        let example = r#"
let client = Client::new("localhost", 2000)?;
let world = client.world()?;
"#
        .trim()
        .to_string();

        let doc = DocBuilder::new("Creates a client".to_string()).add_example(example);
        let attrs = doc.build();

        let all_tokens = attrs
            .iter()
            .map(|attr| attr.to_token_stream().to_string())
            .collect::<Vec<_>>()
            .join(" ");

        assert!(all_tokens.contains("Creates a client"));
        assert!(all_tokens.contains("# Examples"));
        assert!(all_tokens.contains("```rust")); // Just rust, not rust,no_run
        assert!(all_tokens.contains("Client::new"));
    }

    #[test]
    fn test_ffi_todo_doc() {
        let doc = generate_ffi_todo_doc("Actor", "GetId");
        let attrs = doc.build();

        let all_tokens = attrs
            .iter()
            .map(|attr| attr.to_token_stream().to_string())
            .collect::<Vec<_>>()
            .join(" ");

        assert!(all_tokens.contains("TODO: Implement"));
        assert!(all_tokens.contains("Actor_GetId"));
        assert!(all_tokens.contains("# Note"));
        assert!(all_tokens.contains("todo!()"));
    }

    #[test]
    fn test_method_doc_generation() {
        let params = vec![
            ("actor_id", "ID of the actor to retrieve"),
            ("timeout", "Timeout in seconds"),
        ];

        let doc = generate_method_doc(
            Some("Gets an actor by ID"),
            &params,
            Some("The requested actor"),
            Some("Actor might not exist"),
            None,
        );

        let attrs = doc.build();
        let all_tokens = attrs
            .iter()
            .map(|attr| attr.to_token_stream().to_string())
            .collect::<Vec<_>>()
            .join(" ");

        assert!(all_tokens.contains("Gets an actor by ID"));
        assert!(all_tokens.contains("# Parameters"));
        assert!(all_tokens.contains("actor_id"));
        assert!(all_tokens.contains("ID of the actor"));
        assert!(all_tokens.contains("# Returns"));
        assert!(all_tokens.contains("The requested actor"));
        assert!(all_tokens.contains("# Warning"));
        assert!(all_tokens.contains("Actor might not exist"));
    }

    #[test]
    fn test_field_doc_generation() {
        let attrs = generate_field_doc(
            Some("X coordinate of the position"),
            Some("meters"),
            Some("May be NaN in some cases"),
        );

        let all_tokens = attrs
            .iter()
            .map(|attr| attr.to_token_stream().to_string())
            .collect::<Vec<_>>()
            .join(" ");

        assert!(all_tokens.contains("X coordinate"));
        assert!(all_tokens.contains("**Units**: meters"));
        assert!(all_tokens.contains("# Warning"));
        assert!(all_tokens.contains("May be NaN"));
    }

    #[test]
    fn test_module_doc_generation() {
        let attrs = generate_module_doc("geometry", Some("Geometric types and operations"));

        let all_tokens = attrs
            .iter()
            .map(|attr| attr.to_token_stream().to_string())
            .collect::<Vec<_>>()
            .join(" ");

        assert!(all_tokens.contains("# geometry"));
        assert!(all_tokens.contains("Geometric types"));
        assert!(all_tokens.contains("generated from CARLA"));
        assert!(all_tokens.contains("Python API documentation"));
    }
}
