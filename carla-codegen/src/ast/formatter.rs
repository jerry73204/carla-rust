//! Code formatting utilities for generated Rust code

use prettyplease;
use syn::{File, Item};

/// Configuration for code formatting
#[derive(Debug, Clone)]
pub struct FormatConfig {
    /// Whether to use prettyplease for formatting
    pub use_prettyplease: bool,
    /// Indentation size
    pub indent_size: usize,
    /// Maximum line width
    pub max_width: usize,
    /// Whether to format doc comments
    pub format_docs: bool,
}

impl Default for FormatConfig {
    fn default() -> Self {
        Self {
            use_prettyplease: true,
            indent_size: 4,
            max_width: 100,
            format_docs: true,
        }
    }
}

/// Formatter for Rust code using syn AST
#[derive(Debug)]
pub struct RustFormatter {
    config: FormatConfig,
}

impl RustFormatter {
    /// Create a new formatter with default configuration
    pub fn new() -> Self {
        Self {
            config: FormatConfig::default(),
        }
    }

    /// Create a new formatter with custom configuration
    pub fn with_config(config: FormatConfig) -> Self {
        Self { config }
    }

    /// Format a complete Rust file
    pub fn format_file(&self, file: &File) -> Result<String, FormatterError> {
        if self.config.use_prettyplease {
            Ok(prettyplease::unparse(file))
        } else {
            self.format_file_manual(file)
        }
    }

    /// Format a single item
    pub fn format_item(&self, item: &Item) -> Result<String, FormatterError> {
        // Create a temporary file with just this item
        let file = File {
            shebang: None,
            attrs: Vec::new(),
            items: vec![item.clone()],
        };

        self.format_file(&file)
    }

    /// Manual formatting (fallback when prettyplease is not available)
    fn format_file_manual(&self, file: &File) -> Result<String, FormatterError> {
        use quote::ToTokens;

        let mut output = String::new();

        // Format file-level attributes
        for attr in &file.attrs {
            let attr_str = attr.to_token_stream();
            output.push_str(&format!("{attr_str}\n"));
        }

        if !file.attrs.is_empty() {
            output.push('\n');
        }

        // Format items
        for (i, item) in file.items.iter().enumerate() {
            if i > 0 {
                output.push('\n');
            }

            let item_str = self.format_item_manual(item)?;
            output.push_str(&item_str);
        }

        Ok(output)
    }

    /// Format a single item manually
    fn format_item_manual(&self, item: &Item) -> Result<String, FormatterError> {
        use quote::ToTokens;

        let tokens = item.to_token_stream().to_string();

        // Basic formatting - split on braces and add indentation
        let formatted = self.apply_basic_formatting(&tokens);

        Ok(formatted)
    }

    /// Apply basic formatting rules
    fn apply_basic_formatting(&self, code: &str) -> String {
        let mut output = String::new();
        let mut indent_level = 0;
        let mut in_string = false;
        let mut escape_next = false;

        let indent = " ".repeat(self.config.indent_size);

        for ch in code.chars() {
            if escape_next {
                output.push(ch);
                escape_next = false;
                continue;
            }

            match ch {
                '"' if !in_string => {
                    in_string = true;
                    output.push(ch);
                }
                '"' if in_string => {
                    in_string = false;
                    output.push(ch);
                }
                '\\' if in_string => {
                    escape_next = true;
                    output.push(ch);
                }
                '{' if !in_string => {
                    output.push(ch);
                    output.push('\n');
                    indent_level += 1;
                    output.push_str(&indent.repeat(indent_level));
                }
                '}' if !in_string => {
                    if output.ends_with(&indent) {
                        // Remove the last indent before closing brace
                        for _ in 0..self.config.indent_size {
                            output.pop();
                        }
                    }
                    indent_level = indent_level.saturating_sub(1);
                    output.push(ch);
                    output.push('\n');
                    if indent_level > 0 {
                        output.push_str(&indent.repeat(indent_level));
                    }
                }
                ';' if !in_string => {
                    output.push(ch);
                    output.push('\n');
                    if indent_level > 0 {
                        output.push_str(&indent.repeat(indent_level));
                    }
                }
                ',' if !in_string => {
                    output.push(ch);
                    output.push('\n');
                    if indent_level > 0 {
                        output.push_str(&indent.repeat(indent_level));
                    }
                }
                _ => {
                    output.push(ch);
                }
            }
        }

        // Clean up extra whitespace
        self.cleanup_whitespace(&output)
    }

    /// Clean up extra whitespace and empty lines
    fn cleanup_whitespace(&self, code: &str) -> String {
        code.lines()
            .map(|line| line.trim_end()) // Remove trailing whitespace
            .collect::<Vec<_>>()
            .join("\n")
            .replace("\n\n\n", "\n\n") // Reduce triple newlines to double
    }

    /// Format documentation comments
    pub fn format_doc_comment(&self, doc: &str, indent_level: usize) -> String {
        if !self.config.format_docs {
            return doc.to_string();
        }

        let indent = " ".repeat(self.config.indent_size * indent_level);
        let max_line_width = self.config.max_width.saturating_sub(indent.len() + 4); // Account for "/// "

        let mut formatted = String::new();

        for line in doc.lines() {
            let trimmed = line.trim();
            if trimmed.is_empty() {
                formatted.push_str(&format!("{indent}///\n"));
                continue;
            }

            // Word wrap long lines
            if trimmed.len() <= max_line_width {
                formatted.push_str(&format!("{indent}/// {trimmed}\n"));
            } else {
                let words: Vec<&str> = trimmed.split_whitespace().collect();
                let mut current_line = String::new();

                for word in words {
                    if current_line.len() + word.len() < max_line_width {
                        if !current_line.is_empty() {
                            current_line.push(' ');
                        }
                        current_line.push_str(word);
                    } else {
                        if !current_line.is_empty() {
                            formatted.push_str(&format!("{indent}/// {current_line}\n"));
                        }
                        current_line = word.to_string();
                    }
                }

                if !current_line.is_empty() {
                    formatted.push_str(&format!("{indent}/// {current_line}\n"));
                }
            }
        }

        formatted
    }
}

impl Default for RustFormatter {
    fn default() -> Self {
        Self::new()
    }
}

/// Errors that can occur during formatting
#[derive(Debug, thiserror::Error)]
pub enum FormatterError {
    #[error("Formatting failed: {0}")]
    FormatError(String),

    #[error("Invalid syntax: {0}")]
    SyntaxError(String),

    #[error("IO error: {0}")]
    IoError(#[from] std::io::Error),
}

/// Utility function to format a code string using prettyplease
pub fn format_code_string(code: &str) -> Result<String, FormatterError> {
    let syntax_tree: File =
        syn::parse_str(code).map_err(|e| FormatterError::SyntaxError(e.to_string()))?;

    let formatter = RustFormatter::new();
    formatter.format_file(&syntax_tree)
}

/// Utility function to validate that generated code is syntactically correct
pub fn validate_syntax(code: &str) -> Result<(), FormatterError> {
    syn::parse_str::<File>(code).map_err(|e| FormatterError::SyntaxError(e.to_string()))?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use syn::{parse_quote, ItemStruct};

    #[test]
    fn test_format_simple_struct() {
        let item: ItemStruct = parse_quote! {
            pub struct TestStruct {
                pub x: f32,
                pub y: f32,
            }
        };

        let formatter = RustFormatter::new();
        let result = formatter.format_item(&Item::Struct(item));

        assert!(result.is_ok());
        let formatted = result.unwrap();
        assert!(formatted.contains("pub struct TestStruct"));
        assert!(formatted.contains("pub x: f32"));
    }

    #[test]
    fn test_format_with_attributes() {
        let item: ItemStruct = parse_quote! {
            #[derive(Debug, Clone)]
            pub struct TestStruct {
                pub field: i32,
            }
        };

        let formatter = RustFormatter::new();
        let result = formatter.format_item(&Item::Struct(item));

        assert!(result.is_ok());
        let formatted = result.unwrap();
        assert!(formatted.contains("#[derive(Debug, Clone)]"));
    }

    #[test]
    fn test_doc_comment_formatting() {
        let formatter = RustFormatter::new();
        let doc = "This is a very long documentation comment that should be wrapped to multiple lines when it exceeds the maximum line width.";

        let formatted = formatter.format_doc_comment(doc, 0);

        assert!(formatted.contains("///"));
        // Should be split into multiple lines
        assert!(formatted.matches('\n').count() > 1);
    }

    #[test]
    fn test_doc_comment_with_paragraphs() {
        let formatter = RustFormatter::new();
        let doc = "First paragraph.\n\nSecond paragraph with more content.";

        let formatted = formatter.format_doc_comment(doc, 0);

        assert!(formatted.contains("/// First paragraph"));
        assert!(formatted.contains("///\n")); // Empty line
        assert!(formatted.contains("/// Second paragraph"));
    }

    #[test]
    fn test_validate_syntax_valid() {
        let code = r#"
            pub struct TestStruct {
                pub field: i32,
            }
        "#;

        assert!(validate_syntax(code).is_ok());
    }

    #[test]
    fn test_validate_syntax_invalid() {
        let code = r#"
            pub struct TestStruct {
                pub field: i32,
                pub field2 // Missing type and comma
            }
        "#;

        assert!(validate_syntax(code).is_err());
    }

    #[test]
    fn test_format_code_string() {
        let code = "pub struct Test{pub x:i32,pub y:i32}";

        let result = format_code_string(code);
        assert!(result.is_ok());

        let formatted = result.unwrap();
        assert!(formatted.contains("pub struct Test"));
        assert!(formatted.contains("pub x: i32"));
        assert!(formatted.contains("pub y: i32"));
    }

    #[test]
    fn test_custom_format_config() {
        let config = FormatConfig {
            use_prettyplease: false,
            indent_size: 2,
            max_width: 80,
            format_docs: true,
        };

        let formatter = RustFormatter::with_config(config);

        // Test that the formatter uses the custom configuration
        let doc = "Short doc";
        let formatted = formatter.format_doc_comment(doc, 1);

        // Should use 2-space indentation
        assert!(formatted.starts_with("  ///"));
    }

    #[test]
    fn test_cleanup_whitespace() {
        let formatter = RustFormatter::new();
        let code = "line1   \nline2\n\n\nline3";

        let cleaned = formatter.cleanup_whitespace(code);

        // Should remove trailing whitespace and reduce triple newlines
        assert_eq!(cleaned, "line1\nline2\n\nline3");
    }

    #[test]
    fn test_basic_formatting() {
        let formatter = RustFormatter::new();
        let code = "fn test(){let x=1;let y=2;}";

        let formatted = formatter.apply_basic_formatting(code);

        // Should add newlines and indentation
        assert!(formatted.contains("{\n"));
        assert!(formatted.ends_with("}")); // } at the end without newline
        assert!(formatted.contains(";\n"));
    }
}
