# CARLA Code Generator - Error Reporting Design

## Overview

This document outlines a practical error reporting system for the CARLA code generator (`carla-codegen`) focused on improving the development experience for carla-codegen maintainers and contributors. The goal is to provide clear, actionable error messages with reasonable defaults and simple customization options.

## Current Challenges

Based on recent development experience, the current error reporting has several limitations:

1. **Generic Error Messages**: Errors like "Syn parsing error: unexpected token" provide no context about the source or cause
2. **No Source Tracing**: Cannot trace errors back to specific YAML files or lines  
3. **Missing Generated Code Context**: When generated code fails to parse, the offending code is not shown
4. **Unclear Pipeline Stage**: Difficult to determine which stage of the pipeline caused the failure
5. **Poor Developer Experience**: Debugging requires manual investigation of generated files and verbose logs

## Design Goals

### Primary Goals
- **Source Attribution**: Every error should be traceable to the original YAML source
- **Code Context**: Show the problematic generated code alongside parsing errors
- **Pipeline Visibility**: Clear indication of which processing stage failed
- **Simple Implementation**: Minimal code changes, leverage existing infrastructure
- **Developer Efficiency**: Reduce debugging time for carla-codegen development

### Secondary Goals  
- **Error Recovery**: Continue processing when possible to surface multiple errors
- **Reasonable Defaults**: Work well out-of-the-box with minimal configuration
- **CLI Customization**: Simple command-line options for different verbosity levels

## Error Classification

### 1. YAML Parsing Errors
**Source**: YAML file structure issues
**Examples**:
- Invalid YAML syntax
- Missing required fields
- Incorrect field types

### 2. Type Resolution Errors
**Source**: Python to Rust type mapping failures
**Examples**:
- Unknown Python type (e.g., `callback`)
- Unsupported type patterns
- Circular type dependencies

### 3. AST Generation Errors
**Source**: Rust AST construction failures
**Examples**:
- Invalid Rust syntax in generated code
- Syn parsing failures
- Conflicting trait implementations

### 4. Code Generation Errors
**Source**: Template rendering and file output issues
**Examples**:
- File I/O errors
- Template compilation failures
- Rust formatting errors

### 5. Validation Errors
**Source**: Generated code validation failures
**Examples**:
- Rust compilation errors
- Lint failures
- Test failures

## Simple Error Context

### Minimal Source Location

```rust
#[derive(Debug, Clone)]
pub struct SourceContext {
    /// YAML file path
    pub yaml_file: PathBuf,
    /// Class name being processed
    pub class_name: String,
    /// Method name if applicable
    pub method_name: Option<String>,
    /// Parameter name if applicable  
    pub param_name: Option<String>,
    /// Python type that caused the issue
    pub python_type: Option<String>,
}
```

### Generated Code Context

```rust
#[derive(Debug, Clone)]
pub struct CodeContext {
    /// The generated Rust code snippet that failed
    pub code_snippet: String,
    /// Target file path
    pub target_file: PathBuf,
    /// What was being generated
    pub construct: String, // e.g., "ActorAttribute::eq method"
}
```

### Pipeline Stage

```rust
#[derive(Debug, Clone, Copy)]
pub enum Stage {
    YamlParsing,
    TypeResolution,
    AstGeneration,
    CodeFormatting,
    FileOutput,
}
```

## Simplified Error Types

### Core Error Structure

```rust
#[derive(Debug, thiserror::Error)]
pub enum CodegenError {
    #[error("Type '{python_type}' not found in type mappings")]
    UnknownType {
        python_type: String,
        context: SourceContext,
    },

    #[error("Syn parsing error: {message}")]
    SynError {
        message: String,
        context: SourceContext,
        generated_code: Option<CodeContext>,
        #[source]
        syn_error: syn::Error,
    },

    #[error("YAML parsing failed: {message}")]
    YamlError {
        message: String,
        file: PathBuf,
        #[source]
        yaml_error: serde_yaml::Error,
    },

    #[error("Code generation failed: {message}")]
    GenerationError {
        message: String,
        context: SourceContext,
        stage: Stage,
    },
}
```

## Default Configuration & Customization

### Default Error Reporting Configuration

```rust
#[derive(Debug, Clone)]
pub struct ErrorConfig {
    /// Show generated code context in errors (default: true)
    pub show_generated_code: bool,
    /// Show YAML source context (default: true)  
    pub show_source_context: bool,
    /// Continue processing after errors (default: false)
    pub continue_on_error: bool,
    /// Maximum number of errors before stopping (default: 10)
    pub max_errors: usize,
    /// Use colored output (default: auto-detect TTY)
    pub use_colors: bool,
}

impl Default for ErrorConfig {
    fn default() -> Self {
        Self {
            show_generated_code: true,
            show_source_context: true,
            continue_on_error: false,
            max_errors: 10,
            use_colors: atty::is(atty::Stream::Stderr),
        }
    }
}
```

### CLI Customization Options

The CLI will support these flags for error reporting:

```bash
# Basic error reporting (default)
cargo run -- generate --input docs/ --output generated/

# Minimal error output
cargo run -- generate --input docs/ --output generated/ --quiet

# Verbose error output with generated code
cargo run -- generate --input docs/ --output generated/ --verbose

# Continue on errors to collect all issues
cargo run -- generate --input docs/ --output generated/ --continue-on-error

# Disable colors
cargo run -- generate --input docs/ --output generated/ --no-color

# Show first 5 errors only
cargo run -- generate --input docs/ --output generated/ --max-errors 5
```

### API Customization

For programmatic use:

```rust
use carla_codegen::{Generator, ErrorConfig};

let config = ErrorConfig {
    show_generated_code: false,  // Don't show code snippets
    continue_on_error: true,     // Process all files
    max_errors: 50,              // Allow more errors
    ..Default::default()
};

let mut generator = Generator::new()
    .with_error_config(config);

match generator.generate() {
    Ok(()) => println!("Generation completed successfully"),
    Err(errors) => {
        eprintln!("Generation failed with {} errors:", errors.len());
        for error in errors {
            eprintln!("{}", error);
        }
    }
}
```

## Error Message Examples

### Example 1: Type Resolution Error

```
Error: Type resolution failed for 'callback'
  ┌─ ../carla-simulator/PythonAPI/docs/sensor.yml:89:12
  │
89│       - param_name: callback
  │                     ^^^^^^^^ Unknown Python type
90│         type: callback
  │               ^^^^^^^^ 
  │
  = Stage: Type resolution for Sensor.listen parameter 'callback'
  = Source: carla.Sensor.__init__.callback
  = Help: Add a type mapping for 'callback' in your configuration:
          [type_mapping.mappings]
          "callback" = "Box<dyn Fn() + Send + Sync>"
```

### Example 2: AST Generation Error

```
Error: Syn parsing error: unexpected token
  Source: blueprint.yml -> ActorAttribute.__eq__ parameter 'other'
  Python type: bool / int / float / str / carla.Color / carla.ActorAttribute
  Stage: AST generation

  Generated code that failed:
  pub fn eq(&self, other: bool | i32 | f32 | String | crate::rpc::Color | crate::client::ActorAttribute) -> Result<bool> {
                          ^^^^ unexpected token

  Target file: test_temp/carla/actor_attribute.rs
  Context: ActorAttribute::eq method

  Help: Union types in parameters are not supported in Rust syntax.
```

### Example 3: Simple Type Error (Default Output)

```
Error: Type 'callback' not found in type mappings
  Source: sensor.yml -> Sensor.listen parameter 'callback'
  
  Help: Add a type mapping in your configuration:
  [type_mapping.mappings]
  "callback" = "Box<dyn Fn() + Send + Sync>"
```

## Implementation Strategy

### Phase 1: Minimal Implementation (1-2 days)
1. **Enhanced Error Types**: Add `SourceContext` and `CodeContext` to existing error types
2. **CLI Integration**: Add `--continue-on-error`, `--quiet`, `--verbose` flags 
3. **Basic Context Tracking**: Track current class/method being processed
4. **Simple Error Formatting**: Improve error message format with source context

### Phase 2: Code Context (1-2 days)  
1. **Generated Code Capture**: Capture failing generated code snippets in AST errors
2. **Stage Tracking**: Add simple stage tracking to show where errors occur
3. **Error Recovery**: Implement continue-on-error for batch processing
4. **Better Help Messages**: Add common solution suggestions

### Implementation Notes
- **Leverage Existing Infrastructure**: Build on current error handling and tracing
- **Minimal Code Changes**: Add context to existing error paths, don't rewrite
- **Focus on Development**: Optimize for carla-codegen developers, not end users
- **No External Dependencies**: Use existing crates (anyhow, thiserror, tracing)

## Default Configuration

No configuration file needed. Reasonable defaults with CLI overrides:

```rust
// Built-in defaults - no config file required
ErrorConfig {
    show_generated_code: true,    // Always helpful for debugging
    show_source_context: true,    // Always show YAML source info  
    continue_on_error: false,     // Fail fast by default
    max_errors: 10,              // Reasonable limit
    use_colors: auto_detect,     // Auto-detect terminal support
}
```

CLI can override any default:
- `--quiet`: Minimal output, only final error count
- `--verbose`: Show generated code snippets  
- `--continue-on-error`: Process all files
- `--max-errors N`: Set error limit
- `--no-color`: Disable colored output

## Benefits for carla-codegen Development

### Immediate Benefits (Phase 1)
- **Faster Debugging**: Know exactly which YAML class/method caused the error
- **Clear Error Source**: "blueprint.yml -> ActorAttribute.__eq__" instead of generic messages
- **Batch Processing**: `--continue-on-error` to see all issues at once
- **Better CLI Experience**: `--quiet` for CI, `--verbose` for debugging

### Enhanced Benefits (Phase 2)
- **Code Context**: See the actual generated Rust code that failed to parse
- **Stage Visibility**: Know if error is in type resolution vs AST generation vs formatting
- **Solution Hints**: Common fixes for type mapping and syntax issues
- **Error Recovery**: Process entire YAML suite to find all problems

### Development Workflow Improvements
1. **Before**: Generic "syn parsing error" requires manual investigation
2. **After**: "ActorAttribute.__eq__ parameter 'other' has union type 'bool | int | ...' which generates invalid Rust syntax"

3. **Before**: Debug by examining generated files and verbose logs  
4. **After**: Error shows both YAML source and failing generated code snippet

5. **Before**: Fix one error, regenerate, find next error (slow cycle)
6. **After**: `--continue-on-error` shows all errors in one run

## Scope Limitations

### What This Design **Does Not** Include
- **IDE Integration**: No VS Code extensions, language servers, or editor plugins
- **End User Features**: Not designed for users of generated Rust code  
- **Complex Analytics**: No error statistics, pattern analysis, or machine learning
- **Auto-Fix**: No automatic code generation or configuration patching
- **External Validation**: No integration with rustc, clippy, or test runners

### What This Design **Focuses On**
- **carla-codegen Development**: Helping people who modify and maintain the code generator
- **Simple Implementation**: Minimal code changes, leverage existing infrastructure
- **Practical Debugging**: Real problems encountered during recent union type debugging
- **Reasonable Defaults**: Works well out-of-the-box, simple CLI customization

## Implementation Effort

**Phase 1**: ~2-3 days of work
- Add context structs to existing error types
- Thread source context through generation pipeline  
- Add CLI flags and simple formatting improvements

**Phase 2**: ~2-3 days of work
- Capture generated code snippets in AST builder errors
- Add stage tracking and error recovery
- Improve help messages with common solutions

**Total**: ~1 week of focused work for significant debugging improvements

This is a practical, focused enhancement that directly addresses real development pain points without over-engineering or scope creep.