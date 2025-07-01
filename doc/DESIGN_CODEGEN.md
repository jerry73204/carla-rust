# Code Generation Design

## Overview

The CARLA Rust client uses a sophisticated code generation system to automatically create Rust type definitions from CARLA's Python API YAML specifications. This document details the design and implementation of the code generation toolkit.

## Architecture

### Core Components

```
carla-codegen/
├── src/
│   ├── ast/                    # AST builders (syn-based)
│   │   ├── struct_builder.rs   # Struct generation
│   │   ├── impl_builder.rs     # Method implementation
│   │   ├── trait_builder.rs    # Trait derivation
│   │   ├── doc_builder.rs      # Documentation generation
│   │   └── formatter.rs        # Code formatting
│   ├── analyzer/               # Type analysis and resolution
│   ├── generator/              # High-level code generation
│   ├── parser/                 # YAML parsing
│   └── bin/cli.rs             # Command-line interface
```

### Integration Points

1. **Standalone CLI**: `carla-codegen` binary for manual code generation
2. **Build Integration**: Used by `carla-sys/build.rs` during compilation
3. **Pre-generated Code**: Outputs stored in `carla-sys/src/generated/`

## Design Philosophy

### 1. Type-Safe Code Generation

**Syn-Based AST Construction**:
- All code generation uses `syn` crate for AST construction
- Compile-time validation of generated code structure
- Better error messages and IDE support
- Eliminates template syntax errors

**Example**:
```rust
// Type-safe struct generation
let struct_def: ItemStruct = parse_quote! {
    #[derive(Debug, Clone)]
    pub struct Actor {
        pub id: u32,
        pub type_id: String,
    }
};
```

### 2. Dual Code Paths

**FFI Implementation**:
- Real C++ bindings for production use
- Calls through CXX bridge to CARLA library
- Requires CARLA source and build environment

**Stub Implementation**:
- Documentation-only placeholders
- Uses `unreachable!()` for unimplemented functionality
- Enables docs.rs builds without CARLA dependencies

### 3. Intelligent Type Resolution

**Python → Rust Mapping**:
```python
# Python API YAML
- name: "get_velocity"
  return_type: "carla.Vector3D"
  parameters:
    - name: "actor_id"
      type: "int"
```

**Generated Rust**:
```rust
pub fn get_velocity(&self, actor_id: u32) -> crate::Result<Vector3D> {
    todo!("get_velocity not yet implemented - missing FFI function Actor_GetVelocity")
}
```

### 4. Comprehensive Documentation

**Auto-Generated Docs**:
- Method signatures with parameter descriptions
- Return type documentation
- Usage examples and warnings
- FFI implementation status

## Core Builders

### StructBuilder

**Purpose**: Generate Rust struct definitions from CARLA class specifications

**Key Features**:
- Automatic trait derivation based on field types
- Rich documentation with examples
- Field-level validation and warnings
- Support for complex nested types

**Example Usage**:
```rust
let field = StructField::new(
    to_rust_ident("velocity"),
    RustType::Custom("Vector3D".to_string())
)
.with_doc("Current velocity vector".to_string())
.with_units("m/s".to_string());

let struct_def = StructBuilder::new(to_rust_type_name("Actor"), context)
    .with_doc("Represents a CARLA actor".to_string())
    .add_field(field)
    .build()?;
```

### ImplBuilder

**Purpose**: Generate method implementations for CARLA classes

**Key Features**:
- Automatic FFI stub generation
- Parameter validation and documentation
- Return type handling with Result wrapping
- Support for async and unsafe methods

**Generated Method Pattern**:
```rust
impl Actor {
    /// Gets the actor's current velocity vector.
    ///
    /// # Returns
    /// The velocity vector in m/s
    ///
    /// # FFI Status
    /// ⚠️ Not yet implemented - requires FFI function Actor_GetVelocity
    pub fn get_velocity(&self) -> crate::Result<Vector3D> {
        // TODO: Implement using carla-sys FFI interface
        // This requires adding Actor_GetVelocity FFI function
        todo!("get_velocity not yet implemented - missing FFI function Actor_GetVelocity")
    }
}
```

### DocBuilder

**Purpose**: Generate comprehensive documentation for all generated code

**Documentation Sections**:
- **Description**: Method/type purpose and behavior
- **Parameters**: Detailed parameter descriptions with types
- **Returns**: Return value semantics and types
- **Examples**: Usage examples (marked `no_run` for CARLA server dependency)
- **FFI Status**: Implementation status and required FFI functions
- **Warnings**: Important usage notes and limitations

**Example Generated Documentation**:
```rust
/// Spawns a new actor in the world.
///
/// # Parameters
/// - `blueprint`: The actor blueprint to spawn
/// - `transform`: Initial position and rotation
/// - `attach_to`: Optional parent actor for attachment
///
/// # Returns
/// The spawned actor instance
///
/// # Example
/// ```rust,no_run
/// let blueprint = world.get_blueprint_library()
///     .find("vehicle.tesla.model3")?;
/// let transform = Transform::default();
/// let actor = world.spawn_actor(blueprint, transform, None)?;
/// ```
///
/// # FFI Status
/// ⚠️ Not yet implemented - requires FFI function World_SpawnActor
///
/// # Warnings
/// - The spawned actor must be explicitly destroyed to prevent memory leaks
/// - Spawning may fail if the location is occupied
pub fn spawn_actor(&self, blueprint: &Blueprint, transform: Transform, attach_to: Option<&Actor>) -> crate::Result<Actor> {
    todo!("spawn_actor not yet implemented - missing FFI function World_SpawnActor")
}
```

### TraitBuilder

**Purpose**: Intelligently derive traits based on struct field types

**Derivation Rules**:
- `Debug`: Always derived for debugging support
- `Clone`: Derived for value types, avoided for resource handles
- `PartialEq`: Derived for comparable types (avoid for f32/f64)
- `Default`: Derived when all fields have defaults
- `Send`/`Sync`: Derived based on field thread safety

**Example**:
```rust
// Geometry type - safe for all standard traits
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Vector3D {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

// Resource handle - limited trait derivation
#[derive(Debug)]
pub struct Actor {
    inner: SharedPtr<ffi::Actor>,
}
```

## Code Generation Pipeline

### Phase 1: YAML Parsing

**Input**: CARLA Python API YAML files
**Process**:
1. Parse YAML structure into intermediate representation
2. Build class inheritance hierarchy
3. Resolve type dependencies and imports
4. Validate class and method definitions

**Key Challenges**:
- Handling circular dependencies between types
- Resolving inheritance relationships
- Managing module-level imports and exports

### Phase 2: Type Resolution

**Python Type Mapping**:
```yaml
# Python API types → Rust types
int → u32
float → f32
str → String
bool → bool
list → Vec<T>
dict → HashMap<K, V>
carla.Vector3D → crate::geom::Vector3D
```

**Complex Type Handling**:
- Optional parameters: `Option<T>`
- Collections: `Vec<T>`, `HashMap<K, V>`
- References: `&T`, `&mut T`
- Custom CARLA types: Module-qualified paths

### Phase 3: AST Generation

**Struct Generation**:
1. Create struct definition with fields
2. Generate appropriate trait derives
3. Add comprehensive documentation
4. Validate field names and types

**Method Generation**:
1. Parse method signature and parameters
2. Map Python types to Rust equivalents
3. Generate FFI stub with todo!() placeholder
4. Create rich documentation with examples

### Phase 4: Code Output

**Dual Implementation Strategy**:

**FFI Variant** (`generated/ffi/`):
```rust
pub fn get_velocity(&self) -> crate::Result<Vector3D> {
    // Real FFI implementation
    let result = unsafe {
        carla_sys::ffi::Actor_GetVelocity(self.inner.as_ptr())
    };
    Ok(Vector3D::from_ffi(result))
}
```

**Stub Variant** (`generated/stubs/`):
```rust
pub fn get_velocity(&self) -> crate::Result<Vector3D> {
    // Documentation-only implementation
    unreachable!("FFI stub - not intended for execution")
}
```

## Build System Integration

### Feature-Controlled Generation

**Default Build** (pre-generated code):
```bash
cargo build  # Uses src/generated/ffi/
```

**Maintainer Regeneration**:
```bash
export CARLA_ROOT=/path/to/carla-source
cargo build --features save-bindgen  # Regenerates all code
```

**Documentation Build**:
```bash
cargo build --features docs-only  # Uses src/generated/stubs/
```

### Build Script Integration

```rust
// In carla-sys/build.rs
fn generate_rust_types(source_dir: &Path) -> anyhow::Result<()> {
    // Skip generation if using pre-generated code
    if !cfg!(feature = "save-bindgen") {
        println!("cargo:warning=Using pre-generated code from src/generated/");
        return Ok(());
    }
    
    // Run carla-codegen to regenerate types
    let codegen = carla_codegen::CodeGenerator::new()
        .with_source_dir(source_dir)
        .with_output_dir("src/generated");
    
    codegen.generate_all()?;
    Ok(())
}
```

## Error Handling and Validation

### Compile-Time Validation

**AST Validation**:
- Struct field name uniqueness
- Method parameter name conflicts
- Type reference validation
- Identifier naming conventions

**Example Validation**:
```rust
fn validate(&self) -> crate::Result<()> {
    // Check for duplicate field names
    let mut field_names = std::collections::HashSet::new();
    for field in &self.fields {
        let name = field.name.to_string();
        if !field_names.insert(name.clone()) {
            return Err(AstError::InvalidIdentifier(
                format!("Duplicate field name: {name}")
            ).into());
        }
    }
    Ok(())
}
```

### Runtime Error Handling

**Clear Error Messages**:
```rust
#[derive(Debug, thiserror::Error)]
pub enum CodegenError {
    #[error("YAML parsing failed: {source}")]
    YamlError { source: serde_yaml::Error },
    
    #[error("Type resolution failed for '{type_name}': {reason}")]
    TypeResolutionError { type_name: String, reason: String },
    
    #[error("AST generation failed: {source}")]
    AstError { source: AstError },
}
```

## Performance Considerations

### Compilation Speed

**Pre-generated Code Benefits**:
- No YAML parsing during normal builds
- No AST generation overhead
- Faster incremental compilation
- Reduced build dependencies

**Optimization Strategies**:
- Parallel code generation for multiple modules
- Incremental regeneration based on YAML timestamps
- Caching of intermediate representations
- Minimal dependency graphs

### Memory Usage

**Efficient AST Construction**:
- Reuse of common AST nodes
- Lazy evaluation of complex types
- Streaming code output to avoid memory buildup
- Cleanup of intermediate data structures

## Testing Strategy

### Unit Tests

**Builder Testing**:
```rust
#[test]
fn test_struct_builder_with_complex_types() {
    let field = StructField::new(
        to_rust_ident("location"),
        RustType::Option(Box::new(RustType::Custom("Location".to_string())))
    );
    
    let builder = StructBuilder::new(to_rust_type_name("Actor"), context)
        .add_field(field);
    
    let result = builder.build().expect("Failed to build struct");
    let tokens = result.to_token_stream().to_string();
    
    assert!(tokens.contains("location : Option < Location >"));
}
```

### Integration Tests

**End-to-End Generation**:
- Parse real CARLA YAML files
- Generate complete Rust modules
- Compile generated code
- Verify documentation quality

### Property-Based Testing

**Invariant Verification**:
- Generated code always compiles
- All public methods have documentation
- Field names follow Rust conventions
- Type mappings are consistent

## Future Enhancements

### Advanced Code Generation

**Planned Features**:
- Custom derive macro generation
- Advanced trait implementations (From, Into, etc.)
- Async method generation
- Generic type parameter support

### Integration Improvements

**Development Experience**:
- IDE integration for code generation
- Incremental regeneration workflows
- Better error reporting and diagnostics
- Interactive code generation CLI

### Performance Optimizations

**Build Speed**:
- Parallel code generation
- Smarter caching strategies
- Reduced redundant work
- Optimized dependency management

## Conclusion

The code generation system provides a robust, type-safe foundation for creating Rust bindings to CARLA's extensive API. By combining pre-generated code for reliability with intelligent AST construction for maintainability, it enables both rapid development and production-ready code quality.

The dual implementation strategy (FFI vs stubs) ensures the library can be used in various contexts while maintaining a consistent API surface. The comprehensive documentation generation helps bridge the gap between CARLA's Python-centric documentation and Rust developer expectations.