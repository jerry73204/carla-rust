# CARLA Python API to Rust Type Conversion Plan

## Overview

This document outlines a plan to create a tool that converts CARLA's Python API YAML specifications into Rust type definitions. The Python API is documented in YAML files located at `carla-simulator/PythonAPI/docs/`, which describe classes, methods, properties, and their relationships.

**Architecture Update**: The code generation functionality has been moved from the `carla` crate to the `carla-sys` crate to centralize CARLA source dependencies and improve build architecture.

## Quick Status

**Overall Progress**: ✅ Phase 9 Complete, ✅ Phase 10 Complete

**Current Phase**: All phases completed successfully!

**Total Work Items**: 75 tasks across 10 phases (8 new tasks for carla-sys migration)

**Timeline**: 8-9 weeks (carla-sys migration adds 1 week)

**Key Milestones**:
- [x] Core `carla-codegen` crate setup (Phase 1) ✅
- [x] YAML parser implementation (Phase 2) ✅
- [x] Type mapping system (Phase 3) ✅
- [x] Code generation working (Phase 4) ✅
- [x] Configuration system (Phase 5) ✅
- [x] Standalone CLI tool (Phase 6) ✅
- [x] Full test coverage (Phase 7) ✅
- [x] Integration with `carla` crate (Phase 8) ✅
- [x] Migration to carla-sys (Phase 9) ✅
- [x] Syn refactoring (Phase 10) ✅

## Architecture Summary

The implementation follows a phased approach:

### Phase 1-7: Standalone `carla-codegen` Tool
First, we develop `carla-codegen` as a complete standalone code generation tool:
- Independent library and CLI binary
- Can be used manually to generate Rust code from YAML
- Full test coverage and documentation
- No dependencies on the `carla` crate

### Phase 8: Integration with `carla`
Initially integrated with the `carla` crate:
- Use `carla-codegen` as a build dependency
- Call it from `build.rs` for automatic generation
- Generated code lives in `OUT_DIR`
- Seamless integration with existing manual code

### Phase 9: Migration to `carla-sys` (NEW)
Move code generation from `carla` to `carla-sys` crate:
- Centralize CARLA source dependencies in one place
- Generate low-level FFI bindings alongside C++ build
- Export generated types through `carla_sys::generated` module
- Simplify build process with single CARLA_ROOT handling
- Better separation of concerns between sys and high-level API
- Use pre-generated code approach for faster builds

### Phase 10: Syn Refactoring
Refactor code generation to use only syn-based AST generation:
- Remove Tera templates completely
- Use type-safe Rust AST construction throughout
- Improve error handling and code quality validation
- Enable advanced features like automatic trait derivation
- Provide better IDE support and debugging capabilities
- Simplify codebase with single rendering approach

This phased approach ensures:
- The codegen tool is fully tested before integration
- Can be used independently for debugging/development
- Reduces complexity during initial development
- Allows for manual code generation if needed
- Migration to syn provides better maintainability and type safety
- carla-sys migration creates cleaner architecture

## Python API Documentation Structure

### YAML File Organization
The Python API is documented in the following YAML files:
- `actor.yml` - Actor base class and derived types
- `blueprint.yml` - Blueprint system for spawning actors
- `client.yml` - Client connection and commands
- `commands.yml` - Batch command definitions
- `control.yml` - Vehicle and walker control structures
- `geom.yml` - Geometry types (Vector, Location, Transform, etc.)
- `light_manager.yml` - Light management system
- `map.yml` - Map, waypoints, and road network
- `osm2odr.yml` - OpenStreetMap to OpenDRIVE conversion
- `sensor.yml` - Sensor types and configurations
- `sensor_data.yml` - Sensor data structures
- `snapshot.yml` - World snapshot data
- `weather.yml` - Weather parameters
- `world.yml` - World management and queries

### YAML Schema Structure

Each YAML file follows this structure:
```yaml
---
- module_name: carla
  classes:
  - class_name: ClassName
    doc: >
      Class documentation
    instance_variables:
    - var_name: property_name
      type: property_type
      doc: >
        Property documentation
    methods:
    - def_name: method_name
      return: return_type  # optional
      params:
      - param_name: parameter_name
        type: parameter_type
        default: default_value  # optional
        param_units: units  # optional
        doc: >
          Parameter documentation
      doc: >
        Method documentation
      warning: >  # optional
        Warning text
      note: >  # optional
        Note text
```

## Conversion Tool Architecture

### 1. YAML Parser Component
**Purpose**: Parse YAML files and build an intermediate representation

**Key Features**:
- Parse YAML structure into Rust data structures
- Handle special types (carla.Vector3D, list(type), dict, etc.)
- Extract documentation strings
- Capture method signatures and parameter metadata

**Data Structures**:
```rust
#[derive(Debug, Clone)]
struct CarlaModule {
    name: String,
    classes: Vec<CarlaClass>,
}

#[derive(Debug, Clone)]
struct CarlaClass {
    name: String,
    doc: Option<String>,
    parent: Option<String>,
    instance_variables: Vec<InstanceVariable>,
    methods: Vec<Method>,
}

#[derive(Debug, Clone)]
struct InstanceVariable {
    name: String,
    type_info: TypeInfo,
    doc: Option<String>,
}

#[derive(Debug, Clone)]
struct Method {
    name: String,
    return_type: Option<TypeInfo>,
    params: Vec<Parameter>,
    doc: Option<String>,
    warning: Option<String>,
    note: Option<String>,
}
```

### 2. Type Mapping Component
**Purpose**: Map Python types to Rust types

**Type Mappings**:
| Python Type       | Rust Type                          |
|-------------------|------------------------------------|
| `int`             | `i32` or `u32` (context-dependent) |
| `float`           | `f32` or `f64`                     |
| `str`             | `String` or `&str`                 |
| `bool`            | `bool`                             |
| `list(T)`         | `Vec<T>`                           |
| `dict`            | `HashMap<String, T>`               |
| `carla.Vector3D`  | `crate::geom::Vector3D`            |
| `carla.Location`  | `crate::geom::Location`            |
| `carla.Transform` | `crate::geom::Transform`           |
| `carla.Actor`     | `crate::actor::Actor`              |

**Special Cases**:
- Nullable types: Use `Option<T>`
- Callback types: Use function pointers or trait objects
- Enums: Generate Rust enums from known value sets
- Units: Add as documentation comments

### 3. Code Generation Component
**Purpose**: Generate idiomatic Rust code from the intermediate representation

**Generated Code Structure**:
```rust
/// Class documentation from YAML
#[derive(Debug, Clone)]
pub struct ClassName {
    /// Property documentation
    pub property_name: PropertyType,
}

impl ClassName {
    /// Method documentation
    /// 
    /// # Arguments
    /// 
    /// * `param_name` - Parameter documentation
    /// 
    /// # Returns
    /// 
    /// Return value documentation
    pub fn method_name(&self, param_name: ParamType) -> ReturnType {
        todo!("Implement using FFI")
    }
}
```

### 4. FFI Integration Component
**Purpose**: Generate FFI bindings that connect to carla-sys

**Features**:
- Generate extern "C" function declarations
- Create safe wrappers around FFI calls
- Handle ownership and lifetime issues
- Convert between Rust and C++ types

### 5. Validation Component
**Purpose**: Ensure generated code is correct and complete

**Validation Steps**:
- Compare against existing Rust implementation
- Check for missing types or methods
- Validate type conversions
- Ensure documentation completeness

## Design Choices

### Builder Pattern for Kwargs
Python's keyword arguments (kwargs) don't translate directly to Rust. We'll use the builder pattern for methods with optional parameters:

**Python API**:
```python
world.spawn_actor(blueprint, transform, attach_to=parent, attachment=rigid)
```

**Generated Rust API**:
```rust
world.spawn_actor(blueprint, transform)
    .attach_to(parent)
    .attachment(AttachmentType::Rigid)
    .execute()?;
```

**Builder Implementation**:
```rust
pub struct SpawnActorBuilder<'a> {
    world: &'a World,
    blueprint: ActorBlueprint,
    transform: Transform,
    attach_to: Option<&'a Actor>,
    attachment_type: AttachmentType,
}

impl<'a> SpawnActorBuilder<'a> {
    pub fn attach_to(mut self, parent: &'a Actor) -> Self {
        self.attach_to = Some(parent);
        self
    }
    
    pub fn attachment(mut self, attachment_type: AttachmentType) -> Self {
        self.attachment_type = attachment_type;
        self
    }
    
    pub fn execute(self) -> CarlaResult<Actor> {
        // FFI call with all parameters
    }
}
```

### Configuration File Design

**Configuration Schema** (`carla-codegen.toml`):
```toml
[generation]
# Output directory for generated code
output_dir = "src/generated"

# Generate builder patterns for methods with >2 optional params
builder_threshold = 2

# Generate async versions of methods
generate_async = false

# Module organization
module_structure = "nested" # or "flat"

[filters]
# Include/exclude specific classes
include_classes = ["Actor", "Vehicle", "World"]
exclude_classes = ["ObsoleteClass"]

# Include/exclude specific methods
include_methods = ["spawn_actor", "destroy"]
exclude_methods = ["deprecated_method"]

# Skip generation for specific modules
skip_modules = ["osm2odr"]

[type_mapping]
# Custom type mappings
"int" = "i32"
"float" = "f32"
"carla.Vector3D" = "crate::geom::Vector3D"

# Special handling for specific types
[type_mapping.special]
"actor_id" = "u32"  # Always use u32 for actor IDs
"timestamp" = "f64" # Use f64 for timestamps

[naming]
# Naming conventions
method_case = "snake_case"
type_case = "PascalCase"
module_case = "snake_case"

# Prefix/suffix rules
remove_prefix = ["get_", "set_"]
actor_suffix = "Actor"

[documentation]
# Documentation generation options
include_warnings = true
include_notes = true
include_python_examples = false
generate_doc_tests = true

[validation]
# Validation rules
check_existing_impl = true
warn_missing_methods = true
error_on_type_mismatch = false
```

### Target Architecture

**Crate Name**: `carla-codegen`

**Workspace Integration**:
Add to root `Cargo.toml`:
```toml
[workspace]
members = [
    "carla",
    "carla-sys", 
    "carla-src",
    "carla-codegen",  # Code generation library
]
```

**Project Structure**:

```
carla-codegen/              # Code generation library crate
├── Cargo.toml
├── README.md
├── src/
│   ├── lib.rs              # Public API for code generation
│   ├── parser/
│   │   ├── mod.rs          # YAML parsing module
│   │   ├── yaml_schema.rs  # YAML schema definitions
│   │   └── validator.rs    # Schema validation
│   ├── analyzer/
│   │   ├── mod.rs          # Code analysis module
│   │   ├── type_resolver.rs # Type resolution logic
│   │   ├── dependency.rs   # Dependency analysis
│   │   └── inheritance.rs  # Class hierarchy analysis
│   ├── generator/
│   │   ├── mod.rs          # Code generation module
│   │   ├── rust_types.rs   # Rust type generation
│   │   ├── builders.rs     # Builder pattern generation
│   │   ├── traits.rs       # Trait generation
│   │   └── ffi.rs          # FFI wrapper generation
│   ├── config/
│   │   ├── mod.rs          # Configuration module
│   │   ├── schema.rs       # Config file schema
│   │   └── loader.rs       # Config file loader
│   └── templates/          # Code templates
│       ├── struct.rs.tera  # Struct template
│       ├── impl.rs.tera    # Implementation template
│       ├── builder.rs.tera # Builder template
│       └── module.rs.tera  # Module template
├── templates/              # Tera templates for code generation
├── tests/
│   ├── fixtures/          # Test YAML files
│   └── integration/       # Integration tests
└── examples/
    └── generate.rs        # Example usage

carla-sys/                 # FFI layer with code generation
├── Cargo.toml
├── build.rs              # Uses carla-codegen AND builds C++ library
├── carla-codegen.toml    # Configuration for code generation
├── src/
│   ├── lib.rs            # FFI exports and generated types
│   ├── ffi.rs            # CXX FFI definitions
│   ├── generated/        # Generated low-level types
│   │   ├── mod.rs        # Module declarations
│   │   ├── actors.rs     # Generated actor types
│   │   ├── sensors.rs    # Generated sensor types
│   │   └── ...
│   └── bindings/         # Bindgen generated bindings
├── cpp/                  # C++ bridge code
└── include/              # C++ headers

carla/                     # Main CARLA library
├── Cargo.toml
├── src/
│   ├── lib.rs            # High-level API wrapping carla-sys types
│   ├── actor.rs          # Wraps carla_sys::generated::actors
│   ├── sensor.rs         # Wraps carla_sys::generated::sensors
│   └── ... (high-level wrappers)
```

**carla-codegen Dependencies**:
```toml
[package]
name = "carla-codegen"
version = "0.1.0"
edition = "2021"

[dependencies]
serde = { version = "1.0", features = ["derive"] }
serde_yaml = "0.9"
toml = "0.8"
tera = "1.19"              # Template engine
syn = "2.0"                # Rust code parsing
quote = "1.0"              # Rust code generation
proc-macro2 = "1.0"        # Token stream manipulation
anyhow = "1.0"             # Error handling
tracing = "0.1"            # Logging
indexmap = "2.0"           # Ordered maps

[dev-dependencies]
insta = "1.0"              # Snapshot testing
pretty_assertions = "1.0"   # Better test assertions
```

**carla-sys Build Dependencies**:
```toml
# In carla-sys/Cargo.toml
[build-dependencies]
carla-codegen = { path = "../carla-codegen" }
anyhow = "1.0"
cmake = "0.1"
cxx-build = "1.0"

[dependencies]
cxx = "1.0"
# ... other dependencies
```

**carla Dependencies**:
```toml
# In carla/Cargo.toml
[dependencies]
carla-sys = { path = "../carla-sys" }
# Import generated types from carla-sys
# ... other dependencies
```

### FFI Integration Pattern

The generated code in `carla-sys` will provide low-level FFI types:

```rust
// Generated code in carla-sys/src/generated/actors.rs
use crate::ffi;

/// CARLA defines actors as anything that plays a role in the simulation
#[derive(Debug, Clone)]
pub struct Actor {
    pub(crate) inner: ffi::SharedPtr<ffi::Actor>,
}

impl Actor {
    /// Identifier for this actor. Unique during a given episode.
    pub fn id(&self) -> u32 {
        unsafe { ffi::Actor_GetId(&*self.inner) }
    }
    
    /// The identifier of the blueprint this actor was based on
    pub fn type_id(&self) -> String {
        unsafe { ffi::Actor_GetTypeId(&*self.inner) }
    }
    
    /// Returns whether this object was destroyed
    pub fn is_alive(&self) -> bool {
        unsafe { ffi::Actor_IsAlive(&*self.inner) }
    }
    
    /// Tells the simulator to destroy this actor
    pub fn destroy(self) -> Result<bool, ffi::Error> {
        let result = unsafe { ffi::Actor_Destroy(&*self.inner) };
        Ok(result)
    }
}
```

Then `carla` crate wraps these types with high-level API:

```rust
// In carla/src/actor.rs
use carla_sys::generated::actors::Actor as SysActor;

pub struct Actor {
    inner: SysActor,
    // Additional high-level state
}

impl Actor {
    pub fn from_sys(sys: SysActor) -> Self {
        Self { inner: sys }
    }
    
    pub fn id(&self) -> u32 {
        self.inner.id()
    }
    
    // High-level methods with better error handling
    pub fn destroy(self) -> crate::Result<()> {
        self.inner.destroy()
            .map_err(|e| crate::Error::from(e))?;
        Ok(())
    }
}
```

### Code Generation Strategy

**Multi-pass Generation**:
1. **Parse Pass**: Load all YAML files and build complete type graph
2. **Analysis Pass**: Resolve dependencies, inheritance, and type relationships
3. **Planning Pass**: Determine generation order and module structure
4. **Generation Pass**: Generate code using templates
5. **Format Pass**: Run rustfmt on generated code

**Stub Mode Generation**:
When generating for docs-only mode:
- Replace all FFI calls with `unreachable!("docs-only stub")`
- Maintain exact same API surface
- Allow code to compile without CARLA dependencies
- Enable docs.rs to build documentation

Example stub generation:
```rust
// Generated in src/generated/ffi/actors.rs (normal mode)
impl Actor {
    pub fn id(&self) -> u32 {
        unsafe { ffi::Actor_GetId(&*self.inner) }
    }
}

// Generated in src/generated/stubs/actors.rs (docs-only mode)
impl Actor {
    pub fn id(&self) -> u32 {
        unreachable!("docs-only stub: Actor::id")
    }
}
```

**Template-based Generation**:
Using Tera templates for flexibility:

```tera
// struct.rs.tera
{% if doc %}/// {{ doc | wordwrap(width=80) | indent(prefix="/// ") }}{% endif %}
#[derive(Debug, Clone)]
pub struct {{ name }} {
    {% for field in fields %}
    {% if field.doc %}/// {{ field.doc }}{% endif %}
    pub {{ field.name }}: {{ field.rust_type }},
    {% endfor %}
}

impl {{ name }} {
    {% for method in methods %}
    {{ include("method.rs.tera", method=method) }}
    {% endfor %}
}
```

### Integration with Existing Codebase

**Build-time Code Generation in carla-sys**:

The `carla-sys` crate uses `carla-codegen` in its `build.rs` alongside C++ library build:

```rust
// carla-sys/build.rs
use carla_codegen::{Config, Generator};
use std::env;
use std::path::PathBuf;

fn main() -> anyhow::Result<()> {
    // Handle docs-only feature first
    if cfg!(feature = "docs-only") {
        println!("Building in docs-only mode - skipping C++ compilation");
        return Ok(());
    }

    // Build C++ library first
    build_cpp_library()?;
    
    // Then generate Rust types from YAML
    generate_rust_types()?;
    
    Ok(())
}

fn generate_rust_types() -> anyhow::Result<()> {
    // Use CARLA_ROOT from carla-src
    use carla_src::CarlaSource;
    let carla_source = CarlaSource::new()?;
    let source_dir = carla_source.source_dir();
    
    let out_dir = PathBuf::from(env::var("OUT_DIR")?);
    let yaml_dir = source_dir.join("PythonAPI/docs");
    let config_path = PathBuf::from("carla-codegen.toml");
    
    // Skip if YAML docs don't exist
    if !yaml_dir.exists() {
        println!("cargo:warning=CARLA Python API docs not found, skipping code generation");
        return Ok(());
    }
    
    // Load configuration
    let mut config = Config::from_file(&config_path)
        .unwrap_or_else(|_| Config::default());
    
    // Override output to OUT_DIR/generated
    config.output_dir = out_dir.join("generated");
    
    // Create generator
    let mut generator = Generator::new(config);
    
    // Generate code to OUT_DIR
    generator
        .add_yaml_dir(&yaml_dir)?
        .generate()?;
    
    // Rebuild if inputs change
    println!("cargo:rerun-if-changed=carla-codegen.toml");
    println!("cargo:rerun-if-changed={}", yaml_dir.display());
    
    Ok(())
}
```

**Exporting Generated Code from carla-sys**:

In `carla-sys/src/lib.rs`:
```rust
// FFI bindings
pub mod ffi;

// Generated types from Python API
#[path = "generated/mod.rs"]
pub mod generated {
    // Re-export generated modules
    include!(concat!(env!("OUT_DIR"), "/generated/mod.rs"));
}

// Make generated types easily accessible
pub use generated::*;
```

**Using Generated Types in carla**:

In `carla/src/actor.rs`:
```rust
// Import low-level generated types from carla-sys
use carla_sys::generated::actors::Actor as SysActor;
use crate::error::CarlaResult;

// High-level wrapper with additional functionality
pub struct Actor {
    inner: SysActor,
    // Additional state for manual features
    cached_transform: Option<Transform>,
}

impl Actor {
    // Create from sys type
    pub(crate) fn from_sys(inner: SysActor) -> Self {
        Self {
            inner,
            cached_transform: None,
        }
    }
    
    // Delegate to sys implementation
    pub fn id(&self) -> u32 {
        self.inner.id()
    }
    
    // Enhanced method with caching
    pub fn transform(&mut self) -> CarlaResult<Transform> {
        if let Some(cached) = &self.cached_transform {
            return Ok(cached.clone());
        }
        
        let transform = self.inner.transform()
            .map_err(|e| crate::Error::from_sys(e))?;
        self.cached_transform = Some(transform.clone());
        Ok(transform)
    }
    
    // Additional high-level methods
    pub fn safe_destroy(self) -> CarlaResult<()> {
        // Retry logic, cleanup, etc.
        retry::retry(|| {
            self.inner.destroy()
                .map_err(|e| crate::Error::from_sys(e))
        })
    }
}
```

**Extension Traits Pattern**:

```rust
// carla/src/extensions/actor_ext.rs
use crate::generated::actors::Actor as GeneratedActor;

/// Extension trait for generated Actor types
pub trait ActorExt {
    fn is_in_range(&self, other: &Self, distance: f32) -> bool;
    fn distance_to(&self, other: &Self) -> f32;
}

impl ActorExt for GeneratedActor {
    fn is_in_range(&self, other: &Self, distance: f32) -> bool {
        self.distance_to(other) <= distance
    }
    
    fn distance_to(&self, other: &Self) -> f32 {
        let t1 = self.transform();
        let t2 = other.transform();
        t1.location.distance(&t2.location)
    }
}
```

**Selective Type Enhancement**:

```rust
// carla/src/vehicle.rs
use crate::generated::actors::Vehicle as GeneratedVehicle;

pub struct Vehicle {
    inner: GeneratedVehicle,
    // Manual additions
    telemetry_cache: RefCell<Option<(Instant, TelemetryData)>>,
}

impl Vehicle {
    const CACHE_DURATION: Duration = Duration::from_millis(100);
    
    // Enhanced telemetry with caching
    pub fn telemetry_data(&self) -> CarlaResult<TelemetryData> {
        let mut cache = self.telemetry_cache.borrow_mut();
        
        if let Some((timestamp, data)) = &*cache {
            if timestamp.elapsed() < Self::CACHE_DURATION {
                return Ok(data.clone());
            }
        }
        
        let data = self.inner.telemetry_data()?;
        *cache = Some((Instant::now(), data.clone()));
        Ok(data)
    }
}
```

**Directory Structure After Build**:
```
target/debug/build/carla-sys-{hash}/out/
├── generated/              # Generated Rust types
│   ├── mod.rs             # Module declarations
│   ├── actors/
│   │   ├── mod.rs
│   │   ├── actor.rs       # Generated Actor type
│   │   ├── vehicle.rs     # Generated Vehicle type
│   │   └── ...
│   ├── sensors/
│   ├── controls/
│   └── data/
├── libcarla-client.a      # Built C++ library
└── ... (other build artifacts)
```

## Implementation Plan

### Phase 1: Core Infrastructure (Week 1)
1. Create project structure for the conversion tool
2. Implement YAML parser for the schema
3. Build intermediate representation data structures
4. Create basic type mapping system

### Phase 2: Type Mapping (Week 2)
1. Implement comprehensive Python-to-Rust type mappings
2. Handle special CARLA types (geometry, actors, etc.)
3. Create enum detection and generation
4. Handle collection types and generics

### Phase 3: Code Generation (Week 3-4)
1. Implement Rust struct generation
2. Generate impl blocks with methods
3. Add documentation generation
4. Create module structure matching Python API

### Phase 4: FFI Integration (Week 5-6)
1. Generate FFI function declarations
2. Create safe wrapper implementations
3. Handle type conversions at FFI boundary
4. Implement error handling

### Phase 5: Validation and Testing (Week 7)
1. Compare generated code with existing implementation
2. Create test cases for type conversions
3. Validate documentation completeness
4. Manual review and adjustments

## Tool Usage

### Build-time Generation

The primary usage is through `build.rs` in the `carla` crate:

```rust
// carla/build.rs
use carla_codegen::{Config, Generator};

fn main() {
    // Simple usage with defaults
    carla_codegen::generate_default().unwrap();
    
    // Or with custom configuration
    let config = Config::builder()
        .yaml_dir("../carla-simulator/PythonAPI/docs")
        .output_dir(env::var("OUT_DIR").unwrap())
        .config_file("carla-codegen.toml")
        .build();
        
    Generator::new(config).generate().unwrap();
}
```

### Library API

The `carla-codegen` crate provides a flexible API:

```rust
use carla_codegen::{Config, Generator, TypeMapping};

// Create configuration
let mut config = Config::default();
config.add_type_mapping("int", "i32");
config.set_builder_threshold(3);
config.exclude_class("DeprecatedClass");

// Create generator
let mut generator = Generator::new(config);

// Add inputs
generator
    .add_yaml_file("actor.yml")?
    .add_yaml_dir("../docs")?
    .set_output_dir("./generated")?;

// Generate code
generator.generate()?;

// Or generate to string for testing
let generated_code = generator.generate_to_string()?;
```

### Standalone CLI Tool (Optional)

For development and testing, a CLI tool can be built:

```rust
// carla-codegen/examples/cli.rs
use carla_codegen::{Config, Generator};
use clap::Parser;
use std::path::PathBuf;

#[derive(Parser)]
struct Args {
    /// YAML input directory
    #[arg(short, long)]
    input: PathBuf,
    
    /// Output directory
    #[arg(short, long)]
    output: PathBuf,
    
    /// Configuration file
    #[arg(short, long)]
    config: Option<PathBuf>,
    
    /// Dry run (print to stdout)
    #[arg(long)]
    dry_run: bool,
}

fn main() -> anyhow::Result<()> {
    let args = Args::parse();
    
    let config = if let Some(config_path) = args.config {
        Config::from_file(config_path)?
    } else {
        Config::default()
    };
    
    let mut generator = Generator::new(config);
    generator.add_yaml_dir(&args.input)?;
    
    if args.dry_run {
        println!("{}", generator.generate_to_string()?);
    } else {
        generator.set_output_dir(&args.output)?.generate()?;
    }
    
    Ok(())
}
```

### Testing Generated Code

```rust
#[cfg(test)]
mod tests {
    use carla_codegen::{Config, Generator};
    use tempfile::TempDir;
    
    #[test]
    fn test_actor_generation() {
        let temp_dir = TempDir::new().unwrap();
        
        let config = Config::default();
        let mut generator = Generator::new(config);
        
        generator
            .add_yaml_file("tests/fixtures/actor.yml")
            .unwrap()
            .set_output_dir(temp_dir.path())
            .unwrap()
            .generate()
            .unwrap();
        
        // Verify generated files exist
        assert!(temp_dir.path().join("actors/actor.rs").exists());
    }
}
```

## Benefits

1. **Consistency**: Ensures Rust API matches Python API exactly
2. **Automation**: Reduces manual effort in keeping APIs synchronized
3. **Documentation**: Automatically generates comprehensive Rust docs
4. **Type Safety**: Leverages Rust's type system while maintaining Python compatibility
5. **Maintenance**: Easy to update when Python API changes

## Challenges and Solutions

### Challenge 1: Dynamic Python Types
**Solution**: Use trait objects or enums for dynamic behavior

### Challenge 2: Python-specific Features
**Solution**: Provide Rust-idiomatic alternatives (e.g., builders instead of kwargs)

### Challenge 3: Callback and Event Handling
**Solution**: Use closure traits and channel-based communication

### Challenge 4: Lifetime Management
**Solution**: Use reference counting (Arc) for shared ownership

## Example Output

Given this Python API definition:
```yaml
- class_name: Vehicle
  doc: >
    Vehicles are a special type of actor that provide additional control.
  instance_variables:
  - var_name: bounding_box
    type: carla.BoundingBox
  methods:
  - def_name: apply_control
    params:
    - param_name: control
      type: carla.VehicleControl
    doc: >
      Applies a control object to the vehicle.
```

The tool would generate:
```rust
/// Vehicles are a special type of actor that provide additional control.
#[derive(Debug, Clone)]
pub struct Vehicle {
    inner: VehicleHandle,
}

impl Vehicle {
    /// Gets the bounding box of the vehicle.
    pub fn bounding_box(&self) -> BoundingBox {
        // FFI call to get bounding box
        unsafe { carla_sys::Vehicle_GetBoundingBox(self.inner.as_ptr()) }
    }

    /// Applies a control object to the vehicle.
    pub fn apply_control(&self, control: &VehicleControl) -> CarlaResult<()> {
        // FFI call to apply control
        unsafe {
            carla_sys::Vehicle_ApplyControl(self.inner.as_ptr(), control.as_ffi())
        }
        Ok(())
    }
}
```

### Error Handling Strategy

**Generation Errors**:
```rust
#[derive(Debug, thiserror::Error)]
pub enum CodegenError {
    #[error("Failed to parse YAML: {0}")]
    YamlParse(#[from] serde_yaml::Error),
    
    #[error("Type '{0}' not found in type mappings")]
    UnknownType(String),
    
    #[error("Circular dependency detected: {0}")]
    CircularDependency(String),
    
    #[error("Invalid configuration: {0}")]
    Config(String),
    
    #[error("Template error: {0}")]
    Template(#[from] tera::Error),
}
```

**Graceful Degradation**:
- Generate partial code when possible
- Mark unimplemented methods with `todo!()` and detailed comments
- Generate compile-time warnings for missing features

### Migration Strategy

**Incremental Adoption**:
1. **Phase 1**: Set up `carla-codegen` and integrate with `build.rs`
2. **Phase 2**: Generate code alongside existing manual implementation
3. **Phase 3**: Gradually wrap generated types with manual enhancements
4. **Phase 4**: Remove redundant manual code, keep only extensions

**Parallel Implementation**:
```rust
// carla/src/lib.rs during migration
#[cfg(feature = "use-generated")]
mod generated;

#[cfg(feature = "use-generated")]
pub mod actor {
    use crate::generated::actors as gen;
    pub use super::actor_manual::ActorExt; // Keep manual extensions
    
    pub struct Actor {
        inner: gen::Actor,
    }
}

#[cfg(not(feature = "use-generated"))]
pub mod actor {
    pub use super::actor_manual::*; // Use existing manual implementation
}

// Manual extensions always available
mod actor_manual {
    pub trait ActorExt {
        fn safe_destroy(self) -> CarlaResult<()>;
    }
}
```

**Build Configuration**:
```toml
# carla/Cargo.toml
[features]
default = []
use-generated = []     # Opt-in to generated implementation
always-generate = []   # Generate even if not using

[build-dependencies]
carla-codegen = { path = "../carla-codegen" }
```

**Conditional Generation in build.rs**:
```rust
// carla/build.rs
fn main() {
    // Always generate if feature enabled or in CI
    if cfg!(feature = "use-generated") || 
       cfg!(feature = "always-generate") ||
       env::var("CI").is_ok() {
        
        carla_codegen::generate_default()
            .expect("Failed to generate API");
    }
}
```

### Advanced Features

**Trait Generation**:
Generate common traits for actor types:
```rust
pub trait ActorExt {
    fn transform(&self) -> Transform;
    fn set_transform(&mut self, transform: Transform);
    fn destroy(self) -> CarlaResult<()>;
}
```

**Async API Generation**:
Optional async wrappers for blocking calls:
```rust
impl Actor {
    pub async fn transform_async(&self) -> CarlaResult<Transform> {
        let actor = self.clone();
        tokio::task::spawn_blocking(move || actor.transform()).await?
    }
}
```

**Macro Generation**:
Generate helper macros for common patterns:
```rust
/// Generated macro for actor casting
macro_rules! cast_actor {
    ($actor:expr, Vehicle) => {
        $actor.try_into::<Vehicle>()
    };
    ($actor:expr, Walker) => {
        $actor.try_into::<Walker>()
    };
}
```

## Key Design Advantages

### Build-time Generation Benefits

1. **Zero Manual Steps**: Code generation happens automatically during `cargo build`
2. **Always Fresh**: Generated code is never out of sync with YAML specs
3. **No Version Control**: Generated code doesn't need to be committed
4. **Smaller Repository**: Only source files are tracked
5. **CI/CD Friendly**: Works seamlessly in automated builds

### Integration Advantages

1. **Transparent Process**: Developers don't need to know about code generation
2. **Standard Rust Pattern**: Using `build.rs` is idiomatic
3. **Easy Testing**: Can generate different code for tests
4. **Conditional Generation**: Can disable for faster development builds
5. **IDE Friendly**: Generated code is available for completion

### Development Workflow

```bash
# Development cycle
1. Edit carla-codegen.toml configuration
2. Run `cargo build` in carla crate
3. Generated code appears in target/*/build/carla-*/out/
4. Use generated types with manual extensions
5. Iterate as needed

# Testing changes
cargo test --features use-generated    # Test with generated code
cargo test                              # Test with manual code
cargo test --all-features               # Test both
```

## Progress Tracking

### Overall Progress

| Component         | Status      | Progress | Notes                                |
|-------------------|-------------|----------|--------------------------------------|
| Crate Setup       | Completed   | 100%     | carla-codegen crate created          |
| YAML Parser       | Completed   | 100%     | Parse Python API YAML files          |
| Type Mapper       | Completed   | 100%     | Python to Rust type conversion       |
| Code Generator    | Completed   | 100%     | Generate Rust code from IR           |
| Configuration     | Completed   | 100%     | Full config support with filtering   |
| CLI Tool          | Completed   | 100%     | Full CLI with generate/validate/list |
| Testing Framework | Partial     | 80%      | Comprehensive test coverage          |
| Documentation     | Partial     | 60%      | README and inline docs               |
| Carla Integration | Not Started | 0%       | Final integration step               |

### Detailed Work Items

#### Phase 1: Core `carla-codegen` Crate Setup

| Task                                                  | Status       | Testing         | Linting        |
|-------------------------------------------------------|--------------|-----------------|----------------|
| Create `carla-codegen` crate structure                | ✅ Completed | -               | `make lint` ✅ |
| Set up Cargo.toml with dependencies                   | ✅ Completed | `make build` ✅ | `make lint` ✅ |
| Create module structure (parser, analyzer, generator) | ✅ Completed | `make test` ✅  | `make lint` ✅ |
| Add to workspace in root Cargo.toml                   | ✅ Completed | `make build` ✅ | -              |
| Set up error handling types                           | ✅ Completed | `make test` ✅  | `make lint` ✅ |
| Create logging infrastructure                         | ✅ Completed | `make test` ✅  | -              |
| Add Makefile support for carla-codegen                | ✅ Completed | -               | -              |

#### Phase 2: YAML Parser Implementation

| Task                          | Status       | Testing                     | Linting        |
|-------------------------------|--------------|-----------------------------|----------------|
| Define YAML schema structures | ✅ Completed | Unit tests with fixtures ✅ | `make lint` ✅ |
| Implement YAML file reader    | ✅ Completed | Test with sample YAMLs ✅   | `make lint` ✅ |
| Parse module definitions      | ✅ Completed | Test all module types ✅    | -              |
| Parse class definitions       | ✅ Completed | Test inheritance ✅         | -              |
| Parse instance variables      | ✅ Completed | Test type parsing ✅        | -              |
| Parse method signatures       | ✅ Completed | Test param parsing ✅       | -              |
| Extract documentation         | ✅ Completed | Test doc formatting ✅      | -              |
| Validate YAML structure       | ✅ Completed | Test error cases ✅         | `make lint` ✅ |

#### Phase 3: Type System and Analysis

| Task                                 | Status       | Testing                 | Linting        |
|--------------------------------------|--------------|-------------------------|----------------|
| Create type mapping registry         | ✅ Completed | Unit tests ✅           | `make lint` ✅ |
| Implement basic type conversions     | ✅ Completed | Test each mapping ✅    | `make lint` ✅ |
| Handle collection types (list, dict) | ✅ Completed | Test nested types ✅    | -              |
| Handle CARLA-specific types          | ✅ Completed | Test custom types ✅    | -              |
| Implement inheritance resolver       | ✅ Completed | Test class hierarchy ✅ | -              |
| Dependency graph builder             | ✅ Completed | Test circular deps ✅   | -              |
| Type validation logic                | ✅ Completed | Test invalid types ✅   | `make lint` ✅ |

#### Phase 4: Code Generation

| Task                                | Status       | Testing                | Linting                |
|-------------------------------------|--------------|------------------------|------------------------|
| Set up Tera template engine         | ✅ Completed | Template tests ✅      | -                      |
| Create struct generation template   | ✅ Completed | Snapshot tests ✅      | `rustfmt` on output ✅ |
| Create impl block template          | ✅ Completed | Snapshot tests ✅      | `rustfmt` on output ✅ |
| Create method generation logic      | ✅ Completed | Test all signatures ✅ | -                      |
| Generate module structure           | ✅ Completed | Test file layout ✅    | -                      |
| Implement builder pattern generator | ✅ Completed | Test builder API 🟨    | -                      |
| Add documentation generation        | ✅ Completed | Test doc comments ✅   | -                      |
| Format generated code with rustfmt  | ✅ Completed | Verify formatting ✅   | -                      |

#### Phase 5: Configuration System

| Task                       | Status       | Testing                 | Linting        |
|----------------------------|--------------|-------------------------|----------------|
| Define TOML schema         | ✅ Completed | Schema validation ✅    | -              |
| Implement config parser    | ✅ Completed | Config tests ✅         | `make lint` ✅ |
| Type mapping configuration | ✅ Completed | Mapping tests ✅        | -              |
| Filter configuration       | ✅ Completed | Filter tests ✅         | -              |
| Naming convention rules    | ✅ Completed | Name transform tests ✅ | -              |
| Default configuration      | ✅ Completed | Default tests ✅        | `make lint` ✅ |

#### Phase 6: CLI Tool Implementation

| Task                          | Status       | Testing             | Linting        |
|-------------------------------|--------------|---------------------|----------------|
| Create CLI binary structure   | ✅ Completed | -                   | `make lint` ✅ |
| Implement command-line parser | ✅ Completed | CLI tests ✅        | -              |
| Add generate command          | ✅ Completed | E2E tests ✅        | -              |
| Add validate command          | ✅ Completed | Validation tests ✅ | -              |
| Add list command              | ✅ Completed | List tests ✅       | -              |
| Create helpful error messages | ✅ Completed | Error case tests ✅ | -              |
| Add progress reporting        | ✅ Completed | -                   | -              |

#### Phase 7: Testing and Documentation

| Task                      | Status       | Testing               | Linting                  |
|---------------------------|--------------|-----------------------|--------------------------|
| Unit test framework       | ✅ Completed | `make test` ✅        | -                        |
| Integration test suite    | ✅ Completed | `make test` ✅        | -                        |
| Snapshot tests with insta | 🟨 Partial   | `cargo insta test`    | -                        |
| Example YAML fixtures     | ✅ Completed | Fixture validation ✅ | -                        |
| Generated code validation | ✅ Completed | Compile tests ✅      | `make lint` on output ✅ |
| API documentation         | ✅ Completed | `cargo doc` ✅        | -                        |
| Usage examples            | ✅ Completed | Example tests ✅      | -                        |
| README for carla-codegen  | ✅ Completed | -                     | -                        |

#### Phase 8: Integration with `carla` Crate

| Task                                    | Status       | Testing              | Linting        |
|-----------------------------------------|--------------|----------------------|----------------|
| Create example build.rs                 | ✅ Completed | Build test ✅        | `make lint` ✅ |
| Add carla-codegen to build-dependencies | ✅ Completed | `make build` ✅      | -              |
| Implement build.rs integration          | ✅ Completed | Build tests ✅       | -              |
| Create generated.rs include             | ✅ Completed | Compilation test ✅  | -              |
| Test with carla crate                   | ✅ Completed | Integration build ✅ | `make lint` ✅ |
| Add feature flags                       | ✅ Completed | Feature tests ✅     | -              |
| Write migration guide                   | ✅ Completed | -                    | -              |
| Update carla documentation              | ✅ Completed | -                    | -              |

#### Phase 9: Migration to carla-sys (NEW)

| Task                                      | Status       | Testing              | Linting        |
|-------------------------------------------|--------------|----------------------|----------------|
| Update carla-sys build.rs for codegen     | ✅ Completed | Build tests ✅       | `make lint` ✅ |
| Move carla-codegen.toml to carla-sys      | ✅ Completed | Config validation ✅ | -              |
| Export generated types from carla-sys     | ✅ Completed | Export tests ✅      | `make lint` ✅ |
| Update carla crate to use sys types       | ✅ Completed | Integration tests ✅ | `make lint` ✅ |
| Remove codegen from carla build.rs        | ✅ Completed | Build tests ✅       | -              |
| Update documentation for new architecture | ✅ Completed | Doc tests ✅         | -              |
| Test full workspace build                 | ✅ Completed | `make build` ✅      | `make lint` ✅ |
| Create migration guide for users          | ✅ Completed | -                    | -              |

#### Phase 10: Syn Refactoring (NEW)

| Task                                 | Status         | Testing              | Linting     |
|--------------------------------------|----------------|----------------------|-------------|
| Remove Tera dependencies             | ✅ Completed   | Compilation test ✅   | -           |
| Create AST builder module            | ✅ Completed   | Unit tests ✅        | `make lint` ✅ |
| Implement struct generation          | ✅ Completed   | AST validation tests ✅ | `make lint` ✅ |
| Implement impl generation            | ✅ Completed   | Code output tests ✅  | `make lint` ✅ |
| Implement trait derivation           | ✅ Completed   | Derive tests ✅       | `make lint` ✅ |
| Add doc comment generation           | ✅ Completed   | Documentation tests ✅ | -           |
| Replace all Tera templates           | ✅ Completed   | Output comparison ✅  | `make lint` ✅ |
| Remove template directory            | ✅ Completed   | Build tests ✅        | -           |
| Update CLI to remove renderer option | ✅ Completed   | CLI tests ✅          | `make lint` ✅ |
| Performance benchmarking             | ⬜ Optional    | Bench comparisons    | -           |
| Update all documentation             | ✅ Completed   | Doc tests ✅          | -           |

## Phase 9: carla-sys Migration Plan

### Migration Rationale

Moving code generation from `carla` to `carla-sys` provides several architectural benefits:

#### Current Architecture Issues:
- **Duplicated CARLA_ROOT handling**: Both carla and carla-sys need to find CARLA source
- **Build order complexity**: carla depends on carla-sys but generates its own types
- **Unclear separation**: Low-level FFI types mixed with high-level API generation
- **Multiple configuration points**: Each crate manages its own CARLA-related config

#### Benefits of carla-sys Migration:
- **Single source of truth**: CARLA_ROOT and source management centralized in carla-sys
- **Clear layering**: carla-sys provides all low-level types, carla provides high-level API
- **No build-time generation**: Pre-generated code ships with source, no CARLA_ROOT needed for users
- **Faster builds**: No code generation during normal builds, just use pre-generated files
- **docs.rs compatibility**: Stub implementations allow documentation to build without CARLA
- **Reproducible builds**: All users get the same pre-generated code
- **Cleaner dependencies**: carla imports types from carla-sys, no build complexity

### Migration Architecture

#### New Build Flow:
```
carla-sys/
├── build.rs                    # Updated to run code generation
├── carla-codegen.toml         # Moved from carla/ crate
├── src/
│   ├── lib.rs                 # Exports generated types
│   ├── ffi.rs                 # CXX FFI definitions  
│   └── generated/             # Pre-generated code directory
│       ├── ffi/               # Real FFI-backed implementations
│       │   ├── mod.rs         # Module exports
│       │   ├── actor.rs       # Generated actor types
│       │   ├── control.rs     # Generated control types
│       │   └── ...            # Other generated modules
│       └── stubs/             # Stub implementations for docs-only
│           ├── mod.rs         # Module exports with unreachable!()
│           ├── actor.rs       # Stub actor types
│           └── ...            # Other stub modules

carla/
├── build.rs                    # Simplified - no code generation
├── src/
│   └── lib.rs                 # Imports types from carla_sys::generated
```

#### Build Strategy:
1. **Development with save-bindgen**: Generate code to src/generated/
2. **Normal builds**: Use pre-generated code, no CARLA_ROOT needed
3. **Documentation builds**: Use stubs when docs-only is enabled
4. **Pre-generated approach**: Ship generated code with source
5. **Fast builds**: No runtime code generation for users

### Implementation Plan

#### Phase 9.1: Update carla-sys Build System (Day 1-2)
- Modify carla-sys/build.rs to include code generation
- Add carla-codegen as build dependency
- Create src/generated/ directory structure
- Implement conditional generation based on save-bindgen feature

#### Phase 9.2: Move Configuration (Day 3)
- Move carla-codegen.toml from carla/ to carla-sys/
- Update configuration paths and settings
- Test configuration loading in new location
- Update documentation references

#### Phase 9.3: Generate and Export Types (Day 4-5)
- Generate FFI and stub implementations
- Create module exports in carla-sys/src/lib.rs
- Ensure proper visibility and re-exports
- Test type availability from carla crate

#### Phase 9.4: Update carla Crate (Day 6-7)
- Remove code generation from carla/build.rs
- Update imports to use carla_sys::generated types
- Fix any type compatibility issues
- Update feature flags and dependencies

#### Phase 9.5: Testing and Documentation (Day 8)
- Run full workspace build and test suite
- Update all documentation for new architecture
- Create migration guide for users
- Verify docs.rs compatibility

### Testing Strategy

#### Build Testing:
```bash
# Test with save-bindgen feature
cd carla-sys
cargo build --features save-bindgen
# Verify generated files in src/generated/

# Test normal build (uses pre-generated)
cargo build
# Should not require CARLA_ROOT

# Test docs-only build
cargo build --features docs-only
# Should use stub implementations
```

#### Integration Testing:
- **Workspace build**: Ensure full workspace builds correctly
- **Type resolution**: Verify carla crate can use sys types
- **Feature combinations**: Test various feature flag combinations
- **Documentation build**: Verify docs.rs compatibility
- **Performance**: Measure build time improvements

### Migration Timeline

| Day | Focus           | Deliverables                            |
|-----|-----------------|-----------------------------------------|
| 1-2 | Build System    | Updated carla-sys build.rs with codegen |
| 3   | Configuration   | Moved configuration to carla-sys        |
| 4-5 | Type Generation | Generated types exported from carla-sys |
| 6-7 | carla Updates   | Updated imports and removed old codegen |
| 8   | Testing & Docs  | Full validation and documentation       |

### Success Criteria

✅ **Centralized CARLA handling**: All CARLA source dependencies in carla-sys
✅ **No user-facing codegen**: Pre-generated code ships with source
✅ **Faster builds**: No CARLA_ROOT needed for normal builds
✅ **docs.rs compatibility**: Builds successfully with docs-only feature
✅ **Clean architecture**: Clear separation between FFI and high-level API

### Migration Dependencies

#### carla-sys Updates:
```toml
[build-dependencies]
carla-codegen = { path = "../carla-codegen" }

[features]
default = []
docs-only = []  # Use stub implementations
save-bindgen = []  # Regenerate pre-generated code
```

#### carla Updates:
```toml
# Remove from build-dependencies:
# carla-codegen = { path = "../carla-codegen" }

# Update imports to use:
# use carla_sys::generated::*;
```

### Detailed Implementation Roadmap

#### Day 1-2: Update carla-sys Build System

**Tasks:**
- [ ] Add carla-codegen to carla-sys build-dependencies
- [ ] Update carla-sys/build.rs to run code generation
- [ ] Create src/generated/ffi/ and src/generated/stubs/ directories
- [ ] Implement save-bindgen feature logic
- [ ] Generate both FFI and stub implementations
- [ ] Test build with and without CARLA_ROOT

#### Day 3: Move Configuration

**Tasks:**
- [ ] Move carla-codegen.toml from carla/ to carla-sys/
- [ ] Update paths in configuration file
- [ ] Test configuration loading in build.rs
- [ ] Update any hardcoded paths in code
- [ ] Document new configuration location

#### Day 4-5: Generate and Export Types

**Tasks:**
- [ ] Update carla-sys/src/lib.rs to export generated module
- [ ] Add conditional compilation for docs-only vs normal build
- [ ] Create proper module structure for generated types
- [ ] Test type visibility from external crates
- [ ] Verify stub implementations work for docs.rs

#### Day 6-7: Update carla Crate

**Tasks:**
- [ ] Remove carla-codegen from carla/Cargo.toml build-dependencies
- [ ] Remove code generation from carla/build.rs
- [ ] Update imports to use carla_sys::generated types
- [ ] Fix any type compatibility issues
- [ ] Update documentation references

#### Day 8: Testing and Documentation

**Tasks:**
- [ ] Run `make build` to test full workspace
- [ ] Run `make test` for all unit tests
- [ ] Test docs-only build for docs.rs compatibility
- [ ] Update README files with new architecture
- [ ] Create migration guide for users
- [ ] Update API.md with completed status

### Pre-generated Code Structure

```
carla-sys/src/generated/
├── ffi/                    # Real implementations with FFI calls
│   ├── mod.rs             # Module exports
│   ├── actor.rs           # Actor types from YAML
│   ├── blueprint.rs       # Blueprint types
│   ├── client.rs          # Client types
│   ├── control.rs         # Control structures
│   ├── geom.rs           # Geometry types
│   ├── sensor.rs         # Sensor types
│   └── world.rs          # World types
└── stubs/                 # Stub implementations for docs-only
    ├── mod.rs            # Module exports with feature gate
    ├── actor.rs          # Stub actor types (unreachable!())
    └── ...               # Same structure as ffi/
```

### Feature Flag Design

```toml
# carla-sys/Cargo.toml
[features]
default = []
docs-only = []     # Use stub implementations (for docs.rs)
save-bindgen = []  # Regenerate pre-generated code

# carla-sys/src/lib.rs
#[cfg(feature = "docs-only")]
pub use generated::stubs as generated;

#[cfg(not(feature = "docs-only"))]
pub use generated::ffi as generated;
```

## Phase 10: Syn Refactoring Plan

### Overview

After completing the carla-sys migration, refactor the code generation to use syn-based AST generation instead of Tera templates. This provides better type safety, compile-time validation, and more maintainable code generation.

### Implementation Timeline

| Week | Focus               | Deliverables                                |
|------|---------------------|---------------------------------------------|
| 1    | AST Foundation      | Basic struct and impl generation with syn   |
| 2    | Full Implementation | Complete syn renderer with all features     |
| 3    | Migration & Cleanup | Remove Tera, validate output, documentation |

### Phase 10 Work Items

| Task                        | Description                         | Priority |
|-----------------------------|-------------------------------------|----------|
| Remove Tera dependencies    | Remove tera from Cargo.toml         | High     |
| Create AST builder module   | Build syn-based code generation     | High     |
| Implement struct generation | Generate structs using syn AST      | High     |
| Implement impl generation   | Generate impl blocks with methods   | High     |
| Add trait derivation        | Auto-derive Debug, Clone, etc.      | Medium   |
| Add doc comment generation  | Generate documentation from YAML    | Medium   |
| Replace all templates       | Convert all Tera templates to syn   | High     |
| Remove template directory   | Clean up old template files         | Low      |
| Update CLI                  | Remove renderer selection options   | Medium   |
| Performance benchmarking    | Compare syn vs old Tera performance | Low      |
| Update documentation        | Document syn-only approach          | High     |

### Syn Implementation Examples

#### Struct Generation with Syn:
```rust
// src/ast/struct_builder.rs
use syn::{ItemStruct, Field, Visibility, Type};
use quote::quote;

impl StructBuilder {
    pub fn build(&self) -> syn::Result<ItemStruct> {
        let name = &self.name;
        let doc = self.generate_doc_attrs()?;
        let fields = self.generate_fields()?;
        
        Ok(parse_quote! {
            #(#doc)*
            #[derive(Debug, Clone)]
            pub struct #name {
                #(#fields),*
            }
        })
    }
    
    fn generate_fields(&self) -> syn::Result<Vec<Field>> {
        self.fields.iter().map(|f| {
            let name = &f.name;
            let ty = f.rust_type.to_syn_type()?;
            let doc = f.generate_doc_attrs()?;
            
            Ok(parse_quote! {
                #(#doc)*
                pub #name: #ty
            })
        }).collect()
    }
}
```

### Benefits of Syn-based Generation

1. **Type Safety**: Compile-time validation of generated code structure
2. **Better Error Messages**: Precise error locations in generated code
3. **IDE Support**: Better integration with rust-analyzer and IDEs
4. **Maintainability**: No string templating, pure Rust code
5. **Extensibility**: Easy to add new Rust language features
6. **Correctness**: Guaranteed syntactically valid Rust output

### Phase 10 Dependencies

```toml
# carla-codegen/Cargo.toml
[dependencies]
syn = { version = "2.0", features = ["full", "extra-traits", "parsing"] }
quote = "1.0"
proc-macro2 = "1.0"
prettyplease = "0.2"  # For code formatting

# Remove these:
# tera = "1"
# serde_json = "1"  # Only needed for Tera templates
```

### Advanced Syn Features

#### Planned Advanced Capabilities:

**1. Smart Type Inference**
```rust
// Automatically infer correct Rust types from Python context
impl TypeInference {
    fn infer_rust_type(&self, python_type: &str, context: &GenerationContext) -> RustType {
        match (python_type, context.usage_pattern) {
            ("object", UsagePattern::ReturnValue) => RustType::Reference("dyn Any".into()),
            ("list", UsagePattern::Parameter) => RustType::Slice,
            ("list", UsagePattern::ReturnValue) => RustType::Vec,
            _ => self.resolve_basic_type(python_type)
        }
    }
}
```

**2. Intelligent FFI Generation**
```rust
// Generate appropriate FFI method signatures
impl FfiGenerator {
    fn generate_ffi_method(&self, method: &Method) -> TokenStream {
        let carla_prefix = self.generate_carla_prefix(&method.class);
        let params = self.convert_params_to_ffi(&method.params);
        let return_type = self.convert_return_to_ffi(&method.return_type);
        
        quote! {
            pub fn #carla_prefix(#(#params),*) -> #return_type {
                todo!("FFI binding for {}.{} not yet implemented", 
                      #method.class, #method.name)
            }
        }
    }
}
```

**3. Trait Bounds Generation**
```rust
// Automatically generate appropriate trait bounds
impl TraitBounds {
    fn generate_bounds(&self, field_types: &[RustType]) -> Vec<WherePredicate> {
        field_types.iter()
            .filter_map(|ty| self.required_bounds(ty))
            .collect()
    }
    
    fn required_bounds(&self, rust_type: &RustType) -> Option<WherePredicate> {
        match rust_type {
            RustType::Vec(_) => Some(parse_quote!(T: Clone)),
            RustType::HashMap(_, _) => Some(parse_quote!(K: Hash + Eq)),
            _ => None
        }
    }
}
```

**4. Macro-based Code Generation**
```rust
// Generate helper macros for common patterns
macro_rules! carla_actor_impl {
    ($actor_type:ident, $ffi_prefix:literal) => {
        impl $actor_type {
            pub fn id(&self) -> u32 {
                unsafe { 
                    concat_idents!($ffi_prefix, _GetId)(self.inner.as_ptr()) 
                }
            }
            
            pub fn location(&self) -> crate::geom::Location {
                unsafe {
                    let loc = concat_idents!($ffi_prefix, _GetLocation)(self.inner.as_ptr());
                    crate::geom::Location::from_ffi(loc)
                }
            }
        }
    };
}
```

### Performance Optimization Strategies

#### Memory Management:
```rust
// Efficient AST node caching
pub struct AstCache {
    type_cache: HashMap<String, Type>,
    field_cache: HashMap<(String, String), Field>,
    method_cache: HashMap<String, ImplItemMethod>,
}

impl AstCache {
    fn get_or_generate_type(&mut self, type_name: &str) -> &Type {
        self.type_cache.entry(type_name.to_string())
            .or_insert_with(|| self.generate_type(type_name))
    }
}
```

#### Parallel Generation:
```rust
// Generate multiple files in parallel
use rayon::prelude::*;

impl SynRenderer {
    fn generate_parallel(&self, classes: &[Class]) -> Result<Vec<GeneratedFile>> {
        classes.par_iter()
            .map(|class| self.generate_class_file(class))
            .collect()
    }
}
```

#### Incremental Compilation:
```rust
// Only regenerate changed files
impl IncrementalGenerator {
    fn needs_regeneration(&self, class: &Class, output_file: &Path) -> bool {
        let yaml_modified = self.yaml_modification_time(class);
        let output_modified = output_file.metadata()
            .and_then(|m| m.modified())
            .unwrap_or(SystemTime::UNIX_EPOCH);
            
        yaml_modified > output_modified
    }
}
```

### Integration with Build Systems

#### Cargo Integration:
```rust
// build.rs integration with syn renderer
fn main() {
    let config = CarlaCodegenConfig::from_env();
    
    if config.use_syn_renderer() {
        let renderer = SynRenderer::new(config.syn_options());
        renderer.generate_from_build_script()?;
    } else {
        // Fallback to Tera
        let renderer = TeraRenderer::new();
        renderer.generate_from_build_script()?;
    }
}
```

#### IDE Integration:
```toml
# .vscode/settings.json
{
    "rust-analyzer.cargo.features": ["syn-renderer"],
    "rust-analyzer.procMacro.enable": true,
    "rust-analyzer.check.command": "check",
    "rust-analyzer.check.extraArgs": ["--features", "syn-renderer"]
}
```

### Migration Testing Strategy

#### Automated Migration Testing:
```bash
#!/bin/bash
# migration_test.sh

# Test all YAML files with both renderers
for yaml_file in tests/fixtures/*.yml; do
    echo "Testing $yaml_file..."
    
    # Generate with Tera
    carla-codegen generate --renderer tera -i "$yaml_file" -o "output/tera/"
    
    # Generate with Syn  
    carla-codegen generate --renderer syn -i "$yaml_file" -o "output/syn/"
    
    # Compare outputs
    if ! diff -r "output/tera/" "output/syn/"; then
        echo "FAIL: Output differs for $yaml_file"
        exit 1
    fi
    
    # Test compilation
    if ! cargo check --manifest-path "output/syn/Cargo.toml"; then
        echo "FAIL: Syn output doesn't compile for $yaml_file"
        exit 1
    fi
done

echo "All migration tests passed!"
```

#### Continuous Integration:
```yaml
# .github/workflows/migration.yml
name: Syn Migration Tests

on: [push, pull_request]

jobs:
  migration-test:
    runs-on: ubuntu-latest
    steps:
    - uses: actions/checkout@v3
    - uses: actions-rs/toolchain@v1
      with:
        toolchain: stable
        
    - name: Run migration tests
      run: |
        cargo test --features migration-compat
        ./scripts/migration_test.sh
        
    - name: Performance benchmark
      run: |
        cargo bench --features migration-compat
        
    - name: Upload results
      uses: actions/upload-artifact@v3
      with:
        name: migration-results
        path: target/criterion/
```

### Documentation Migration

#### Updated README Example:
```markdown
## Code Generation

carla-codegen supports two rendering engines:

### Syn Renderer (Recommended)
Type-safe AST-based generation with compile-time validation:

```bash
carla-codegen generate --renderer syn -i yaml/ -o src/generated/
```

Features:
- ✅ Compile-time syntax validation
- ✅ Better error messages  
- ✅ Automatic code formatting
- ✅ Advanced trait derivation

### Tera Renderer (Legacy)
Template-based generation for compatibility:

```bash  
carla-codegen generate --renderer tera -i yaml/ -o src/generated/
```

Use for edge cases where syn renderer has issues.
```

#### Migration Guide:
```markdown
# Migrating to Syn Renderer

## For Library Users

1. **Update your config:**
   ```toml
   [generator]
   renderer = "syn"
   ```

2. **Test your build:**
   ```bash
   cargo clean
   cargo build --features codegen
   ```

3. **Report issues:**
   If you encounter problems, temporarily use:
   ```toml
   [generator]  
   renderer = "tera"
   ```

## For Contributors

1. **Template → AST conversion:**
   - Replace Tera templates with AST builders
   - Use `parse_quote!` macro for complex syntax
   - Add proper error handling

2. **Testing:**
   ```bash
   cargo test --features migration-compat
   ```
```

### Future Enhancements

#### Phase 10: Advanced Syn Features (Future)
1. **Procedural Macros**: Generate custom derive macros for CARLA types
2. **Type Safety**: Compile-time verification of FFI bindings
3. **Code Analysis**: Static analysis of generated code for optimization
4. **Plugin System**: Allow custom AST transformations
5. **Language Server**: IDE integration for code generation debugging

#### Syn 3.0 Preparation:
- Monitor syn crate development for breaking changes
- Plan migration strategy for major version updates
- Contribute back improvements to syn ecosystem

### Metrics and Monitoring

#### Success Metrics:
- **Migration Completion**: 100% of templates converted to syn
- **Performance**: <2x slowdown compared to Tera
- **Quality**: Zero compilation errors in generated code  
- **Adoption**: >80% of users switch to syn renderer
- **Maintenance**: <50% reduction in template-related issues

#### Monitoring Dashboard:
```toml
[metrics]
track_generation_time = true
track_output_size = true
track_compilation_errors = true
report_usage_stats = true
```

### Testing Strategy

#### Using the Makefile
The project uses a centralized Makefile for consistent builds, tests, and linting:

```bash
# Build the entire workspace including carla-codegen
make build

# Run all tests
make test

# Run linting (rustfmt + clippy)
make lint

# Clean build artifacts
make clean
```

#### Unit Tests
- Test each component in isolation
- Use mock data for dependencies
- Cover edge cases and error conditions
- Located in `src/` modules with `#[cfg(test)]`

#### Integration Tests
- Test full pipeline from YAML to generated code
- Use real YAML files from CARLA
- Verify generated code compiles
- Located in `tests/` directory

#### Snapshot Tests
- Use `insta` for generated code snapshots
- Review changes with `cargo insta review`
- Ensure consistent output across runs

### Development Workflow

1. **Initial Setup**:
   ```bash
   # Create the crate
   cargo new --lib carla-codegen
   cd carla-codegen
   
   # Build and test
   make build
   make test
   make lint
   ```

2. **Development Cycle**:
   ```bash
   # Write code
   # Run tests frequently
   make test
   
   # Check formatting and lints
   make lint
   
   # Build entire workspace
   make build
   ```

3. **Pre-commit**:
   ```bash
   # Always run before committing
   make lint
   make test
   ```

### Progress Update Guidelines

When updating progress:
1. Change task status: `⬜ Not Started` → `🟨 In Progress` → `✅ Completed`
2. Update the overall progress percentage in each phase
3. Update the Quick Status section at the top
4. Add notes about any blockers or changes
5. Update testing status after running tests
6. Mark linting as done after passing checks

Example status progression:
- `⬜ Not Started` - Task not begun
- `🟨 In Progress` - Actively working on task
- `✅ Completed` - Task finished and tested
- `❌ Blocked` - Task blocked by dependency
- `⏸️ On Hold` - Task paused for later

## Next Steps

### Completed (Phases 1-8):
1. ✅ Create the `carla-codegen` library crate
2. ✅ Implement basic YAML parser and type mapping
3. ✅ Create minimal `build.rs` integration in `carla`
4. ✅ Generate a single type (e.g., Actor) as proof-of-concept
5. ✅ Extend to full Python API coverage
6. ✅ Add configuration system
7. ✅ Implement builder patterns and extensions
8. ✅ Create migration guide for transitioning existing code

### Phase 9: Migration to carla-sys
9. Update carla-sys build.rs to include code generation
10. Move configuration to carla-sys directory
11. Export generated types through carla_sys::generated module
12. Update carla crate to import types from carla-sys
13. Remove code generation from carla build.rs
14. Test and validate the new architecture
15. Update all documentation to reflect the change

### Phase 10: Syn Refactoring (NEW)
16. Remove all Tera template dependencies
17. Implement syn-based AST generation throughout
18. Add type-safe Rust code generation with compile-time validation
19. Implement advanced features like automatic trait derivation
20. Remove renderer selection options (syn-only)
21. Performance benchmark and optimize syn-based generation
22. Update documentation for syn-only architecture
23. Clean up and remove all Tera-related code


