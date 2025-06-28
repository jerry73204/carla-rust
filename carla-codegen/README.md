# carla-codegen

Code generator for converting CARLA Python API YAML documentation to Rust type definitions.

## Overview

This crate parses CARLA's Python API documentation (in YAML format) and generates corresponding Rust types with proper FFI bindings to the C++ CARLA library. It provides both a library API and a command-line tool for code generation.

## Features

- 🔍 **YAML Parsing**: Parse CARLA's Python API documentation from YAML files
- 🔄 **Type Resolution**: Convert Python types to idiomatic Rust types
- 📊 **Dependency Analysis**: Analyze class inheritance and type dependencies
- 🏗️ **Code Generation**: Generate complete Rust modules with structs and implementations
- ⚙️ **Configuration**: Flexible configuration system with filtering and customization
- 🖥️ **CLI Tool**: Full-featured command-line interface for standalone usage
- 🧪 **Well Tested**: Comprehensive test suite with unit and integration tests

## Installation

Add to your `Cargo.toml`:

```toml
[dependencies]
carla-codegen = "0.1.0"
```

Or install the CLI tool:

```bash
cargo install carla-codegen
```

## Usage

### Command Line Interface

Generate Rust code from YAML files:

```bash
# Basic usage
carla-codegen generate -i ./yaml_files -o ./generated

# With custom configuration
carla-codegen generate -i ./yaml_files -o ./generated -c config.toml

# With inline options
carla-codegen generate -i ./yaml_files -o ./generated \
  --exclude-class DeprecatedClass \
  --type-mapping "actor_id=u32" \
  --builder-threshold 3

# Validate YAML files
carla-codegen validate -i ./yaml_files

# List available classes and modules
carla-codegen list -i ./yaml_files --classes
```

### As a Library

```rust
use carla_codegen::{config::Config, Generator};

// Basic usage with default configuration
let config = Config::default();
let mut generator = Generator::new(config);
generator.add_yaml_dir("../carla-simulator/PythonAPI/docs")?;
generator.generate()?;

// Advanced usage with custom configuration
let config = Config::builder()
    .yaml_dir("../carla-simulator/PythonAPI/docs")
    .output_dir("src/generated")
    .builder_threshold(3)
    .add_type_mapping("actor_id".to_string(), "u32".to_string())
    .exclude_class("DeprecatedClass".to_string())
    .build();

let mut generator = Generator::new(config);
generator.generate()?;
```

### Configuration

Create a `carla-codegen.toml` file for detailed configuration:

```toml
# Output settings
output_dir = "src/generated"
builder_threshold = 2
module_structure = "nested"

# Type mappings
[type_mapping]
mappings = { "int" = "i32", "float" = "f32", "actor_id" = "u32" }

# Filtering
[filters]
exclude_classes = ["DeprecatedClass"]
skip_modules = ["osm2odr"]

# Naming conventions
[naming]
method_case = "snake_case"
remove_prefix = ["get_", "set_"]

# Documentation
[documentation]
include_warnings = true
generate_doc_tests = true
```

## Generated Code Example

Given this Python API definition:
```yaml
- class_name: Vehicle
  doc: Vehicles are a special type of actor
  instance_variables:
  - var_name: bounding_box
    type: carla.BoundingBox
  methods:
  - def_name: apply_control
    params:
    - param_name: control
      type: carla.VehicleControl
```

The tool generates:
```rust
/// Vehicles are a special type of actor
#[derive(Debug, Clone)]
pub struct Vehicle {
    pub(crate) inner: carla_sys::ffi::SharedPtr<ffi::Vehicle>,
    /// Bounding box of the vehicle
    pub bounding_box: crate::geom::BoundingBox,
}

impl Vehicle {
    /// Applies a control object to the vehicle
    pub fn apply_control(&self, control: &VehicleControl) -> Result<()> {
        // TODO: Implement using carla-sys FFI interface
        todo!("Vehicle::apply_control not yet implemented - missing FFI function Vehicle_ApplyControl")
    }
}
```

## Architecture

The code generator follows a multi-phase approach:

1. **YAML Parsing**: Parse Python API documentation into structured data
2. **Type Analysis**: Resolve types and analyze dependencies and inheritance
3. **Code Generation**: Generate Rust code using configurable templates
4. **Integration**: Provide both library and CLI interfaces

## Development Status

- ✅ **Phase 1**: Core infrastructure
- ✅ **Phase 2**: YAML parser implementation  
- ✅ **Phase 3**: Type system and analysis
- ✅ **Phase 4**: Code generation
- ✅ **Phase 5**: Configuration system
- ✅ **Phase 6**: CLI tool
- ✅ **Phase 7**: Testing and documentation
- ⬜ **Phase 8**: Integration with carla crate

## Contributing

1. Ensure all tests pass: `cargo test`
2. Run linting: `cargo clippy && cargo fmt`
3. Add tests for new functionality
4. Update documentation as needed

## Examples

See the `examples/` directory for:
- Basic library usage
- Advanced configuration
- CLI usage patterns
- Configuration file examples

## License

MIT - See LICENSE file for details