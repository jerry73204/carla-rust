# Complete Guide: Testing CARLA Code Generation

This guide shows how to test generating all types from `carla-simulator/PythonAPI/docs/` using the carla-codegen CLI.

## Quick Start

### Option 1: Automated Test Script
```bash
# Run the complete test suite
./scripts/test_carla_generation.sh
```

### Option 2: Manual Step-by-Step

## Prerequisites

1. **Rust and Cargo installed**
   ```bash
   # Install Rust if not already installed
   curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
   source ~/.cargo/env
   ```

2. **Git installed** (for cloning CARLA repository)

3. **Build carla-codegen**
   ```bash
   cargo build --release
   ```

## Step-by-Step Process

### 1. Get CARLA Documentation Files

#### Option A: Clone Full CARLA Repository
```bash
# Clone CARLA repository (this will be large ~2GB)
git clone --depth 1 --branch 0.10.0 https://github.com/carla-simulator/carla.git carla-simulator
```

#### Option B: Download Just the Docs (Recommended)
```bash
# Clone only what we need
git clone --depth 1 --branch 0.10.0 --single-branch https://github.com/carla-simulator/carla.git carla-simulator
```

#### Option C: Manual Download
1. Go to https://github.com/carla-simulator/carla/tree/0.10.0/PythonAPI/docs
2. Download individual YAML files you need

### 2. Verify CARLA Documentation Structure

```bash
# Check available documentation files
find carla-simulator/PythonAPI/docs -name "*.yml" -o -name "*.yaml"
```

Expected files include:
- `actor.yml` - Actor base classes
- `client.yml` - Client and World classes  
- `sensor.yml` - Sensor types
- `control.yml` - Control structures
- `geom.yml` - Geometry types
- And many more...

### 3. Choose or Create Configuration

#### Use Provided Configuration
```bash
# Use the comprehensive CARLA configuration
cp configs/carla_full_generation.toml my_config.toml
```

#### Create Custom Configuration
```toml
# minimal_carla_config.toml
output_dir = "generated"

[formatting]
enable_rustfmt = true

[type_mapping.mappings]
"int" = "i32"
"str" = "String"
"bool" = "bool"
"carla.Location" = "crate::geom::Location"
"carla.Actor" = "crate::client::Actor"

[method_signatures]
use_result_return_types = true
```

### 4. Run Code Generation

#### Generate All Types
```bash
./target/release/carla-codegen generate \
  --input carla-simulator/PythonAPI/docs/ \
  --config configs/carla_full_generation.toml \
  --output generated_carla \
  --verbose
```

#### Generate Specific Module Only
```bash
./target/release/carla-codegen generate \
  --input carla-simulator/PythonAPI/docs/client.yml \
  --config configs/carla_full_generation.toml \
  --output generated_client
```

#### Generate with CLI Overrides
```bash
./target/release/carla-codegen generate \
  --input carla-simulator/PythonAPI/docs/ \
  --config configs/carla_full_generation.toml \
  --output generated_carla \
  --exclude-class "DeprecatedClass" \
  --type-mapping "int=i64" \
  --builder-threshold 3
```

### 5. Examine Generated Output

```bash
# Check generated structure
tree generated_carla/

# Count generated files
find generated_carla -name "*.rs" | wc -l

# Look at a sample file
cat generated_carla/carla/client/actor.rs
```

Expected structure:
```
generated_carla/
├── mod.rs                    # Root module exports
├── carla/
│   ├── mod.rs               # Carla module exports
│   ├── client/              # Client types
│   │   ├── mod.rs
│   │   ├── actor.rs
│   │   ├── vehicle.rs
│   │   └── world.rs
│   ├── sensor/              # Sensor types
│   ├── geom/                # Geometry types
│   └── rpc/                 # RPC structures
```

### 6. Test Generated Code Quality

```bash
# Create test project
cargo new test_carla_bindings --lib
cd test_carla_bindings

# Copy generated code
cp -r ../generated_carla/* src/

# Check syntax (will have missing dependencies but syntax should be valid)
cargo check
```

## Configuration Options

### Input Sources

1. **Single file**: `--input carla-simulator/PythonAPI/docs/actor.yml`
2. **Directory**: `--input carla-simulator/PythonAPI/docs/`  
3. **Multiple files**: `--input file1.yml,file2.yml`

### Output Control

```toml
[formatting]
enable_rustfmt = true
line_width = 100
```

### Method Signature Control

```toml
[method_signatures]
# Control self parameter types
auto_detect_self_type = true
owned_self_patterns = ["destroy", "delete"]
mut_self_patterns = ["set_*", "add_*"]
ref_self_patterns = ["get_*", "is_*"]

# Return type handling
use_result_return_types = true
error_type = "crate::error::Result<()>"
```

### Type Mapping

```toml
[type_mapping.mappings]
"carla.Location" = "crate::geom::Location"
"carla.Actor" = "crate::client::Actor"
"list" = "Vec"
```

### Filtering

```toml
[filters]
exclude_classes = ["DeprecatedClass"]
exclude_methods = ["__del__", "__getattr__"]
skip_modules = ["test", "internal"]
```

## Common Use Cases

### 1. Generate Core CARLA Types Only
```bash
# Focus on essential types
./target/release/carla-codegen generate \
  --input carla-simulator/PythonAPI/docs/client.yml,carla-simulator/PythonAPI/docs/actor.yml \
  --config configs/carla_full_generation.toml \
  --output carla_core
```

### 2. Generate with Custom Error Types
```toml
[method_signatures]
use_result_return_types = true
error_type = "anyhow::Result<()>"
```

### 3. Generate Async-Ready Code
```toml
[method_signatures]
generate_async_variants = true
```

### 4. Generate with Builder Patterns
```toml
[argument_handling]
enable_builder_patterns = true
builder_threshold = 2
optional_args_style = "builder"
```

## Troubleshooting

### Common Issues

1. **"No YAML files found"**
   - Check the path to carla-simulator/PythonAPI/docs/
   - Ensure YAML files exist and are accessible

2. **"rustfmt not found"**
   - Install rustfmt: `rustup component add rustfmt`
   - Or disable in config: `enable_rustfmt = false`

3. **Generated code doesn't compile**
   - This is expected for bindings without FFI implementation
   - Focus on checking syntax and structure

4. **Missing type mappings**
   - Add missing types to `[type_mapping.mappings]`
   - Check logs for unmapped types

### Debug Mode

```bash
# Enable verbose logging
RUST_LOG=debug ./target/release/carla-codegen generate \
  --input carla-simulator/PythonAPI/docs/ \
  --config configs/carla_full_generation.toml \
  --output debug_output \
  --verbose
```

## Integration with Rust Projects

Once generated, integrate the bindings:

```rust
// In your Rust project
mod carla {
    include!(concat!(env!("OUT_DIR"), "/carla/mod.rs"));
}

use carla::client::{Client, World};
use carla::geom::Location;
```

## Performance Tips

1. **Use specific files instead of full directory** for faster generation
2. **Configure filters** to exclude unneeded classes
3. **Disable rustfmt** for faster iteration during development
4. **Use parallel generation** for large codebases (future feature)

This guide should help you successfully test and use carla-codegen with the full CARLA simulator documentation.