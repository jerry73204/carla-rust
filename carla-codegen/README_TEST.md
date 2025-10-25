# CARLA Code Generation Testing

This document describes the test infrastructure for the CARLA code generator.

## Overview

The `scripts/test-codegen.sh` script automates the process of:
1. Generating Rust code from CARLA Python API YAML files
2. Creating a test crate to verify the generated code compiles
3. Running various validation checks
4. Producing detailed logs for debugging

## Test Crate Structure

The script creates a `carla-test-codegen` directory at the repository root with:

```
carla-test-codegen/
├── Cargo.toml          # Test crate configuration (excluded from workspace)
├── src/
│   ├── lib.rs          # Main library file
│   ├── error.rs        # Error types required by generated code
│   ├── carla/          # Generated CARLA types
│   │   ├── mod.rs
│   │   ├── actor.rs
│   │   ├── world.rs
│   │   └── ...
│   └── command/        # Generated command types
│       ├── mod.rs
│       └── ...
```

## Running Tests

From the repository root:

```bash
make test-codegen
```

Or directly:

```bash
./scripts/test-codegen.sh
```

## Test Process

1. **Setup**: Creates the test crate directory structure
2. **Code Generation**: Runs the code generator with the full configuration
3. **Syntax Validation**: Checks basic syntax of generated files
4. **Cargo Check**: Verifies the code compiles as a complete crate
5. **Cargo Build**: Attempts to build the library (may fail due to todo!() placeholders)

## Configuration

- **Input**: `carla-simulator/PythonAPI/docs` (YAML files)
- **Config**: `carla-codegen/configs/carla_full_generation.toml`
- **Output**: `carla-test-codegen/src/`

## Workspace Exclusion

The test crate is excluded from the main Cargo workspace to avoid conflicts:
- Added to `exclude` list in root `Cargo.toml`
- Has its own `[workspace]` section in `carla-test-codegen/Cargo.toml`
- Added to `.gitignore` to avoid committing generated test code

## Logs

All logs are saved to `logs/YYYY-MM-DD/codegen/`:
- `codegen_*.log` - Code generation output
- `syntax_check_*.log` - Syntax validation results
- `cargo_check_*.log` - Cargo check output
- `build_*.log` - Build output (if applicable)
- `summary_*.log` - Test summary with all results

## Known Issues

1. **WeatherParameters type path**: The generated code currently has an issue with unqualified type paths in the `eq` and `ne` methods of `WeatherParameters`. This needs to be fixed in the code generator.

2. **Unused imports**: Many generated files import `Result` but don't use it, causing warnings.

3. **todo!() placeholders**: All methods are implemented with `todo!()` macros, which will panic if called.

## Success Criteria

The test is considered successful if:
- Code generation completes without errors
- Generated files pass basic syntax validation
- The test crate passes `cargo check` (compilation)

Warnings are acceptable and expected due to the placeholder implementations.

## Debugging Failed Tests

If tests fail:

1. Check the summary log for an overview
2. Review the specific log file mentioned in the error message
3. Common issues:
   - Missing imports (e.g., HashMap)
   - Unqualified type paths
   - Syntax errors in generated code

## Future Improvements

- Fix the WeatherParameters type path issue in the generator
- Add option to suppress unused import warnings
- Generate minimal FFI stubs instead of todo!() for testing
- Add integration tests that verify specific code patterns