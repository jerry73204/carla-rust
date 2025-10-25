# CARLA Rust Client - Project Roadmap

## Current Status

**Architecture**: Code Generation-Based Approach  
**Current Phase**: Phase 1 - Completing Code Generation  
**Progress**: Working on fixing codegen issues  
**Last Updated**: 2025-07-01

## Architecture Overview

The project has transitioned to a fully code-generated architecture:
- **carla-codegen**: Generates all types and methods from CARLA's Python API YAML files
- **carla-sys**: FFI layer with generated types and C++ bridge functions
- **carla**: High-level Rust API wrapping generated types

## Three-Phase Development Plan

| Phase | Description              | Status         | Current Focus                   |
|-------|--------------------------|----------------|---------------------------------|
| **1** | Complete Code Generation | 🚧 In Progress | Fixing type resolution issues   |
| **2** | carla-sys Integration    | ⏳ Next        | Re-enable and integrate codegen |
| **3** | High-Level carla Crate   | ⏳ Future      | Create ergonomic wrappers       |

## Phase 1: Complete Code Generation (🚧 In Progress)
**Goal**: Finish carla-codegen to generate all types, methods, and FFI stubs

### Completed ✅
- YAML parsing of CARLA Python API documentation
- Python to Rust type mapping with configuration
- Syn-based AST code generation (replaced Tera templates)
- FFI stub generation with CXX bridge support
- Hierarchical configuration system
- CLI tool with multiple commands
- 73 unit tests passing
- Advanced features:
  - Enum detection for enum-like classes
  - Method signature analysis (self-type detection)
  - Special methods mapping (Python magic methods to Rust traits)
  - Smart trait derivation based on class analysis
  - Dual implementation (FFI for production, stubs for docs.rs)
- **Recent Fixes**:
  - Fixed syn parsing errors for union types (e.g., `bool / int / float / str / carla.Color / carla.ActorAttribute`)
  - Fixed double Result wrapping in method signatures
  - Union type parameters now generate multiple trait implementations
  - Complex nested types like `list([name,world, actor, relative])` now supported

### Remaining Tasks
- [x] ~~Fix type resolution for complex nested types (e.g., `list([name,world, actor, relative])`)~~ ✅ **COMPLETED**
- [ ] Handle circular dependencies between types
- [x] **Enhanced Error Reporting** (High Priority): ✅ **COMPLETED**
  - [x] Add source context tracking (YAML file → class → method → parameter)
  - [x] Show generated code snippets in AST parsing errors
  - [x] Add `--continue-on-error` CLI flag for batch processing
  - [x] Improve error messages with actionable suggestions
  - [x] Add stage tracking (type resolution, AST generation, formatting)
- [ ] **Generate Error Module** (🔥 Critical - Blocking compilation)
  - [ ] Generate `error.rs` with basic error types and Result alias
  - [ ] Include error module in root `mod.rs`
  - [ ] Make generation configurable for different target crates
- [ ] **Fix Final Type Resolution Issues** (High Priority - 7 remaining errors)
  - [ ] Fix unqualified types in Result return values
  - [ ] Fix missing HashMap imports
  - [ ] Fix parameter type paths in special methods
  - [ ] Fix nested generic type qualification
- [ ] Add missing type mappings (e.g., `callback` types)
- [ ] Add validation for generated code
- [ ] Performance optimization for large YAML files

### Code Generation Fixes (🆕 High Priority)
Based on `make test-codegen` analysis, the following fixes are needed:

#### Step 1: Core Type System Fixes ✅
- [x] **Remove Type Remapping Feature**:
  - [x] Remove complex type mapping configuration
  - [x] Use direct `carla.X` → `crate::carla::X` mapping
  - [x] Update configuration files
- [x] **Fix Type References**:
  - [x] Replace all `crate::road::*` with `crate::carla::*`
  - [x] Replace all `crate::geom::*` with `crate::carla::*`
  - [x] Replace all `crate::client::*` with `crate::carla::*`
- [x] **Fix Generic Type Parameters**:
  - [x] Parse `list(T)` → `Vec<T>` with proper type
  - [x] Parse `dict[K,V]` → `HashMap<K,V>` with types
  - [x] Handle nested generics correctly
- [x] **Fix Recursive Types**:
  - [x] Detect self-referential types (e.g., `Actor.parent`)
  - [x] Wrap recursive references in `Box<T>`
  - [x] Handle optional recursive types with `Option<Box<T>>`

#### Step 2: Code Quality Fixes ✅
- [x] **Fix Naming Conventions**:
  - [x] Convert enum variants to UpperCamelCase
  - [x] Handle special cases (e.g., `FL_Wheel` → `FlWheel`)
  - [x] Preserve numbers in names
- [x] **Fix Invalid Trait Derivations**:
  - [x] Check field types before deriving `Copy`
  - [x] Only derive `Copy` for types with all Copy fields
  - [x] Always derive `Debug` and `Clone`
- [x] **Add Missing Imports**:
  - [x] Detect HashMap usage and add `use std::collections::HashMap`
  - [x] Track all standard library types used
  - [x] Generate import statements at file top

#### Step 3: Complete the System ✅
- [x] **Add Missing Type Definitions**:
  - [x] Generate all referenced enums (e.g., `ActorState`, `TrafficLightState`)
  - [x] Create type aliases for common patterns
  - [x] Ensure no unresolved type references
- [x] **Update Test Generation**:
  - [x] Include proper dependencies in generated Cargo.toml
  - [x] Add serde/serde_json when needed
  - [x] Fix rustc syntax check command

### Type Resolution System Revision (🆕 Next Priority)

A comprehensive revision to address the remaining compilation errors and provide a robust foundation:

#### Step 1: Implement Two-Phase Type Resolution
- [ ] **Phase 1: AST-Level Resolution**:
  - [ ] Keep types as `RustType` enum throughout pipeline
  - [ ] Avoid early string conversion
  - [ ] Enable context-aware transformations
- [ ] **Phase 2: Context-Aware String Generation**:
  - [ ] Create `TypeContext` with location info (parameter/return/field)
  - [ ] Include containing class/module information
  - [ ] Track import requirements
  - [ ] Handle generic nesting properly

#### Step 2: Create Centralized Type Registry
- [ ] **Build TypeRegistry**:
  - [ ] Catalog all known CARLA types with correct paths
  - [ ] List standard Rust types (String, Vec, etc.)
  - [ ] Map type aliases to their expansions
  - [ ] Track import requirements per type
- [ ] **Registry Features**:
  - [ ] Type validation during resolution
  - [ ] Path lookup for any type
  - [ ] Import generation helpers

#### Step 3: Implement Smart Type Inference
- [ ] **Context-Aware Defaults**:
  - [ ] `other` in eq/ne methods → same type as self
  - [ ] `actor_ids` → `Vec<i32>`
  - [ ] `callback` → appropriate function signature
  - [ ] Collection parameters → infer from context
- [ ] **Pattern Matching**:
  - [ ] Build parameter name patterns database
  - [ ] Use method name context for inference
  - [ ] Apply CARLA API conventions

#### Step 4: Build Type Resolution Pipeline
- [ ] **Unified TypeResolver Trait**:
  - [ ] Single entry point for all type resolution
  - [ ] Context-aware resolution
  - [ ] Consistent error handling
- [ ] **Pipeline Stages**:
  - [ ] Parser: String → RustType AST
  - [ ] Validator: Check against registry
  - [ ] Enhancer: Add module paths
  - [ ] Optimizer: Handle special cases

#### Step 5: Improve Import Collection
- [ ] **AST-Based Collection**:
  - [ ] Extract imports from RustType nodes
  - [ ] Recursive handling for nested types
  - [ ] Automatic deduplication
- [ ] **Smart Import Generation**:
  - [ ] Only import what's needed
  - [ ] Group imports logically
  - [ ] Handle re-exports properly

#### Step 6: Add Type Validation Layer
- [ ] **Pre-Generation Validation**:
  - [ ] No bare generic types (Vec without <T>)
  - [ ] All module paths exist
  - [ ] No circular dependencies
  - [ ] Valid trait implementations
- [ ] **Error Reporting**:
  - [ ] Clear messages for type errors
  - [ ] Suggest fixes when possible
  - [ ] Source location tracking

#### Step 7: Migration and Testing
- [ ] **Migration Plan**:
  - [ ] Keep old system during transition
  - [ ] Migrate one module at a time
  - [ ] Validate each migration step
- [ ] **Comprehensive Testing**:
  - [ ] Unit tests for each resolver component
  - [ ] Integration tests for full pipeline
  - [ ] Regression tests for fixed issues

### Error Handling Implementation (🆕 High Priority)

Based on test failures showing missing `crate::error` module, implement comprehensive error handling:

**Current Status**: The generated code expects `crate::error::Result` but no error module is generated, causing compilation failures. This is the primary blocker for successful code generation.

**Remaining Issues** (7 compilation errors, down from 141):
1. Missing error module in generated output
2. Unqualified types in return values (`Result<Actor>`, `Result<Vec<Actor>>`)
3. Missing imports (`HashMap`)
4. Wrong module paths (`crate::WeatherParameters` instead of `crate::carla::WeatherParameters`)
5. Over-qualified standard types (`Vec<crate::carla::Vec<f32>>`)

#### Step 1: Fix Generated Code Error Module Usage ✅
- [x] **Update Import Generation**:
  - [x] Ensure generated files use correct error module path
  - [x] Handle both `carla-sys` and `carla` crate contexts
  - [x] Add conditional imports based on target crate
- [x] **Fix Test Harness**:
  - [x] Ensure error module is available at correct path
  - [x] Update test crate structure to match expectations
  - [x] Add proper module re-exports
- [ ] **Generate Error Module in Output**:
  - [ ] Create `error.rs` in output root directory
  - [ ] Generate basic `CarlaError` enum with common variants:
    - `NotImplemented(String)` - For todo! placeholders
    - `FfiError(String)` - For FFI boundary errors
    - `InvalidParameter(String)` - For validation errors
    - `ConversionError(String)` - For type conversions
  - [ ] Add `Result<T>` type alias
  - [ ] Include proper derives (Debug, Error, Clone)
  - [ ] Add documentation explaining usage
  - [ ] Update root `mod.rs` to include error module
- [ ] **Make Error Generation Configurable**:
  - [ ] Add config option to enable/disable error generation
  - [ ] Allow customizing error type name
  - [ ] Choose error handling crate (thiserror vs manual)
  - [ ] Target-specific generation (minimal for carla-sys, rich for carla)
- [ ] **Fix Remaining Type Resolution Issues**:
  - [ ] Fix unqualified `Actor` in return types (e.g., `Result<Actor>`)
    - Issue: Return type wrapping happens after type qualification
    - Solution: Ensure qualification happens before Result wrapping
  - [ ] Fix `HashMap` missing import
    - Issue: HashMap usage not detected in type string parsing
    - Solution: Improve import collection from resolved types
  - [ ] Fix `crate::WeatherParameters` should be `crate::carla::WeatherParameters`
    - Issue: Special handling for `other` parameter in __eq__/__ne__ methods
    - Solution: Ensure parameter type resolution uses correct module path
  - [ ] Fix `Vec<crate::carla::Vec<f32>>` incorrect qualification
    - Issue: Nested generics wrongly qualifying standard types
    - Solution: Fix recursive generic type processing to not qualify std types

#### Step 2: Implement Layer-Specific Error Types
- [ ] **carla-codegen Errors**:
  - [ ] Create `CodegenError` enum with variants:
    - `YamlParse(String)` - YAML parsing failures
    - `TypeResolution(String)` - Type resolution issues
    - `SynGeneration(String)` - AST generation errors
    - `IoError(std::io::Error)` - File I/O errors
  - [ ] Add source location tracking to all errors
  - [ ] Implement error context propagation
- [ ] **carla-sys FFI Errors**:
  - [ ] Create `FfiError` enum with variants:
    - `CxxException(String)` - C++ exceptions
    - `NullPointer` - Null pointer from C++
    - `InvalidUtf8` - String conversion errors
    - `ConversionError(String)` - Type conversion failures
  - [ ] Add FFI boundary error handling
- [ ] **carla High-Level Errors**:
  - [ ] Create `CarlaError` enum with variants:
    - `ConnectionFailed(String)` - Server connection issues
    - `ActorNotFound(ActorId)` - Missing actors
    - `InvalidBlueprint(String)` - Blueprint errors
    - `SimulationError(String)` - Runtime errors
    - `Ffi(carla_sys::FfiError)` - Wrapped FFI errors
  - [ ] Implement error downcasting

#### Step 3: Error Module Convention
- [ ] **Standardize Error Module Structure**:
  - [ ] Document required error module exports
  - [ ] Create error module template
  - [ ] Add validation in code generator
- [ ] **Generated Method Signatures**:
  - [ ] All fallible operations return `Result<T>`
  - [ ] Use `todo!()` with descriptive messages for unimplemented
  - [ ] Add `#[doc]` comments about missing FFI functions

#### Step 4: Error Propagation Flow
- [ ] **Implement Error Chain**:
  - [ ] C++ exceptions → CXX bridge → FfiError
  - [ ] FfiError → CarlaError with context
  - [ ] Add backtrace support for debugging
- [ ] **Rich Error Messages**:
  - [ ] Include operation context
  - [ ] Add suggestions for common issues
  - [ ] Format errors for CLI display

#### Step 5: Testing Error Conditions
- [ ] **Error Test Suite**:
  - [ ] Test each error variant
  - [ ] Verify error propagation
  - [ ] Check error message quality
- [ ] **Integration Tests**:
  - [ ] Test error handling with CARLA server
  - [ ] Verify graceful degradation
  - [ ] Test recovery scenarios

### Known Issues
- Some YAML files have duplicate fields causing warnings
- ~~Complex tuple-like types fail to parse~~ ✅ **FIXED**
- CXX limitations with opaque C++ types
- 12 remaining type resolution errors after initial fixes
- Missing type mappings for some Python types (e.g., `callback`)
- ~~Generic error messages make debugging difficult~~ ✅ **FIXED** (enhanced error reporting implemented)
- **Code Generation Issues** (see [BUGS.md](../carla-codegen/doc/BUGS.md) for details):
  - Type remapping causes module path mismatches (141 compilation errors)
  - Recursive types cause infinite size errors
  - Invalid Copy trait derivations
  - Missing generic type parameters
  - Incorrect enum variant naming

## Phase 2: carla-sys Integration (⏳ Next)
**Goal**: Re-enable carla-sys and integrate code generation

### Tasks
1. **Enable carla-sys in Workspace**:
   - [ ] Re-add carla-sys to workspace Cargo.toml
   - [ ] Update build.rs to use carla-codegen
   - [ ] Configure pre-generated code location
   - [ ] Test compilation with generated types

2. **Implement FFI Functions**:
   - [ ] Review generated C++ headers
   - [ ] Implement core FFI functions in cpp/carla_sys_bridge.cpp
   - [ ] Priority order:
     - Client connection and world access
     - Actor spawning and manipulation
     - Blueprint library access
     - Basic sensor functionality
   - [ ] Test FFI functions with simple C++ tests

3. **CXX Bridge Integration**:
   - [ ] Update src/ffi.rs to include generated declarations
   - [ ] Handle type conversions between C++ and Rust
   - [ ] Implement collection wrappers (WaypointList, etc.)
   - [ ] Add safety documentation

## Phase 3: High-Level carla Crate (⏳ Future)
**Goal**: Create ergonomic Rust wrappers around generated code

### Tasks
1. **Enable carla Crate**:
   - [ ] Re-add carla to workspace Cargo.toml
   - [ ] Import generated types from carla-sys
   - [ ] Set up module structure matching CARLA organization

2. **Implement Wrapper Patterns**:
   - [ ] **Mirror/Wrapper Pattern** for complex types:
     - Client, World, Actor, Vehicle, Walker
     - Add lifetime management and safety guarantees
     - Implement Drop traits for proper cleanup
   
   - [ ] **Trait Extension Pattern** for simple types:
     - Vector3D, Transform, Location, Rotation
     - Add mathematical operations
     - Implement standard traits (Add, Sub, etc.)
   
   - [ ] **Pure Rust Types** for new functionality:
     - Collection helpers and iterators
     - Async wrappers for callbacks
     - Builder pattern implementations

3. **Update Examples**:
   - [ ] Port existing examples to new API
   - [ ] Create comprehensive example suite:
     - Basic connection and world queries
     - Vehicle spawning and control
     - Sensor setup and data processing
     - Traffic scenarios
   - [ ] Add error handling examples

## Implementation Strategy

### Recommended Approach
1. **Focus on carla-codegen completion first** - This is the foundation
2. **Implement carla-sys FFI incrementally** - Start with core functionality
3. **Build carla wrappers iteratively** - Add features as FFI becomes available

### Testing Strategy
- Unit tests for code generation logic
- FFI tests using simple C++ programs
- Integration tests via examples (no server required for compilation)
- Full integration tests with CARLA server

### Quality Checkpoints
- [ ] All YAML files parse without errors
- [ ] Generated code compiles without warnings
- [ ] Basic example (connection + spawn) works
- [ ] All examples from Python API have Rust equivalents

## Known Challenges & Solutions

1. **Complex Type Mappings**:
   - ~~Challenge: Types like `list([name,world, actor, relative])`~~ ✅ **SOLVED**
   - ~~Solution: Create custom type aliases or tuple structs~~ ✅ **IMPLEMENTED**

2. **Union Type Parameters**:
   - ~~Challenge: Python union types like `bool / int / float / str` in method parameters~~ ✅ **SOLVED**
   - ~~Solution: Generate multiple trait implementations for each type in union~~ ✅ **IMPLEMENTED**

3. **Error Debugging Difficulty**:
   - Challenge: Generic error messages like "syn parsing error: unexpected token"
   - Solution: Enhanced error reporting with source context and generated code snippets
   - Status: 📋 **DESIGNED** → Implementation planned

4. **Circular Dependencies**:
   - Challenge: Some CARLA types reference each other
   - Solution: Use forward declarations and smart pointers

5. **CXX Limitations**:
   - Challenge: Cannot return opaque C++ types by value
   - Solution: Use pointers/references and wrapper types

6. **Sensor Callbacks**:
   - Challenge: C++ callbacks to Rust functions
   - Solution: Use CXX's callback support with proper lifetime management

## Success Metrics

- ✅ 100% of CARLA Python API types generated
- ✅ 100% of methods have signatures generated
- ⏳ Core FFI functions implemented (Client, World, Actor)
- ⏳ Basic examples working with CARLA server
- ⏳ Performance comparable to Python client

## Previous Work (Needs Adaptation)

The following work was completed under the old manual implementation approach and needs to be adapted to the new code generation architecture:

| Component | Previous Status | Required Changes |
|-----------|----------------|------------------|
| carla-sys | Had manual FFI bindings | Replace with generated code |
| carla | Manual type definitions | Use generated types + wrappers |
| Examples | Used old API | Port to new generated API |

## Future Considerations

Once the three phases are complete:
- Add async/await support for non-blocking operations
- Implement streaming APIs for sensor data
- Create derive macros for custom user types
- Build higher-level abstractions (scenario DSL, etc.)
- WebAssembly compatibility
- Distributed simulation support

## Contributing

To contribute to the current phase:
1. Check the "Remaining Tasks" in Phase 1
2. Run `make test-codegen` to test code generation
3. Fix any type resolution errors
4. Add tests for your changes
5. Run `make lint` before submitting

## References

- [Architecture Document](ARCH.md) - Overall system design
- [Code Generation Design](../carla-codegen/doc/DESIGN_CODEGEN.md) - Simplified codegen architecture
- [Code Generation Bugs](../carla-codegen/doc/BUGS.md) - Current issues and fixes
- [Error Reporting Design](DESIGN_ERROR_REPORTING.md) - Enhanced error reporting system
- [Rust API Design](DESIGN_RUST_API.md) - API design principles (still applicable)
- [CLAUDE.md](../CLAUDE.md) - Development guidelines
