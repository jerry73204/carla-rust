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

### Remaining Tasks
- [ ] Fix type resolution for complex nested types (e.g., `list([name,world, actor, relative])`)
- [ ] Handle circular dependencies between types
- [ ] Improve error messages for type mapping failures
- [ ] Add validation for generated code
- [ ] Continue-on-error mode for partial generation

### Known Issues
- Some YAML files have duplicate fields causing warnings
- Complex tuple-like types fail to parse
- CXX limitations with opaque C++ types

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
   - Challenge: Types like `list([name,world, actor, relative])`
   - Solution: Create custom type aliases or tuple structs

2. **Circular Dependencies**:
   - Challenge: Some CARLA types reference each other
   - Solution: Use forward declarations and smart pointers

3. **CXX Limitations**:
   - Challenge: Cannot return opaque C++ types by value
   - Solution: Use pointers/references and wrapper types

4. **Sensor Callbacks**:
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
- [Code Generation Design](DESIGN_CODEGEN.md) - Detailed codegen architecture
- [Rust API Design](DESIGN_RUST_API.md) - API design principles (still applicable)
- [CLAUDE.md](../CLAUDE.md) - Development guidelines
