# CARLA Rust Client Design Principles

## Overview

This document outlines the core design principles and philosophy behind the CARLA Rust client library. These principles guide architectural decisions, API design, and implementation choices.

## Core Design Philosophy

### 1. **Safety First**
Rust's memory safety guarantees are paramount. The library should leverage Rust's type system to prevent common C++ pitfalls.

**Implementation**:
- Use CXX for safe C++ interoperability
- Wrap raw pointers in safe Rust types
- Leverage RAII for resource management
- Validate invariants at compile-time where possible

**Example**:
```rust
// Safe wrapper around C++ CARLA types
pub struct Actor {
    inner: SharedPtr<ffi::Actor>,  // CXX managed pointer
    world_ref: Arc<World>,         // Ensure world outlives actor
}
```

### 2. **Zero-Cost Abstractions**
High-level APIs should not introduce runtime overhead compared to direct C++ usage.

**Implementation**:
- Zero-copy data structures where possible
- Compile-time code generation
- Inlined wrapper functions
- Efficient memory layouts

**Example**:
```rust
// Zero-cost wrapper around C++ vector data
pub struct WaypointList<'a> {
    data: &'a [ffi::Waypoint],  // Direct reference to C++ data
}
```

### 3. **Ergonomic APIs**
The Rust API should feel natural to Rust developers and leverage Rust idioms.

**Rust Conventions**:
- Use `Result<T, E>` for fallible operations
- Implement standard traits (`Debug`, `Clone`, `PartialEq`)
- Follow Rust naming conventions (`snake_case` methods)
- Use builders for complex configuration
- Provide iterator interfaces

**Example**:
```rust
// Rust-idiomatic API
let vehicles: Vec<Vehicle> = world
    .actors()
    .filter_map(|actor| actor.try_into_vehicle().ok())
    .filter(|vehicle| vehicle.velocity().length() > 10.0)
    .collect();
```

### 4. **Predictable Resource Management**
Resource lifetimes should be clear and managed automatically where possible.

**Strategies**:
- Use RAII for automatic cleanup
- Clear ownership semantics
- Explicit lifetime relationships
- Async-friendly resource handling

### 5. **Comprehensive Error Handling**
All failure modes should be represented in the type system with helpful error messages.

**Error Design**:
```rust
#[derive(Debug, thiserror::Error)]
pub enum CarlaError {
    #[error("Connection failed: {reason}")]
    ConnectionFailed { reason: String },
    
    #[error("Actor {id} not found")]
    ActorNotFound { id: u32 },
    
    #[error("Invalid configuration: {field} = {value}")]
    InvalidConfig { field: String, value: String },
}
```

## API Design Guidelines

### Method Naming
- Use Rust conventions: `get_id()` → `id()`
- Be descriptive: `apply_control()` not `apply()`
- Use consistent terminology across the API

### Return Types
- **Fallible operations**: `Result<T, CarlaError>`
- **Optional values**: `Option<T>`
- **Collections**: Use standard Rust collections or custom zero-copy wrappers
- **Builders**: Return `Self` for method chaining

### Parameter Design
- **Borrowing**: Use `&self` for read-only operations
- **Ownership**: Take ownership for configuration objects
- **References**: Use `&T` for large objects, `T` for small ones
- **Generics**: Use bounded generics for flexibility

### Async Design
- **Future-ready**: Design APIs to support async in the future
- **Non-blocking**: Prefer non-blocking operations where possible
- **Cancellation**: Support operation cancellation

## Type System Design

### Wrapper Types
All CARLA C++ types should have corresponding Rust wrapper types:

```rust
// FFI layer (carla-sys)
pub mod ffi {
    pub struct Actor { /* CXX bridge */ }
    pub struct World { /* CXX bridge */ }
}

// High-level API (carla)
pub struct Actor {
    inner: SharedPtr<ffi::Actor>,
    // Additional Rust-specific state
}
```

### Trait Implementation
Implement standard Rust traits where semantically appropriate:

```rust
impl Debug for Actor {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Actor")
            .field("id", &self.id())
            .field("type_id", &self.type_id())
            .finish()
    }
}

impl PartialEq for Actor {
    fn eq(&self, other: &Self) -> bool {
        self.id() == other.id()
    }
}
```

### Lifetime Management
Use Rust's lifetime system to enforce correct usage:

```rust
pub struct Waypoint<'w> {
    inner: SharedPtr<ffi::Waypoint>,
    _world: PhantomData<&'w World>,  // Waypoint can't outlive World
}
```

## Code Generation Design

### Generated vs Manual Code
- **Generated**: Basic types, method stubs, simple wrappers
- **Manual**: Complex logic, Rust-specific features, optimizations

### Stub Philosophy
Generate method stubs with clear TODOs rather than silent failures:

```rust
pub fn get_semantic_tags(&self) -> Result<Vec<SemanticTag>> {
    // TODO: Implement using carla-sys FFI interface
    // This requires adding Actor_GetSemanticTags FFI function
    todo!("Actor::get_semantic_tags not yet implemented - missing FFI function Actor_GetSemanticTags")
}
```

### Documentation Generation
Generated code should include comprehensive documentation:

```rust
/// Gets the actor's current velocity vector.
///
/// # Returns
/// The velocity vector in m/s
///
/// # Example
/// ```rust,no_run
/// let velocity = actor.velocity()?;
/// println!("Speed: {} m/s", velocity.length());
/// ```
pub fn velocity(&self) -> Result<Vector3D> { ... }
```

## Testing Strategy

### Unit Tests
- Test individual components in isolation
- Mock external dependencies
- Focus on edge cases and error conditions

### Integration Tests
- Test full workflows end-to-end
- Use real CARLA server when available
- Graceful degradation when server unavailable

### Property-Based Testing
- Generate random inputs for robustness testing
- Verify invariants hold across operations
- Test serialization/deserialization roundtrips

### Documentation Tests
- All public APIs should have working examples
- Examples should be realistic use cases
- Use `no_run` for examples requiring CARLA server

## Performance Considerations

### Memory Management
- Minimize allocations in hot paths
- Reuse buffers where possible
- Use zero-copy techniques
- Consider memory-mapped data for large datasets

### FFI Optimization
- Batch operations when possible
- Minimize string conversions
- Use move semantics for large objects
- Cache frequently accessed data

### Compilation Time
- Use pre-generated code to avoid build-time dependencies
- Minimize template instantiation
- Reduce macro complexity
- Optimize dependency graph

## Backward Compatibility

### API Versioning
- Follow semantic versioning (SemVer)
- Deprecation warnings before removal
- Migration guides for breaking changes
- Multiple version support when feasible

### CARLA Version Support
- Support multiple CARLA versions when possible
- Feature detection for version-specific functionality
- Clear documentation of version requirements

## Documentation Standards

### Code Comments
- Focus on "why" not "what"
- Explain non-obvious design decisions
- Include safety considerations
- Reference relevant CARLA documentation

### API Documentation
- Complete parameter descriptions
- Clear return value semantics
- Comprehensive examples
- Error condition documentation

### Architecture Documentation
- High-level design rationale
- Component interaction diagrams
- Performance characteristics
- Security considerations

## Hybrid Architecture Implementation

### Implementation Workflow

#### **Phase 1: Foundation Setup**
```bash
# Archive existing implementation
mkdir -p archive/carla-old-$(date +%Y%m%d)
cp -r carla/src/* archive/carla-old-$(date +%Y%m%d)/

# Clear for fresh start
rm -rf carla/src/*
```

#### **Phase 2: Enhanced Code Generation**
Configure carla-codegen for comprehensive coverage:

```toml
[generation]
generate_constructors = true
generate_conversions = true
generate_debug_impls = true
generate_default_impls = true
generate_builder_patterns = true

[type_mapping]
"carla.Vector3D" = "crate::geom::Vector3D"
"carla.Transform" = "crate::geom::Transform"
"carla.Actor" = "crate::actor::Actor"
```

#### **Phase 3: Wrapper Implementation**
Create ergonomic wrappers for complex types:

```rust
// carla/src/actor.rs
pub struct Actor {
    inner: carla_sys::generated::Actor,
    world_ref: std::sync::Weak<crate::World>,
}

impl Actor {
    // Delegate to generated
    pub fn get_velocity(&self) -> crate::Result<Vector3D> {
        self.inner.get_velocity()
    }
    
    // Ergonomic API
    pub fn velocity(&self) -> crate::Result<Vector3D> {
        self.get_velocity()
    }
    
    // Safe variants
    pub fn try_velocity(&self) -> Option<Vector3D> {
        self.velocity().ok()
    }
}
```

#### **Phase 4: Trait Extensions**
Add functionality to simple types via traits:

```rust
pub trait Vector3DExt {
    fn length(&self) -> f32;
    fn normalize(&self) -> Vector3D;
    fn dot(&self, other: &Vector3D) -> f32;
}

impl Vector3DExt for Vector3D {
    fn length(&self) -> f32 {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }
    
    fn normalize(&self) -> Vector3D {
        let len = self.length();
        if len > 0.0 {
            Vector3D {
                x: self.x / len,
                y: self.y / len,
                z: self.z / len,
            }
        } else {
            *self
        }
    }
}
```

### Benefits of Hybrid Approach

1. **Clear Ownership**: All user-facing types owned by carla crate
2. **Comprehensive Coverage**: Generated code ensures complete API coverage
3. **Ergonomic APIs**: Builder patterns, trait extensions, type safety
4. **Future-Proof**: Easy to add async support and new features
5. **Developer Experience**: Intuitive API following Rust conventions

## Future-Proofing

### Extensibility
- Design for extension without modification
- Use trait-based interfaces for simple types
- Wrapper pattern for complex behavior
- Support for custom types and behaviors
- Plugin-like architecture for advanced features

### Technology Evolution
- Async/await readiness with prepared stream types
- WASM compatibility considerations
- Integration with emerging Rust ecosystem tools
- Support for new CARLA features through regeneration