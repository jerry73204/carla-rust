# Actor Management in CARLA Rust

This document describes the findings from testing actor management functionality in the CARLA Rust client library, based on running examples against CARLA 0.10.0.

## Overview

The CARLA Rust library provides comprehensive actor management through a layered architecture:
- **High-level Rust API**: Safe, idiomatic interfaces in the `carla` crate
- **FFI Bridge**: C++ bridge layer in `carla-sys` using CXX
- **CARLA C++ Client**: Underlying CARLA client library

## Key Findings from Example Testing

### 1. Actor Types and Blueprint System

The blueprint system works reliably for discovering available actor types:
- **Vehicles**: 11 blueprints available (e.g., `vehicle.mini.cooper`, `vehicle.dodge.charger`)
- **Sensors**: 16 sensor types
- **Walkers**: 37 pedestrian models  
- **Props**: 83 static props
- **Traffic**: Traffic lights and signs are spawned automatically with the map

Notable finding: Tesla Model 3 (`vehicle.tesla.model3`) is not available in CARLA 0.10.0, despite being commonly referenced in documentation.

### 2. Actor Spawning

Vehicle spawning works but has stability issues:

```rust
// Spawning works
let actor = world.spawn_actor(&vehicle_bp, &spawn_point, None)?;

// Actor is created successfully
println!("Actor ID: {}", actor.id());        // Works
println!("Type ID: {}", actor.type_id());    // Works
```

However, spawned actors may become invalid immediately in some cases, as evidenced by `is_alive()` returning false right after spawning.

### 3. Actor Properties and Methods

Accessing actor properties can cause crashes in CARLA 0.10.0:

```rust
// These operations may cause std::exception crashes:
let transform = vehicle.transform();  // CRASH
let velocity = vehicle.velocity();    // CRASH
vehicle.set_autopilot(true, None)?;  // CRASH (Traffic Manager not available)
```

The crashes appear to be related to the unstable nature of CARLA 0.10.0 and missing subsystems like the Traffic Manager.

### 4. Actor Destruction

Based on the design in DESTROY.md, the library implements a sophisticated destruction system:

#### Current Implementation
- **Automatic destruction via Drop trait**: Actors are destroyed when going out of scope
- **FFI layer**: `Actor_Destroy(&self) -> bool` bridges to C++
- **C++ handling**: Manages RPC calls to server and episode cleanup
- **Error handling**: Logs failures but doesn't propagate errors in Drop

#### Destruction Behavior Observed
When attempting to destroy traffic infrastructure actors:
```
ERROR: failed to destroy actor 24 : std::exception
ERROR: failed to destroy actor 23 : std::exception
...
```

This is expected behavior - traffic lights, signs, and other infrastructure actors cannot be destroyed as they are managed by the map.

### 5. Static vs Dynamic Actors

The library distinguishes between:
- **Dynamic actors**: Vehicles, walkers, sensors (can be spawned/destroyed)
- **Static actors**: Traffic lights, signs, infrastructure (permanent map features)

Attempting to destroy static actors results in errors, which is correct behavior.

### 6. Actor Collections

The `ActorList` type provides efficient iteration over actors:
```rust
let actors = world.actors()?;
println!("Total actors: {}", actors.len());

// Group by type
for actor in actors.iter() {
    let type_id = actor.type_id();
    // Process actor...
}
```

## Stability Issues in CARLA 0.10.0

### Observed Problems
1. **Server crashes**: Frequent segmentation faults, especially when:
   - Accessing vehicle properties (transform, velocity)
   - Enabling autopilot without Traffic Manager
   - Applying vehicle controls

2. **Actor lifecycle issues**: 
   - Actors may be destroyed immediately after spawning
   - `is_alive()` returns false for newly spawned actors

3. **Missing subsystems**:
   - Traffic Manager not available/running
   - Autopilot functionality crashes

### Crash Patterns
From the server logs:
```
Signal 11 caught.
CommonUnixCrashHandler: Signal=11
Segmentation fault (core dumped)
```

These crashes occur when the Rust client attempts certain operations, suggesting instability in the CARLA 0.10.0 server implementation.

## Best Practices

### 1. Error Handling
Always handle potential failures gracefully:
```rust
match vehicle.set_autopilot(true, None) {
    Ok(_) => println!("Autopilot enabled"),
    Err(e) => {
        println!("Could not enable autopilot: {}", e);
        // Fall back to manual control or skip
    }
}
```

### 2. Actor Validation
Check actor state before operations:
```rust
if actor.is_alive() {
    // Perform operations
} else {
    println!("Actor is no longer valid");
}
```

### 3. Cleanup
While actors have automatic cleanup via Drop, explicit cleanup is recommended:
```rust
// Convert specialized types back to Actor for destruction
let mut actor = vehicle.into_actor();
actor.destroy()?;
```

### 4. Avoid Problematic Operations
In CARLA 0.10.0, avoid or carefully handle:
- Accessing transform/velocity of vehicles
- Enabling autopilot
- Applying vehicle controls
- Destroying static infrastructure

## Future Improvements

Based on DESTROY.md, planned enhancements include:
1. **Explicit destroy() method**: Better error handling than Drop
2. **Atomic state tracking**: Thread-safe destruction state
3. **Enhanced error types**: Specific errors for destruction scenarios
4. **Operation validation**: Prevent operations on destroyed actors

## Example Code Patterns

### Safe Actor Spawning
```rust
// Get blueprint and spawn point
let vehicle_bp = blueprint_library.find("vehicle.mini.cooper")??;
let spawn_point = map.spawn_points().get(0).unwrap();

// Spawn with error handling
let actor = world.spawn_actor(&vehicle_bp, &spawn_point, None)?;

// Verify actor is valid before use
if actor.is_alive() {
    // Safe to use actor
} else {
    println!("Warning: Actor invalid immediately after spawn");
}
```

### Safe Actor Cleanup
```rust
// For dynamic actors only
fn cleanup_actors(world: &World) -> Result<()> {
    let actors = world.actors()?;
    
    for actor in actors.iter() {
        // Skip infrastructure actors
        if !actor.type_id().contains("traffic") && 
           actor.type_id() != "spectator" {
            actor.destroy().ok(); // Ignore errors for already destroyed
        }
    }
    Ok(())
}
```

## Conclusion

The CARLA Rust library provides a comprehensive actor management API that successfully wraps the C++ CARLA client. However, CARLA 0.10.0 exhibits significant stability issues that limit the practical use of many actor operations. The library handles these gracefully where possible, but users should be prepared for server crashes when performing certain operations.

The planned improvements in DESTROY.md will enhance the safety and usability of actor management, particularly around destruction semantics and error handling.