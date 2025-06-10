# carla-sys2

Bindgen-based FFI bindings for the CARLA simulator C library.

This crate provides low-level Rust bindings to `libcarla_c`, a C wrapper around the CARLA simulator's C++ client library. The bindings are generated automatically using [bindgen](https://github.com/rust-lang/rust-bindgen).

## Requirements

- **libcarla_c**: The C wrapper library must be built first
- **CARLA 0.10.0**: The CARLA simulator source code (included as submodule)
- **clang/LLVM**: Required by bindgen for parsing C headers

## Building

1. **Build the C library**:
   ```bash
   cd ../libcarla_c
   ./build_libcarla_c.sh
   ```

2. **Build carla-sys2**:
   ```bash
   cargo build
   ```

3. **Run tests**:
   ```bash
   LD_LIBRARY_PATH=../libcarla_c/install/lib cargo test
   ```

## Generated Bindings

The crate automatically generates Rust bindings for:

- **Client API**: Connection, world management (`carla_client_*`)
- **World API**: Simulation control, actor management (`carla_world_*`)  
- **Actor API**: Vehicle/pedestrian control (`carla_actor_*`)
- **Map API**: Road network queries (`carla_map_*`, `carla_waypoint_*`)
- **Types**: Geometry, errors, enums (`carla_vector3d_t`, `carla_error_t`, etc.)

## Safety

All functions are marked `unsafe` as they directly interface with C code. Callers must ensure:
- Pointers are valid and properly aligned
- Memory is not accessed after being freed  
- C strings are null-terminated
- Objects are not used after destruction

## Example Usage

```rust
use carla_sys2::*;

unsafe {
    // Connect to CARLA server
    let client = carla_client_new(b"localhost\0".as_ptr() as *const i8, 2000, 1);
    
    // Get world
    let world = carla_client_get_world(client);
    
    // Tick simulation
    let frame = carla_world_tick(world, 5000);
    
    // Cleanup
    carla_world_free(world);
    carla_client_free(client);
}
```

## Architecture

- **C Headers**: Located in `../libcarla_c/include/carla_c/`
- **Shared Library**: Built to `../libcarla_c/install/lib/libcarla_c.so`
- **Bindings**: Generated to `$OUT_DIR/bindings.rs` during build
- **Wrapper**: `wrapper.h` includes all C headers for bindgen

This approach provides a simpler alternative to the autocxx-based `carla-sys` crate for users who prefer traditional bindgen workflows.