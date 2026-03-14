# Testing and Examples Guide

This guide explains how to work with tests and examples in the carla-rust library, including unit tests and Cargo examples that demonstrate CARLA functionality.

## Table of Contents

1. [Test Categories](#test-categories)
2. [Running Examples](#running-examples)
3. [Setting Up the Simulator](#setting-up-the-simulator)
4. [Writing Unit Tests](#writing-unit-tests)
5. [Troubleshooting](#troubleshooting)

---

## Test Categories

The carla-rust library has two main categories of validation:

### 1. Unit Tests

- **Location:** Inline with source code (using `#[cfg(test)]` modules)
- **Purpose:** Test individual functions, data structures, and pure logic
- **Requirements:** No CARLA simulator needed
- **Execution:** Fast (< 1 second per test)
- **Run with:** `cargo test`

### 2. Cargo Examples

- **Location:** `carla/examples/` directory
- **Purpose:** Demonstrate library functionality and serve as runnable documentation
- **Requirements:** Running CARLA simulator required
- **Execution:** Manual, one example at a time
- **Run with:** `cargo run --example <name>`

**Note:** Examples replaced integration tests for demonstrating CARLA simulator interaction. Examples are easier to run, understand, and maintain compared to test infrastructure.

---

## Running Examples

### Prerequisites

Before running examples, you must have a CARLA simulator running on your system.

### Running a Single Example

```bash
# Run any example
cargo run --example connect
cargo run --example spawn_vehicle
cargo run --example walker_control
```

### Available Examples

See `carla/examples/README.md` for a complete list of available examples. Examples are organized into categories:

- **Basic**: `connect`, `world_info`, `blueprints`
- **Vehicles**: `spawn_vehicle`, `multiple_vehicles`, `vehicle_transform`, `vehicle_attributes`
- **Walkers**: `spawn_walker`, `walker_control`, `walker_directions`, `multiple_walkers`

### Learning Path

Recommended order for exploring examples:

1. Start with `connect.rs` to verify simulator connection
2. Explore `world_info.rs` and `blueprints.rs` for basic operations
3. Try vehicle examples: `spawn_vehicle.rs` → `vehicle_transform.rs`
4. Try walker examples: `spawn_walker.rs` → `walker_control.rs`

---

## Setting Up the Simulator

Examples require a running CARLA simulator. Follow these steps:

### Download CARLA

Download CARLA simulator binaries from the [official releases](https://github.com/carla-simulator/carla/releases):

- CARLA 0.9.14
- CARLA 0.9.15
- CARLA 0.9.16 (default)

Extract to a directory on your system.

### Setup Simulator Symlinks (Optional)

The repository can use symlinks to reference your CARLA installations:

```bash
# Navigate to project root
cd /path/to/carla-rust

# Manually create symlinks:
ln -s /path/to/CARLA_0.9.14 simulators/carla-0.9.14
ln -s /path/to/CARLA_0.9.15 simulators/carla-0.9.15
ln -s /path/to/CARLA_0.9.16 simulators/carla-0.9.16
```

### Start the Simulator

Before running examples, start the CARLA simulator:

```bash
# Start CARLA normally
cd /path/to/CARLA_0.9.16
./CarlaUE4.sh

# For headless mode (no graphics, faster):
./CarlaUE4.sh -RenderOffScreen

# For low quality mode (faster):
./CarlaUE4.sh -quality-level=Low
```

The simulator will start on `localhost:2000` by default.

### Verify Simulator is Running

Test connection with the Python API or run the connect example:

```bash
# Using Rust example
cargo run --example connect

# Or using Python
cd /path/to/CARLA_0.9.16/PythonAPI/examples
python3 -c "import carla; client = carla.Client('localhost', 2000); print(client.get_server_version())"
```

---

## Writing Unit Tests

### Creating Unit Tests

Unit tests are placed inline with the source code:

```rust
// In carla/src/geom/location.rs
pub struct Location {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_location_creation() {
        let loc = Location { x: 1.0, y: 2.0, z: 3.0 };
        assert_eq!(loc.x, 1.0);
        assert_eq!(loc.y, 2.0);
        assert_eq!(loc.z, 3.0);
    }

    #[test]
    fn test_location_default() {
        let loc = Location::default();
        assert_eq!(loc.x, 0.0);
        assert_eq!(loc.y, 0.0);
        assert_eq!(loc.z, 0.0);
    }
}
```

### Running Unit Tests

```bash
# Run all unit tests
cargo test

# Run tests for specific crate
cargo test -p carla
cargo test -p carla-sys

# Run specific test
cargo test test_location_creation

# Run tests with output
cargo test -- --nocapture
```

### Test Organization

- **Pure functions:** Test mathematical operations, conversions, etc.
- **Data structures:** Test constructors, getters, setters
- **Traits:** Test trait implementations
- **FFI bindings:** Test that FFI types can be constructed (no simulator needed)

**Do NOT test simulator interaction in unit tests** - use examples for that.

---

## Troubleshooting

### Examples

#### "Failed to connect to CARLA simulator"

**Cause:** CARLA simulator is not running or not accessible.

**Solutions:**
1. Ensure CARLA is running: `./CarlaUE4.sh`
2. Check CARLA is listening on port 2000 (default)
3. Try restarting CARLA
4. Check firewall settings

#### "Failed to spawn vehicle/walker"

**Cause:** Spawn point is occupied or invalid.

**Solutions:**
1. Restart CARLA to clear all actors
2. Try running the example again
3. Some spawn points may be invalid depending on the map

#### Actors remain in simulation after example exits

This is expected behavior. Examples spawn actors but don't destroy them.

**Solution:** Restart CARLA to clear all actors.

**Note:** Actor destruction is not yet implemented in the Rust API.

#### Example runs but nothing happens in simulator window

**Cause:** Example code may have completed before you could observe the result.

**Solutions:**
1. Look at console output from the example
2. Add `std::thread::sleep(Duration::from_secs(5))` to pause before exit
3. Use CARLA's spectator mode to follow spawned actors

### Unit Tests

#### "linking with `cc` failed"

**Cause:** Missing CARLA C++ client library or LLVM toolchain.

**Solutions:**
1. Ensure you have LLVM 11-13 installed: `sudo apt install clang-12 libclang-12-dev`
2. Run `cargo clean` and rebuild
3. Check that `CARLA_VERSION` environment variable is set correctly

#### Compilation errors in carla-sys

**Cause:** Autocxx binding generation issues or LLVM version incompatibility.

**Solutions:**
1. Ensure LLVM version 11-13 (14+ not supported)
2. Clear build cache: `cargo clean`
3. Try rebuilding: `cargo build`

---

## Best Practices

### For Examples

1. **Keep examples simple and focused** - One concept per example
2. **Include clear comments** - Explain what the code does
3. **Use expect() with descriptive messages** - Better error messages for users
4. **Print results to console** - Show what's happening
5. **Document prerequisites** - State simulator requirements clearly

### For Unit Tests

1. **Test one thing per test** - Makes failures easier to debug
2. **Use descriptive test names** - `test_location_distance_calculation`, not `test1`
3. **Test edge cases** - Zero values, negative numbers, empty collections
4. **Avoid simulator dependencies** - Unit tests should be fast and not require CARLA
5. **Use assertions with messages** - `assert!(cond, "Expected X but got Y")`

---

## CI/CD Integration

Unit tests run automatically in CI/CD:

```yaml
# In .github/workflows/ci.yml
- name: Run tests
  run: cargo test
```

Examples are NOT run in CI/CD since they require a running CARLA simulator. They serve as manual verification and documentation.

---

## Further Reading

- [Cargo Book - Tests](https://doc.rust-lang.org/cargo/guide/tests.html)
- [Rust Book - Testing](https://doc.rust-lang.org/book/ch11-00-testing.html)
- [CARLA Documentation](https://carla.readthedocs.io/)
- [Examples README](../carla/examples/README.md)
