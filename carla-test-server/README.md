# carla-test-server

Test infrastructure for CARLA Rust client with automatic server management.

## Overview

This crate provides a clean attribute macro `#[with_carla_server]` that automatically manages CARLA server lifecycle for integration tests. The implementation uses a single-server approach with file-based locking for sequential execution, ensuring tests run without conflicts while allowing non-CARLA tests to run in parallel.

## Features

- **Automatic Server Management**: Each test gets a fresh CARLA server instance
- **Sequential Execution**: File-based locking ensures only one CARLA test runs at a time
- **Process Isolation**: Works seamlessly with cargo test and nextest
- **Flexible Configuration**: TOML-based configuration with sensible defaults
- **Clean API**: Simple attribute macro with two usage styles
- **Proper Cleanup**: Guaranteed server cleanup even on test panic

## Usage

Add to your `Cargo.toml`:

```toml
[dev-dependencies]
carla-test-server = { path = "../carla-test-server" }
```

### Example Tests

#### Style 1: Explicit Client Parameter

```rust
use carla_test_server::with_carla_server;

#[with_carla_server]
fn test_client_connection(client: &carla::client::Client) {
    let version = client.server_version()
        .expect("Failed to get server version");
    assert!(!version.is_empty());
}
```

#### Style 2: Implicit Client in Scope

```rust
use carla_test_server::with_carla_server;

#[with_carla_server]
fn test_world_operations() {
    // client is automatically available in scope
    let world = client.world().expect("Failed to get world");
    let map = world.map().expect("Failed to get map");
    assert!(!map.name().is_empty());
}
```

## Configuration

Create a `carla_server.toml` file in your project:

```toml
[server]
executable_path = "/path/to/carla/CarlaUnreal.sh"
port = 2000
startup_timeout_seconds = 30
shutdown_timeout_seconds = 10
quality_level = "Low"
windowed = false
additional_args = ["-nosound", "-benchmark"]

[coordination]
lock_file = "/tmp/carla_test_server.lock"
timeout_seconds = 300

[logging]
log_dir = "/tmp/carla_test_logs"
capture_output = true
```

## Architecture

### Components

1. **Procedural Macro** (`carla-test-server-macros`): Transforms test functions
2. **Server Management**: Process lifecycle, health checks, and cleanup
3. **Coordination**: File-based locking for sequential execution
4. **Configuration**: TOML-based settings with validation

### Key Types

- `ServerCoordinator`: Trait for different coordination strategies
- `FileLockCoordinator`: File-based implementation for sequential execution
- `CarlaTestServer`: Manages CARLA server process lifecycle
- `ServerResource`: RAII wrapper ensuring proper cleanup

## Running Tests

```bash
# Run all tests (CARLA tests run sequentially)
DISPLAY=:1 cargo test

# Run with nextest (recommended)
DISPLAY=:1 cargo nextest run

# Run specific test
DISPLAY=:1 cargo test test_client_connection
```

## Environment Variables

- `DISPLAY`: Required for windowed mode (automatically propagated)
- `LD_LIBRARY_PATH`: Preserved for shared libraries
- `PATH`: Preserved for finding executables
- `HOME`: Preserved for user settings

## Debugging

Server logs are captured to the configured log directory:
- `carla_server_PORT_stdout.log`
- `carla_server_PORT_stderr.log`

Enable debug logging:
```bash
RUST_LOG=carla_test_server=debug cargo test
```

## Future Enhancements

The architecture supports future extensions:
- Server pooling for parallel CARLA tests
- Docker/container support
- Mock server for unit tests
- Per-test configuration overrides