# Testing Guide

This guide explains how to work with tests in the carla-rust library.

## Table of Contents

1. [Test Categories](#test-categories)
2. [Running Tests](#running-tests)
3. [Integration Test Infrastructure](#integration-test-infrastructure)
4. [Writing Integration Tests](#writing-integration-tests)
5. [Setting Up the Simulator](#setting-up-the-simulator)
6. [Troubleshooting](#troubleshooting)

---

## Test Categories

### 1. Unit Tests

- **Location:** Inline with source code (using `#[cfg(test)]` modules)
- **Purpose:** Test individual functions, data structures, and pure logic
- **Requirements:** No CARLA simulator needed
- **Execution:** Fast (< 1 second per test)
- **Run with:** `just test-unit`

### 2. Integration Tests

- **Location:** `carla/tests/` directory
- **Purpose:** Test library functionality against a real CARLA simulator
- **Requirements:** Running CARLA simulator required
- **Execution:** Discovered and run by nextest
- **Run with:** `just test-integration`

### 3. Examples

- **Location:** `carla/examples/` directory
- **Purpose:** Demonstrate library functionality and serve as runnable documentation
- **Requirements:** Running CARLA simulator required
- **Execution:** Manual, one example at a time
- **Run with:** `cargo run --profile dev-release --example <name>`

---

## Running Tests

### Unit Tests (no CARLA needed)

```bash
just test-unit

# Or directly:
cargo nextest run --cargo-profile dev-release -E 'kind(lib)'
```

### Integration Tests (CARLA required)

**External mode** — start CARLA yourself, then run tests:

```bash
# Start CARLA on port 2000 (default)
./CarlaUE4.sh -RenderOffScreen -quality-level=Low

# In another terminal:
just test-integration
```

**Managed mode** — provide a start script and tests manage CARLA automatically:

```bash
CARLA_TEST_START_SCRIPT=./scripts/start-carla-test.sh just test-integration
```

**Parallel servers** — run multiple CARLA instances for faster test execution:

```bash
CARLA_TEST_SERVERS=2 \
CARLA_TEST_START_SCRIPT=./scripts/start-carla-test.sh \
cargo nextest run --cargo-profile dev-release -E 'kind(test)' \
    --test-group-max-threads carla-server=2
```

### All Tests (unit + integration)

```bash
just test          # Unit tests across all CARLA versions (no simulator)
just test-integration  # Integration tests (requires simulator)
```

---

## Integration Test Infrastructure

### Architecture

```
.config/nextest.toml          # Test group: carla-server (max-threads=1)
carla/tests/
├── common/
│   ├── mod.rs                # CLIENT/LEASE lazy statics, world()/client() helpers
│   ├── pool.rs               # ServerLease: file-lock allocator, port management
│   └── process.rs            # ManagedCarla: process group, graceful kill
├── connect.rs                # Scenario: connection, world info, maps, blueprints
└── ...                       # More scenarios over time
```

### Server Pool

The infrastructure manages a **pool of CARLA server instances**. Each test binary acquires exclusive access to one server via file locks in `tmp/test-servers/`.

**Environment variables:**

| Variable | Default | Purpose |
|----------|---------|---------|
| `CARLA_TEST_SERVERS` | `1` | Number of concurrent CARLA instances |
| `CARLA_TEST_BASE_PORT` | `2000` | Base port; instance N uses `base + N*10` |
| `CARLA_TEST_START_SCRIPT` | (none) | Script to start CARLA. Receives port as `$1`. If unset, assumes servers are already running. |

### Nextest Configuration

Integration tests are assigned to the `carla-server` test group with `max-threads=1` (serial by default). Override for parallel execution:

```bash
cargo nextest run -E 'kind(test)' --test-group-max-threads carla-server=N
```

### Process Safety

When using managed mode (`CARLA_TEST_START_SCRIPT`), child processes are:
- Spawned in their own process group
- Protected with `PR_SET_PDEATHSIG(SIGKILL)` for orphan prevention
- Gracefully killed (SIGTERM → wait → SIGKILL) on lease drop

---

## Writing Integration Tests

### Test Organization

Each test file is a **scenario** — a group of related tests that share a single CARLA connection:

```rust
// carla/tests/my_scenario.rs
mod common;

use carla::prelude::*;
use common::{client, world};

#[test]
fn t01_first_test() {
    let w = world();
    // ...
}

#[test]
fn t02_second_test() {
    let mut w = world();
    // ...
}
```

### Conventions

- **Prefix tests with `tNN_`** for deterministic ordering (nextest runs tests alphabetically)
- **Use `common::client()`** for read-only operations
- **Use `common::world()`** to get a `World` handle
- **Import `carla::prelude::*`** for trait methods like `destroy()`, `id()`
- **Clean up spawned actors** in each test (call `actor.destroy()`)

### Adding a New Scenario

1. Create `carla/tests/my_scenario.rs`
2. Add `mod common;` at the top
3. Write `#[test]` functions with `tNN_` prefixes
4. Tests automatically join the `carla-server` nextest group

---

## Setting Up the Simulator

### Download CARLA

Download CARLA simulator from the [official releases](https://github.com/carla-simulator/carla/releases).

Supported versions: 0.9.14, 0.9.15, 0.9.16, 0.10.0

### Start the Simulator

```bash
# Normal mode
./CarlaUE4.sh

# Headless (for CI/testing)
./CarlaUE4.sh -RenderOffScreen

# Low quality (faster)
./CarlaUE4.sh -RenderOffScreen -quality-level=Low

# Custom port
./CarlaUE4.sh -RenderOffScreen -carla-rpc-port=2010
```

### Example Start Script

For managed mode, create a script like `scripts/start-carla-test.sh`:

```bash
#!/bin/bash
# $1 = port number
export VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json
exec /opt/carla/CarlaUE4.sh -RenderOffScreen -quality-level=Low -carla-rpc-port=${1:-2000}
```

---

## Troubleshooting

### "CARLA server must be running"

Tests assume a CARLA simulator is running. Either:
1. Start CARLA manually on port 2000
2. Set `CARLA_TEST_START_SCRIPT` to auto-start

### Tests hang or timeout

- Check CARLA is responsive: `cargo run --profile dev-release --example connect`
- Default timeout is 120s per test (configurable in `.config/nextest.toml`)
- If CARLA crashed, restart it and re-run tests

### "Failed to spawn vehicle"

- Restart CARLA to clear existing actors
- Some spawn points may be occupied

### Linking errors

- Ensure `libclang-dev` is installed
- Run `cargo clean` and rebuild

---

## Best Practices

1. **One concept per scenario file** — keep test files focused
2. **Clean up after yourself** — destroy spawned actors
3. **Use descriptive test names** — `t01_connect_and_version`, not `t01_test`
4. **Don't rely on test ordering** — each test should set up its own preconditions
5. **Use `expect()` with messages** — better failure diagnostics
