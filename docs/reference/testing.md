# Testing Guide

This guide explains how to work with tests in the carla-rust library.

## Table of Contents

1. [Test Categories](#test-categories)
2. [Running Tests](#running-tests)
3. [Integration Test Infrastructure](#integration-test-infrastructure)
4. [Writing Integration Tests](#writing-integration-tests)
5. [Configuration](#configuration)
6. [CARLA Server Setup](#carla-server-setup)
7. [Troubleshooting](#troubleshooting)
8. [Future Work](#future-work)

---

## Test Categories

### 1. Unit Tests

- **Location:** Inline with source code (`#[cfg(test)]` modules)
- **Purpose:** Test pure logic — data structures, geometry, parsing
- **Requirements:** No CARLA simulator needed
- **Run with:** `just test-unit`

### 2. Integration Tests

- **Location:** `carla/tests/`
- **Purpose:** Test library functionality against a real CARLA simulator
- **Requirements:** Running CARLA simulator on port 2000
- **Run with:** `just test-integration` or `just test-integration-external`

### 3. Examples

- **Location:** `carla/examples/`
- **Purpose:** Demonstrate library functionality; runnable documentation
- **Requirements:** Running CARLA simulator
- **Run with:** `cargo run --profile dev-release --example <name>`

---

## Running Tests

### Unit Tests (no CARLA needed)

```bash
just test-unit
```

### Integration Tests — managed mode

`just test-integration` starts CARLA automatically, runs all tests, then stops CARLA on exit (success or failure):

```bash
just test-integration
```

Optional environment variables:

| Variable | Default | Purpose |
|----------|---------|---------|
| `CARLA_TEST_SERVERS` | `1` | Number of concurrent CARLA instances |
| `CARLA_TEST_BASE_PORT` | `2000` | Base port; instance N uses `base + N*10` |
| `CARLA_DIR` | `~/Downloads/CARLA_0.9.16` | Path to CARLA installation |
| `CARLA_MEMORY_MAX` | `10G` | cgroup memory cap for CARLA process |

### Integration Tests — external mode

Start CARLA yourself first, then run tests. CARLA is left running when tests finish:

```bash
# Terminal 1 — keep running between test runs
./scripts/start-carla-test.sh 2000

# Terminal 2 — fast iteration (~1s per run once CARLA is up)
just test-integration-external
```

External mode is the recommended workflow during active development: CARLA starts once and stays up across many `just test-integration-external` runs.

---

## Integration Test Infrastructure

### Architecture

```
.config/nextest.toml            # Test group, timeouts
.config/carla-test.toml         # Probe intervals and world-readiness timeouts
scripts/
├── start-carla-test.sh         # Starts one CARLA instance; blocks until TCP ready
└── carla-config/
    ├── apply.sh                # Patches CARLA INI files for low-memory testing
    ├── restore.sh              # Reverts INI patches
    ├── Engine/Config/Linux/LinuxEngine.ini
    └── CarlaUE4/Config/DefaultEngine.ini
carla/tests/
├── common/
│   ├── mod.rs          # CLIENT / LEASE lazy statics, world() and client() helpers
│   ├── pool.rs         # ServerLease: flock-based pool, world-readiness probe
│   └── config.rs       # Reads .config/carla-test.toml
└── connect.rs          # Scenario: connection, world info, blueprints, spawn
```

### Lifecycle

**`just test-integration`** (managed):
1. Bash starts N CARLA instances via `scripts/start-carla-test.sh` (one per pool slot)
2. Each start script blocks until CARLA's TCP port is ready, then exits
3. `cargo nextest run` launches test binaries
4. Each binary calls `ServerLease::acquire()` → flock on `tmp/test-servers/port-NNNN.lock`
5. `pool.rs` probes `world()` until the game world responds, then runs the test functions
6. On nextest exit (pass or fail), `trap _cleanup` kills all CARLA processes

**`just test-integration-external`** (manual):
- Steps 1–2 are the user's responsibility
- Steps 3–5 are identical
- Step 6 does not run — CARLA is left running

### Server Pool

Each test file (`connect.rs`, future `vehicle_lifecycle.rs`, etc.) is compiled into its own binary. Nextest runs binaries as separate processes. The pool uses `flock(LOCK_EX | LOCK_NB)` on files in `tmp/test-servers/` to give each binary exclusive access to one CARLA instance.

With the default `CARLA_TEST_SERVERS=1` and `max-threads=1` in nextest config, binaries run serially on a single CARLA instance — each one waits for the lock, verifies world readiness, runs its tests, releases the lock.

### World-Readiness Probe

TCP being open does not mean the game world is loaded. After TCP is up, `pool.rs` calls `client.world()` in a loop until it responds or the configured deadline passes. With Town01 (the default test map), world is ready within ~1s of TCP; larger maps may take 60–120s.

If CARLA's TCP port is not open when `ServerLease::acquire()` is called, the test panics immediately with:

```
CARLA server is not running on port 2000.
Start it with:  just test-integration
Or manually:    ./scripts/start-carla-test.sh 2000
```

### CARLA Configuration Patches

`scripts/carla-config/apply.sh` patches two INI files in the CARLA installation before each start. The patches are applied automatically by `start-carla-test.sh`.

Key settings:
- Default map: **Town01** (smallest map; loads fast and uses ~1–2 GB RAM)
- `g.TimeoutForBlockOnRenderFence=300000` — fixes crash on RTX 5090 / driver 580
- Reduced texture streaming pool, mesh distance fields disabled, view distance 50%
- FPS cap at 10 for headless testing

Restore originals: `./scripts/carla-config/restore.sh`

---

## Writing Integration Tests

### Adding a New Scenario

Create `carla/tests/my_scenario.rs`:

```rust
mod common;

use carla::prelude::*;
use common::{client, world};

#[test]
fn t01_something() {
    let w = world();
    // ...
}

#[test]
fn t02_something_else() {
    let mut w = world();
    // ...
}
```

That's it — no registration needed. Nextest discovers all `#[test]` functions in `carla/tests/*.rs` and automatically assigns them to the `carla-server` group.

### Conventions

- **Prefix tests with `tNN_`** for deterministic ordering within a scenario (nextest runs tests from a single binary in alphabetical order)
- **Use `common::world()`** to get a `World` handle; `common::client()` for read-only client operations
- **Import `carla::prelude::*`** for actor trait methods (`id()`, `destroy()`, etc.)
- **Always destroy spawned actors** — the world is shared across all tests in the file
- **Each test should be independent** — do not rely on state left by a previous test

### Example Scenario

```rust
mod common;

use carla::prelude::*;
use common::world;

#[test]
fn t01_spawn_vehicle() {
    let mut w = world();
    let bp = w.blueprint_library().unwrap()
        .find("vehicle.tesla.model3").unwrap().unwrap();
    let spawn = w.map().unwrap()
        .recommended_spawn_points().unwrap()
        .get(0).unwrap();

    let actor = w.spawn_actor(&bp, spawn).expect("spawn failed");
    assert!(actor.id() > 0);
    actor.destroy().unwrap();
}
```

---

## Configuration

### `.config/nextest.toml`

Controls nextest behaviour for the `carla-server` test group:

```toml
[test-groups.carla-server]
max-threads = 1   # serialise test binaries; raise to match CARLA_TEST_SERVERS

[[profile.default.overrides]]
filter = "kind(test) and package(carla)"
test-group = "carla-server"
slow-timeout = { period = "180s", terminate-after = 3 }
threads-required = 1
```

### `.config/carla-test.toml`

Controls how `pool.rs` waits for world readiness:

```toml
[timeouts]
world_probe_secs = 30          # timeout for each world() RPC attempt
world_probe_interval_secs = 5  # sleep between attempts
world_ready_secs = 240         # total budget after TCP is open
```

---

## CARLA Server Setup

### Installation

Download CARLA from the [official releases](https://github.com/carla-simulator/carla/releases).

Supported versions: 0.9.14, 0.9.15, 0.9.16, 0.10.0

Default installation path: `~/Downloads/CARLA_0.9.16` (override with `CARLA_DIR`).

### Start Script

`scripts/start-carla-test.sh PORT` starts CARLA on the given port and blocks until TCP is ready. Key options it applies:

- `-RenderOffScreen` — headless rendering
- `-opengl` — workaround for RTX 5090 / driver 580 Vulkan hang
- `-quality-level=Low` — reduced GPU load
- `-nosound` — suppress ALSA errors on headless servers
- `systemd-run --scope -p MemoryMax=10G` — cgroup memory cap

---

## Troubleshooting

### "CARLA server is not running on port 2000"

Run `just test-integration` (managed, starts CARLA automatically) or `./scripts/start-carla-test.sh 2000` first.

### Tests are very slow on first run

CARLA startup takes ~60s. Subsequent test binaries reuse the running instance and take <1s each. Use `just test-integration-external` during development to avoid paying startup cost on every run.

### CARLA gets OOM-killed

This server has limited free RAM. The INI patches reduce CARLA's footprint significantly, but Town01 still needs ~2–4 GB. If OOM kills persist:
- Check `journalctl --user -n 50` for scope stop events
- Try `CARLA_MEMORY_MAX=8G just test-integration` to adjust the cgroup cap
- Check `/tmp/carla-test-port-2000.log` for CARLA's own output

### "Version mismatch" warning

```
WARNING: Client API version = 7a0b5709a
WARNING: Simulator API version = 0.9.16
```

This is a non-fatal warning from the C++ client library. Tests pass regardless.

---

## Future Work

The infrastructure is in place. Planned test scenarios, roughly in priority order:

### `carla/tests/vehicle_lifecycle.rs`

- Spawn a vehicle at each recommended spawn point
- Apply throttle/brake/steer control inputs
- Step the simulator and verify physics (position changes, velocity non-zero)
- Destroy vehicle; verify it no longer appears in actor list

### `carla/tests/sensors.rs`

- Attach a RGB camera and verify frame callback fires
- Attach a LiDAR sensor and verify point cloud is non-empty
- Attach a collision sensor and trigger a collision
- Attach a GNSS sensor and verify coordinate output

### `carla/tests/traffic_manager.rs`

- Register a vehicle with the Traffic Manager
- Verify autopilot mode engages
- Step simulation and verify vehicle moves autonomously

### `carla/tests/map.rs`

- `waypoint_at()` returns valid waypoints for spawn points
- `topology()` returns a non-empty road graph
- Lane type and road ID fields are populated

### CI Integration

- Run `just test-integration` in CI with a pre-installed CARLA
- Add a separate CI job for unit tests (`just test-unit`) that runs on every PR without CARLA
