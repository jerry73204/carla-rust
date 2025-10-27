# CARLA Simulator Symlinks

This directory contains symlinks to CARLA simulator binaries for testing.

## Setup

The symlinks in this directory are placeholders. You need to replace them with actual paths to your CARLA installations.

**Required CARLA versions:**
- 0.9.14
- 0.9.15
- 0.9.16

**Steps:**

1. Download CARLA simulator binaries for each version from [CARLA releases](https://github.com/carla-simulator/carla/releases)

2. Extract each version to a directory on your system

3. Replace the placeholder symlinks with your actual CARLA installation paths:

```bash
# Remove placeholder symlinks
rm simulators/carla-0.9.14
rm simulators/carla-0.9.15
rm simulators/carla-0.9.16

# Create symlinks to your CARLA installations
ln -s /path/to/your/CARLA_0.9.14 simulators/carla-0.9.14
ln -s /path/to/your/CARLA_0.9.15 simulators/carla-0.9.15
ln -s /path/to/your/CARLA_0.9.16 simulators/carla-0.9.16
```

Replace `/path/to/your/CARLA_*` with the actual paths to your CARLA installations.

## Using in Tests

Tests reference simulators using relative paths from project root:

```rust
// Example: Start CARLA simulator for 0.9.16
let simulator_path = "simulators/carla-0.9.16/CarlaUE4.sh";
```

Or use environment variables:

```bash
# Run test with specific CARLA version
CARLA_BIN=simulators/carla-0.9.16/CarlaUE4.sh cargo test
```

## Structure

Each CARLA directory should contain:
- `CarlaUE4.sh` - Main simulator binary (Linux)
- `CarlaUE4/` - Game content
- `PythonAPI/` - Python client library

## Notes

- Each developer maintains their own symlinks pointing to their local CARLA installations
- Simulators can run only one scenario at a time - tests must use exclusive access (see `docs/roadmap.md` Phase 0)
