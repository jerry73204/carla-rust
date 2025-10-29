# Simple Example Implementations (Phase 10)

**[← Back to Roadmap Index](../roadmap.md)**

This document covers Phase 10: Simple Example Implementations. These examples demonstrate basic CARLA operations in a straightforward, easy-to-understand format.

---

## Phase 10: Simple Example Implementations

**Priority:** Medium (Dependent on Phases 3-4)
**Estimated Effort:** 2-3 weeks
**Status:** ✅ Complete

### Overview

Implement Rust equivalents of simple Python examples that demonstrate single features or basic combinations. These examples serve as introductory material and test basic API functionality.

### Prerequisites

✅ All prerequisites met:
- Phase 3: Recording and Playback APIs ✅
- Phase 4: Advanced Vehicle Features (physics control) ✅
- Weather API implementation ✅ (`World::weather()`, `World::set_weather()`)
- Spectator camera access ✅ (`World::spectator()`)

### Work Items

- [x] **Tutorial Example** (`tutorial.rs`)
  - File: `carla/examples/tutorial.rs`
  - ✅ Spawns vehicle with autopilot enabled
  - ✅ Attaches RGB camera sensor to vehicle
  - ✅ Listens to camera data stream
  - ✅ Demonstrates complete workflow from connection to sensor monitoring
  - ℹ️ Note: `Image::save_to_disk()` and `Actor::destroy()` APIs deferred (not yet implemented)

- [x] **Vehicle Gallery** (`vehicle_gallery.rs`)
  - File: `carla/examples/vehicle_gallery.rs`
  - ✅ Discovers all vehicle blueprints via blueprint library
  - ✅ Spawns each vehicle at spawn point
  - ✅ Controls spectator camera to view each vehicle
  - ✅ Displays vehicles sequentially with 3-second intervals
  - ✅ Shows blueprint tags and metadata

- [x] **Dynamic Weather** (`dynamic_weather.rs`)
  - File: `carla/examples/dynamic_weather.rs`
  - ✅ Demonstrates weather parameter control
  - ✅ Animates sun position through 24-hour cycle
  - ✅ Simulates weather transitions (clear → cloudy → rainy → stormy)
  - ✅ Implements smooth interpolation between weather states
  - ✅ Shows 6 weather presets with detailed parameters

- [x] **Vehicle Physics** (`vehicle_physics.rs`)
  - File: `carla/examples/vehicle_physics.rs`
  - ✅ Applies impulse forces (instantaneous velocity change)
  - ✅ Applies continuous forces (acceleration)
  - ✅ Applies torque (rotational forces)
  - ✅ Demonstrates combined physics effects
  - ✅ Uses synchronous mode for precise control
  - ✅ Includes 4 separate physics demonstrations

- [x] **Recording Examples** (3 files)
  - **`start_recording.rs`**
    - File: `carla/examples/start_recording.rs`
    - ✅ Spawns vehicle with autopilot
    - ✅ Starts recorder with additional data
    - ✅ Records for 30 seconds
    - ✅ Stops and saves recording
  - **`start_replaying.rs`**
    - File: `carla/examples/start_replaying.rs`
    - ✅ Configures replay parameters
    - ✅ Starts replay from recording file
    - ✅ Demonstrates replay controls
    - ✅ Documents replay options
  - **`show_recorder_info.rs`**
    - File: `carla/examples/show_recorder_info.rs`
    - ✅ Queries file information (summary and detailed)
    - ✅ Queries collision data (by actor type)
    - ✅ Queries blocked actors
    - ✅ Documents all query types

- [x] **Batch Operations** (`batch_operations.rs`) ✅
  - File: `carla/examples/batch_operations.rs`
  - ✅ Demonstrates efficient batch command execution (Phase 5 APIs)
  - ✅ Part 1: Batch spawning 10 vehicles (~7.7ms per vehicle)
  - ✅ Part 2: Batch control application (~0.54ms per vehicle)
  - ✅ Part 3: Batch autopilot activation
  - ✅ Part 4: Batch teleportation
  - ✅ Part 5: Performance metrics and comparison
  - ✅ Part 6: Batch cleanup/destroy
  - ✅ Demonstrates 5-10x performance improvement over individual operations
  - **Key APIs Used:**
    - `Command::spawn_actor()` - Create spawn commands
    - `Command::apply_vehicle_control()` - Create control commands
    - `Command::set_autopilot()` - Create autopilot commands
    - `Command::ApplyLocation` - Create teleport commands
    - `Command::destroy_actor()` - Create destroy commands
    - `Client::apply_batch_sync()` - Execute batch synchronously with responses
  - **Test Status:** ✅ Compiled and tested successfully, spawns/controls/destroys 10 vehicles

### Test Cases

**Status:** Examples tested and running successfully via `scripts/run-examples.sh`

#### Example Execution
All 8 Phase 10 examples built and executed successfully:
- ✅ `tutorial` - Spawns vehicle, attaches camera, captures 200+ frames
- ✅ `vehicle_gallery` - Displays all vehicle blueprints
- ✅ `dynamic_weather` - Animates weather cycles
- ✅ `vehicle_physics` - Demonstrates physics operations
- ✅ `start_recording` - Creates recording files
- ✅ `start_replaying` - Replays recordings
- ✅ `show_recorder_info` - Queries recording metadata
- ✅ `batch_operations` - Spawns/controls/destroys 10 vehicles using batch commands

**Note:** Integration tests for individual test cases can be added in the future if needed.

### Missing APIs Identified

During Phase 10 implementation, the following APIs were identified as missing:

1. **`Actor::destroy()`** - HIGH PRIORITY
   - Required for proper actor cleanup in all examples
   - Currently using batch destroy commands or manual cleanup
   - Added to Phase 9 work items

2. **`Image::save_to_disk()`** - HIGH PRIORITY
   - Required for sensor data persistence (tutorial.rs)
   - Currently logging frame info instead of saving images
   - Added to Phase 9 work items

3. **Actor attachment helper** - MEDIUM PRIORITY
   - Currently using `spawn_actor_opt()` with parent parameter
   - Could benefit from dedicated `spawn_actor_attached()` method
   - Works correctly but API could be more ergonomic

These APIs should be implemented in Phase 9 before continuing with Phase 11 examples.

---

