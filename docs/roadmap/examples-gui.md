# GUI Example Implementations (Phases 12-14)

**[← Back to Roadmap Index](../roadmap.md)**

This document covers Phases 12-14: GUI Example Implementations with Macroquad. These examples demonstrate interactive CARLA applications with real-time visualization, keyboard controls, and comprehensive feature showcases.

**Last Updated:** 2025-11-07

## Contents

- [Foundation: Error Handling Improvements](#foundation-error-handling-improvements) ✅ COMPLETE
- [Phase 12: Manual Control - Complete Interactive Example](#phase-12-manual-control---complete-interactive-example) ✅ COMPLETE
- [Phase 13: Core GUI Examples](#phase-13-core-gui-examples)
  - [13.1: Automatic Control](#phase-131-automatic-control)
  - [13.2: Synchronous Mode GUI](#phase-132-synchronous-mode-gui)
  - [13.3: Dynamic Weather GUI](#phase-133-dynamic-weather-gui)
- [Phase 14: Advanced GUI Examples](#phase-14-advanced-gui-examples)
  - [14.1: Bounding Boxes Visualization](#phase-141-bounding-boxes-visualization)
  - [14.2: Multiple Sensor Grid View](#phase-142-multiple-sensor-grid-view)
  - [14.3: No Rendering Mode (Performance Demo)](#phase-143-no-rendering-mode-performance-demo)
  - [14.4: Walker Skeleton Visualization](#phase-144-walker-skeleton-visualization)
- [Phase 15: Specialized GUI Examples (Optional)](#phase-15-specialized-gui-examples-optional)
  - [15.1: Steering Wheel Support](#phase-151-steering-wheel-support)
  - [15.2: LiDAR 3D Visualization](#phase-152-lidar-3d-visualization-optional)

---

## Foundation: Error Handling Improvements

**Priority:** HIGH (Foundation for all examples)
**Status:** ✅ COMPLETE (2025-11-07)
**Completion:** Phases 1-3 Complete
**Reference:** See `docs/roadmap/error-handling.md` for full design

### Summary

Implemented structured error handling to replace generic `anyhow` errors with actionable, classified error types. This foundation improves error recovery and diagnostics in all CARLA applications.

### Key Features

✅ **7 Error Categories:**
- `ConnectionError` - Network issues, timeouts, disconnections
- `ResourceError` - Blueprint/actor/map not found
- `OperationError` - Spawn failures, simulation errors
- `ValidationError` - Invalid configuration, type mismatches
- `MapError` - Map loading/topology issues
- `SensorError` - Sensor configuration/data errors
- `InternalError` - FFI errors, unexpected states

✅ **Helper Methods:**
- `is_connection_error()` - Detect network/server issues
- `is_timeout()` - Detect operation timeouts
- `is_not_found()` - Detect missing resources
- `is_retriable()` - Determine if retry is appropriate
- `is_validation_error()` - Detect invalid input

✅ **FFI Error Classification:**
- Smart C++ exception parsing using message patterns
- Duration extraction from timeout messages ("5 seconds" → `Duration`)
- Resource type identification (blueprint, actor, sensor)
- Blueprint ID extraction from error messages

### Implementation

**Files Created:**
- `carla/src/error.rs` (670+ lines) - Core error types
- `carla/src/error/ffi.rs` (320 lines) - FFI parsing
- `carla-sys/csrc/carla_rust/error.hpp` (130 lines) - C++ classification

**APIs Migrated:**
- `World::spawn_actor()` → `OperationError::SpawnFailed`
- `World::wait_for_tick()` → `ConnectionError::Timeout`
- `ActorBuilder::new()` → `ResourceError::NotFound`
- `ActorBuilder::set_attribute()` → `ValidationError::InvalidConfiguration`
- `ActorBuilder::spawn_vehicle/sensor()` → `ValidationError::InvalidConfiguration`

**Example Created:**
- `carla/examples/error_handling_patterns.rs` - Demonstrates all error patterns

### Benefits for Examples

**Connection Recovery:**
```rust
match world.wait_for_tick() {
    Err(e) if e.is_connection_error() => {
        // Restart CARLA connection
        reconnect_to_server();
    }
    Err(e) if e.is_retriable() => {
        // Retry transient failures
        retry_with_backoff();
    }
    Ok(snapshot) => { /* process */ }
}
```

**Resource Fallback:**
```rust
let blueprint = match world.actor_builder("vehicle.tesla.model3") {
    Err(e) if e.is_not_found() => {
        // Try fallback blueprint
        world.actor_builder("vehicle.audi.a2")?
    }
    result => result?
};
```

**Better Error Messages:**
```
Before: "Error: unable to find blueprint 'vehicle.nonexistent.model'"
After:  "Resource not found: Blueprint 'vehicle.nonexistent.model'
         Available blueprints:
         - vehicle.tesla.model3
         - vehicle.audi.a2
         - vehicle.bmw.grandtourer
         ..."
```

### Test Results

✅ All 15 unit tests passing (8 core + 7 FFI)
✅ Build succeeds with zero warnings
✅ Example compiles and demonstrates all patterns

---

## Overview

### GUI Framework: Macroquad

**Decision Date:** 2025-10-29
**Status:** ✅ Selected and proven in Phase 12

All GUI examples use **Macroquad v0.4** as the official pygame replacement:
- Simple, pygame-like API
- Cross-platform (Windows, macOS, Linux, WASM)
- Built-in features: textures, text, keyboard/mouse, audio
- Zero configuration required

**Example Code Pattern:**
```rust
use macroquad::prelude::*;

#[macroquad::main("CARLA Example")]
async fn main() {
    // Setup
    let client = carla::Client::default();
    let mut world = client.world();

    // Main loop
    loop {
        clear_background(BLACK);

        // Render camera/HUD
        draw_texture(&texture, 0., 0., WHITE);
        draw_text("Speed: 45 km/h", 20., 40., 30., WHITE);

        // Handle input
        if is_key_pressed(KeyCode::Escape) { break; }

        next_frame().await
    }
}
```

### Code Organization Strategy

**Decision Date:** 2025-11-01 (see `docs/gui-code-sharing-analysis.md`)

**Current Approach:**
- Each example maintains its own module structure
- Copy and adapt code from `manual_control` as needed
- Extract common modules only after 3+ examples show clear duplication
- Prioritize example clarity over DRY principle

**Future Considerations:**
- Phase 2: Extract `carla/examples/common/` module after Phase 13
- Phase 3: Consider separate `carla-gui` crate only if externally demanded

---

## Phase 12: Manual Control - Complete Interactive Example

**Priority:** HIGH (Flagship example)
**Status:** ✅ COMPLETE (37/37 work items, 100% feature parity)
**Completion Date:** 2025-11-01
**Python Equivalent:** `manual_control.py` (1367 lines)

### Summary

The most comprehensive CARLA example - a fully interactive vehicle simulator with:
- 40+ keyboard controls (WASD, arrows, lights, transmission, autopilot, etc.)
- 14 camera/sensor types with live switching
- 5 camera positions
- Real-time HUD with telemetry, graphs, and nearby vehicles
- Weather cycling (14 presets)
- Map layer management
- Recording/replay system
- Radar visualization with velocity-based coloring
- Camera frame recording to PNG

**Implementation:** `carla/examples/manual_control/`
- 3,500+ lines of well-organized Rust code
- Modular structure: `camera.rs`, `hud.rs`, `keyboard.rs`, `world.rs`, `ui/`, `sensors/`
- Thread-safe sensor callbacks using Arc/Mutex
- Comprehensive error handling and documentation

### Key Features Completed

✅ All 37 work items complete (12 subphases)
✅ Vehicle control (manual + autopilot)
✅ 14 sensor types (RGB, depth, semantic, LiDAR, DVS, optical flow, etc.)
✅ Real-time HUD with FPS, speed, compass, IMU, GNSS
✅ Collision detection with history graph
✅ Manual transmission with gear shifting
✅ Light controls (position, low/high beam, fog, interior, blinkers)
✅ Weather and map layer control
✅ Session recording/replay (Ctrl+R, Ctrl+P)
✅ Camera frame recording to `_out/` (R key) - **COMPLETED 2025-11-01**
✅ Radar visualization with debug points (G key) - **COMPLETED 2025-11-01**
✅ Help overlay (H key) with all controls
✅ Ackermann steering mode

**Reference:** See Phase 12 section below for historical development details.

---

## Phase 13: Core GUI Examples

**Priority:** HIGH
**Estimated Effort:** 3-4 weeks total
**Status:** ✅ COMPLETE (2025-11-05)
**Dependencies:** Phase 12 ✅ Complete

These examples demonstrate essential CARLA workflows with GUI:
- Automatic navigation with AI agents ✅
- Synchronous simulation mode ✅
- Dynamic environment control ✅

### Phase 13.1: Automatic Control GUI ✅

**Priority:** HIGH
**Estimated Effort:** 1 week (after Agent Phase completion)
**Python Equivalent:** `automatic_control.py` (872 lines)
**Target:** `carla/examples/automatic_control_gui.rs` (435 lines)
**Status:** ✅ COMPLETE (2025-11-05)
**Completion:** See `tmp/phase-13.1-completion.md`

Demonstrates AI-driven navigation using CARLA's navigation agents with real-time HUD display.

#### Implementation Summary

**Agent Dependencies:** ✅ All Required (BasicAgent, LocalPlanner, GlobalRoutePlanner)

This example demonstrates the completed CARLA agent system in Rust. See **[Navigation Agents Implementation](agents.md)** for agent API details.

#### Features

- BasicAgent with traffic rules and obstacle avoidance
- Real-time HUD showing agent status and telemetry
- Keyboard controls for manual override
- Camera feed with destination marker visualization
- Optional: BehaviorAgent with multiple behavior profiles
- Optional: ConstantVelocityAgent for testing

#### Work Items

- [ ] **13.1.1: Verify Agent APIs Available**
  - Verify [Agent Phase](agents.md) implementation is complete
  - Test BasicAgent, LocalPlanner, GlobalRoutePlanner APIs
  - Run verification example: `basic_agent_minimal.rs`
  - Confirm agent can navigate autonomously
  - Test: basic_agent_minimal example reaches destination

- [ ] **13.1.2: Basic Window and Vehicle Spawning**
  - Copy and adapt `World` struct from manual_control
  - Implement vehicle spawning with destination selection
  - Display black window with basic text
  - Test: Vehicle spawns, destination set, window displays

- [ ] **13.1.3: Camera Integration**
  - Copy `CameraManager` from manual_control
  - Simplify to RGB camera only (no sensor switching)
  - Single camera position (rear chase view)
  - Test: Camera feed displays at 60 FPS

- [ ] **13.1.4: Agent Integration**
  - Instantiate BasicAgent with vehicle
  - Set destination using random spawn point
  - Integrate agent into main loop
  - Test: Agent initialized correctly

- [ ] **13.1.5: Agent Control Loop**
  - Implement agent.run_step() in main loop
  - Apply agent VehicleControl to player
  - Handle destination reached event
  - Automatic respawn with new destination
  - Test: Agent drives to destination autonomously

- [ ] **13.1.6: HUD Implementation**
  - Copy and simplify `Hud` from manual_control
  - Display: FPS, speed, location, heading, agent status
  - Show destination distance
  - Show agent state (navigating, stopped, destination reached)
  - Remove: manual control bars, gear display
  - Test: HUD updates in real-time

- [ ] **13.1.7: Sensor Integration**
  - Copy CollisionSensor, GnssSensor from manual_control
  - Display collision history graph
  - Display GNSS coordinates
  - Test: Sensors update correctly

- [ ] **13.1.8: Keyboard Controls**
  - Implement simplified keyboard handling
  - P key: Toggle agent on/off (manual override)
  - R key: Respawn at new location with new destination
  - F1: Toggle HUD
  - H: Toggle help
  - ESC/Q: Quit
  - Test: All keys work correctly

- [ ] **13.1.9: Destination Visualization**
  - Draw debug arrow pointing to destination
  - Update destination marker as vehicle moves
  - Clear visualization when destination reached
  - Test: Arrow visible and points correctly

- [ ] **13.1.10: Help System**
  - Copy and adapt `HelpText` from manual_control
  - List all agent controls
  - Describe agent behaviors
  - Test: Help overlay displays all controls

- [ ] **13.1.11: Weather and Environment (Optional)**
  - Copy weather cycling from manual_control
  - C/Shift+C: Cycle weather
  - Test: Weather changes apply

**Estimated Code Reuse from manual_control:**
- CameraManager: 60% (remove sensor switching, recording)
- Hud: 40% (remove manual control displays, add agent status)
- CollisionSensor: 95%
- GnssSensor: 95%
- World: 50% (remove map layers, recording features)
- ui/help_text.rs: 70% (different controls)
- ui/fading_text.rs: 95%

**Success Criteria:**
- [ ] Agent navigates autonomously to destination
- [ ] HUD shows agent status and telemetry
- [ ] Manual override works (P key disables agent)
- [ ] Collision detection works
- [ ] Help overlay shows all controls
- [ ] Maintains 60 FPS
- [ ] Destination reached detection works
- [ ] Auto-respawn with new destination works

**Dependencies:**
- ✅ Phase 12 (Manual Control) - Code reuse source
- ⚠️ **Agent Phase (A.1-A.4)** - **CRITICAL BLOCKER**
- See: [Navigation Agents Implementation](agents.md)

---

### Phase 13.2: Synchronous Mode GUI ✅

**Priority:** MEDIUM
**Estimated Effort:** 3-5 days
**Python Equivalent:** `synchronous_mode.py` (195 lines)
**Target:** `carla/examples/synchronous_mode_gui.rs` (525 lines)
**Status:** ✅ COMPLETE (2025-11-05)
**Completion:** See `tmp/phase-13.2-13.3-completion.md`

Demonstrates frame-perfect synchronous simulation with visual feedback.

#### Features

- Synchronous mode context manager
- Fixed delta time ticks (30 FPS simulation)
- Dual camera display (RGB + semantic segmentation blended)
- Frame counter and timing statistics
- Simple waypoint following

#### Implementation Summary

**Success Criteria:** ✅ All Met
- ✅ Synchronous mode with EpisodeSettings (fixed 30 FPS)
- ✅ Dual camera system (RGB + semantic segmentation)
- ✅ 70/30 blend ratio for visualization
- ✅ Frame counter and FPS comparison (real vs simulated)
- ✅ Waypoint-based teleport navigation
- ✅ Pause/resume, reset controls
- ✅ Help overlay system
- ✅ Clean async mode restoration on exit
- ✅ 525 lines (well-documented)

---

### Phase 13.3: Dynamic Weather GUI ✅

**Priority:** LOW
**Estimated Effort:** 2-3 days
**Python Equivalent:** `dynamic_weather.py` (142 lines)
**Target:** `carla/examples/dynamic_weather_gui.rs` (732 lines)
**Status:** ✅ COMPLETE (2025-11-05)
**Completion:** See `tmp/phase-13.2-13.3-completion.md`

Demonstrates smooth weather transitions with visual feedback.

#### Features

- Smooth weather parameter interpolation
- 8-hour day/night cycle simulation
- Real-time weather parameter display
- Preset weather switching
- Time of day control

#### Implementation Summary

**Success Criteria:** ✅ All Met
- ✅ Weather interpolation system with 2-second smooth transitions
- ✅ 14 weather presets (Clear/Cloudy/Wet/Rain at Noon/Sunset)
- ✅ Weather preset cycling (C / Shift+C)
- ✅ 8-hour day/night cycle with sun position calculation
- ✅ Time controls: pause/resume, speed adjustment (0.1x-10x)
- ✅ Jump to time of day (1-9 keys)
- ✅ Comprehensive HUD showing all 14 weather parameters
- ✅ Help overlay with control documentation
- ✅ Maintains 60 FPS (macroquad)
- ✅ 732 lines (well-documented)

---

## Phase 14: Advanced GUI Examples

**Priority:** MEDIUM
**Estimated Effort:** 4-5 weeks total
**Status:** Ready to Start
**Dependencies:** Phase 13 ✅ COMPLETE

These examples demonstrate advanced visualization and computer vision techniques.

### Phase 14.1: Bounding Boxes Visualization

**Priority:** HIGH
**Estimated Effort:** 1-1.5 weeks
**Python Equivalent:** `bounding_boxes.py` (292 lines), `client_bounding_boxes.py` (215 lines)
**Target:** `carla/examples/bounding_boxes.rs`, `carla/examples/client_bounding_boxes.rs`

Demonstrates 2D/3D bounding box computation and visualization for computer vision.

#### Features

- 2D bounding boxes projected on camera image
- 3D bounding boxes in world space
- Server-side vs client-side computation
- Different colors per actor type (vehicles, pedestrians, traffic signs)
- Distance filtering and culling

#### Work Items

**Part A: Server-Side Bounding Boxes (`bounding_boxes.rs`)**

- [ ] **14.1.1: Camera Projection Matrix**
  - Implement camera intrinsic matrix K calculation
  - FOV to focal length conversion
  - Principal point calculation
  - Test: Matrix values match CARLA camera specs

- [ ] **14.1.2: 3D to 2D Projection**
  - World coordinates to camera coordinates transformation
  - Camera coordinates to image coordinates projection
  - Handle points behind camera (negative Z)
  - Test: Known 3D point projects to correct 2D pixel

- [ ] **14.1.3: Actor Enumeration**
  - Get all vehicles, walkers, traffic signs from world
  - Filter by distance from camera (0-50m)
  - Filter by in-camera-view check
  - Test: Correct actors enumerated

- [ ] **14.1.4: Bounding Box Computation**
  - Get actor bounding box from CARLA
  - Transform 8 corner points to world coordinates
  - Project 8 corners to 2D image coordinates
  - Find min/max to create 2D bounding rectangle
  - Test: Bounding boxes surround actors correctly

- [ ] **14.1.5: Bounding Box Rendering**
  - Draw 2D rectangles on camera image
  - Color by actor type: vehicles (blue), walkers (red), signs (yellow)
  - Draw actor distance label
  - Test: Boxes render correctly over camera feed

- [ ] **14.1.6: 3D Bounding Box Visualization**
  - Draw 12 edges of 3D bounding box using debug lines
  - Use world.debug.draw_line()
  - Color by actor type
  - Test: 3D boxes visible in CARLA spectator view

- [ ] **14.1.7: Keyboard Controls**
  - Space: Toggle 2D boxes on/off
  - B: Toggle 3D boxes on/off
  - +/-: Adjust max distance
  - F: Toggle distance filter
  - Test: All toggles work

**Part B: Client-Side Bounding Boxes (`client_bounding_boxes.rs`)**

- [ ] **14.1.8: Client-Side Computation**
  - Replicate server bounding box logic client-side
  - Compare performance: server vs client
  - Benchmark frame timing
  - Test: Results match server-side computation

- [ ] **14.1.9: Occlusion Handling**
  - Implement depth buffer checking (optional)
  - Draw only visible bounding boxes
  - Handle partial occlusion
  - Test: Occluded boxes not drawn

- [ ] **14.1.10: Performance Optimization**
  - Spatial partitioning for actor filtering
  - Frustum culling before projection
  - Batch rendering of boxes
  - Test: Maintains 60 FPS with 100+ actors

**Success Criteria:**
- [ ] 2D boxes correctly surround all visible actors
- [ ] 3D boxes visible in spectator view
- [ ] Color coding by actor type works
- [ ] Distance filtering works correctly
- [ ] Client-side matches server-side results
- [ ] Maintains 60 FPS with 50+ actors
- [ ] Code demonstrates projection math clearly

---

### Phase 14.2: Multiple Sensor Grid View

**Priority:** MEDIUM
**Estimated Effort:** 1 week
**Python Equivalent:** `visualize_multiple_sensors.py` (268 lines)
**Target:** `carla/examples/visualize_multiple_sensors.rs`

Demonstrates multi-viewport rendering with sensor fusion.

#### Features

- 2x3 grid layout (6 viewports)
- 4 RGB cameras (front, rear, left, right)
- 2 LiDAR sensors (standard + semantic)
- Bird's-eye view LiDAR visualization
- Synchronized sensor updates

#### Work Items

- [ ] **14.2.1: Grid Layout System**
  - Calculate viewport positions for 2x3 grid
  - Each viewport: 1280/3 x 720/2 pixels
  - Implement layout manager
  - Test: Grid divides screen evenly

- [ ] **14.2.2: Multi-Camera Setup**
  - Spawn 4 RGB cameras (front/rear/left/right)
  - Position cameras around vehicle
  - All cameras at 1280/3 x 720/2 resolution
  - Test: All 4 cameras produce images

- [ ] **14.2.3: LiDAR Setup**
  - Spawn regular LiDAR sensor
  - Spawn semantic LiDAR sensor
  - Configure point cloud density
  - Test: LiDAR data arrives correctly

- [ ] **14.2.4: LiDAR Bird's-Eye View**
  - Convert 3D point cloud to 2D top-down view
  - Map Z-height to color gradient
  - Scale and center on vehicle
  - Test: LiDAR visualization shows surroundings

- [ ] **14.2.5: Semantic LiDAR Coloring**
  - Color points by semantic tag
  - Road (gray), vehicles (blue), pedestrians (red)
  - Buildings (brown), vegetation (green)
  - Test: Semantic colors display correctly

- [ ] **14.2.6: Synchronized Rendering**
  - Render all 6 viewports each frame
  - Handle different data rates gracefully
  - Maintain aspect ratios
  - Test: All viewports update smoothly

- [ ] **14.2.7: Viewport Labels**
  - Draw labels: "Front", "Rear", "Left", "Right", "LiDAR", "Semantic LiDAR"
  - Show FPS per viewport
  - Test: Labels visible and correct

- [ ] **14.2.8: Keyboard Controls**
  - Number keys 1-6: Focus single viewport (fullscreen)
  - 0: Return to grid view
  - ESC: Quit
  - Test: Viewport switching works

**Success Criteria:**
- [ ] All 6 viewports render simultaneously
- [ ] LiDAR bird's-eye view shows 3D environment
- [ ] Semantic LiDAR colors make sense
- [ ] Can zoom individual viewport to fullscreen
- [ ] Maintains 30+ FPS with all sensors active
- [ ] Grid layout is clear and organized

---

### Phase 14.3: No Rendering Mode (Performance Demo)

**Priority:** MEDIUM
**Estimated Effort:** 3-4 days
**Python Equivalent:** `no_rendering_mode.py` (143 lines)
**Target:** `carla/examples/no_rendering_mode.rs`

Demonstrates large-scale simulation with rendering disabled for performance testing.

#### Features

- CARLA rendering disabled (no_rendering_mode)
- 100+ vehicles spawned
- Statistics overlay (vehicle count, FPS, memory)
- Minimal GUI (stats only, no camera)
- Performance monitoring

#### Work Items

- [ ] **14.3.1: No Rendering Mode Setup**
  - Enable no_rendering_mode in WorldSettings
  - Verify CARLA server performance
  - Test: CARLA runs without rendering overhead

- [ ] **14.3.2: Large-Scale Vehicle Spawning**
  - Spawn 100-150 vehicles at random locations
  - Enable autopilot for all vehicles
  - Handle spawn failures gracefully
  - Test: All vehicles spawn and drive

- [ ] **14.3.3: Statistics Collection**
  - Count active vehicles
  - Measure simulation FPS
  - Track physics step time
  - Monitor memory usage
  - Test: Stats update every frame

- [ ] **14.3.4: Minimal GUI**
  - Small window (400x300 pixels)
  - Display statistics in table format
  - No camera feed (rendering disabled)
  - Update at 10 Hz (not every frame)
  - Test: GUI displays stats clearly

- [ ] **14.3.5: Performance Monitoring**
  - Graph FPS over time
  - Graph vehicle count over time
  - Show min/max/avg FPS
  - Test: Graphs update correctly

- [ ] **14.3.6: Scalability Test**
  - Button: Spawn +10 vehicles
  - Button: Destroy 10 vehicles
  - Measure FPS impact
  - Test: Performance scales as expected

- [ ] **14.3.7: Keyboard Controls**
  - +: Spawn 10 more vehicles
  - -: Destroy 10 vehicles
  - R: Reset (destroy all, respawn 100)
  - ESC: Quit
  - Test: All controls work

**Success Criteria:**
- [ ] Can spawn 100+ vehicles
- [ ] CARLA rendering is disabled
- [ ] Statistics display correctly
- [ ] Demonstrates performance headroom
- [ ] Shows simulation FPS >> rendering FPS
- [ ] Useful for performance benchmarking

---

### Phase 14.4: Walker Skeleton Visualization

**Priority:** LOW
**Estimated Effort:** 4-5 days
**Python Equivalent:** `draw_skeleton.py` (198 lines)
**Target:** `carla/examples/draw_skeleton.rs`

Demonstrates walker bone structure and skeletal animation.

#### Features

- Real-time bone transform visualization
- Skeleton overlay on camera view
- Bone connections (parent-child hierarchy)
- Multiple walkers with different animations
- 3D skeleton in world space

#### Work Items

- [ ] **14.4.1: Bone Transform API**
  - Implement get_bone_transforms() bindings
  - Parse bone name and transform data
  - Build bone hierarchy tree
  - Test: Bone data retrieved correctly

- [ ] **14.4.2: Walker Spawning**
  - Spawn 5-10 walkers at random locations
  - Start different walk animations
  - Enable AI navigation
  - Test: Walkers animate correctly

- [ ] **14.4.3: Bone Projection**
  - Get bone world positions
  - Project to 2D camera coordinates
  - Handle off-screen bones
  - Test: Bones project correctly

- [ ] **14.4.4: Skeleton Rendering (2D)**
  - Draw lines between connected bones
  - Draw circles at joint positions
  - Color by walker ID
  - Overlay on camera feed
  - Test: Skeleton overlays walker correctly

- [ ] **14.4.5: Skeleton Rendering (3D)**
  - Use debug.draw_line() for 3D bones
  - Draw in CARLA world space
  - Update every frame
  - Test: 3D skeleton visible in spectator

- [ ] **14.4.6: Bone Name Labels**
  - Display bone names near joints
  - Only for selected walker
  - Font size adapts to distance
  - Test: Labels readable and positioned correctly

- [ ] **14.4.7: Walker Selection**
  - Click walker to select
  - Show detailed skeleton for selected walker
  - Tab: Cycle through walkers
  - Test: Selection works correctly

- [ ] **14.4.8: Keyboard Controls**
  - S: Toggle skeleton display
  - B: Toggle bone labels
  - Tab: Next walker
  - 2D/3D: Toggle render mode
  - Test: All toggles work

**Success Criteria:**
- [ ] Skeleton overlays match walker pose
- [ ] Bone connections accurate
- [ ] Animations smooth (60 FPS)
- [ ] 3D skeleton visible in world
- [ ] Can select and inspect individual walkers
- [ ] Useful for animation debugging

---

## Phase 15: Specialized GUI Examples (Optional)

**Priority:** LOW
**Estimated Effort:** 2-3 weeks total
**Status:** Future Work
**Dependencies:** Phase 14 recommended

These examples require additional hardware or specialized libraries.

### Phase 15.1: Steering Wheel Support

**Priority:** LOW
**Estimated Effort:** 1-1.5 weeks
**Python Equivalent:** `manual_control_steeringwheel.py` (997 lines)
**Target:** `carla/examples/manual_control_steeringwheel.rs`

Demonstrates hardware input from racing wheels and pedals.

#### Features

- Logitech G29/G920 support (via gilrs crate)
- Analog steering input
- Analog throttle/brake pedals
- Force feedback (if supported)
- Button mapping for lights, gears, etc.
- Visual input display on HUD

#### Work Items

- [ ] **15.1.1: Gamepad Library Integration**
  - Add `gilrs = "0.10"` dependency
  - Initialize gamepad context
  - Enumerate connected devices
  - Test: Steering wheel detected

- [ ] **15.1.2: Analog Input Mapping**
  - Map steering axis (-1.0 to 1.0)
  - Map throttle axis (0.0 to 1.0)
  - Map brake axis (0.0 to 1.0)
  - Apply dead zones and curves
  - Test: Smooth analog control

- [ ] **15.1.3: Button Mapping**
  - Map gear shift buttons
  - Map light controls
  - Map camera switching
  - Map handbrake button
  - Test: All buttons work

- [ ] **15.1.4: Force Feedback**
  - Detect FF-capable devices
  - Apply resistance based on speed
  - Road surface vibration effects
  - Collision feedback
  - Test: FF effects work (if hardware supports)

- [ ] **15.1.5: HUD Integration**
  - Copy HUD from manual_control
  - Add input visualizer (steering angle, pedal positions)
  - Show connected device name
  - Test: HUD displays hardware state

- [ ] **15.1.6: Fallback to Keyboard**
  - Detect no gamepad scenario
  - Fall back to WASD controls
  - Show warning message
  - Test: Works without wheel connected

**Success Criteria:**
- [ ] Steering wheel provides smooth control
- [ ] Analog inputs mapped correctly
- [ ] Force feedback works (if hardware supports)
- [ ] Can play with wheel exactly like manual_control
- [ ] Graceful fallback if no wheel detected

---

### Phase 15.2: LiDAR 3D Visualization (Optional)

**Priority:** VERY LOW
**Estimated Effort:** 1-2 weeks
**Python Equivalent:** `open3d_lidar.py` (157 lines)
**Target:** `carla/examples/open3d_lidar.rs`

Demonstrates 3D point cloud visualization with rotation/zoom.

**Note:** Requires 3D graphics library (not macroquad)

#### Options

**Option A: Use three-d crate**
- Full 3D rendering pipeline
- Point cloud rendering
- Camera controls (orbit, zoom)
- Estimated effort: 1.5 weeks

**Option B: Use bevy engine**
- Game engine with ECS
- Built-in 3D rendering
- More complex but feature-rich
- Estimated effort: 2 weeks

**Option C: Defer to external tools**
- Save point clouds to PLY/PCD format
- Use external viewers (CloudCompare, MeshLab)
- Document export workflow
- Estimated effort: 2-3 days

#### Recommendation

**Defer to Phase 16** or provide PLY export utility instead.
- True 3D visualization requires different rendering approach
- Macroquad is 2D-focused
- Most users will use external 3D viewers
- Export utility is more practical

---

## Phase 12: Manual Control - Historical Development

**Status:** ✅ COMPLETE (2025-11-01)
**Work Items:** 37/37 (100%)

### Overview

The most comprehensive CARLA example - implemented incrementally over 12 subphases. This section documents the development history and implementation strategy.

**Complexity Level:** VERY HIGH
- 1367 lines of Python code
- 13 distinct component classes
- 14 different sensor types
- 40+ key bindings
- Real-time HUD with graphs and telemetry

### Implementation History

#### Subphase 12.1: Foundation and Basic Window ✅

- [x] **12.1.1: Project Setup and Dependencies**
- [x] **12.1.2: Vehicle Spawning and Basic Display**

#### Subphase 12.2: Camera and Basic Rendering ✅

- [x] **12.2.1: RGB Camera Integration**
- [x] **12.2.2: Camera Position Switching** (5 positions)

#### Subphase 12.3: Basic Vehicle Control ✅

- [x] **12.3.1: Keyboard Input System** (WASD/arrows)
- [x] **12.3.2: Autopilot Toggle** (P key)

#### Subphase 12.4: HUD - Basic Telemetry ✅

- [x] **12.4.1: HUD Struct and FPS Display**
- [x] **12.4.2: Vehicle Telemetry** (speed, location, names)

#### Subphase 12.5: Advanced Sensors ✅

- [x] **12.5.1: IMUSensor Implementation** (compass, accel, gyro)
- [x] **12.5.2: GnssSensor Implementation** (GPS coordinates)
- [x] **12.5.3: CollisionSensor Implementation** (history graph)
- [x] **12.5.4: LaneInvasionSensor Implementation** (lane markings)

#### Subphase 12.6: HUD - Advanced Display ✅

- [x] **12.6.1: Control State Display** (throttle/brake/steer bars)
- [x] **12.6.2: Nearby Vehicles List** (sorted by distance)

#### Subphase 12.7: Advanced Controls ✅

- [x] **12.7.1: Manual Transmission** (M, comma/period keys)
- [x] **12.7.2: Light Controls** (L, Shift+L, I, Z, X keys)
- [x] **12.7.3: Ackermann Controller** (F key, experimental)

#### Subphase 12.8: Multiple Camera Types ✅

- [x] **12.8.1: Camera Type Switching** (14 sensor types, backtick/N/1-9 keys)
- [x] **12.8.2: LiDAR Visualization** (bird's-eye view)

#### Subphase 12.9: Weather and World Control ✅

- [x] **12.9.1: Weather Cycling** (14 presets, C/Shift+C keys)
- [x] **12.9.2: Map Layer Control** (B/Shift+B keys, 11 layers)

#### Subphase 12.10: Recording and Replay ✅

- [x] **12.10.1: Recording System** (Ctrl+R)
- [x] **12.10.2: Replay System** (Ctrl+P, time adjustment)
- [x] **12.10.3: Camera Recording** (R key, PNG to _out/) - **COMPLETED 2025-11-01**

#### Subphase 12.11: Advanced Features ✅

- [x] **12.11.1: Radar Visualization** (G key, velocity-based coloring) - **COMPLETED 2025-11-01**
- [x] **12.11.2: Vehicle Door Control** (O key)
- [x] **12.11.3: Constant Velocity Mode** (Ctrl+W)
- [x] **12.11.4: Vehicle Telemetry Overlay** (T key, CARLA built-in)

#### Subphase 12.12: Help System and Notifications ✅

- [x] **12.12.1: FadingText Notification System** (2-second fade)
- [x] **12.12.2: HelpText Overlay** (H/Shift+/, all 40+ controls)
- [x] **12.12.3: Info Toggle** (F1 key)

### Recent Completions (2025-11-01)

**Camera Recording (12.10.3):**
- Thread-safe PNG frame saving to `_out/` directory
- Sequential 8-digit naming (00000000.png, ...)
- Works with all 14 sensor types
- Non-blocking saves in callback thread
- `Arc<AtomicU64>` frame counter
- Implementation: `camera.rs:647-662`, `camera.rs:732-743`

**Radar Visualization (12.11.1):**
- Radar sensor spawning (35°×20° FOV, 1500 pts/sec)
- Spherical to Cartesian coordinate conversion
- Velocity-based color coding (red=approaching, blue=receding)
- Debug point rendering (0.1s lifetime)
- `Arc<Mutex<bool>>` thread-safe toggling
- Implementation: `sensors/radar.rs` (full module)

### Success Criteria (All Met)

- [x] Vehicle spawns and displays camera view
- [x] All 40+ keyboard shortcuts work correctly
- [x] HUD shows all telemetry in real-time
- [x] 14 sensor types render correctly
- [x] 5 camera positions switch smoothly
- [x] Recording/replay system works (session + camera)
- [x] Weather changes apply
- [x] Map layers load/unload
- [x] Help overlay shows all controls
- [x] Radar visualization with debug points

### Key Technical Achievements

**Threading Fix (2025-10-30):**
- Solved macroquad texture creation on wrong thread
- Two-stage pipeline: sensor callback → RGBA buffer → main thread texture
- `Arc<Mutex<Option<(Vec<u8>, u32, u32)>>>` for thread-safe data passing

**Modular Architecture:**
- Clean separation: main, camera, hud, keyboard, world, ui, sensors
- ~3,500 lines well-organized Rust
- Reusable components for future examples

---

## Summary: Phase Progress

| Phase | Example              | Status      | Work Items   | Estimated Effort |
|-------|----------------------|-------------|--------------|------------------|
| 12    | Manual Control       | ✅ COMPLETE | 37/37 (100%) | 4-6 weeks        |
| 13.1  | Automatic Control    | ⏳ Ready    | 0/10         | 1 week           |
| 13.2  | Synchronous Mode GUI | ⏳ Ready    | 0/7          | 3-5 days         |
| 13.3  | Dynamic Weather GUI  | ⏳ Ready    | 0/7          | 2-3 days         |
| 14.1  | Bounding Boxes       | ⏳ Ready    | 0/10         | 1-1.5 weeks      |
| 14.2  | Multiple Sensors     | ⏳ Ready    | 0/8          | 1 week           |
| 14.3  | No Rendering Mode    | ⏳ Ready    | 0/7          | 3-4 days         |
| 14.4  | Walker Skeleton      | ⏳ Ready    | 0/8          | 4-5 days         |
| 15.1  | Steering Wheel       | ⏳ Future   | 0/6          | 1-1.5 weeks      |
| 15.2  | LiDAR 3D Vis         | ⏳ Deferred | 0/0          | See options      |

**Total:** 1/10 examples complete, 9 examples planned

**Next Priority:** Phase 13.1 (Automatic Control) - demonstrates AI navigation agents

---

## Dependencies

### Required Crates

**Core:**
- `macroquad = "0.4"` - 2D rendering and window management
- `eyre = "0.6"` - Error handling
- `tracing = "0.1"` - Logging
- `tracing-subscriber = "0.3"` - Log formatting

**Optional (Phase 15+):**
- `gilrs = "0.10"` - Gamepad/steering wheel input
- `three-d = "0.16"` OR `bevy = "0.12"` - 3D visualization (Phase 15.2)

### CARLA APIs

All examples use existing APIs from Phases 0-11:
- Vehicle control and physics
- Sensor data (cameras, LiDAR, radar, GNSS, IMU)
- World manipulation (weather, map layers)
- Recording and replay
- Actor spawning and management
- Debug visualization

---

## Learning Resources

### Macroquad
- Official guide: https://mq.agical.se/
- API documentation: https://docs.rs/macroquad
- Examples: https://github.com/not-fl3/macroquad/tree/master/examples

### CARLA
- Python examples: `CARLA_*/PythonAPI/examples/`
- Manual control reference: `manual_control.py`
- Sensor documentation: https://carla.readthedocs.io/en/latest/ref_sensors/

### Computer Vision
- Camera projection math
- Bounding box computation
- Coordinate transformations

---

**[← Back to Roadmap Index](../roadmap.md)**
