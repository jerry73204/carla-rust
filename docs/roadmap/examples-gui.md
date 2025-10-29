# GUI Example Implementations (Phases 12-14)

**[← Back to Roadmap Index](../roadmap.md)**

This document covers Phases 12-14: GUI Example Implementations with Macroquad. These examples demonstrate interactive CARLA applications with real-time visualization, keyboard controls, and comprehensive feature showcases.

## Contents

- [Phase 12: Manual Control - Complete Interactive Example](#phase-12-manual-control---complete-interactive-example)
- [Phase 13: Advanced Example Implementations (GUI with Macroquad)](#phase-13-advanced-example-implementations-gui-with-macroquad)
- [Phase 14: Specialized Examples (Reference/Future)](#phase-14-specialized-examples-referencefuture)

---

## Phase 12: Manual Control - Complete Interactive Example

**Priority:** HIGH (Flagship example, comprehensive feature showcase)
**Estimated Effort:** 4-6 weeks
**Status:** Not Started
**Python Equivalent:** `manual_control.py` (1367 lines, 13 major components)

### Overview

Implement the most comprehensive CARLA example - a fully interactive vehicle simulator with real-time controls, multiple sensors, HUD display, and advanced features. This is the flagship example that demonstrates the full capabilities of the carla-rust library.

**Complexity Level:** VERY HIGH
- 1367 lines of Python code
- 13 distinct component classes
- 14 different sensor types
- Complex keyboard input system (40+ key bindings)
- Real-time HUD with graphs and telemetry
- Recording/replay system integration
- Weather and map layer manipulation
- Multiple camera positions and types

**Why a Dedicated Phase?**

Manual control is significantly more complex than other Phase 13 examples:
- **Scope:** 3-4x larger than typical examples
- **Integration:** Touches nearly every CARLA API
- **UI Complexity:** Multi-layer rendering (camera, HUD, help text, notifications)
- **State Management:** Vehicle, sensors, recording, weather, map layers
- **Input Handling:** Both event-based (key press) and continuous (key down) input

This dedicated phase allows for incremental implementation with clear milestones.

### Component Inventory (from Python)

The Python `manual_control.py` consists of 13 major components:

1. **World Class** (lines 176-351)
   - Vehicle spawning and respawning
   - All sensor management (collision, lane invasion, GNSS, IMU, radar, camera)
   - Weather cycling with presets
   - Map layer management
   - Recording state management

2. **KeyboardControl Class** (lines 358-642)
   - Event-based input (key press/release)
   - Continuous input (key held down)
   - Vehicle control (throttle, brake, steer, gear, handbrake)
   - Walker control (speed, direction, jump)
   - Ackermann steering mode
   - Light state management (position, low beam, high beam, fog, interior, blinkers)

3. **HUD Class** (lines 649-797)
   - Real-time telemetry display (FPS, speed, location, compass)
   - IMU data (accelerometer, gyroscope)
   - Vehicle control state (throttle, steer, brake, gear)
   - Collision history graph (200 frame rolling window)
   - Nearby vehicles list (sorted by distance)
   - Notification system integration

4. **FadingText Class** (lines 804-826)
   - Temporary notification display
   - Time-based fade animation

5. **HelpText Class** (lines 833-856)
   - Full help overlay with all controls
   - Toggle on/off with H key

6. **CollisionSensor Class** (lines 863-895)
   - Collision detection and history
   - Intensity tracking (4000 frame buffer)

7. **LaneInvasionSensor Class** (lines 902-926)
   - Lane marking crossing detection
   - Lane type identification

8. **GnssSensor Class** (lines 933-954)
   - GPS coordinates (latitude, longitude)

9. **IMUSensor Class** (lines 961-993)
   - Accelerometer, gyroscope, compass
   - Value clamping (±99.9)

10. **RadarSensor Class** (lines 1000-1061)
    - Object detection with debug visualization
    - Velocity-based color coding

11. **CameraManager Class** (lines 1067-1216)
    - 14 different sensor types
    - 5 camera positions for vehicles, 5 for walkers
    - Recording to disk capability
    - Sensor data → pygame surface conversion

12. **game_loop() Function** (lines 1223-1287)
    - Main loop: tick, input, update, render
    - Synchronous mode management
    - Cleanup handling

13. **main() Function** (lines 1294-1367)
    - CLI argument parsing
    - Initialization and teardown

### Work Items (Incremental Implementation)

The implementation is broken down into 12 incremental phases, building from basic to advanced:

#### Subphase 12.1: Foundation and Basic Window

- [ ] **12.1.1: Project Setup and Dependencies**
  - Add macroquad v0.4 dependency
  - Create `carla/examples/manual_control.rs`
  - Set up basic async main loop structure
  - Test: Window opens and responds to ESC key

- [ ] **12.1.2: Vehicle Spawning and Basic Display**
  - Implement World struct (vehicle management only)
  - Spawn vehicle at spawn point 0
  - Display black window with simple "Manual Control" text
  - Enable autopilot by default
  - Test: Vehicle spawns, window displays, autopilot works

#### Subphase 12.2: Camera and Basic Rendering

- [ ] **12.2.1: RGB Camera Integration**
  - Implement basic CameraManager (RGB sensor only)
  - Spawn RGB camera attached to vehicle
  - Render camera feed to macroquad window
  - Test: Live camera view displays at 60 FPS

- [ ] **12.2.2: Camera Position Switching**
  - Implement 5 camera positions (rear chase, hood, side, high, fixed)
  - Add TAB key to cycle camera positions
  - Update CameraManager to respawn sensor at new position
  - Test: TAB cycles through all 5 positions smoothly

#### Subphase 12.3: Basic Vehicle Control

- [ ] **12.3.1: Keyboard Input System**
  - Implement KeyboardControl struct
  - Parse WASD/Arrow keys for throttle, brake, steer
  - Apply VehicleControl to player
  - Test: Vehicle responds to keyboard input

- [ ] **12.3.2: Autopilot Toggle**
  - Add P key to toggle autopilot
  - Display notification when autopilot changes
  - Disable manual control when autopilot active
  - Test: P key switches between manual and autopilot

#### Subphase 12.4: HUD - Basic Telemetry

- [ ] **12.4.1: HUD Struct and FPS Display**
  - Implement HUD struct with server/client FPS tracking
  - Render semi-transparent overlay (220x720 pixels, alpha 100)
  - Display FPS counters at top
  - Test: HUD shows FPS, overlay doesn't block camera view

- [ ] **12.4.2: Vehicle Telemetry**
  - Add speed (km/h) display
  - Add location (x, y) display
  - Add vehicle name display
  - Add map name display
  - Test: Telemetry updates in real-time as vehicle moves

#### Subphase 12.5: Advanced Sensors (IMU, GNSS, Collision)

- [ ] **12.5.1: IMUSensor Implementation**
  - Spawn IMU sensor on vehicle
  - Display compass heading with cardinal directions (N/S/E/W)
  - Display accelerometer (x, y, z) values
  - Display gyroscope (x, y, z) values
  - Test: IMU data updates smoothly, compass shows correct heading

- [ ] **12.5.2: GnssSensor Implementation**
  - Spawn GNSS sensor on vehicle
  - Display GPS coordinates (latitude, longitude) with 6 decimal precision
  - Test: GNSS updates, coordinates match vehicle position

- [ ] **12.5.3: CollisionSensor Implementation**
  - Spawn collision sensor on vehicle
  - Maintain 4000 frame collision history
  - Display collision intensity graph (200 points, rolling window)
  - Show collision notification on impact
  - Test: Graph updates on collision, notification appears

- [ ] **12.5.4: LaneInvasionSensor Implementation**
  - Spawn lane invasion sensor on vehicle
  - Display notification when crossing lane markings
  - Show lane marking type (Solid, Broken, etc.)
  - Test: Notification appears when driving over lane lines

#### Subphase 12.6: HUD - Advanced Display

- [ ] **12.6.1: Control State Display**
  - Show throttle, steer, brake as horizontal bars (0.0-1.0)
  - Show reverse, hand brake, manual gear shift as checkboxes
  - Show current gear (R/N/1/2/3/...)
  - Test: Bars and checkboxes update as vehicle is controlled

- [ ] **12.6.2: Nearby Vehicles List**
  - Query all vehicles in world
  - Calculate distances from player
  - Sort by distance (closest first)
  - Display top N vehicles within 200m with type and distance
  - Test: List updates as vehicles approach/leave

#### Subphase 12.7: Advanced Controls

- [ ] **12.7.1: Manual Transmission**
  - Add M key to toggle manual gear shifting
  - Add comma/period keys to shift up/down
  - Display "Manual Transmission" notification
  - Update HUD to show manual mode
  - Test: Gear shifts manually, notification appears

- [ ] **12.7.2: Light Controls**
  - Implement light state management (bitflags)
  - L key: cycle through position → low beam → fog → off
  - Shift+L: toggle high beam
  - Ctrl+L: toggle special lights
  - I key: toggle interior
  - Z key: left blinker, X key: right blinker
  - Automatically set brake lights and reverse lights
  - Test: All light keys work, auto lights engage correctly

- [ ] **12.7.3: Ackermann Controller (Optional)**
  - Add F key to toggle Ackermann controller
  - Implement Ackermann speed control (km/h target)
  - Display Ackermann info in HUD when active
  - Q key: reverse direction
  - Test: Ackermann mode activates, speed control works

#### Subphase 12.8: Multiple Camera Types

- [ ] **12.8.1: Camera Type Switching**
  - Extend CameraManager to support 14 sensor types:
    - RGB, Depth (Raw/Gray/Logarithmic)
    - Semantic Segmentation (Raw/CityScapes)
    - Instance Segmentation (Raw/CityScapes)
    - Cosmos Control, LiDAR, DVS
    - RGB Distorted, Optical Flow, Normals
  - Add backtick (`) and N keys to cycle sensors
  - Add number keys 1-9, Ctrl+1-9 for direct sensor selection
  - Display sensor name notification on switch
  - Test: All 14 sensor types render correctly

- [ ] **12.8.2: LiDAR Visualization**
  - Convert LiDAR point cloud to 2D bird's-eye view
  - Render points as white dots on black background
  - Match Python's top-down projection
  - Test: LiDAR view shows points around vehicle

#### Subphase 12.9: Weather and World Control

- [ ] **12.9.1: Weather Cycling**
  - Define 10+ weather presets (clear, rain, fog, etc.)
  - C key: next weather, Shift+C: previous weather
  - Display weather name notification
  - Test: Weather changes smoothly, notification shows preset name

- [ ] **12.9.2: Map Layer Control**
  - B key: load next map layer
  - Shift+B: unload next map layer
  - Supported layers: All, None, Buildings, Decals, Foliage, Ground, ParkedVehicles, Particles, Props, StreetLights, Walls
  - Display layer name notification
  - Test: Layers load/unload, environment updates visually

#### Subphase 12.10: Recording and Replay

- [ ] **12.10.1: Recording System**
  - Ctrl+R: start/stop recording to "manual_recording.log"
  - Track recording_enabled state
  - Display "Recording ON/OFF" notification
  - Store recording_start time for replay
  - Test: Recording creates file, notification appears

- [ ] **12.10.2: Replay System**
  - Ctrl+P: replay "manual_recording.log"
  - Destroy sensors before replay to fix camera
  - Disable autopilot during replay
  - Ctrl+Minus/Plus: adjust replay start time (±1 second)
  - Shift+Minus/Plus: adjust by ±10 seconds
  - Display replay notification and start time
  - Test: Replay works, start time adjusts, camera positioned correctly

- [ ] **12.10.3: Camera Recording**
  - R key: toggle camera recording (save images to _out/)
  - Display "Recording ON/OFF" notification
  - Save current frame on each render (frame_NNNNNNNN.png format)
  - Test: Images save to disk, numbering is sequential

#### Subphase 12.11: Advanced Features

- [ ] **12.11.1: Radar Visualization**
  - Spawn radar sensor on vehicle
  - G key: toggle radar visualization
  - Draw radar detections as 3D debug points (color by velocity)
  - Test: Radar shows detections, G key toggles display

- [ ] **12.11.2: Vehicle Door Control**
  - O key: toggle all doors open/close
  - Handle exception gracefully (not all vehicles have doors)
  - Display "Opening/Closing Doors" notification
  - Test: Doors open/close on vehicles that support it

- [ ] **12.11.3: Constant Velocity Mode**
  - Ctrl+W: toggle constant velocity mode (60 km/h = 17 m/s)
  - Disable velocity when toggled off
  - Display notification with mode state
  - Test: Vehicle maintains constant speed when enabled

- [ ] **12.11.4: Vehicle Telemetry Overlay**
  - T key: toggle vehicle debug telemetry (CARLA's built-in)
  - Display notification with state
  - Test: Telemetry appears/disappears in CARLA window

#### Subphase 12.12: Help System and Notifications

- [ ] **12.12.1: FadingText Notification System**
  - Implement FadingText struct with alpha fade
  - Position at bottom of screen (width x 40 pixels)
  - 2-second display with fade out
  - Test: Notifications appear, fade smoothly

- [ ] **12.12.2: HelpText Overlay**
  - Implement HelpText struct with full control listing
  - H key or Shift+/: toggle help display
  - Centered overlay (780 pixels wide)
  - Semi-transparent background (alpha 220)
  - List all 40+ key bindings with descriptions
  - Test: Help appears/disappears, all keys listed

- [ ] **12.12.3: Info Toggle**
  - F1 key: toggle HUD info display
  - Hide/show all telemetry (keep notifications and help)
  - Test: F1 hides telemetry, shows black screen with camera only

### Python Reference

**File:** `~/Downloads/carla-simulator/CARLA_0.9.16/PythonAPI/examples/manual_control.py`
**Lines:** 1367
**Key Features:**
- 40+ keyboard shortcuts
- 14 sensor types
- 5 camera positions
- Real-time HUD with graphs
- Recording/replay integration
- Weather and map control
- Light management
- Ackermann steering

### Implementation Strategy

**Recommended Approach:**
1. **Incremental development** - Implement subphases 12.1-12.12 sequentially
2. **Test after each subphase** - Ensure stability before adding complexity
3. **Commit frequently** - Each subphase should be a separate commit
4. **Document controls** - Update help text as features are added
5. **Performance monitoring** - Ensure 60 FPS maintained throughout

**Key Technical Decisions:**
- Use `macroquad::prelude::*` for all rendering
- Store sensors in World struct (avoid complex lifetimes)
- Use `Arc<Mutex<>>` for sensor data shared with callbacks
- Implement custom texture conversion from CARLA BGRA to macroquad RGBA
- Use `is_key_pressed()` for toggle events, `is_key_down()` for continuous control

**Estimated Timeline:**
- Subphases 12.1-12.3: 1 week (foundation)
- Subphases 12.4-12.6: 1 week (HUD and sensors)
- Subphases 12.7-12.8: 1.5 weeks (advanced controls and cameras)
- Subphases 12.9-12.10: 1 week (world manipulation and recording)
- Subphases 12.11-12.12: 1.5 weeks (polish and help system)
- **Total: 6 weeks** (with testing and refinement)

### Success Criteria

- [ ] Vehicle spawns and displays camera view
- [ ] All 40+ keyboard shortcuts work correctly
- [ ] HUD shows all telemetry in real-time
- [ ] 14 sensor types render correctly
- [ ] 5 camera positions switch smoothly
- [ ] Recording/replay system works
- [ ] Weather changes apply
- [ ] Map layers load/unload
- [ ] Help overlay shows all controls
- [ ] Maintains 60 FPS with all features enabled
- [ ] Code is well-documented and maintainable

### Dependencies

**New Rust APIs:** None required - all APIs already implemented

**Macroquad Features Used:**
- `clear_background()`, `draw_texture()`, `draw_text()`, `draw_rectangle()`
- `is_key_pressed()`, `is_key_down()`, `next_frame().await`
- Texture creation from raw pixel data
- Font rendering with custom sizes

**Existing CARLA APIs:**
- All Phase 0-11 APIs (vehicle, sensors, world, recording, etc.)
- Camera projection utilities (from lidar_to_camera example)

---

## Phase 13: Advanced Example Implementations (GUI with Macroquad)

**Priority:** Medium (UI framework selected, ready for implementation)
**Estimated Effort:** 4-5 weeks
**Status:** Ready to Start (UI framework: Macroquad selected 2025-10-29)
**✅ UI Framework:** Macroquad v0.4 - pygame replacement for Rust

### Overview

Implement advanced examples with real-time GUI visualization using Macroquad. These examples demonstrate professional-grade CARLA applications with interactive controls and visual feedback, matching Python's pygame examples.

**Key Features:**
- Real-time sensor visualization (cameras, LiDAR)
- Interactive vehicle control with keyboard/gamepad input
- HUD overlays with telemetry data
- Multi-viewport grid layouts
- 2D bounding box visualization

**Note**: Phase 2 (Debug utilities) is already complete. Ready to begin implementation with Macroquad.

### Prerequisites

- Phase 2: Debug and Visualization Utilities ✅ (Complete)
- All sensor APIs complete
- UI framework integration: **Macroquad** (decision finalized 2025-10-29)
- Math library integration (nalgebra) ✅ (Already in use)
- Image processing utilities ✅ (`image` crate already integrated)

### UI Framework: Macroquad

**Decision Date:** 2025-10-29
**Status:** ✅ Selected as the official pygame replacement for Phase 12 and 13

After comprehensive research and evaluation of Rust alternatives to Python's pygame, **Macroquad** has been selected as the official GUI framework for GUI examples (Phases 12 and 13).

#### Why Macroquad?

**Core Information:**
- **Website:** https://macroquad.rs/
- **Version:** 0.4.14 (Latest: 2025-03-20)
- **GitHub:** 2,500+ stars, actively maintained
- **Philosophy:** Simple and easy to use, heavily inspired by raylib

**Key Advantages:**
- ✅ **Most pygame-like experience** - Simple, intuitive API matching Python pygame's ease of use
- ✅ **Zero configuration** - Works out of the box, no complex setup required
- ✅ **Beginner-friendly** - Avoids complex Rust concepts (lifetimes/borrowing) in the API
- ✅ **Cross-platform** - Windows, macOS, Linux, Web (WASM), iOS, Android
- ✅ **Built-in features** - Texture loading, text rendering, keyboard/mouse input, audio support
- ✅ **Efficient rendering** - Automatic geometry batching for 2D graphics
- ✅ **Active development** - Latest release March 2025, new tutorials published regularly
- ✅ **Excellent resources** - Official guide at mq.agical.se, growing community

**Why Not Alternatives?**
- **Winit + Pixels:** More boilerplate, requires manual event loop management, separate crates for text/audio
- **SDL2:** No WebAssembly support, C dependency, missing modern features in Rust bindings
- **Bevy:** Full game engine - overkill for simple 2D visualization examples

**Example Code:**
```rust
use macroquad::prelude::*;

#[macroquad::main("CARLA Sensor View")]
async fn main() {
    loop {
        clear_background(BLACK);

        // Display camera feed
        draw_texture(&camera_texture, 0., 0., WHITE);

        // HUD overlay
        draw_text("Speed: 45 km/h", 20., 40., 30., WHITE);
        draw_text("Throttle: 0.8", 20., 70., 30., WHITE);

        // Handle input
        if is_key_pressed(KeyCode::Escape) {
            break;
        }

        next_frame().await
    }
}
```

#### Implementation Plan

**Dependency:**
```toml
[dependencies]
macroquad = "0.4"
```

**Strategy:**
1. Add macroquad dependency to Phase 12/13 example `Cargo.toml`
2. Keep Phase 11 headless examples unchanged for CI/testing compatibility
3. Create `*_gui.rs` variants of Phase 11 examples with macroquad GUI
4. All new interactive examples (manual_control, bounding_boxes) use macroquad
5. Use macroquad's async main loop pattern for all GUI examples

**Learning Resources:**
- Official guide: https://mq.agical.se/
- API documentation: https://docs.rs/macroquad
- GitHub examples: https://github.com/not-fl3/macroquad/tree/master/examples
- 2025 tutorial series: "Rust Breakout with Macroquad" (theanswers42.com)

### Work Items

#### Full GUI Versions of Phase 11 Examples

- [ ] **Automatic Control (Full GUI)** (`automatic_control_gui.rs`)
  - **Python Equivalent:** `automatic_control.py` (pygame)
  - **Rust Implementation:** macroquad
  - Real-time window showing camera view
  - HUD overlay with telemetry (speed, throttle, steering, location)
  - Keyboard controls for manual override
  - **APIs Used:**
    - `macroquad` - Window and rendering
    - `draw_texture()` - Display camera feed
    - `draw_text()` - HUD overlay
    - `Vehicle::transform()`, `velocity()`, `control()` - Telemetry
  - Demonstrates: GUI integration, HUD rendering, real-time visualization

- [ ] **Synchronous Mode (Full GUI)** (`synchronous_mode_gui.rs`)
  - **Python Equivalent:** `synchronous_mode.py` (pygame)
  - **Rust Implementation:** macroquad
  - Real-time window showing synchronized camera view
  - HUD overlay showing frame number, FPS, simulation stats
  - Fixed delta time tick visualization
  - **APIs Used:**
    - `macroquad` - Window and rendering
    - `World::tick()` - Frame-by-frame simulation
    - `draw_texture()` - Camera display
    - `get_fps()` - Frame rate monitoring
  - Demonstrates: Synchronous mode with visual feedback, frame timing

- [ ] **Multiple Sensor Visualization (Full GUI)** (`visualize_multiple_sensors_gui.rs`)
  - **Python Equivalent:** `visualize_multiple_sensors.py` (pygame)
  - **Rust Implementation:** macroquad
  - Real-time window with 2×3 grid layout
  - 6 sensor views: 4 RGB cameras (left/front/right/rear) + LiDAR + Semantic LiDAR
  - Real-time bird's-eye LiDAR point rendering
  - ESC/Q to quit
  - **APIs Used:**
    - `macroquad` - Window and grid rendering
    - `draw_texture()` - Multiple camera viewports
    - `draw_circle()` - LiDAR point visualization
    - Grid layout calculations for 2×3 arrangement
  - Demonstrates: Multi-viewport rendering, grid layouts, real-time sensor fusion

#### Interactive and Advanced Examples

- [ ] **Manual Control** - **MOVED TO DEDICATED PHASE 12**
  - **Note:** Due to high complexity (1367 lines, 13 components, 40+ controls), manual_control has been moved to its own dedicated phase
  - **See:** Phase 12 for complete implementation breakdown
  - **Status:** Phase 12 provides 12 incremental implementation phases with 37 work items

- [ ] **Bounding Boxes** (2 examples)
  - **Python Equivalent:** `bounding_boxes.py`, `client_bounding_boxes.py` (pygame)
  - **Rust Implementation:** macroquad
  - `bounding_boxes.rs` - 2D/3D bbox generation and display
  - `client_bounding_boxes.rs` - Client-side bbox computation
  - Real-time visualization with bbox overlays on camera feed
  - Different colors for vehicles, pedestrians, traffic signs
  - **APIs Used:**
    - `macroquad` - Window and drawing primitives
    - `draw_rectangle_lines()` - Bounding box rendering
    - `carla::sensor::camera` - Projection utilities
    - `World::get_actors()` - Actor enumeration for bbox generation
  - Demonstrates: Computer vision, projection math, object detection visualization

- [ ] **Advanced Visualization** (3 examples)
  - `draw_skeleton.rs` - Walker skeleton bone visualization
    - **Python Equivalent:** `draw_skeleton.py` (pygame)
    - **Rust Implementation:** macroquad with `draw_line()` for bones
  - `no_rendering_mode.rs` - Large-scale simulation (100+ vehicles) with stats overlay
    - **Python Equivalent:** `no_rendering_mode.py` (pygame)
    - **Rust Implementation:** macroquad for HUD, CARLA rendering disabled
  - `open3d_lidar.rs` - LiDAR point cloud 3D visualization (OPTIONAL)
    - Requires 3D visualization library (not macroquad)
    - Consider `three-d` or `bevy` for true 3D rendering
  - Demonstrates: Debug drawing, scalability, performance monitoring

- [ ] **Hardware Input** (`manual_control_steeringwheel.rs`)
  - **Python Equivalent:** `manual_control_steeringwheel.py` (pygame + joystick)
  - **Rust Implementation:** macroquad + gilrs
  - Steering wheel and pedal support via `gilrs` crate (Rust gamepad library)
  - Analog input mapping (steering angle, throttle, brake)
  - Force feedback (if device supports it)
  - Real-time macroquad display with input visualization
  - **APIs Used:**
    - `macroquad` - Window and rendering
    - `gilrs` - Gamepad/wheel input
    - `Vehicle::apply_control()` - Analog control input
  - Demonstrates: Hardware integration, analog input handling

### Test Cases

#### Manual Testing Required
- Most examples in this phase require human interaction
- Create test checklist for manual verification
- Document expected behavior and controls

#### Automated Tests (Where Possible)
- `test_projection_matrix` - Verify camera projection math
- `test_bbox_computation` - 2D bbox from 3D coordinates
- `test_instance_segmentation_decode` - Decode actor IDs
- `test_large_scale_spawn` - 100 vehicles spawn successfully

### Dependencies

**External Crates**:
- **winit** (v0.29+) - Window and event handling
- **pixels** (v0.13+) - 2D rendering
- **nalgebra** (v0.32+) - Matrix and vector math
- **image** (v0.24+) - Image encoding/decoding
- **gilrs** (v0.10+) - Game controller/wheel input

**New Utilities**:
```rust
// Camera projection (new module)
pub mod camera_projection {
    pub fn build_projection_matrix(width: u32, height: u32, fov: f32) -> Matrix3<f32>;
    pub fn world_to_camera(point: Location, camera_transform: &Transform) -> Vector3<f32>;
    pub fn project_to_2d(point_3d: Vector3<f32>, k_matrix: &Matrix3<f32>) -> (f32, f32);
}

// Instance segmentation utilities
impl SemanticSegmentationImage {
    pub fn decode_instance_ids(&self) -> (Vec<u8>, Vec<u16>);
}
```

**See**: `docs/example_implementation_plan.md` for UI patterns and design guidelines

---

## Phase 14: Specialized Examples (Reference/Future)

**Priority:** Very Low (Deferred)
**Estimated Effort:** Varies by integration
**Status:** Documentation Only
**⚠️ Note:** No implementation planned - documentation reference only

### Overview

Document external integration examples from the Python repository. Most are deferred as they require external systems, proprietary SDKs, or are better served by separate integration crates.

**Note**: These examples are explicitly deferred. Implementation is not planned unless there is specific user demand.

### Examples (Reference Only)

- **InvertedAI Traffic** (`invertedai_traffic.py`)
  - Status: Deferred - requires InvertedAI API subscription
  - Integration point: Traffic generation via external AI

- **NVIDIA Cosmos Generation** (`carla_cosmos_gen.py`)
  - Status: Deferred - requires NVIDIA Cosmos SDK
  - Integration point: Synthetic data generation

- **V2X Communication** (`V2XDemo.py`, `test_addsecondvx.py`)
  - Status: Deferred - requires V2X protocol implementation
  - Integration point: Vehicle-to-vehicle/infrastructure communication

- **ROS2 Bridge** (`ros2/*`)
  - Status: Future - recommend separate `carla-ros2` crate
  - Integration point: ROS2 message conversion and topics

- **RSS Safety** (`rss/*`)
  - Status: Deferred - requires Intel RSS library bindings
  - Integration point: Safety validation and constraint checking

### Work Items

- [ ] **Integration Documentation**
  - Document FFI requirements for each integration
  - Identify Rust crate alternatives where available
  - Create integration guide for common patterns

- [ ] **External Crate Recommendations**
  - ROS2: `rclrs` (existing ROS2 Rust bindings)
  - V2X: Protocol-specific crates (when available)
  - RSS: Custom FFI bindings (complex undertaking)

### Rationale

These integrations are deferred because they:
1. Require external paid services (InvertedAI)
2. Require proprietary SDKs (NVIDIA Cosmos)
3. Are better served by dedicated integration crates (ROS2)
4. Require extensive additional FFI work (RSS)

**Recommendation**: Document integration patterns but don't implement unless there's specific user demand.

---

