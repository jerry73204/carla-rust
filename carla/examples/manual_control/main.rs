//! CARLA Manual Control - Complete Interactive Example
//!
//! This is the flagship example demonstrating the full capabilities of carla-rust.
//!
//! ## Features
//!
//! - **Full Vehicle Control**: WASD/arrow keys for throttle, brake, steer
//! - **Multiple Sensors**: 14 camera/sensor types, 5 camera positions
//! - **Real-time HUD**: FPS, speed, location, compass, IMU, GNSS, collision graph
//! - **Advanced Controls**: Manual transmission, lights, Ackermann steering
//! - **World Manipulation**: Weather cycling, map layer management
//! - **Recording/Replay**: Full session recording and playback
//! - **Interactive UI**: Help overlay, notifications, telemetry display
//!
//! ## Phase 13 Implementation Status
//!
//! See `docs/roadmap.md` Phase 13 for the complete implementation plan.
//!
//! **Current Status:** Foundation setup complete
//! - ✅ Package structure
//! - ✅ Dependencies (carla, macroquad, eyre, tracing)
//! - ✅ Module organization (13 components)
//! - ⏳ Phase 1: Foundation and basic window (TODO)
//!
//! ## Running
//!
//! ```bash
//! # Ensure CARLA simulator is running
//! cargo run --example manual_control
//!
//! # With custom settings
//! cargo run --example manual_control -- --sync --autopilot
//! ```
//!
//! ## Controls
//!
//! Press **H** in-game to see all controls (Phase 12.2 TODO).

// Module declarations
mod camera;
mod hud;
mod keyboard;
mod sensors;
mod ui;
mod world;

use eyre::Result;
use macroquad::prelude::*;
use tracing::info;

/// Application configuration
pub struct Config {
    pub width: u32,
    pub height: u32,
    pub host: String,
    pub port: u16,
    pub sync: bool,
    pub autopilot: bool,
    pub filter: String,
    pub gamma: f32,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            width: 1280,
            height: 720,
            host: "127.0.0.1".to_string(),
            port: 2000,
            sync: true,
            autopilot: false,
            filter: "vehicle.*".to_string(),
            gamma: 2.2,
        }
    }
}

/// Window configuration function for macroquad
fn window_conf() -> Conf {
    Conf {
        window_title: "CARLA Manual Control".to_string(),
        window_width: 1280,
        window_height: 720,
        window_resizable: false,
        ..Default::default()
    }
}

/// Main entry point
#[macroquad::main(window_conf)]
async fn main() -> Result<()> {
    // Initialize tracing subscriber for logging
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::from_default_env()
                .add_directive(tracing::Level::INFO.into()),
        )
        .init();

    info!("CARLA Manual Control starting...");
    info!("✅ Subphases 12.1-12.6 implemented (Basic functionality complete)");

    // Parse configuration (TODO: Add CLI arguments)
    let config = Config::default();

    // Connect to CARLA server
    info!("Connecting to CARLA at {}:{}", config.host, config.port);
    let mut client = carla::client::Client::connect(&config.host, config.port, None);

    // ✅ Subphase 12.1.2: Create world and spawn vehicle
    let mut world = self::world::World::new(&client, &config)?;
    info!("✓ World initialized, vehicle spawned");

    // ✅ Subphase 12.2.1: Create camera manager and spawn RGB camera
    let mut camera = self::camera::CameraManager::new(config.width, config.height, config.gamma);
    camera.spawn_camera(&mut world)?;
    info!("✓ Camera spawned");

    // ✅ Subphase 12.3.1: Create keyboard controller
    let mut keyboard = self::keyboard::KeyboardControl::new(config.autopilot);

    // ✅ Subphase 12.4: Create HUD
    let mut hud = self::hud::Hud::new(config.width, config.height);

    // ✅ Subphase 12.5: Spawn all sensors
    hud.spawn_sensors(&mut world)?;
    info!("✓ All sensors spawned");

    // ✅ Subphase 12.3.2: Create UI components
    let mut notifications = self::ui::FadingText::default();
    // ✅ Subphase 12.12.2: Help overlay
    let mut help = self::ui::HelpText::new(config.width as f32, config.height as f32);

    info!("✓ All components initialized");
    info!("Press WASD/Arrow keys to control vehicle");
    info!("Press P to toggle autopilot");
    info!("Press TAB to cycle camera positions");
    info!("Press ESC to quit");

    // ✅ Subphase 12.1.2, 12.2, 12.3, and 12.4: Main game loop
    loop {
        let delta_time = get_frame_time();

        // ✅ Subphase 12.3.1: Parse continuous vehicle keys (WASD/arrows)
        keyboard.parse_vehicle_keys(delta_time);

        // ✅ Subphase 12.3.1: Apply vehicle control
        keyboard.apply_control(&mut world)?;

        // ✅ Subphase 12.3.2: Parse keyboard events (P for autopilot, ESC to quit)
        // ✅ Subphase 12.10: Recording and replay (Ctrl+R, Ctrl+P, Ctrl+Minus/Plus, R)
        // ✅ Subphase 12.12: Help and HUD toggle (H, F1)
        if keyboard.parse_events(
            &mut client,
            &mut world,
            &mut hud,
            &mut camera,
            &mut notifications,
            &mut help,
        ) {
            info!("Quitting...");
            break;
        }

        // ✅ Subphase 12.2.2: Handle TAB key for camera switching
        if is_key_pressed(KeyCode::Tab) {
            info!("TAB pressed - switching camera");
            camera.toggle_camera(&mut world)?;
        }

        // ✅ Subphase 12.4: Update HUD telemetry
        hud.update(&world, delta_time);

        // ✅ Subphase 12.3.2: Update notification fade timer
        notifications.update(delta_time);

        // Render camera view
        camera.render()?;

        // ✅ Subphase 12.4: Render HUD overlay
        hud.render()?;

        // ✅ Subphase 12.3.2: Render notifications
        notifications.render();

        // ✅ Subphase 12.12.2: Render help overlay
        help.render();

        next_frame().await;
    }

    info!("Cleaning up...");
    camera.destroy();
    world.destroy();
    info!("✓ Cleanup complete");
    Ok(())
}

// TODO Phase 1-12: Implementation plan
//
// Phase 1: Foundation and Basic Window
//   - [x] 1.1: Project setup (DONE - package created)
//   - [ ] 1.2: Vehicle spawning and display
//
// Phase 2: Camera and Basic Rendering
//   - [ ] 2.1: RGB camera integration
//   - [ ] 2.2: Camera position switching
//
// Phase 3: Basic Vehicle Control
//   - [ ] 3.1: Keyboard input system (WASD)
//   - [ ] 3.2: Autopilot toggle (P key)
//
// Phase 4: HUD - Basic Telemetry
//   - [ ] 4.1: HUD struct and FPS display
//   - [ ] 4.2: Vehicle telemetry (speed, location)
//
// Phase 5: Advanced Sensors
//   - [ ] 5.1: IMUSensor implementation
//   - [ ] 5.2: GnssSensor implementation
//   - [ ] 5.3: CollisionSensor implementation
//   - [ ] 5.4: LaneInvasionSensor implementation
//
// Phase 6: HUD - Advanced Display
//   - [ ] 6.1: Control state display (bars, checkboxes)
//   - [ ] 6.2: Nearby vehicles list
//
// Phase 7: Advanced Controls
//   - [ ] 7.1: Manual transmission
//   - [ ] 7.2: Light controls
//   - [ ] 7.3: Ackermann controller
//
// Phase 8: Multiple Camera Types
//   - [ ] 8.1: Camera type switching (14 types)
//   - [ ] 8.2: LiDAR visualization
//
// Phase 9: Weather and World Control
//   - [ ] 9.1: Weather cycling
//   - [ ] 9.2: Map layer control
//
// Phase 10: Recording and Replay
//   - [ ] 10.1: Recording system
//   - [ ] 10.2: Replay system
//   - [ ] 10.3: Camera recording
//
// Phase 11: Advanced Features
//   - [ ] 11.1: Radar visualization
//   - [ ] 11.2: Vehicle door control
//   - [ ] 11.3: Constant velocity mode
//   - [ ] 11.4: Vehicle telemetry overlay
//
// Phase 12: Help System and Notifications
//   - [ ] 12.1: FadingText notification system
//   - [ ] 12.2: HelpText overlay
//   - [ ] 12.3: Info toggle (F1)
