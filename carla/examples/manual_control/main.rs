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
    info!("See docs/roadmap.md Phase 13 for implementation plan");

    // Parse configuration (TODO: Add CLI arguments)
    let config = Config::default();

    // Connect to CARLA server
    info!("Connecting to CARLA at {}:{}", config.host, config.port);
    let client = carla::client::Client::connect(&config.host, config.port, None);

    // TODO Phase 1.2: Create world and spawn vehicle
    let mut _world = self::world::World::new(&client, &config)?;
    info!("✓ World initialized");

    // TODO Phase 2.1: Create camera manager
    let mut _camera = self::camera::CameraManager::new(config.width, config.height, config.gamma);

    // TODO Phase 3: Create keyboard controller
    let mut _keyboard = self::keyboard::KeyboardControl::new(config.autopilot);

    // TODO Phase 4: Create HUD
    let mut _hud = self::hud::HUD::new(config.width, config.height);

    // TODO Phase 12: Create UI components
    let mut _notifications = self::ui::FadingText::default();
    let mut _help = self::ui::HelpText::new(config.width as f32, config.height as f32);

    info!("✓ All components initialized");
    info!("⚠️  Phase 1-12 implementation needed - see TODOs in source files");

    // TODO Phase 1.1: Implement main game loop
    // - Clear background
    // - Update world (tick in sync mode)
    // - Parse keyboard input
    // - Apply vehicle control
    // - Update HUD
    // - Render camera
    // - Render HUD
    // - Render notifications
    // - Render help
    // - Handle quit (ESC)

    // Placeholder main loop for Phase 1.1
    loop {
        clear_background(BLACK);

        // Display placeholder text
        let text = "CARLA Manual Control - Phase 1.1 TODO";
        let text_width = measure_text(text, None, 40, 1.0).width;
        draw_text(
            text,
            (screen_width() - text_width) / 2.0,
            screen_height() / 2.0,
            40.0,
            WHITE,
        );

        let instructions = "See src/main.rs for implementation TODO items";
        let inst_width = measure_text(instructions, None, 20, 1.0).width;
        draw_text(
            instructions,
            (screen_width() - inst_width) / 2.0,
            screen_height() / 2.0 + 40.0,
            20.0,
            GRAY,
        );

        let quit = "Press ESC to quit";
        let quit_width = measure_text(quit, None, 20, 1.0).width;
        draw_text(
            quit,
            (screen_width() - quit_width) / 2.0,
            screen_height() / 2.0 + 80.0,
            20.0,
            DARKGRAY,
        );

        // ESC to quit
        if is_key_pressed(KeyCode::Escape) {
            info!("Quitting...");
            break;
        }

        next_frame().await;
    }

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
