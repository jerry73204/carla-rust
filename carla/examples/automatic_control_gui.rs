//! Automatic Control GUI - BasicAgent Navigation Demo
//!
//! Demonstrates autonomous vehicle control using BasicAgent with a real-time GUI.
//! The agent navigates to random destinations while avoiding obstacles and following traffic rules.
//!
//! # Features
//! - BasicAgent autonomous navigation
//! - Real-time camera feed from vehicle
//! - HUD showing speed, location, destination distance
//! - Agent status display
//! - Manual override capability
//! - Automatic respawn with new destinations
//!
//! # Controls
//! - **P**: Toggle agent on/off (manual override - autopilot mode)
//! - **R**: Respawn at new location with new destination
//! - **H**: Toggle help overlay
//! - **ESC/Q**: Quit
//!
//! # Usage
//! ```bash
//! cargo run --example automatic_control_gui --profile dev-release
//! ```

use anyhow::Result;
use carla::{
    agents::navigation::{BasicAgent, BasicAgentConfig},
    client::{ActorBase, Client, Sensor, Vehicle, World as CarlaWorld},
    geom::Location,
    sensor::data::Image as CarlaImage,
};
use macroquad::prelude::*;
use std::sync::{Arc, Mutex};

const WINDOW_WIDTH: u32 = 1280;
const WINDOW_HEIGHT: u32 = 720;
const CAMERA_WIDTH: u32 = 1280;
const CAMERA_HEIGHT: u32 = 720;

fn window_conf() -> Conf {
    Conf {
        window_title: "CARLA - Automatic Control (BasicAgent)".to_owned(),
        window_width: WINDOW_WIDTH as i32,
        window_height: WINDOW_HEIGHT as i32,
        window_resizable: false,
        ..Default::default()
    }
}

struct CameraManager {
    sensor: Sensor,
    texture: Texture2D,
    latest_image: Arc<Mutex<Option<CarlaImage>>>,
}

impl CameraManager {
    fn new(world: &mut CarlaWorld, vehicle: &Vehicle) -> Result<Self> {
        // Create RGB camera
        let blueprint_library = world.blueprint_library();
        let camera_bp = blueprint_library
            .find("sensor.camera.rgb")
            .ok_or_else(|| anyhow::anyhow!("Camera blueprint not found"))?;

        // Set camera attributes
        let mut camera_bp = camera_bp;
        let _ = camera_bp.set_attribute("image_size_x", &CAMERA_WIDTH.to_string());
        let _ = camera_bp.set_attribute("image_size_y", &CAMERA_HEIGHT.to_string());
        let _ = camera_bp.set_attribute("fov", "110");

        // Spawn camera behind vehicle
        let mut camera_transform = carla::geom::Transform {
            location: Location::new(0.0, 0.0, 0.0),
            rotation: carla::geom::Rotation::new(0.0, 0.0, 0.0),
        };
        camera_transform.location.x = -5.5;
        camera_transform.location.z = 2.5;

        let camera = world.spawn_actor_attached(
            &camera_bp,
            &camera_transform,
            vehicle,
            carla::rpc::AttachmentType::SpringArm,
        )?;

        let sensor =
            Sensor::try_from(camera).map_err(|_| anyhow::anyhow!("Failed to cast to Sensor"))?;

        // Create texture
        let texture = Texture2D::from_rgba8(
            CAMERA_WIDTH as u16,
            CAMERA_HEIGHT as u16,
            &vec![0u8; (CAMERA_WIDTH * CAMERA_HEIGHT * 4) as usize],
        );
        texture.set_filter(FilterMode::Linear);

        // Setup listener
        let latest_image = Arc::new(Mutex::new(None));
        let latest_image_clone = Arc::clone(&latest_image);

        sensor.listen(move |data| {
            if let Ok(image) = CarlaImage::try_from(data) {
                if let Ok(mut img) = latest_image_clone.lock() {
                    *img = Some(image);
                }
            }
        });

        Ok(Self {
            sensor,
            texture,
            latest_image,
        })
    }

    fn update(&mut self) -> bool {
        if let Ok(mut img_opt) = self.latest_image.lock() {
            if let Some(image) = img_opt.take() {
                // Convert CARLA Color format to RGBA
                let raw_data = image.as_slice();
                let mut rgba_data = Vec::with_capacity((CAMERA_WIDTH * CAMERA_HEIGHT * 4) as usize);

                for color in raw_data {
                    rgba_data.push(color.r); // R
                    rgba_data.push(color.g); // G
                    rgba_data.push(color.b); // B
                    rgba_data.push(color.a); // A
                }

                // Create new texture from RGBA data
                self.texture =
                    Texture2D::from_rgba8(CAMERA_WIDTH as u16, CAMERA_HEIGHT as u16, &rgba_data);
                self.texture.set_filter(FilterMode::Linear);
                return true;
            }
        }
        false
    }

    fn render(&self) {
        draw_texture_ex(
            &self.texture,
            0.0,
            0.0,
            WHITE,
            DrawTextureParams {
                dest_size: Some(vec2(WINDOW_WIDTH as f32, WINDOW_HEIGHT as f32)),
                ..Default::default()
            },
        );
    }
}

struct Hud {
    show: bool,
}

impl Hud {
    fn new() -> Self {
        Self { show: true }
    }

    fn toggle(&mut self) {
        self.show = !self.show;
    }

    fn render(
        &self,
        vehicle: &Vehicle,
        agent: &BasicAgent,
        dest_location: &Location,
        agent_enabled: bool,
    ) {
        if !self.show {
            return;
        }

        let y_offset = 20.0;
        let line_height = 20.0;
        let mut y = y_offset;

        // Get vehicle data
        let transform = vehicle.transform();
        let location = transform.location;
        let velocity = vehicle.velocity();
        let speed_ms = velocity.length();
        let speed_kmh = speed_ms * 3.6;

        // Calculate distance to destination
        let dx = dest_location.x - location.x;
        let dy = dest_location.y - location.y;
        let dz = dest_location.z - location.z;
        let distance = (dx * dx + dy * dy + dz * dz).sqrt();

        // Agent status
        let agent_status = if agent_enabled {
            if agent.done() {
                "Destination Reached"
            } else {
                "Navigating"
            }
        } else {
            "Manual (Autopilot)"
        };

        // Render HUD elements with semi-transparent background
        let texts = vec![
            format!("CARLA - Automatic Control (BasicAgent)"),
            format!(""),
            format!("Agent Status: {}", agent_status),
            format!("Speed: {:.1} km/h", speed_kmh),
            format!(
                "Location: ({:.1}, {:.1}, {:.1})",
                location.x, location.y, location.z
            ),
            format!(
                "Destination: ({:.1}, {:.1}, {:.1})",
                dest_location.x, dest_location.y, dest_location.z
            ),
            format!("Distance to Dest: {:.1} m", distance),
            format!(""),
            format!("Controls:"),
            format!("  P - Toggle Agent / Autopilot"),
            format!("  R - Respawn with new destination"),
            format!("  H - Toggle this HUD"),
            format!("  ESC/Q - Quit"),
        ];

        for text in texts {
            // Semi-transparent background
            let text_width = measure_text(&text, None, 20, 1.0).width;
            draw_rectangle(
                10.0,
                y - 15.0,
                text_width + 10.0,
                line_height,
                Color::from_rgba(0, 0, 0, 180),
            );

            // Text
            draw_text(&text, 15.0, y, 20.0, WHITE);
            y += line_height;
        }
    }
}

struct HelpOverlay {
    show: bool,
}

impl HelpOverlay {
    fn new() -> Self {
        Self { show: false }
    }

    fn toggle(&mut self) {
        self.show = !self.show;
    }

    fn render(&self) {
        if !self.show {
            return;
        }

        // Full screen semi-transparent background
        draw_rectangle(
            0.0,
            0.0,
            WINDOW_WIDTH as f32,
            WINDOW_HEIGHT as f32,
            Color::from_rgba(0, 0, 0, 200),
        );

        let center_x = WINDOW_WIDTH as f32 / 2.0;
        let start_y = 100.0;
        let line_height = 30.0;

        let help_text = vec![
            ("AUTOMATIC CONTROL - BASICAGENT", 30.0),
            ("", 20.0),
            (
                "The vehicle is controlled by an autonomous agent that navigates",
                20.0,
            ),
            (
                "to random destinations while avoiding obstacles and following traffic rules.",
                20.0,
            ),
            ("", 20.0),
            ("CONTROLS:", 25.0),
            ("", 20.0),
            (
                "P              Toggle Agent On/Off (Manual Override with Autopilot)",
                20.0,
            ),
            (
                "R              Respawn at new location with new destination",
                20.0,
            ),
            ("H              Toggle this help overlay", 20.0),
            ("ESC / Q        Quit application", 20.0),
            ("", 20.0),
            ("AGENT FEATURES:", 25.0),
            ("", 20.0),
            ("• Autonomous navigation to destination", 20.0),
            ("• Obstacle detection and avoidance", 20.0),
            ("• Traffic light compliance", 20.0),
            ("• Speed limit following", 20.0),
            ("• Smooth path following with PID control", 20.0),
            ("", 20.0),
            ("Press H to close this help", 20.0),
        ];

        let mut y = start_y;
        for (text, size) in help_text {
            let text_dims = measure_text(text, None, size as u16, 1.0);
            let x = center_x - text_dims.width / 2.0;
            draw_text(text, x, y, size, WHITE);
            y += line_height;
        }
    }
}

#[macroquad::main(window_conf)]
async fn main() -> Result<()> {
    println!("Automatic Control GUI - Connecting to CARLA...");

    // Connect to CARLA
    let client = Client::connect("localhost", 2000, 0);
    let mut world = client.world();

    println!("Connected! Setting up scene...");

    // Get spawn points
    let map = world.map();
    let spawn_points = map.recommended_spawn_points();

    if spawn_points.len() < 2 {
        return Err(anyhow::anyhow!("Need at least 2 spawn points"));
    }

    // Spawn vehicle
    let blueprint_library = world.blueprint_library();
    let vehicle_bp = blueprint_library
        .filter("vehicle.*")
        .iter()
        .next()
        .ok_or_else(|| anyhow::anyhow!("No vehicle blueprints found"))?;

    let spawn_point = &spawn_points.as_slice()[0];
    let vehicle_actor = world.spawn_actor(&vehicle_bp, spawn_point)?;
    let vehicle = Vehicle::try_from(vehicle_actor)
        .map_err(|_| anyhow::anyhow!("Failed to cast to Vehicle"))?;

    println!("Vehicle spawned: {}", vehicle.type_id());

    // Create camera
    let mut camera = CameraManager::new(&mut world, &vehicle)?;
    println!("Camera attached");

    // Create BasicAgent
    let config = BasicAgentConfig::default();
    let mut agent = BasicAgent::new(vehicle.clone(), config, None, None)?;

    // Set initial destination
    let dest_transform = &spawn_points.as_slice()[1];
    let dest_isometry = dest_transform.to_na();
    let mut destination = Location::from_na_translation(&dest_isometry.translation);

    println!(
        "Setting destination to ({:.1}, {:.1}, {:.1})",
        destination.x, destination.y, destination.z
    );
    agent.set_destination(destination, None, true)?;

    // Create UI
    let mut hud = Hud::new();
    let mut help = HelpOverlay::new();
    let mut agent_enabled = true;

    println!("Starting main loop...");
    println!("Press H for help");

    loop {
        // Handle input
        if is_key_pressed(KeyCode::Escape) || is_key_pressed(KeyCode::Q) {
            break;
        }

        if is_key_pressed(KeyCode::H) {
            help.toggle();
        }

        if is_key_pressed(KeyCode::F1) {
            hud.toggle();
        }

        if is_key_pressed(KeyCode::P) {
            agent_enabled = !agent_enabled;
            if agent_enabled {
                println!("Agent enabled");
            } else {
                println!("Agent disabled - switching to autopilot");
                vehicle.set_autopilot(true);
            }
        }

        if is_key_pressed(KeyCode::R) {
            // Respawn at new location
            println!("Respawning...");
            let new_spawn_idx = (rand::gen_range(0, spawn_points.len())) as usize;
            let new_dest_idx = (rand::gen_range(0, spawn_points.len())) as usize;

            let new_spawn = &spawn_points.as_slice()[new_spawn_idx];
            vehicle.set_transform(new_spawn);

            let dest_transform = &spawn_points.as_slice()[new_dest_idx];
            let dest_isometry = dest_transform.to_na();
            destination = Location::from_na_translation(&dest_isometry.translation);

            println!(
                "New destination: ({:.1}, {:.1}, {:.1})",
                destination.x, destination.y, destination.z
            );

            // Reset agent
            let config = BasicAgentConfig::default();
            agent = BasicAgent::new(vehicle.clone(), config, None, None)?;
            agent.set_destination(destination, None, true)?;
            agent_enabled = true;
        }

        // Update agent
        if agent_enabled {
            if agent.done() {
                println!("Destination reached! Respawning...");
                // Auto-respawn with new destination
                let new_dest_idx = (rand::gen_range(0, spawn_points.len())) as usize;
                let dest_transform = &spawn_points.as_slice()[new_dest_idx];
                let dest_isometry = dest_transform.to_na();
                destination = Location::from_na_translation(&dest_isometry.translation);

                println!(
                    "New destination: ({:.1}, {:.1}, {:.1})",
                    destination.x, destination.y, destination.z
                );

                agent.set_destination(destination, None, true)?;
            } else {
                // Get control from agent
                let control = agent.run_step()?;
                vehicle.apply_control(&control);
            }
        }

        // Update camera
        camera.update();

        // Render
        clear_background(BLACK);
        camera.render();
        hud.render(&vehicle, &agent, &destination, agent_enabled);
        help.render();

        next_frame().await;
    }

    println!("Cleaning up...");
    vehicle.destroy();
    camera.sensor.destroy();
    println!("Done!");

    Ok(())
}
