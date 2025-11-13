//! Synchronous Mode GUI - Fixed Timestep Simulation Demo
//!
//! Demonstrates CARLA's synchronous mode with frame-perfect simulation timing.
//! Shows dual camera view (RGB + semantic segmentation blend) with waypoint following.
//!
//! # Features
//! - Synchronous mode with fixed 30 FPS simulation
//! - Dual camera display (RGB base + semantic overlay)
//! - Frame counter and timing statistics
//! - Simple waypoint following (teleport-based)
//! - Pause/resume capability
//!
//! # Controls
//! - **Space**: Pause/unpause simulation
//! - **R**: Reset vehicle to spawn point
//! - **H**: Toggle help overlay
//! - **ESC/Q**: Quit
//!
//! # Usage
//! ```bash
//! cargo run --example synchronous_mode_gui --profile dev-release
//! ```

use anyhow::Result;
use carla::{
    client::{ActorBase, Client, Sensor, Vehicle, World as CarlaWorld},
    geom::Location,
    rpc::{EpisodeSettings, WeatherParameters},
    sensor::data::Image as CarlaImage,
};
use macroquad::prelude::*;
use std::{
    sync::{Arc, Mutex},
    time::Duration,
};

const WINDOW_WIDTH: u32 = 800;
const WINDOW_HEIGHT: u32 = 600;
const CAMERA_WIDTH: u32 = 800;
const CAMERA_HEIGHT: u32 = 600;
const FIXED_DELTA_SECONDS: f32 = 1.0 / 30.0; // 30 FPS simulation

fn window_conf() -> Conf {
    Conf {
        window_title: "CARLA - Synchronous Mode Demo".to_owned(),
        window_width: WINDOW_WIDTH as i32,
        window_height: WINDOW_HEIGHT as i32,
        window_resizable: false,
        ..Default::default()
    }
}

struct CameraManager {
    rgb_sensor: Sensor,
    semantic_sensor: Sensor,
    texture: Texture2D,
    latest_rgb: Arc<Mutex<Option<CarlaImage>>>,
    latest_semantic: Arc<Mutex<Option<CarlaImage>>>,
}

impl CameraManager {
    fn new(world: &mut CarlaWorld, vehicle: &Vehicle) -> Result<Self> {
        let blueprint_library = world.blueprint_library();

        // Create RGB camera
        let rgb_bp = blueprint_library
            .find("sensor.camera.rgb")
            .ok_or_else(|| anyhow::anyhow!("RGB camera blueprint not found"))?;

        let mut rgb_bp = rgb_bp;
        let _ = rgb_bp.set_attribute("image_size_x", &CAMERA_WIDTH.to_string());
        let _ = rgb_bp.set_attribute("image_size_y", &CAMERA_HEIGHT.to_string());
        let _ = rgb_bp.set_attribute("fov", "110");

        // Create semantic segmentation camera
        let semantic_bp = blueprint_library
            .find("sensor.camera.semantic_segmentation")
            .ok_or_else(|| anyhow::anyhow!("Semantic camera blueprint not found"))?;

        let mut semantic_bp = semantic_bp;
        let _ = semantic_bp.set_attribute("image_size_x", &CAMERA_WIDTH.to_string());
        let _ = semantic_bp.set_attribute("image_size_y", &CAMERA_HEIGHT.to_string());
        let _ = semantic_bp.set_attribute("fov", "110");

        // Camera transform (rear view)
        let mut camera_transform = carla::geom::Transform {
            location: Location::new(0.0, 0.0, 0.0),
            rotation: carla::geom::Rotation::new(0.0, 0.0, 0.0),
        };
        camera_transform.location.x = -5.5;
        camera_transform.location.z = 2.5;

        // Spawn cameras
        let rgb_camera = world.spawn_actor_attached(
            &rgb_bp,
            &camera_transform,
            vehicle,
            carla::rpc::AttachmentType::SpringArm,
        )?;

        let semantic_camera = world.spawn_actor_attached(
            &semantic_bp,
            &camera_transform,
            vehicle,
            carla::rpc::AttachmentType::SpringArm,
        )?;

        let rgb_sensor = Sensor::try_from(rgb_camera)
            .map_err(|_| anyhow::anyhow!("Failed to cast to Sensor"))?;
        let semantic_sensor = Sensor::try_from(semantic_camera)
            .map_err(|_| anyhow::anyhow!("Failed to cast to Sensor"))?;

        // Create texture
        let texture = Texture2D::from_rgba8(
            CAMERA_WIDTH as u16,
            CAMERA_HEIGHT as u16,
            &vec![0u8; (CAMERA_WIDTH * CAMERA_HEIGHT * 4) as usize],
        );
        texture.set_filter(FilterMode::Linear);

        // Setup listeners
        let latest_rgb = Arc::new(Mutex::new(None));
        let latest_rgb_clone = Arc::clone(&latest_rgb);

        let latest_semantic = Arc::new(Mutex::new(None));
        let latest_semantic_clone = Arc::clone(&latest_semantic);

        rgb_sensor.listen(move |data| {
            if let Ok(image) = CarlaImage::try_from(data) {
                if let Ok(mut img) = latest_rgb_clone.lock() {
                    *img = Some(image);
                }
            }
        });

        semantic_sensor.listen(move |data| {
            if let Ok(image) = CarlaImage::try_from(data) {
                if let Ok(mut img) = latest_semantic_clone.lock() {
                    *img = Some(image);
                }
            }
        });

        Ok(Self {
            rgb_sensor,
            semantic_sensor,
            texture,
            latest_rgb,
            latest_semantic,
        })
    }

    fn update(&mut self) -> bool {
        let rgb_opt = self.latest_rgb.lock().ok().and_then(|mut img| img.take());
        let semantic_opt = self
            .latest_semantic
            .lock()
            .ok()
            .and_then(|mut img| img.take());

        if let (Some(rgb), Some(semantic)) = (rgb_opt, semantic_opt) {
            // Blend RGB with semantic segmentation
            let rgb_data = rgb.as_slice();
            let semantic_data = semantic.as_slice();
            let mut rgba_data = Vec::with_capacity((CAMERA_WIDTH * CAMERA_HEIGHT * 4) as usize);

            for i in 0..(CAMERA_WIDTH * CAMERA_HEIGHT) as usize {
                let rgb_color = &rgb_data[i];
                let sem_color = &semantic_data[i];

                // Blend: 70% RGB + 30% semantic
                let r = ((rgb_color.r as f32 * 0.7) + (sem_color.r as f32 * 0.3)) as u8;
                let g = ((rgb_color.g as f32 * 0.7) + (sem_color.g as f32 * 0.3)) as u8;
                let b = ((rgb_color.b as f32 * 0.7) + (sem_color.b as f32 * 0.3)) as u8;

                rgba_data.push(r);
                rgba_data.push(g);
                rgba_data.push(b);
                rgba_data.push(255);
            }

            self.texture =
                Texture2D::from_rgba8(CAMERA_WIDTH as u16, CAMERA_HEIGHT as u16, &rgba_data);
            self.texture.set_filter(FilterMode::Linear);
            return true;
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
        frame_count: u64,
        simulated_fps: f32,
        vehicle: &Vehicle,
        paused: bool,
        waypoint_info: &str,
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

        let real_fps = get_fps();

        let texts = vec![
            "CARLA - Synchronous Mode Demo".to_string(),
            "".to_string(),
            format!("Frame: {}", frame_count),
            format!("Real FPS: {:.1}", real_fps),
            format!("Simulated FPS: {:.1}", simulated_fps),
            format!("Status: {}", if paused { "PAUSED" } else { "Running" }),
            "".to_string(),
            format!("Speed: {:.1} km/h", speed_kmh),
            format!(
                "Location: ({:.1}, {:.1}, {:.1})",
                location.x, location.y, location.z
            ),
            format!("Waypoint: {}", waypoint_info),
            "".to_string(),
            "Controls:".to_string(),
            "  Space - Pause/Resume".to_string(),
            "  R - Reset".to_string(),
            "  H - Toggle HUD".to_string(),
            "  ESC/Q - Quit".to_string(),
        ];

        for text in texts {
            let text_width = measure_text(&text, None, 20, 1.0).width;
            draw_rectangle(
                10.0,
                y - 15.0,
                text_width + 10.0,
                line_height,
                Color::from_rgba(0, 0, 0, 180),
            );
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
            ("SYNCHRONOUS MODE DEMO", 30.0),
            ("", 20.0),
            (
                "This demo shows CARLA's synchronous mode with fixed timestep simulation.",
                20.0,
            ),
            (
                "The simulation runs at exactly 30 FPS (33.3ms per frame).",
                20.0,
            ),
            ("", 20.0),
            ("CONTROLS:", 25.0),
            ("", 20.0),
            ("Space          Pause/Resume simulation", 20.0),
            ("R              Reset vehicle to spawn point", 20.0),
            ("H              Toggle this help overlay", 20.0),
            ("ESC / Q        Quit application", 20.0),
            ("", 20.0),
            ("VISUALIZATION:", 25.0),
            ("", 20.0),
            ("• RGB camera view blended with semantic segmentation", 20.0),
            ("• Frame counter shows simulation frames", 20.0),
            ("• Real FPS vs Simulated FPS comparison", 20.0),
            ("• Vehicle follows waypoints along the road", 20.0),
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
    println!("Synchronous Mode GUI - Connecting to CARLA...");

    // Connect to CARLA
    let client = Client::connect("localhost", 2000, 0);
    let mut world = client.world();

    println!("Connected! Enabling synchronous mode...");

    // Enable synchronous mode
    let settings = EpisodeSettings {
        synchronous_mode: true,
        fixed_delta_seconds: Some(FIXED_DELTA_SECONDS as f64),
        ..Default::default()
    };
    world.apply_settings(&settings, Duration::from_secs(2));

    println!("Synchronous mode enabled (30 FPS fixed timestep)");

    // Set weather (clear noon)
    let weather = WeatherParameters {
        cloudiness: 0.0,
        precipitation: 0.0,
        precipitation_deposits: 0.0,
        wind_intensity: 0.0,
        sun_azimuth_angle: 0.0,
        sun_altitude_angle: 70.0,
        fog_density: 0.0,
        fog_distance: 0.0,
        wetness: 0.0,
        fog_falloff: 0.0,
        scattering_intensity: 0.0,
        mie_scattering_scale: 0.0,
        rayleigh_scattering_scale: 0.0331,
        dust_storm: 0.0,
    };
    world.set_weather(&weather);

    // Get spawn points
    let map = world.map();
    let spawn_points = map.recommended_spawn_points();

    if spawn_points.is_empty() {
        return Err(anyhow::anyhow!("No spawn points available"));
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

    // Disable physics for smooth waypoint following
    vehicle.set_simulate_physics(false);

    println!("Vehicle spawned: {}", vehicle.type_id());

    // Create camera
    let mut camera = CameraManager::new(&mut world, &vehicle)?;
    println!("Cameras attached (RGB + Semantic)");

    // Create UI
    let mut hud = Hud::new();
    let mut help = HelpOverlay::new();

    // Simulation state
    let mut frame_count: u64 = 0;
    let mut paused = false;
    let mut simulated_fps = 30.0;
    let mut last_frame_time = get_time();

    // Get initial waypoint
    let initial_transform = vehicle.transform();
    let mut current_waypoint = map
        .waypoint_at(&initial_transform.location)
        .ok_or_else(|| anyhow::anyhow!("No waypoint at spawn location"))?;

    println!("Starting main loop...");
    println!("Press H for help");

    // Tick once to start
    world.tick();

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

        if is_key_pressed(KeyCode::Space) {
            paused = !paused;
            println!("Simulation {}", if paused { "PAUSED" } else { "RESUMED" });
        }

        if is_key_pressed(KeyCode::R) {
            println!("Resetting vehicle...");
            vehicle.set_transform(spawn_point);
            current_waypoint = map
                .waypoint_at(&spawn_point.location)
                .ok_or_else(|| anyhow::anyhow!("No waypoint at spawn location"))?;
        }

        // Simulation tick
        if !paused {
            // Follow waypoints
            let next_waypoints = current_waypoint.next(1.5);
            if !next_waypoints.is_empty() {
                // Pick random next waypoint
                let idx = (rand::gen_range(0, next_waypoints.len() as i32)) as usize;
                current_waypoint = next_waypoints.get(idx).unwrap().clone();

                // Teleport vehicle to waypoint
                vehicle.set_transform(&current_waypoint.transform());
            }

            // Tick world
            world.tick();
            frame_count += 1;

            // Calculate simulated FPS
            let current_time = get_time();
            let delta = current_time - last_frame_time;
            if delta > 0.0 {
                simulated_fps = 1.0 / delta as f32;
            }
            last_frame_time = current_time;
        }

        // Update camera
        camera.update();

        // Prepare waypoint info
        let waypoint_info = format!(
            "Road {}, Lane {}",
            current_waypoint.road_id(),
            current_waypoint.lane_id()
        );

        // Render
        clear_background(BLACK);
        camera.render();
        hud.render(frame_count, simulated_fps, &vehicle, paused, &waypoint_info);
        help.render();

        next_frame().await;
    }

    println!("Cleaning up...");

    // Restore asynchronous mode
    let settings = EpisodeSettings {
        synchronous_mode: false,
        fixed_delta_seconds: None,
        ..Default::default()
    };
    world.apply_settings(&settings, Duration::from_secs(2));

    vehicle.destroy();
    camera.rgb_sensor.destroy();
    camera.semantic_sensor.destroy();

    println!("Done!");

    Ok(())
}
