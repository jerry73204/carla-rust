//! Multiple Sensor Grid View - Phase 14.2
//!
//! This example demonstrates a 2x3 grid layout showing multiple camera and LiDAR sensors
//! in real-time. It matches the Python PythonAPI/examples/visualize_multiple_sensors.py
//! reference implementation.
//!
//! Grid Layout:
//! ```
//! [Left Camera]   [Front Camera]   [Right Camera]
//! [LiDAR]        [Rear Camera]    [Semantic LiDAR]
//! ```
//!
//! Features:
//! - 4 RGB cameras at different angles (left, front, right, rear)
//! - Regular LiDAR with bird's-eye view (white points on black)
//! - Semantic LiDAR with bird's-eye view (white points on black)
//! - 2x3 grid layout with synchronized rendering
//! - Real-time sensor data display
//!
//! Run with: `cargo run --example sensor_grid_view --profile dev-release`

use anyhow::{Context, Result};
use carla::{
    client::{ActorBase, Client, Sensor, Vehicle, World as CarlaWorld},
    geom::Transform,
    rpc::{ActorId, AttachmentType},
    sensor::data::{
        Image, LidarDetection, LidarMeasurement, SemanticLidarDetection, SemanticLidarMeasurement,
    },
};
use macroquad::prelude::*;
use std::{
    collections::HashMap,
    sync::{Arc, Mutex},
    time::Duration,
};

const WINDOW_WIDTH: i32 = 1280;
const WINDOW_HEIGHT: i32 = 720;
const GRID_ROWS: usize = 2;
const GRID_COLS: usize = 3;
const LIDAR_RANGE: f32 = 100.0;

// ViewportInfo tracks which grid cell renders which sensor
#[derive(Debug, Clone)]
struct ViewportInfo {
    row: usize,
    col: usize,
    sensor_type: String,
    actor_id: ActorId,
}

impl ViewportInfo {
    fn new(row: usize, col: usize, sensor_type: String, actor_id: ActorId) -> Self {
        Self {
            row,
            col,
            sensor_type,
            actor_id,
        }
    }

    fn get_bounds(&self, window_width: i32, window_height: i32) -> (i32, i32, i32, i32) {
        let cell_width = window_width / GRID_COLS as i32;
        let cell_height = window_height / GRID_ROWS as i32;
        let x = self.col as i32 * cell_width;
        let y = self.row as i32 * cell_height;
        (x, y, cell_width, cell_height)
    }
}

// DisplayManager manages the 2x3 grid layout
struct DisplayManager {
    viewports: Vec<ViewportInfo>,
    camera_textures: HashMap<ActorId, Texture2D>,
    lidar_textures: HashMap<ActorId, Texture2D>,
}

impl DisplayManager {
    fn new() -> Self {
        Self {
            viewports: Vec::new(),
            camera_textures: HashMap::new(),
            lidar_textures: HashMap::new(),
        }
    }

    fn add_sensor(&mut self, row: usize, col: usize, sensor_type: String, actor_id: ActorId) {
        self.viewports
            .push(ViewportInfo::new(row, col, sensor_type, actor_id));
    }

    fn update_camera_texture(&mut self, actor_id: ActorId, texture: Texture2D) {
        self.camera_textures.insert(actor_id, texture);
    }

    fn update_lidar_texture(&mut self, actor_id: ActorId, texture: Texture2D) {
        self.lidar_textures.insert(actor_id, texture);
    }

    fn render(&self) {
        let window_width = screen_width() as i32;
        let window_height = screen_height() as i32;

        // Clear background
        clear_background(BLACK);

        // Render each viewport
        for viewport in &self.viewports {
            let (x, y, width, height) = viewport.get_bounds(window_width, window_height);

            // Draw sensor data
            if viewport.sensor_type.contains("Camera") {
                if let Some(texture) = self.camera_textures.get(&viewport.actor_id) {
                    draw_texture_ex(
                        texture,
                        x as f32,
                        y as f32,
                        WHITE,
                        DrawTextureParams {
                            dest_size: Some(vec2(width as f32, height as f32)),
                            ..Default::default()
                        },
                    );
                }
            } else if viewport.sensor_type.contains("LiDAR") {
                if let Some(texture) = self.lidar_textures.get(&viewport.actor_id) {
                    draw_texture_ex(
                        texture,
                        x as f32,
                        y as f32,
                        WHITE,
                        DrawTextureParams {
                            dest_size: Some(vec2(width as f32, height as f32)),
                            ..Default::default()
                        },
                    );
                }
            }

            // Draw grid lines
            draw_rectangle_lines(x as f32, y as f32, width as f32, height as f32, 2.0, GRAY);

            // Draw label
            draw_text(
                &viewport.sensor_type,
                x as f32 + 10.0,
                y as f32 + 25.0,
                20.0,
                WHITE,
            );
        }

        // Draw help text
        draw_text(
            "ESC/Q: Quit",
            10.0,
            window_height as f32 - 10.0,
            20.0,
            WHITE,
        );
    }
}

// CameraSensor manages a single camera sensor
struct CameraSensor {
    _sensor: Sensor,
    actor_id: ActorId,
    latest_image: Arc<Mutex<Option<Image>>>,
}

impl CameraSensor {
    fn spawn(world: &mut CarlaWorld, transform: &Transform, parent: &Vehicle) -> Result<Self> {
        let blueprint_library = world.blueprint_library();
        let camera_bp = blueprint_library
            .iter()
            .find(|bp| bp.id() == "sensor.camera.rgb")
            .context("Failed to find RGB camera blueprint")?;

        let actor = world
            .spawn_actor_opt(&camera_bp, transform, Some(parent), AttachmentType::Rigid)
            .context("Failed to spawn camera sensor")?;

        let sensor = Sensor::try_from(actor)
            .map_err(|_| anyhow::anyhow!("Failed to convert actor to sensor"))?;

        let actor_id = sensor.id();
        let latest_image = Arc::new(Mutex::new(None));
        let image_clone = Arc::clone(&latest_image);

        sensor.listen(move |data| {
            if let Ok(image) = Image::try_from(data) {
                *image_clone.lock().unwrap() = Some(image);
            }
        });

        Ok(Self {
            _sensor: sensor,
            actor_id,
            latest_image,
        })
    }

    fn get_latest_image(&self) -> Option<Image> {
        self.latest_image.lock().unwrap().clone()
    }

    fn actor_id(&self) -> ActorId {
        self.actor_id
    }
}

// Type alias for LiDAR points to reduce complexity
type LidarPoints = Arc<Mutex<Option<Vec<(f32, f32)>>>>;

// LidarSensor manages a single LiDAR sensor
struct LidarSensor {
    _sensor: Sensor,
    actor_id: ActorId,
    latest_points: LidarPoints,
}

impl LidarSensor {
    fn spawn(world: &mut CarlaWorld, parent: &Vehicle, is_semantic: bool) -> Result<Self> {
        let blueprint_library = world.blueprint_library();
        let blueprint_id = if is_semantic {
            "sensor.lidar.ray_cast_semantic"
        } else {
            "sensor.lidar.ray_cast"
        };

        let mut lidar_bp = blueprint_library
            .iter()
            .find(|bp| bp.id() == blueprint_id)
            .context("Failed to find LiDAR blueprint")?
            .clone();

        // Set LiDAR parameters matching Python example
        let _ = lidar_bp.set_attribute("range", "100.0");
        let _ = lidar_bp.set_attribute("rotation_frequency", "10.0");
        let _ = lidar_bp.set_attribute("channels", "64");
        let _ = lidar_bp.set_attribute("points_per_second", "100000");

        let transform = Transform {
            location: carla::geom::Location::new(0.0, 0.0, 0.0),
            rotation: carla::geom::Rotation::new(0.0, 0.0, 0.0),
        };

        let actor = world
            .spawn_actor_opt(&lidar_bp, &transform, Some(parent), AttachmentType::Rigid)
            .context("Failed to spawn LiDAR sensor")?;

        let sensor = Sensor::try_from(actor)
            .map_err(|_| anyhow::anyhow!("Failed to convert actor to sensor"))?;

        let actor_id = sensor.id();
        let latest_points = Arc::new(Mutex::new(None));
        let points_clone = Arc::clone(&latest_points);

        if is_semantic {
            sensor.listen(move |data| {
                if let Ok(measurement) = SemanticLidarMeasurement::try_from(data) {
                    let points: Vec<(f32, f32)> = measurement
                        .as_slice()
                        .iter()
                        .map(|detection: &SemanticLidarDetection| {
                            (detection.point.x, detection.point.y)
                        })
                        .collect();
                    *points_clone.lock().unwrap() = Some(points);
                }
            });
        } else {
            sensor.listen(move |data| {
                if let Ok(measurement) = LidarMeasurement::try_from(data) {
                    let points: Vec<(f32, f32)> = measurement
                        .as_slice()
                        .iter()
                        .map(|detection: &LidarDetection| (detection.point.x, detection.point.y))
                        .collect();
                    *points_clone.lock().unwrap() = Some(points);
                }
            });
        }

        Ok(Self {
            _sensor: sensor,
            actor_id,
            latest_points,
        })
    }

    fn get_latest_points(&self) -> Option<Vec<(f32, f32)>> {
        self.latest_points.lock().unwrap().clone()
    }

    fn actor_id(&self) -> ActorId {
        self.actor_id
    }
}

// Render LiDAR points to texture (bird's-eye view)
fn render_lidar_to_texture(points: &[(f32, f32)], width: usize, height: usize) -> Texture2D {
    let mut bytes = vec![0u8; width * height * 4]; // RGBA

    let min_size = width.min(height) as f32;
    let scale = min_size / (LIDAR_RANGE * 2.0);
    let center_x = width as f32 / 2.0;
    let center_y = height as f32 / 2.0;

    // Render points as white dots
    for (x, y) in points {
        let px = (x * scale + center_x) as i32;
        let py = (y * scale + center_y) as i32;

        if px >= 0 && px < width as i32 && py >= 0 && py < height as i32 {
            let idx = ((py as usize * width) + px as usize) * 4;
            if idx + 3 < bytes.len() {
                bytes[idx] = 255; // R
                bytes[idx + 1] = 255; // G
                bytes[idx + 2] = 255; // B
                bytes[idx + 3] = 255; // A
            }
        }
    }

    Texture2D::from_image(&macroquad::texture::Image {
        width: width as u16,
        height: height as u16,
        bytes,
    })
}

// Convert CARLA Image to macroquad Texture2D
fn image_to_texture(image: &Image) -> Texture2D {
    // CARLA images are BGRA, need to convert to RGBA
    let raw_bytes = image.as_raw_bytes();
    let mut rgba_bytes = Vec::with_capacity(raw_bytes.len());
    for chunk in raw_bytes.chunks(4) {
        rgba_bytes.push(chunk[2]); // R
        rgba_bytes.push(chunk[1]); // G
        rgba_bytes.push(chunk[0]); // B
        rgba_bytes.push(chunk[3]); // A
    }

    Texture2D::from_image(&macroquad::texture::Image {
        width: image.width() as u16,
        height: image.height() as u16,
        bytes: rgba_bytes,
    })
}

fn window_conf() -> Conf {
    Conf {
        window_title: "CARLA Multiple Sensor Grid View".to_owned(),
        window_width: WINDOW_WIDTH,
        window_height: WINDOW_HEIGHT,
        window_resizable: false,
        ..Default::default()
    }
}

#[macroquad::main(window_conf)]
async fn main() -> Result<()> {
    println!("=== CARLA Multiple Sensor Grid View ===");
    println!("Connecting to CARLA...");

    let mut client = Client::connect("127.0.0.1", 2000, Some(2));
    client.set_timeout(Duration::from_secs(10));

    let mut world = client.world();
    println!("Connected to world: {}", world.map().name());

    // Spawn vehicle
    println!("Spawning vehicle...");
    let blueprint_library = world.blueprint_library();
    let vehicle_bp = blueprint_library
        .iter()
        .find(|bp| bp.id() == "vehicle.tesla.model3")
        .context("Failed to find vehicle blueprint")?;

    let spawn_points = world.map().recommended_spawn_points();
    let spawn_point = spawn_points.get(0).context("No spawn points available")?;

    let vehicle_actor = world
        .spawn_actor(&vehicle_bp, spawn_point)
        .context("Failed to spawn vehicle")?;

    let vehicle = Vehicle::try_from(vehicle_actor)
        .map_err(|_| anyhow::anyhow!("Failed to convert actor to vehicle"))?;

    vehicle.set_autopilot(true);
    println!("Vehicle spawned with autopilot enabled");

    // Setup display manager
    let mut display_manager = DisplayManager::new();

    // Setup cameras at different angles (all at z=2.4m above vehicle)
    println!("Setting up cameras...");

    // Left camera (row 0, col 0)
    let left_camera = CameraSensor::spawn(
        &mut world,
        &Transform {
            location: carla::geom::Location::new(0.0, 0.0, 2.4),
            rotation: carla::geom::Rotation {
                pitch: 0.0,
                yaw: -90.0,
                roll: 0.0,
            },
        },
        &vehicle,
    )?;
    display_manager.add_sensor(0, 0, "Left Camera".to_string(), left_camera.actor_id());

    // Front camera (row 0, col 1)
    let front_camera = CameraSensor::spawn(
        &mut world,
        &Transform {
            location: carla::geom::Location::new(0.0, 0.0, 2.4),
            rotation: carla::geom::Rotation {
                pitch: 0.0,
                yaw: 0.0,
                roll: 0.0,
            },
        },
        &vehicle,
    )?;
    display_manager.add_sensor(0, 1, "Front Camera".to_string(), front_camera.actor_id());

    // Right camera (row 0, col 2)
    let right_camera = CameraSensor::spawn(
        &mut world,
        &Transform {
            location: carla::geom::Location::new(0.0, 0.0, 2.4),
            rotation: carla::geom::Rotation {
                pitch: 0.0,
                yaw: 90.0,
                roll: 0.0,
            },
        },
        &vehicle,
    )?;
    display_manager.add_sensor(0, 2, "Right Camera".to_string(), right_camera.actor_id());

    // Rear camera (row 1, col 1)
    let rear_camera = CameraSensor::spawn(
        &mut world,
        &Transform {
            location: carla::geom::Location::new(0.0, 0.0, 2.4),
            rotation: carla::geom::Rotation {
                pitch: 0.0,
                yaw: 180.0,
                roll: 0.0,
            },
        },
        &vehicle,
    )?;
    display_manager.add_sensor(1, 1, "Rear Camera".to_string(), rear_camera.actor_id());

    // Setup LiDAR sensors
    println!("Setting up LiDAR sensors...");

    // Regular LiDAR (row 1, col 0)
    let lidar = LidarSensor::spawn(&mut world, &vehicle, false)?;
    display_manager.add_sensor(1, 0, "LiDAR".to_string(), lidar.actor_id());

    // Semantic LiDAR (row 1, col 2)
    let semantic_lidar = LidarSensor::spawn(&mut world, &vehicle, true)?;
    display_manager.add_sensor(
        1,
        2,
        "Semantic LiDAR".to_string(),
        semantic_lidar.actor_id(),
    );

    println!("\nAll sensors ready! Grid view active.");
    println!("Press ESC or Q to quit");

    // Main rendering loop
    loop {
        // Handle input
        if is_key_pressed(KeyCode::Escape) || is_key_pressed(KeyCode::Q) {
            println!("\nExiting...");
            break;
        }

        // Update camera textures
        if let Some(image) = left_camera.get_latest_image() {
            let texture = image_to_texture(&image);
            display_manager.update_camera_texture(left_camera.actor_id(), texture);
        }
        if let Some(image) = front_camera.get_latest_image() {
            let texture = image_to_texture(&image);
            display_manager.update_camera_texture(front_camera.actor_id(), texture);
        }
        if let Some(image) = right_camera.get_latest_image() {
            let texture = image_to_texture(&image);
            display_manager.update_camera_texture(right_camera.actor_id(), texture);
        }
        if let Some(image) = rear_camera.get_latest_image() {
            let texture = image_to_texture(&image);
            display_manager.update_camera_texture(rear_camera.actor_id(), texture);
        }

        // Update LiDAR textures
        if let Some(points) = lidar.get_latest_points() {
            let cell_width = WINDOW_WIDTH / GRID_COLS as i32;
            let cell_height = WINDOW_HEIGHT / GRID_ROWS as i32;
            let texture =
                render_lidar_to_texture(&points, cell_width as usize, cell_height as usize);
            display_manager.update_lidar_texture(lidar.actor_id(), texture);
        }
        if let Some(points) = semantic_lidar.get_latest_points() {
            let cell_width = WINDOW_WIDTH / GRID_COLS as i32;
            let cell_height = WINDOW_HEIGHT / GRID_ROWS as i32;
            let texture =
                render_lidar_to_texture(&points, cell_width as usize, cell_height as usize);
            display_manager.update_lidar_texture(semantic_lidar.actor_id(), texture);
        }

        // Render display
        display_manager.render();

        next_frame().await;
    }

    println!("Cleaning up...");
    println!("Note: Actors remain in simulation (manual cleanup required)");

    Ok(())
}
