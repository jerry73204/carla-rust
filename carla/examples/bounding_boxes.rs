//! Bounding Boxes Visualization - 2D/3D Box Detection Demo
//!
//! Demonstrates 2D and 3D bounding box computation and visualization for computer vision.
//! Shows vehicles, pedestrians, and traffic lights with colored bounding boxes.
//!
//! # Features
//! - 2D bounding boxes projected onto camera image
//! - 3D bounding boxes in world space (visible in spectator)
//! - Color coding by actor type (vehicles=blue, walkers=red, traffic lights=yellow)
//! - Distance filtering
//! - Real-time camera feed with overlays
//!
//! # Controls
//! - **Space**: Toggle 2D boxes on/off
//! - **B**: Toggle 3D boxes on/off
//! - **+/-**: Adjust max distance filter
//! - **F**: Toggle distance filter on/off
//! - **H**: Toggle help overlay
//! - **ESC/Q**: Quit
//!
//! # Usage
//! ```bash
//! cargo run --example bounding_boxes --profile dev-release
//! ```

use anyhow::Result;
use carla::{
    client::{Actor, ActorBase, Client, Sensor, Vehicle, World as CarlaWorld},
    geom::{Location, Transform, Vector3D},
    rpc::Color,
    sensor::data::Image as CarlaImage,
};
use macroquad::prelude::*;
use std::sync::{Arc, Mutex};

const WINDOW_WIDTH: u32 = 1280;
const WINDOW_HEIGHT: u32 = 720;
const CAMERA_WIDTH: u32 = 1280;
const CAMERA_HEIGHT: u32 = 720;
const DEFAULT_MAX_DISTANCE: f32 = 50.0;

fn window_conf() -> Conf {
    Conf {
        window_title: "CARLA - Bounding Boxes Visualization".to_owned(),
        window_width: WINDOW_WIDTH as i32,
        window_height: WINDOW_HEIGHT as i32,
        window_resizable: false,
        ..Default::default()
    }
}

/// Camera intrinsic matrix for 3D to 2D projection
struct CameraIntrinsics {
    focal_length: f32,
    image_width: f32,
    image_height: f32,
}

impl CameraIntrinsics {
    fn new(fov: f32, image_width: u32, image_height: u32) -> Self {
        // Convert FOV to focal length
        // focal_length = image_width / (2.0 * tan(fov / 2.0))
        let fov_rad = fov.to_radians();
        let focal_length = (image_width as f32) / (2.0 * (fov_rad / 2.0).tan());

        Self {
            focal_length,
            image_width: image_width as f32,
            image_height: image_height as f32,
        }
    }

    /// Project 3D world point to 2D image coordinates
    /// Returns None if point is behind camera
    fn project_3d_to_2d(
        &self,
        point: &Vector3D,
        camera_transform: &Transform,
    ) -> Option<(f32, f32)> {
        // Transform world point to camera space
        let camera_pos = &camera_transform.location;
        let relative = Vector3D::new(
            point.x - camera_pos.x,
            point.y - camera_pos.y,
            point.z - camera_pos.z,
        );

        // Apply camera rotation (simplified - assumes camera looking forward)
        // In CARLA, camera forward is +X, right is +Y, up is +Z
        let yaw = camera_transform.rotation.yaw.to_radians();
        let pitch = camera_transform.rotation.pitch.to_radians();

        // Rotate point to camera space
        let cos_yaw = yaw.cos();
        let sin_yaw = yaw.sin();
        let cos_pitch = pitch.cos();
        let sin_pitch = pitch.sin();

        // Apply yaw rotation (around Z-axis)
        let x1 = relative.x * cos_yaw + relative.y * sin_yaw;
        let y1 = -relative.x * sin_yaw + relative.y * cos_yaw;
        let z1 = relative.z;

        // Apply pitch rotation (around Y-axis)
        let x2 = x1 * cos_pitch - z1 * sin_pitch;
        let y2 = y1;
        let z2 = x1 * sin_pitch + z1 * cos_pitch;

        // Check if point is behind camera (negative X in CARLA camera space)
        if x2 <= 0.0 {
            return None;
        }

        // Project to image plane
        // u = focal_length * (y / x) + cx
        // v = focal_length * (-z / x) + cy
        let cx = self.image_width / 2.0;
        let cy = self.image_height / 2.0;

        let u = self.focal_length * (y2 / x2) + cx;
        let v = self.focal_length * (-z2 / x2) + cy;

        Some((u, v))
    }
}

/// 2D bounding box in image space
#[derive(Debug, Clone)]
struct BoundingBox2D {
    min_x: f32,
    min_y: f32,
    max_x: f32,
    max_y: f32,
    distance: f32,
    color: macroquad::color::Color,
    label: String,
}

struct CameraManager {
    sensor: Sensor,
    texture: Texture2D,
    latest_image: Arc<Mutex<Option<CarlaImage>>>,
    transform: Transform,
    intrinsics: CameraIntrinsics,
}

impl CameraManager {
    fn new(world: &mut CarlaWorld, vehicle: &Vehicle) -> Result<Self> {
        let blueprint_library = world.blueprint_library();
        let camera_bp = blueprint_library
            .find("sensor.camera.rgb")
            .ok_or_else(|| anyhow::anyhow!("Camera blueprint not found"))?;

        let mut camera_bp = camera_bp;
        let _ = camera_bp.set_attribute("image_size_x", &CAMERA_WIDTH.to_string());
        let _ = camera_bp.set_attribute("image_size_y", &CAMERA_HEIGHT.to_string());
        let fov = 90.0;
        let _ = camera_bp.set_attribute("fov", &fov.to_string());

        // Spawn camera on hood of vehicle
        let mut camera_transform = Transform {
            location: Location::new(0.0, 0.0, 0.0),
            rotation: carla::geom::Rotation::new(0.0, 0.0, 0.0),
        };
        camera_transform.location.x = 1.6;
        camera_transform.location.z = 1.7;

        let camera = world.spawn_actor_attached(
            &camera_bp,
            &camera_transform,
            vehicle,
            carla::rpc::AttachmentType::Rigid,
        )?;

        let sensor =
            Sensor::try_from(camera).map_err(|_| anyhow::anyhow!("Failed to cast to Sensor"))?;

        let texture = Texture2D::from_rgba8(
            CAMERA_WIDTH as u16,
            CAMERA_HEIGHT as u16,
            &vec![0u8; (CAMERA_WIDTH * CAMERA_HEIGHT * 4) as usize],
        );
        texture.set_filter(FilterMode::Linear);

        let latest_image = Arc::new(Mutex::new(None));
        let latest_image_clone = Arc::clone(&latest_image);

        sensor.listen(move |data| {
            if let Ok(image) = CarlaImage::try_from(data) {
                if let Ok(mut img) = latest_image_clone.lock() {
                    *img = Some(image);
                }
            }
        });

        let intrinsics = CameraIntrinsics::new(fov, CAMERA_WIDTH, CAMERA_HEIGHT);

        Ok(Self {
            sensor,
            texture,
            latest_image,
            transform: camera_transform,
            intrinsics,
        })
    }

    fn update(&mut self, vehicle_transform: &Transform) -> bool {
        // Update camera world transform
        self.transform = self.compute_world_transform(vehicle_transform);

        if let Ok(mut img_opt) = self.latest_image.lock() {
            if let Some(image) = img_opt.take() {
                let raw_data = image.as_slice();
                let mut rgba_data = Vec::with_capacity((CAMERA_WIDTH * CAMERA_HEIGHT * 4) as usize);

                for color in raw_data {
                    rgba_data.push(color.r);
                    rgba_data.push(color.g);
                    rgba_data.push(color.b);
                    rgba_data.push(color.a);
                }

                self.texture =
                    Texture2D::from_rgba8(CAMERA_WIDTH as u16, CAMERA_HEIGHT as u16, &rgba_data);
                self.texture.set_filter(FilterMode::Linear);
                return true;
            }
        }
        false
    }

    fn compute_world_transform(&self, vehicle_transform: &Transform) -> Transform {
        // Compute camera's world transform from vehicle transform and relative offset
        let yaw = vehicle_transform.rotation.yaw.to_radians();
        let cos_yaw = yaw.cos();
        let sin_yaw = yaw.sin();

        // Rotate offset by vehicle yaw
        let rotated_x = self.transform.location.x * cos_yaw - self.transform.location.y * sin_yaw;
        let rotated_y = self.transform.location.x * sin_yaw + self.transform.location.y * cos_yaw;

        Transform {
            location: Location::new(
                vehicle_transform.location.x + rotated_x,
                vehicle_transform.location.y + rotated_y,
                vehicle_transform.location.z + self.transform.location.z,
            ),
            rotation: vehicle_transform.rotation,
        }
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

fn get_bounding_box_2d(
    actor: &Actor,
    camera_transform: &Transform,
    intrinsics: &CameraIntrinsics,
) -> Option<BoundingBox2D> {
    let actor_transform = actor.transform();
    let actor_location = &actor_transform.location;

    // Calculate distance
    let dx = actor_location.x - camera_transform.location.x;
    let dy = actor_location.y - camera_transform.location.y;
    let dz = actor_location.z - camera_transform.location.z;
    let distance = (dx * dx + dy * dy + dz * dz).sqrt();

    // Get bounding box (use version-gated API)
    #[cfg(carla_version_0916)]
    let bbox = actor.bounding_box();

    #[cfg(not(carla_version_0916))]
    let bbox = {
        // For CARLA 0.9.14/0.9.15, use approximate bounding box
        carla::geom::BoundingBox {
            transform: Transform {
                location: Location::new(0.0, 0.0, 0.0),
                rotation: carla::geom::Rotation::new(0.0, 0.0, 0.0),
            },
            extent: Vector3D::new(2.0, 1.0, 1.0), // Approximate vehicle size
        }
    };

    // Get 8 corners of bounding box in local space
    let extent = &bbox.extent;
    let corners = [
        Vector3D::new(-extent.x, -extent.y, -extent.z),
        Vector3D::new(extent.x, -extent.y, -extent.z),
        Vector3D::new(-extent.x, extent.y, -extent.z),
        Vector3D::new(extent.x, extent.y, -extent.z),
        Vector3D::new(-extent.x, -extent.y, extent.z),
        Vector3D::new(extent.x, -extent.y, extent.z),
        Vector3D::new(-extent.x, extent.y, extent.z),
        Vector3D::new(extent.x, extent.y, extent.z),
    ];

    // Transform corners to world space
    let yaw = actor_transform.rotation.yaw.to_radians();
    let cos_yaw = yaw.cos();
    let sin_yaw = yaw.sin();

    let mut projected_points = Vec::new();
    for corner in &corners {
        // Rotate corner by actor yaw
        let rotated_x = corner.x * cos_yaw - corner.y * sin_yaw;
        let rotated_y = corner.x * sin_yaw + corner.y * cos_yaw;

        let world_point = Vector3D::new(
            actor_location.x + rotated_x,
            actor_location.y + rotated_y,
            actor_location.z + corner.z + bbox.transform.location.z,
        );

        // Project to 2D
        if let Some((u, v)) = intrinsics.project_3d_to_2d(&world_point, camera_transform) {
            projected_points.push((u, v));
        }
    }

    // Need at least some points visible
    if projected_points.is_empty() {
        return None;
    }

    // Find min/max to create bounding rectangle
    let min_x = projected_points
        .iter()
        .map(|(x, _)| *x)
        .fold(f32::INFINITY, f32::min);
    let max_x = projected_points
        .iter()
        .map(|(x, _)| *x)
        .fold(f32::NEG_INFINITY, f32::max);
    let min_y = projected_points
        .iter()
        .map(|(_, y)| *y)
        .fold(f32::INFINITY, f32::min);
    let max_y = projected_points
        .iter()
        .map(|(_, y)| *y)
        .fold(f32::NEG_INFINITY, f32::max);

    // Clamp to image bounds
    let min_x = min_x.max(0.0).min(CAMERA_WIDTH as f32);
    let max_x = max_x.max(0.0).min(CAMERA_WIDTH as f32);
    let min_y = min_y.max(0.0).min(CAMERA_HEIGHT as f32);
    let max_y = max_y.max(0.0).min(CAMERA_HEIGHT as f32);

    // Determine color by actor type
    let type_id = actor.type_id();
    let (color, label_prefix) = if type_id.contains("vehicle") {
        (BLUE, "Vehicle")
    } else if type_id.contains("walker") {
        (RED, "Pedestrian")
    } else if type_id.contains("traffic.traffic_light") {
        (YELLOW, "Traffic Light")
    } else {
        (GREEN, "Other")
    };

    Some(BoundingBox2D {
        min_x,
        min_y,
        max_x,
        max_y,
        distance,
        color,
        label: format!("{} {:.1}m", label_prefix, distance),
    })
}

fn draw_bounding_boxes(boxes: &[BoundingBox2D]) {
    for bbox in boxes {
        // Draw rectangle
        draw_rectangle_lines(
            bbox.min_x,
            bbox.min_y,
            bbox.max_x - bbox.min_x,
            bbox.max_y - bbox.min_y,
            2.0,
            bbox.color,
        );

        // Draw label background
        let label_y = bbox.min_y - 20.0;
        if label_y > 0.0 {
            let text_dims = measure_text(&bbox.label, None, 16, 1.0);
            draw_rectangle(
                bbox.min_x,
                label_y,
                text_dims.width + 4.0,
                18.0,
                macroquad::color::Color::from_rgba(0, 0, 0, 180),
            );
            draw_text(&bbox.label, bbox.min_x + 2.0, label_y + 14.0, 16.0, WHITE);
        }
    }
}

fn draw_3d_bounding_box(world: &mut CarlaWorld, actor: &Actor, color: Color, life_time: f32) {
    let transform = actor.transform();

    #[cfg(carla_version_0916)]
    let bbox = actor.bounding_box();

    #[cfg(not(carla_version_0916))]
    let bbox = {
        carla::geom::BoundingBox {
            transform: Transform {
                location: Location::new(0.0, 0.0, 0.0),
                rotation: carla::geom::Rotation::new(0.0, 0.0, 0.0),
            },
            extent: Vector3D::new(2.0, 1.0, 1.0),
        }
    };

    let extent = &bbox.extent;
    let loc = &transform.location;
    let yaw = transform.rotation.yaw.to_radians();
    let cos_yaw = yaw.cos();
    let sin_yaw = yaw.sin();

    // 8 corners of bounding box
    let corners = [
        Vector3D::new(-extent.x, -extent.y, -extent.z),
        Vector3D::new(extent.x, -extent.y, -extent.z),
        Vector3D::new(-extent.x, extent.y, -extent.z),
        Vector3D::new(extent.x, extent.y, -extent.z),
        Vector3D::new(-extent.x, -extent.y, extent.z),
        Vector3D::new(extent.x, -extent.y, extent.z),
        Vector3D::new(-extent.x, extent.y, extent.z),
        Vector3D::new(extent.x, extent.y, extent.z),
    ];

    // Transform corners to world space
    let mut world_corners = Vec::new();
    for corner in &corners {
        let rotated_x = corner.x * cos_yaw - corner.y * sin_yaw;
        let rotated_y = corner.x * sin_yaw + corner.y * cos_yaw;

        world_corners.push(Location::new(
            loc.x + rotated_x,
            loc.y + rotated_y,
            loc.z + corner.z + bbox.transform.location.z,
        ));
    }

    // Draw 12 edges of the box
    let edges = [
        // Bottom face
        (0, 1),
        (1, 3),
        (3, 2),
        (2, 0),
        // Top face
        (4, 5),
        (5, 7),
        (7, 6),
        (6, 4),
        // Vertical edges
        (0, 4),
        (1, 5),
        (2, 6),
        (3, 7),
    ];

    for (i, j) in &edges {
        world.debug().draw_line(
            world_corners[*i],
            world_corners[*j],
            0.1,
            color,
            life_time,
            false,
        );
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
            macroquad::color::Color::from_rgba(0, 0, 0, 200),
        );

        let center_x = WINDOW_WIDTH as f32 / 2.0;
        let start_y = 100.0;
        let line_height = 30.0;

        let help_text = vec![
            ("BOUNDING BOXES VISUALIZATION", 30.0),
            ("", 20.0),
            (
                "Demonstrates 2D and 3D bounding box detection for computer vision.",
                20.0,
            ),
            ("", 20.0),
            ("CONTROLS:", 25.0),
            ("", 20.0),
            ("Space          Toggle 2D bounding boxes", 20.0),
            (
                "B              Toggle 3D bounding boxes (visible in spectator)",
                20.0,
            ),
            ("+/-            Adjust max distance filter", 20.0),
            ("F              Toggle distance filter on/off", 20.0),
            ("H              Toggle this help", 20.0),
            ("ESC / Q        Quit", 20.0),
            ("", 20.0),
            ("COLOR CODING:", 25.0),
            ("", 20.0),
            ("Blue           Vehicles", 20.0),
            ("Red            Pedestrians", 20.0),
            ("Yellow         Traffic Lights", 20.0),
            ("Green          Other actors", 20.0),
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
    println!("Bounding Boxes Visualization - Connecting to CARLA...");

    let client = Client::connect("localhost", 2000, 0);
    let mut world = client.world();

    println!("Connected! Setting up scene...");

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

    // Enable autopilot
    vehicle.set_autopilot(true);

    println!("Vehicle spawned: {}", vehicle.type_id());

    // Create camera
    let mut camera = CameraManager::new(&mut world, &vehicle)?;
    println!("Camera attached");

    // Create UI
    let mut help = HelpOverlay::new();

    // Settings
    let mut show_2d_boxes = true;
    let mut show_3d_boxes = true;
    let mut max_distance = DEFAULT_MAX_DISTANCE;
    let mut distance_filter_enabled = true;

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

        if is_key_pressed(KeyCode::Space) {
            show_2d_boxes = !show_2d_boxes;
            println!("2D boxes: {}", if show_2d_boxes { "ON" } else { "OFF" });
        }

        if is_key_pressed(KeyCode::B) {
            show_3d_boxes = !show_3d_boxes;
            println!("3D boxes: {}", if show_3d_boxes { "ON" } else { "OFF" });
        }

        if is_key_pressed(KeyCode::Equal) || is_key_pressed(KeyCode::KpAdd) {
            max_distance += 10.0;
            println!("Max distance: {:.0}m", max_distance);
        }

        if is_key_pressed(KeyCode::Minus) || is_key_pressed(KeyCode::KpSubtract) {
            max_distance = (max_distance - 10.0).max(10.0);
            println!("Max distance: {:.0}m", max_distance);
        }

        if is_key_pressed(KeyCode::F) {
            distance_filter_enabled = !distance_filter_enabled;
            println!(
                "Distance filter: {}",
                if distance_filter_enabled { "ON" } else { "OFF" }
            );
        }

        // Update camera
        let vehicle_transform = vehicle.transform();
        camera.update(&vehicle_transform);

        // Get all actors
        let actors = world.actors();
        let mut bounding_boxes = Vec::new();

        // Compute bounding boxes for visible actors
        for actor in actors.iter() {
            let type_id = actor.type_id();

            // Filter relevant actor types
            if !type_id.contains("vehicle")
                && !type_id.contains("walker")
                && !type_id.contains("traffic.traffic_light")
            {
                continue;
            }

            // Skip our own vehicle
            if actor.id() == vehicle.id() {
                continue;
            }

            // Compute 2D bounding box
            if let Some(bbox) = get_bounding_box_2d(&actor, &camera.transform, &camera.intrinsics) {
                // Apply distance filter
                if distance_filter_enabled && bbox.distance > max_distance {
                    continue;
                }

                bounding_boxes.push(bbox);

                // Draw 3D bounding box
                if show_3d_boxes {
                    let color = if type_id.contains("vehicle") {
                        Color {
                            r: 0,
                            g: 0,
                            b: 255,
                            a: 255,
                        }
                    } else if type_id.contains("walker") {
                        Color {
                            r: 255,
                            g: 0,
                            b: 0,
                            a: 255,
                        }
                    } else if type_id.contains("traffic.traffic_light") {
                        Color {
                            r: 255,
                            g: 255,
                            b: 0,
                            a: 255,
                        }
                    } else {
                        Color {
                            r: 0,
                            g: 255,
                            b: 0,
                            a: 255,
                        }
                    };

                    draw_3d_bounding_box(&mut world, &actor, color, 0.1);
                }
            }
        }

        // Render
        clear_background(BLACK);
        camera.render();

        if show_2d_boxes {
            draw_bounding_boxes(&bounding_boxes);
        }

        // Draw HUD
        let hud_text = format!(
            "2D Boxes: {} | 3D Boxes: {} | Distance: {:.0}m | Filter: {} | Actors: {}",
            if show_2d_boxes { "ON" } else { "OFF" },
            if show_3d_boxes { "ON" } else { "OFF" },
            max_distance,
            if distance_filter_enabled { "ON" } else { "OFF" },
            bounding_boxes.len()
        );
        let text_width = measure_text(&hud_text, None, 20, 1.0).width;
        draw_rectangle(
            10.0,
            10.0,
            text_width + 10.0,
            25.0,
            macroquad::color::Color::from_rgba(0, 0, 0, 180),
        );
        draw_text(&hud_text, 15.0, 28.0, 20.0, WHITE);

        help.render();

        next_frame().await;
    }

    println!("Cleaning up...");
    vehicle.destroy();
    camera.sensor.destroy();
    println!("Done!");

    Ok(())
}
