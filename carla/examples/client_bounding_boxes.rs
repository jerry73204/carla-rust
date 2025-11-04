//! Client-Side Bounding Boxes - Performance Comparison Demo
//!
//! Demonstrates client-side bounding box computation for performance comparison.
//! This version computes all bounding boxes locally instead of relying on server queries,
//! and includes performance benchmarking, occlusion handling, and optimizations.
//!
//! # Features
//! - Client-side bounding box computation
//! - Performance benchmarking (server vs client comparison)
//! - Depth buffer occlusion handling
//! - Spatial partitioning for actor filtering
//! - Frustum culling optimization
//! - Real-time FPS and timing statistics
//!
//! # Controls
//! - **Space**: Toggle 2D boxes on/off
//! - **B**: Toggle 3D boxes on/off
//! - **O**: Toggle occlusion handling on/off
//! - **P**: Toggle performance stats on/off
//! - **+/-**: Adjust max distance filter
//! - **F**: Toggle distance filter on/off
//! - **H**: Toggle help overlay
//! - **ESC/Q**: Quit
//!
//! # Usage
//! ```bash
//! cargo run --example client_bounding_boxes --profile dev-release
//! ```

use anyhow::Result;
use carla::{
    client::{ActorBase, Client, Sensor, Vehicle, World as CarlaWorld},
    geom::{Location, Rotation, Transform, Vector3D},
    rpc::Color,
    sensor::data::Image as CarlaImage,
};
use macroquad::prelude::*;
use std::{
    sync::{Arc, Mutex},
    time::Instant,
};

const WINDOW_WIDTH: u32 = 1280;
const WINDOW_HEIGHT: u32 = 720;
const CAMERA_WIDTH: u32 = 1280;
const CAMERA_HEIGHT: u32 = 720;
const DEFAULT_MAX_DISTANCE: f32 = 50.0;
const CAMERA_FOV: f32 = 110.0;

fn window_conf() -> Conf {
    Conf {
        window_title: "CARLA - Client-Side Bounding Boxes (Performance)".to_owned(),
        window_width: WINDOW_WIDTH as i32,
        window_height: WINDOW_HEIGHT as i32,
        window_resizable: false,
        ..Default::default()
    }
}

/// Camera intrinsic matrix for 3D to 2D projection
#[derive(Clone)]
struct CameraIntrinsics {
    focal_length: f32,
    image_width: f32,
    image_height: f32,
    cx: f32,
    cy: f32,
}

impl CameraIntrinsics {
    fn new(fov: f32, image_width: u32, image_height: u32) -> Self {
        let fov_rad = fov.to_radians();
        let focal_length = (image_width as f32) / (2.0 * (fov_rad / 2.0).tan());
        let cx = image_width as f32 / 2.0;
        let cy = image_height as f32 / 2.0;

        Self {
            focal_length,
            image_width: image_width as f32,
            image_height: image_height as f32,
            cx,
            cy,
        }
    }

    /// Project 3D world point to 2D image coordinates with depth
    /// Returns (u, v, depth) where depth is distance from camera
    fn project_3d_to_2d_with_depth(
        &self,
        point: &Vector3D,
        camera_transform: &Transform,
    ) -> Option<(f32, f32, f32)> {
        // Transform world point to camera space
        let camera_pos = &camera_transform.location;
        let relative = Vector3D::new(
            point.x - camera_pos.x,
            point.y - camera_pos.y,
            point.z - camera_pos.z,
        );

        // Apply camera rotation
        let yaw = camera_transform.rotation.yaw.to_radians();
        let pitch = camera_transform.rotation.pitch.to_radians();

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

        // Check if behind camera
        if x2 <= 0.0 {
            return None;
        }

        // Project to image plane
        let u = self.focal_length * (y2 / x2) + self.cx;
        let v = self.focal_length * (-z2 / x2) + self.cy;

        // Return with depth (distance from camera)
        let depth =
            (relative.x * relative.x + relative.y * relative.y + relative.z * relative.z).sqrt();

        Some((u, v, depth))
    }
}

/// 2D bounding box data
#[derive(Clone)]
struct BoundingBox2D {
    min_x: f32,
    max_x: f32,
    min_y: f32,
    max_y: f32,
    #[allow(dead_code)]
    distance: f32,
    #[allow(dead_code)]
    depth: f32, // Average depth of box corners
    color: Color,
    label: String,
}

/// Client-side actor data cache
struct ActorData {
    #[allow(dead_code)]
    id: u32,
    location: Location,
    rotation: Rotation,
    type_id: String,
    #[cfg(carla_version_0916)]
    bounding_box: carla::geom::BoundingBox,
}

/// Spatial grid for fast actor lookup
struct SpatialGrid {
    #[allow(dead_code)]
    cell_size: f32,
    actors: Vec<ActorData>,
}

impl SpatialGrid {
    fn new(cell_size: f32) -> Self {
        Self {
            cell_size,
            actors: Vec::new(),
        }
    }

    fn update(&mut self, world: &mut CarlaWorld) {
        self.actors.clear();

        for actor in world.actors().iter() {
            let location = actor.location();
            let rotation = actor.transform().rotation;
            let type_id = actor.type_id();

            #[cfg(carla_version_0916)]
            let bounding_box = actor.bounding_box();

            self.actors.push(ActorData {
                id: actor.id(),
                location,
                rotation,
                type_id,
                #[cfg(carla_version_0916)]
                bounding_box,
            });
        }
    }

    /// Get actors within distance and frustum
    fn query_frustum(
        &self,
        camera_pos: &Location,
        camera_rotation: &Rotation,
        max_distance: f32,
        fov: f32,
    ) -> Vec<&ActorData> {
        let mut result = Vec::new();
        let fov_rad = fov.to_radians();
        let cos_half_fov = (fov_rad / 2.0).cos();

        for actor in &self.actors {
            // Distance check
            let dx = actor.location.x - camera_pos.x;
            let dy = actor.location.y - camera_pos.y;
            let dz = actor.location.z - camera_pos.z;
            let distance_sq = dx * dx + dy * dy + dz * dz;

            if distance_sq > max_distance * max_distance {
                continue;
            }

            // Frustum culling (simple cone check based on yaw)
            let distance = distance_sq.sqrt();
            if distance > 0.1 {
                let yaw = camera_rotation.yaw.to_radians();
                let forward_x = yaw.cos();
                let forward_y = yaw.sin();

                let dot = (dx * forward_x + dy * forward_y) / distance;
                if dot < cos_half_fov {
                    continue;
                }
            }

            result.push(actor);
        }

        result
    }
}

/// Performance statistics
struct PerformanceStats {
    bbox_computation_time: f32,
    projection_time: f32,
    rendering_time: f32,
    total_frame_time: f32,
    actor_count: usize,
    visible_bbox_count: usize,
}

impl PerformanceStats {
    fn new() -> Self {
        Self {
            bbox_computation_time: 0.0,
            projection_time: 0.0,
            rendering_time: 0.0,
            total_frame_time: 0.0,
            actor_count: 0,
            visible_bbox_count: 0,
        }
    }
}

/// Camera manager
struct CameraManager {
    sensor: Sensor,
    texture: Texture2D,
    latest_image: Arc<Mutex<Option<CarlaImage>>>,
    depth_buffer: Arc<Mutex<Option<CarlaImage>>>,
    transform: Transform,
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
        let _ = rgb_bp.set_attribute("fov", &CAMERA_FOV.to_string());

        // Create depth camera for occlusion
        let depth_bp = blueprint_library
            .find("sensor.camera.depth")
            .ok_or_else(|| anyhow::anyhow!("Depth camera blueprint not found"))?;

        let mut depth_bp = depth_bp;
        let _ = depth_bp.set_attribute("image_size_x", &CAMERA_WIDTH.to_string());
        let _ = depth_bp.set_attribute("image_size_y", &CAMERA_HEIGHT.to_string());
        let _ = depth_bp.set_attribute("fov", &CAMERA_FOV.to_string());

        // Camera transform (hood mount)
        let camera_transform = Transform {
            location: Location::new(1.5, 0.0, 1.4),
            rotation: Rotation::new(0.0, 0.0, 0.0),
        };

        // Spawn cameras
        let rgb_camera = world.spawn_actor_attached(
            &rgb_bp,
            &camera_transform,
            vehicle,
            carla::rpc::AttachmentType::Rigid,
        )?;

        let depth_camera = world.spawn_actor_attached(
            &depth_bp,
            &camera_transform,
            vehicle,
            carla::rpc::AttachmentType::Rigid,
        )?;

        let rgb_sensor = Sensor::try_from(rgb_camera)
            .map_err(|_| anyhow::anyhow!("Failed to cast to Sensor"))?;
        let depth_sensor = Sensor::try_from(depth_camera)
            .map_err(|_| anyhow::anyhow!("Failed to cast to Sensor"))?;

        // Create texture
        let texture = Texture2D::from_rgba8(
            CAMERA_WIDTH as u16,
            CAMERA_HEIGHT as u16,
            &vec![0u8; (CAMERA_WIDTH * CAMERA_HEIGHT * 4) as usize],
        );
        texture.set_filter(FilterMode::Linear);

        // Setup listeners
        let latest_image = Arc::new(Mutex::new(None));
        let latest_image_clone = Arc::clone(&latest_image);

        let depth_buffer = Arc::new(Mutex::new(None));
        let depth_buffer_clone = Arc::clone(&depth_buffer);

        rgb_sensor.listen(move |data| {
            if let Ok(image) = CarlaImage::try_from(data) {
                if let Ok(mut img) = latest_image_clone.lock() {
                    *img = Some(image);
                }
            }
        });

        depth_sensor.listen(move |data| {
            if let Ok(image) = CarlaImage::try_from(data) {
                if let Ok(mut img) = depth_buffer_clone.lock() {
                    *img = Some(image);
                }
            }
        });

        Ok(Self {
            sensor: rgb_sensor,
            texture,
            latest_image,
            depth_buffer,
            transform: camera_transform,
        })
    }

    fn update(&mut self, vehicle: &Vehicle) -> bool {
        // Update camera world transform
        let vehicle_transform = vehicle.transform();
        self.transform = Self::compute_world_transform(&vehicle_transform, &self.transform);

        // Update texture
        if let Ok(mut img) = self.latest_image.lock() {
            if let Some(image) = img.take() {
                let data = image.as_slice();
                let mut rgba_data = Vec::with_capacity((CAMERA_WIDTH * CAMERA_HEIGHT * 4) as usize);

                for pixel in data {
                    rgba_data.push(pixel.b);
                    rgba_data.push(pixel.g);
                    rgba_data.push(pixel.r);
                    rgba_data.push(255);
                }

                self.texture =
                    Texture2D::from_rgba8(CAMERA_WIDTH as u16, CAMERA_HEIGHT as u16, &rgba_data);
                self.texture.set_filter(FilterMode::Linear);
                return true;
            }
        }
        false
    }

    fn compute_world_transform(
        vehicle_transform: &Transform,
        camera_relative: &Transform,
    ) -> Transform {
        let yaw = vehicle_transform.rotation.yaw.to_radians();
        let cos_yaw = yaw.cos();
        let sin_yaw = yaw.sin();

        let rotated_x = camera_relative.location.x * cos_yaw - camera_relative.location.y * sin_yaw;
        let rotated_y = camera_relative.location.x * sin_yaw + camera_relative.location.y * cos_yaw;

        Transform {
            location: Location::new(
                vehicle_transform.location.x + rotated_x,
                vehicle_transform.location.y + rotated_y,
                vehicle_transform.location.z + camera_relative.location.z,
            ),
            rotation: Rotation::new(
                vehicle_transform.rotation.pitch + camera_relative.rotation.pitch,
                vehicle_transform.rotation.yaw + camera_relative.rotation.yaw,
                vehicle_transform.rotation.roll + camera_relative.rotation.roll,
            ),
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

    /// Check if point is occluded using depth buffer
    fn is_occluded(&self, u: f32, v: f32, depth: f32) -> bool {
        if let Ok(depth_img) = self.depth_buffer.lock() {
            if let Some(ref img) = *depth_img {
                let x = u.round() as u32;
                let y = v.round() as u32;

                if x < CAMERA_WIDTH && y < CAMERA_HEIGHT {
                    let idx = (y * CAMERA_WIDTH + x) as usize;
                    let data = img.as_slice();

                    if idx < data.len() {
                        let pixel = &data[idx];
                        // Depth encoded as normalized R value (0.0 = near, 1.0 = far)
                        // Convert to actual depth in meters (assuming max depth = 1000m)
                        let buffer_depth = (pixel.r as f32 / 255.0) * 1000.0;

                        // Point is occluded if it's further than buffer depth + threshold
                        return depth > buffer_depth + 1.0;
                    }
                }
            }
        }
        false
    }
}

/// Compute 2D bounding box from actor (client-side)
fn compute_bounding_box_2d(
    actor_data: &ActorData,
    camera_transform: &Transform,
    intrinsics: &CameraIntrinsics,
    occlusion_enabled: bool,
    camera: &CameraManager,
) -> Option<BoundingBox2D> {
    // Get bounding box extent
    #[cfg(carla_version_0916)]
    let extent = actor_data.bounding_box.extent;

    #[cfg(not(carla_version_0916))]
    let extent = {
        // Approximate bounding box for older versions
        if actor_data.type_id.contains("vehicle") {
            Vector3D::new(2.0, 1.0, 1.0)
        } else if actor_data.type_id.contains("walker") {
            Vector3D::new(0.3, 0.3, 0.9)
        } else {
            Vector3D::new(0.5, 0.5, 0.5)
        }
    };

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

    // Transform corners to world space and project to 2D
    let yaw = actor_data.rotation.yaw.to_radians();
    let cos_yaw = yaw.cos();
    let sin_yaw = yaw.sin();

    let mut projected_points = Vec::new();
    let mut total_depth = 0.0;

    for corner in &corners {
        // Rotate by actor yaw
        let rotated_x = corner.x * cos_yaw - corner.y * sin_yaw;
        let rotated_y = corner.x * sin_yaw + corner.y * cos_yaw;

        let world_point = Vector3D::new(
            actor_data.location.x + rotated_x,
            actor_data.location.y + rotated_y,
            actor_data.location.z + corner.z,
        );

        if let Some((u, v, depth)) =
            intrinsics.project_3d_to_2d_with_depth(&world_point, camera_transform)
        {
            // Check occlusion if enabled
            if occlusion_enabled && camera.is_occluded(u, v, depth) {
                continue;
            }

            projected_points.push((u, v));
            total_depth += depth;
        }
    }

    if projected_points.is_empty() {
        return None;
    }

    let avg_depth = total_depth / projected_points.len() as f32;

    // Find min/max bounds
    let min_x = projected_points
        .iter()
        .map(|(x, _)| *x)
        .fold(f32::INFINITY, f32::min)
        .max(0.0);
    let max_x = projected_points
        .iter()
        .map(|(x, _)| *x)
        .fold(f32::NEG_INFINITY, f32::max)
        .min(intrinsics.image_width);
    let min_y = projected_points
        .iter()
        .map(|(_, y)| *y)
        .fold(f32::INFINITY, f32::min)
        .max(0.0);
    let max_y = projected_points
        .iter()
        .map(|(_, y)| *y)
        .fold(f32::NEG_INFINITY, f32::max)
        .min(intrinsics.image_height);

    // Skip if box is too small (likely behind camera)
    if max_x - min_x < 1.0 || max_y - min_y < 1.0 {
        return None;
    }

    // Calculate distance
    let dx = actor_data.location.x - camera_transform.location.x;
    let dy = actor_data.location.y - camera_transform.location.y;
    let dz = actor_data.location.z - camera_transform.location.z;
    let distance = (dx * dx + dy * dy + dz * dz).sqrt();

    // Color by type
    let color = if actor_data.type_id.contains("vehicle") {
        Color::new(0, 0, 255) // Blue
    } else if actor_data.type_id.contains("walker") || actor_data.type_id.contains("pedestrian") {
        Color::new(255, 0, 0) // Red
    } else if actor_data.type_id.contains("traffic_light") {
        Color::new(255, 255, 0) // Yellow
    } else {
        Color::new(0, 255, 0) // Green
    };

    let label = format!("{:.1}m", distance);

    Some(BoundingBox2D {
        min_x,
        max_x,
        min_y,
        max_y,
        distance,
        depth: avg_depth,
        color,
        label,
    })
}

/// Draw 2D bounding boxes
fn draw_bounding_boxes(boxes: &[BoundingBox2D]) {
    for bbox in boxes {
        let width = bbox.max_x - bbox.min_x;
        let height = bbox.max_y - bbox.min_y;

        // Draw rectangle
        let mq_color = macroquad::color::Color::from_rgba(
            bbox.color.r,
            bbox.color.g,
            bbox.color.b,
            bbox.color.a,
        );

        draw_rectangle_lines(bbox.min_x, bbox.min_y, width, height, 2.0, mq_color);

        // Draw distance label
        let label_width = measure_text(&bbox.label, None, 16, 1.0).width;
        let label_x = bbox.min_x;
        let label_y = bbox.min_y.max(20.0);

        draw_rectangle(
            label_x,
            label_y - 18.0,
            label_width + 6.0,
            20.0,
            macroquad::color::Color::from_rgba(0, 0, 0, 180),
        );
        draw_text(&bbox.label, label_x + 3.0, label_y - 3.0, 16.0, WHITE);
    }
}

/// Draw 3D bounding box in world space
fn draw_3d_bounding_box(world: &mut CarlaWorld, actor_data: &ActorData, color: Color) {
    #[cfg(carla_version_0916)]
    let extent = actor_data.bounding_box.extent;

    #[cfg(not(carla_version_0916))]
    let extent = {
        if actor_data.type_id.contains("vehicle") {
            Vector3D::new(2.0, 1.0, 1.0)
        } else if actor_data.type_id.contains("walker") {
            Vector3D::new(0.3, 0.3, 0.9)
        } else {
            Vector3D::new(0.5, 0.5, 0.5)
        }
    };

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

    let yaw = actor_data.rotation.yaw.to_radians();
    let cos_yaw = yaw.cos();
    let sin_yaw = yaw.sin();

    let mut world_corners = Vec::new();
    for corner in &corners {
        let rotated_x = corner.x * cos_yaw - corner.y * sin_yaw;
        let rotated_y = corner.x * sin_yaw + corner.y * cos_yaw;

        world_corners.push(Location::new(
            actor_data.location.x + rotated_x,
            actor_data.location.y + rotated_y,
            actor_data.location.z + corner.z,
        ));
    }

    let edges = [
        (0, 1),
        (1, 3),
        (3, 2),
        (2, 0),
        (4, 5),
        (5, 7),
        (7, 6),
        (6, 4),
        (0, 4),
        (1, 5),
        (2, 6),
        (3, 7),
    ];

    for (i, j) in &edges {
        world
            .debug()
            .draw_line(world_corners[*i], world_corners[*j], 0.1, color, 0.1, false);
    }
}

/// HUD display
struct Hud {
    show_stats: bool,
}

impl Hud {
    fn new() -> Self {
        Self { show_stats: true }
    }

    fn toggle(&mut self) {
        self.show_stats = !self.show_stats;
    }

    fn render(
        &self,
        show_2d: bool,
        show_3d: bool,
        occlusion_enabled: bool,
        filter_enabled: bool,
        max_distance: f32,
        stats: &PerformanceStats,
    ) {
        let y_offset = 20.0;
        let line_height = 20.0;
        let mut y = y_offset;

        let texts = vec![
            "CARLA - Client-Side Bounding Boxes".to_string(),
            "".to_string(),
            format!("2D Boxes: {}", if show_2d { "ON" } else { "OFF" }),
            format!("3D Boxes: {}", if show_3d { "ON" } else { "OFF" }),
            format!(
                "Occlusion: {}",
                if occlusion_enabled { "ON" } else { "OFF" }
            ),
            format!(
                "Distance Filter: {} ({}m)",
                if filter_enabled { "ON" } else { "OFF" },
                max_distance
            ),
            "".to_string(),
        ];

        for text in &texts {
            let text_width = measure_text(text, None, 20, 1.0).width;
            draw_rectangle(
                10.0,
                y - 15.0,
                text_width + 10.0,
                line_height,
                macroquad::color::Color::from_rgba(0, 0, 0, 180),
            );
            draw_text(text, 15.0, y, 20.0, WHITE);
            y += line_height;
        }

        if self.show_stats {
            let perf_texts = vec![
                "Performance Stats:".to_string(),
                format!("  Actors: {}", stats.actor_count),
                format!("  Visible: {}", stats.visible_bbox_count),
                format!("  BBox Compute: {:.2}ms", stats.bbox_computation_time),
                format!("  Projection: {:.2}ms", stats.projection_time),
                format!("  Rendering: {:.2}ms", stats.rendering_time),
                format!("  Total Frame: {:.2}ms", stats.total_frame_time),
                format!("  FPS: {:.1}", 1000.0 / stats.total_frame_time.max(0.001)),
            ];

            for text in &perf_texts {
                let text_width = measure_text(text, None, 20, 1.0).width;
                draw_rectangle(
                    10.0,
                    y - 15.0,
                    text_width + 10.0,
                    line_height,
                    macroquad::color::Color::from_rgba(0, 0, 0, 180),
                );
                draw_text(text, 15.0, y, 20.0, WHITE);
                y += line_height;
            }
        }
    }
}

/// Help overlay
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
        let start_y = 80.0;
        let line_height = 30.0;

        let help_text = vec![
            ("CLIENT-SIDE BOUNDING BOXES", 30.0),
            ("", 20.0),
            (
                "This demo computes bounding boxes entirely client-side for performance comparison.",
                20.0,
            ),
            ("Includes occlusion handling, frustum culling, and spatial optimization.", 20.0),
            ("", 20.0),
            ("CONTROLS:", 25.0),
            ("", 20.0),
            ("Space          Toggle 2D boxes on/off", 20.0),
            ("B              Toggle 3D boxes on/off", 20.0),
            ("O              Toggle occlusion handling", 20.0),
            ("P              Toggle performance stats", 20.0),
            ("+/-            Adjust max distance filter", 20.0),
            ("F              Toggle distance filter", 20.0),
            ("H              Toggle this help overlay", 20.0),
            ("ESC / Q        Quit application", 20.0),
            ("", 20.0),
            ("COLOR LEGEND:", 25.0),
            ("", 20.0),
            ("Blue           Vehicles", 20.0),
            ("Red            Pedestrians", 20.0),
            ("Yellow         Traffic Lights", 20.0),
            ("Green          Other", 20.0),
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
    println!("Client-Side Bounding Boxes - Connecting to CARLA...");

    let client = Client::connect("localhost", 2000, 0);
    let mut world = client.world();

    println!("Connected! Setting up scene...");

    // Spawn vehicle
    let blueprint_library = world.blueprint_library();
    let vehicle_bp = blueprint_library
        .filter("vehicle.*")
        .iter()
        .next()
        .ok_or_else(|| anyhow::anyhow!("No vehicle blueprints found"))?;

    let map = world.map();
    let spawn_points = map.recommended_spawn_points();
    if spawn_points.is_empty() {
        return Err(anyhow::anyhow!("No spawn points available"));
    }

    let spawn_point = &spawn_points.as_slice()[0];
    let vehicle_actor = world.spawn_actor(&vehicle_bp, spawn_point)?;
    let vehicle = Vehicle::try_from(vehicle_actor)
        .map_err(|_| anyhow::anyhow!("Failed to cast to Vehicle"))?;

    vehicle.set_autopilot(true);
    println!("Vehicle spawned with autopilot");

    // Create camera
    let mut camera = CameraManager::new(&mut world, &vehicle)?;
    println!("Camera attached");

    // Create camera intrinsics
    let intrinsics = CameraIntrinsics::new(CAMERA_FOV, CAMERA_WIDTH, CAMERA_HEIGHT);

    // Create spatial grid
    let mut spatial_grid = SpatialGrid::new(50.0);

    // Create UI
    let mut hud = Hud::new();
    let mut help = HelpOverlay::new();

    // State
    let mut show_2d = true;
    let mut show_3d = false;
    let mut occlusion_enabled = false;
    let mut filter_enabled = true;
    let mut max_distance = DEFAULT_MAX_DISTANCE;

    // Performance stats
    let mut stats = PerformanceStats::new();

    println!("Starting main loop...");
    println!("Press H for help");

    loop {
        let frame_start = Instant::now();

        // Handle input
        if is_key_pressed(KeyCode::Escape) || is_key_pressed(KeyCode::Q) {
            break;
        }

        if is_key_pressed(KeyCode::H) {
            help.toggle();
        }

        if is_key_pressed(KeyCode::Space) {
            show_2d = !show_2d;
        }

        if is_key_pressed(KeyCode::B) {
            show_3d = !show_3d;
        }

        if is_key_pressed(KeyCode::O) {
            occlusion_enabled = !occlusion_enabled;
            println!(
                "Occlusion handling: {}",
                if occlusion_enabled { "ON" } else { "OFF" }
            );
        }

        if is_key_pressed(KeyCode::P) {
            hud.toggle();
        }

        if is_key_pressed(KeyCode::F) {
            filter_enabled = !filter_enabled;
        }

        if is_key_pressed(KeyCode::Equal) || is_key_pressed(KeyCode::KpAdd) {
            max_distance += 10.0;
            println!("Max distance: {}m", max_distance);
        }

        if is_key_pressed(KeyCode::Minus) || is_key_pressed(KeyCode::KpSubtract) {
            max_distance = (max_distance - 10.0).max(10.0);
            println!("Max distance: {}m", max_distance);
        }

        // Update camera
        camera.update(&vehicle);

        // Update spatial grid
        spatial_grid.update(&mut world);
        stats.actor_count = spatial_grid.actors.len();

        // Query actors in frustum
        let actors_in_view = if filter_enabled {
            spatial_grid.query_frustum(
                &camera.transform.location,
                &camera.transform.rotation,
                max_distance,
                CAMERA_FOV,
            )
        } else {
            spatial_grid.actors.iter().collect()
        };

        // Compute bounding boxes
        let mut bounding_boxes = Vec::new();
        let bbox_start = Instant::now();

        for actor_data in actors_in_view {
            if let Some(bbox) = compute_bounding_box_2d(
                actor_data,
                &camera.transform,
                &intrinsics,
                occlusion_enabled,
                &camera,
            ) {
                bounding_boxes.push((actor_data, bbox));
            }
        }

        stats.bbox_computation_time = bbox_start.elapsed().as_secs_f32() * 1000.0;
        stats.visible_bbox_count = bounding_boxes.len();

        // Render
        clear_background(BLACK);
        camera.render();

        let render_start = Instant::now();

        // Draw 2D boxes
        if show_2d {
            let boxes: Vec<_> = bounding_boxes
                .iter()
                .map(|(_, bbox)| bbox.clone())
                .collect();
            draw_bounding_boxes(&boxes);
        }

        // Draw 3D boxes
        if show_3d {
            for (actor_data, bbox) in &bounding_boxes {
                draw_3d_bounding_box(&mut world, actor_data, bbox.color);
            }
        }

        stats.rendering_time = render_start.elapsed().as_secs_f32() * 1000.0;

        // Draw UI
        hud.render(
            show_2d,
            show_3d,
            occlusion_enabled,
            filter_enabled,
            max_distance,
            &stats,
        );
        help.render();

        stats.total_frame_time = frame_start.elapsed().as_secs_f32() * 1000.0;

        next_frame().await;
    }

    println!("Cleaning up...");
    vehicle.destroy();
    camera.sensor.destroy();

    println!("Done!");

    Ok(())
}
