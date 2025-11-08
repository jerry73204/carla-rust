//! Draw walker skeleton visualization.
//!
//! This example demonstrates:
//! - Spawning multiple walkers with AI navigation
//! - Retrieving bone transforms in real-time
//! - Projecting 3D bone positions to 2D camera space
//! - Drawing skeleton overlay on camera feed
//! - Drawing 3D skeleton in world space
//! - Walker selection and bone name labels
//!
//! Controls:
//! - S: Toggle skeleton overlay
//! - B: Toggle bone name labels
//! - 3: Toggle 3D skeleton in world
//! - Tab: Cycle through walkers
//! - ESC: Quit
//!
//! Run with: `cargo run --example draw_skeleton --profile dev-release`

use carla::{
    client::{Client, Sensor, Walker, World as CarlaWorld},
    geom::{Location, Rotation, Transform},
    rpc::{Color, WalkerBoneControlOut},
    sensor::data::Image as CarlaImage,
};
use macroquad::prelude::*;
use std::sync::{Arc, Mutex};

const CAMERA_WIDTH: i32 = 1280;
const CAMERA_HEIGHT: i32 = 720;
const NUM_WALKERS: usize = 5;

/// Bone connections defining the skeleton hierarchy.
/// Based on CARLA's skeletal structure.
const BONE_CONNECTIONS: &[(&str, &str)] = &[
    // Spine
    ("crl_root", "crl_hips__C"),
    ("crl_hips__C", "crl_spine__C"),
    ("crl_spine__C", "crl_spine01__C"),
    ("crl_spine01__C", "crl_shoulder__L"),
    ("crl_spine01__C", "crl_shoulder__R"),
    ("crl_spine01__C", "crl_neck__C"),
    ("crl_neck__C", "crl_Head__C"),
    // Left arm
    ("crl_shoulder__L", "crl_arm__L"),
    ("crl_arm__L", "crl_foreArm__L"),
    ("crl_foreArm__L", "crl_hand__L"),
    // Right arm
    ("crl_shoulder__R", "crl_arm__R"),
    ("crl_arm__R", "crl_foreArm__R"),
    ("crl_foreArm__R", "crl_hand__R"),
    // Left leg
    ("crl_hips__C", "crl_thigh__L"),
    ("crl_thigh__L", "crl_leg__L"),
    ("crl_leg__L", "crl_foot__L"),
    ("crl_foot__L", "crl_toes__L"),
    // Right leg
    ("crl_hips__C", "crl_thigh__R"),
    ("crl_thigh__R", "crl_leg__R"),
    ("crl_leg__R", "crl_foot__R"),
    ("crl_foot__R", "crl_toes__R"),
];

/// Camera intrinsics for 3D to 2D projection
#[derive(Debug, Clone)]
struct CameraIntrinsics {
    focal_length: f32,
    image_width: f32,
    image_height: f32,
}

impl CameraIntrinsics {
    fn new(fov: f32, width: i32, height: i32) -> Self {
        let image_width = width as f32;
        let image_height = height as f32;
        let focal_length = image_width / (2.0 * (fov * std::f32::consts::PI / 360.0).tan());

        Self {
            focal_length,
            image_width,
            image_height,
        }
    }

    /// Project a 3D world point to 2D image coordinates.
    /// Returns None if the point is behind the camera.
    fn project_3d_to_2d(
        &self,
        world_point: &Location,
        camera_transform: &Transform,
    ) -> Option<(f32, f32)> {
        // Transform world point to camera space
        let relative = Location::new(
            world_point.x - camera_transform.location.x,
            world_point.y - camera_transform.location.y,
            world_point.z - camera_transform.location.z,
        );

        // Apply camera rotation (yaw and pitch)
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

        // Check if point is behind camera
        if x2 <= 0.0 {
            return None;
        }

        // Project to image plane
        let cx = self.image_width / 2.0;
        let cy = self.image_height / 2.0;

        let u = self.focal_length * (y2 / x2) + cx;
        let v = self.focal_length * (-z2 / x2) + cy;

        Some((u, v))
    }
}

struct CameraManager {
    #[allow(dead_code)] // Sensor must be kept alive to continue receiving data
    sensor: Sensor,
    texture: Texture2D,
    latest_image: Arc<Mutex<Option<CarlaImage>>>,
    transform: Transform,
    intrinsics: CameraIntrinsics,
}

impl CameraManager {
    fn new(world: &mut CarlaWorld) -> anyhow::Result<Self> {
        let blueprint_library = world.blueprint_library();
        let camera_bp = blueprint_library
            .find("sensor.camera.rgb")
            .ok_or_else(|| anyhow::anyhow!("Camera blueprint not found"))?;

        let mut camera_bp = camera_bp;
        let _ = camera_bp.set_attribute("image_size_x", &CAMERA_WIDTH.to_string());
        let _ = camera_bp.set_attribute("image_size_y", &CAMERA_HEIGHT.to_string());
        let fov = 90.0;
        let _ = camera_bp.set_attribute("fov", &fov.to_string());

        // Spawn camera at a good vantage point
        let spawn_points = world.map().recommended_spawn_points();
        let spawn_point = spawn_points
            .get(0)
            .ok_or_else(|| anyhow::anyhow!("No spawn points available"))?;

        let camera_transform = Transform {
            location: Location::new(
                spawn_point.location.x,
                spawn_point.location.y,
                spawn_point.location.z + 2.0, // Elevated view
            ),
            rotation: Rotation::new(-15.0, spawn_point.rotation.yaw, 0.0), // Look down slightly
        };

        let camera = world.spawn_actor(&camera_bp, &camera_transform)?;
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

    fn update(&mut self) -> bool {
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
}

struct WalkerInfo {
    walker: Walker,
    color: macroquad::prelude::Color,
}

struct SkeletonVisualizer {
    world: CarlaWorld,
    camera: CameraManager,
    walkers: Vec<WalkerInfo>,
    selected_walker_idx: usize,
    show_skeleton: bool,
    show_labels: bool,
    show_3d_skeleton: bool,
}

impl SkeletonVisualizer {
    fn new() -> anyhow::Result<Self> {
        let client = Client::connect("127.0.0.1", 2000, None);
        let mut world = client.world();

        println!("Connected to CARLA server");

        // Spawn camera
        let camera = CameraManager::new(&mut world)?;
        println!("Camera spawned");

        // Spawn walkers
        let walkers = Self::spawn_walkers(&mut world)?;
        println!("Spawned {} walkers", walkers.len());

        Ok(Self {
            world,
            camera,
            walkers,
            selected_walker_idx: 0,
            show_skeleton: true,
            show_labels: true,
            show_3d_skeleton: true,
        })
    }

    fn spawn_walkers(world: &mut CarlaWorld) -> anyhow::Result<Vec<WalkerInfo>> {
        let blueprint_library = world.blueprint_library();
        let walker_blueprints = blueprint_library.filter("walker.pedestrian.*");

        if walker_blueprints.is_empty() {
            return Err(anyhow::anyhow!("No walker blueprints found"));
        }

        let spawn_points = world.map().recommended_spawn_points();
        if spawn_points.is_empty() {
            return Err(anyhow::anyhow!("No spawn points available"));
        }

        let colors = [
            macroquad::prelude::Color::new(1.0, 0.0, 0.0, 1.0), // Red
            macroquad::prelude::Color::new(0.0, 1.0, 0.0, 1.0), // Green
            macroquad::prelude::Color::new(0.0, 0.0, 1.0, 1.0), // Blue
            macroquad::prelude::Color::new(1.0, 1.0, 0.0, 1.0), // Yellow
            macroquad::prelude::Color::new(1.0, 0.0, 1.0, 1.0), // Magenta
        ];

        let mut walkers = Vec::new();

        for i in 0..NUM_WALKERS.min(spawn_points.len()) {
            let bp_list = walker_blueprints.iter().collect::<Vec<_>>();
            let bp = &bp_list[i % bp_list.len()];
            let spawn_point = spawn_points.get(i).unwrap();

            // Spawn walker slightly offset from spawn point
            let walker_transform = Transform {
                location: Location::new(
                    spawn_point.location.x + (i as f32 * 3.0) - 6.0, // Spread walkers out
                    spawn_point.location.y,
                    spawn_point.location.z + 0.5,
                ),
                rotation: Rotation::new(0.0, spawn_point.rotation.yaw, 0.0),
            };

            match world.spawn_actor(bp, &walker_transform) {
                Ok(actor) => {
                    if let Ok(walker) = Walker::try_from(actor) {
                        // Enable AI control
                        let ai_bp = blueprint_library
                            .find("controller.ai.walker")
                            .ok_or_else(|| anyhow::anyhow!("AI controller blueprint not found"))?;

                        if let Ok(controller_actor) = world.spawn_actor(&ai_bp, &walker_transform) {
                            if let Ok(controller) =
                                carla::client::WalkerAIController::try_from(controller_actor)
                            {
                                controller.start();

                                // Start walker moving forward
                                controller.set_max_speed(1.4); // Normal walking speed
                            }
                        }

                        let color = colors[i % colors.len()];
                        walkers.push(WalkerInfo { walker, color });
                        println!(
                            "Walker {} spawned at ({:.1}, {:.1}, {:.1})",
                            i,
                            walker_transform.location.x,
                            walker_transform.location.y,
                            walker_transform.location.z
                        );
                    }
                }
                Err(e) => {
                    println!("Failed to spawn walker {}: {}", i, e);
                }
            }
        }

        if walkers.is_empty() {
            return Err(anyhow::anyhow!("Failed to spawn any walkers"));
        }

        Ok(walkers)
    }

    fn update(&mut self) {
        self.camera.update();
    }

    fn handle_input(&mut self) {
        if is_key_pressed(KeyCode::S) {
            self.show_skeleton = !self.show_skeleton;
            println!(
                "Skeleton overlay: {}",
                if self.show_skeleton { "ON" } else { "OFF" }
            );
        }

        if is_key_pressed(KeyCode::B) {
            self.show_labels = !self.show_labels;
            println!(
                "Bone labels: {}",
                if self.show_labels { "ON" } else { "OFF" }
            );
        }

        if is_key_pressed(KeyCode::Key3) {
            self.show_3d_skeleton = !self.show_3d_skeleton;
            println!(
                "3D skeleton: {}",
                if self.show_3d_skeleton { "ON" } else { "OFF" }
            );
        }

        if is_key_pressed(KeyCode::Tab) {
            self.selected_walker_idx = (self.selected_walker_idx + 1) % self.walkers.len();
            println!("Selected walker {}", self.selected_walker_idx);
        }
    }

    fn draw(&mut self) {
        clear_background(macroquad::prelude::BLACK);

        // Draw camera feed
        let screen_width = screen_width();
        let screen_height = screen_height();
        draw_texture_ex(
            &self.camera.texture,
            0.0,
            0.0,
            macroquad::prelude::WHITE,
            DrawTextureParams {
                dest_size: Some(Vec2::new(screen_width, screen_height)),
                ..Default::default()
            },
        );

        // Draw skeletons
        if self.show_skeleton {
            // Collect bone transforms first to avoid borrow checker issues
            let walker_data: Vec<_> = self
                .walkers
                .iter()
                .enumerate()
                .map(|(idx, walker_info)| {
                    let bone_transforms = walker_info.walker.get_bones_transform();
                    let is_selected = idx == self.selected_walker_idx;
                    (bone_transforms, walker_info.color, is_selected)
                })
                .collect();

            for (bone_transforms, color, is_selected) in walker_data {
                self.draw_skeleton_2d(&bone_transforms, color, is_selected);

                if self.show_3d_skeleton {
                    self.draw_skeleton_3d(&bone_transforms, color);
                }
            }
        }

        // Draw UI
        self.draw_ui();
    }

    fn draw_skeleton_2d(
        &self,
        bone_transforms: &WalkerBoneControlOut,
        color: macroquad::prelude::Color,
        is_selected: bool,
    ) {
        // Build bone name to position map
        let mut bone_positions = std::collections::HashMap::new();
        let mut projected_positions = std::collections::HashMap::new();

        for bone in &bone_transforms.bone_transforms {
            bone_positions.insert(bone.bone_name.clone(), bone.world.location);

            if let Some((u, v)) = self
                .camera
                .intrinsics
                .project_3d_to_2d(&bone.world.location, &self.camera.transform)
            {
                // Only keep points within screen bounds
                if u >= 0.0 && u < screen_width() && v >= 0.0 && v < screen_height() {
                    projected_positions.insert(bone.bone_name.clone(), (u, v));
                }
            }
        }

        // Draw bone connections
        let line_thickness = if is_selected { 3.0 } else { 2.0 };

        for (bone1_name, bone2_name) in BONE_CONNECTIONS {
            if let (Some(&(x1, y1)), Some(&(x2, y2))) = (
                projected_positions.get(*bone1_name),
                projected_positions.get(*bone2_name),
            ) {
                draw_line(x1, y1, x2, y2, line_thickness, color);
            }
        }

        // Draw bone joints
        let joint_radius = if is_selected { 5.0 } else { 3.0 };

        for (x, y) in projected_positions.values() {
            draw_circle(*x, *y, joint_radius, color);
        }

        // Draw bone labels if enabled and walker is selected
        if self.show_labels && is_selected {
            for (bone_name, (x, y)) in &projected_positions {
                // Draw label with background
                let label = bone_name.replace("crl_", "");
                let text_size = 16;
                let text_dims = measure_text(&label, None, text_size, 1.0);

                draw_rectangle(
                    x + 10.0,
                    y - text_dims.height / 2.0,
                    text_dims.width + 4.0,
                    text_dims.height + 4.0,
                    macroquad::prelude::Color::new(0.0, 0.0, 0.0, 0.7),
                );

                draw_text(
                    &label,
                    x + 12.0,
                    y + text_dims.height / 2.0,
                    text_size as f32,
                    macroquad::prelude::WHITE,
                );
            }
        }
    }

    fn draw_skeleton_3d(
        &mut self,
        bone_transforms: &WalkerBoneControlOut,
        color: macroquad::prelude::Color,
    ) {
        // Build bone name to position map
        let mut bone_positions = std::collections::HashMap::new();

        for bone in &bone_transforms.bone_transforms {
            bone_positions.insert(bone.bone_name.clone(), bone.world.location);
        }

        // Draw 3D lines in world space
        let carla_color = Color {
            r: (color.r * 255.0) as u8,
            g: (color.g * 255.0) as u8,
            b: (color.b * 255.0) as u8,
            a: 255,
        };

        for (bone1_name, bone2_name) in BONE_CONNECTIONS {
            if let (Some(&pos1), Some(&pos2)) = (
                bone_positions.get(*bone1_name),
                bone_positions.get(*bone2_name),
            ) {
                self.world.debug().draw_line(
                    pos1,
                    pos2,
                    0.02, // thickness
                    carla_color,
                    0.1, // life time (seconds)
                    false,
                );
            }
        }

        // Draw spheres at joints
        for bone in &bone_transforms.bone_transforms {
            self.world.debug().draw_point(
                bone.world.location,
                0.03, // size
                carla_color,
                0.1, // life time
                false,
            );
        }
    }

    fn draw_ui(&self) {
        let ui_x = 10.0;
        let mut ui_y = 10.0;
        let line_height = 25.0;
        let text_size = 20;

        // Draw controls
        let controls = [
            "Controls:",
            "  S - Toggle skeleton overlay",
            "  B - Toggle bone labels",
            "  3 - Toggle 3D skeleton",
            "  Tab - Cycle walkers",
            "  ESC - Quit",
            "",
            &format!("Walkers: {}", self.walkers.len()),
            &format!("Selected: Walker {}", self.selected_walker_idx),
            &format!(
                "Skeleton: {}",
                if self.show_skeleton { "ON" } else { "OFF" }
            ),
            &format!("Labels: {}", if self.show_labels { "ON" } else { "OFF" }),
            &format!("3D: {}", if self.show_3d_skeleton { "ON" } else { "OFF" }),
        ];

        for line in &controls {
            draw_text(
                line,
                ui_x,
                ui_y,
                text_size as f32,
                macroquad::prelude::WHITE,
            );
            ui_y += line_height;
        }
    }

    fn run(&mut self) {
        loop {
            if is_key_pressed(KeyCode::Escape) {
                break;
            }

            self.handle_input();
            self.update();
            self.draw();

            // Tick simulation
            if let Err(e) = self.world.wait_for_tick() {
                println!("Failed to tick: {}", e);
                break;
            }
        }

        println!("Shutting down...");
    }
}

#[macroquad::main("CARLA - Draw Skeleton")]
async fn main() -> anyhow::Result<()> {
    println!("=== CARLA Walker Skeleton Visualization ===\n");

    let mut visualizer = SkeletonVisualizer::new()?;
    visualizer.run();

    Ok(())
}
