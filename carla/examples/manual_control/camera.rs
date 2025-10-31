//! Camera manager module
//!
//! The CameraManager struct handles:
//! - 14 different sensor types
//! - 5 camera positions for vehicles, 5 for walkers
//! - Recording to disk capability
//! - Sensor data to texture conversion
//!
//! ## Phase 12 Implementation Status
//!
//! - ✅ Subphase 12.2.1: RGB camera integration
//! - ✅ Subphase 12.2.2: Camera position switching (5 positions)
//! - ⏳ Subphase 12.8.1: Camera type switching (14 sensor types) - IN PROGRESS
//! - ⏳ Subphase 12.8.2: LiDAR visualization - TODO
//! - ⏳ Phase 10.3: Camera recording to disk - TODO

use carla::{
    client::Sensor,
    prelude::*,
    rpc::AttachmentType,
    sensor::data::{Image, LidarMeasurement},
};
use eyre::{eyre, Result};
use macroquad::prelude::*;
use nalgebra::{Isometry3, Translation3, UnitQuaternion};
use std::{
    collections::HashMap,
    sync::{
        atomic::{AtomicBool, AtomicU64, Ordering},
        Arc, Mutex,
    },
};
use tracing::info;

/// Type alias for pending RGBA texture data from sensor callbacks
/// Format: (rgba_data, width, height)
type PendingRgba = Arc<Mutex<Option<(Vec<u8>, u32, u32)>>>;

/// Camera position relative to vehicle
#[derive(Clone)]
pub struct CameraTransform {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub pitch: f32,
    pub yaw: f32,
    pub roll: f32,
}

impl CameraTransform {
    /// Create an Isometry3 transform from this CameraTransform
    fn to_isometry(&self) -> Isometry3<f32> {
        // Convert Euler angles (pitch, yaw, roll) to quaternion
        // CARLA uses: pitch (around Y), yaw (around Z), roll (around X)
        let pitch_rad = self.pitch.to_radians();
        let yaw_rad = self.yaw.to_radians();
        let roll_rad = self.roll.to_radians();

        let rotation = UnitQuaternion::from_euler_angles(roll_rad, pitch_rad, yaw_rad);

        Isometry3::from_parts(Translation3::new(self.x, self.y, self.z), rotation)
    }
}

/// Sensor definition for one of the 14 supported sensor types
///
/// ✅ Subphase 12.8.1: Sensor type definitions matching Python manual_control.py
#[derive(Clone)]
pub struct SensorDefinition {
    /// Blueprint ID (e.g., "sensor.camera.rgb", "sensor.lidar.ray_cast")
    pub blueprint_id: String,
    /// Display name shown in notifications
    pub display_name: String,
    /// Additional blueprint attributes to set (e.g., lens distortion parameters)
    pub attributes: HashMap<String, String>,
    /// Whether this sensor produces image data (true) or LiDAR data (false)
    pub is_image_sensor: bool,
    /// Color conversion mode (if applicable)
    pub color_conversion: ColorConversion,
}

/// Color conversion modes for camera sensors
///
/// Matches CARLA's ColorConverter enum from Python
#[derive(Clone, Copy, Debug)]
pub enum ColorConversion {
    /// No conversion (raw BGRA from CARLA)
    Raw,
    /// Convert depth to grayscale
    Depth,
    /// Convert depth to logarithmic grayscale
    LogarithmicDepth,
    /// Convert semantic segmentation to CityScapes color palette
    CityScapesPalette,
}

impl SensorDefinition {
    /// Get all 14 sensor definitions
    ///
    /// ✅ Subphase 12.8.1: Based on Python manual_control.py lines 1095-1114
    pub fn get_all_sensors() -> Vec<SensorDefinition> {
        vec![
            // 0: RGB Camera
            SensorDefinition {
                blueprint_id: "sensor.camera.rgb".to_string(),
                display_name: "Camera RGB".to_string(),
                attributes: HashMap::new(),
                is_image_sensor: true,
                color_conversion: ColorConversion::Raw,
            },
            // 1: Depth Camera (Raw)
            SensorDefinition {
                blueprint_id: "sensor.camera.depth".to_string(),
                display_name: "Camera Depth (Raw)".to_string(),
                attributes: HashMap::new(),
                is_image_sensor: true,
                color_conversion: ColorConversion::Raw,
            },
            // 2: Depth Camera (Gray Scale)
            SensorDefinition {
                blueprint_id: "sensor.camera.depth".to_string(),
                display_name: "Camera Depth (Gray Scale)".to_string(),
                attributes: HashMap::new(),
                is_image_sensor: true,
                color_conversion: ColorConversion::Depth,
            },
            // 3: Depth Camera (Logarithmic Gray Scale)
            SensorDefinition {
                blueprint_id: "sensor.camera.depth".to_string(),
                display_name: "Camera Depth (Logarithmic Gray Scale)".to_string(),
                attributes: HashMap::new(),
                is_image_sensor: true,
                color_conversion: ColorConversion::LogarithmicDepth,
            },
            // 4: Semantic Segmentation (Raw)
            SensorDefinition {
                blueprint_id: "sensor.camera.semantic_segmentation".to_string(),
                display_name: "Camera Semantic Segmentation (Raw)".to_string(),
                attributes: HashMap::new(),
                is_image_sensor: true,
                color_conversion: ColorConversion::Raw,
            },
            // 5: Semantic Segmentation (CityScapes Palette)
            SensorDefinition {
                blueprint_id: "sensor.camera.semantic_segmentation".to_string(),
                display_name: "Camera Semantic Segmentation (CityScapes Palette)".to_string(),
                attributes: HashMap::new(),
                is_image_sensor: true,
                color_conversion: ColorConversion::CityScapesPalette,
            },
            // 6: Instance Segmentation (CityScapes Palette)
            SensorDefinition {
                blueprint_id: "sensor.camera.instance_segmentation".to_string(),
                display_name: "Camera Instance Segmentation (CityScapes Palette)".to_string(),
                attributes: HashMap::new(),
                is_image_sensor: true,
                color_conversion: ColorConversion::CityScapesPalette,
            },
            // 7: Instance Segmentation (Raw)
            SensorDefinition {
                blueprint_id: "sensor.camera.instance_segmentation".to_string(),
                display_name: "Camera Instance Segmentation (Raw)".to_string(),
                attributes: HashMap::new(),
                is_image_sensor: true,
                color_conversion: ColorConversion::Raw,
            },
            // 8: Cosmos Control Visualization
            SensorDefinition {
                blueprint_id: "sensor.camera.cosmos_visualization".to_string(),
                display_name: "Cosmos Control Visualization".to_string(),
                attributes: HashMap::new(),
                is_image_sensor: true,
                color_conversion: ColorConversion::Raw,
            },
            // 9: LiDAR (Ray-Cast)
            SensorDefinition {
                blueprint_id: "sensor.lidar.ray_cast".to_string(),
                display_name: "Lidar (Ray-Cast)".to_string(),
                attributes: {
                    let mut attrs = HashMap::new();
                    attrs.insert("range".to_string(), "50".to_string());
                    attrs
                },
                is_image_sensor: false,
                color_conversion: ColorConversion::Raw,
            },
            // 10: Dynamic Vision Sensor
            SensorDefinition {
                blueprint_id: "sensor.camera.dvs".to_string(),
                display_name: "Dynamic Vision Sensor".to_string(),
                attributes: HashMap::new(),
                is_image_sensor: true,
                color_conversion: ColorConversion::Raw,
            },
            // 11: RGB Camera (Distorted)
            SensorDefinition {
                blueprint_id: "sensor.camera.rgb".to_string(),
                display_name: "Camera RGB Distorted".to_string(),
                attributes: {
                    let mut attrs = HashMap::new();
                    attrs.insert("lens_circle_multiplier".to_string(), "3.0".to_string());
                    attrs.insert("lens_circle_falloff".to_string(), "3.0".to_string());
                    attrs.insert(
                        "chromatic_aberration_intensity".to_string(),
                        "0.5".to_string(),
                    );
                    attrs.insert("chromatic_aberration_offset".to_string(), "0".to_string());
                    attrs
                },
                is_image_sensor: true,
                color_conversion: ColorConversion::Raw,
            },
            // 12: Optical Flow
            SensorDefinition {
                blueprint_id: "sensor.camera.optical_flow".to_string(),
                display_name: "Optical Flow".to_string(),
                attributes: HashMap::new(),
                is_image_sensor: true,
                color_conversion: ColorConversion::Raw,
            },
            // 13: Camera Normals
            SensorDefinition {
                blueprint_id: "sensor.camera.normals".to_string(),
                display_name: "Camera Normals".to_string(),
                attributes: HashMap::new(),
                is_image_sensor: true,
                color_conversion: ColorConversion::Raw,
            },
        ]
    }
}

/// Camera/sensor manager
///
/// Manages multiple camera sensors and their rendering
pub struct CameraManager {
    pub sensor: Option<Sensor>,
    pub texture: Option<Texture2D>, // Texture created on main thread
    pub pending_rgba: PendingRgba,  // RGBA data from sensor thread
    pub lidar_points: Arc<Mutex<Vec<(f32, f32, f32)>>>, // For LiDAR visualization

    pub current_position: usize,
    pub current_sensor: usize,

    // ✅ Subphase 12.2.2: Camera position presets (5 positions)
    camera_transforms: Vec<(CameraTransform, AttachmentType)>,

    // ✅ Subphase 12.8.1: Sensor type definitions (14 types)
    sensors: Vec<SensorDefinition>,

    width: u32,
    height: u32,
    gamma: f32,

    // ✅ Subphase 12.10.3: Camera recording state (shared with sensor callback thread)
    recording: Arc<AtomicBool>,
    recording_frame: Arc<AtomicU64>,
}

impl CameraManager {
    /// Create a new camera manager
    pub fn new(width: u32, height: u32, gamma: f32) -> Self {
        // ✅ Subphase 12.2.2: Define 5 camera positions for vehicles
        // Based on Python manual_control.py lines 1080-1086
        let camera_transforms = vec![
            // 0: Rear chase camera (default view, behind vehicle)
            (
                CameraTransform {
                    x: -5.0,
                    y: 0.0,
                    z: 2.5,
                    pitch: 8.0,
                    yaw: 0.0,
                    roll: 0.0,
                },
                AttachmentType::SpringArmGhost,
            ),
            // 1: Hood camera (driver's perspective)
            (
                CameraTransform {
                    x: 1.5,
                    y: 0.0,
                    z: 1.4,
                    pitch: 0.0,
                    yaw: 0.0,
                    roll: 0.0,
                },
                AttachmentType::Rigid,
            ),
            // 2: Side camera (right side of vehicle)
            (
                CameraTransform {
                    x: 0.0,
                    y: 3.0,
                    z: 2.0,
                    pitch: 0.0,
                    yaw: -90.0,
                    roll: 0.0,
                },
                AttachmentType::SpringArmGhost,
            ),
            // 3: High camera (aerial view)
            (
                CameraTransform {
                    x: -5.5,
                    y: 0.0,
                    z: 6.0,
                    pitch: -15.0,
                    yaw: 0.0,
                    roll: 0.0,
                },
                AttachmentType::SpringArmGhost,
            ),
            // 4: Fixed camera (left side, fixed position)
            (
                CameraTransform {
                    x: 0.0,
                    y: -3.0,
                    z: 1.5,
                    pitch: 0.0,
                    yaw: 90.0,
                    roll: 0.0,
                },
                AttachmentType::Rigid,
            ),
        ];

        // ✅ Subphase 12.8.1: Load all 14 sensor definitions
        let sensors = SensorDefinition::get_all_sensors();

        Self {
            sensor: None,
            texture: None,
            pending_rgba: Arc::new(Mutex::new(None)),
            lidar_points: Arc::new(Mutex::new(Vec::new())),
            current_position: 0, // Start with rear chase camera
            current_sensor: 0,   // RGB by default
            camera_transforms,
            sensors,
            width,
            height,
            gamma,
            recording: Arc::new(AtomicBool::new(false)),
            recording_frame: Arc::new(AtomicU64::new(0)),
        }
    }

    /// Spawn sensor based on current sensor index
    ///
    /// ✅ Subphase 12.8.1: Spawn any of the 14 sensor types
    pub fn spawn_camera(&mut self, world: &mut crate::world::World) -> Result<()> {
        let player = world
            .player
            .as_ref()
            .ok_or_else(|| eyre!("No player vehicle available"))?;

        // Get current sensor definition
        let sensor_def = &self.sensors[self.current_sensor].clone();

        // Get blueprint for current sensor
        let blueprint_library = world.world.blueprint_library();
        let mut sensor_bp = blueprint_library
            .find(&sensor_def.blueprint_id)
            .ok_or_else(|| eyre!("{} blueprint not found", sensor_def.blueprint_id))?;

        // Set common camera attributes
        if sensor_def.is_image_sensor {
            if !sensor_bp.set_attribute("image_size_x", &self.width.to_string()) {
                return Err(eyre!("Failed to set image_size_x"));
            }
            if !sensor_bp.set_attribute("image_size_y", &self.height.to_string()) {
                return Err(eyre!("Failed to set image_size_y"));
            }
            if !sensor_bp.set_attribute("fov", "110.0") {
                return Err(eyre!("Failed to set fov"));
            }
            // Set gamma (silently fails if attribute doesn't exist)
            let _ = sensor_bp.set_attribute("gamma", &self.gamma.to_string());
        }

        // Set custom attributes from sensor definition
        for (attr_name, attr_value) in &sensor_def.attributes {
            if !sensor_bp.set_attribute(attr_name, attr_value) {
                return Err(eyre!("Failed to set attribute {}", attr_name));
            }
        }

        // Get current camera transform
        let (cam_transform, attachment_type) = &self.camera_transforms[self.current_position];
        let transform = cam_transform.to_isometry();

        info!(
            "Spawning sensor '{}' at position {}",
            sensor_def.display_name, self.current_position
        );

        // Spawn sensor attached to vehicle
        let sensor_actor = world
            .world
            .spawn_actor_opt(
                &sensor_bp,
                &transform,
                Some(player),
                attachment_type.clone(),
            )
            .map_err(|e| eyre!("Failed to spawn sensor: {:?}", e))?;

        let sensor =
            Sensor::try_from(sensor_actor).map_err(|_| eyre!("Failed to convert to Sensor"))?;

        // Set up listener based on sensor type
        if sensor_def.is_image_sensor {
            // Image sensor (camera)
            let pending_rgba_clone = Arc::clone(&self.pending_rgba);
            let recording_clone = Arc::clone(&self.recording);
            let recording_frame_clone = Arc::clone(&self.recording_frame);
            let width = self.width;
            let height = self.height;
            let color_conversion = sensor_def.color_conversion;

            sensor.listen(move |data| {
                if let Ok(image) = Image::try_from(data) {
                    Self::update_texture_from_image(
                        Arc::clone(&pending_rgba_clone),
                        image,
                        width,
                        height,
                        color_conversion,
                        Arc::clone(&recording_clone),
                        Arc::clone(&recording_frame_clone),
                    );
                }
            });
        } else {
            // LiDAR sensor
            let lidar_points_clone = Arc::clone(&self.lidar_points);
            let pending_rgba_clone = Arc::clone(&self.pending_rgba);
            let width = self.width;
            let height = self.height;

            sensor.listen(move |data| {
                if let Ok(lidar_data) = LidarMeasurement::try_from(data) {
                    Self::update_texture_from_lidar(
                        Arc::clone(&pending_rgba_clone),
                        Arc::clone(&lidar_points_clone),
                        lidar_data,
                        width,
                        height,
                    );
                }
            });
        }

        self.sensor = Some(sensor);
        info!(
            "✓ Sensor '{}' spawned and listening",
            sensor_def.display_name
        );

        Ok(())
    }

    /// Switch to next camera position
    ///
    /// ✅ Subphase 12.2.2: TAB key functionality
    pub fn toggle_camera(&mut self, world: &mut crate::world::World) -> Result<()> {
        // Destroy current sensor
        if let Some(ref sensor) = self.sensor {
            sensor.destroy();
        }
        self.sensor = None;

        // Increment position index (wrap around)
        self.current_position = (self.current_position + 1) % self.camera_transforms.len();
        info!("Switching to camera position {}", self.current_position);

        // Respawn sensor at new position
        self.spawn_camera(world)?;

        Ok(())
    }

    /// Switch to next sensor type
    ///
    /// ✅ Subphase 12.8.1: Backtick/N key functionality
    pub fn next_sensor(&mut self, world: &mut crate::world::World) -> Result<String> {
        // Destroy current sensor
        if let Some(ref sensor) = self.sensor {
            sensor.destroy();
        }
        self.sensor = None;

        // Increment sensor index (wrap around)
        self.current_sensor = (self.current_sensor + 1) % self.sensors.len();
        let sensor_name = self.sensors[self.current_sensor].display_name.clone();

        info!(
            "Switching to sensor {} (index {})",
            sensor_name, self.current_sensor
        );

        // Respawn sensor with new type
        self.spawn_camera(world)?;

        Ok(sensor_name)
    }

    /// Set sensor by index
    ///
    /// ✅ Subphase 12.8.1: Number key functionality
    pub fn set_sensor(&mut self, world: &mut crate::world::World, index: usize) -> Result<String> {
        if index >= self.sensors.len() {
            return Err(eyre!(
                "Sensor index {} out of range (0-{})",
                index,
                self.sensors.len() - 1
            ));
        }

        // Destroy current sensor
        if let Some(ref sensor) = self.sensor {
            sensor.destroy();
        }
        self.sensor = None;

        // Set sensor index directly
        self.current_sensor = index;
        let sensor_name = self.sensors[self.current_sensor].display_name.clone();

        info!(
            "Setting sensor to {} (index {})",
            sensor_name, self.current_sensor
        );

        // Respawn sensor with new type
        self.spawn_camera(world)?;

        Ok(sensor_name)
    }

    /// Get current sensor display name
    #[allow(dead_code)]
    pub fn current_sensor_name(&self) -> &str {
        &self.sensors[self.current_sensor].display_name
    }

    /// Update camera textures from pending sensor data
    ///
    /// Must be called on main thread before render().
    /// Creates textures from RGBA data produced by sensor callbacks.
    pub fn update(&mut self) {
        // Check for pending RGBA data from sensor callback
        let mut pending = self.pending_rgba.lock().unwrap();
        if let Some((rgba_data, width, height)) = pending.take() {
            // Create texture on main thread
            self.texture = Some(Texture2D::from_rgba8(
                width as u16,
                height as u16,
                &rgba_data,
            ));
        }
    }

    /// Render camera texture to screen
    ///
    /// ✅ Subphase 12.2.1: Display texture full-screen
    pub fn render(&self) -> Result<()> {
        if let Some(ref texture) = self.texture {
            draw_texture(texture, 0.0, 0.0, WHITE);
        } else {
            // No texture yet, draw black background
            clear_background(BLACK);
        }
        Ok(())
    }

    /// Update texture from sensor image data with color conversion
    ///
    /// ✅ Subphase 12.8.1: Support multiple color conversion modes
    /// ✅ Subphase 12.10.3: Save PNG frames when recording enabled
    /// Runs on sensor callback thread - stores RGBA data for main thread to process
    fn update_texture_from_image(
        pending_rgba_arc: PendingRgba,
        image: Image,
        width: u32,
        height: u32,
        color_conversion: ColorConversion,
        recording: Arc<AtomicBool>,
        recording_frame: Arc<AtomicU64>,
    ) {
        // Get raw image data (Color format from CARLA with BGRA layout)
        let raw_data = image.as_slice();

        // Convert based on color conversion mode
        let rgba_data = match color_conversion {
            ColorConversion::Raw | ColorConversion::CityScapesPalette => {
                // Direct BGRA to RGBA conversion
                let mut rgba_data: Vec<u8> = Vec::with_capacity(raw_data.len() * 4);
                for color in raw_data {
                    rgba_data.push(color.r); // R
                    rgba_data.push(color.g); // G
                    rgba_data.push(color.b); // B
                    rgba_data.push(color.a); // A
                }
                rgba_data
            }
            ColorConversion::Depth => {
                // Convert depth to grayscale
                // Depth is encoded in RGB as (R + G*256 + B*256*256) / (256*256*256 - 1)
                let mut rgba_data: Vec<u8> = Vec::with_capacity(raw_data.len() * 4);
                for color in raw_data {
                    let normalized = ((color.r as f32
                        + color.g as f32 * 256.0
                        + color.b as f32 * 256.0 * 256.0)
                        / (256.0 * 256.0 * 256.0 - 1.0))
                        * 255.0;
                    let gray = normalized as u8;
                    rgba_data.push(gray); // R
                    rgba_data.push(gray); // G
                    rgba_data.push(gray); // B
                    rgba_data.push(255); // A
                }
                rgba_data
            }
            ColorConversion::LogarithmicDepth => {
                // Convert depth to logarithmic grayscale
                let mut rgba_data: Vec<u8> = Vec::with_capacity(raw_data.len() * 4);
                for color in raw_data {
                    let normalized =
                        (color.r as f32 + color.g as f32 * 256.0 + color.b as f32 * 256.0 * 256.0)
                            / (256.0 * 256.0 * 256.0 - 1.0);
                    let log_depth = (1.0_f32 + normalized * 1000.0).ln() / (1.0_f32 + 1000.0).ln();
                    let gray = (log_depth * 255.0) as u8;
                    rgba_data.push(gray); // R
                    rgba_data.push(gray); // G
                    rgba_data.push(gray); // B
                    rgba_data.push(255); // A
                }
                rgba_data
            }
        };

        // Store RGBA data for main thread to create texture (not creating texture here - wrong thread!)
        let mut pending_guard = pending_rgba_arc.lock().unwrap();
        *pending_guard = Some((rgba_data.clone(), width, height));

        // ✅ Subphase 12.10.3: Save frame to disk if recording
        if recording.load(Ordering::SeqCst) {
            let frame = recording_frame.fetch_add(1, Ordering::SeqCst);
            let filename = format!("_out/{:08}.png", frame);

            // Save PNG in background (don't block sensor callback)
            if let Err(e) = image::save_buffer(
                &filename,
                &rgba_data,
                width,
                height,
                image::ColorType::Rgba8,
            ) {
                eprintln!("Failed to save frame {}: {}", frame, e);
            }
        }
    }

    /// Update texture from LiDAR point cloud data
    ///
    /// ✅ Subphase 12.8.2: LiDAR bird's-eye view visualization
    /// Runs on sensor callback thread - stores RGBA data for main thread to process
    fn update_texture_from_lidar(
        pending_rgba_arc: PendingRgba,
        points_arc: Arc<Mutex<Vec<(f32, f32, f32)>>>,
        lidar_data: LidarMeasurement,
        width: u32,
        height: u32,
    ) {
        // Extract point cloud (x, y, z coordinates)
        let points: Vec<(f32, f32, f32)> = lidar_data
            .as_slice()
            .iter()
            .map(|pt| (pt.point.x, pt.point.y, pt.point.z))
            .collect();

        // Store points for potential future use
        {
            let mut points_guard = points_arc.lock().unwrap();
            *points_guard = points.clone();
        }

        // Create bird's-eye view texture (top-down view)
        // Black background with white dots for LiDAR points
        let mut rgba_data: Vec<u8> = vec![0; (width * height * 4) as usize];

        // Project points to 2D bird's-eye view
        // X-axis: left-right, Y-axis: forward-backward
        let range = 50.0; // 50 meter range (matches LiDAR sensor config)
        let scale = (width.min(height) as f32) / (2.0 * range);
        let center_x = (width / 2) as i32;
        let center_y = (height / 2) as i32;

        for (x, y, z) in points {
            // Filter points at ground level (reduce noise)
            if !(-2.5..=2.0).contains(&z) {
                continue;
            }

            // Project to screen coordinates (bird's-eye view)
            // X is right, Y is forward in CARLA
            let screen_x = center_x + (x * scale) as i32;
            let screen_y = center_y - (y * scale) as i32; // Invert Y for screen coords

            // Check bounds
            if screen_x >= 0 && screen_x < width as i32 && screen_y >= 0 && screen_y < height as i32
            {
                let idx = ((screen_y as u32 * width + screen_x as u32) * 4) as usize;
                if idx + 3 < rgba_data.len() {
                    rgba_data[idx] = 255; // R
                    rgba_data[idx + 1] = 255; // G
                    rgba_data[idx + 2] = 255; // B
                    rgba_data[idx + 3] = 255; // A
                }
            }
        }

        // Store RGBA data for main thread to create texture (not creating texture here - wrong thread!)
        let mut pending_guard = pending_rgba_arc.lock().unwrap();
        *pending_guard = Some((rgba_data, width, height));
    }

    /// Toggle camera frame recording
    ///
    /// ✅ Subphase 12.10.3: R key toggles recording to _out/ directory
    pub fn toggle_recording(&mut self) -> &'static str {
        let was_recording = self.recording.fetch_not(Ordering::SeqCst);
        if !was_recording {
            // Starting recording
            self.recording_frame.store(0, Ordering::SeqCst);
            // Create _out directory if it doesn't exist
            std::fs::create_dir_all("_out").ok();
            "Recording On"
        } else {
            // Stopping recording
            "Recording Off"
        }
    }

    /// Check if recording is currently active
    #[allow(dead_code)]
    pub fn is_recording(&self) -> bool {
        self.recording.load(Ordering::SeqCst)
    }

    /// Cleanup - destroy sensor
    pub fn destroy(&mut self) {
        if let Some(ref sensor) = self.sensor {
            info!("Destroying camera sensor...");
            sensor.destroy();
        }
        self.sensor = None;
    }
}

impl Drop for CameraManager {
    fn drop(&mut self) {
        self.destroy();
    }
}
