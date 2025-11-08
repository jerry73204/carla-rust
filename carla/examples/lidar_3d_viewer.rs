//! LiDAR 3D Point Cloud Viewer
//!
//! Demonstrates real-time 3D visualization of CARLA LiDAR sensor data using kiss3d.
//! This example replicates the functionality of Python's `open3d_lidar.py` example.
//!
//! # Features
//! - Real-time 3D point cloud visualization from LiDAR sensor
//! - Dual coloring modes: intensity-based and height-based
//! - Interactive 3D camera controls (orbit, pan, zoom)
//! - Performance metrics display
//! - Adjustable point size
//! - Support for high point density (50k-100k points)
//!
//! # Controls
//! - **Mouse**: Orbit camera (left-click drag), pan (right-click drag), zoom (scroll)
//! - **C**: Toggle coloring mode (intensity / height)
//! - **+/-**: Increase/decrease point size
//! - **ESC**: Quit
//!
//! # Usage
//! ```bash
//! cargo run --example lidar_3d_viewer --profile dev-release
//! ```

use carla::{
    client::{Client, Sensor, Vehicle, World as CarlaWorld},
    geom::{Location, Rotation, Transform},
    sensor::data::LidarMeasurement,
};
use kiss3d::{
    light::Light,
    nalgebra::Point3,
    window::{State, Window},
};
use std::{
    sync::{Arc, Mutex},
    time::Instant,
};

const LIDAR_CHANNELS: u32 = 32;
const LIDAR_RANGE: f32 = 50.0;
const LIDAR_POINTS_PER_SECOND: u32 = 100000;
const LIDAR_ROTATION_FREQUENCY: f32 = 10.0;

/// Coloring mode for point cloud visualization
#[derive(Debug, Clone, Copy, PartialEq)]
enum ColorMode {
    Intensity,
    Height,
}

/// Viridis colormap for visualization
fn viridis_colormap(t: f32) -> Point3<f32> {
    let t = t.clamp(0.0, 1.0);

    // Simplified viridis colormap (linear interpolation through key points)
    if t < 0.25 {
        let s = t / 0.25;
        Point3::new(
            0.267 + s * (0.283 - 0.267),
            0.005 + s * (0.141 - 0.005),
            0.329 + s * (0.459 - 0.329),
        )
    } else if t < 0.5 {
        let s = (t - 0.25) / 0.25;
        Point3::new(
            0.283 + s * (0.253 - 0.283),
            0.141 + s * (0.265 - 0.141),
            0.459 + s * (0.530 - 0.459),
        )
    } else if t < 0.75 {
        let s = (t - 0.5) / 0.25;
        Point3::new(
            0.253 + s * (0.206 - 0.253),
            0.265 + s * (0.478 - 0.265),
            0.530 + s * (0.474 - 0.530),
        )
    } else {
        let s = (t - 0.75) / 0.25;
        Point3::new(
            0.206 + s * (0.993 - 0.206),
            0.478 + s * (0.906 - 0.478),
            0.474 + s * (0.144 - 0.474),
        )
    }
}

/// LiDAR sensor manager
struct LidarManager {
    _sensor: Sensor,
    latest_data: Arc<Mutex<Option<LidarMeasurement>>>,
}

impl LidarManager {
    fn new(world: &mut CarlaWorld, vehicle: &Vehicle) -> Result<Self, Box<dyn std::error::Error>> {
        let blueprint_library = world.blueprint_library();
        let lidar_bp = blueprint_library
            .find("sensor.lidar.ray_cast")
            .ok_or("LiDAR blueprint not found")?;

        let mut lidar_bp = lidar_bp;
        let _ = lidar_bp.set_attribute("channels", &LIDAR_CHANNELS.to_string());
        let _ = lidar_bp.set_attribute("range", &LIDAR_RANGE.to_string());
        let _ = lidar_bp.set_attribute("points_per_second", &LIDAR_POINTS_PER_SECOND.to_string());
        let _ = lidar_bp.set_attribute("rotation_frequency", &LIDAR_ROTATION_FREQUENCY.to_string());
        let _ = lidar_bp.set_attribute("upper_fov", "15.0");
        let _ = lidar_bp.set_attribute("lower_fov", "-25.0");

        let lidar_transform = Transform {
            location: Location::new(0.0, 0.0, 2.5),
            rotation: Rotation::new(0.0, 0.0, 0.0),
        };

        let lidar = world.spawn_actor_attached(
            &lidar_bp,
            &lidar_transform,
            vehicle,
            carla::rpc::AttachmentType::Rigid,
        )?;

        let sensor = Sensor::try_from(lidar).map_err(|_| "Failed to cast to Sensor")?;

        let latest_data = Arc::new(Mutex::new(None));
        let latest_data_clone = Arc::clone(&latest_data);

        sensor.listen(move |data| {
            if let Ok(lidar_data) = LidarMeasurement::try_from(data) {
                if let Ok(mut data_lock) = latest_data_clone.lock() {
                    *data_lock = Some(lidar_data);
                }
            }
        });

        Ok(Self {
            _sensor: sensor,
            latest_data,
        })
    }

    fn get_latest_data(&self) -> Option<LidarMeasurement> {
        self.latest_data.lock().ok()?.take()
    }
}

/// Application state for kiss3d
struct AppState {
    lidar: LidarManager,
    color_mode: ColorMode,
    point_size: f32,
    last_frame_time: Instant,
    frame_count: u32,
    fps: f32,
    points: Vec<(Point3<f32>, Point3<f32>)>, // (position, color)
}

impl AppState {
    fn new(lidar: LidarManager) -> Self {
        Self {
            lidar,
            color_mode: ColorMode::Intensity,
            point_size: 3.0,
            last_frame_time: Instant::now(),
            frame_count: 0,
            fps: 0.0,
            points: Vec::new(),
        }
    }
}

impl State for AppState {
    fn step(&mut self, window: &mut Window) {
        // Process keyboard events
        use kiss3d::event::{Action, Key, WindowEvent};
        for event in window.events().iter() {
            match event.value {
                WindowEvent::Key(Key::C, Action::Press, _) => {
                    self.color_mode = match self.color_mode {
                        ColorMode::Intensity => ColorMode::Height,
                        ColorMode::Height => ColorMode::Intensity,
                    };
                    println!("Color mode: {:?}", self.color_mode);
                }
                WindowEvent::Key(Key::Equals, Action::Press, _) => {
                    self.point_size = (self.point_size + 0.5).min(10.0);
                    println!("Point size: {:.1}", self.point_size);
                }
                WindowEvent::Key(Key::Minus, Action::Press, _) => {
                    self.point_size = (self.point_size - 0.5).max(1.0);
                    println!("Point size: {:.1}", self.point_size);
                }
                _ => {}
            }
        }

        // Update point cloud from LiDAR data
        if let Some(lidar_data) = self.lidar.get_latest_data() {
            let lidar_points = lidar_data.as_slice();

            // Find min/max Z for height coloring
            let min_z = lidar_points
                .iter()
                .map(|p| p.point.z)
                .fold(f32::INFINITY, f32::min);
            let max_z = lidar_points
                .iter()
                .map(|p| p.point.z)
                .fold(f32::NEG_INFINITY, f32::max);
            let z_range = (max_z - min_z).max(0.01); // Avoid division by zero

            self.points.clear();
            for point in lidar_points {
                // Convert CARLA to kiss3d coordinates (CARLA: X forward, Y right, Z up)
                // kiss3d uses OpenGL: X right, Y up, Z backward (into screen)
                // So we transform: kiss3d_x = carla_y, kiss3d_y = carla_z, kiss3d_z = -carla_x
                let pos = Point3::new(point.point.y, point.point.z, -point.point.x);

                let color = match self.color_mode {
                    ColorMode::Intensity => {
                        let intensity = point.intensity.clamp(0.0, 1.0);
                        viridis_colormap(intensity)
                    }
                    ColorMode::Height => {
                        // Normalize height to 0-1 range
                        let height_norm = (point.point.z - min_z) / z_range;
                        viridis_colormap(height_norm)
                    }
                };

                self.points.push((pos, color));
            }
        }

        // Draw all points
        for (pos, color) in &self.points {
            window.draw_point(pos, color);
        }

        // Calculate FPS
        self.frame_count += 1;
        let elapsed = self.last_frame_time.elapsed().as_secs_f32();
        if elapsed >= 1.0 {
            self.fps = self.frame_count as f32 / elapsed;
            self.frame_count = 0;
            self.last_frame_time = Instant::now();
        }

        // Print stats occasionally
        if self.frame_count.is_multiple_of(60) {
            println!(
                "Points: {} | FPS: {:.1} | Mode: {:?} | Size: {:.1}",
                self.points.len(),
                self.fps,
                self.color_mode,
                self.point_size
            );
        }
    }
}

fn main() {
    println!("=== CARLA LiDAR 3D Point Cloud Viewer ===\n");

    // Connect to CARLA
    let client = Client::connect("127.0.0.1", 2000, None);
    let mut world = client.world();
    println!("Connected to CARLA server");

    // Spawn vehicle
    let blueprint_library = world.blueprint_library();
    let vehicle_bp = blueprint_library
        .find("vehicle.tesla.model3")
        .expect("Vehicle blueprint not found");

    let spawn_points = world.map().recommended_spawn_points();
    let spawn_point = spawn_points.get(0).expect("No spawn points available");

    let vehicle_actor = world
        .spawn_actor(&vehicle_bp, spawn_point)
        .expect("Failed to spawn vehicle");
    let vehicle = Vehicle::try_from(vehicle_actor).expect("Failed to cast to Vehicle");
    println!("Spawned vehicle");

    // Setup LiDAR
    let lidar = LidarManager::new(&mut world, &vehicle).expect("Failed to create LiDAR sensor");
    println!("LiDAR sensor configured:");
    println!("  Channels: {}", LIDAR_CHANNELS);
    println!("  Range: {} m", LIDAR_RANGE);
    println!("  Points/sec: {}", LIDAR_POINTS_PER_SECOND);

    println!("\nControls:");
    println!("  C: Toggle color mode (intensity/height)");
    println!("  +/-: Adjust point size");
    println!("  ESC: Quit\n");

    // Create window and run
    let mut window = Window::new("CARLA - LiDAR 3D Point Cloud Viewer");
    window.set_light(Light::StickToCamera);
    window.set_background_color(0.0, 0.0, 0.0);

    let state = AppState::new(lidar);
    window.render_loop(state);
}
