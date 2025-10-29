//! Multiple Sensor Visualization Example
//!
//! Demonstrates spawning multiple sensor types on a single vehicle and
//! saving their outputs for visualization. Shows sensor fusion and
//! multi-modal perception.
//!
//! # Sensors Demonstrated
//!
//! - RGB Camera - Standard color image
//! - Depth Camera - Distance to objects
//! - Semantic Segmentation - Object classification by color
//! - LiDAR - 3D point cloud
//!
//! # What it does
//!
//! 1. Spawns a vehicle with autopilot
//! 2. Attaches multiple sensor types to the vehicle
//! 3. Captures synchronized sensor data
//! 4. Saves sensor outputs to disk for visualization
//!
//! # Output
//!
//! Creates `_out/sensors/` directory with:
//! - RGB images: `rgb_XXXX.png`
//! - Depth images: `depth_XXXX.png`
//! - Semantic segmentation: `semantic_XXXX.png`
//! - Point cloud statistics in console
//!
//! # Usage
//!
//! ```bash
//! cargo run --example visualize_multiple_sensors
//! ```

use carla::{
    client::{ActorBase, Client, Sensor, Vehicle},
    rpc::AttachmentType,
    sensor::data::{Image, LidarMeasurement},
};
use nalgebra::{Translation3, UnitQuaternion};
use std::{
    fs,
    sync::{Arc, Mutex},
    thread,
    time::Duration,
};

const IMAGE_WIDTH: u32 = 800;
const IMAGE_HEIGHT: u32 = 600;
const FOV: f32 = 90.0;
const NUM_FRAMES: usize = 20;

#[derive(Default)]
struct SensorFrame {
    rgb_image: Option<Image>,
    depth_image: Option<Image>,
    semantic_image: Option<Image>,
    lidar_measurement: Option<LidarMeasurement>,
    frame_number: usize,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("Multiple Sensor Visualization Example");
    println!("=====================================\n");

    // Create output directory
    fs::create_dir_all("_out/sensors")?;

    // Connect to CARLA
    let client = Client::default();
    let mut world = client.world();

    println!("Connected to CARLA simulator");
    println!("Map: {}\n", world.map().name());

    // Spawn vehicle
    println!("Spawning vehicle...");
    let bp_lib = world.blueprint_library();
    let vehicle_bp = bp_lib
        .find("vehicle.tesla.model3")
        .ok_or("Tesla Model 3 blueprint not found")?;

    let spawn_points = world.map().recommended_spawn_points();
    let spawn_point = spawn_points.get(0).ok_or("No spawn points available")?;

    let vehicle_actor = world
        .spawn_actor(&vehicle_bp, &spawn_point)
        .map_err(|e| format!("Failed to spawn vehicle: {:?}", e))?;
    let vehicle = Vehicle::try_from(vehicle_actor).map_err(|_| "Failed to convert to vehicle")?;
    vehicle.set_autopilot(true);
    println!(" Vehicle spawned with autopilot\n");

    // Define sensor mount position (front hood)
    let sensor_transform = nalgebra::Isometry3::from_parts(
        Translation3::new(2.0, 0.0, 1.5),
        UnitQuaternion::identity(),
    );

    // ========================================================================
    // Configure and spawn sensors
    // ========================================================================

    println!("Spawning sensors...");

    // 1. RGB Camera
    let mut rgb_bp = bp_lib
        .find("sensor.camera.rgb")
        .ok_or("RGB camera not found")?;
    configure_camera(&mut rgb_bp)?;
    let rgb_sensor = spawn_sensor(&mut world, &rgb_bp, &sensor_transform, &vehicle)?;
    println!("   RGB Camera");

    // 2. Depth Camera
    let mut depth_bp = bp_lib
        .find("sensor.camera.depth")
        .ok_or("Depth camera not found")?;
    configure_camera(&mut depth_bp)?;
    let depth_sensor = spawn_sensor(&mut world, &depth_bp, &sensor_transform, &vehicle)?;
    println!("   Depth Camera");

    // 3. Semantic Segmentation Camera
    let mut semantic_bp = bp_lib
        .find("sensor.camera.semantic_segmentation")
        .ok_or("Semantic camera not found")?;
    configure_camera(&mut semantic_bp)?;
    let semantic_sensor = spawn_sensor(&mut world, &semantic_bp, &sensor_transform, &vehicle)?;
    println!("   Semantic Segmentation Camera");

    // 4. LiDAR
    let mut lidar_bp = bp_lib
        .find("sensor.lidar.ray_cast")
        .ok_or("LiDAR not found")?;
    if !lidar_bp.set_attribute("range", "50.0") {
        return Err("Failed to set lidar range".into());
    }
    if !lidar_bp.set_attribute("rotation_frequency", "20.0") {
        return Err("Failed to set lidar rotation_frequency".into());
    }
    if !lidar_bp.set_attribute("channels", "32") {
        return Err("Failed to set lidar channels".into());
    }
    if !lidar_bp.set_attribute("points_per_second", "100000") {
        return Err("Failed to set lidar points_per_second".into());
    }

    let lidar_transform = nalgebra::Isometry3::from_parts(
        Translation3::new(0.0, 0.0, 2.5),
        UnitQuaternion::identity(),
    );
    let lidar_sensor = spawn_sensor(&mut world, &lidar_bp, &lidar_transform, &vehicle)?;
    println!("   LiDAR\n");

    // ========================================================================
    // Set up sensor listeners
    // ========================================================================

    let current_frame: Arc<Mutex<SensorFrame>> = Arc::new(Mutex::new(SensorFrame::default()));

    // RGB listener
    let frame_rgb = current_frame.clone();
    rgb_sensor.listen(move |sensor_data| {
        if let Ok(image) = Image::try_from(sensor_data) {
            let mut frame = frame_rgb.lock().unwrap();
            frame.rgb_image = Some(image);
        }
    });

    // Depth listener
    let frame_depth = current_frame.clone();
    depth_sensor.listen(move |sensor_data| {
        if let Ok(image) = Image::try_from(sensor_data) {
            let mut frame = frame_depth.lock().unwrap();
            frame.depth_image = Some(image);
        }
    });

    // Semantic listener
    let frame_semantic = current_frame.clone();
    semantic_sensor.listen(move |sensor_data| {
        if let Ok(image) = Image::try_from(sensor_data) {
            let mut frame = frame_semantic.lock().unwrap();
            frame.semantic_image = Some(image);
        }
    });

    // LiDAR listener
    let frame_lidar = current_frame.clone();
    lidar_sensor.listen(move |sensor_data| {
        if let Ok(measurement) = LidarMeasurement::try_from(sensor_data) {
            let mut frame = frame_lidar.lock().unwrap();
            frame.lidar_measurement = Some(measurement);
        }
    });

    println!(" All sensors listening\n");

    // ========================================================================
    // Capture and save sensor data
    // ========================================================================

    println!("Capturing {} frames...\n", NUM_FRAMES);

    for i in 0..NUM_FRAMES {
        thread::sleep(Duration::from_millis(100));

        let frame = current_frame.lock().unwrap();

        // Check if we have all sensor data
        if frame.rgb_image.is_some()
            && frame.depth_image.is_some()
            && frame.semantic_image.is_some()
            && frame.lidar_measurement.is_some()
        {
            println!("Frame {:02}: Saving all sensor outputs...", i);

            // Save RGB
            if let Some(ref rgb) = frame.rgb_image {
                let path = format!("_out/sensors/rgb_{:04}.png", i);
                rgb.save_to_disk(&path)?;
            }

            // Save Depth
            if let Some(ref depth) = frame.depth_image {
                let path = format!("_out/sensors/depth_{:04}.png", i);
                depth.save_to_disk(&path)?;
            }

            // Save Semantic
            if let Some(ref semantic) = frame.semantic_image {
                let path = format!("_out/sensors/semantic_{:04}.png", i);
                semantic.save_to_disk(&path)?;
            }

            // Log LiDAR stats
            if let Some(ref lidar) = frame.lidar_measurement {
                println!(
                    "         LiDAR: {} points, {} channels",
                    lidar.len(),
                    lidar.channel_count()
                );
            }
        } else {
            println!("Frame {:02}: Waiting for all sensors...", i);
        }
    }

    // ========================================================================
    // Cleanup
    // ========================================================================

    println!("\n Capture complete!");
    println!(" Sensor data saved to _out/sensors/\n");

    println!("Cleaning up...");
    rgb_sensor.stop();
    depth_sensor.stop();
    semantic_sensor.stop();
    lidar_sensor.stop();

    rgb_sensor.destroy();
    depth_sensor.destroy();
    semantic_sensor.destroy();
    lidar_sensor.destroy();
    vehicle.destroy();

    println!(" Cleanup complete\n");

    println!("=== Visualization Demo Complete ===");
    println!("\nGenerated sensor outputs:");
    println!("  RGB images: _out/sensors/rgb_*.png");
    println!("  Depth maps: _out/sensors/depth_*.png");
    println!("  Semantic segmentation: _out/sensors/semantic_*.png");
    println!("  LiDAR statistics logged to console");

    Ok(())
}

fn configure_camera(
    bp: &mut carla::client::ActorBlueprint,
) -> Result<(), Box<dyn std::error::Error>> {
    if !bp.set_attribute("image_size_x", &IMAGE_WIDTH.to_string()) {
        return Err("Failed to set image_size_x".into());
    }
    if !bp.set_attribute("image_size_y", &IMAGE_HEIGHT.to_string()) {
        return Err("Failed to set image_size_y".into());
    }
    if !bp.set_attribute("fov", &FOV.to_string()) {
        return Err("Failed to set fov".into());
    }
    Ok(())
}

fn spawn_sensor(
    world: &mut carla::client::World,
    bp: &carla::client::ActorBlueprint,
    transform: &nalgebra::Isometry3<f32>,
    parent: &Vehicle,
) -> Result<Sensor, Box<dyn std::error::Error>> {
    let actor = world.spawn_actor_opt(bp, transform, Some(parent), AttachmentType::Rigid)?;
    Sensor::try_from(actor).map_err(|_| "Failed to convert to Sensor".into())
}
