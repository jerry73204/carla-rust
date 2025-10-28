//! Tutorial Example - Basic CARLA Rust Workflow
//!
//! This example demonstrates the fundamental CARLA workflow:
//! 1. Connect to simulator
//! 2. Spawn a vehicle with autopilot
//! 3. Attach a camera sensor
//! 4. Save sensor data to disk
//!
//! Prerequisites:
//! - CARLA simulator must be running
//! - Directory ./tutorial_output/ will be created for sensor data
//!
//! Run with:
//! ```bash
//! cargo run --example tutorial
//! ```

use carla::{
    client::{ActorBase, Client, Sensor, Vehicle},
    sensor::data::Image,
};
use std::{
    fs,
    path::Path,
    sync::{Arc, Mutex},
    thread,
    time::Duration,
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== CARLA Rust Tutorial ===\n");

    // Step 1: Connect to CARLA
    println!("Step 1: Connecting to CARLA simulator...");
    let client = Client::connect("localhost", 2000, None);
    let mut world = client.world();
    println!("✓ Connected! Current map: {}\n", world.map().name());

    // Create output directory for sensor data
    let output_dir = Path::new("./tutorial_output");
    if !output_dir.exists() {
        fs::create_dir(output_dir)?;
        println!("Created output directory: {}\n", output_dir.display());
    }

    // Step 2: Spawn a vehicle with autopilot
    println!("Step 2: Spawning vehicle with autopilot...");

    // Get vehicle blueprint
    let blueprint_library = world.blueprint_library();
    let vehicle_bp = blueprint_library
        .find("vehicle.tesla.model3")
        .ok_or("Tesla Model 3 blueprint not found")?;

    // Get spawn point
    let spawn_points = world.map().recommended_spawn_points();
    let spawn_point = spawn_points.get(0).ok_or("No spawn points available")?;

    // Spawn vehicle
    let actor = world.spawn_actor(&vehicle_bp, &spawn_point)?;
    let vehicle = Vehicle::try_from(actor).map_err(|_| "Failed to convert to vehicle")?;

    println!("✓ Vehicle spawned (ID: {})", vehicle.id());

    // Enable autopilot
    vehicle.set_autopilot(true);
    println!("✓ Autopilot enabled\n");

    // Let vehicle drive a bit
    println!("Letting vehicle drive for 3 seconds...");
    thread::sleep(Duration::from_secs(3));

    // Step 3: Attach a camera sensor
    println!("\nStep 3: Attaching RGB camera sensor...");

    // Get camera blueprint
    let camera_bp = blueprint_library
        .find("sensor.camera.rgb")
        .ok_or("RGB camera blueprint not found")?;

    // Configure camera resolution
    let mut camera_bp = camera_bp;
    if !camera_bp.set_attribute("image_size_x", "800") {
        return Err("Failed to set camera image_size_x".into());
    }
    if !camera_bp.set_attribute("image_size_y", "600") {
        return Err("Failed to set camera image_size_y".into());
    }
    if !camera_bp.set_attribute("fov", "90.0") {
        return Err("Failed to set camera fov".into());
    }

    // Create transform for camera (mounted on vehicle roof)
    let camera_transform = nalgebra::Isometry3::from_parts(
        nalgebra::Translation3::new(0.0, 0.0, 2.0), // 2m above vehicle
        nalgebra::UnitQuaternion::identity(),
    );

    // Spawn camera attached to vehicle
    use carla::rpc::AttachmentType;
    let camera_actor = world.spawn_actor_opt(
        &camera_bp,
        &camera_transform,
        Some(&vehicle),
        AttachmentType::Rigid,
    )?;
    let camera = Sensor::try_from(camera_actor).map_err(|_| "Failed to convert to sensor")?;

    println!("✓ Camera attached (ID: {})\n", camera.id());

    // Step 4: Save sensor data to disk
    println!("Step 4: Listening to camera and saving images...");

    let frame_count = Arc::new(Mutex::new(0));
    let frame_count_clone = Arc::clone(&frame_count);

    camera.listen(move |sensor_data| {
        // Try to cast to Image
        if let Ok(image) = Image::try_from(sensor_data) {
            let mut count = frame_count_clone.lock().unwrap();
            *count += 1;

            // Save every 10th frame
            if *count % 10 == 0 {
                let filename = format!("tutorial_output/frame_{:04}.png", count);
                match image.save_to_disk(&filename) {
                    Ok(_) => println!(
                        "Frame {}: {}x{} pixels, FOV: {:.1}° - Saved to {}",
                        count,
                        image.width(),
                        image.height(),
                        image.fov_angle(),
                        filename
                    ),
                    Err(e) => eprintln!("Failed to save frame {}: {}", count, e),
                }
            }
        }
    });

    println!("✓ Camera listening. Capturing images for 20 seconds...\n");

    // Let the simulation run and collect data
    for i in 1..=20 {
        thread::sleep(Duration::from_secs(1));
        println!("Recording... {} seconds", i);
    }

    // Cleanup
    println!("\n=== Cleanup ===");
    camera.stop();
    println!("✓ Camera stopped");

    let camera_destroyed = camera.destroy();
    println!("✓ Camera destroyed: {}", camera_destroyed);

    let vehicle_destroyed = vehicle.destroy();
    println!("✓ Vehicle destroyed: {}", vehicle_destroyed);

    let total_frames = *frame_count.lock().unwrap();
    println!("\n=== Tutorial Complete ===");
    println!("Total frames captured: {}", total_frames);
    println!("Saved frames: {}", total_frames / 10);

    Ok(())
}
