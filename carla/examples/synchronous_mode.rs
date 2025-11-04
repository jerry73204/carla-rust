//! Synchronous Mode Example
//!
//! This example demonstrates synchronous simulation mode with multiple sensors.
//! In synchronous mode, the simulation waits for tick() calls, enabling
//! deterministic sensor data collection and precise control.
//!
//! Prerequisites:
//! - CARLA simulator must be running
//!
//! Run with:
//! ```bash
//! cargo run --example synchronous_mode
//! ```

use carla::{
    client::{ActorBase, Client, Sensor, Vehicle},
    geom::{Location, Rotation},
    rpc::AttachmentType,
    sensor::data::Image,
};
use std::{
    sync::{Arc, Mutex},
    thread,
    time::Duration,
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== CARLA Synchronous Mode Example ===\n");

    // Connect to CARLA
    println!("Connecting to CARLA simulator...");
    let client = Client::connect("localhost", 2000, None);
    let mut world = client.world();
    println!("✓ Connected! Current map: {}\n", world.map().name());

    // Enable synchronous mode
    println!("Configuring synchronous mode...");
    let mut settings = world.settings();
    settings.synchronous_mode = true;
    settings.fixed_delta_seconds = Some(0.05); // 20 FPS
    world.apply_settings(&settings, Duration::from_secs(5));
    println!("✓ Synchronous mode enabled (fixed_delta: 0.05s, 20 FPS)\n");

    // Spawn a vehicle with autopilot
    println!("Spawning vehicle with autopilot...");
    let blueprint_library = world.blueprint_library();
    let vehicle_bp = blueprint_library
        .find("vehicle.tesla.model3")
        .ok_or("Tesla Model 3 blueprint not found")?;

    let spawn_points = world.map().recommended_spawn_points();
    let spawn_point = spawn_points.get(0).ok_or("No spawn points available")?;

    let actor = world.spawn_actor(&vehicle_bp, spawn_point)?;
    let vehicle = Vehicle::try_from(actor).map_err(|_| "Failed to convert to vehicle")?;
    vehicle.set_autopilot(true);
    println!("✓ Vehicle spawned (ID: {})\n", vehicle.id());

    // Spawn RGB camera
    println!("Spawning sensors...");
    let mut camera_rgb_bp = blueprint_library
        .find("sensor.camera.rgb")
        .ok_or("RGB camera blueprint not found")?;
    if !camera_rgb_bp.set_attribute("image_size_x", "800") {
        return Err("Failed to set RGB camera image_size_x".into());
    }
    if !camera_rgb_bp.set_attribute("image_size_y", "600") {
        return Err("Failed to set RGB camera image_size_y".into());
    }
    if !camera_rgb_bp.set_attribute("fov", "90.0") {
        return Err("Failed to set RGB camera fov".into());
    }

    let camera_transform = carla::geom::Transform {
        location: Location::new(-5.0, 0.0, 3.0), // Behind and above vehicle
        rotation: Rotation::new(-0.15_f32.to_degrees(), 0.0, 0.0), // Slight downward angle
    };

    let camera_rgb_actor = world.spawn_actor_opt(
        &camera_rgb_bp,
        &camera_transform,
        Some(&vehicle),
        AttachmentType::SpringArm,
    )?;
    let camera_rgb =
        Sensor::try_from(camera_rgb_actor).map_err(|_| "Failed to convert to sensor")?;

    // Spawn semantic segmentation camera
    let mut camera_seg_bp = blueprint_library
        .find("sensor.camera.semantic_segmentation")
        .ok_or("Semantic camera blueprint not found")?;
    if !camera_seg_bp.set_attribute("image_size_x", "800") {
        return Err("Failed to set semantic camera image_size_x".into());
    }
    if !camera_seg_bp.set_attribute("image_size_y", "600") {
        return Err("Failed to set semantic camera image_size_y".into());
    }
    if !camera_seg_bp.set_attribute("fov", "90.0") {
        return Err("Failed to set semantic camera fov".into());
    }

    let camera_seg_actor = world.spawn_actor_opt(
        &camera_seg_bp,
        &camera_transform,
        Some(&vehicle),
        AttachmentType::SpringArm,
    )?;
    let camera_seg =
        Sensor::try_from(camera_seg_actor).map_err(|_| "Failed to convert to sensor")?;

    println!("✓ RGB camera spawned (ID: {})", camera_rgb.id());
    println!("✓ Semantic camera spawned (ID: {})\n", camera_seg.id());

    // Setup sensor data collection
    let rgb_frame_count = Arc::new(Mutex::new(0));
    let seg_frame_count = Arc::new(Mutex::new(0));

    let rgb_count_clone = Arc::clone(&rgb_frame_count);
    camera_rgb.listen(move |sensor_data| {
        if let Ok(_image) = Image::try_from(sensor_data) {
            let mut count = rgb_count_clone.lock().unwrap();
            *count += 1;
        }
    });

    let seg_count_clone = Arc::clone(&seg_frame_count);
    camera_seg.listen(move |sensor_data| {
        if let Ok(_image) = Image::try_from(sensor_data) {
            let mut count = seg_count_clone.lock().unwrap();
            *count += 1;
        }
    });

    println!("✓ Sensors listening\n");

    // Run simulation for 100 ticks (5 seconds at 20 FPS)
    println!("Running synchronized simulation...");
    println!("┌──────┬────────────────────────────┬────────────┬──────────────┐");
    println!("│ Tick │      Vehicle Location      │ Speed      │ Sensor Data  │");
    println!("├──────┼────────────────────────────┼────────────┼──────────────┤");

    for tick in 0..100 {
        // Tick the world (advance simulation by one frame)
        world.tick();

        // Display status every 10 ticks
        if tick % 10 == 0 {
            let location = vehicle.transform().location;
            let velocity = vehicle.velocity();
            let speed = (velocity.x.powi(2) + velocity.y.powi(2) + velocity.z.powi(2)).sqrt();
            let rgb_frames = *rgb_frame_count.lock().unwrap();
            let seg_frames = *seg_frame_count.lock().unwrap();

            println!(
                "│ {:>4} │ ({:>7.1}, {:>7.1}, {:>5.1}) │ {:>5.1} km/h │ RGB:{:>3} SEG:{:>3} │",
                tick,
                location.x,
                location.y,
                location.z,
                speed * 3.6,
                rgb_frames,
                seg_frames
            );
        }

        // Small sleep to prevent busy-waiting
        thread::sleep(Duration::from_millis(10));
    }

    println!("└──────┴────────────────────────────┴────────────┴──────────────┘");

    // Final statistics
    let final_rgb_frames = *rgb_frame_count.lock().unwrap();
    let final_seg_frames = *seg_frame_count.lock().unwrap();

    println!("\n=== Synchronous Mode Statistics ===");
    println!("Total ticks: 100");
    println!("Fixed delta: 0.05s (20 FPS)");
    println!("Simulated time: 5.0 seconds");
    println!("RGB camera frames: {}", final_rgb_frames);
    println!("Semantic camera frames: {}", final_seg_frames);
    println!(
        "Frame synchronization: {}",
        if final_rgb_frames == final_seg_frames {
            "✓ Perfect"
        } else {
            "⚠ Drift detected"
        }
    );

    // Restore asynchronous mode
    println!("\nRestoring asynchronous mode...");
    camera_rgb.stop();
    camera_seg.stop();

    settings.synchronous_mode = false;
    settings.fixed_delta_seconds = None;
    world.apply_settings(&settings, Duration::from_secs(5));
    println!("✓ Asynchronous mode restored");

    // Cleanup
    println!("\nCleaning up...");
    camera_rgb.destroy();
    camera_seg.destroy();
    vehicle.destroy();
    println!("✓ All actors destroyed");

    println!("\n=== Synchronous Mode Demo Complete ===");
    println!("\nKey Features Demonstrated:");
    println!("  • Synchronous mode configuration");
    println!("  • Fixed timestep simulation (deterministic)");
    println!("  • Multiple synchronized sensors");
    println!("  • Tick-based simulation control");
    println!("  • Frame synchronization verification");
    println!("  • Mode restoration and cleanup");

    Ok(())
}
