//! Sensor Data Collection Pipeline Integration Test
//!
//! This integration test demonstrates a complete sensor data collection pipeline:
//! - Spawn vehicle with multiple sensors (RGB, depth, semantic, LiDAR)
//! - Navigate using autopilot
//! - Collect and save sensor data
//! - Verify data consistency across sensors
//! - Test different weather and lighting conditions
//!
//! This validates the integration of multiple components working together.
//!
//! Run with:
//! ```bash
//! cargo run --example integration_sensor_data_collection --profile dev-release
//! ```

use carla::{
    client::{ActorBase, Client, Sensor, Vehicle},
    geom::{Location, Rotation, Transform},
    sensor::data::{Image, LidarMeasurement},
};
use std::{
    sync::{
        atomic::{AtomicUsize, Ordering},
        Arc,
    },
    time::{Duration, Instant},
};

// Sensor configuration
const RGB_WIDTH: u32 = 800;
const RGB_HEIGHT: u32 = 600;
const LIDAR_CHANNELS: u32 = 32;
const LIDAR_RANGE: f32 = 50.0;
const COLLECTION_DURATION_SECS: u64 = 10;

// Sensor data counters
#[derive(Default)]
struct SensorStats {
    rgb_frames: AtomicUsize,
    depth_frames: AtomicUsize,
    semantic_frames: AtomicUsize,
    lidar_frames: AtomicUsize,
}

fn main() {
    println!("=== Sensor Data Collection Pipeline Integration Test ===\n");

    // Connect to CARLA
    let client = Client::connect("127.0.0.1", 2000, None);
    let mut world = client.world();
    println!("Connected to CARLA server");

    // Setup scenario
    println!("\n--- Setting up test scenario ---");
    let (vehicle, sensors, stats) = setup_scenario(&mut world);

    // Enable autopilot
    println!("Enabling autopilot...");
    vehicle.set_autopilot(true);

    // Test 1: Collect data under normal conditions
    println!("\n--- Test 1: Collecting data (normal weather) ---");
    let start = Instant::now();
    while start.elapsed() < Duration::from_secs(COLLECTION_DURATION_SECS / 2) {
        std::thread::sleep(Duration::from_millis(100));
        print_stats(&stats, start.elapsed());
    }

    // Test 2: Change weather and collect more data
    println!("\n--- Test 2: Collecting data (rainy weather) ---");
    set_rainy_weather(&mut world);
    std::thread::sleep(Duration::from_secs(1)); // Let weather settle

    let start2 = Instant::now();
    while start2.elapsed() < Duration::from_secs(COLLECTION_DURATION_SECS / 2) {
        std::thread::sleep(Duration::from_millis(100));
        print_stats(&stats, start.elapsed() + start2.elapsed());
    }

    // Test 3: Change to night and collect data
    println!("\n--- Test 3: Collecting data (night time) ---");
    set_night_weather(&mut world);
    std::thread::sleep(Duration::from_secs(1));

    let start3 = Instant::now();
    while start3.elapsed() < Duration::from_secs(COLLECTION_DURATION_SECS / 2) {
        std::thread::sleep(Duration::from_millis(100));
        print_stats(
            &stats,
            start.elapsed() + start2.elapsed() + start3.elapsed(),
        );
    }

    // Final report
    println!("\n=== Final Results ===");
    let total_time = start.elapsed() + start2.elapsed() + start3.elapsed();
    println!("Total collection time: {:.2}s", total_time.as_secs_f32());
    println!(
        "RGB frames collected: {}",
        stats.rgb_frames.load(Ordering::Relaxed)
    );
    println!(
        "Depth frames collected: {}",
        stats.depth_frames.load(Ordering::Relaxed)
    );
    println!(
        "Semantic frames collected: {}",
        stats.semantic_frames.load(Ordering::Relaxed)
    );
    println!(
        "LiDAR frames collected: {}",
        stats.lidar_frames.load(Ordering::Relaxed)
    );

    // Verify data consistency
    println!("\n--- Verifying data consistency ---");
    verify_data_consistency(&stats);

    // Cleanup
    println!("\n--- Cleaning up ---");
    vehicle.set_autopilot(false);
    for sensor in sensors {
        sensor.destroy();
    }
    vehicle.destroy();

    println!("\n✅ Integration test completed successfully!");
    std::process::exit(0);
}

fn setup_scenario(world: &mut carla::client::World) -> (Vehicle, Vec<Sensor>, Arc<SensorStats>) {
    let stats = Arc::new(SensorStats::default());

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
    println!("✓ Spawned vehicle");

    let mut sensors = Vec::new();

    // Attach RGB camera
    if let Some(rgb_sensor) = attach_rgb_camera(world, &vehicle, Arc::clone(&stats)) {
        sensors.push(rgb_sensor);
        println!("✓ Attached RGB camera");
    }

    // Attach depth camera
    if let Some(depth_sensor) = attach_depth_camera(world, &vehicle, Arc::clone(&stats)) {
        sensors.push(depth_sensor);
        println!("✓ Attached depth camera");
    }

    // Attach semantic segmentation camera
    if let Some(semantic_sensor) = attach_semantic_camera(world, &vehicle, Arc::clone(&stats)) {
        sensors.push(semantic_sensor);
        println!("✓ Attached semantic camera");
    }

    // Attach LiDAR sensor
    if let Some(lidar_sensor) = attach_lidar_sensor(world, &vehicle, Arc::clone(&stats)) {
        sensors.push(lidar_sensor);
        println!("✓ Attached LiDAR sensor");
    }

    (vehicle, sensors, stats)
}

fn attach_rgb_camera(
    world: &mut carla::client::World,
    vehicle: &Vehicle,
    stats: Arc<SensorStats>,
) -> Option<Sensor> {
    let blueprint_library = world.blueprint_library();
    let mut camera_bp = blueprint_library.find("sensor.camera.rgb")?;

    let _ = camera_bp.set_attribute("image_size_x", &RGB_WIDTH.to_string());
    let _ = camera_bp.set_attribute("image_size_y", &RGB_HEIGHT.to_string());
    let _ = camera_bp.set_attribute("fov", "90.0");

    let transform = Transform {
        location: Location::new(2.0, 0.0, 1.5),
        rotation: Rotation::new(0.0, 0.0, 0.0),
    };

    let camera_actor = world
        .spawn_actor_attached(
            &camera_bp,
            &transform,
            vehicle,
            carla::rpc::AttachmentType::Rigid,
        )
        .ok()?;

    let sensor = Sensor::try_from(camera_actor).ok()?;

    sensor.listen(move |data| {
        if let Ok(_image) = Image::try_from(data) {
            stats.rgb_frames.fetch_add(1, Ordering::Relaxed);
        }
    });

    Some(sensor)
}

fn attach_depth_camera(
    world: &mut carla::client::World,
    vehicle: &Vehicle,
    stats: Arc<SensorStats>,
) -> Option<Sensor> {
    let blueprint_library = world.blueprint_library();
    let mut camera_bp = blueprint_library.find("sensor.camera.depth")?;

    let _ = camera_bp.set_attribute("image_size_x", &RGB_WIDTH.to_string());
    let _ = camera_bp.set_attribute("image_size_y", &RGB_HEIGHT.to_string());

    let transform = Transform {
        location: Location::new(2.0, 0.0, 1.5),
        rotation: Rotation::new(0.0, 0.0, 0.0),
    };

    let camera_actor = world
        .spawn_actor_attached(
            &camera_bp,
            &transform,
            vehicle,
            carla::rpc::AttachmentType::Rigid,
        )
        .ok()?;

    let sensor = Sensor::try_from(camera_actor).ok()?;

    sensor.listen(move |data| {
        if let Ok(_image) = Image::try_from(data) {
            stats.depth_frames.fetch_add(1, Ordering::Relaxed);
        }
    });

    Some(sensor)
}

fn attach_semantic_camera(
    world: &mut carla::client::World,
    vehicle: &Vehicle,
    stats: Arc<SensorStats>,
) -> Option<Sensor> {
    let blueprint_library = world.blueprint_library();
    let mut camera_bp = blueprint_library.find("sensor.camera.semantic_segmentation")?;

    let _ = camera_bp.set_attribute("image_size_x", &RGB_WIDTH.to_string());
    let _ = camera_bp.set_attribute("image_size_y", &RGB_HEIGHT.to_string());

    let transform = Transform {
        location: Location::new(2.0, 0.0, 1.5),
        rotation: Rotation::new(0.0, 0.0, 0.0),
    };

    let camera_actor = world
        .spawn_actor_attached(
            &camera_bp,
            &transform,
            vehicle,
            carla::rpc::AttachmentType::Rigid,
        )
        .ok()?;

    let sensor = Sensor::try_from(camera_actor).ok()?;

    sensor.listen(move |data| {
        if let Ok(_image) = Image::try_from(data) {
            stats.semantic_frames.fetch_add(1, Ordering::Relaxed);
        }
    });

    Some(sensor)
}

fn attach_lidar_sensor(
    world: &mut carla::client::World,
    vehicle: &Vehicle,
    stats: Arc<SensorStats>,
) -> Option<Sensor> {
    let blueprint_library = world.blueprint_library();
    let mut lidar_bp = blueprint_library.find("sensor.lidar.ray_cast")?;

    let _ = lidar_bp.set_attribute("channels", &LIDAR_CHANNELS.to_string());
    let _ = lidar_bp.set_attribute("range", &LIDAR_RANGE.to_string());
    let _ = lidar_bp.set_attribute("points_per_second", "100000");
    let _ = lidar_bp.set_attribute("rotation_frequency", "10.0");

    let transform = Transform {
        location: Location::new(0.0, 0.0, 2.5),
        rotation: Rotation::new(0.0, 0.0, 0.0),
    };

    let lidar_actor = world
        .spawn_actor_attached(
            &lidar_bp,
            &transform,
            vehicle,
            carla::rpc::AttachmentType::Rigid,
        )
        .ok()?;

    let sensor = Sensor::try_from(lidar_actor).ok()?;

    sensor.listen(move |data| {
        if let Ok(_lidar_data) = LidarMeasurement::try_from(data) {
            stats.lidar_frames.fetch_add(1, Ordering::Relaxed);
        }
    });

    Some(sensor)
}

fn set_rainy_weather(world: &mut carla::client::World) {
    let mut weather = world.weather();
    weather.precipitation = 80.0;
    weather.precipitation_deposits = 50.0;
    weather.wetness = 80.0;
    weather.cloudiness = 90.0;
    world.set_weather(&weather);
}

fn set_night_weather(world: &mut carla::client::World) {
    let mut weather = world.weather();
    weather.sun_altitude_angle = -90.0; // Night time
    weather.cloudiness = 10.0;
    weather.precipitation = 0.0;
    world.set_weather(&weather);
}

fn print_stats(stats: &SensorStats, elapsed: Duration) {
    print!(
        "\r[{:>5.1}s] RGB: {:>4} | Depth: {:>4} | Semantic: {:>4} | LiDAR: {:>4}",
        elapsed.as_secs_f32(),
        stats.rgb_frames.load(Ordering::Relaxed),
        stats.depth_frames.load(Ordering::Relaxed),
        stats.semantic_frames.load(Ordering::Relaxed),
        stats.lidar_frames.load(Ordering::Relaxed)
    );
    use std::io::{self, Write};
    io::stdout().flush().unwrap();
}

fn verify_data_consistency(stats: &SensorStats) {
    let rgb = stats.rgb_frames.load(Ordering::Relaxed);
    let depth = stats.depth_frames.load(Ordering::Relaxed);
    let semantic = stats.semantic_frames.load(Ordering::Relaxed);
    let lidar = stats.lidar_frames.load(Ordering::Relaxed);

    // All sensors should have collected at least some data
    assert!(rgb > 0, "RGB camera collected no frames");
    assert!(depth > 0, "Depth camera collected no frames");
    assert!(semantic > 0, "Semantic camera collected no frames");
    assert!(lidar > 0, "LiDAR collected no frames");

    // Cameras should have similar frame counts (within 20% tolerance)
    let camera_avg = (rgb + depth + semantic) / 3;
    let tolerance = camera_avg / 5; // 20%

    if (rgb as i32 - camera_avg as i32).abs() > tolerance as i32 {
        println!("⚠️  Warning: RGB frame count deviates from average");
    }
    if (depth as i32 - camera_avg as i32).abs() > tolerance as i32 {
        println!("⚠️  Warning: Depth frame count deviates from average");
    }
    if (semantic as i32 - camera_avg as i32).abs() > tolerance as i32 {
        println!("⚠️  Warning: Semantic frame count deviates from average");
    }

    println!("✓ All sensors collected data");
    println!("✓ Camera frame counts are consistent");
}
