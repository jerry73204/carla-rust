//! Comprehensive sensor system demonstration for carla-cxx.
//!
//! This example shows how to:
//! - Connect to CARLA server
//! - Spawn different types of sensors (Camera, LiDAR, Radar, IMU, GNSS)
//! - Set up sensor callbacks
//! - Process different sensor data types
//! - Implement sensor data filtering and analysis

use anyhow::Result;
use carla_cxx::{
    ClientWrapper, SensorData, SensorWrapper, SimpleLocation, SimpleRotation, SimpleTransform,
};
use std::{
    sync::{Arc, Mutex},
    thread,
    time::Duration,
};

/// Sensor data processor for demonstration
struct SensorDataProcessor {
    image_count: Arc<Mutex<u32>>,
    lidar_count: Arc<Mutex<u32>>,
    radar_count: Arc<Mutex<u32>>,
    imu_count: Arc<Mutex<u32>>,
    gnss_count: Arc<Mutex<u32>>,
}

impl SensorDataProcessor {
    fn new() -> Self {
        Self {
            image_count: Arc::new(Mutex::new(0)),
            lidar_count: Arc::new(Mutex::new(0)),
            radar_count: Arc::new(Mutex::new(0)),
            imu_count: Arc::new(Mutex::new(0)),
            gnss_count: Arc::new(Mutex::new(0)),
        }
    }

    fn process_data(&self, data: SensorData) {
        match data {
            SensorData::Image(image_data) => {
                let mut count = self.image_count.lock().unwrap();
                *count += 1;
                println!(
                    "📷 Image #{}: {}x{} pixels, FOV: {:.1}°, Size: {} bytes",
                    *count,
                    image_data.width,
                    image_data.height,
                    image_data.fov,
                    image_data.data.len()
                );

                // Convert to RGBA for processing (example)
                let rgba_data = image_data.as_rgba();
                println!("   Converted to RGBA format: {} bytes", rgba_data.len());
            }
            SensorData::LiDAR(lidar_data) => {
                let mut count = self.lidar_count.lock().unwrap();
                *count += 1;
                println!("🎯 LiDAR #{}: {} points", *count, lidar_data.point_count);

                // Filter points by distance
                let filtered = lidar_data.filter_by_distance(50.0);
                println!(
                    "   Points within 50m: {} (filtered from {})",
                    filtered.point_count, lidar_data.point_count
                );

                // Filter by intensity
                let high_intensity = lidar_data.filter_by_intensity(0.5);
                println!(
                    "   High intensity points (>0.5): {}",
                    high_intensity.point_count
                );

                // Get points in a box around the sensor
                let nearby_points =
                    lidar_data.get_points_in_box(-10.0, 10.0, -10.0, 10.0, -2.0, 2.0);
                println!("   Points in nearby box: {}", nearby_points.point_count);
            }
            SensorData::Radar(radar_data) => {
                let mut count = self.radar_count.lock().unwrap();
                *count += 1;
                println!(
                    "📡 Radar #{}: {} detections",
                    *count,
                    radar_data.detections.len()
                );

                // Filter by velocity (moving objects)
                let moving_objects = radar_data.filter_by_velocity(1.0); // 1 m/s threshold
                println!(
                    "   Moving objects (>1 m/s): {}",
                    moving_objects.detections.len()
                );

                // Filter by distance
                let nearby_objects = radar_data.filter_by_distance(100.0);
                println!(
                    "   Objects within 100m: {}",
                    nearby_objects.detections.len()
                );

                // Show detection details
                for (i, detection) in radar_data.detections.iter().take(3).enumerate() {
                    println!(
                        "   Detection {}: vel={:.2} m/s, depth={:.1}m, azimuth={:.2}°",
                        i + 1,
                        detection.velocity,
                        detection.depth,
                        detection.azimuth.to_degrees()
                    );
                }
            }
            SensorData::IMU(imu_data) => {
                let mut count = self.imu_count.lock().unwrap();
                *count += 1;
                if *count % 10 == 0 {
                    // Print every 10th IMU reading to avoid spam
                    println!(
                        "🧭 IMU #{}: accel_mag={:.2} m/s², gyro_mag={:.2} rad/s, compass={:.1}°",
                        *count,
                        imu_data.acceleration_magnitude(),
                        imu_data.angular_velocity_magnitude(),
                        imu_data.compass.to_degrees()
                    );
                }
            }
            SensorData::GNSS(gnss_data) => {
                let mut count = self.gnss_count.lock().unwrap();
                *count += 1;
                if *count % 5 == 0 {
                    // Print every 5th GNSS reading
                    println!(
                        "🌍 GNSS #{}: lat={:.6}°, lon={:.6}°, alt={:.1}m",
                        *count, gnss_data.latitude, gnss_data.longitude, gnss_data.altitude
                    );
                }
            }
            SensorData::Raw(raw_data) => {
                println!("📊 Raw sensor data: {} bytes", raw_data.len());
            }
            SensorData::Collision(collision) => {
                println!(
                    "💥 Collision detected! Other actor: {}, impulse: {:.2} N",
                    collision.other_actor_id,
                    collision.impulse_magnitude()
                );
            }
            SensorData::LaneInvasion(lane_invasion) => {
                println!(
                    "🚨 Lane invasion detected! Crossings: {}",
                    lane_invasion.crossed_lane_markings.len()
                );
            }
        }
    }

    fn print_statistics(&self) {
        println!("\n📊 Sensor Statistics:");
        println!("   Images processed: {}", *self.image_count.lock().unwrap());
        println!(
            "   LiDAR scans processed: {}",
            *self.lidar_count.lock().unwrap()
        );
        println!(
            "   Radar detections processed: {}",
            *self.radar_count.lock().unwrap()
        );
        println!(
            "   IMU readings processed: {}",
            *self.imu_count.lock().unwrap()
        );
        println!(
            "   GNSS readings processed: {}",
            *self.gnss_count.lock().unwrap()
        );
    }
}

fn main() -> Result<()> {
    println!("🚗 CARLA Sensor System Demo");
    println!("==============================");

    // Connect to CARLA server
    println!("🔌 Connecting to CARLA server...");
    let mut client = ClientWrapper::new("localhost", 2000)?;
    client.set_timeout(Duration::from_secs(10));

    println!("📡 Server version: {}", client.get_server_version());

    // Get world and blueprint library
    let world = client.get_world();
    let blueprint_library = world.get_blueprint_library();

    println!("🌍 Connected to world with ID: {}", world.get_id());

    // Create sensor data processor
    let processor = Arc::new(SensorDataProcessor::new());

    // Define spawn point for sensors
    let spawn_point = SimpleTransform::new(
        SimpleLocation::new(0.0, 0.0, 2.0),
        SimpleRotation::new(0.0, 0.0, 0.0),
    );

    println!("\n🎯 Setting up sensors...");

    // Try to spawn different types of sensors
    let mut sensors = Vec::new();

    // 1. RGB Camera
    if let Some(camera_bp) = blueprint_library.find("sensor.camera.rgb") {
        println!("📷 Setting up RGB camera...");
        if let Ok(camera_actor) = world.spawn_actor(&camera_bp, &spawn_point, None) {
            if let Some(mut camera_sensor) = SensorWrapper::from_actor(camera_actor.get_actor()) {
                let processor_clone = Arc::clone(&processor);
                camera_sensor.listen(move |data| {
                    processor_clone.process_data(data);
                })?;
                sensors.push(camera_sensor);
                println!("   ✅ RGB camera active");
            }
        }
    }

    // 2. LiDAR
    if let Some(lidar_bp) = blueprint_library.find("sensor.lidar.ray_cast") {
        println!("🎯 Setting up LiDAR...");
        if let Ok(lidar_actor) = world.spawn_actor(&lidar_bp, &spawn_point, None) {
            if let Some(mut lidar_sensor) = SensorWrapper::from_actor(lidar_actor.get_actor()) {
                let processor_clone = Arc::clone(&processor);
                lidar_sensor.listen(move |data| {
                    processor_clone.process_data(data);
                })?;
                sensors.push(lidar_sensor);
                println!("   ✅ LiDAR active");
            }
        }
    }

    // 3. Radar
    if let Some(radar_bp) = blueprint_library.find("sensor.other.radar") {
        println!("📡 Setting up Radar...");
        if let Ok(radar_actor) = world.spawn_actor(&radar_bp, &spawn_point, None) {
            if let Some(mut radar_sensor) = SensorWrapper::from_actor(radar_actor.get_actor()) {
                let processor_clone = Arc::clone(&processor);
                radar_sensor.listen(move |data| {
                    processor_clone.process_data(data);
                })?;
                sensors.push(radar_sensor);
                println!("   ✅ Radar active");
            }
        }
    }

    // 4. IMU
    if let Some(imu_bp) = blueprint_library.find("sensor.other.imu") {
        println!("🧭 Setting up IMU...");
        if let Ok(imu_actor) = world.spawn_actor(&imu_bp, &spawn_point, None) {
            if let Some(mut imu_sensor) = SensorWrapper::from_actor(imu_actor.get_actor()) {
                let processor_clone = Arc::clone(&processor);
                imu_sensor.listen(move |data| {
                    processor_clone.process_data(data);
                })?;
                sensors.push(imu_sensor);
                println!("   ✅ IMU active");
            }
        }
    }

    // 5. GNSS
    if let Some(gnss_bp) = blueprint_library.find("sensor.other.gnss") {
        println!("🌍 Setting up GNSS...");
        if let Ok(gnss_actor) = world.spawn_actor(&gnss_bp, &spawn_point, None) {
            if let Some(mut gnss_sensor) = SensorWrapper::from_actor(gnss_actor.get_actor()) {
                let processor_clone = Arc::clone(&processor);
                gnss_sensor.listen(move |data| {
                    processor_clone.process_data(data);
                })?;
                sensors.push(gnss_sensor);
                println!("   ✅ GNSS active");
            }
        }
    }

    println!("\n🎯 {} sensors active, collecting data...", sensors.len());
    println!("⏰ Running for 30 seconds...");

    // Main polling loop - this processes all sensor callbacks
    println!("\n📊 Polling sensor data for 30 seconds...");
    let start_time = std::time::Instant::now();
    let mut poll_count = 0;

    while start_time.elapsed() < Duration::from_secs(30) {
        let mut data_received = false;

        // Poll all sensors
        for sensor in &sensors {
            if let Ok(got_data) = sensor.poll() {
                if got_data {
                    data_received = true;
                }
            }
        }

        if data_received {
            poll_count += 1;
        }

        // Small sleep to avoid busy waiting
        thread::sleep(Duration::from_millis(10));

        // Print progress every 5 seconds
        if start_time.elapsed().as_secs() % 5 == 0 && start_time.elapsed().as_millis() % 5000 < 50 {
            println!(
                "   📈 Processed {} sensor data packets so far...",
                poll_count
            );
        }
    }

    // Print final statistics
    processor.print_statistics();

    // Stop all sensors
    println!("\n🛑 Stopping sensors...");
    for sensor in &sensors {
        sensor.stop();
    }

    println!("✅ Sensor demo completed!");
    Ok(())
}
