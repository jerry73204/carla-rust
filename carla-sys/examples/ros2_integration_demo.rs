//! ROS2 Integration Demo for CARLA
//!
//! This example demonstrates how to use CARLA's simplified ROS2 integration features
//! for enabling sensors to publish data to ROS2 topics.
//!
//! # Prerequisites
//! - CARLA simulator running with ROS2 support enabled
//! - ROS2 environment set up (if you want to see the published topics)
//!
//! # Usage
//! ```bash
//! cargo run --example ros2_integration_demo
//! ```

use carla_sys::{
    ffi, ros2_utils, ClientWrapper, SensorROS2Ext, SimpleLocation, SimpleRotation, SimpleTransform,
};
use std::{thread, time::Duration};

fn main() -> anyhow::Result<()> {
    println!("🤖 CARLA ROS2 Integration Demo (Simplified)");
    println!("=============================================");

    // Connect to CARLA server
    println!("\n📡 Connecting to CARLA server...");
    let mut client = ClientWrapper::new("localhost", 2000)?;
    client.set_timeout(Duration::from_secs(10));

    let world = client.get_world()?;
    println!("✅ Connected to world: {}", world.get_map()?.get_name());

    println!("\n📋 Note: This demo shows simplified ROS2 sensor control.");
    println!("        For full ROS2 features, use the CARLA ROS2 bridge or Python API.");

    // Get blueprint library and find vehicle blueprint
    println!("\n🚗 Setting up test vehicle...");
    let blueprint_library = world.get_blueprint_library()?;
    let vehicle_bp = blueprint_library
        .find("vehicle.tesla.model3")
        .ok_or_else(|| anyhow::anyhow!("Tesla Model 3 blueprint not found"))?;

    // Set ROS2 name attribute on the vehicle blueprint
    vehicle_bp.set_attribute("ros_name", "ego_vehicle");
    vehicle_bp.set_attribute("role_name", "ego_vehicle");

    // Spawn the vehicle
    let spawn_points = world.get_map()?.get_recommended_spawn_points();
    let default_spawn =
        SimpleTransform::new(SimpleLocation::new(0.0, 0.0, 0.5), SimpleRotation::ZERO);
    let spawn_point = spawn_points.first().unwrap_or(&default_spawn);

    let vehicle = world.spawn_actor(&vehicle_bp, spawn_point, None)?;
    println!("✅ Spawned vehicle with ID: {}", vehicle.get_id());

    println!("\n📢 Note: ROS2 names are set via blueprint attributes (ros_name)");

    // Spawn sensors and enable them for ROS2
    println!("\n📷 Setting up sensors with ROS2 publishing...");

    // RGB Camera
    let camera_bp = blueprint_library
        .find("sensor.camera.rgb")
        .ok_or_else(|| anyhow::anyhow!("RGB camera blueprint not found"))?;

    camera_bp.set_attribute("image_size_x", "800");
    camera_bp.set_attribute("image_size_y", "600");
    camera_bp.set_attribute("fov", "90.0");
    camera_bp.set_attribute("ros_name", "rgb_front");

    let camera_transform = SimpleTransform::new(
        SimpleLocation::new(2.0, 0.0, 1.5), // Front of vehicle
        SimpleRotation::ZERO,
    );

    let camera = world.spawn_actor(&camera_bp, &camera_transform, Some(vehicle.get_actor()))?;
    println!("✅ Spawned RGB camera with ID: {}", camera.get_id());

    // Enable ROS2 for the camera
    let camera_sensor = ffi::Actor_CastToSensor(camera.get_shared_ptr());
    if let Some(sensor) = camera_sensor.as_ref() {
        sensor.enable_for_ros();
        println!("✅ Enabled ROS2 publishing for RGB camera");
        println!("   Expected topic: /carla/ego_vehicle/rgb_front/image");
    }

    // LiDAR Sensor
    let lidar_bp = blueprint_library
        .find("sensor.lidar.ray_cast")
        .ok_or_else(|| anyhow::anyhow!("LiDAR blueprint not found"))?;

    lidar_bp.set_attribute("range", "50.0");
    lidar_bp.set_attribute("rotation_frequency", "10.0");
    lidar_bp.set_attribute("channels", "32");
    lidar_bp.set_attribute("ros_name", "lidar");

    let lidar_transform = SimpleTransform::new(
        SimpleLocation::new(0.0, 0.0, 2.5), // Top of vehicle
        SimpleRotation::ZERO,
    );

    let lidar = world.spawn_actor(&lidar_bp, &lidar_transform, Some(vehicle.get_actor()))?;
    println!("✅ Spawned LiDAR with ID: {}", lidar.get_id());

    // Enable ROS2 for the LiDAR
    let lidar_sensor = ffi::Actor_CastToSensor(lidar.get_shared_ptr());
    if let Some(sensor) = lidar_sensor.as_ref() {
        sensor.enable_for_ros();
        println!("✅ Enabled ROS2 publishing for LiDAR");
        println!("   Expected topic: /carla/ego_vehicle/lidar/point_cloud");
    }

    // IMU Sensor
    let imu_bp = blueprint_library
        .find("sensor.other.imu")
        .ok_or_else(|| anyhow::anyhow!("IMU blueprint not found"))?;

    imu_bp.set_attribute("ros_name", "imu");

    let imu_transform = SimpleTransform::new(
        SimpleLocation::new(0.0, 0.0, 0.0),
        SimpleRotation::new(0.0, 0.0, 0.0),
    );
    let imu = world.spawn_actor(&imu_bp, &imu_transform, Some(vehicle.get_actor()))?;
    println!("✅ Spawned IMU with ID: {}", imu.get_id());

    // Enable ROS2 for the IMU
    let imu_sensor = ffi::Actor_CastToSensor(imu.get_shared_ptr());
    if let Some(sensor) = imu_sensor.as_ref() {
        sensor.enable_for_ros();
        println!("✅ Enabled ROS2 publishing for IMU");
        println!("   Expected topic: /carla/ego_vehicle/imu/data");
    }

    // Verify ROS2 status for all sensors
    println!("\n🔍 Verifying ROS2 status for all sensors...");
    let sensors = vec![
        camera_sensor.as_ref().unwrap(),
        lidar_sensor.as_ref().unwrap(),
        imu_sensor.as_ref().unwrap(),
    ];

    for (i, sensor) in sensors.iter().enumerate() {
        let sensor_name = match i {
            0 => "RGB Camera",
            1 => "LiDAR",
            2 => "IMU",
            _ => "Unknown",
        };
        println!(
            "   {} ROS2 enabled: {}",
            sensor_name,
            sensor.is_enabled_for_ros()
        );
    }

    // Use utility function to manage all sensors
    println!("\n🔄 Demonstrating utility functions...");

    // Disable all sensors
    ros2_utils::disable_sensors_for_ros2(&sensors);
    println!("✅ Disabled ROS2 for all sensors using utility function");

    // Verify they're disabled
    for (i, sensor) in sensors.iter().enumerate() {
        let sensor_name = match i {
            0 => "RGB Camera",
            1 => "LiDAR",
            2 => "IMU",
            _ => "Unknown",
        };
        println!(
            "   {} ROS2 enabled: {}",
            sensor_name,
            sensor.is_enabled_for_ros()
        );
    }

    // Re-enable all sensors
    ros2_utils::enable_sensors_for_ros2(&sensors);
    println!("✅ Re-enabled ROS2 for all sensors using utility function");

    // Run simulation with ROS2 publishing
    println!("\n🎮 Running simulation with ROS2 publishing...");
    println!("   Check your ROS2 topics with: ros2 topic list");
    println!("   Monitor camera images with: ros2 run image_view image_view image:=/carla/ego_vehicle/rgb_front/image");
    println!("   Monitor point cloud with: ros2 run rviz2 rviz2");

    for frame in 0..30 {
        // Tick the world (this will trigger sensor data publishing)
        world.tick(Duration::from_millis(50));

        if frame % 10 == 0 {
            println!("   Frame {frame}: Sensors publishing to ROS2 topics");
        }

        thread::sleep(Duration::from_millis(50));
    }

    // Demonstrate topic name generation
    println!("\n🏷️  Demonstrating topic name utilities...");
    let actor_name = "ego_vehicle";
    let sensor_types = [
        (ros2_utils::sensor_names::RGB_CAMERA, "rgb_front"),
        (ros2_utils::sensor_names::LIDAR, "lidar"),
        (ros2_utils::sensor_names::IMU, "imu"),
    ];

    for (sensor_type, sensor_name) in &sensor_types {
        let full_name = format!("{sensor_name}_{sensor_type}");
        let topic = ros2_utils::generate_topic_name(actor_name, &full_name);
        println!("   {sensor_type}: {topic}");
    }

    // Cleanup
    println!("\n🧹 Cleaning up...");

    // Disable ROS2 for all sensors
    ros2_utils::disable_sensors_for_ros2(&sensors);

    // Destroy actors
    camera.destroy();
    lidar.destroy();
    imu.destroy();
    vehicle.destroy();

    println!("✅ Destroyed all actors");
    println!("✅ Demo completed successfully!");

    println!("\n💡 Tips for using CARLA ROS2:");
    println!("   - Set 'ros_name' attribute on blueprints to control topic names");
    println!("   - Use sensor.enable_for_ros() / disable_for_ros() for individual control");
    println!("   - Use ros2_utils functions for bulk operations");
    println!("   - Monitor topics with standard ROS2 tools");
    println!("   - For advanced ROS2 features, use the official CARLA ROS2 bridge");

    Ok(())
}
