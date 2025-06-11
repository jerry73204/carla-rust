use carla_sys::*;
use std::ffi::CString;

extern "C" fn camera_callback(data: *mut carla_sensor_data_t, user_data: *mut std::ffi::c_void) {
    unsafe {
        if data.is_null() {
            println!("Received null sensor data");
            return;
        }

        // Get sensor data type
        let data_type = carla_sensor_data_get_type(data);
        println!("Received sensor data of type: {}", data_type);

        // Get basic info
        let frame = carla_sensor_data_get_frame(data);
        let timestamp = carla_sensor_data_get_timestamp(data);
        println!("Frame: {}, Timestamp: {:.6}s", frame, timestamp);

        // Handle different sensor types
        match data_type {
            carla_sensor_data_type_t_CARLA_SENSOR_DATA_IMAGE => {
                let image_data = carla_sensor_data_get_image(data);
                println!(
                    "Image: {}x{}, FOV: {}, Size: {} bytes",
                    image_data.width, image_data.height, image_data.fov, image_data.raw_data_size
                );
            }
            carla_sensor_data_type_t_CARLA_SENSOR_DATA_LIDAR => {
                let point_count = carla_lidar_data_get_point_count(data);
                let horizontal_angle = carla_lidar_data_get_horizontal_angle(data);
                let channels = carla_lidar_data_get_channels(data);
                println!(
                    "LiDAR: {} points, {} channels, {:.1}° horizontal angle",
                    point_count, channels, horizontal_angle
                );
            }
            carla_sensor_data_type_t_CARLA_SENSOR_DATA_IMU => {
                let accelerometer = carla_imu_data_get_accelerometer(data);
                let gyroscope = carla_imu_data_get_gyroscope(data);
                let compass = carla_imu_data_get_compass(data);
                println!("IMU: accel({:.2}, {:.2}, {:.2}) m/s², gyro({:.2}, {:.2}, {:.2}) rad/s, compass: {:.2}°", 
                    accelerometer.x, accelerometer.y, accelerometer.z,
                    gyroscope.x, gyroscope.y, gyroscope.z, compass);
            }
            carla_sensor_data_type_t_CARLA_SENSOR_DATA_GNSS => {
                let latitude = carla_gnss_data_get_latitude(data);
                let longitude = carla_gnss_data_get_longitude(data);
                let altitude = carla_gnss_data_get_altitude(data);
                println!(
                    "GNSS: lat: {:.6}°, lon: {:.6}°, alt: {:.2}m",
                    latitude, longitude, altitude
                );
            }
            carla_sensor_data_type_t_CARLA_SENSOR_DATA_COLLISION => {
                let actor = carla_collision_data_get_actor(data);
                let other_actor = carla_collision_data_get_other_actor(data);
                let normal_impulse = carla_collision_data_get_normal_impulse(data);
                println!(
                    "Collision: impulse({:.2}, {:.2}, {:.2})",
                    normal_impulse.x, normal_impulse.y, normal_impulse.z
                );

                if !actor.is_null() {
                    println!("  Actor involved: {:p}", actor);
                }
                if !other_actor.is_null() {
                    println!("  Other actor: {:p}", other_actor);
                }
            }
            _ => {
                println!("Unknown or unhandled sensor data type: {}", data_type);
            }
        }

        // Clean up sensor data (important!)
        carla_sensor_data_destroy(data);

        // User data example (if provided)
        if !user_data.is_null() {
            let counter = user_data as *mut u32;
            *counter += 1;
            println!("Processed {} sensor messages", *counter);
        }
    }
}

fn main() {
    println!("CARLA Sensor Callback Example");
    println!("==============================");
    println!();

    // This example demonstrates the sensor callback API structure
    // Note: This cannot actually connect to CARLA without a running server

    unsafe {
        // Create client connection
        let host = CString::new("localhost").unwrap();
        let client = carla_client_new(host.as_ptr(), 2000, 1);

        if client.is_null() {
            println!("❌ Failed to create CARLA client");
            return;
        }

        println!("✅ Created CARLA client");

        // Set reasonable timeout
        carla_client_set_timeout(client, 5000);
        println!("⏱️  Set client timeout to 5000ms");

        // Example of how sensor callback would work:
        // 1. Get world from client (carla_client_get_world)
        // 2. Get blueprint library (carla_world_get_blueprint_library)
        // 3. Find camera/sensor blueprint (carla_blueprint_library_find)
        // 4. Spawn sensor actor (carla_world_spawn_actor)
        // 5. Cast actor to sensor (carla_actor_as_sensor)
        // 6. Register callback (carla_sensor_listen)

        println!();
        println!("📷 Example workflow for sensor callbacks:");
        println!("1. Connect to CARLA server");
        println!("2. Get world and blueprint library");
        println!("3. Find sensor blueprint (e.g., 'sensor.camera.rgb')");
        println!("4. Set sensor attributes (resolution, FOV, etc.)");
        println!("5. Spawn sensor at desired location");
        println!("6. Register callback function with carla_sensor_listen()");
        println!("7. Process incoming sensor data in callback");
        println!("8. Clean up with carla_sensor_data_destroy()");

        // Counter for callback example
        let _callback_counter: u32 = 0;

        // Example of registering a callback (sensor would need to be valid)
        // carla_sensor_listen(sensor, Some(camera_callback), &mut callback_counter as *mut _ as *mut std::ffi::c_void);

        println!();
        println!("🔧 Callback function features:");
        println!("- Type-safe sensor data identification");
        println!("- Automatic conversion from C++ to C structures");
        println!("- Memory management with carla_sensor_data_destroy()");
        println!("- Support for user data (counters, contexts, etc.)");
        println!("- Thread-safe callback storage and cleanup");

        println!();
        println!("📊 Supported sensor types:");
        println!("- RGB/Depth/Semantic cameras (Image)");
        println!("- LiDAR point clouds");
        println!("- Semantic LiDAR with object tagging");
        println!("- Radar detections");
        println!("- IMU measurements (accelerometer, gyroscope, compass)");
        println!("- GNSS positioning (latitude, longitude, altitude)");
        println!("- Collision events");
        println!("- Lane invasion events");
        println!("- Obstacle detection events");
        println!("- DVS event arrays");

        // Clean up
        carla_client_free(client);
        println!();
        println!("🧹 Cleaned up CARLA client");
    }
}
