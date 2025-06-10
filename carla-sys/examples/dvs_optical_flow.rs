//! Example demonstrating DVS (Dynamic Vision Sensor) and Optical Flow sensors
//!
//! This example shows how to:
//! - Connect to CARLA simulator
//! - Spawn DVS and optical flow sensors
//! - Process sensor data through callbacks
//! - Analyze DVS events and optical flow data

use carla_sys::*;
use std::{
    ffi::{c_void, CString},
    ptr, thread,
    time::Duration,
};

/// DVS sensor callback
unsafe extern "C" fn dvs_callback(data: *mut carla_sensor_data_t, _user_data: *mut c_void) {
    println!("DVS callback triggered!");

    let data_type = carla_sensor_data_get_type(data);
    if data_type == carla_sensor_data_type_t_CARLA_SENSOR_DATA_DVS_EVENT_ARRAY {
        let mut event_count: usize = 0;
        let events = carla_dvs_get_events(data, &mut event_count);

        println!("DVS Events: {} total events", event_count);

        if !events.is_null() && event_count > 0 {
            // Analyze events
            let mut analysis: carla_dvs_analysis_t = std::mem::zeroed();
            let error = carla_dvs_analyze_events(data, &mut analysis);

            if error == carla_error_t_CARLA_ERROR_NONE {
                println!("  Positive events: {}", analysis.positive_events);
                println!("  Negative events: {}", analysis.negative_events);
                println!("  Event rate: {:.2}", analysis.event_rate);
                println!("  Activity density: {:.4}", analysis.activity_density);
            }

            // Convert events to image
            let mut image_data: *mut u8 = ptr::null_mut();
            let mut image_size: usize = 0;
            let time_window = 0.1; // 100ms
            let error =
                carla_dvs_events_to_image(data, time_window, &mut image_data, &mut image_size);

            if error == carla_error_t_CARLA_ERROR_NONE && !image_data.is_null() {
                println!("  Converted to image: {} bytes", image_size);
                carla_image_free_data(image_data);
            }
        }
    }
}

/// Optical flow sensor callback
unsafe extern "C" fn optical_flow_callback(
    data: *mut carla_sensor_data_t,
    _user_data: *mut c_void,
) {
    println!("Optical flow callback triggered!");

    let data_type = carla_sensor_data_get_type(data);
    if data_type == carla_sensor_data_type_t_CARLA_SENSOR_DATA_OPTICAL_FLOW_IMAGE {
        let _flow_data = carla_sensor_data_get_optical_flow(data);
        let width = carla_optical_flow_data_get_width(data);
        let height = carla_optical_flow_data_get_height(data);

        println!("Optical Flow: {}x{} pixels", width, height);

        // Analyze optical flow
        let mut analysis: carla_optical_flow_analysis_t = std::mem::zeroed();
        let error = carla_image_analyze_optical_flow(data, &mut analysis);

        if error == carla_error_t_CARLA_ERROR_NONE {
            println!(
                "  Average flow: ({:.3}, {:.3})",
                analysis.average_flow.x, analysis.average_flow.y
            );
            println!(
                "  Motion magnitude avg: {:.3}, max: {:.3}",
                analysis.magnitude_avg, analysis.magnitude_max
            );
            println!(
                "  Moving pixels: {} ({:.2}%)",
                analysis.moving_pixels,
                analysis.motion_density * 100.0
            );
            println!(
                "  Dominant direction: ({:.3}, {:.3})",
                analysis.dominant_direction.x, analysis.dominant_direction.y
            );
        }

        // Get magnitude data
        let mut magnitude_data: *mut f32 = ptr::null_mut();
        let mut data_size: usize = 0;
        let error = carla_image_optical_flow_magnitude(data, &mut magnitude_data, &mut data_size);

        if error == carla_error_t_CARLA_ERROR_NONE && !magnitude_data.is_null() {
            println!("  Magnitude data: {} floats", data_size);
            carla_optical_flow_free_data(magnitude_data);
        }
    }
}

fn main() {
    unsafe {
        let host = CString::new("localhost").unwrap();
        let port = 2000;
        let timeout = 0;

        println!("=== DVS and Optical Flow Example ===");

        // Connect to CARLA server
        println!("Connecting to CARLA server at {}:{}...", "localhost", port);
        let client = carla_client_new(host.as_ptr(), port, timeout);
        if client.is_null() {
            eprintln!("Failed to connect to CARLA server");
            return;
        }

        carla_client_set_timeout(client, 10000);

        // Get world
        let world = carla_client_get_world(client);
        if world.is_null() {
            eprintln!("Failed to get world");
            carla_client_free(client);
            return;
        }

        // Get blueprint library
        let blueprints = carla_world_get_blueprint_library(world);
        if blueprints.is_null() {
            eprintln!("Failed to get blueprint library");
            carla_world_free(world);
            carla_client_free(client);
            return;
        }

        // Get spawn points
        let map = carla_world_get_map(world);
        let spawn_points = carla_map_get_spawn_points(map);
        if spawn_points.is_null() || carla_transform_list_size(spawn_points) == 0 {
            eprintln!("No spawn points available");
            carla_map_free(map);
            carla_world_free(world);
            carla_client_free(client);
            return;
        }

        let mut spawn_point = carla_transform_list_get(spawn_points, 0);
        spawn_point.location.z += 2.0; // Mount sensors higher

        println!(
            "Spawn point: ({:.2}, {:.2}, {:.2})",
            spawn_point.location.x, spawn_point.location.y, spawn_point.location.z
        );

        // Create DVS sensor
        let dvs_bp_id = CString::new("sensor.camera.dvs").unwrap();
        let dvs_bp = carla_blueprint_library_find(blueprints, dvs_bp_id.as_ptr());
        if !dvs_bp.is_null() {
            println!("Creating DVS sensor...");
            let dvs_result = carla_world_spawn_actor(world, dvs_bp, &spawn_point, ptr::null_mut());

            if dvs_result.error == carla_error_t_CARLA_ERROR_NONE && !dvs_result.actor.is_null() {
                println!("DVS sensor spawned successfully");

                // Set up DVS callback
                let dvs_sensor = carla_actor_as_sensor(dvs_result.actor);
                if !dvs_sensor.is_null() {
                    carla_sensor_listen(dvs_sensor, Some(dvs_callback), ptr::null_mut());
                    println!("DVS sensor listening...");
                }
            } else {
                eprintln!("Failed to spawn DVS sensor");
            }
        } else {
            eprintln!("DVS sensor blueprint not found");
        }

        // Create optical flow sensor
        spawn_point.location.y += 1.0; // Offset slightly
        let flow_bp_id = CString::new("sensor.camera.optical_flow").unwrap();
        let flow_bp = carla_blueprint_library_find(blueprints, flow_bp_id.as_ptr());
        if !flow_bp.is_null() {
            println!("Creating optical flow sensor...");
            let flow_result =
                carla_world_spawn_actor(world, flow_bp, &spawn_point, ptr::null_mut());

            if flow_result.error == carla_error_t_CARLA_ERROR_NONE && !flow_result.actor.is_null() {
                println!("Optical flow sensor spawned successfully");

                // Set up optical flow callback
                let flow_sensor = carla_actor_as_sensor(flow_result.actor);
                if !flow_sensor.is_null() {
                    carla_sensor_listen(flow_sensor, Some(optical_flow_callback), ptr::null_mut());
                    println!("Optical flow sensor listening...");
                }
            } else {
                eprintln!("Failed to spawn optical flow sensor");
            }
        } else {
            eprintln!("Optical flow sensor blueprint not found");
        }

        // Run simulation for a few ticks
        println!("Collecting sensor data for 10 ticks...");
        for i in 0..10 {
            carla_world_tick(world, 5000); // 5 second timeout
            thread::sleep(Duration::from_secs(1));
            println!("Tick {}/10", i + 1);
        }

        println!("Example completed!");

        // Cleanup
        carla_transform_list_free(spawn_points);
        carla_map_free(map);
        carla_blueprint_library_free(blueprints);
        carla_world_free(world);
        carla_client_free(client);
    }
}
