//! Example demonstrating Ackermann steering control in CARLA 0.10.0
//!
//! This example shows how to:
//! - Use the new Ackermann steering model for precise vehicle control
//! - Configure PID controller parameters for different driving scenarios
//! - Compare Ackermann control with traditional throttle/brake/steer control

use carla_sys::*;
use std::{
    ffi::CString,
    ptr,
    thread,
    time::Duration,
};

fn main() {
    println!("=== CARLA 0.10.0 Ackermann Control Example ===");
    println!();

    unsafe {
        // Connect to CARLA server
        let host = CString::new("localhost").unwrap();
        let client = carla_client_new(host.as_ptr(), 2000, 0);
        
        if client.is_null() {
            eprintln!("Failed to connect to CARLA server");
            eprintln!("Make sure CARLA is running on localhost:2000");
            return;
        }

        carla_client_set_timeout(client, 10000);
        println!("✅ Connected to CARLA server");

        // Get world and blueprint library
        let world = carla_client_get_world(client);
        if world.is_null() {
            eprintln!("Failed to get world");
            carla_client_free(client);
            return;
        }

        let blueprints = carla_world_get_blueprint_library(world);
        if blueprints.is_null() {
            eprintln!("Failed to get blueprint library");
            carla_world_free(world);
            carla_client_free(client);
            return;
        }

        // Find a vehicle blueprint
        let vehicle_bp_id = CString::new("vehicle.tesla.model3").unwrap();
        let vehicle_bp = carla_blueprint_library_find(blueprints, vehicle_bp_id.as_ptr());
        
        if vehicle_bp.is_null() {
            eprintln!("Tesla Model 3 blueprint not found, trying default vehicle");
            let default_bp_id = CString::new("vehicle.*").unwrap();
            let mut count = 0;
            let vehicle_bp = carla_blueprint_library_filter(blueprints, default_bp_id.as_ptr(), &mut count);
            if vehicle_bp.is_null() {
                eprintln!("No vehicle blueprints found");
                carla_blueprint_library_free(blueprints);
                carla_world_free(world);
                carla_client_free(client);
                return;
            }
        }

        // Get spawn points
        let map = carla_world_get_map(world);
        let spawn_points = carla_map_get_spawn_points(map);
        if spawn_points.is_null() || carla_transform_list_size(spawn_points) == 0 {
            eprintln!("No spawn points available");
            carla_map_free(map);
            carla_blueprint_library_free(blueprints);
            carla_world_free(world);
            carla_client_free(client);
            return;
        }

        let spawn_point = carla_transform_list_get(spawn_points, 0);
        println!("🚗 Spawning vehicle at ({:.1}, {:.1}, {:.1})", 
                 spawn_point.location.x, spawn_point.location.y, spawn_point.location.z);

        // Spawn vehicle
        let spawn_result = carla_world_spawn_actor(world, vehicle_bp, &spawn_point, ptr::null_mut());
        if spawn_result.error != carla_error_t_CARLA_ERROR_NONE || spawn_result.actor.is_null() {
            eprintln!("Failed to spawn vehicle");
            carla_transform_list_free(spawn_points);
            carla_map_free(map);
            carla_blueprint_library_free(blueprints);
            carla_world_free(world);
            carla_client_free(client);
            return;
        }

        println!("✅ Vehicle spawned successfully");

        let vehicle = carla_actor_as_vehicle(spawn_result.actor);
        if vehicle.is_null() {
            eprintln!("Failed to cast actor to vehicle");
            carla_actor_destroy(spawn_result.actor);
            carla_transform_list_free(spawn_points);
            carla_map_free(map);
            carla_blueprint_library_free(blueprints);
            carla_world_free(world);
            carla_client_free(client);
            return;
        }

        // Configure Ackermann controller settings
        let controller_settings = carla_ackermann_controller_settings_t {
            speed_kp: 1.0,    // Speed proportional gain
            speed_ki: 0.2,    // Speed integral gain  
            speed_kd: 0.05,   // Speed derivative gain
            accel_kp: 0.8,    // Acceleration proportional gain
            accel_ki: 0.15,   // Acceleration integral gain
            accel_kd: 0.03,   // Acceleration derivative gain
        };

        println!("🎛️  Configuring Ackermann controller:");
        println!("   Speed PID: P={:.2}, I={:.2}, D={:.2}", 
                 controller_settings.speed_kp, controller_settings.speed_ki, controller_settings.speed_kd);
        println!("   Accel PID: P={:.2}, I={:.2}, D={:.2}",
                 controller_settings.accel_kp, controller_settings.accel_ki, controller_settings.accel_kd);

        // Apply controller settings (this would be a real API call)
        // carla_vehicle_set_ackermann_controller_settings(vehicle, &controller_settings);

        println!();
        println!("🎯 Testing different Ackermann control scenarios:");

        // Scenario 1: Gentle acceleration with straight driving
        println!("\n1. Gentle acceleration - straight driving");
        let gentle_control = carla_vehicle_ackermann_control_t {
            steer: 0.0,       // Straight ahead
            steer_speed: 0.0, // No steering motion
            speed: 10.0,      // 10 m/s target speed (36 km/h)
            acceleration: 1.0, // Gentle 1 m/s² acceleration
            jerk: 0.5,        // Smooth 0.5 m/s³ jerk limit
        };

        apply_ackermann_control_with_feedback(vehicle, &gentle_control, "gentle acceleration");
        thread::sleep(Duration::from_secs(2));

        // Scenario 2: Moderate left turn
        println!("\n2. Moderate left turn");
        let left_turn_control = carla_vehicle_ackermann_control_t {
            steer: 15.0,      // 15° left turn
            steer_speed: 30.0, // 30°/s steering speed
            speed: 8.0,       // Reduce speed for turn
            acceleration: 0.0, // Maintain current speed
            jerk: 0.3,        // Lower jerk for smoother turn
        };

        apply_ackermann_control_with_feedback(vehicle, &left_turn_control, "left turn");
        thread::sleep(Duration::from_secs(3));

        // Scenario 3: Sharp right turn with deceleration
        println!("\n3. Sharp right turn with deceleration");
        let right_turn_control = carla_vehicle_ackermann_control_t {
            steer: -25.0,     // 25° right turn
            steer_speed: 45.0, // Faster steering for sharp turn
            speed: 5.0,       // Slow down for sharp turn
            acceleration: -1.5, // Decelerate
            jerk: 0.8,        // Higher jerk allowed for emergency-like maneuver
        };

        apply_ackermann_control_with_feedback(vehicle, &right_turn_control, "sharp right turn");
        thread::sleep(Duration::from_secs(3));

        // Scenario 4: Return to straight with acceleration
        println!("\n4. Return to straight with acceleration");
        let straight_accel_control = carla_vehicle_ackermann_control_t {
            steer: 0.0,       // Straighten out
            steer_speed: 60.0, // Quick steering correction
            speed: 15.0,      // Higher target speed
            acceleration: 2.0, // Stronger acceleration
            jerk: 1.0,        // Allow higher jerk for performance
        };

        apply_ackermann_control_with_feedback(vehicle, &straight_accel_control, "straight acceleration");
        thread::sleep(Duration::from_secs(3));

        // Scenario 5: Emergency braking
        println!("\n5. Emergency braking");
        let emergency_brake_control = carla_vehicle_ackermann_control_t {
            steer: 0.0,       // Keep straight
            steer_speed: 0.0, // No steering
            speed: 0.0,       // Target zero speed
            acceleration: -8.0, // Emergency braking
            jerk: 3.0,        // High jerk allowed for emergency
        };

        apply_ackermann_control_with_feedback(vehicle, &emergency_brake_control, "emergency braking");
        thread::sleep(Duration::from_secs(3));

        println!("\n🏁 Ackermann control demonstration completed!");

        // Compare with traditional control
        println!("\n📊 Comparison: Ackermann vs Traditional Control");
        println!("Ackermann advantages:");
        println!("  • Precise steering angle control (degrees)");
        println!("  • Speed-based control instead of throttle/brake");
        println!("  • Built-in jerk limiting for passenger comfort");
        println!("  • Better for autonomous driving algorithms");
        println!("  • More predictable vehicle behavior");
        println!();
        println!("Traditional control:");
        println!("  • Direct throttle/brake/steer inputs (0-1 range)");
        println!("  • More intuitive for human drivers");
        println!("  • Lower-level control for racing scenarios");

        // Cleanup
        carla_actor_destroy(spawn_result.actor);
        carla_transform_list_free(spawn_points);
        carla_map_free(map);
        carla_blueprint_library_free(blueprints);
        carla_world_free(world);
        carla_client_free(client);

        println!("\n🧹 Cleanup completed");
    }
}

unsafe fn apply_ackermann_control_with_feedback(
    vehicle: *mut carla_vehicle_t,
    control: &carla_vehicle_ackermann_control_t,
    scenario_name: &str,
) {
    println!("   Applying {} control:", scenario_name);
    println!("     Steer: {:.1}° @ {:.1}°/s", control.steer, control.steer_speed);
    println!("     Speed: {:.1} m/s", control.speed);
    println!("     Acceleration: {:.1} m/s²", control.acceleration);
    println!("     Jerk limit: {:.1} m/s³", control.jerk);

    // Apply the Ackermann control
    let result = carla_vehicle_apply_ackermann_control(vehicle, control);
    
    if result == carla_error_t_CARLA_ERROR_NONE {
        println!("     ✅ Control applied successfully");
    } else {
        println!("     ❌ Failed to apply control (error: {})", result);
    }

    // In a real scenario, you would monitor the vehicle's response:
    // - Get current transform: carla_actor_get_transform(vehicle as *mut carla_actor_t)
    // - Get current velocity: carla_vehicle_get_velocity(vehicle)
    // - Monitor control effectiveness and adjust PID parameters if needed
}