//! Example demonstrating vehicle door and light control in CARLA 0.10.0
//!
//! This example shows how to:
//! - Control individual vehicle doors (open/close)
//! - Manage complex vehicle lighting scenarios  
//! - Simulate realistic vehicle behaviors (parking, emergency, etc.)

use carla_sys::*;
use std::{ffi::CString, ptr, thread, time::Duration};

fn main() {
    println!("=== CARLA 0.10.0 Vehicle Doors & Lights Example ===");
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

        // Spawn a vehicle
        let vehicle = spawn_test_vehicle(world, blueprints);
        if vehicle.is_null() {
            eprintln!("Failed to spawn test vehicle");
            carla_blueprint_library_free(blueprints);
            carla_world_free(world);
            carla_client_free(client);
            return;
        }

        println!("✅ Vehicle spawned successfully");
        println!();

        // Demonstrate door controls
        demonstrate_door_controls(vehicle);
        thread::sleep(Duration::from_secs(2));

        // Demonstrate lighting scenarios
        demonstrate_lighting_scenarios(vehicle);
        thread::sleep(Duration::from_secs(2));

        // Demonstrate combined scenarios
        demonstrate_realistic_scenarios(vehicle);

        println!("\n🏁 Door and light control demonstration completed!");

        // Cleanup
        carla_actor_destroy(vehicle as *mut carla_actor_t);
        carla_blueprint_library_free(blueprints);
        carla_world_free(world);
        carla_client_free(client);

        println!("🧹 Cleanup completed");
    }
}

unsafe fn spawn_test_vehicle(
    world: *mut carla_world_t,
    blueprints: *mut carla_blueprint_library_t,
) -> *mut carla_vehicle_t {
    // Try to find a passenger car
    let car_bp_id = CString::new("vehicle.tesla.model3").unwrap();
    let mut vehicle_bp = carla_blueprint_library_find(blueprints, car_bp_id.as_ptr());

    if vehicle_bp.is_null() {
        // Fallback to any vehicle
        let any_vehicle_id = CString::new("vehicle.*").unwrap();
        let mut count = 0;
        let filtered_bps =
            carla_blueprint_library_filter(blueprints, any_vehicle_id.as_ptr(), &mut count);
        if !filtered_bps.is_null() && count > 0 {
            unsafe {
                vehicle_bp = *filtered_bps;
            }
        }
    }

    if vehicle_bp.is_null() {
        return ptr::null_mut();
    }

    // Get spawn points
    let map = carla_world_get_map(world);
    let spawn_points = carla_map_get_spawn_points(map);
    if spawn_points.is_null() || carla_transform_list_size(spawn_points) == 0 {
        carla_map_free(map);
        return ptr::null_mut();
    }

    let spawn_point = carla_transform_list_get(spawn_points, 0);

    // Spawn vehicle
    let spawn_result = carla_world_spawn_actor(world, vehicle_bp, &spawn_point, ptr::null_mut());

    carla_transform_list_free(spawn_points);
    carla_map_free(map);

    if spawn_result.error != carla_error_t_CARLA_ERROR_NONE || spawn_result.actor.is_null() {
        return ptr::null_mut();
    }

    carla_actor_as_vehicle(spawn_result.actor)
}

unsafe fn demonstrate_door_controls(vehicle: *mut carla_vehicle_t) {
    println!("🚪 Door Control Demonstration");
    println!("============================");

    // Define all doors
    let doors = [
        (
            "Front Left (Driver)",
            carla_vehicle_door_t_CARLA_VEHICLE_DOOR_FL,
        ),
        (
            "Front Right (Passenger)",
            carla_vehicle_door_t_CARLA_VEHICLE_DOOR_FR,
        ),
        ("Rear Left", carla_vehicle_door_t_CARLA_VEHICLE_DOOR_RL),
        ("Rear Right", carla_vehicle_door_t_CARLA_VEHICLE_DOOR_RR),
        ("Hood", carla_vehicle_door_t_CARLA_VEHICLE_DOOR_HOOD),
        ("Trunk", carla_vehicle_door_t_CARLA_VEHICLE_DOOR_TRUNK),
    ];

    // 1. Open doors one by one
    println!("\n1. Opening doors individually:");
    for (door_name, door_id) in doors.iter() {
        println!("   Opening {}...", door_name);
        let result = carla_vehicle_open_door(vehicle, *door_id);
        if result == carla_error_t_CARLA_ERROR_NONE {
            println!("     ✅ {} opened successfully", door_name);
        } else {
            println!("     ❌ Failed to open {} (error: {})", door_name, result);
        }
        thread::sleep(Duration::from_millis(500));
    }

    thread::sleep(Duration::from_secs(2));

    // 2. Close doors one by one
    println!("\n2. Closing doors individually:");
    for (door_name, door_id) in doors.iter().rev() {
        println!("   Closing {}...", door_name);
        let result = carla_vehicle_close_door(vehicle, *door_id);
        if result == carla_error_t_CARLA_ERROR_NONE {
            println!("     ✅ {} closed successfully", door_name);
        } else {
            println!("     ❌ Failed to close {} (error: {})", door_name, result);
        }
        thread::sleep(Duration::from_millis(500));
    }

    // 3. Open all doors at once
    println!("\n3. Opening all doors simultaneously:");
    let result = carla_vehicle_open_door(vehicle, carla_vehicle_door_t_CARLA_VEHICLE_DOOR_ALL);
    if result == carla_error_t_CARLA_ERROR_NONE {
        println!("     ✅ All doors opened successfully");
    } else {
        println!("     ❌ Failed to open all doors (error: {})", result);
    }

    thread::sleep(Duration::from_secs(1));

    // 4. Close all doors at once
    println!("\n4. Closing all doors simultaneously:");
    let result = carla_vehicle_close_door(vehicle, carla_vehicle_door_t_CARLA_VEHICLE_DOOR_ALL);
    if result == carla_error_t_CARLA_ERROR_NONE {
        println!("     ✅ All doors closed successfully");
    } else {
        println!("     ❌ Failed to close all doors (error: {})", result);
    }
}

unsafe fn demonstrate_lighting_scenarios(vehicle: *mut carla_vehicle_t) {
    println!("\n💡 Lighting Scenarios Demonstration");
    println!("==================================");

    // Define lighting scenarios
    let scenarios = [
        (
            "Daytime Running",
            carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_POSITION,
        ),
        (
            "Night Driving",
            carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_POSITION
                | carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_LOW_BEAM,
        ),
        (
            "High Beam",
            carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_POSITION
                | carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_HIGH_BEAM,
        ),
        (
            "Left Turn Signal",
            carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_POSITION
                | carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_LOW_BEAM
                | carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_LEFT_BLINKER,
        ),
        (
            "Right Turn Signal",
            carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_POSITION
                | carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_LOW_BEAM
                | carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_RIGHT_BLINKER,
        ),
        (
            "Emergency Flashers",
            carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_POSITION
                | carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_LEFT_BLINKER
                | carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_RIGHT_BLINKER,
        ),
        (
            "Braking",
            carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_POSITION
                | carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_LOW_BEAM
                | carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_BRAKE,
        ),
        (
            "Reverse",
            carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_POSITION
                | carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_REVERSE,
        ),
        (
            "Emergency Vehicle (Sirens)",
            carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_POSITION
                | carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_SPECIAL1  // Sirens
                | carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_SPECIAL2,
        ),
        (
            "Fog Lights",
            carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_POSITION
                | carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_LOW_BEAM
                | carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_FOG,
        ),
    ];

    for (scenario_name, light_state) in scenarios.iter() {
        println!("\n   Scenario: {}", scenario_name);
        println!("     Light state: 0x{:X}", light_state);

        let result = carla_vehicle_set_light_state(vehicle, *light_state);
        if result == carla_error_t_CARLA_ERROR_NONE {
            println!("     ✅ Lights set successfully");
        } else {
            println!("     ❌ Failed to set lights (error: {})", result);
        }

        thread::sleep(Duration::from_secs(1));
    }

    // Turn off all lights
    println!("\n   Turning off all lights...");
    let result = carla_vehicle_set_light_state(
        vehicle,
        carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_NONE,
    );
    if result == carla_error_t_CARLA_ERROR_NONE {
        println!("     ✅ All lights turned off");
    } else {
        println!("     ❌ Failed to turn off lights (error: {})", result);
    }
}

unsafe fn demonstrate_realistic_scenarios(vehicle: *mut carla_vehicle_t) {
    println!("\n🎬 Realistic Scenario Demonstrations");
    println!("===================================");

    // Scenario 1: Driver getting in
    println!("\n1. Driver Getting In:");
    println!("   Opening driver door...");
    carla_vehicle_open_door(vehicle, carla_vehicle_door_t_CARLA_VEHICLE_DOOR_FL);
    println!("   Turning on interior lights...");
    carla_vehicle_set_light_state(
        vehicle,
        carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_INTERIOR,
    );
    thread::sleep(Duration::from_secs(2));

    println!("   Closing driver door...");
    carla_vehicle_close_door(vehicle, carla_vehicle_door_t_CARLA_VEHICLE_DOOR_FL);
    println!("   Starting engine (position lights on)...");
    carla_vehicle_set_light_state(
        vehicle,
        carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_POSITION,
    );
    thread::sleep(Duration::from_secs(1));

    // Scenario 2: Loading groceries
    println!("\n2. Loading Groceries:");
    println!("   Opening trunk...");
    carla_vehicle_open_door(vehicle, carla_vehicle_door_t_CARLA_VEHICLE_DOOR_TRUNK);
    thread::sleep(Duration::from_secs(3));
    println!("   Closing trunk...");
    carla_vehicle_close_door(vehicle, carla_vehicle_door_t_CARLA_VEHICLE_DOOR_TRUNK);

    // Scenario 3: Night driving departure
    println!("\n3. Night Driving Departure:");
    println!("   Turning on headlights...");
    let night_lights = carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_POSITION
        | carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_LOW_BEAM;
    carla_vehicle_set_light_state(vehicle, night_lights);
    thread::sleep(Duration::from_secs(1));

    // Scenario 4: Emergency situation
    println!("\n4. Emergency Situation:");
    println!("   Activating emergency flashers...");
    let emergency_lights = carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_POSITION
        | carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_LEFT_BLINKER
        | carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_RIGHT_BLINKER;
    carla_vehicle_set_light_state(vehicle, emergency_lights);

    println!("   Opening hood for inspection...");
    carla_vehicle_open_door(vehicle, carla_vehicle_door_t_CARLA_VEHICLE_DOOR_HOOD);
    thread::sleep(Duration::from_secs(3));

    println!("   Closing hood...");
    carla_vehicle_close_door(vehicle, carla_vehicle_door_t_CARLA_VEHICLE_DOOR_HOOD);

    println!("   Turning off emergency flashers...");
    carla_vehicle_set_light_state(
        vehicle,
        carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_POSITION,
    );

    // Scenario 5: Valet parking
    println!("\n5. Valet Parking:");
    println!("   Turning on interior lights...");
    carla_vehicle_set_light_state(
        vehicle,
        carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_POSITION
            | carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_INTERIOR,
    );

    println!("   Opening all doors for valet...");
    carla_vehicle_open_door(vehicle, carla_vehicle_door_t_CARLA_VEHICLE_DOOR_ALL);
    thread::sleep(Duration::from_secs(2));

    println!("   Closing all doors...");
    carla_vehicle_close_door(vehicle, carla_vehicle_door_t_CARLA_VEHICLE_DOOR_ALL);

    println!("   Turning off all lights...");
    carla_vehicle_set_light_state(
        vehicle,
        carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_NONE,
    );

    println!("\n✨ All realistic scenarios completed!");
}
