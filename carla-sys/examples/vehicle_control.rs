//! Example demonstrating vehicle control functionality in carla-sys2
//!
//! This example shows how to use the new vehicle control bindings including:
//! - Basic vehicle control (throttle, brake, steering)
//! - Autopilot control  
//! - Vehicle lighting
//! - Door control (CARLA 0.10.0 feature)
//! - Wheel control

use carla_sys::*;

fn main() {
    println!("CARLA Vehicle Control Example");
    println!("=============================");

    // Note: This example demonstrates the API without connecting to an actual CARLA server
    // In a real application, you would need a running CARLA server

    demonstrate_vehicle_control_structures();
    demonstrate_vehicle_lighting();
    demonstrate_vehicle_doors();
    demonstrate_ackermann_control();
}

fn demonstrate_vehicle_control_structures() {
    println!("\n1. Vehicle Control Structures");
    println!("-----------------------------");

    // Basic vehicle control
    let mut control = carla_vehicle_control_t {
        throttle: 0.8,            // 80% throttle
        steer: -0.3,              // 30% left steering
        brake: 0.0,               // No braking
        hand_brake: false,        // Hand brake off
        reverse: false,           // Forward gear
        manual_gear_shift: false, // Automatic transmission
        gear: 1,                  // First gear
    };

    println!("Basic Vehicle Control:");
    println!("  Throttle: {:.1}%", control.throttle * 100.0);
    println!("  Steering: {:.1} (negative = left)", control.steer);
    println!("  Brake: {:.1}%", control.brake * 100.0);
    println!("  Gear: {}", control.gear);

    // Simulate braking
    control.throttle = 0.0;
    control.brake = 0.5;
    println!("\nAfter applying brakes:");
    println!("  Throttle: {:.1}%", control.throttle * 100.0);
    println!("  Brake: {:.1}%", control.brake * 100.0);
}

fn demonstrate_vehicle_lighting() {
    println!("\n2. Vehicle Lighting System");
    println!("--------------------------");

    // Individual lights
    let position_lights = carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_POSITION;
    let low_beam = carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_LOW_BEAM;
    let high_beam = carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_HIGH_BEAM;
    let brake_lights = carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_BRAKE;
    let left_blinker = carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_LEFT_BLINKER;
    let right_blinker = carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_RIGHT_BLINKER;

    println!("Individual Light States:");
    println!("  Position lights: 0x{:X}", position_lights);
    println!("  Low beam: 0x{:X}", low_beam);
    println!("  High beam: 0x{:X}", high_beam);
    println!("  Brake lights: 0x{:X}", brake_lights);

    // Combine multiple lights (bitwise OR)
    let headlights = position_lights | low_beam;
    let emergency_lights = left_blinker | right_blinker;
    let night_driving = position_lights | low_beam | brake_lights;

    println!("\nCombined Light Configurations:");
    println!("  Headlights (position + low beam): 0x{:X}", headlights);
    println!("  Emergency flashers: 0x{:X}", emergency_lights);
    println!("  Night driving setup: 0x{:X}", night_driving);

    // Special lights (sirens, etc.)
    let special1 = carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_SPECIAL1; // Sirens
    let special2 = carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_SPECIAL2;
    println!("  Special lights (sirens): 0x{:X}", special1);
    println!("  Special lights 2: 0x{:X}", special2);
}

fn demonstrate_vehicle_doors() {
    println!("\n3. Vehicle Door Control (CARLA 0.10.0)");
    println!("--------------------------------------");

    // Door enumeration
    let doors = [
        ("Front Left", carla_vehicle_door_t_CARLA_VEHICLE_DOOR_FL),
        ("Front Right", carla_vehicle_door_t_CARLA_VEHICLE_DOOR_FR),
        ("Rear Left", carla_vehicle_door_t_CARLA_VEHICLE_DOOR_RL),
        ("Rear Right", carla_vehicle_door_t_CARLA_VEHICLE_DOOR_RR),
        ("Hood", carla_vehicle_door_t_CARLA_VEHICLE_DOOR_HOOD),
        ("Trunk", carla_vehicle_door_t_CARLA_VEHICLE_DOOR_TRUNK),
        ("All Doors", carla_vehicle_door_t_CARLA_VEHICLE_DOOR_ALL),
    ];

    println!("Available Door Controls:");
    for (name, door_id) in doors.iter() {
        println!("  {}: {}", name, door_id);
    }

    println!("\nDoor operations would use:");
    println!("  carla_vehicle_open_door(vehicle, CARLA_VEHICLE_DOOR_FL)");
    println!("  carla_vehicle_close_door(vehicle, CARLA_VEHICLE_DOOR_TRUNK)");
}

fn demonstrate_ackermann_control() {
    println!("\n4. Ackermann Vehicle Control");
    println!("----------------------------");

    // Ackermann control for precise steering
    let ackermann_control = carla_vehicle_ackermann_control_t {
        steer: 15.0,       // 15 degrees steering angle
        steer_speed: 45.0, // 45 degrees/second steering speed
        speed: 10.0,       // 10 m/s target speed
        acceleration: 1.5, // 1.5 m/s² acceleration
        jerk: 0.8,         // 0.8 m/s³ jerk limit
    };

    println!("Ackermann Control Parameters:");
    println!("  Steering angle: {:.1}°", ackermann_control.steer);
    println!("  Steering speed: {:.1}°/s", ackermann_control.steer_speed);
    println!("  Target speed: {:.1} m/s", ackermann_control.speed);
    println!("  Acceleration: {:.1} m/s²", ackermann_control.acceleration);
    println!("  Jerk limit: {:.1} m/s³", ackermann_control.jerk);

    // Ackermann controller settings (PID parameters)
    let controller_settings = carla_ackermann_controller_settings_t {
        speed_kp: 0.8,  // Speed proportional gain
        speed_ki: 0.15, // Speed integral gain
        speed_kd: 0.05, // Speed derivative gain
        accel_kp: 0.6,  // Acceleration proportional gain
        accel_ki: 0.1,  // Acceleration integral gain
        accel_kd: 0.03, // Acceleration derivative gain
    };

    println!("\nAckermann Controller PID Settings:");
    println!(
        "  Speed PID: P={:.2}, I={:.2}, D={:.2}",
        controller_settings.speed_kp, controller_settings.speed_ki, controller_settings.speed_kd
    );
    println!(
        "  Accel PID: P={:.2}, I={:.2}, D={:.2}",
        controller_settings.accel_kp, controller_settings.accel_ki, controller_settings.accel_kd
    );
}

#[allow(dead_code)]
fn demonstrate_api_usage_patterns() {
    println!("\n5. API Usage Examples");
    println!("---------------------");

    println!("Example vehicle control workflow:");
    println!("```rust");
    println!("unsafe {{");
    println!("    // Get vehicle from actor");
    println!("    let vehicle = carla_actor_as_vehicle(actor);");
    println!("    if vehicle.is_null() {{");
    println!("        return; // Not a vehicle");
    println!("    }}");
    println!("");
    println!("    // Create control command");
    println!("    let control = carla_vehicle_control_t {{");
    println!("        throttle: 0.7,");
    println!("        steer: 0.2,");
    println!("        brake: 0.0,");
    println!("        hand_brake: false,");
    println!("        reverse: false,");
    println!("        manual_gear_shift: false,");
    println!("        gear: 0,");
    println!("    }};");
    println!("");
    println!("    // Apply vehicle control");
    println!("    let result = carla_vehicle_apply_control(vehicle, &control);");
    println!("    if result != carla_error_t_CARLA_ERROR_NONE {{");
    println!("        eprintln!(\"Failed to apply vehicle control\");");
    println!("    }}");
    println!("");
    println!("    // Set autopilot");
    println!("    carla_vehicle_set_autopilot_default_port(vehicle, true);");
    println!("");
    println!("    // Control lighting");
    println!("    let lights = CARLA_VEHICLE_LIGHT_POSITION | CARLA_VEHICLE_LIGHT_LOW_BEAM;");
    println!("    carla_vehicle_set_light_state(vehicle, lights);");
    println!("");
    println!("    // Open driver door (CARLA 0.10.0)");
    println!("    carla_vehicle_open_door(vehicle, CARLA_VEHICLE_DOOR_FL);");
    println!("}}");
    println!("```");
}
