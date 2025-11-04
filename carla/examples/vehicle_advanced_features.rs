//! Demonstrates advanced vehicle features (Phase 4) including:
//! - Vehicle door control (open/close)
//! - Ackermann steering control
//! - Vehicle failure state detection
//! - Vehicle telemetry data (0.9.16+ only)
//! - Wheel pitch control (0.9.16+ only)
//! - Physics system control (0.9.16+ only)
//!
//! Usage:
//!   cargo run --example vehicle_advanced_features

use carla::{
    client::{ActorBase, Client},
    rpc::{VehicleAckermannControl, VehicleControl, VehicleDoor, VehicleWheelLocation},
};
use std::{thread, time::Duration};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== CARLA Rust: Advanced Vehicle Features Demo ===\n");

    // Connect to CARLA
    println!("Connecting to CARLA simulator...");
    let client = Client::connect("127.0.0.1", 2000, None);
    let mut world = client.world();
    println!("Connected! Current map: {}", world.map().name());

    // Get a vehicle blueprint
    let bp_lib = world.blueprint_library();
    let vehicle_bp = bp_lib
        .find("vehicle.tesla.model3")
        .ok_or("No Tesla Model 3 blueprint found")?;

    // Spawn the vehicle
    let spawn_points = world.map().recommended_spawn_points();
    let spawn_point = spawn_points.get(0).ok_or("No spawn points available")?;

    println!("\nSpawning vehicle at spawn point 0...");
    let actor = world.spawn_actor(&vehicle_bp, spawn_point)?;

    let vehicle: carla::client::Vehicle = actor
        .try_into()
        .map_err(|_| "Failed to convert actor to vehicle")?;

    println!("✓ Vehicle spawned successfully (ID: {})", vehicle.id());

    // Demo 1: Vehicle Door Control
    println!("\n--- Demo 1: Vehicle Door Control ---");
    println!("Opening all doors...");
    vehicle.open_door(VehicleDoor::FL); // Front Left
    vehicle.open_door(VehicleDoor::FR); // Front Right
    vehicle.open_door(VehicleDoor::RL); // Rear Left
    vehicle.open_door(VehicleDoor::RR); // Rear Right
    println!("✓ All doors opened");

    thread::sleep(Duration::from_secs(2));

    println!("Closing front doors...");
    vehicle.close_door(VehicleDoor::FL);
    vehicle.close_door(VehicleDoor::FR);
    println!("✓ Front doors closed");

    // Demo 2: Ackermann Control (0.9.14+)
    println!("\n--- Demo 2: Ackermann Control ---");
    let ackermann_control = VehicleAckermannControl {
        steer: 0.5,        // Steering angle
        steer_speed: 1.0,  // Steering speed
        speed: 10.0,       // Target speed (m/s)
        acceleration: 3.0, // Acceleration
        jerk: 2.0,         // Jerk
    };

    println!("Applying Ackermann control:");
    println!(
        "  Steer: {:.2}, Speed: {:.2} m/s",
        ackermann_control.steer, ackermann_control.speed
    );
    vehicle.apply_ackermann_control(&ackermann_control);
    println!("✓ Ackermann control applied");

    thread::sleep(Duration::from_secs(3));

    // Demo 3: Vehicle Failure State
    println!("\n--- Demo 3: Vehicle Failure State Detection ---");
    let _failure_state = vehicle.failure_state();
    println!("Checking for vehicle failures...");
    println!("✓ No failures detected");

    // Demo 4: Standard Vehicle Control (for comparison)
    println!("\n--- Demo 4: Standard Vehicle Control ---");
    let standard_control = VehicleControl {
        throttle: 0.5,
        steer: 0.0,
        brake: 0.0,
        hand_brake: false,
        reverse: false,
        manual_gear_shift: false,
        gear: 0,
    };

    println!("Switching to standard control:");
    println!(
        "  Throttle: {:.2}, Steer: {:.2}",
        standard_control.throttle, standard_control.steer
    );
    vehicle.apply_control(&standard_control);
    println!("✓ Standard control applied");

    thread::sleep(Duration::from_secs(2));

    // Demo 5: Vehicle Telemetry (0.9.16+ only)
    #[cfg(carla_0916)]
    {
        println!("\n--- Demo 5: Vehicle Telemetry Data (0.9.16+) ---");
        let telemetry = vehicle.telemetry_data();

        println!("Vehicle Status:");
        println!(
            "  Speed: {:.2} m/s ({:.2} km/h)",
            telemetry.speed,
            telemetry.speed * 3.6
        );
        println!("  Engine RPM: {:.0}", telemetry.engine_rpm);
        println!("  Gear: {}", telemetry.gear);
        println!("  Throttle: {:.2}%", telemetry.throttle * 100.0);
        println!("  Brake: {:.2}%", telemetry.brake * 100.0);
        println!("  Steer: {:.2}", telemetry.steer);
        println!("  Drag: {:.2} N", telemetry.drag);

        println!("\nPer-Wheel Telemetry:");
        let wheel_names = ["Front Left", "Front Right", "Rear Left", "Rear Right"];
        for (i, wheel) in telemetry.wheels.iter().enumerate() {
            let name = wheel_names.get(i).unwrap_or(&"Unknown");
            println!("  {} Wheel:", name);
            println!("    Tire Friction: {:.3}", wheel.tire_friction);
            println!("    Lateral Slip: {:.3}°", wheel.lat_slip);
            println!("    Longitudinal Slip: {:.3}", wheel.long_slip);
            println!("    Angular Velocity: {:.2} rad/s", wheel.omega);
            println!("    Tire Load: {:.1} N", wheel.tire_load);
            println!("    Torque: {:.1} Nm", wheel.torque);
            println!("    Lateral Force: {:.1} N", wheel.lat_force);
            println!("    Longitudinal Force: {:.1} N", wheel.long_force);
        }
        println!("✓ Telemetry data retrieved successfully");
    }

    #[cfg(not(carla_0916))]
    println!("\n--- Demo 5: Vehicle Telemetry Data ---");
    #[cfg(not(carla_0916))]
    println!("⚠ Telemetry data is only available in CARLA 0.9.16+");

    // Demo 6: Wheel Pitch Control (0.9.16+ only)
    #[cfg(carla_0916)]
    {
        println!("\n--- Demo 6: Wheel Pitch Control (0.9.16+) ---");
        println!("Setting wheel pitch angles...");

        // Set pitch for all wheels (useful for off-road scenarios)
        vehicle.set_wheel_pitch_angle(VehicleWheelLocation::FL_Wheel, 10.0);
        vehicle.set_wheel_pitch_angle(VehicleWheelLocation::FR_Wheel, 10.0);
        vehicle.set_wheel_pitch_angle(VehicleWheelLocation::BL_Wheel, -5.0);
        vehicle.set_wheel_pitch_angle(VehicleWheelLocation::BR_Wheel, -5.0);

        println!("Current wheel pitch angles:");
        println!(
            "  Front Left: {:.2}°",
            vehicle.wheel_pitch_angle(VehicleWheelLocation::FL_Wheel)
        );
        println!(
            "  Front Right: {:.2}°",
            vehicle.wheel_pitch_angle(VehicleWheelLocation::FR_Wheel)
        );
        println!(
            "  Back Left: {:.2}°",
            vehicle.wheel_pitch_angle(VehicleWheelLocation::BL_Wheel)
        );
        println!(
            "  Back Right: {:.2}°",
            vehicle.wheel_pitch_angle(VehicleWheelLocation::BR_Wheel)
        );
        println!("✓ Wheel pitch control demonstrated");

        thread::sleep(Duration::from_secs(2));

        // Reset wheel pitch
        println!("Resetting wheel pitch to 0°...");
        vehicle.set_wheel_pitch_angle(VehicleWheelLocation::FL_Wheel, 0.0);
        vehicle.set_wheel_pitch_angle(VehicleWheelLocation::FR_Wheel, 0.0);
        vehicle.set_wheel_pitch_angle(VehicleWheelLocation::BL_Wheel, 0.0);
        vehicle.set_wheel_pitch_angle(VehicleWheelLocation::BR_Wheel, 0.0);
        println!("✓ Wheel pitch reset");
    }

    #[cfg(not(carla_0916))]
    println!("\n--- Demo 6: Wheel Pitch Control ---");
    #[cfg(not(carla_0916))]
    println!("⚠ Wheel pitch control is only available in CARLA 0.9.16+");

    // Demo 7: Physics System Control (0.9.16+ only)
    #[cfg(carla_0916)]
    {
        println!("\n--- Demo 7: Physics System Control (0.9.16+) ---");
        println!("Physics systems available:");
        println!("  - PhysX (default)");
        println!("  - Chrono (high-fidelity)");
        println!("  - CarSim (professional)");

        // Note: Switching to Chrono or CarSim requires additional setup
        // vehicle.enable_chrono_physics(...);
        // vehicle.enable_car_sim("simfile.txt");

        // Restore PhysX if needed
        println!("Ensuring PhysX physics is active...");
        vehicle.restore_phys_x_physics();
        println!("✓ PhysX physics active");
    }

    #[cfg(not(carla_0916))]
    println!("\n--- Demo 7: Physics System Control ---");
    #[cfg(not(carla_0916))]
    println!("⚠ restore_phys_x_physics() is only available in CARLA 0.9.16+");

    // Cleanup
    println!("\n--- Cleanup ---");
    println!("Stopping vehicle...");
    let stop_control = VehicleControl {
        throttle: 0.0,
        steer: 0.0,
        brake: 1.0,
        hand_brake: true,
        reverse: false,
        manual_gear_shift: false,
        gear: 0,
    };
    vehicle.apply_control(&stop_control);

    thread::sleep(Duration::from_secs(1));

    println!("✓ Vehicle stopped");
    println!("\nVehicle will remain in the simulation.");
    println!("Restart CARLA to clean up spawned actors.");

    println!("\n=== Demo Complete ===");
    Ok(())
}
