//! Comprehensive demonstration of the carla-cxx Actor System
//!
//! This example shows how to:
//! - Connect to CARLA server
//! - Spawn different types of actors (vehicles, walkers, traffic lights)
//! - Cast actors to specific types
//! - Control vehicle movement and lights
//! - Control walker movement
//! - Manage traffic light states
//! - Use sensor functionality

use anyhow::Result;
use carla_cxx::{
    ActorBlueprintExt, ClientWrapper, SensorWrapper, SimpleLocation, SimpleRotation,
    SimpleTransform, TrafficLightState, TrafficLightTiming, TrafficLightWrapper, VehicleControl,
    VehicleLightState, VehicleWrapper, WalkerControl, WalkerWrapper,
};
use std::time::Duration;

fn main() -> Result<()> {
    println!("=== CARLA-CXX Actor System Demo ===\n");

    // Note: This example demonstrates the API structure, but won't actually run
    // without a CARLA server running on localhost:2000

    // Connect to CARLA server
    println!("🔌 Connecting to CARLA server...");
    match ClientWrapper::new("localhost", 2000) {
        Ok(mut client) => {
            println!("✅ Connected to CARLA server");
            println!("📊 Server version: {}", client.get_server_version());

            // Set a timeout for operations
            client.set_timeout(Duration::from_secs(10));
            println!("⏱️  Client timeout: {:?}", client.get_timeout());

            // Get the simulation world
            let world = client.get_world();
            println!("🌍 World ID: {}", world.get_id());

            // Get blueprint library
            let blueprint_library = world.get_blueprint_library();
            println!("📚 Blueprint library size: {}", blueprint_library.size());

            // Demonstrate vehicle spawning and control
            demonstrate_vehicle_system(&world, &blueprint_library)?;

            // Demonstrate walker spawning and control
            demonstrate_walker_system(&world, &blueprint_library)?;

            // Demonstrate traffic light control
            demonstrate_traffic_light_system(&world)?;

            // Demonstrate sensor functionality
            demonstrate_sensor_system(&world, &blueprint_library)?;
        }
        Err(e) => {
            println!("❌ Failed to connect to CARLA server: {}", e);
            println!("💡 Make sure CARLA server is running on localhost:2000");
            println!("   This example demonstrates the API structure even without a server.");
        }
    }

    println!("\n✅ Actor system demo completed!");
    Ok(())
}

fn demonstrate_vehicle_system(
    world: &carla_cxx::WorldWrapper,
    blueprint_library: &carla_cxx::BlueprintLibraryWrapper,
) -> Result<()> {
    println!("\n🚗 === Vehicle System Demo ===");

    // Find a vehicle blueprint
    if let Some(vehicle_bp) = blueprint_library.find("vehicle.tesla.model3") {
        println!("🔍 Found vehicle blueprint: {}", vehicle_bp.get_id());
        println!("🏷️  Tags: {:?}", vehicle_bp.get_tags());

        // Set some vehicle attributes
        if vehicle_bp.contains_attribute("color") {
            println!("🎨 Setting vehicle color...");
            // Note: This would modify the blueprint
        }

        // Define spawn location
        let spawn_transform = SimpleTransform::new(
            SimpleLocation::new(0.0, 0.0, 0.5), // Slightly above ground
            SimpleRotation::new(0.0, 0.0, 0.0), // No rotation
        );

        println!("📍 Spawn location: {:?}", spawn_transform.location);

        // Try to spawn the vehicle
        match world.try_spawn_actor(&vehicle_bp, &spawn_transform, None) {
            Some(actor) => {
                println!("✅ Vehicle spawned with ID: {}", actor.get_id());

                // Cast to Vehicle for specific vehicle operations
                if let Some(vehicle) = VehicleWrapper::from_actor(actor.get_actor()) {
                    println!("🎯 Successfully cast to Vehicle");

                    // Demonstrate vehicle control
                    let control = VehicleControl::new()
                        .throttle(0.5) // 50% throttle
                        .steer(0.1) // Slight right turn
                        .brake(0.0); // No braking

                    println!("🚙 Applying vehicle control: throttle=0.5, steer=0.1");
                    vehicle.apply_control(&control)?;

                    // Get current control state
                    let current_control = vehicle.get_control();
                    println!("📊 Current control state: {:?}", current_control);

                    // Set autopilot
                    println!("🤖 Enabling autopilot...");
                    vehicle.set_autopilot(true);

                    // Get speed information
                    println!("📈 Current speed: {:.2} m/s", vehicle.get_speed());
                    println!("🚦 Speed limit: {:.2} m/s", vehicle.get_speed_limit());

                    // Demonstrate light controls
                    let light_state = VehicleLightState::POSITION | VehicleLightState::LOW_BEAM;
                    vehicle.set_light_state(light_state);
                    println!("💡 Set vehicle lights: {:?}", light_state);

                    let current_lights = vehicle.get_light_state();
                    println!("💡 Current light state: {:?}", current_lights);

                    // Clean up - destroy the vehicle
                    println!("🧹 Destroying vehicle...");
                    if actor.destroy() {
                        println!("✅ Vehicle destroyed successfully");
                    }
                } else {
                    println!("❌ Failed to cast actor to Vehicle");
                }
            }
            None => {
                println!("❌ Failed to spawn vehicle (location might be occupied)");
            }
        }
    } else {
        println!("❌ Vehicle blueprint 'vehicle.tesla.model3' not found");
    }

    Ok(())
}

fn demonstrate_walker_system(
    world: &carla_cxx::WorldWrapper,
    blueprint_library: &carla_cxx::BlueprintLibraryWrapper,
) -> Result<()> {
    println!("\n🚶 === Walker System Demo ===");

    // Find a walker blueprint
    if let Some(walker_bp) = blueprint_library.find("walker.pedestrian.0001") {
        println!("🔍 Found walker blueprint: {}", walker_bp.get_id());

        // Define spawn location
        let spawn_transform =
            SimpleTransform::new(SimpleLocation::new(10.0, 0.0, 0.5), SimpleRotation::ZERO);

        // Try to spawn the walker
        match world.try_spawn_actor(&walker_bp, &spawn_transform, None) {
            Some(actor) => {
                println!("✅ Walker spawned with ID: {}", actor.get_id());

                // Cast to Walker for specific walker operations
                if let Some(walker) = WalkerWrapper::from_actor(actor.get_actor()) {
                    println!("🎯 Successfully cast to Walker");

                    // Demonstrate walker control
                    let control = WalkerControl::new()
                        .direction(1.0, 0.0, 0.0) // Walk forward (X direction)
                        .speed(2.0) // 2 m/s walking speed
                        .jump(false); // Not jumping

                    println!("🚶 Applying walker control: forward at 2 m/s");
                    walker.apply_control(&control)?;

                    // Get current control state
                    let current_control = walker.get_control();
                    println!("📊 Current walker control: {:?}", current_control);

                    // Get speed information
                    println!("📈 Current walker speed: {:.2} m/s", walker.get_speed());

                    // Demonstrate direction change
                    let turn_control = WalkerControl::new()
                        .direction(0.0, 1.0, 0.0) // Turn to walk sideways
                        .speed(1.5) // Slower speed
                        .jump(false);

                    println!("🔄 Changing walker direction to sideways");
                    walker.apply_control(&turn_control)?;

                    // Clean up - destroy the walker
                    println!("🧹 Destroying walker...");
                    if actor.destroy() {
                        println!("✅ Walker destroyed successfully");
                    }
                } else {
                    println!("❌ Failed to cast actor to Walker");
                }
            }
            None => {
                println!("❌ Failed to spawn walker");
            }
        }
    } else {
        println!("❌ Walker blueprint 'walker.pedestrian.0001' not found");
        println!("💡 Available walker blueprints might have different names");
    }

    Ok(())
}

fn demonstrate_traffic_light_system(world: &carla_cxx::WorldWrapper) -> Result<()> {
    println!("\n🚦 === Traffic Light System Demo ===");

    // In a real scenario, you would get traffic lights from the world
    // For this demo, we'll show how the API would work
    println!("🔍 Looking for traffic lights in the world...");

    // This is how you would use traffic lights if you had them:
    println!("📚 Traffic Light API demonstration:");

    // Show traffic light state management
    println!("\n🚦 Traffic Light States:");
    let states = [
        TrafficLightState::Red,
        TrafficLightState::Yellow,
        TrafficLightState::Green,
        TrafficLightState::Off,
    ];

    for state in &states {
        println!(
            "  {}: go={}, stop={}, caution={}",
            state,
            state.is_go(),
            state.is_stop(),
            state.is_caution()
        );
    }

    // Show traffic light timing
    println!("\n⏱️ Traffic Light Timing:");
    let timing = TrafficLightTiming::default();
    println!(
        "  Default timing: red={}s, yellow={}s, green={}s",
        timing.red_time, timing.yellow_time, timing.green_time
    );
    println!("  Total cycle time: {}s", timing.total_cycle_time());

    let quick_timing = TrafficLightTiming::quick();
    println!(
        "  Quick timing: red={}s, yellow={}s, green={}s",
        quick_timing.red_time, quick_timing.yellow_time, quick_timing.green_time
    );

    println!(
        "💡 To use traffic lights, find them in the world's actor list and cast to TrafficLight"
    );

    Ok(())
}

fn demonstrate_sensor_system(
    world: &carla_cxx::WorldWrapper,
    blueprint_library: &carla_cxx::BlueprintLibraryWrapper,
) -> Result<()> {
    println!("\n📷 === Sensor System Demo ===");

    // Find a camera sensor blueprint
    if let Some(camera_bp) = blueprint_library.find("sensor.camera.rgb") {
        println!("🔍 Found camera sensor blueprint: {}", camera_bp.get_id());

        // Set camera attributes
        if camera_bp.contains_attribute("image_size_x") {
            println!("🖼️  Camera supports image_size_x attribute");
        }
        if camera_bp.contains_attribute("fov") {
            println!("👁️  Camera supports field of view attribute");
        }

        // Define spawn location (camera position)
        let spawn_transform = SimpleTransform::new(
            SimpleLocation::new(0.0, 0.0, 2.0), // 2 meters high
            SimpleRotation::new(0.0, 0.0, 0.0), // Looking forward
        );

        // Try to spawn the sensor
        match world.try_spawn_actor(&camera_bp, &spawn_transform, None) {
            Some(actor) => {
                println!("✅ Camera sensor spawned with ID: {}", actor.get_id());

                // Cast to Sensor for specific sensor operations
                if let Some(sensor) = SensorWrapper::from_actor(actor.get_actor()) {
                    println!("🎯 Successfully cast to Sensor");

                    // Check sensor status
                    println!("🔍 Sensor listening status: {}", sensor.is_listening());

                    // Note: Sensor callbacks are complex in CXX and not fully implemented
                    println!("💡 Sensor callback system is TODO - requires complex CXX design");

                    // Stop the sensor (if it was listening)
                    sensor.stop();
                    println!("⏹️  Sensor stopped");

                    println!(
                        "🔍 Sensor listening status after stop: {}",
                        sensor.is_listening()
                    );

                    // Clean up - destroy the sensor
                    println!("🧹 Destroying sensor...");
                    if actor.destroy() {
                        println!("✅ Sensor destroyed successfully");
                    }
                } else {
                    println!("❌ Failed to cast actor to Sensor");
                }
            }
            None => {
                println!("❌ Failed to spawn camera sensor");
            }
        }
    } else {
        println!("❌ Camera sensor blueprint 'sensor.camera.rgb' not found");
    }

    Ok(())
}
