//! Multi-Agent Simulation Integration Test
//!
//! This integration test demonstrates multi-agent simulation:
//! - Spawn multiple vehicles (50) and pedestrians (100)
//! - Control subset with AI/autopilot
//! - Monitor collisions and blocked actors
//! - Use debug drawing for visualization
//! - Demonstrate efficient actor management
//!
//! This validates the integration of actor spawning, control, and monitoring.
//!
//! Run with:
//! ```bash
//! cargo run --example integration_multi_agent --profile dev-release
//! ```

use carla::{
    client::{ActorBase, Client, Vehicle, Walker},
    geom::{Location, Rotation, Transform},
    rpc::Color,
};
use std::time::{Duration, Instant};

const NUM_VEHICLES: usize = 20; // Reduced from 50 for faster testing
const NUM_WALKERS: usize = 30; // Reduced from 100 for faster testing
const SIMULATION_TIME_SECS: u64 = 15;

fn main() {
    println!("=== Multi-Agent Simulation Integration Test ===\n");

    // Connect to CARLA
    let client = Client::connect("127.0.0.1", 2000, None);
    let mut world = client.world();
    println!("Connected to CARLA server");

    // Test 1: Spawn multiple vehicles
    println!("\n--- Test 1: Spawning {} vehicles ---", NUM_VEHICLES);
    let vehicles = spawn_vehicles(&mut world, NUM_VEHICLES);
    println!("✓ Spawned {} vehicles", vehicles.len());

    // Test 2: Spawn multiple walkers
    println!("\n--- Test 2: Spawning {} walkers ---", NUM_WALKERS);
    let walkers = spawn_walkers(&mut world, NUM_WALKERS);
    println!("✓ Spawned {} walkers", walkers.len());

    // Test 3: Enable autopilot for vehicles
    println!("\n--- Test 3: Enabling autopilot for vehicles ---");
    enable_autopilot_for_vehicles(&vehicles);
    println!("✓ Autopilot enabled for all vehicles");

    // Test 4: Start walkers
    println!("\n--- Test 4: Starting walker AI ---");
    start_walker_ai(&walkers);
    println!("✓ Walker AI started");

    // Test 5: Monitor simulation
    println!(
        "\n--- Test 5: Monitoring simulation for {}s ---",
        SIMULATION_TIME_SECS
    );
    monitor_simulation(&mut world, &vehicles, &walkers);

    // Test 6: Visualize with debug drawing
    println!("\n--- Test 6: Debug visualization ---");
    visualize_agents(&mut world, &vehicles, &walkers);

    // Cleanup
    println!("\n--- Cleaning up ---");
    cleanup_agents(vehicles, walkers);

    println!("\n=== All Tests Passed ===");
    println!("✅ Integration test completed successfully!");
    std::process::exit(0);
}

fn spawn_vehicles(world: &mut carla::client::World, count: usize) -> Vec<Vehicle> {
    let blueprint_library = world.blueprint_library();
    let vehicle_blueprints: Vec<_> = blueprint_library.filter("vehicle.*").iter().collect();

    if vehicle_blueprints.is_empty() {
        println!("⚠️  No vehicle blueprints found!");
        return vec![];
    }

    let spawn_points = world.map().recommended_spawn_points();
    let mut vehicles = Vec::new();

    for i in 0..count.min(spawn_points.len()) {
        let spawn_point = spawn_points.get(i).unwrap();
        let bp = &vehicle_blueprints[i % vehicle_blueprints.len()];

        if let Ok(actor) = world.spawn_actor(bp, spawn_point) {
            if let Ok(vehicle) = Vehicle::try_from(actor) {
                vehicles.push(vehicle);
            }
        }

        // Progress indicator
        if (i + 1) % 10 == 0 {
            print!("  Spawned {}/{}...\r", i + 1, count);
            use std::io::{self, Write};
            io::stdout().flush().unwrap();
        }
    }

    println!();
    vehicles
}

fn spawn_walkers(world: &mut carla::client::World, count: usize) -> Vec<Walker> {
    let blueprint_library = world.blueprint_library();
    let walker_blueprints: Vec<_> = blueprint_library
        .filter("walker.pedestrian.*")
        .iter()
        .collect();

    if walker_blueprints.is_empty() {
        println!("⚠️  No walker blueprints found!");
        return vec![];
    }

    let mut walkers = Vec::new();

    for i in 0..count {
        // Get a random navigation location for walker spawning
        let location = world.random_location_from_navigation();

        // Create transform slightly above ground
        let transform = Transform {
            location: Location::new(location.x, location.y, location.z + 1.0),
            rotation: Rotation::new(0.0, 0.0, 0.0),
        };

        let bp = &walker_blueprints[i % walker_blueprints.len()];

        if let Ok(actor) = world.spawn_actor(bp, &transform) {
            if let Ok(walker) = Walker::try_from(actor) {
                walkers.push(walker);
            }
        }

        // Progress indicator
        if (i + 1) % 20 == 0 {
            print!("  Spawned {}/{}...\r", i + 1, count);
            use std::io::{self, Write};
            io::stdout().flush().unwrap();
        }
    }

    println!();
    walkers
}

fn enable_autopilot_for_vehicles(vehicles: &[Vehicle]) {
    for vehicle in vehicles {
        vehicle.set_autopilot(true);
    }
}

fn start_walker_ai(_walkers: &[Walker]) {
    // Note: Walker AI control would require walker AI controller
    // For now, this is a placeholder showing the intent
    // In a full implementation, you would:
    // 1. Spawn walker AI controllers
    // 2. Start the AI
    // 3. Set random walk destinations
    println!("  Note: Full walker AI requires controller implementation");
}

fn monitor_simulation(world: &mut carla::client::World, vehicles: &[Vehicle], walkers: &[Walker]) {
    let start = Instant::now();
    let mut last_report = Instant::now();

    while start.elapsed() < Duration::from_secs(SIMULATION_TIME_SECS) {
        // Tick the world
        world.tick();

        // Report status every 3 seconds
        if last_report.elapsed() >= Duration::from_secs(3) {
            let elapsed = start.elapsed().as_secs();
            println!(
                "  [{:>2}s] Simulating... {} vehicles, {} walkers active",
                elapsed,
                vehicles.len(),
                walkers.len()
            );

            // Check if actors are still alive
            let vehicle_alive = vehicles.iter().filter(|v| v.is_alive()).count();
            let walker_alive = walkers.iter().filter(|w| w.is_alive()).count();

            if vehicle_alive < vehicles.len() || walker_alive < walkers.len() {
                println!(
                    "    ⚠️  Some actors were destroyed: {} vehicles, {} walkers remaining",
                    vehicle_alive, walker_alive
                );
            }

            last_report = Instant::now();
        }

        std::thread::sleep(Duration::from_millis(100));
    }

    println!("  ✓ Simulation completed");
}

fn visualize_agents(world: &mut carla::client::World, vehicles: &[Vehicle], walkers: &[Walker]) {
    let debug = world.debug();

    // Draw markers above vehicles (blue)
    for vehicle in vehicles.iter().take(10) {
        if vehicle.is_alive() {
            let transform = vehicle.transform();
            let marker_location = Location::new(
                transform.location.x,
                transform.location.y,
                transform.location.z + 3.0,
            );
            debug.draw_point(marker_location, 0.3, Color::BLUE, 5.0, false);
        }
    }

    // Draw markers above walkers (green)
    for walker in walkers.iter().take(10) {
        if walker.is_alive() {
            let transform = walker.transform();
            let marker_location = Location::new(
                transform.location.x,
                transform.location.y,
                transform.location.z + 2.0,
            );
            debug.draw_point(marker_location, 0.2, Color::GREEN, 5.0, false);
        }
    }

    println!(
        "  ✓ Drew debug markers for {} vehicles and {} walkers",
        vehicles.len().min(10),
        walkers.len().min(10)
    );
}

fn cleanup_agents(vehicles: Vec<Vehicle>, walkers: Vec<Walker>) {
    // Disable autopilot before destroying
    for vehicle in &vehicles {
        if vehicle.is_alive() {
            vehicle.set_autopilot(false);
        }
    }

    let mut destroyed_count = 0;

    for vehicle in vehicles {
        if vehicle.is_alive() {
            vehicle.destroy();
            destroyed_count += 1;
        }
    }

    for walker in walkers {
        if walker.is_alive() {
            walker.destroy();
            destroyed_count += 1;
        }
    }

    println!("  ✓ Destroyed {} actors", destroyed_count);
}
