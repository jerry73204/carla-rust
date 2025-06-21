//! Example demonstrating ActorList method chaining.

use anyhow::Result;
use carla::{client::Client, traits::ActorT};

fn main() -> Result<()> {
    // Connect to CARLA server
    let mut client = Client::new("127.0.0.1", 2000, None)?;
    let world = client.world()?;

    // Get all actors as an ActorList
    let actors = world.actors()?;

    println!("=== ActorList Method Chaining Demo ===");
    println!("Total actors in world: {}", actors.len());

    // Example 1: Find vehicles
    let vehicles = actors.find_by_type("vehicle");
    println!("\n1. All vehicles: {}", vehicles.len());

    // Example 2: Chain find_by_type and filter
    let fast_vehicles = actors
        .find_by_type("vehicle")
        .filter(|actor| actor.velocity().length() > 5.0);
    println!("2. Fast vehicles (>5 m/s): {}", fast_vehicles.len());

    // Example 3: Multiple filters chained
    let moving_static_vehicles = actors
        .find_by_type("vehicle")
        .filter(|actor| actor.velocity().length() > 0.1)
        .filter(|actor| actor.velocity().length() < 10.0);
    println!(
        "3. Moderately moving vehicles (0.1-10 m/s): {}",
        moving_static_vehicles.len()
    );

    // Example 4: Find walkers and filter by location
    let nearby_walkers = actors.find_by_type("walker").filter(|actor| {
        let pos = actor.transform().location;
        (pos.x.powi(2) + pos.y.powi(2)).sqrt() < 100.0 // Within 100m of origin
    });
    println!("4. Walkers within 100m of origin: {}", nearby_walkers.len());

    // Example 5: Complex query - sensors on vehicles
    let vehicle_sensors = actors
        .find_by_type("sensor")
        .filter(|actor| actor.type_id().contains("camera") || actor.type_id().contains("lidar"));
    println!("5. Camera/LiDAR sensors: {}", vehicle_sensors.len());

    // Example 6: Convert filtered result to Vec when needed
    let static_actors_vec: Vec<_> = actors
        .filter(|actor| actor.velocity().length() < 0.01)
        .to_vec();
    println!("6. Static actors (as Vec): {}", static_actors_vec.len());

    // Example 7: Use any() and all() with filtered lists
    let has_fast_vehicles = vehicles.any(|actor| actor.velocity().length() > 20.0);
    let all_vehicles_alive = vehicles.all(|actor| actor.is_alive());
    println!("7. Has fast vehicles (>20 m/s): {}", has_fast_vehicles);
    println!("   All vehicles alive: {}", all_vehicles_alive);

    // Example 8: Iterate through filtered results
    println!("\n8. Iterating through fast vehicles:");
    for (i, actor) in fast_vehicles.iter().enumerate().take(3) {
        println!(
            "   {}. {} (ID: {}, speed: {:.2} m/s)",
            i + 1,
            actor.type_id(),
            actor.id(),
            actor.velocity().length()
        );
    }

    // Example 9: Get IDs of filtered actors without loading full data
    let fast_vehicle_ids: Vec<u32> = fast_vehicles.ids().to_vec();
    println!(
        "\n9. Fast vehicle IDs: {:?}",
        &fast_vehicle_ids[..fast_vehicle_ids.len().min(5)]
    );

    // Example 10: Compare efficiency - chained vs separate calls
    println!("\n10. Performance comparison:");

    // Method 1: Chain operations (efficient)
    let chained_result = actors
        .find_by_type("vehicle")
        .filter(|actor| actor.velocity().length() > 1.0);
    println!("    Chained operations: {} results", chained_result.len());

    // Method 2: Convert to Vec then filter (less efficient)
    let vehicles_vec = actors.find_by_type("vehicle").to_vec();
    let filtered_vec: Vec<_> = vehicles_vec
        .into_iter()
        .filter(|actor| actor.velocity().length() > 1.0)
        .collect();
    println!("    Vec-based operations: {} results", filtered_vec.len());

    println!("\n=== Demo Complete ===");
    Ok(())
}
