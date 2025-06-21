//! Example demonstrating the use of ActorList for efficient actor iteration.

use carla::{client::Client, error::CarlaResult, traits::ActorT};

fn main() -> CarlaResult<()> {
    // Connect to CARLA server
    let mut client = Client::new("localhost", 2000, None)?;
    let world = client.world()?;

    // Get all actors in the world
    let actors = world.actors()?;
    println!("Total actors in world: {}", actors.len());

    // Iterate through actors without loading all at once
    println!("\nIterating through actors:");
    for actor in &actors {
        println!("  - {} (ID: {})", actor.type_id(), actor.id());
    }

    // Find all vehicles efficiently
    let vehicles = actors.find_by_type("vehicle");
    println!("\nFound {} vehicles", vehicles.len());

    // Check if any vehicle is moving fast
    let fast_threshold = 13.89; // 50 km/h = 13.89 m/s
    let has_fast_vehicle = actors.any(|actor| {
        actor.type_id().contains("vehicle") && actor.velocity().length() > fast_threshold
    });
    println!(
        "\nAny vehicle moving > {}m/s (50km/h)? {}",
        fast_threshold, has_fast_vehicle
    );

    // Get traffic lights using specialized methods
    let traffic_lights = world.traffic_lights()?;
    println!("\nTraffic lights in world: {}", traffic_lights.len());

    // Get traffic lights in a specific junction (example junction ID)
    let junction_id = 1;
    let junction_lights = world.traffic_lights_in_junction(junction_id)?;
    println!(
        "Traffic lights in junction {}: {}",
        junction_id,
        junction_lights.len()
    );

    // Example: Find traffic lights near a waypoint
    if let Ok(map) = world.map() {
        // Get a waypoint at origin
        let location = carla::geom::Location::new(0.0, 0.0, 0.0);
        if let Some(waypoint) = map.waypoint(location) {
            let nearby_lights = world.traffic_lights_from_waypoint(&waypoint, 50.0)?;
            println!(
                "Traffic lights within 50m of origin: {}",
                nearby_lights.len()
            );
        }
    }

    // Apply a function to each actor
    let mut vehicle_count = 0;
    let mut pedestrian_count = 0;
    actors.for_each(|actor| {
        if actor.type_id().contains("vehicle") {
            vehicle_count += 1;
        } else if actor.type_id().contains("walker.pedestrian") {
            pedestrian_count += 1;
        }
    });
    println!(
        "\nVehicles: {}, Pedestrians: {}",
        vehicle_count, pedestrian_count
    );

    // Filter actors by custom predicate
    let static_actors = actors.filter(|actor| {
        actor.velocity().length() < 0.1 // Nearly stationary
    });
    println!("Static actors: {}", static_actors.len());

    Ok(())
}
