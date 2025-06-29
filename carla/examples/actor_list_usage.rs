//! Example demonstrating how to use ActorList efficiently.

use anyhow::Result;
use carla::{actor::ActorExt, client::Client};

fn main() -> Result<()> {
    // Connect to CARLA server
    let client = Client::new("127.0.0.1", 2000, None)?;
    let world = client.world()?;

    // Get all actors as an ActorList (efficient - no conversion)
    let actors = world.actors()?;

    println!("Total actors in world: {}", actors.len());

    // Iterate through actors without converting to Vec
    println!("\nIterating through actors:");
    for actor in &actors {
        println!("  - {} (ID: {})", actor.type_id(), actor.id());
    }

    // Get just the actor IDs without fetching full actor data
    println!("\nActor IDs only:");
    for id in actors.ids() {
        println!("  - ID: {id}");
    }

    // Find specific types of actors
    let vehicles = actors.find_by_type("vehicle");
    println!("\nFound {} vehicles", vehicles.len());

    let walkers = actors.find_by_type("walker");
    println!("Found {} walkers", walkers.len());

    // Find a specific actor by ID
    if let Some(first_id) = actors.ids().first() {
        if let Some(actor) = actors.find_by_id(*first_id) {
            println!("\nFound actor with ID {}: {}", first_id, actor.type_id());
        }
    }

    // Filter actors using a custom predicate
    let moving_actors = actors.filter(|actor| {
        let velocity = actor.velocity();
        velocity.length() > 0.1 // Moving faster than 0.1 m/s
    });
    println!("\n{} actors are moving", moving_actors.len());

    // Check if any vehicle is moving fast
    let has_fast_vehicle = actors.any(|actor| {
        actor.type_id().contains("vehicle") && actor.velocity().length() > 20.0 // > 20 m/s
    });
    println!("\nAny vehicle moving fast? {has_fast_vehicle}");

    // Get traffic lights efficiently
    let traffic_lights = world.traffic_lights()?;
    println!("\nFound {} traffic lights", traffic_lights.len());

    // Convert to Vec only when needed
    if actors.len() < 100 {
        let actor_vec = actors.to_vec();
        println!("\nConverted {} actors to Vec", actor_vec.len());
    }

    Ok(())
}
