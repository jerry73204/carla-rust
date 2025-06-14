//! World interaction demonstration for carla-cxx.
//!
//! This example demonstrates the world interaction features:
//! - Ray casting between points
//! - Point projection along directions
//! - Ground projection for finding surface intersections
//! - Traffic light queries from waypoints and junctions
//! - Pedestrian navigation and crossing behavior
//! - Actor queries and filtering

use anyhow::Result;
use carla_cxx::{
    ray_casting, world_defaults, ActorListExt, ClientWrapper, LabelledPoint, LaneType,
    OptionalLabelledPointExt, OptionalLocationExt, SimpleLocation,
};
use std::time::Duration;

fn main() -> Result<()> {
    println!("🌍 CARLA World Interaction Demo");
    println!("================================");

    // Connect to CARLA server
    println!("🔌 Connecting to CARLA server...");
    let mut client = ClientWrapper::new("localhost", 2000)?;
    println!(
        "✅ Connected! Server version: {}",
        client.get_server_version()
    );

    // Set timeout for client operations
    client.set_timeout(Duration::from_secs(10));

    // Get the simulation world
    let world = client.get_world();
    println!("🌍 World ID: {}", world.get_id());

    // === RAY CASTING DEMONSTRATIONS ===
    println!("\n🔬 Ray Casting Demonstrations");
    println!("-----------------------------");

    // Demo 1: Basic ray casting between two points
    let start_point = SimpleLocation {
        x: 0.0,
        y: 0.0,
        z: 10.0,
    };
    let end_point = SimpleLocation {
        x: 100.0,
        y: 0.0,
        z: -10.0,
    };

    println!(
        "📍 Casting ray from ({:.1}, {:.1}, {:.1}) to ({:.1}, {:.1}, {:.1})",
        start_point.x, start_point.y, start_point.z, end_point.x, end_point.y, end_point.z
    );

    let ray_results = world.cast_ray(&start_point, &end_point);
    println!(
        "🎯 Ray casting found {} intersection points:",
        ray_results.len()
    );

    for (i, point) in ray_results.iter().enumerate() {
        let labelled = LabelledPoint::from(*point);
        println!(
            "   {}. Location: ({:.2}, {:.2}, {:.2}) - Label: {:?}",
            i + 1,
            labelled.location.x,
            labelled.location.y,
            labelled.location.z,
            labelled.label
        );
    }

    // Demo 2: Point projection downward for ground finding
    let test_location = SimpleLocation {
        x: 50.0,
        y: 20.0,
        z: 50.0,
    };
    let downward_direction = ray_casting::downward_ray(test_location, 100.0);

    println!(
        "\n📍 Projecting point ({:.1}, {:.1}, {:.1}) downward to find ground...",
        test_location.x, test_location.y, test_location.z
    );

    let projection_result = world.project_point(
        &test_location,
        &downward_direction,
        world_defaults::POINT_PROJECTION_DISTANCE,
    );

    if projection_result.is_some() {
        let point = projection_result.get().unwrap();
        println!(
            "🎯 Ground found at: ({:.2}, {:.2}, {:.2}) - Label: {:?}",
            point.location.x, point.location.y, point.location.z, point.label
        );
    } else {
        println!("❌ No ground intersection found");
    }

    // Demo 3: Ground projection (specialized downward projection)
    println!(
        "\n📍 Using ground projection for location ({:.1}, {:.1}, {:.1})...",
        test_location.x, test_location.y, test_location.z
    );

    let ground_result =
        world.ground_projection(&test_location, world_defaults::GROUND_PROJECTION_DISTANCE);

    if ground_result.is_some() {
        let point = ground_result.get().unwrap();
        println!(
            "🎯 Ground projection: ({:.2}, {:.2}, {:.2}) - Label: {:?}",
            point.location.x, point.location.y, point.location.z, point.label
        );
    } else {
        println!("❌ No ground found within search distance");
    }

    // === ACTOR QUERIES ===
    println!("\n👥 Actor Query Demonstrations");
    println!("-----------------------------");

    // Demo 4: Get all actors in the world
    let all_actors = world.get_actors();
    println!("🎭 Total actors in world: {}", all_actors.len());

    if !all_actors.is_empty() {
        println!("📋 First 10 actor IDs:");
        for (i, &actor_id) in all_actors.get_ids().iter().take(10).enumerate() {
            println!("   {}. Actor ID: {}", i + 1, actor_id);
        }

        // Demo 5: Get specific actors by IDs
        let first_few_ids = &all_actors.get_ids()[..std::cmp::min(3, all_actors.len())];
        println!("\n🔍 Querying specific actors by IDs: {:?}", first_few_ids);
        let specific_actors = world.get_actors_by_ids(first_few_ids);
        println!("✅ Retrieved {} specific actors", specific_actors.len());

        // Demo 6: Get individual actor details
        if let Some(&first_id) = all_actors.get_ids().first() {
            println!("\n🔍 Getting details for actor ID {}", first_id);
            if let Some(actor) = world.get_actor(first_id) {
                println!("✅ Actor found:");
                println!("   🆔 ID: {}", actor.get_id());
                println!("   🏷️  Type: {}", actor.get_type_id());
                println!(
                    "   📍 Location: ({:.2}, {:.2}, {:.2})",
                    actor.get_location().x,
                    actor.get_location().y,
                    actor.get_location().z
                );
            } else {
                println!("❌ Actor with ID {} not found", first_id);
            }
        }
    }

    // === NAVIGATION AND TRAFFIC QUERIES ===
    println!("\n🚦 Navigation and Traffic Demonstrations");
    println!("---------------------------------------");

    // Demo 7: Get random navigation location
    let random_nav_location = world.get_random_location_from_navigation();
    if random_nav_location.is_some() {
        let location = random_nav_location.get().unwrap();
        println!(
            "🎲 Random navigation location: ({:.2}, {:.2}, {:.2})",
            location.x, location.y, location.z
        );
    } else {
        println!("❌ No random navigation location available");
    }

    // Demo 8: Set pedestrian crossing behavior
    println!("🚶 Setting pedestrian cross factor to 80% compliance...");
    world.set_pedestrians_cross_factor(0.8);
    println!("✅ Pedestrian crossing behavior updated");

    // Demo 9: Traffic light queries (requires waypoints and traffic lights)
    let map = world.get_map();
    let spawn_points = map.get_recommended_spawn_points();

    if !spawn_points.is_empty() {
        let test_transform = spawn_points[0];
        let test_waypoint =
            map.get_waypoint(&test_transform.location, true, Some(LaneType::Driving));

        if let Some(waypoint_wrapper) = test_waypoint {
            println!("\n🚦 Querying traffic lights near waypoint...");
            let nearby_lights = world.get_traffic_lights_from_waypoint(
                waypoint_wrapper.get_waypoint(),
                world_defaults::TRAFFIC_LIGHT_QUERY_DISTANCE,
            );
            println!(
                "🔍 Found {} traffic lights within {:.1}m",
                nearby_lights.len(),
                world_defaults::TRAFFIC_LIGHT_QUERY_DISTANCE
            );

            if !nearby_lights.is_empty() {
                println!("🚦 Traffic light IDs: {:?}", nearby_lights.get_ids());
            }
        }
    }

    // === DEMONSTRATION SUMMARY ===
    println!("\n📊 World Interaction Demo Summary");
    println!("==================================");
    println!("✅ Ray casting: Demonstrated intersection finding");
    println!("✅ Point projection: Showed directional intersection queries");
    println!("✅ Ground projection: Found ground intersections");
    println!("✅ Actor queries: Retrieved and inspected world actors");
    println!("✅ Navigation: Accessed random navigation locations");
    println!("✅ Traffic queries: Searched for nearby traffic lights");
    println!("✅ Pedestrian behavior: Configured crossing compliance");

    println!("\n🎉 World interaction demonstration completed successfully!");
    println!("💡 These features enable advanced autonomous driving scenarios:");
    println!("   - 🔍 Sensor simulation and obstacle detection");
    println!("   - 🗺️  Path planning and navigation");
    println!("   - 🚦 Traffic-aware behavior");
    println!("   - 👥 Multi-agent coordination");

    Ok(())
}
