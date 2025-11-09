//! Waypoint visualization demonstration.
//!
//! This example demonstrates the `draw_waypoints()` utility function
//! for visualizing navigation waypoints in the CARLA simulator.
//!
//! # Usage
//! ```bash
//! cargo run --example waypoint_visualization --profile dev-release
//! ```

use anyhow::Result;
use carla::{agents::tools::draw_waypoints, client::Client};

fn main() -> Result<()> {
    println!("Waypoint Visualization Demo - Connecting to CARLA...");

    // Connect to CARLA
    let client = Client::connect("localhost", 2000, 0);
    let mut world = client.world();
    let map = world.map();

    println!("Connected! Visualizing waypoints...");

    // Get a starting location from spawn points
    let spawn_points = map.recommended_spawn_points();
    let spawn_points_slice = spawn_points.as_slice();
    if spawn_points_slice.is_empty() {
        return Err(anyhow::anyhow!("No spawn points available"));
    }

    let start_location = spawn_points_slice[0].location;
    println!(
        "Start location: ({:.1}, {:.1}, {:.1})",
        start_location.x, start_location.y, start_location.z
    );

    // Get waypoint at start location
    let waypoint = map
        .waypoint_at(&start_location)
        .ok_or_else(|| anyhow::anyhow!("No waypoint found at start location"))?;
    println!("Got waypoint on road {}", waypoint.road_id());

    // Get debug helper for drawing
    let debug = world.debug();

    // Example 1: Visualize next waypoints along the road
    println!("\nExample 1: Next waypoints (every 5m)");
    let next_waypoints = waypoint.next(5.0);
    println!("  Found {} next waypoint(s)", next_waypoints.len());
    // Use the iter() method to iterate without allocating - zero-cost!
    draw_waypoints(&debug, next_waypoints.iter(), 0.5, 10.0);

    // Example 2: Visualize waypoints further ahead
    if let Some(next_wp) = next_waypoints.get(0) {
        println!("\nExample 2: Waypoints 10m ahead");
        let further = next_wp.next(10.0);
        println!("  Found {} waypoint(s)", further.len());
        draw_waypoints(&debug, further.iter(), 0.7, 10.0);
    } else {
        println!("\nExample 2: Skipped (no next waypoints)");
    }

    // Example 3: Visualize waypoints until lane end
    println!("\nExample 3: Waypoints until lane end");
    let until_end = waypoint.next_until_lane_end(5.0);
    println!("  Found {} waypoint(s) until lane end", until_end.len());
    // Draw these with higher z-offset for distinction
    draw_waypoints(&debug, until_end.iter(), 1.0, 10.0);

    // Example 4: Visualize adjacent lane waypoints if available
    println!("\nExample 4: Adjacent lane waypoints");
    if waypoint.left_lane_marking().is_some() {
        println!("  Found left lane marking");
        // Get left lane waypoint and draw it
        if let Some(left_wp) = waypoint.left() {
            let left_wps = vec![left_wp];
            draw_waypoints(&debug, &left_wps, 0.5, 10.0);
        }
    }

    if waypoint.right_lane_marking().is_some() {
        println!("  Found right lane marking");
        // Get right lane waypoint and draw it
        if let Some(right_wp) = waypoint.right() {
            let right_wps = vec![right_wp];
            draw_waypoints(&debug, &right_wps, 0.5, 10.0);
        }
    }

    // Example 5: Visualize a path to a destination
    println!("\nExample 5: Path to destination");
    if spawn_points_slice.len() >= 2 {
        let dest_location = spawn_points_slice[1].location;
        println!(
            "  Destination: ({:.1}, {:.1}, {:.1})",
            dest_location.x, dest_location.y, dest_location.z
        );

        // Get waypoints along a simple path (using next() repeatedly)
        let mut path_waypoints = vec![waypoint.clone()];
        let mut current = waypoint;

        for _ in 0..20 {
            let next = current.next(5.0);
            if let Some(next_wp) = next.get(0) {
                path_waypoints.push(next_wp.clone());
                current = next_wp;
            } else {
                break;
            }
        }

        println!("  Drawing path with {} waypoints", path_waypoints.len());
        // Can pass a Vec or slice directly too!
        draw_waypoints(&debug, &path_waypoints, 0.5, 15.0);
    }

    println!("\nVisualization complete!");
    println!("Waypoints are drawn as green arrows showing direction.");
    println!("Arrows will persist for 10-15 seconds.");
    println!("Open CARLA simulator to see the visualization.");
    println!();
    println!("Note: draw_waypoints() accepts:");
    println!("  - WaypointList::iter() - zero-cost lazy iteration");
    println!("  - &[Waypoint] - slices");
    println!("  - &Vec<Waypoint> - vectors");

    Ok(())
}
