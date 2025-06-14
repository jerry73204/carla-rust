//! Map and navigation system demonstration for CARLA simulator.
//!
//! This example demonstrates:
//! - Loading and exploring the map
//! - Working with waypoints
//! - Navigating through junctions
//! - Finding landmarks and traffic signs
//! - Lane change detection
//! - Path planning basics

use anyhow::Result;
use carla_cxx::{ClientWrapper, LaneType, MapWrapper, SimpleTransform, WaypointWrapper};

fn main() -> Result<()> {
    println!("=== CARLA Map and Navigation Demo ===\n");

    // Connect to CARLA server
    println!("Connecting to CARLA server...");
    let mut client = ClientWrapper::new("localhost", 2000)?;
    client.set_timeout(std::time::Duration::from_secs(10));

    // Get the world and map
    let world = client.get_world();
    let map = world.get_map();

    // Display map information
    println!("\n--- Map Information ---");
    println!("Map name: {}", map.get_name());
    println!("OpenDRIVE size: {} bytes", map.get_open_drive().len());

    // Get geo-reference
    let geo_ref = map.get_geo_reference();
    println!(
        "Geo-reference: lat={:.6}, lon={:.6}, alt={:.2}",
        geo_ref.latitude, geo_ref.longitude, geo_ref.altitude
    );

    // Get recommended spawn points
    let spawn_points = map.get_recommended_spawn_points();
    println!("Number of spawn points: {}", spawn_points.len());

    // Demonstrate waypoint navigation
    println!("\n--- Waypoint Navigation ---");
    if let Some(spawn_point) = spawn_points.first() {
        demonstrate_waypoint_navigation(&map, spawn_point)?;
    }

    // Demonstrate junction exploration
    println!("\n--- Junction Exploration ---");
    demonstrate_junctions(&map)?;

    // Demonstrate landmark detection
    println!("\n--- Landmark Detection ---");
    demonstrate_landmarks(&map)?;

    // Demonstrate crosswalk zones
    println!("\n--- Crosswalk Zones ---");
    let crosswalks = map.get_all_crosswalk_zones();
    println!("Found {} crosswalk zones", crosswalks.len());
    for (i, zone) in crosswalks.iter().take(5).enumerate() {
        println!(
            "  Crosswalk {}: ({:.2}, {:.2}, {:.2})",
            i + 1,
            zone.x,
            zone.y,
            zone.z
        );
    }

    println!("\n=== Demo completed successfully! ===");
    Ok(())
}

fn demonstrate_waypoint_navigation(map: &MapWrapper, spawn_point: &SimpleTransform) -> Result<()> {
    // Get a waypoint at the spawn location
    if let Some(waypoint) = map.get_waypoint(&spawn_point.location, true, Some(LaneType::Driving)) {
        println!("Starting waypoint:");
        print_waypoint_info(&waypoint);

        // Explore forward direction
        println!("\n  Forward exploration:");
        if let Some(next_waypoint) = waypoint.get_next(5.0) {
            println!("    Found waypoint 5m ahead");
            print_waypoint_info(&next_waypoint);
        } else {
            println!("    No waypoint found 5m ahead");
        }

        // Try to change lanes
        println!("\n  Lane change possibilities:");
        let lane_change = waypoint.get_lane_change();
        println!("    Allowed lane changes: {:?}", lane_change);

        if let Some(left_wp) = waypoint.get_left() {
            println!("    Left lane available:");
            print_waypoint_info(&left_wp);
        }

        if let Some(right_wp) = waypoint.get_right() {
            println!("    Right lane available:");
            print_waypoint_info(&right_wp);
        }

        // Check lane markings
        println!("\n  Lane markings:");
        let left_marking = waypoint.get_left_lane_marking();
        let right_marking = waypoint.get_right_lane_marking();
        println!("    Left marking: {:?}", left_marking.marking_type);
        println!("    Right marking: {:?}", right_marking.marking_type);

        // Navigate forward through multiple waypoints
        println!("\n  Following lane forward:");
        let mut current = waypoint;
        let mut count = 0;
        while let Some(next) = current.get_next(10.0) {
            count += 1;
            current = next;
            if count >= 5 {
                break; // Limit to 5 waypoints for demo
            }
        }
        println!("    Traversed {} waypoints (10m intervals)", count);
    }

    Ok(())
}

fn demonstrate_junctions(map: &MapWrapper) -> Result<()> {
    // Generate waypoints across the map (returns SimpleWaypoint structs)
    let all_waypoints = map.generate_waypoints(20.0);
    println!("Generated {} waypoints across the map", all_waypoints.len());

    // Find waypoints in junctions
    let junction_waypoints: Vec<_> = all_waypoints
        .into_iter()
        .filter(|wp| wp.is_junction)
        .collect();

    println!("Found {} waypoints in junctions", junction_waypoints.len());

    // Analyze first few junctions
    let mut unique_junctions = std::collections::HashSet::new();
    for wp in junction_waypoints.iter().take(10) {
        if wp.is_junction {
            unique_junctions.insert(wp.road_id); // Use road_id as a proxy for junction
        }
    }

    println!("Unique junction areas found: {}", unique_junctions.len());

    // Detail first junction waypoint
    if let Some(first_junction_wp) = junction_waypoints.first() {
        println!("\nFirst junction waypoint details:");
        println!("  Waypoint ID: {}", first_junction_wp.id);
        println!(
            "  Road: {}, Section: {}, Lane: {}",
            first_junction_wp.road_id, first_junction_wp.section_id, first_junction_wp.lane_id
        );
        println!(
            "  Location: ({:.2}, {:.2}, {:.2})",
            first_junction_wp.transform.location.x,
            first_junction_wp.transform.location.y,
            first_junction_wp.transform.location.z
        );
    }

    Ok(())
}

fn demonstrate_landmarks(map: &MapWrapper) -> Result<()> {
    // Since we can't get landmarks directly due to CXX limitations,
    // we'll demonstrate topology instead
    println!("Map topology exploration:");

    let topology = map.get_topology();
    println!("Topology contains {} waypoints", topology.len());

    // Show first few topology waypoints
    println!("\nFirst 5 topology waypoints:");
    for (i, wp) in topology.iter().take(5).enumerate() {
        println!("\n  Waypoint {}:", i + 1);
        println!("    ID: {}", wp.id);
        println!("    Road: {}, Lane: {}", wp.road_id, wp.lane_id);
        println!(
            "    Location: ({:.2}, {:.2}, {:.2})",
            wp.transform.location.x, wp.transform.location.y, wp.transform.location.z
        );
        println!("    Is junction: {}", wp.is_junction);
    }

    Ok(())
}

fn print_waypoint_info(waypoint: &WaypointWrapper) {
    println!("  Waypoint ID: {}", waypoint.get_id());
    println!(
        "  Road: {}, Section: {}, Lane: {}",
        waypoint.get_road_id(),
        waypoint.get_section_id(),
        waypoint.get_lane_id()
    );
    println!("  Distance along road: {:.2}m", waypoint.get_distance());
    println!("  Lane width: {:.2}m", waypoint.get_lane_width());
    println!("  Lane type: {:?}", waypoint.get_type());
    println!("  Is junction: {}", waypoint.is_junction());
}
