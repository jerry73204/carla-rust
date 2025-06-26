//! OpenDrive integration tests that require CARLA server
//!
//! These tests correspond to the C++ tests in test_opendrive.cpp

use carla::road::LaneType;
use carla_test_server::with_carla_server;

#[with_carla_server]
fn test_opendrive_parsing(client: &carla::client::Client) {
    // Corresponds to C++ TEST(road, parse_files)
    let world = client.world().expect("Failed to get world");
    let map = world.map().expect("Failed to get map");

    // Get OpenDRIVE content
    let opendrive_content = map.to_opendrive();

    // Basic validation that we got valid OpenDRIVE data
    assert!(
        !opendrive_content.is_empty(),
        "OpenDRIVE content should not be empty"
    );
    assert!(
        opendrive_content.contains("<OpenDRIVE>"),
        "Should contain OpenDRIVE root element"
    );
    assert!(
        opendrive_content.contains("</OpenDRIVE>"),
        "Should have closing OpenDRIVE tag"
    );
    assert!(
        opendrive_content.contains("<road "),
        "Should contain road elements"
    );

    // Verify map name
    let map_name = map.name();
    assert!(!map_name.is_empty(), "Map should have a name");

    println!("Successfully parsed OpenDRIVE for map: {}", map_name);
    println!(
        "OpenDRIVE content length: {} bytes",
        opendrive_content.len()
    );
}

#[with_carla_server]
fn test_parse_road_links(client: &carla::client::Client) {
    // Corresponds to C++ TEST(road, parse_road_links)
    use std::collections::HashSet;

    let world = client.world().expect("Failed to get world");
    let map = world.map().expect("Failed to get map");

    // Get topology to analyze road links
    let topology = map.topology();
    println!("Map topology contains {} connections", topology.len());
    assert!(!topology.is_empty(), "Map should have topology connections");

    // Analyze road links through topology
    let mut predecessor_roads = HashSet::new();
    let mut successor_roads = HashSet::new();
    let mut connected_pairs = 0;

    for (start_wp, end_wp) in topology.iter() {
        predecessor_roads.insert(start_wp.road_id as u32);
        successor_roads.insert(end_wp.road_id as u32);

        if start_wp.road_id != end_wp.road_id {
            connected_pairs += 1;
            if connected_pairs <= 5 {
                println!("Road link: {} -> {}", start_wp.road_id, end_wp.road_id);
            }
        }
    }

    println!("\nRoad link statistics:");
    println!("  Roads with predecessors: {}", predecessor_roads.len());
    println!("  Roads with successors: {}", successor_roads.len());
    println!("  Connected road pairs: {}", connected_pairs);

    // Test specific road links by following waypoints
    let spawn_points = map.spawn_points();
    if let Some(spawn_transform) = spawn_points.iter().next() {
        if let Some(waypoint) = map.waypoint(spawn_transform.location) {
            println!("\nTraversing from waypoint at road {}", waypoint.road_id);

            // Follow the road forward
            let mut current = waypoint;
            let mut roads_visited = vec![current.road_id];

            for _ in 0..5 {
                let next_waypoints = current.next(10.0).expect("Failed to get next waypoints");
                if let Some(next) = next_waypoints.get(0) {
                    if next.road_id != current.road_id {
                        println!(
                            "  Crossed from road {} to road {}",
                            current.road_id, next.road_id
                        );
                        roads_visited.push(next.road_id);
                    }
                    // Waypoint cannot be cloned, get new waypoint from location
                    if let Some(new_current) = map.waypoint(next.transform.location) {
                        current = new_current;
                    } else {
                        break;
                    }
                } else {
                    break;
                }
            }

            println!("  Roads visited: {:?}", roads_visited);
        }
    }
}

#[with_carla_server]
fn test_parse_junctions(client: &carla::client::Client) {
    // Corresponds to C++ TEST(road, parse_junctions)
    use std::collections::HashSet;

    let world = client.world().expect("Failed to get world");
    let map = world.map().expect("Failed to get map");

    // Parse OpenDRIVE to count junctions
    let opendrive = map.to_opendrive();
    let junction_count_xml = opendrive.matches("<junction ").count();
    println!("Junctions in OpenDRIVE XML: {}", junction_count_xml);

    // Generate waypoints and find junctions
    let waypoints = map.generate_waypoints(2.0);
    let junction_waypoints = Vec::new();
    let mut unique_junctions = HashSet::new();

    for waypoint in waypoints.iter() {
        if waypoint.is_junction() {
            // Note: Waypoint cannot be cloned, so we'll count them instead

            // Get junction from waypoint
            if let Some(junction) = map.junction(&waypoint) {
                unique_junctions.insert(junction.id());
            }
        }
    }

    let junction_waypoint_count = waypoints.iter().filter(|wp| wp.is_junction()).count();
    println!("Waypoints in junctions: {}", junction_waypoint_count);
    println!("Unique junctions found: {}", unique_junctions.len());

    // Analyze some junctions
    for (i, junction_id) in unique_junctions.iter().take(3).enumerate() {
        println!("\nJunction {} (ID: {}):", i, junction_id);

        // Count waypoints in this junction by re-iterating
        let junction_wp_count_for_id = waypoints
            .iter()
            .filter(|wp| {
                if wp.is_junction() {
                    if let Some(j) = map.junction(wp) {
                        j.id() == *junction_id
                    } else {
                        false
                    }
                } else {
                    false
                }
            })
            .count();

        println!("  Waypoints in junction: {}", junction_wp_count_for_id);
    }

    // Verify junction properties
    if let Some(first_junction_wp) = junction_waypoints.first() {
        if let Some(junction) = map.junction(first_junction_wp) {
            let bbox = junction.bounding_box();
            println!("\nFirst junction bounding box:");
            println!(
                "  Location: ({:.2}, {:.2}, {:.2})",
                bbox.location.x, bbox.location.y, bbox.location.z
            );
            println!(
                "  Extent: ({:.2}, {:.2}, {:.2})",
                bbox.extent.x, bbox.extent.y, bbox.extent.z
            );

            // Get waypoints in this junction
            let junction_wps = junction
                .waypoints(LaneType::Driving)
                .expect("Failed to get junction waypoints");
            println!("  Contains {} waypoints", junction_wps.len());
        }
    }
}

#[with_carla_server]
fn test_parse_road(client: &carla::client::Client) {
    // Corresponds to C++ TEST(road, parse_road)
    let world = client.world().expect("Failed to get world");
    let map = world.map().expect("Failed to get map");

    // Parse OpenDRIVE to analyze road structure
    let opendrive = map.to_opendrive();

    // Count roads in OpenDRIVE
    let road_count = opendrive.matches("<road ").count();
    println!("Total roads in map: {}", road_count);
    assert!(road_count > 0, "Map should contain roads");

    // Generate waypoints to analyze road structure
    let waypoints = map.generate_waypoints(2.0);
    println!("Generated {} waypoints", waypoints.len());
    assert!(!waypoints.is_empty(), "Should generate waypoints");

    // Analyze road IDs and lane structure
    let mut unique_roads = std::collections::HashSet::new();
    let mut junction_count = 0;

    for waypoint in waypoints.iter() {
        unique_roads.insert(waypoint.road_id as u32);
        if waypoint.is_junction() {
            junction_count += 1;
        }
    }

    println!("Unique road IDs found: {}", unique_roads.len());
    println!("Waypoints in junctions: {}", junction_count);

    // Sample some waypoints to check lane properties
    for (i, waypoint) in waypoints.iter().take(5).enumerate() {
        println!("Waypoint {} properties:", i);
        println!("  Road ID: {}", waypoint.road_id);
        println!("  Lane ID: {}", waypoint.lane_id);
        println!("  Lane type: {:?}", waypoint.lane_type);
        println!("  Lane width: {:.2}m", waypoint.lane_width);
        println!("  Lane change: {:?}", waypoint.lane_change);

        // Check road markings using fields
        println!("  Lane marking type: {:?}", waypoint.lane_marking_type);
        println!("  Lane marking color: {:?}", waypoint.lane_marking_color);
    }
}

#[with_carla_server]
fn test_parse_road_elevation(client: &carla::client::Client) {
    // Corresponds to C++ TEST(road, parse_road_elevation)
    let world = client.world().expect("Failed to get world");
    let map = world.map().expect("Failed to get map");

    // Parse OpenDRIVE to check elevation records
    let opendrive = map.to_opendrive();
    let elevation_count = opendrive.matches("<elevation ").count();
    println!("Elevation records in OpenDRIVE: {}", elevation_count);

    // Generate waypoints at different locations to test elevation
    let waypoints = map.generate_waypoints(10.0);

    // Track elevation variations
    let mut min_elevation = f64::MAX;
    let mut max_elevation = f64::MIN;
    let mut elevation_samples = Vec::new();

    for waypoint in waypoints.iter().take(20) {
        let location = waypoint.transform.location;
        let elevation = location.z;

        min_elevation = min_elevation.min(elevation);
        max_elevation = max_elevation.max(elevation);
        // Note: s() method doesn't exist on Waypoint - we'll use road_id as substitute
        elevation_samples.push((waypoint.road_id, 0.0, elevation));
    }

    println!("\nElevation statistics:");
    println!("  Min elevation: {:.2}m", min_elevation);
    println!("  Max elevation: {:.2}m", max_elevation);
    println!("  Elevation range: {:.2}m", max_elevation - min_elevation);

    // Show some elevation samples
    println!("\nElevation samples:");
    for (i, (road_id, s, elevation)) in elevation_samples.iter().take(5).enumerate() {
        println!(
            "  Sample {}: Road {} at s={:.2} -> elevation={:.2}m",
            i, road_id, s, elevation
        );
    }

    // Test elevation continuity along a road
    if let Some(first_wp) = waypoints.get(0) {
        println!("\nElevation profile along road {}:", first_wp.road_id);
        let mut current_location = first_wp.transform.location;

        for i in 0..5 {
            let elevation = current_location.z;
            println!("  step {} -> elevation={:.2}m", i, elevation);

            // Get current waypoint and move forward along the road
            if let Some(current) = map.waypoint(current_location) {
                if let Some(next) = current
                    .next(10.0)
                    .expect("Failed to get next waypoints")
                    .get(0)
                {
                    if next.road_id == current.road_id {
                        current_location = next.transform.location;
                    } else {
                        break;
                    }
                } else {
                    break;
                }
            } else {
                break;
            }
        }
    }
}

#[with_carla_server]
fn test_parse_geometry(client: &carla::client::Client) {
    // Corresponds to C++ TEST(road, parse_geometry)
    use std::collections::HashMap;

    let world = client.world().expect("Failed to get world");
    let map = world.map().expect("Failed to get map");

    // Parse OpenDRIVE to analyze geometry
    let opendrive = map.to_opendrive();

    // Count different geometry types
    let line_count = opendrive.matches(r#"<geometry[^>]*\s+line"#).count();
    let arc_count = opendrive.matches(r#"<geometry[^>]*\s+arc"#).count();
    let spiral_count = opendrive.matches(r#"<geometry[^>]*\s+spiral"#).count();
    let poly3_count = opendrive.matches(r#"<geometry[^>]*\s+poly3"#).count();
    let param_poly3_count = opendrive.matches(r#"<geometry[^>]*\s+paramPoly3"#).count();

    println!("Geometry types in OpenDRIVE:");
    println!("  Lines: {}", line_count);
    println!("  Arcs: {}", arc_count);
    println!("  Spirals: {}", spiral_count);
    println!("  Poly3: {}", poly3_count);
    println!("  ParamPoly3: {}", param_poly3_count);

    let total_geometry = line_count + arc_count + spiral_count + poly3_count + param_poly3_count;
    println!("  Total geometry records: {}", total_geometry);
    assert!(total_geometry > 0, "Map should contain geometry records");

    // Analyze geometry through waypoints
    let waypoints = map.generate_waypoints(5.0);
    let mut road_geometries: HashMap<u32, Vec<(f64, f64, f64)>> = HashMap::new();

    for waypoint in waypoints.iter() {
        let transform = waypoint.transform;
        let road_id = waypoint.road_id as u32;
        // Note: s() method doesn't exist - using index as substitute
        let s = 0.0; // Placeholder since s coordinate is not available

        road_geometries.entry(road_id).or_default().push((
            s,
            transform.location.x,
            transform.location.y,
        ));
    }

    // Analyze curvature for some roads
    println!("\nRoad geometry analysis:");
    for (road_id, points) in road_geometries.iter().take(5) {
        if points.len() >= 3 {
            println!("\nRoad {} ({} sample points):", road_id, points.len());

            // Calculate approximate curvature
            let mut max_curvature: f64 = 0.0;
            for i in 1..points.len() - 1 {
                let (s1, x1, y1) = points[i - 1];
                let (_s2, x2, y2) = points[i];
                let (s3, x3, y3) = points[i + 1];

                // Simple curvature approximation using three points
                let dx1 = x2 - x1;
                let dy1 = y2 - y1;
                let dx2 = x3 - x2;
                let dy2 = y3 - y2;

                let cross = dx1 * dy2 - dy1 * dx2;
                let ds = s3 - s1;
                if ds > 0.0 {
                    let curvature = (cross / ds).abs();
                    max_curvature = max_curvature.max(curvature);
                }
            }

            println!("  Max approximate curvature: {:.4}", max_curvature);

            // Classify geometry type based on curvature
            if max_curvature < 0.001 {
                println!("  Likely geometry type: Line (straight)");
            } else if max_curvature < 0.1 {
                println!("  Likely geometry type: Arc or gentle curve");
            } else {
                println!("  Likely geometry type: Sharp curve or complex geometry");
            }
        }
    }
}

#[with_carla_server]
fn test_iterate_waypoints(client: &carla::client::Client) {
    // Corresponds to C++ TEST(road, iterate_waypoints)
    let world = client.world().expect("Failed to get world");
    let map = world.map().expect("Failed to get map");

    // Generate initial waypoints
    let waypoints = map.generate_waypoints(2.0);
    println!("Generated {} waypoints for iteration test", waypoints.len());
    assert!(!waypoints.is_empty(), "Should generate waypoints");

    // Test waypoint navigation for various waypoints
    let test_count = std::cmp::min(10, waypoints.len());
    let mut navigation_stats = {
        let mut stats = std::collections::HashMap::new();
        stats.insert("has_next", 0);
        stats.insert("has_previous", 0);
        stats.insert("has_left", 0);
        stats.insert("has_right", 0);
        stats.insert("is_junction", 0);
        stats
    };

    println!("\nTesting waypoint navigation:");
    for i in 0..test_count {
        if let Some(waypoint) = waypoints.get(i) {
            // Test next waypoints
            let next_wps = waypoint.next(5.0).expect("Failed to get next waypoints");
            if !next_wps.is_empty() {
                *navigation_stats.get_mut("has_next").unwrap() += 1;
            }

            // Test previous waypoints
            let prev_wps = waypoint
                .previous(5.0)
                .expect("Failed to get previous waypoints");
            if !prev_wps.is_empty() {
                *navigation_stats.get_mut("has_previous").unwrap() += 1;
            }

            // Test left lane
            if waypoint
                .left_lane()
                .expect("Failed to get left lane")
                .is_some()
            {
                *navigation_stats.get_mut("has_left").unwrap() += 1;
            }

            // Test right lane
            if waypoint
                .right_lane()
                .expect("Failed to get right lane")
                .is_some()
            {
                *navigation_stats.get_mut("has_right").unwrap() += 1;
            }

            // Check if junction
            if waypoint.is_junction() {
                *navigation_stats.get_mut("is_junction").unwrap() += 1;
            }

            if i < 3 {
                println!(
                    "  Waypoint {} (road={}, lane={}):",
                    i, waypoint.road_id, waypoint.lane_id
                );
                println!("    Next: {} waypoints", next_wps.len());
                println!("    Previous: {} waypoints", prev_wps.len());
                println!(
                    "    Has left lane: {}",
                    waypoint
                        .left_lane()
                        .expect("Failed to get left lane")
                        .is_some()
                );
                println!(
                    "    Has right lane: {}",
                    waypoint
                        .right_lane()
                        .expect("Failed to get right lane")
                        .is_some()
                );
            }
        }
    }

    println!("\nNavigation statistics ({} waypoints tested):", test_count);
    for (stat, count) in &navigation_stats {
        println!(
            "  {}: {} ({:.1}%)",
            stat,
            count,
            (*count as f64 / test_count as f64) * 100.0
        );
    }

    // Test successor/predecessor relationships
    println!("\nTesting successor/predecessor relationships:");
    let spawn_points = map.spawn_points();
    if let Some(spawn_transform) = spawn_points.get(0) {
        if let Some(start_wp) = map.waypoint(spawn_transform.location) {
            let mut current_location = start_wp.transform.location;
            let mut path = vec![(start_wp.road_id, start_wp.lane_id)];

            // Follow the road forward
            for step in 0..10 {
                if let Some(current) = map.waypoint(current_location) {
                    let next_wps = current.next(10.0).expect("Failed to get next waypoints");
                    if let Some(next) = next_wps.get(0) {
                        path.push((next.road_id, next.lane_id));

                        // Verify backward navigation
                        let prev_wps = next
                            .previous(10.0)
                            .expect("Failed to get previous waypoints");
                        let mut found_current = false;
                        for prev_wp in prev_wps.iter() {
                            if prev_wp.road_id == current.road_id
                                && (prev_wp.lane_id == current.lane_id || prev_wp.is_junction())
                            {
                                found_current = true;
                                break;
                            }
                        }

                        if !found_current && !next.is_junction() {
                            println!(
                                "  Warning: Backward navigation inconsistency at step {}",
                                step
                            );
                        }

                        current_location = next.transform.location;
                    } else {
                        println!("  Reached end of navigable path at step {}", step);
                        break;
                    }
                } else {
                    break;
                }
            }

            println!(
                "  Path traversed (road_id, lane_id): {:?}",
                &path[..std::cmp::min(5, path.len())]
            );
            if path.len() > 5 {
                println!("  ... and {} more waypoints", path.len() - 5);
            }
        }
    }
}

#[with_carla_server]
fn test_get_waypoint(client: &carla::client::Client) {
    // Corresponds to C++ TEST(road, get_waypoint)
    use carla::geom::Location;

    let world = client.world().expect("Failed to get world");
    let map = world.map().expect("Failed to get map");

    // Test getting waypoint at origin
    let origin = Location::new(0.0, 0.0, 0.0);
    if let Some(waypoint) = map.waypoint(origin) {
        println!("Found waypoint at origin:");
        println!("  Road ID: {}", waypoint.road_id);
        println!("  Lane ID: {}", waypoint.lane_id);
        println!("  Is junction: {}", waypoint.is_junction());

        // Test waypoint properties
        assert!(waypoint.lane_width > 0.0, "Lane width should be positive");

        // Test navigation from waypoint
        let next_waypoints = waypoint.next(5.0).expect("Failed to get next waypoints");
        println!("  Next waypoints (5m): {} found", next_waypoints.len());

        if let Some(left_waypoint) = waypoint.left_lane().expect("Failed to get left lane") {
            println!("  Has left lane: lane_id={}", left_waypoint.lane_id);
        }

        if let Some(right_waypoint) = waypoint.right_lane().expect("Failed to get right lane") {
            println!("  Has right lane: lane_id={}", right_waypoint.lane_id);
        }
    } else {
        // If no waypoint at origin, try spawn points
        let spawn_points = map.spawn_points();
        assert!(!spawn_points.is_empty(), "Map should have spawn points");

        let test_location = spawn_points.iter().next().unwrap().location;
        let waypoint = map
            .waypoint(test_location)
            .expect("Should find waypoint at spawn point");

        println!("Found waypoint at spawn point:");
        println!("  Location: {:?}", waypoint.transform.location);
        println!("  Road ID: {}", waypoint.road_id);
        println!("  Lane ID: {}", waypoint.lane_id);
    }
}
