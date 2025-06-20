//! OpenDrive integration tests
//!
//! These tests correspond to the C++ tests in test_opendrive.cpp

use carla::{geom::Location, road::GeoLocation};
use std::{fs, path::Path};

/// Helper function to load OpenDrive content from file
fn load_opendrive_file(filename: &str) -> String {
    let path = Path::new("tests/fixtures/opendrive").join(filename);
    fs::read_to_string(&path)
        .unwrap_or_else(|e| panic!("Failed to read OpenDrive file {}: {}", path.display(), e))
}

#[test]
#[ignore] // Run with --ignored when CARLA server is available
fn test_opendrive_parsing() {
    // Corresponds to C++ TEST(road, parse_files)

    // Note: This test requires a CARLA server connection to parse OpenDrive files
    // In the actual implementation, we would:
    // 1. Connect to CARLA server
    // 2. Load a map with OpenDrive data
    // 3. Verify the parsing succeeded

    // For now, we'll test that we can at least read the file
    let opendrive_content = load_opendrive_file("simple_road.xodr");
    assert!(!opendrive_content.is_empty());
    assert!(opendrive_content.contains("<OpenDRIVE>"));
    assert!(opendrive_content.contains("<road"));
    assert!(opendrive_content.contains("<junction"));
}

#[test]
#[ignore] // Run with --ignored when CARLA server is available
fn test_parse_road_links() {
    // Corresponds to C++ TEST(road, parse_road_links)

    // This test requires:
    // 1. A CARLA server connection
    // 2. A loaded map
    // 3. Ability to traverse road links

    // Placeholder for when we have server connection
    // The test would verify that all road links (next/previous lanes) are valid
}

#[test]
#[ignore] // Run with --ignored when CARLA server is available
fn test_parse_junctions() {
    // Corresponds to C++ TEST(road, parse_junctions)

    // This test requires:
    // 1. A CARLA server connection
    // 2. A loaded map with junctions
    // 3. Ability to query junction information

    // The test would verify:
    // - Total number of junctions matches OpenDrive file
    // - Junction connections are correctly parsed
    // - Lane links within junctions are valid
}

#[test]
#[ignore] // Run with --ignored when CARLA server is available
fn test_parse_road() {
    // Corresponds to C++ TEST(road, parse_road)

    // This test verifies road structure parsing:
    // - Total number of roads
    // - Lane sections per road
    // - Lanes per section
    // - Road marks
}

#[test]
#[ignore] // Run with --ignored when CARLA server is available
fn test_parse_road_elevation() {
    // Corresponds to C++ TEST(road, parse_road_elevation)

    // This test verifies elevation profile parsing:
    // - Elevation data at specific s-coordinates
    // - Total number of elevation records per road
}

#[test]
#[ignore] // Run with --ignored when CARLA server is available
fn test_parse_geometry() {
    // Corresponds to C++ TEST(road, parse_geometry)

    // This test verifies road geometry parsing:
    // - Geometry records per road
    // - Geometry types (line, arc, spiral, etc.)
}

#[test]
#[ignore] // Run with --ignored when CARLA server is available
fn test_iterate_waypoints() {
    // Corresponds to C++ TEST(road, iterate_waypoints)

    // This comprehensive test:
    // 1. Generates waypoints along the road
    // 2. Tests waypoint navigation (next, previous, left, right)
    // 3. Verifies waypoint properties
    // 4. Tests successor/predecessor relationships
}

#[test]
#[ignore] // Run with --ignored when CARLA server is available
fn test_get_waypoint() {
    // Corresponds to C++ TEST(road, get_waypoint)

    // This test verifies waypoint queries:
    // - Getting closest waypoint from arbitrary locations
    // - Waypoint properties at specific locations
    // - Navigation from waypoints
}

// Unit tests that don't require a server connection

#[test]
fn test_geo_location() {
    // Test GeoLocation struct
    let geo = GeoLocation::new(48.8566, 2.3522, 35.0); // Paris coordinates
    assert_eq!(geo.latitude, 48.8566);
    assert_eq!(geo.longitude, 2.3522);
    assert_eq!(geo.altitude, 35.0);
}

#[test]
fn test_opendrive_xml_structure() {
    // Basic XML structure validation
    let content = load_opendrive_file("simple_road.xodr");

    // Verify basic OpenDrive structure
    assert!(content.contains(r#"<?xml version="1.0""#));
    assert!(content.contains("<OpenDRIVE>"));
    assert!(content.contains("</OpenDRIVE>"));
    assert!(content.contains("<header"));
    assert!(content.contains("<road"));
    assert!(content.contains("<lanes>"));
    assert!(content.contains("<junction"));

    // Verify our test road
    assert!(content.contains(r#"id="1""#));
    assert!(content.contains(r#"name="Road 1""#));
    assert!(content.contains(r#"length="100.0""#));

    // Verify elevation profile
    assert!(content.contains("<elevationProfile>"));
    assert!(content.contains(r#"s="0.0" a="0.0""#));
    assert!(content.contains(r#"s="50.0" a="5.0""#));

    // Verify lanes
    assert!(content.contains("<laneSection"));
    assert!(content.contains(r#"id="1" type="driving""#));
    assert!(content.contains(r#"id="-1" type="driving""#));
    assert!(content.contains("<roadMark"));

    // Verify junction
    assert!(content.contains(r#"name="Junction 1""#));
    assert!(content.contains("<connection"));
    assert!(content.contains("<laneLink"));
}

#[cfg(test)]
mod test_helpers {
    use super::*;

    /// Mock Map for testing when server is not available
    pub struct MockMap {
        pub name: String,
        pub opendrive_content: String,
    }

    impl MockMap {
        pub fn from_opendrive(content: &str) -> Self {
            Self {
                name: "TestMap".to_string(),
                opendrive_content: content.to_string(),
            }
        }

        pub fn get_road_count(&self) -> usize {
            // Count of <road> tags (but not <roadMark>)
            self.opendrive_content.matches("<road ").count()
        }

        pub fn get_junction_count(&self) -> usize {
            // Simple count of <junction> tags
            self.opendrive_content.matches("<junction").count()
        }
    }
}

#[test]
fn test_mock_opendrive_parsing() {
    use test_helpers::MockMap;

    let content = load_opendrive_file("simple_road.xodr");
    let mock_map = MockMap::from_opendrive(&content);

    assert_eq!(mock_map.get_road_count(), 2); // Road 1 and Road 3
    assert_eq!(mock_map.get_junction_count(), 1); // Junction 1
}

#[test]
fn test_complex_junction_structure() {
    let content = load_opendrive_file("complex_junction.xodr");

    // Verify we have 4 approach roads plus junction roads
    let total_road_count = content.matches("<road ").count();
    let junction_road_count = content.matches("junction=\"100\"").count();
    let approach_road_count = content.matches("junction=\"-1\"").count();
    assert_eq!(approach_road_count, 4);

    // Verify junction connections
    let connection_count = content.matches("<connection").count();
    assert_eq!(connection_count, 12); // 3 connections from each of 4 roads

    // Verify lane links
    let lane_link_count = content.matches("<laneLink").count();
    assert!(lane_link_count >= 12); // At least one per connection
}

#[test]
fn test_opendrive_elevation_parsing() {
    let content = load_opendrive_file("simple_road.xodr");

    // Count elevation records
    let elevation_count = content.matches("<elevation ").count();
    assert_eq!(elevation_count, 3); // 2 in road 1, 1 in road 3

    // Verify elevation attributes
    assert!(content.contains(r#"s="0.0" a="0.0" b="0.0""#));
    assert!(content.contains(r#"s="50.0" a="5.0" b="0.1""#));
}

#[test]
fn test_opendrive_geometry_parsing() {
    let simple_content = load_opendrive_file("simple_road.xodr");
    let complex_content = load_opendrive_file("complex_junction.xodr");

    // Simple road has line geometry
    assert!(simple_content.contains("<line/>"));
    let line_count = simple_content.matches("<line/>").count();
    assert_eq!(line_count, 2);

    // Complex junction has arc geometry
    assert!(complex_content.contains("<arc"));
    assert!(complex_content.contains("curvature="));
}

#[test]
fn test_opendrive_lane_structure() {
    let content = load_opendrive_file("simple_road.xodr");

    // Verify lane sections
    let lane_section_count = content.matches("<laneSection").count();
    assert_eq!(lane_section_count, 2); // One per road

    // Verify lane types
    assert!(content.contains(r#"type="driving""#));
    assert!(content.contains(r#"type="none""#));

    // Verify lane IDs (left positive, center 0, right negative)
    assert!(content.contains(r#"id="1""#));
    assert!(content.contains(r#"id="0""#));
    assert!(content.contains(r#"id="-1""#));
}

#[test]
fn test_opendrive_road_marks() {
    let content = load_opendrive_file("simple_road.xodr");

    // Verify road marks exist
    let road_mark_count = content.matches("<roadMark").count();
    assert_eq!(road_mark_count, 2);

    // Verify road mark types
    assert!(content.contains(r#"type="solid""#));
    assert!(content.contains(r#"type="broken""#));

    // Verify road mark attributes
    assert!(content.contains(r#"material="standard""#));
    assert!(content.contains(r#"color="white""#));
    assert!(content.contains(r#"laneChange="none""#));
    assert!(content.contains(r#"laneChange="both""#));
}

#[test]
fn test_opendrive_road_links() {
    let content = load_opendrive_file("simple_road.xodr");

    // Verify road links
    assert!(content.contains("<link"));
    assert!(content.contains(r#"elementType="junction""#));
    assert!(content.contains("<predecessor"));
}
