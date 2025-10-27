//! Integration tests for vehicle operations
//!
//! These tests demonstrate common vehicle operations and serve as examples
//! for writing integration tests with the simulator.
//!
//! Run with:
//! ```bash
//! cargo test --test test_vehicle_operations -- --ignored
//! ```

#[path = "test_utils/mod.rs"]
mod test_utils;

use carla::client::ActorBase;
use serial_test::serial;
use test_utils::with_simulator;

#[test]
#[serial]
#[ignore]
fn test_vehicle_spawn() {
    with_simulator(|world| {
        // Get blueprint library
        let blueprint_library = world.blueprint_library();

        // Find vehicle blueprint
        let vehicle_bp = blueprint_library
            .find("vehicle.tesla.model3")
            .expect("Should find Tesla Model 3 blueprint");

        // Get spawn points
        let spawn_points = world.map().recommended_spawn_points();
        assert!(!spawn_points.is_empty(), "Map should have spawn points");

        let spawn_point = spawn_points
            .get(0)
            .expect("Should have at least one spawn point");

        // Spawn vehicle
        let vehicle = world
            .spawn_actor(&vehicle_bp, &spawn_point)
            .expect("Should spawn vehicle");

        // Verify vehicle is alive
        assert!(vehicle.is_alive(), "Vehicle should be alive after spawn");

        // Verify vehicle has expected type
        assert!(
            vehicle.type_id().contains("vehicle.tesla.model3"),
            "Vehicle should have correct type ID"
        );

        println!(
            "Spawned vehicle: {} (ID: {})",
            vehicle.type_id(),
            vehicle.id()
        );
    });
}

#[test]
#[serial]
#[ignore]
fn test_multiple_vehicle_spawn() {
    with_simulator(|world| {
        let blueprint_library = world.blueprint_library();
        let vehicle_bp = blueprint_library
            .find("vehicle.tesla.model3")
            .expect("Should find vehicle blueprint");

        let spawn_points = world.map().recommended_spawn_points();

        // Spawn multiple vehicles
        let mut vehicles = Vec::new();
        let spawn_count = 5.min(spawn_points.len());

        for i in 0..spawn_count {
            let spawn_point = spawn_points.get(i).unwrap();
            match world.spawn_actor(&vehicle_bp, &spawn_point) {
                Ok(vehicle) => {
                    assert!(vehicle.is_alive());
                    vehicles.push(vehicle);
                }
                Err(e) => {
                    eprintln!("Warning: Failed to spawn vehicle {}: {}", i, e);
                }
            }
        }

        assert!(!vehicles.is_empty(), "Should spawn at least one vehicle");
        println!("Spawned {} vehicles", vehicles.len());

        // Verify all vehicles are alive
        for vehicle in &vehicles {
            assert!(vehicle.is_alive(), "All vehicles should be alive");
        }
    });
}

#[test]
#[serial]
#[ignore]
fn test_vehicle_transform() {
    with_simulator(|world| {
        let blueprint_library = world.blueprint_library();
        let vehicle_bp = blueprint_library
            .find("vehicle.tesla.model3")
            .expect("Should find vehicle blueprint");

        let spawn_points = world.map().recommended_spawn_points();
        let spawn_point = spawn_points.get(0).expect("Should have spawn point");

        let vehicle = world
            .spawn_actor(&vehicle_bp, &spawn_point)
            .expect("Should spawn vehicle");

        // Get vehicle transform
        let transform = vehicle.transform();

        println!("Vehicle transform: {:?}", transform);

        // Get location (translation component)
        let location = vehicle.location();

        println!("Vehicle location: {:?}", location);

        // Verify location has finite coordinates
        assert!(
            location.x.is_finite() && location.y.is_finite() && location.z.is_finite(),
            "Location should have finite coordinates"
        );
    });
}

#[test]
#[serial]
#[ignore]
fn test_vehicle_attributes() {
    with_simulator(|world| {
        let blueprint_library = world.blueprint_library();
        let vehicle_bp = blueprint_library
            .find("vehicle.tesla.model3")
            .expect("Should find vehicle blueprint");

        let spawn_points = world.map().recommended_spawn_points();
        let spawn_point = spawn_points.get(0).expect("Should have spawn point");

        let vehicle = world
            .spawn_actor(&vehicle_bp, &spawn_point)
            .expect("Should spawn vehicle");

        // Get vehicle attributes
        let attributes = vehicle.attributes();

        println!("Vehicle attributes:");
        for attr in attributes.iter() {
            println!("  {} = {:?}", attr.id(), attr.value());
        }

        // Common vehicle attributes
        let role_name = attributes
            .iter()
            .find(|a| a.id() == "role_name")
            .map(|a| a.value());
        println!("Role name: {:?}", role_name);
    });
}

// Note: Actor destruction is not yet implemented in the Rust API
// This test is commented out until the destroy() method is available
//
// #[test]
// #[serial]
// #[ignore]
// fn test_vehicle_destroy() {
//     with_simulator(|world| {
//         let blueprint_library = world.blueprint_library();
//         let vehicle_bp = blueprint_library
//             .find("vehicle.tesla.model3")
//             .expect("Should find vehicle blueprint");
//
//         let spawn_points = world.map().recommended_spawn_points();
//         let spawn_point = spawn_points.get(0).expect("Should have spawn point");
//
//         let vehicle = world
//             .spawn_actor(&vehicle_bp, &spawn_point)
//             .expect("Should spawn vehicle");
//
//         let vehicle_id = vehicle.id();
//         assert!(vehicle.is_alive(), "Vehicle should be alive");
//
//         // TODO: Implement when destroy() is available
//         // vehicle.destroy().expect("Should destroy vehicle");
//
//         // Verify vehicle is destroyed
//         // let actors = world.actors();
//         // let found = actors.iter().any(|a| a.id() == vehicle_id);
//         // assert!(!found, "Vehicle should not exist after destruction");
//     });
// }

#[test]
#[serial]
#[ignore]
fn test_blueprint_filtering() {
    with_simulator(|world| {
        let blueprint_library = world.blueprint_library();

        // Filter vehicles
        let vehicles = blueprint_library.filter("vehicle.*");
        assert!(!vehicles.is_empty(), "Should find vehicle blueprints");

        println!("Found {} vehicle blueprints", vehicles.len());

        // Filter specific vehicle
        let tesla_bps = blueprint_library.filter("vehicle.tesla.*");
        assert!(!tesla_bps.is_empty(), "Should find Tesla blueprints");

        println!("Found {} Tesla blueprints", tesla_bps.len());

        // List some blueprints
        for (i, bp) in tesla_bps.iter().enumerate().take(5) {
            println!("  {}: {}", i, bp.id());
        }
    });
}
