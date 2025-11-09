//! Debug Drawing and Color Utilities Tests
//!
//! Tests for debug drawing functionality and color operations (Phase 2).
//!
//! # Test Categories
//! - Unit tests: Color creation, constants, conversions
//! - Integration tests: Debug drawing primitives (points, lines, arrows, boxes, strings)
//!
//! Run with:
//! ```bash
//! cargo run --example test_debug_draw --profile dev-release
//! ```

use carla::{
    client::{ActorBase, Client},
    geom::{BoundingBox, Location, Rotation, Vector3D},
    rpc::Color,
};
use std::time::Duration;

type TestResult = Result<(), Box<dyn std::error::Error>>;

fn main() {
    println!("=== Debug Drawing and Color Tests ===\n");

    // Connect to CARLA
    let client = Client::connect("127.0.0.1", 2000, None);
    let mut world = client.world();
    println!("Connected to CARLA server\n");

    // Setup scenario
    setup_scenario(&mut world);

    // Run tests
    let mut passed = 0;
    let mut failed = 0;

    // Unit tests (color operations)
    run_test(
        "test_color_creation",
        test_color_creation,
        &mut passed,
        &mut failed,
    );
    run_test(
        "test_color_constants",
        test_color_constants,
        &mut passed,
        &mut failed,
    );
    run_test(
        "test_color_to_native",
        || test_color_to_native(&world),
        &mut passed,
        &mut failed,
    );

    // Integration tests (debug drawing)
    run_test(
        "test_debug_draw_point",
        || test_debug_draw_point(&mut world),
        &mut passed,
        &mut failed,
    );
    run_test(
        "test_debug_draw_line",
        || test_debug_draw_line(&mut world),
        &mut passed,
        &mut failed,
    );
    run_test(
        "test_debug_draw_arrow",
        || test_debug_draw_arrow(&mut world),
        &mut passed,
        &mut failed,
    );
    run_test(
        "test_debug_draw_box",
        || test_debug_draw_box(&mut world),
        &mut passed,
        &mut failed,
    );
    run_test(
        "test_debug_draw_string",
        || test_debug_draw_string(&mut world),
        &mut passed,
        &mut failed,
    );
    run_test(
        "test_debug_shapes_lifetime",
        || test_debug_shapes_lifetime(&mut world),
        &mut passed,
        &mut failed,
    );
    run_test(
        "test_debug_multiple_shapes",
        || test_debug_multiple_shapes(&mut world),
        &mut passed,
        &mut failed,
    );
    run_test(
        "test_debug_shape_colors",
        || test_debug_shape_colors(&mut world),
        &mut passed,
        &mut failed,
    );

    // Report results
    println!("\n=== Results ===");
    println!("Passed: {}", passed);
    println!("Failed: {}", failed);
    std::process::exit(if failed > 0 { 1 } else { 0 });
}

fn setup_scenario(world: &mut carla::client::World) {
    println!("Setting up test scenario...");

    // Spawn a vehicle at a spawn point for reference
    let blueprint_library = world.blueprint_library();
    let vehicle_bp = blueprint_library
        .find("vehicle.tesla.model3")
        .expect("Vehicle blueprint not found");

    let spawn_points = world.map().recommended_spawn_points();
    let spawn_point = spawn_points.get(0).expect("No spawn points available");

    let _vehicle = world
        .spawn_actor(&vehicle_bp, spawn_point)
        .expect("Failed to spawn vehicle");

    // Position spectator camera to view debug shapes
    let spectator = world.spectator();
    let camera_location = Location::new(
        spawn_point.location.x + 10.0,
        spawn_point.location.y,
        spawn_point.location.z + 5.0,
    );
    let camera_rotation = Rotation::new(0.0, 180.0, 0.0);
    spectator.set_transform(&carla::geom::Transform {
        location: camera_location,
        rotation: camera_rotation,
    });

    println!("Scenario setup complete\n");
}

fn run_test<F>(name: &str, test_fn: F, passed: &mut i32, failed: &mut i32)
where
    F: FnOnce() -> TestResult,
{
    print!("Testing {}... ", name);
    match test_fn() {
        Ok(_) => {
            println!("✓ PASS");
            *passed += 1;
        }
        Err(e) => {
            println!("✗ FAIL: {}", e);
            *failed += 1;
        }
    }
}

// ===== Unit Tests =====

fn test_color_creation() -> TestResult {
    let color = Color::new(100, 150, 200);
    assert_eq!(color.r, 100);
    assert_eq!(color.g, 150);
    assert_eq!(color.b, 200);
    assert_eq!(color.a, 255);

    let color_rgba = Color::rgba(10, 20, 30, 40);
    assert_eq!(color_rgba.r, 10);
    assert_eq!(color_rgba.g, 20);
    assert_eq!(color_rgba.b, 30);
    assert_eq!(color_rgba.a, 40);

    Ok(())
}

fn test_color_constants() -> TestResult {
    assert_eq!(Color::RED, Color::new(255, 0, 0));
    assert_eq!(Color::GREEN, Color::new(0, 255, 0));
    assert_eq!(Color::BLUE, Color::new(0, 0, 255));
    assert_eq!(Color::WHITE, Color::new(255, 255, 255));
    assert_eq!(Color::BLACK, Color::new(0, 0, 0));
    assert_eq!(Color::YELLOW, Color::new(255, 255, 0));
    assert_eq!(Color::CYAN, Color::new(0, 255, 255));
    assert_eq!(Color::MAGENTA, Color::new(255, 0, 255));
    Ok(())
}

fn test_color_to_native(_world: &carla::client::World) -> TestResult {
    // Test conversion to FFI type
    let color = Color::new(128, 64, 32);
    let ffi_color: carla_sys::carla_rust::sensor::data::FfiColor = color.into();
    assert_eq!(ffi_color.r, 128);
    assert_eq!(ffi_color.g, 64);
    assert_eq!(ffi_color.b, 32);
    assert_eq!(ffi_color.a, 255);
    Ok(())
}

// ===== Integration Tests =====

fn test_debug_draw_point(world: &mut carla::client::World) -> TestResult {
    let debug = world.debug();
    let location = Location::new(0.0, 0.0, 1.0);

    // Draw a point and verify it doesn't panic
    debug.draw_point(location, 0.5, Color::RED, 2.0, false);

    // Give server time to process
    std::thread::sleep(Duration::from_millis(100));
    Ok(())
}

fn test_debug_draw_line(world: &mut carla::client::World) -> TestResult {
    let debug = world.debug();
    let begin = Location::new(0.0, 0.0, 1.0);
    let end = Location::new(10.0, 0.0, 1.0);

    // Draw a line
    debug.draw_line(begin, end, 0.1, Color::GREEN, 2.0, false);

    std::thread::sleep(Duration::from_millis(100));
    Ok(())
}

fn test_debug_draw_arrow(world: &mut carla::client::World) -> TestResult {
    let debug = world.debug();
    let begin = Location::new(0.0, 0.0, 1.0);
    let end = Location::new(0.0, 0.0, 5.0);

    // Draw an arrow pointing up
    debug.draw_arrow(begin, end, 0.1, 0.2, Color::BLUE, 2.0, false);

    std::thread::sleep(Duration::from_millis(100));
    Ok(())
}

fn test_debug_draw_box(world: &mut carla::client::World) -> TestResult {
    let debug = world.debug();
    let bbox = BoundingBox::new(Location::new(0.0, 0.0, 1.0), Vector3D::new(2.0, 1.0, 1.0));
    let rotation = Rotation::new(0.0, 45.0, 0.0);

    // Draw a bounding box
    debug.draw_box(&bbox, rotation, 0.1, Color::YELLOW, 2.0, false);

    std::thread::sleep(Duration::from_millis(100));
    Ok(())
}

fn test_debug_draw_string(world: &mut carla::client::World) -> TestResult {
    let debug = world.debug();
    let location = Location::new(0.0, 0.0, 5.0);

    // Draw text label
    debug.draw_string(location, "Test Label", true, Color::WHITE, 2.0, false);

    std::thread::sleep(Duration::from_millis(100));
    Ok(())
}

fn test_debug_shapes_lifetime(world: &mut carla::client::World) -> TestResult {
    let debug = world.debug();
    let location = Location::new(5.0, 0.0, 1.0);

    // Draw a shape with short lifetime (0.5 seconds)
    debug.draw_point(location, 1.0, Color::MAGENTA, 0.5, false);

    // Wait for it to disappear
    std::thread::sleep(Duration::from_millis(600));

    // If we got here without errors, the shape was drawn and should have disappeared
    Ok(())
}

fn test_debug_multiple_shapes(world: &mut carla::client::World) -> TestResult {
    let debug = world.debug();

    // Draw multiple shapes simultaneously
    for i in 0..5 {
        let x = i as f32 * 2.0;
        debug.draw_point(Location::new(x, 0.0, 1.0), 0.3, Color::RED, 2.0, false);
    }

    std::thread::sleep(Duration::from_millis(100));
    Ok(())
}

fn test_debug_shape_colors(world: &mut carla::client::World) -> TestResult {
    let debug = world.debug();

    // Draw shapes in different colors
    let colors = [
        Color::RED,
        Color::GREEN,
        Color::BLUE,
        Color::YELLOW,
        Color::CYAN,
    ];

    for (i, color) in colors.iter().enumerate() {
        let y = i as f32 * 2.0;
        debug.draw_point(Location::new(0.0, y, 1.0), 0.3, *color, 2.0, false);
    }

    std::thread::sleep(Duration::from_millis(100));
    Ok(())
}
