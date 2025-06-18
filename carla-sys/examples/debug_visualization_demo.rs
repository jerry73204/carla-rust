//! Debug visualization demonstration for carla-sys.
//!
//! This example shows how to use the debug drawing API to visualize:
//! - Actor positions and bounding boxes
//! - Waypoints and paths
//! - Vehicle trajectories
//! - Transforms and coordinate systems
//! - Text labels and annotations

use anyhow::Result;
use carla_sys::{
    debug_colors, ActorListExt, ActorWrapper, ClientWrapper, DebugDrawExt, SimpleBoundingBox,
    SimpleLocation, SimpleRotation, SimpleTransform, SimpleVector3D, WorldWrapper,
};
use std::time::Duration;

fn main() -> Result<()> {
    println!("🎨 CARLA Debug Visualization Demo");
    println!("=================================");

    // Connect to CARLA server
    println!("\n🔌 Connecting to CARLA server...");
    let mut client = ClientWrapper::new("localhost", 2000)?;
    println!(
        "✅ Connected! Server version: {}",
        client.get_server_version()
    );

    client.set_timeout(Duration::from_secs(10));

    let world = client.get_world();
    println!("🌍 World ID: {}", world.get_id());

    // Clear any existing debug shapes by drawing an invisible point with 0 lifetime
    world
        .debug_draw()
        .life_time(0.0)
        .point(&SimpleLocation::new(0.0, 0.0, 0.0), 0.0);

    println!("\n🎨 Starting debug visualization demonstrations...");

    // === DEMO 1: Basic Shapes ===
    demo_basic_shapes(&world)?;
    std::thread::sleep(Duration::from_secs(2));

    // === DEMO 2: Actor Visualization ===
    demo_actor_visualization(&world)?;
    std::thread::sleep(Duration::from_secs(2));

    // === DEMO 3: Path Visualization ===
    demo_path_visualization(&world)?;
    std::thread::sleep(Duration::from_secs(2));

    // === DEMO 4: Coordinate Systems ===
    demo_coordinate_systems(&world)?;
    std::thread::sleep(Duration::from_secs(2));

    // === DEMO 5: Dynamic Visualizations ===
    demo_dynamic_visualizations(&world)?;

    println!("\n✅ Debug visualization demonstration completed!");
    println!("💡 All debug shapes will persist until cleared or the simulation is reset.");

    Ok(())
}

/// Demonstrate basic debug shapes
fn demo_basic_shapes(world: &WorldWrapper) -> Result<()> {
    println!("\n📊 Demo 1: Basic Debug Shapes");
    println!("------------------------------");

    // Draw points with different colors and sizes
    println!("🔵 Drawing colored points...");
    let point_base = SimpleLocation::new(10.0, 0.0, 0.5);

    world
        .debug_draw()
        .color(debug_colors::RED)
        .life_time(30.0)
        .point(&point_base, 0.2);

    world
        .debug_draw()
        .color(debug_colors::GREEN)
        .life_time(30.0)
        .point(&SimpleLocation::new(10.0, 2.0, 0.5), 0.3);

    world
        .debug_draw()
        .color(debug_colors::BLUE)
        .life_time(30.0)
        .point(&SimpleLocation::new(10.0, 4.0, 0.5), 0.4);

    // Draw lines with different thicknesses
    println!("📏 Drawing lines with varying thickness...");
    let line_start = SimpleLocation::new(15.0, 0.0, 0.5);

    for i in 0..5 {
        let thickness = 0.05 + (i as f32 * 0.05);
        let end = SimpleLocation::new(15.0, (i as f64 + 1.0) * 2.0, 0.5);

        world
            .debug_draw()
            .color(debug_colors::CYAN)
            .thickness(thickness)
            .life_time(30.0)
            .line(&line_start, &end);
    }

    // Draw arrows pointing in different directions
    println!("➡️  Drawing directional arrows...");
    let arrow_center = SimpleLocation::new(20.0, 0.0, 0.5);
    let arrow_length = 3.0;

    for i in 0..8 {
        let angle = (i as f64) * std::f64::consts::PI / 4.0;
        let end = SimpleLocation::new(
            arrow_center.x + arrow_length * angle.cos(),
            arrow_center.y + arrow_length * angle.sin(),
            arrow_center.z,
        );

        world
            .debug_draw()
            .color(debug_colors::YELLOW)
            .thickness(0.1)
            .life_time(30.0)
            .arrow(&arrow_center, &end, 0.3);
    }

    // Draw boxes of different sizes
    println!("📦 Drawing bounding boxes...");
    for i in 0..3 {
        let size = 0.5 + (i as f64 * 0.5);
        let location = SimpleLocation::new(25.0, i as f64 * 3.0, size);
        let bbox = SimpleBoundingBox {
            location,
            extent: SimpleVector3D::new(size, size, size),
        };

        let color = match i {
            0 => debug_colors::ORANGE,
            1 => debug_colors::PURPLE,
            _ => debug_colors::PINK,
        };

        world
            .debug_draw()
            .color(color)
            .thickness(0.05)
            .life_time(30.0)
            .box_shape(
                &bbox,
                &SimpleRotation::new(0.0, (i as f32 * 15.0) as f64, 0.0),
            );
    }

    // Draw text labels
    println!("📝 Drawing text labels...");
    world
        .debug_draw()
        .color(debug_colors::WHITE)
        .life_time(30.0)
        .text(&SimpleLocation::new(10.0, -3.0, 2.0), "Points", true);

    world
        .debug_draw()
        .color(debug_colors::WHITE)
        .life_time(30.0)
        .text(&SimpleLocation::new(15.0, -3.0, 2.0), "Lines", true);

    world
        .debug_draw()
        .color(debug_colors::WHITE)
        .life_time(30.0)
        .text(&SimpleLocation::new(20.0, -3.0, 2.0), "Arrows", true);

    world
        .debug_draw()
        .color(debug_colors::WHITE)
        .life_time(30.0)
        .text(&SimpleLocation::new(25.0, -3.0, 2.0), "Boxes", true);

    Ok(())
}

/// Demonstrate actor visualization
fn demo_actor_visualization(world: &WorldWrapper) -> Result<()> {
    println!("\n🚗 Demo 2: Actor Visualization");
    println!("-------------------------------");

    let actors = world.get_actors();
    if actors.is_empty() {
        println!("⚠️  No actors found in the world.");
        return Ok(());
    }

    println!("🎭 Visualizing {} actors...", actors.len());

    // Get first few actors to visualize
    let actor_ids = actors.get_ids();
    let sample_size = std::cmp::min(10, actor_ids.len());

    for &actor_id in &actor_ids[..sample_size] {
        if let Some(actor) = world.get_actor(actor_id) {
            visualize_actor(world, &actor)?;
        }
    }

    Ok(())
}

/// Visualize a single actor
fn visualize_actor(world: &WorldWrapper, actor: &ActorWrapper) -> Result<()> {
    let location = actor.get_location();
    let transform = actor.get_transform();
    let type_id = actor.get_type_id();

    // Determine color based on actor type
    let color = if type_id.contains("vehicle") {
        debug_colors::CYAN
    } else if type_id.contains("walker") {
        debug_colors::YELLOW
    } else if type_id.contains("traffic") {
        debug_colors::ORANGE
    } else {
        debug_colors::GRAY
    };

    // Draw actor position
    world
        .debug_draw()
        .color(color)
        .life_time(20.0)
        .point(&location, 0.3);

    // Draw actor transform axes
    world
        .debug_draw()
        .life_time(20.0)
        .transform(&transform, 1.0);

    // Draw actor ID label
    let label_location = SimpleLocation::new(location.x, location.y, location.z + 2.0);
    let label = format!("ID: {}", actor.get_id());

    world
        .debug_draw()
        .color(debug_colors::WHITE)
        .life_time(20.0)
        .text(&label_location, &label, true);

    Ok(())
}

/// Demonstrate path visualization
fn demo_path_visualization(world: &WorldWrapper) -> Result<()> {
    println!("\n🛤️  Demo 3: Path Visualization");
    println!("-------------------------------");

    // Create a sample curved path
    println!("📍 Drawing a curved path...");
    let mut path_points = Vec::new();
    let num_points = 20;
    let radius = 10.0;

    for i in 0..=num_points {
        let t = i as f64 / num_points as f64;
        let angle = t * std::f64::consts::PI;

        let x = 40.0 + radius * angle.cos();
        let y = t * 20.0;
        let z = 0.5 + 2.0 * (angle * 2.0).sin();

        path_points.push(SimpleLocation::new(x, y, z));
    }

    // Draw the path
    world
        .debug_draw()
        .color(debug_colors::MAGENTA)
        .thickness(0.2)
        .life_time(30.0)
        .path(&path_points);

    // Add waypoint markers along the path
    println!("🎯 Adding waypoint markers...");
    for (i, point) in path_points.iter().enumerate() {
        if i % 4 == 0 {
            // Draw waypoint marker
            world
                .debug_draw()
                .color(debug_colors::YELLOW)
                .life_time(30.0)
                .point(point, 0.4);

            // Draw waypoint number
            let text_loc = SimpleLocation::new(point.x, point.y, point.z + 1.0);
            let text = format!("WP{}", i / 4);

            world
                .debug_draw()
                .color(debug_colors::WHITE)
                .life_time(30.0)
                .text(&text_loc, &text, false);
        }
    }

    // Draw a spiral path
    println!("🌀 Drawing a spiral path...");
    let mut spiral_points = Vec::new();
    let spiral_turns = 3.0;
    let spiral_height = 10.0;

    for i in 0..=50 {
        let t = i as f64 / 50.0;
        let angle = t * 2.0 * std::f64::consts::PI * spiral_turns;
        let radius = 5.0 * (1.0 - t); // Decreasing radius

        let x = 60.0 + radius * angle.cos();
        let y = radius * angle.sin();
        let z = 0.5 + t * spiral_height;

        spiral_points.push(SimpleLocation::new(x, y, z));
    }

    world
        .debug_draw()
        .color(debug_colors::GREEN)
        .thickness(0.15)
        .life_time(30.0)
        .path(&spiral_points);

    Ok(())
}

/// Demonstrate coordinate system visualization
fn demo_coordinate_systems(world: &WorldWrapper) -> Result<()> {
    println!("\n📐 Demo 4: Coordinate Systems");
    println!("-----------------------------");

    // World coordinate system at origin
    println!("🌍 Drawing world coordinate system...");
    let world_origin = SimpleTransform::new(
        SimpleLocation::new(0.0, 0.0, 0.5),
        SimpleRotation::new(0.0, 0.0, 0.0),
    );

    world
        .debug_draw()
        .thickness(0.3)
        .life_time(30.0)
        .transform(&world_origin, 5.0);

    world
        .debug_draw()
        .color(debug_colors::WHITE)
        .life_time(30.0)
        .text(&SimpleLocation::new(0.0, 0.0, 6.0), "World Origin", true);

    // Rotated coordinate systems
    println!("🔄 Drawing rotated coordinate systems...");
    for i in 0..4 {
        let angle = i as f32 * 45.0;
        let x = 10.0 + (i as f64 * 5.0);

        let transform = SimpleTransform::new(
            SimpleLocation::new(x, 10.0, 0.5),
            SimpleRotation::new(0.0, angle as f64, 0.0),
        );

        world
            .debug_draw()
            .thickness(0.1)
            .life_time(30.0)
            .transform(&transform, 2.0);

        let label = format!("Yaw: {}°", angle);
        world
            .debug_draw()
            .color(debug_colors::WHITE)
            .life_time(30.0)
            .text(&SimpleLocation::new(x, 10.0, 3.0), &label, false);
    }

    Ok(())
}

/// Demonstrate dynamic visualizations
fn demo_dynamic_visualizations(world: &WorldWrapper) -> Result<()> {
    println!("\n🎬 Demo 5: Dynamic Visualizations");
    println!("---------------------------------");

    // Animated sphere
    println!("⚪ Creating animated sphere visualization...");
    let sphere_center = SimpleLocation::new(30.0, 20.0, 3.0);

    for frame in 0..30 {
        let scale = 1.0 + 0.5 * (frame as f32 * 0.2).sin();
        let color_value = ((frame as f32 / 30.0) * 255.0) as u8;

        world
            .debug_draw()
            .rgb(255 - color_value, color_value, 128)
            .thickness(0.05)
            .life_time(0.2) // Short lifetime for animation
            .sphere(&sphere_center, 2.0 * scale, 16);

        std::thread::sleep(Duration::from_millis(100));
    }

    // Velocity vectors for moving actors
    println!("🏃 Visualizing actor velocities...");
    let actors = world.get_actors();
    let actor_ids = actors.get_ids();

    for _ in 0..20 {
        for &actor_id in &actor_ids[..std::cmp::min(5, actor_ids.len())] {
            if let Some(actor) = world.get_actor(actor_id) {
                let location = actor.get_location();

                // Simulate velocity (in real usage, get actual velocity from vehicle)
                let velocity = SimpleVector3D::new(
                    (actor_id as f64 * 0.1).sin() * 5.0,
                    (actor_id as f64 * 0.1).cos() * 5.0,
                    0.0,
                );

                world
                    .debug_draw()
                    .color(debug_colors::CYAN)
                    .thickness(0.1)
                    .life_time(0.1)
                    .velocity(&location, &velocity, 0.5);
            }
        }

        std::thread::sleep(Duration::from_millis(100));
    }

    // Pulsing debug text
    println!("📝 Creating pulsing text effect...");
    let text_location = SimpleLocation::new(40.0, 30.0, 5.0);

    for i in 0..20 {
        let intensity = ((i as f32 * 0.3).sin() + 1.0) / 2.0;
        let color_value = (intensity * 255.0) as u8;

        world
            .debug_draw()
            .rgb(color_value, color_value, 255)
            .life_time(0.15)
            .text(&text_location, "CARLA Debug Viz", true);

        std::thread::sleep(Duration::from_millis(100));
    }

    Ok(())
}
