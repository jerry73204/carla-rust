//! Debug visualization example
//!
//! This example demonstrates the DebugHelper API for drawing geometric
//! primitives and text in the CARLA simulator for debugging and visualization.
//!
//! Prerequisites:
//! - CARLA simulator must be running
//!
//! Run with:
//! ```bash
//! cargo run --example debug_visualization
//! ```

use carla::{
    client::Client,
    geom::{BoundingBox, Location, Rotation, Vector3D},
    rpc::Color,
};
use std::{thread, time::Duration};

fn main() {
    println!("Connecting to CARLA simulator...");
    let client = Client::connect("localhost", 2000, None);
    println!("Connected!");

    // Get the current world
    println!("\nGetting current world...");
    let mut world = client.world();
    println!("World ready!");

    // Get debug helper
    let debug = world.debug();

    // Origin point for all drawings
    let origin = Location::new(0.0, 0.0, 0.5);

    println!("\nDrawing debug visualizations...");

    // 1. Draw colored points in a grid
    println!("1. Drawing colored points...");
    let colors = [
        ("Red", Color::RED),
        ("Green", Color::GREEN),
        ("Blue", Color::BLUE),
        ("Yellow", Color::YELLOW),
        ("Cyan", Color::CYAN),
        ("Magenta", Color::MAGENTA),
        ("White", Color::WHITE),
    ];

    for (i, (name, color)) in colors.iter().enumerate() {
        let x = (i as f32) * 2.0;
        let location = Location::new(origin.x + x, origin.y, origin.z);
        debug.draw_point(location, 0.5, *color, 60.0, false);

        // Label each point
        let label_pos = Location::new(origin.x + x, origin.y, origin.z + 1.0);
        debug.draw_string(label_pos, name, true, Color::WHITE, 60.0, false);
    }

    // 2. Draw lines forming a box outline
    println!("2. Drawing lines...");
    let line_start = Location::new(origin.x, origin.y + 5.0, origin.z);
    let line_points = [
        Location::new(0.0, 0.0, 0.0),
        Location::new(5.0, 0.0, 0.0),
        Location::new(5.0, 5.0, 0.0),
        Location::new(0.0, 5.0, 0.0),
        Location::new(0.0, 0.0, 0.0),
        Location::new(0.0, 0.0, 3.0),
        Location::new(5.0, 0.0, 3.0),
        Location::new(5.0, 0.0, 0.0),
        Location::new(5.0, 0.0, 3.0),
        Location::new(5.0, 5.0, 3.0),
        Location::new(5.0, 5.0, 0.0),
        Location::new(5.0, 5.0, 3.0),
        Location::new(0.0, 5.0, 3.0),
        Location::new(0.0, 5.0, 0.0),
        Location::new(0.0, 5.0, 3.0),
        Location::new(0.0, 0.0, 3.0),
    ];

    for i in 0..line_points.len() - 1 {
        let start = Location::new(
            line_start.x + line_points[i].x,
            line_start.y + line_points[i].y,
            line_start.z + line_points[i].z,
        );
        let end = Location::new(
            line_start.x + line_points[i + 1].x,
            line_start.y + line_points[i + 1].y,
            line_start.z + line_points[i + 1].z,
        );
        debug.draw_line(start, end, 0.1, Color::GREEN, 60.0, false);
    }

    debug.draw_string(
        Location::new(line_start.x + 2.5, line_start.y + 2.5, line_start.z + 4.0),
        "Line Box",
        true,
        Color::GREEN,
        60.0,
        false,
    );

    // 3. Draw arrows showing coordinate axes
    println!("3. Drawing arrows...");
    let arrow_x = origin.x;
    let arrow_y = origin.y + 12.0;
    let arrow_z = origin.z;
    let arrow_length = 5.0;

    // X-axis (red)
    debug.draw_arrow(
        Location::new(arrow_x, arrow_y, arrow_z),
        Location::new(arrow_x + arrow_length, arrow_y, arrow_z),
        0.1,
        0.5,
        Color::RED,
        60.0,
        false,
    );
    debug.draw_string(
        Location::new(arrow_x + arrow_length, arrow_y, arrow_z + 0.5),
        "X",
        true,
        Color::RED,
        60.0,
        false,
    );

    // Y-axis (green)
    debug.draw_arrow(
        Location::new(arrow_x, arrow_y, arrow_z),
        Location::new(arrow_x, arrow_y + arrow_length, arrow_z),
        0.1,
        0.5,
        Color::GREEN,
        60.0,
        false,
    );
    debug.draw_string(
        Location::new(arrow_x, arrow_y + arrow_length, arrow_z + 0.5),
        "Y",
        true,
        Color::GREEN,
        60.0,
        false,
    );

    // Z-axis (blue)
    debug.draw_arrow(
        Location::new(arrow_x, arrow_y, arrow_z),
        Location::new(arrow_x, arrow_y, arrow_z + arrow_length),
        0.1,
        0.5,
        Color::BLUE,
        60.0,
        false,
    );
    debug.draw_string(
        Location::new(arrow_x, arrow_y, arrow_z + arrow_length),
        "Z",
        true,
        Color::BLUE,
        60.0,
        false,
    );

    // 4. Draw bounding boxes with different rotations
    println!("4. Drawing bounding boxes...");
    let box_base = Location::new(origin.x + 20.0, origin.y, origin.z);

    for i in 0..4 {
        let yaw = i as f32 * 30.0;
        let y_offset = (i as f32) * 5.0;
        let bbox = BoundingBox::new(
            Location::new(box_base.x, box_base.y + y_offset, box_base.z + 1.5),
            Vector3D {
                x: 2.0,
                y: 1.0,
                z: 1.5,
            },
        );
        let rotation = Rotation {
            pitch: 0.0,
            yaw,
            roll: 0.0,
        };

        // Use different colors for each box
        let color = match i {
            0 => Color::RED,
            1 => Color::GREEN,
            2 => Color::BLUE,
            _ => Color::YELLOW,
        };

        debug.draw_box(&bbox, rotation, 0.1, color, 60.0, false);

        // Label each box with its rotation
        debug.draw_string(
            Location::new(box_base.x, box_base.y + y_offset, box_base.z + 3.5),
            &format!("Yaw: {}°", yaw),
            true,
            Color::WHITE,
            60.0,
            false,
        );
    }

    // 5. Draw a text label with title
    println!("5. Drawing text labels...");
    debug.draw_string(
        Location::new(origin.x + 10.0, origin.y + 20.0, origin.z + 5.0),
        "CARLA Debug Visualization",
        true,
        Color::CYAN,
        60.0,
        false,
    );

    debug.draw_string(
        Location::new(origin.x + 10.0, origin.y + 20.0, origin.z + 3.5),
        "Points, Lines, Arrows, Boxes, and Text",
        false,
        Color::WHITE,
        60.0,
        false,
    );

    println!("\n✓ Debug visualizations drawn!");
    println!("  All visualizations will persist for 60 seconds.");
    println!("  Switch to spectator view in CARLA to see them.");
    println!("  Location: Around origin (0, 0, 0)");

    // Wait a few seconds to ensure visualizations are visible
    println!("\nWaiting 5 seconds for visualizations to be visible...");
    thread::sleep(Duration::from_secs(5));

    println!("✓ Example complete!");
    println!("  Note: Visualizations will remain visible for 60 seconds total.");
}
