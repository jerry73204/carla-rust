//! Walker movement directions example
//!
//! This example demonstrates how to control a walker's movement
//! in different directions (forward, right, backward, left).
//!
//! Prerequisites:
//! - CARLA simulator must be running
//!
//! Run with:
//! ```bash
//! cargo run --example walker_directions
//! ```

use carla::{
    client::{ActorBase, Client, Walker},
    geom::{FfiVector3D, Vector3D},
    rpc::WalkerControl,
};

fn main() {
    println!("Connecting to CARLA simulator...");
    let client = Client::connect("localhost", 2000, None);
    println!("Connected!");

    // Load default map for clean world state
    println!("\nLoading map: Town10HD_Opt...");
    let mut world = client.load_world("Town10HD_Opt");
    println!("Map loaded!");
    let blueprint_library = world.blueprint_library();

    // Spawn a walker
    let walker_bp = blueprint_library
        .filter("walker.pedestrian.*")
        .get(0)
        .expect("Failed to find walker blueprint");

    let spawn_points = world.map().recommended_spawn_points();
    let spawn_point = spawn_points.get(0).expect("No spawn points available");

    let walker_actor = world
        .spawn_actor(&walker_bp, &spawn_point)
        .expect("Failed to spawn walker");

    let walker: Walker = walker_actor
        .try_into()
        .expect("Failed to convert to Walker type");

    println!("Walker spawned: ID {}\n", walker.id());

    // Test different movement directions
    let directions = [
        (
            "Forward (+X)",
            Vector3D {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
        ),
        (
            "Right (+Y)",
            Vector3D {
                x: 0.0,
                y: 1.0,
                z: 0.0,
            },
        ),
        (
            "Backward (-X)",
            Vector3D {
                x: -1.0,
                y: 0.0,
                z: 0.0,
            },
        ),
        (
            "Left (-Y)",
            Vector3D {
                x: 0.0,
                y: -1.0,
                z: 0.0,
            },
        ),
    ];

    println!("Testing different movement directions:\n");

    for (name, direction) in directions.iter() {
        // SAFETY: Vector3D and carla::geom::Vector3D have identical memory layout
        let control = unsafe {
            WalkerControl {
                direction: std::mem::transmute::<FfiVector3D, carla_sys::carla::geom::Vector3D>(
                    direction.into_ffi(),
                ),
                speed: 2.0, // m/s
                jump: false,
            }
        };

        walker.apply_control(&control);

        println!(
            "✓ {} - Direction: ({:.1}, {:.1}, {:.1})",
            name, direction.x, direction.y, direction.z
        );
    }

    // Final control: diagonal movement
    println!("\nBonus - Diagonal movement:");
    let direction = Vector3D {
        x: 0.707, // ~45 degrees
        y: 0.707,
        z: 0.0,
    };
    // SAFETY: Vector3D and carla::geom::Vector3D have identical memory layout
    let diagonal_control = unsafe {
        WalkerControl {
            direction: std::mem::transmute::<FfiVector3D, carla_sys::carla::geom::Vector3D>(
                direction.into_ffi(),
            ),
            speed: 2.0,
            jump: false,
        }
    };

    walker.apply_control(&diagonal_control);
    println!(
        "✓ Diagonal (+X+Y) - Direction: ({:.3}, {:.3}, {:.3})",
        diagonal_control.direction.x, diagonal_control.direction.y, diagonal_control.direction.z
    );

    println!("\nNote: The last control command is active.");
    println!("Walker is now walking diagonally in the simulation.");
}
