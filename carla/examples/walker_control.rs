//! Walker control example
//!
//! This example demonstrates how to spawn a walker and apply
//! movement control to make it walk.
//!
//! Prerequisites:
//! - CARLA simulator must be running
//!
//! Run with:
//! ```bash
//! cargo run --example walker_control
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

    println!("Walker spawned: ID {}", walker_actor.id());

    // Convert to Walker type to access control methods
    let walker: Walker = walker_actor
        .try_into()
        .expect("Failed to convert to Walker type");

    // Create movement control - walk forward (X direction) at 1.5 m/s
    let direction = Vector3D {
        x: 1.0,
        y: 0.0,
        z: 0.0,
    };
    // SAFETY: Vector3D and carla::geom::Vector3D have identical memory layout
    let control = unsafe {
        WalkerControl {
            direction: std::mem::transmute::<FfiVector3D, carla_sys::carla::geom::Vector3D>(
                direction.into_ffi(),
            ),
            speed: 1.5, // m/s
            jump: false,
        }
    };

    println!("\nApplying walker control:");
    println!(
        "  Direction: ({:.1}, {:.1}, {:.1})",
        control.direction.x, control.direction.y, control.direction.z
    );
    println!("  Speed: {} m/s", control.speed);
    println!("  Jump: {}", control.jump);

    // Apply control
    walker.apply_control(&control);

    println!("✓ Control applied successfully!");

    // Retrieve current control state
    let current_control = walker.control();
    println!("\nCurrent walker control:");
    println!(
        "  Direction: ({:.1}, {:.1}, {:.1})",
        current_control.direction.x, current_control.direction.y, current_control.direction.z
    );
    println!("  Speed: {:.2} m/s", current_control.speed);
    println!("  Jump: {}", current_control.jump);

    println!("\nWalker is now walking forward.");
    println!("The walker will continue moving in the simulation.");
}
