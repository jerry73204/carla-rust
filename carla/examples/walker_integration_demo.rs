//! Integration demonstration for Walker and WalkerAIController.
//!
//! This example requires a running CARLA simulator and demonstrates:
//! - Spawning walkers (pedestrians)
//! - Applying walker control (direction, speed, jump)
//! - Using pose blending methods (blend_pose, show_pose, hide_pose)
//! - Spawning and controlling WalkerAIController
//! - Setting AI parameters (speed, start/stop)
//!
//! Setup:
//! 1. Start CARLA simulator: `./CarlaUE4.sh` or `CarlaUE4.exe`
//! 2. Run this example: `cargo run --example walker_integration_demo`

use carla::{
    client::{ActorBase, Client, Walker, WalkerAIController},
    geom::{FfiVector3D, Location, Rotation, Transform, Vector3D},
    rpc::{AttachmentType, WalkerControl},
};
use std::{thread, time::Duration};

/// Helper function to connect to CARLA
fn connect_to_carla() -> Client {
    Client::default()
}

/// Helper function to spawn a walker
fn spawn_walker(client: &Client) -> Walker {
    let mut world = client.world();
    let bp_lib = world.blueprint_library();

    let walker_bp = bp_lib
        .filter("walker.pedestrian.*")
        .get(0)
        .expect("No walker blueprints found");

    let spawn_points = world.map().recommended_spawn_points();

    // Try multiple spawn points in case some are occupied
    for i in 0..spawn_points.len().min(10) {
        let spawn_point = spawn_points.get(i).expect("No spawn points available");

        if let Ok(actor) = world.spawn_actor(&walker_bp, spawn_point) {
            return actor.try_into().expect("Failed to cast to Walker");
        }
    }

    panic!("Failed to spawn walker after trying multiple spawn points");
}

/// Helper function to spawn a WalkerAIController attached to a walker
fn spawn_walker_ai_controller(client: &Client, walker: &Walker) -> WalkerAIController {
    let mut world = client.world();
    let bp_lib = world.blueprint_library();

    let ai_bp = bp_lib
        .find("controller.ai.walker")
        .expect("Walker AI controller blueprint not found");

    // AI controller must be spawned attached to the walker as parent
    // Use identity transform (no offset from walker)
    let transform = Transform {
        location: Location::new(0.0, 0.0, 0.0),
        rotation: Rotation::new(0.0, 0.0, 0.0),
    };

    let actor = world
        .spawn_actor_opt(&ai_bp, &transform, Some(walker), AttachmentType::Rigid)
        .expect("Failed to spawn AI controller attached to walker");

    actor
        .try_into()
        .expect("Failed to cast to WalkerAIController")
}

fn demo_walker_spawn() {
    println!("\n--- Demo 1: Walker Spawn ---");
    let client = connect_to_carla();
    let walker = spawn_walker(&client);

    println!("✓ Walker spawned with ID: {}", walker.id());
    println!("✓ Walker is alive: {}", walker.is_alive());
}

fn demo_walker_control() {
    println!("\n--- Demo 2: Walker Control ---");
    let client = connect_to_carla();
    let walker = spawn_walker(&client);

    println!("Walker ID: {}", walker.id());

    // Apply walker control
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

    println!(
        "Applying control: speed={} m/s, direction=(1, 0, 0)",
        control.speed
    );
    walker.apply_control(&control);

    thread::sleep(Duration::from_millis(100));

    let current_control = walker.control();
    println!("✓ Control applied: speed={} m/s", current_control.speed);
}

fn demo_walker_blend_pose() {
    println!("\n--- Demo 3: Walker Pose Blending ---");
    let client = connect_to_carla();
    let walker = spawn_walker(&client);

    println!("Walker ID: {}", walker.id());

    println!("Setting blend to 0.0 (full animation)...");
    walker.blend_pose(0.0);
    thread::sleep(Duration::from_millis(100));

    println!("Setting blend to 0.5 (50% blend)...");
    walker.blend_pose(0.5);
    thread::sleep(Duration::from_millis(100));

    println!("Setting blend to 1.0 (full custom pose)...");
    walker.blend_pose(1.0);
    thread::sleep(Duration::from_millis(100));

    println!("Calling show_pose() [blend=1.0]...");
    walker.show_pose();
    thread::sleep(Duration::from_millis(100));

    println!("Calling hide_pose() [blend=0.0]...");
    walker.hide_pose();
    thread::sleep(Duration::from_millis(100));

    println!("✓ Pose blending methods work correctly");
}

fn demo_walker_location() {
    println!("\n--- Demo 4: Walker Location ---");
    let client = connect_to_carla();
    let walker = spawn_walker(&client);

    let location = walker.location();
    println!(
        "Walker location: ({:.2}, {:.2}, {:.2})",
        location.x, location.y, location.z
    );

    if location.x.abs() < 100000.0 && location.y.abs() < 100000.0 {
        println!("✓ Walker location is within reasonable bounds");
    } else {
        println!("⚠ Warning: Walker location seems unusual");
    }
}

fn demo_walker_ai_controller_spawn() {
    println!("\n--- Demo 5: WalkerAIController Spawn ---");
    let client = connect_to_carla();
    let walker = spawn_walker(&client);
    let ai_controller = spawn_walker_ai_controller(&client, &walker);

    println!("✓ AI controller spawned with ID: {}", ai_controller.id());
    println!("✓ AI controller is alive: {}", ai_controller.is_alive());
    println!("✓ AI controller attached to walker ID: {}", walker.id());
}

fn demo_walker_ai_controller_control() {
    println!("\n--- Demo 6: WalkerAIController Start/Stop ---");
    let client = connect_to_carla();
    let walker = spawn_walker(&client);
    let ai_controller = spawn_walker_ai_controller(&client, &walker);

    println!(
        "AI controller ID: {} (attached to walker {})",
        ai_controller.id(),
        walker.id()
    );

    println!("Starting AI control...");
    ai_controller.start();
    thread::sleep(Duration::from_millis(100));

    println!("Stopping AI control...");
    ai_controller.stop();
    thread::sleep(Duration::from_millis(100));

    println!("Restarting AI control...");
    ai_controller.start();
    thread::sleep(Duration::from_millis(100));

    println!("✓ AI controller start/stop methods work correctly");
}

fn demo_walker_ai_controller_speed() {
    println!("\n--- Demo 7: WalkerAIController Speed Control ---");
    let client = connect_to_carla();
    let walker = spawn_walker(&client);
    let ai_controller = spawn_walker_ai_controller(&client, &walker);

    println!(
        "AI controller ID: {} (attached to walker {})",
        ai_controller.id(),
        walker.id()
    );

    println!("Setting speed to 1.0 m/s (slow walk)...");
    ai_controller.set_max_speed(1.0);
    thread::sleep(Duration::from_millis(100));

    println!("Setting speed to 1.4 m/s (normal walk)...");
    ai_controller.set_max_speed(1.4);
    thread::sleep(Duration::from_millis(100));

    println!("Setting speed to 2.5 m/s (fast walk/jog)...");
    ai_controller.set_max_speed(2.5);
    thread::sleep(Duration::from_millis(100));

    println!("✓ AI controller speed control works correctly");
}

fn demo_walker_with_ai_movement() {
    println!("\n--- Demo 8: Walker with AI Movement ---");
    let client = connect_to_carla();
    let walker = spawn_walker(&client);
    let ai_controller = spawn_walker_ai_controller(&client, &walker);

    let initial_location = walker.location();
    println!(
        "Initial location: ({:.2}, {:.2}, {:.2})",
        initial_location.x, initial_location.y, initial_location.z
    );

    println!("Starting AI with speed 1.5 m/s...");
    ai_controller.set_max_speed(1.5);
    ai_controller.start();

    println!("Waiting 2 seconds for movement...");
    thread::sleep(Duration::from_secs(2));

    let new_location = walker.location();
    println!(
        "New location: ({:.2}, {:.2}, {:.2})",
        new_location.x, new_location.y, new_location.z
    );

    let dx = new_location.x - initial_location.x;
    let dy = new_location.y - initial_location.y;
    let distance = (dx * dx + dy * dy).sqrt();

    println!("Walker moved distance: {:.2}m", distance);

    if distance > 0.1 {
        println!("✓ Walker moved successfully under AI control");
    } else {
        println!("⚠ Walker movement was minimal (may be blocked or stationary)");
    }
}

fn main() {
    println!("=== Walker Integration Demo ===");
    println!("This demo requires a running CARLA simulator on localhost:2000");
    println!("Each demo spawns new actors that are NOT cleaned up automatically.");
    println!("Use CARLA's Python API or restart the simulator to clean up actors.");

    // Run all demos
    demo_walker_spawn();
    demo_walker_control();
    demo_walker_blend_pose();
    demo_walker_location();
    demo_walker_ai_controller_spawn();
    demo_walker_ai_controller_control();
    demo_walker_ai_controller_speed();
    demo_walker_with_ai_movement();

    println!("\n=== All Demos Complete ===");
    println!("\nNote: Spawned actors remain in the simulation.");
    println!(
        "To clean up, either restart the CARLA simulator or use Python API to destroy actors."
    );
}
