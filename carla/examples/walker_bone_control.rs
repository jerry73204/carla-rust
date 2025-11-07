//! Demonstrates walker bone control for custom animations.
//!
//! This example shows how to:
//! - Spawn a walker pedestrian
//! - Retrieve current bone transforms
//! - Set custom bone transforms
//! - Blend between animation and custom pose
//!
//! Run with: `cargo run --example walker_bone_control --profile dev-release`

use carla::{
    client::{ActorBase, Client, Walker},
    geom::{Location, Rotation, Transform},
    rpc::{BoneTransformDataIn, WalkerBoneControlIn},
};

fn main() {
    // Connect to CARLA server
    println!("Connecting to CARLA server at 127.0.0.1:2000");
    let client = Client::connect("127.0.0.1", 2000, None);

    let mut world = client.world();

    // Get walker blueprints
    let bp_lib = world.blueprint_library();
    let walker_bp = match bp_lib.filter("walker.pedestrian.*").get(0) {
        Some(bp) => bp,
        None => {
            eprintln!("No walker blueprints found");
            std::process::exit(1);
        }
    };

    // Get spawn point
    let spawn_points = world.map().recommended_spawn_points();
    let spawn_point = match spawn_points.get(0) {
        Some(point) => point,
        None => {
            eprintln!("No spawn points available");
            std::process::exit(1);
        }
    };

    // Spawn walker
    println!("Spawning walker at {:?}", spawn_point.location);
    let actor = match world.spawn_actor(&walker_bp, spawn_point) {
        Ok(actor) => actor,
        Err(e) => {
            eprintln!("Failed to spawn walker: {}", e);
            std::process::exit(1);
        }
    };

    let walker: Walker = match actor.try_into() {
        Ok(w) => w,
        Err(_) => {
            eprintln!("Failed to convert actor to walker");
            std::process::exit(1);
        }
    };

    println!("Walker spawned with ID: {}", walker.id());

    // Get current bone transforms
    println!("\n=== Getting current bone transforms ===");
    let bone_transforms = walker.get_bones_transform();
    println!("Found {} bones", bone_transforms.bone_transforms.len());

    // Print first few bones as examples
    for (i, bone) in bone_transforms
        .bone_transforms
        .iter()
        .take(5)
        .enumerate()
    {
        println!("\nBone {}: {}", i, bone.bone_name);
        println!("  World: loc({:.2}, {:.2}, {:.2}), rot({:.2}, {:.2}, {:.2})",
            bone.world.location.x, bone.world.location.y, bone.world.location.z,
            bone.world.rotation.pitch, bone.world.rotation.yaw, bone.world.rotation.roll);
        println!("  Component: loc({:.2}, {:.2}, {:.2}), rot({:.2}, {:.2}, {:.2})",
            bone.component.location.x, bone.component.location.y, bone.component.location.z,
            bone.component.rotation.pitch, bone.component.rotation.yaw, bone.component.rotation.roll);
    }

    // Create custom bone transforms
    println!("\n=== Setting custom bone transforms ===");
    let custom_bones = WalkerBoneControlIn {
        bone_transforms: vec![
            BoneTransformDataIn {
                bone_name: "crl_arm__L".to_string(),
                transform: Transform {
                    location: Location::new(0.0, 0.0, 0.0),
                    rotation: Rotation {
                        pitch: 0.0,
                        yaw: 45.0,
                        roll: 0.0,
                    },
                },
            },
            BoneTransformDataIn {
                bone_name: "crl_arm__R".to_string(),
                transform: Transform {
                    location: Location::new(0.0, 0.0, 0.0),
                    rotation: Rotation {
                        pitch: 0.0,
                        yaw: -45.0,
                        roll: 0.0,
                    },
                },
            },
        ],
    };

    walker.set_bones(&custom_bones);
    println!("Applied custom transforms to {} bones", custom_bones.bone_transforms.len());

    // Show custom pose
    println!("\n=== Showing custom pose ===");
    walker.show_pose();
    println!("Custom pose is now visible (blend factor: 1.0)");

    // Wait a bit to see the pose
    std::thread::sleep(std::time::Duration::from_secs(2));

    // Blend 50/50
    println!("\n=== Blending pose (50% animation, 50% custom) ===");
    walker.blend_pose(0.5);
    println!("Pose blended at 50%");

    std::thread::sleep(std::time::Duration::from_secs(2));

    // Hide custom pose
    println!("\n=== Hiding custom pose ===");
    walker.hide_pose();
    println!("Returned to animation pose (blend factor: 0.0)");

    std::thread::sleep(std::time::Duration::from_secs(1));

    // Get transforms again to verify they changed
    println!("\n=== Verifying bone transforms after modifications ===");
    let final_transforms = walker.get_bones_transform();
    println!("Retrieved {} bones in final state", final_transforms.bone_transforms.len());

    // Find and print our modified bones
    for bone_name in &["crl_arm__L", "crl_arm__R"] {
        if let Some(bone) = final_transforms
            .bone_transforms
            .iter()
            .find(|b| b.bone_name == *bone_name)
        {
            println!("\n{}: rot({:.2}, {:.2}, {:.2})",
                bone_name,
                bone.world.rotation.pitch,
                bone.world.rotation.yaw,
                bone.world.rotation.roll);
        }
    }

    println!("\n=== Walker bone control demo complete ===");
}
