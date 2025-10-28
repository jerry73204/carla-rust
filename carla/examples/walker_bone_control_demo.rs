//! Demonstration of WalkerBoneControl types and data structures.
//!
//! This example shows how to create and manipulate bone transform data
//! for custom walker animations. These structures can be used with
//! `Walker::set_bones()` (when available) to control individual bones
//! in a walker's skeleton.
//!
//! Run with: `cargo run --example walker_bone_control_demo`

use carla::{
    geom::{Location, Rotation, Transform},
    rpc::{BoneTransformDataIn, WalkerBoneControlIn},
};

/// Helper to create a default transform
fn default_transform() -> Transform {
    Transform {
        location: Location {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        rotation: Rotation {
            pitch: 0.0,
            yaw: 0.0,
            roll: 0.0,
        },
    }
}

fn main() {
    println!("=== Walker Bone Control Demo ===\n");

    // 1. Create a simple bone transform
    println!("1. Creating a BoneTransformDataIn with default transform:");
    let bone = BoneTransformDataIn {
        bone_name: "crl_arm__L".to_string(),
        transform: default_transform(),
    };
    println!("   Bone name: {}", bone.bone_name);
    println!(
        "   Location: ({}, {}, {})\n",
        bone.transform.location.x, bone.transform.location.y, bone.transform.location.z
    );

    // 2. Create a bone transform with custom values
    println!("2. Creating a BoneTransformDataIn with custom transform:");
    let transform = Transform {
        location: Location {
            x: 1.0,
            y: 2.0,
            z: 3.0,
        },
        rotation: Rotation {
            pitch: 10.0,
            yaw: 20.0,
            roll: 30.0,
        },
    };

    let custom_bone = BoneTransformDataIn {
        bone_name: "crl_leg__R".to_string(),
        transform,
    };
    println!("   Bone name: {}", custom_bone.bone_name);
    println!(
        "   Location: ({}, {}, {})",
        custom_bone.transform.location.x,
        custom_bone.transform.location.y,
        custom_bone.transform.location.z
    );
    println!(
        "   Rotation: (pitch={}, yaw={}, roll={})\n",
        custom_bone.transform.rotation.pitch,
        custom_bone.transform.rotation.yaw,
        custom_bone.transform.rotation.roll
    );

    // 3. Create an empty WalkerBoneControlIn
    println!("3. Creating empty WalkerBoneControlIn:");
    let control = WalkerBoneControlIn::default();
    println!("   Bone count: {}\n", control.bone_transforms.len());

    // 4. Create WalkerBoneControlIn with multiple bones
    println!("4. Creating WalkerBoneControlIn with multiple bones:");
    let bones = vec![
        BoneTransformDataIn {
            bone_name: "crl_arm__L".to_string(),
            transform: default_transform(),
        },
        BoneTransformDataIn {
            bone_name: "crl_arm__R".to_string(),
            transform: default_transform(),
        },
        BoneTransformDataIn {
            bone_name: "crl_leg__L".to_string(),
            transform: default_transform(),
        },
        BoneTransformDataIn {
            bone_name: "crl_leg__R".to_string(),
            transform: default_transform(),
        },
    ];

    let multi_bone_control = WalkerBoneControlIn {
        bone_transforms: bones,
    };
    println!(
        "   Bone count: {}",
        multi_bone_control.bone_transforms.len()
    );
    println!("   Bones:");
    for (i, bone) in multi_bone_control.bone_transforms.iter().enumerate() {
        println!("     {}. {}", i + 1, bone.bone_name);
    }
    println!();

    // 5. Clone WalkerBoneControlIn
    println!("5. Cloning WalkerBoneControlIn:");
    let cloned = multi_bone_control.clone();
    println!(
        "   Original bone count: {}",
        multi_bone_control.bone_transforms.len()
    );
    println!("   Cloned bone count: {}", cloned.bone_transforms.len());
    println!(
        "   First bone matches: {}\n",
        cloned.bone_transforms[0].bone_name == multi_bone_control.bone_transforms[0].bone_name
    );

    // 6. Build WalkerBoneControlIn incrementally
    println!("6. Building WalkerBoneControlIn incrementally:");
    let mut incremental_control = WalkerBoneControlIn::default();
    println!(
        "   Initial bone count: {}",
        incremental_control.bone_transforms.len()
    );

    incremental_control
        .bone_transforms
        .push(BoneTransformDataIn {
            bone_name: "bone1".to_string(),
            transform: default_transform(),
        });
    println!(
        "   After adding bone1: {}",
        incremental_control.bone_transforms.len()
    );

    incremental_control
        .bone_transforms
        .push(BoneTransformDataIn {
            bone_name: "bone2".to_string(),
            transform: default_transform(),
        });
    println!(
        "   After adding bone2: {}",
        incremental_control.bone_transforms.len()
    );

    println!("\n=== Demo Complete ===");
    println!(
        "\nNote: These data structures can be used with Walker::blend_pose(), Walker::show_pose(),"
    );
    println!(
        "and Walker::hide_pose() methods to control walker animations in the CARLA simulator."
    );
    println!(
        "Advanced bone manipulation methods (set_bones, get_bones_transform) are currently deferred."
    );
}
