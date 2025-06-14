//! Walker Control System Demo
//!
//! This example demonstrates the comprehensive walker control capabilities
//! of the carla-cxx library, including:
//! - Basic walker control (direction, speed, jump)
//! - Walker AI controller for autonomous navigation
//! - Bone control for custom animations
//! - Walker physics and movement

use anyhow::Result;
use carla_cxx::{
    Vector3D, WalkerAIBehavior, WalkerAIControllerWrapper, WalkerControl, WalkerLocation,
    WalkerWrapper,
};

fn main() -> Result<()> {
    println!("🚶 CARLA Walker Control System Demo");
    println!("===================================");

    // This demo shows all walker control features without requiring a running CARLA server
    demonstrate_basic_walker_control()?;
    demonstrate_walker_ai_control()?;
    demonstrate_pose_control()?;
    demonstrate_walker_physics()?;

    println!("\n✅ Walker control system demo completed successfully!");
    println!("Note: To test with actual walkers, run this with a CARLA server instance.");

    Ok(())
}

fn demonstrate_basic_walker_control() -> Result<()> {
    println!("\n🎮 Basic Walker Control");
    println!("----------------------");

    // Create walker control for different scenarios
    let walking_forward = WalkerControl::new().direction(1.0, 0.0, 0.0).speed(1.4); // Normal walking speed

    println!(
        "Walking Forward: direction=({:.1}, {:.1}, {:.1}), speed={} m/s",
        walking_forward.direction.x,
        walking_forward.direction.y,
        walking_forward.direction.z,
        walking_forward.speed
    );

    let running_diagonal = WalkerControl::new()
        .direction(0.707, 0.707, 0.0) // 45 degree angle
        .speed(3.0); // Running speed

    println!(
        "Running Diagonal: direction=({:.3}, {:.3}, {:.3}), speed={} m/s",
        running_diagonal.direction.x,
        running_diagonal.direction.y,
        running_diagonal.direction.z,
        running_diagonal.speed
    );

    let jumping = WalkerControl::new()
        .direction(0.0, 0.0, 0.0)
        .speed(0.0)
        .jump(true);

    println!("Jumping: jump={}", jumping.jump);

    Ok(())
}

fn demonstrate_walker_ai_control() -> Result<()> {
    println!("\n🤖 Walker AI Controller");
    println!("----------------------");

    // Create AI behavior configurations
    let cautious_walker = WalkerAIBehavior {
        max_speed: 1.0,
        cross_roads: true,
        random_crossing_probability: 0.05,
    };

    println!(
        "Cautious Walker: max_speed={} m/s, cross_roads={}, random_crossing_prob={}",
        cautious_walker.max_speed,
        cautious_walker.cross_roads,
        cautious_walker.random_crossing_probability
    );

    let hurried_walker = WalkerAIBehavior {
        max_speed: 2.5,
        cross_roads: true,
        random_crossing_probability: 0.3,
    };

    println!(
        "Hurried Walker: max_speed={} m/s, cross_roads={}, random_crossing_prob={}",
        hurried_walker.max_speed,
        hurried_walker.cross_roads,
        hurried_walker.random_crossing_probability
    );

    // Demonstrate navigation points
    let destinations = vec![
        WalkerLocation::new(100.0, 50.0, 0.0),
        WalkerLocation::new(-50.0, 100.0, 0.0),
        WalkerLocation::new(0.0, 0.0, 0.0),
    ];

    println!("\nNavigation destinations:");
    for (i, dest) in destinations.iter().enumerate() {
        println!("  {}: ({:.1}, {:.1}, {:.1})", i + 1, dest.x, dest.y, dest.z);
    }

    Ok(())
}

fn demonstrate_pose_control() -> Result<()> {
    println!("\n🦴 Walker Pose Control");
    println!("---------------------");

    println!("Walker pose blending allows mixing between:");
    println!("  • Default walking animation");
    println!("  • Custom pose from animation system");
    println!("  • Previous animation frame");

    println!("\nAvailable pose control methods:");
    println!("  • blend_pose(0.0) - Full animation (default walking)");
    println!("  • blend_pose(0.5) - 50% animation, 50% custom pose");
    println!("  • blend_pose(1.0) - Full custom pose");
    println!("  • show_pose() - Equivalent to blend_pose(1.0)");
    println!("  • hide_pose() - Equivalent to blend_pose(0.0)");
    println!("  • get_pose_from_animation() - Capture current animation frame");

    println!("\nTypical pose control workflow:");
    println!("  1. Call get_pose_from_animation() to capture a pose");
    println!("  2. Use blend_pose() to mix with walking animation");
    println!("  3. Adjust blend value for smooth transitions");

    Ok(())
}

fn demonstrate_walker_physics() -> Result<()> {
    println!("\n⚙️  Walker Physics and Movement");
    println!("------------------------------");

    // Calculate walker movement vectors
    let movement_scenarios = vec![
        ("Forward Walk", Vector3D::new(1.0, 0.0, 0.0), 1.4),
        ("Sideways Walk", Vector3D::new(0.0, 1.0, 0.0), 1.0),
        ("Diagonal Run", Vector3D::new(0.707, 0.707, 0.0), 2.5),
        ("Backward Walk", Vector3D::new(-1.0, 0.0, 0.0), 0.8),
    ];

    for (name, direction, speed) in movement_scenarios {
        let normalized = direction.normalized();
        let velocity = Vector3D::new(
            normalized.x * speed,
            normalized.y * speed,
            normalized.z * speed,
        );

        println!(
            "{}: velocity=({:.2}, {:.2}, {:.2}) m/s, speed={:.1} m/s",
            name,
            velocity.x,
            velocity.y,
            velocity.z,
            velocity.length()
        );
    }

    println!("\nWalker capabilities:");
    println!("• Normal walking speed: 1.0-1.5 m/s");
    println!("• Fast walking speed: 1.5-2.0 m/s");
    println!("• Running speed: 2.0-4.0 m/s");
    println!("• Jump height: ~0.5-1.0 meters");
    println!("• Turn rate: Instant (no rotation inertia)");

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_walker_control_creation() {
        let control = WalkerControl::new().direction(1.0, 0.0, 0.0).speed(1.5);

        assert_eq!(control.speed, 1.5);
        assert_eq!(control.direction.x, 1.0);
        assert!(!control.jump);
    }

    #[test]
    fn test_walker_ai_behavior_defaults() {
        let behavior = WalkerAIBehavior::default();
        assert_eq!(behavior.max_speed, 1.4);
        assert!(behavior.cross_roads);
        assert_eq!(behavior.random_crossing_probability, 0.1);
    }

    #[test]
    fn test_walker_location_distance() {
        let loc1 = WalkerLocation::new(0.0, 0.0, 0.0);
        let loc2 = WalkerLocation::new(3.0, 4.0, 0.0);

        assert_eq!(loc1.distance_to(&loc2), 5.0);
    }

    #[test]
    fn test_vector_normalization() {
        let vec = Vector3D::new(3.0, 4.0, 0.0);
        let normalized = vec.normalized();

        assert!((normalized.length() - 1.0).abs() < 0.001);
        assert!((normalized.x - 0.6).abs() < 0.001);
        assert!((normalized.y - 0.8).abs() < 0.001);
    }
}
