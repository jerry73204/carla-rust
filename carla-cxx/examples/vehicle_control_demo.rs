//! Vehicle Control System Demo
//!
//! This example demonstrates the comprehensive vehicle control capabilities
//! of the carla-cxx library, including:
//! - Basic vehicle control (throttle, brake, steering)
//! - Advanced Ackermann control for precise vehicle dynamics
//! - Physics control for engine, transmission, and wheel parameters
//! - Vehicle door control (CARLA 0.10.0 feature)
//! - Vehicle telemetry and monitoring

use anyhow::Result;
use carla_cxx::{
    AckermannControl, EnginePhysics, SimpleVector3D, SteeringPhysics, TransmissionPhysics,
    VehicleControl, VehicleDoorType, VehiclePhysicsControl, WheelPhysicsControl,
};

fn main() -> Result<()> {
    println!("🚗 CARLA Vehicle Control System Demo");
    println!("=====================================");

    // This demo shows all vehicle control features without requiring a running CARLA server
    demonstrate_control_structures()?;
    demonstrate_physics_control()?;
    demonstrate_ackermann_control()?;
    demonstrate_door_control()?;
    demonstrate_telemetry_features()?;

    println!("\n✅ Vehicle control system demo completed successfully!");
    println!("Note: To test with actual vehicles, run this with a CARLA server instance.");

    Ok(())
}

fn demonstrate_control_structures() -> Result<()> {
    println!("\n📋 Basic Vehicle Control Structures");
    println!("-----------------------------------");

    // Create basic vehicle control
    let basic_control = VehicleControl::new()
        .throttle(0.8)
        .brake(0.0)
        .steer(-0.2)
        .hand_brake(false)
        .reverse(false);

    println!(
        "Basic Control: throttle={}, brake={}, steer={}",
        basic_control.throttle, basic_control.brake, basic_control.steer
    );

    // Create Ackermann control for precise vehicle dynamics
    let ackermann_control = AckermannControl::new()
        .steer(0.5)
        .steer_speed(1.0)
        .speed(30.0)
        .acceleration(2.0)
        .jerk(1.0);

    println!(
        "Ackermann Control: steer={}, speed={} km/h, accel={} m/s²",
        ackermann_control.steer, ackermann_control.speed, ackermann_control.acceleration
    );

    Ok(())
}

fn demonstrate_physics_control() -> Result<()> {
    println!("\n⚙️  Advanced Physics Control");
    println!("---------------------------");

    // Create comprehensive engine physics
    let engine = EnginePhysics {
        torque_curve_max_rpm: 5000.0,
        torque_curve_max_torque_nm: 400.0,
        max_rpm: 6000.0,
        moi: 1.0,
        damping_rate_full_throttle: 0.15,
        damping_rate_zero_throttle_clutch_engaged: 2.0,
        damping_rate_zero_throttle_clutch_disengaged: 0.35,
    };

    // Create transmission physics
    let transmission = TransmissionPhysics {
        use_gear_autobox: true,
        gear_switch_time: 0.5,
        clutch_strength: 10.0,
        final_ratio: 4.0,
    };

    // Create steering physics
    let steering = SteeringPhysics {
        curve_at_zero: 1.0,
        curve_at_one: 0.5,
    };

    // Create complete vehicle physics control
    let physics_control = VehiclePhysicsControl {
        engine,
        transmission,
        mass: 1500.0,
        drag_coefficient: 0.25,
        center_of_mass: SimpleVector3D {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        steering,
        use_sweep_wheel_collision: true,
    };

    println!(
        "Engine: max_rpm={}, max_torque={} Nm, moi={}",
        physics_control.engine.max_rpm,
        physics_control.engine.torque_curve_max_torque_nm,
        physics_control.engine.moi
    );

    println!(
        "Transmission: auto_gears={}, gear_time={}s, final_ratio={}",
        physics_control.transmission.use_gear_autobox,
        physics_control.transmission.gear_switch_time,
        physics_control.transmission.final_ratio
    );

    println!(
        "Vehicle: mass={}kg, drag_coeff={}, sweep_collision={}",
        physics_control.mass,
        physics_control.drag_coefficient,
        physics_control.use_sweep_wheel_collision
    );

    Ok(())
}

fn demonstrate_ackermann_control() -> Result<()> {
    println!("\n🎯 Ackermann Steering Control");
    println!("-----------------------------");

    // Ackermann control provides more precise vehicle dynamics
    // compared to basic throttle/brake/steer control

    let scenarios = vec![
        (
            "Gentle Turn",
            AckermannControl::new()
                .steer(0.2)
                .speed(25.0)
                .acceleration(0.0),
        ),
        (
            "Sharp Turn",
            AckermannControl::new()
                .steer(0.8)
                .speed(15.0)
                .acceleration(-1.0),
        ),
        (
            "Lane Change",
            AckermannControl::new()
                .steer(-0.3)
                .speed(35.0)
                .acceleration(0.5),
        ),
        (
            "Emergency Stop",
            AckermannControl::new()
                .steer(0.0)
                .speed(0.0)
                .acceleration(-8.0)
                .jerk(2.0),
        ),
    ];

    for (name, control) in scenarios {
        println!(
            "{}: steer={:.1}, speed={:.1} km/h, accel={:.1} m/s², jerk={:.1}",
            name, control.steer, control.speed, control.acceleration, control.jerk
        );
    }

    Ok(())
}

fn demonstrate_door_control() -> Result<()> {
    println!("\n🚪 Vehicle Door Control (CARLA 0.10.0)");
    println!("--------------------------------------");

    let doors = vec![
        ("Front Left", VehicleDoorType::FrontLeft),
        ("Front Right", VehicleDoorType::FrontRight),
        ("Rear Left", VehicleDoorType::RearLeft),
        ("Rear Right", VehicleDoorType::RearRight),
    ];

    for (name, door_type) in doors {
        println!("{} door: type={:?}", name, door_type);
    }

    println!("Note: Door state querying is not available in CARLA 0.10.0 API");
    println!("Available operations: open_door(), close_door()");

    Ok(())
}

fn demonstrate_telemetry_features() -> Result<()> {
    println!("\n📊 Vehicle Telemetry Features");
    println!("-----------------------------");

    // Create wheel physics control examples
    let mut front_wheel = WheelPhysicsControl::default();
    front_wheel.tire_friction = 2.5;
    front_wheel.max_steer_angle = 0.6108;
    front_wheel.radius = 0.35;

    let mut rear_wheel = WheelPhysicsControl::default();
    rear_wheel.tire_friction = 2.0;
    rear_wheel.max_steer_angle = 0.0;
    rear_wheel.radius = 0.35;

    println!(
        "Front Wheel: friction={}, steer_angle={:.3} rad, radius={}m",
        front_wheel.tire_friction, front_wheel.max_steer_angle, front_wheel.radius
    );

    println!(
        "Rear Wheel: friction={}, steer_angle={:.3} rad, radius={}m",
        rear_wheel.tire_friction, rear_wheel.max_steer_angle, rear_wheel.radius
    );

    println!("\nAvailable telemetry:");
    println!("• Vehicle velocity (3D vector)");
    println!("• Angular velocity (3D vector)");
    println!("• Acceleration (3D vector)");
    println!("• Tire friction (average across wheels)");
    println!("• Engine RPM");
    println!("• Current gear ratio");
    println!("• Wheel physics parameters");
    println!("• Gear physics controls");

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_vehicle_control_creation() {
        let control = VehicleControl::new().throttle(0.5).brake(0.2).steer(0.1);

        assert_eq!(control.throttle, 0.5);
        assert_eq!(control.brake, 0.2);
        assert_eq!(control.steer, 0.1);
    }

    #[test]
    fn test_ackermann_control_creation() {
        let control = AckermannControl::new().speed(25.0).acceleration(1.5);

        assert_eq!(control.speed, 25.0);
        assert_eq!(control.acceleration, 1.5);
    }

    #[test]
    fn test_physics_control_defaults() {
        let physics = VehiclePhysicsControl::default();
        assert_eq!(physics.mass, 1000.0);
        assert_eq!(physics.drag_coefficient, 0.3);
        assert!(physics.transmission.use_gear_autobox);
    }

    #[test]
    fn test_wheel_physics_defaults() {
        let wheel = WheelPhysicsControl::default();
        assert_eq!(wheel.tire_friction, 2.0);
        assert_eq!(wheel.radius, 0.3);
        assert_eq!(wheel.max_brake_torque, 1500.0);
    }
}
