//! Pins the units of `ActorBase::angular_velocity`.
//!
//! CARLA reports actor angular velocity in **degrees** per second, while the IMU
//! sensor's gyroscope reports radians. This crate's doc comment claimed radians for both
//! until it was measured, and a caller that mixes them is wrong by a factor of 57.3 with
//! no error anywhere. The test differentiates the actor's own yaw over known simulation
//! steps and checks which unit the reading agrees with.

mod common;

use std::time::Duration;

use carla::{
    client::{ActorBase, ActorKind},
    geom::{Location, Rotation, Transform},
    rpc::VehicleControl,
};
use common::{client, world};

const DT: f64 = 0.05;

/// Wrap a degree difference into (-180, 180].
fn wrap_degrees(delta: f32) -> f32 {
    (delta + 180.0).rem_euclid(360.0) - 180.0
}

#[test]
fn t01_angular_velocity_is_degrees_per_second() {
    let _ = client();
    let mut world = world();

    let original = world.settings().expect("Failed to read episode settings");
    let mut settings = original.clone();
    settings.synchronous_mode = true;
    settings.fixed_delta_seconds = Some(DT);
    world
        .apply_settings(&settings, Duration::from_secs(10))
        .expect("Failed to enter synchronous mode");

    let restore = |world: &mut carla::client::World| {
        let _ = world.apply_settings(&original, Duration::from_secs(10));
    };

    let library = world
        .blueprint_library()
        .expect("Failed to get blueprint library");
    let Some(blueprint) = library
        .find("vehicle.tesla.model3")
        .expect("Failed to look up blueprint")
    else {
        restore(&mut world);
        eprintln!("vehicle.tesla.model3 not available; skipping");
        return;
    };

    let map = world.map().expect("Failed to get map");
    let spawn_points = map
        .recommended_spawn_points()
        .expect("Failed to get spawn points");
    let spawn = spawn_points
        .get(0)
        .expect("Map must have at least one spawn point");

    let actor = world
        .spawn_actor(&blueprint, spawn)
        .expect("Failed to spawn vehicle");
    let ActorKind::Vehicle(vehicle) = actor.into_kinds() else {
        restore(&mut world);
        panic!("Spawned actor is not a vehicle");
    };

    // Level ground, known heading, then a steady turn under throttle.
    let start = Transform {
        location: Location {
            z: spawn.location.z + 0.5,
            ..spawn.location
        },
        rotation: Rotation {
            pitch: 0.0,
            yaw: 0.0,
            roll: 0.0,
        },
    };
    vehicle
        .set_transform(&start)
        .expect("Failed to place vehicle");
    for _ in 0..10 {
        world.tick().expect("tick");
    }

    vehicle
        .apply_control(&VehicleControl {
            throttle: 0.6,
            steer: 0.6,
            brake: 0.0,
            hand_brake: false,
            reverse: false,
            manual_gear_shift: false,
            gear: 0,
        })
        .expect("Failed to apply control");

    // Discard the transient while the vehicle builds up yaw rate, then average.
    let mut previous_yaw: Option<f32> = None;
    let mut differentiated_deg_s = 0.0_f64;
    let mut reported = 0.0_f64;
    let mut samples = 0_u32;

    for step in 0..80 {
        world.tick().expect("tick");

        let yaw = vehicle
            .transform()
            .expect("Failed to read transform")
            .rotation
            .yaw;
        let angular = vehicle
            .angular_velocity()
            .expect("Failed to read angular velocity");

        if let Some(previous) = previous_yaw {
            let rate = wrap_degrees(yaw - previous) as f64 / DT;
            if step > 40 && rate.abs() > 5.0 {
                differentiated_deg_s += rate;
                reported += angular.z as f64;
                samples += 1;
            }
        }
        previous_yaw = Some(yaw);
    }

    let _ = vehicle.destroy();
    restore(&mut world);

    assert!(
        samples >= 10,
        "The vehicle never reached a steady turn ({samples} samples); the test cannot \
         distinguish the units"
    );

    let differentiated_deg_s = differentiated_deg_s / samples as f64;
    let reported = reported / samples as f64;
    let ratio_against_degrees = reported / differentiated_deg_s;
    let ratio_against_radians = reported / differentiated_deg_s.to_radians();

    println!("differentiated yaw rate : {differentiated_deg_s:8.4} deg/s");
    println!("angular_velocity().z    : {reported:8.4}");
    println!("ratio against deg/s     : {ratio_against_degrees:8.4}");
    println!("ratio against rad/s     : {ratio_against_radians:8.4}");

    assert!(
        (ratio_against_degrees - 1.0).abs() < 0.1,
        "angular_velocity() should equal the differentiated yaw rate in DEGREES per \
         second, but the ratio was {ratio_against_degrees:.4} (against radians it was \
         {ratio_against_radians:.4})"
    );
}
