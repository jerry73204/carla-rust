//! CARLA 1.0's client API: what changed from 0.10, checked against a live 1.0 server.
//!
//! Each test is compiled only with the `carla-10` feature.

#![cfg(carla_10)]

mod common;

use std::time::Duration;

use carla::{
    client::{ActorBase, Vehicle, World},
    geom::{Rotation, Vector3D},
    rpc::EpisodeSettings,
};
use common::{client, world};

fn spawn_vehicle(world: &mut World) -> Vehicle {
    let library = world.blueprint_library().expect("blueprint library");
    let bp = library
        .filter("vehicle.*")
        .expect("filter")
        .get(0)
        .expect("get")
        .expect("a vehicle blueprint");
    let spawn_points = world
        .map()
        .expect("map")
        .recommended_spawn_points()
        .expect("spawn points");
    let actor = world
        .spawn_actor(&bp, spawn_points.get(0).expect("a spawn point"))
        .expect("spawn");
    Vehicle::try_from(actor).unwrap_or_else(|_| panic!("the blueprint is not a vehicle"))
}

#[test]
fn t01_versions_name_1_0() {
    let c = client();
    let server = c.server_version().expect("server version");
    let own = c.client_version().expect("client version");
    println!(
        "server {server}, client {own}, crate {}",
        carla::CARLA_VERSION
    );
    assert_eq!(carla::CARLA_VERSION, "1.0");
    assert!(server.starts_with("1.0"), "server reports {server}");
}

#[test]
fn t02_settings_round_trip_the_1_0_fields() {
    let mut w = world();
    let original = w.settings().expect("settings");
    assert!(!original.deterministic_collisions, "off by default");

    let mut changed = original.clone();
    changed.deterministic_collisions = true;
    changed.adjust_signs_height_to_ground = true;
    w.apply_settings(&changed, Duration::from_secs(10))
        .expect("apply");
    let read: EpisodeSettings = w.settings().expect("settings");
    w.apply_settings(&original, Duration::from_secs(10))
        .expect("restore");

    assert!(read.deterministic_collisions);
    assert!(read.adjust_signs_height_to_ground);
}

#[test]
fn t03_telemetry_reports_four_wheels() {
    let mut w = world();
    let vehicle = spawn_vehicle(&mut w);
    // A fresh handle may not have a physics state yet; let the server run a few frames.
    for _ in 0..5 {
        let _ = w.wait_for_tick_or_timeout(Duration::from_secs(2));
    }
    let telemetry = vehicle.telemetry_data();
    let _ = vehicle.destroy();
    let telemetry = telemetry.expect("telemetry");
    println!("{telemetry:?}");
    assert_eq!(telemetry.wheels.len(), 4);
    assert!(telemetry.speed.is_finite());
}

#[test]
fn t04_destroy_asks_the_server() {
    let mut w = world();
    let vehicle = spawn_vehicle(&mut w);
    // 1.0: alive straight after the spawn, before any snapshot lists it.
    assert!(vehicle.is_alive().expect("is_alive"));
    // 1.0: destroy() right after spawn_actor() succeeds (0.10 could answer false).
    assert!(vehicle.destroy().expect("destroy"));
    assert!(!vehicle.destroy().expect("second destroy"));
}

#[test]
fn t05_rotation_follows_unreal_semantics() {
    // 1.0's changelog says the pitch and roll order was corrected; LibCarla 1.0's
    // RotateVector documents the result (verified by CARLA on a UE 5.8 server):
    // +pitch tilts the forward axis up, +roll tilts the right axis down.
    let up = Rotation {
        pitch: 20.0,
        yaw: 0.0,
        roll: 0.0,
    }
    .rotate_vector(&Vector3D::new(1.0, 0.0, 0.0));
    assert!((up.z - 20f32.to_radians().sin()).abs() < 1e-5, "{up:?}");

    let rolled = Rotation {
        pitch: 0.0,
        yaw: 0.0,
        roll: 25.0,
    }
    .rotate_vector(&Vector3D::new(0.0, 1.0, 0.0));
    assert!(
        (rolled.z + 25f32.to_radians().sin()).abs() < 1e-5,
        "{rolled:?}"
    );
}
