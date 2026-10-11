//! Regression tests for the CARLA 0.10 C++ layer, which once compiled against 0.9.x-layout shims.
//!
//! carla-sys parses `csrc_compat_0100/` shims with autocxx, and until the C++ compile switched to
//! CARLA's real headers, the shim layouts also reached libcarla_client: `physics_control()`
//! returned hard-coded defaults (mass 1000 kg, no wheels), `apply_physics_control()` sent a shim
//! struct that the server rejected with "container size overflow", and every `rpc::Command`
//! (184 bytes in the shim, 264 in libcarla) crossed into `apply_batch` with the wrong stride.
//! CARLA 0.10 also slices every exception to a bare `std::exception`, so a `wait_for_tick`
//! timeout came back as an error instead of `Ok(None)`. Each test runs on any version.

mod common;

use std::time::{Duration, Instant};

use carla::{
    client::{ActorBase, ActorKind, Vehicle, World},
    geom::{Location, Vector3D},
    rpc::Command,
};
use common::{client, world};

fn sync_world() -> (World, carla::rpc::EpisodeSettings) {
    let _ = client();
    let mut world = world();
    let original = world.settings().expect("settings");
    let mut settings = original.clone();
    settings.synchronous_mode = true;
    settings.fixed_delta_seconds = Some(0.05);
    world
        .apply_settings(&settings, Duration::from_secs(10))
        .expect("synchronous mode");
    (world, original)
}

fn spawn_vehicle(world: &mut World) -> Vehicle {
    let library = world.blueprint_library().expect("blueprint library");
    // 0.9.x ships vehicle.tesla.model3; 0.10 ships vehicle.lincoln.mkz.
    let blueprint = ["vehicle.tesla.model3", "vehicle.lincoln.mkz"]
        .iter()
        .find_map(|id| library.find(id).ok().flatten())
        .expect("a four-wheeled vehicle blueprint");
    let spawn_points = world
        .map()
        .expect("map")
        .recommended_spawn_points()
        .expect("spawn points");
    let actor = spawn_points
        .iter()
        .find_map(|sp| world.spawn_actor(&blueprint, sp).ok())
        .expect("spawn a vehicle");
    let ActorKind::Vehicle(vehicle) = actor.into_kinds() else {
        panic!("spawned actor is not a vehicle");
    };
    for _ in 0..5 {
        world.tick().expect("tick");
    }
    vehicle
}

#[test]
fn t01_physics_control_round_trips() {
    let (mut world, original) = sync_world();
    let vehicle = spawn_vehicle(&mut world);

    let physics = vehicle.physics_control().expect("physics_control");
    assert_eq!(
        physics.wheels.len(),
        4,
        "a four-wheeled vehicle reports four wheels"
    );
    assert!(physics.mass > 0.0);

    let mut heavier = physics.clone();
    heavier.mass = physics.mass + 123.0;
    vehicle
        .apply_physics_control(&heavier)
        .expect("apply_physics_control");
    world.tick().expect("tick");
    let read_back = vehicle.physics_control().expect("physics_control");
    assert!(
        (read_back.mass - heavier.mass).abs() < 1e-3,
        "mass {} after applying {}",
        read_back.mass,
        heavier.mass
    );
    assert_eq!(read_back.wheels.len(), 4);

    vehicle.destroy().expect("destroy");
    let _ = world.apply_settings(&original, Duration::from_secs(10));
}

#[test]
fn t02_batch_commands_apply_in_order() {
    let (mut world, original) = sync_world();
    let vehicle = spawn_vehicle(&mut world);
    let physics = vehicle.physics_control().expect("physics_control");

    let mut first = vehicle.transform().expect("transform");
    first.location.x += 5.0;
    let mut last = first.clone();
    last.location.y += 3.0;
    let id = vehicle.id();
    let frame_before = world.snapshot().expect("snapshot").frame();
    let mut client =
        carla::client::Client::connect("127.0.0.1", world_port(), None).expect("second client");
    let responses = client
        .apply_batch_sync(
            vec![
                Command::ApplyTransform {
                    actor_id: id,
                    transform: first,
                },
                Command::ApplyTargetVelocity {
                    actor_id: id,
                    velocity: Vector3D::new(0.0, 0.0, 0.0),
                },
                Command::ApplyVehiclePhysicsControl {
                    actor_id: id,
                    physics_control: physics,
                },
                Command::ApplyTransform {
                    actor_id: id,
                    transform: last.clone(),
                },
            ],
            true,
        )
        .expect("apply_batch_sync");
    assert_eq!(responses.len(), 4);
    for r in &responses {
        assert!(r.error().is_none(), "batch command failed: {:?}", r.error());
    }
    // The tick cue ran on the second client; this client's cached episode state catches up with
    // it asynchronously, and reading the transform before it does sees the car still at spawn.
    let deadline = Instant::now() + Duration::from_secs(5);
    while world.snapshot().expect("snapshot").frame() <= frame_before {
        assert!(
            Instant::now() < deadline,
            "no snapshot after the batch's tick"
        );
        std::thread::sleep(Duration::from_millis(10));
    }
    // Within a metre: the tick that follows the batch lets the car settle and roll. The two
    // transforms are 3 m apart in y, so this still tells the last command from the first.
    let at = vehicle.transform().expect("transform").location;
    assert!(
        (at.x - last.location.x).abs() < 1.0 && (at.y - last.location.y).abs() < 1.0,
        "vehicle at ({}, {}), last command put it at ({}, {})",
        at.x,
        at.y,
        last.location.x,
        last.location.y
    );

    vehicle.destroy().expect("destroy");
    let _ = world.apply_settings(&original, Duration::from_secs(10));
}

#[test]
fn t03_wait_for_tick_times_out_as_none() {
    let (mut world, original) = sync_world();
    // Synchronous and nobody ticking: the wait must run out and report it as None.
    let start = Instant::now();
    let result = world.wait_for_tick_or_timeout(Duration::from_millis(500));
    let elapsed = start.elapsed();
    assert!(
        matches!(result, Ok(None)),
        "expected Ok(None), got {:?}",
        result.map(|s| s.map(|s| s.frame()))
    );
    assert!(
        elapsed >= Duration::from_millis(450),
        "returned after {elapsed:?}"
    );
    let _ = world.apply_settings(&original, Duration::from_secs(10));
    let _ = &mut world;
}

#[test]
fn t04_geolocation_follows_the_left_handed_frame() {
    let map = world().map().expect("map");
    let origin = map
        .transform_to_geolocation(&Location::new(0.0, 0.0, 0.0))
        .expect("geo");
    let south = map
        .transform_to_geolocation(&Location::new(0.0, 100.0, 0.0))
        .expect("geo");
    let east = map
        .transform_to_geolocation(&Location::new(100.0, 0.0, 0.0))
        .expect("geo");
    let reference = map.geo_reference().expect("geo reference");
    assert!((origin.latitude - reference.latitude).abs() < 1e-9);
    // CARLA: x east, y south. 100 m is about 9e-4 degrees.
    assert!(east.longitude > origin.longitude, "+x must be east");
    assert!((east.latitude - origin.latitude).abs() < 1e-7);
    // CARLA's frame is left-handed, so +y is south and should lower the latitude. CARLA 0.10 does
    // that. LibCarla 0.9.x does not: its GeoLocation::Transform raises the latitude with +y,
    // mirroring the map north-south (CARLA's own 0.9.16 Python client agrees, so this is the
    // server library's convention, not this crate's). Callers converting GNSS to a map frame
    // must therefore flip latitude on 0.9.x and not on 0.10.
    #[cfg(carla_version_0100)]
    assert!(south.latitude < origin.latitude, "0.10: +y must be south");
    #[cfg(not(carla_version_0100))]
    assert!(
        south.latitude > origin.latitude,
        "0.9.x: +y raises latitude"
    );
}

fn world_port() -> u16 {
    std::env::var("CARLA_TEST_BASE_PORT")
        .ok()
        .and_then(|p| p.parse().ok())
        .unwrap_or(2000)
}
