use carla::client::{Client, TrySpawnActorOptions};
use nalgebra::{Isometry3, Translation3, UnitQuaternion};
use std::time::{Duration, Instant};

fn main() {
    let mut client = Client::default();
    client.set_timeout(Duration::from_secs(5));
    let mut world = client.load_world("Town07");

    let spawn_points = world.map().recommended_spawn_points();

    let spectator = world.spectator();
    // let spectator = spectator
    //     .try_into_sensor()
    //     .unwrap_or_else(|_| panic!("spectator is not a sensor"));
    // spectator.listen(move |_data| {
    //     println!("data received");
    // });

    let blulib = world.blueprint_library();
    // blulib.iter().for_each(|blu| {
    //     println!("{}", blu.id());
    // });

    let vehicle = {
        let blu = blulib.find("vehicle.tesla.model3").unwrap();
        let pose = Isometry3 {
            translation: Translation3::new(72.31, -7.55, 1.0),
            rotation: UnitQuaternion::from_euler_angles(0.01, 1.13, -62.79),
        };
        let actor = world.try_spawn_actor(&blu, &pose).unwrap();
        actor
            .try_into_vehicle()
            .unwrap_or_else(|_| panic!("not a vehicle"))
    };

    let lidar = {
        let blu = blulib.find("sensor.camera.rgb").unwrap();
        let actor = world
            .try_spawn_actor_opt(
                &blu,
                &Isometry3::identity(),
                TrySpawnActorOptions {
                    parent: Some(&vehicle),
                    ..Default::default()
                },
            )
            .unwrap();
        actor
            .try_into_sensor()
            .unwrap_or_else(|_| panic!("not a sensor"))
    };

    lidar.listen(move |_data| {
        println!("data received");
    });

    loop {
        world.wait_for_tick();
    }
}
