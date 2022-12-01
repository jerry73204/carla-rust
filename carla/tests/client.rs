use carla::client::Client;
use nalgebra::{Isometry3, Translation3, UnitQuaternion};

#[test]
fn client_connect() {
    let client = Client::default();
    let mut world = client.load_world("Town07");

    let spectator = world.spectator();
    let spectator = spectator
        .try_into_sensor()
        .unwrap_or_else(|_| panic!("spectator is not a sensor"));
    spectator.listen(move |_data| {
        println!("data received");
    });

    let blulib = world.blueprint_library();
    let vblu = blulib.find("vehicle.tesla.model3").unwrap();
    let pose = Isometry3 {
        translation: Translation3::new(72.31, -7.55, 1.0),
        rotation: UnitQuaternion::from_euler_angles(0.01, 1.13, -62.79),
    };
    let _actor = world.try_spawn_actor(&vblu, &pose, None).unwrap();
}
