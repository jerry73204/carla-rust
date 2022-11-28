use carla_rust::Client;
use nalgebra::{Isometry3, Translation3, UnitQuaternion};

#[test]
fn client_connect() {
    let client = Client::default();
    let mut world = client.load_world("Town07");
    let blulib = world.blueprint_library();
    let vblu = blulib.find("vehicle.tesla.model3").unwrap();
    let pose = Isometry3 {
        translation: Translation3::new(72.31, -7.55, 1.0),
        rotation: UnitQuaternion::from_euler_angles(0.01, 1.13, -62.79),
    };
    let actor = world.try_spawn_actor(&vblu, &pose, None).unwrap();
}
