use anyhow::Result;
use carla::{
    client::{Client, Sensor, Vehicle},
    sensor::data::{Image, LidarMeasurement},
};
use nalgebra::{Isometry3, Translation3, UnitQuaternion};
use std::time::Duration;

fn main() -> Result<()> {
    let mut client = Client::default();
    client.set_timeout(Duration::from_secs(5));

    // Use synchronous mode
    let mut world = client.load_world("Town07");
    let is_sync = world.settings().synchronous_mode;

    // Spawn objects
    let vehicle_pose = Isometry3 {
        translation: Translation3::new(72.31, -7.55, 1.0),
        rotation: UnitQuaternion::from_euler_angles(
            0.01_f32.to_radians(),
            1.13_f32.to_radians(),
            -62.79_f32.to_radians(),
        ),
    };

    let vehicle: Vehicle = world
        .actor_builder("vehicle.tesla.model3")?
        .spawn_vehicle(&vehicle_pose)?;
    let camera: Sensor = world.actor_builder("sensor.camera.rgb")?.spawn_sensor_opt(
        &Isometry3::identity(),
        Some(&vehicle),
        None,
    )?;
    let lidar: Sensor = world
        .actor_builder("sensor.lidar.ray_cast")?
        .set_attribute("channels", "32")?
        .set_attribute("points_per_second", "90000")?
        .set_attribute("rotation_frequency", "40")?
        .set_attribute("range", "20")?
        .spawn_sensor_opt(&Isometry3::identity(), Some(&vehicle), None)?;

    // Set sensor callbacks
    camera.listen(move |data| {
        let image: Image = data.try_into().unwrap();
        let fov = image.fov_angle();
        let array = image.as_array();
        println!("image shape={:?} fov={}", array.shape(), fov);
    });

    lidar.listen(move |data| {
        let measure: LidarMeasurement = data.try_into().unwrap();
        let n_channels = measure.channel_count();
        println!("lidar n_channels={:?}", n_channels);
    });

    // Spin the simulator
    loop {
        if is_sync {
            world.tick();
        } else {
            world.wait_for_tick();
        }
    }
}
