//! Custom V2X sensor example (CARLA 0.9.16)
//!
//! Parks three cars on the current map with a `sensor.other.v2x_custom` on
//! each roof, each
//! broadcasting a short text every tick in synchronous mode, and prints the
//! received power of every message. Shows that a sender must itself be
//! listening for its messages to be delivered.
//!
//! Prerequisites:
//! - CARLA 0.9.16 simulator must be running
//!
//! Run with:
//! ```bash
//! CARLA_VERSION=0.9.16 cargo run --profile dev-release --example v2x_custom
//! ```

#[cfg(carla_version_0916)]
fn main() -> Result<(), Box<dyn std::error::Error>> {
    use carla::{
        client::{ActorBase, Client, Sensor},
        geom::{Location, Rotation, Transform},
        rpc::AttachmentType,
        sensor::data::CustomV2XEvent,
    };
    use std::{
        sync::{Arc, Mutex},
        time::Duration,
    };

    let client = Client::connect("localhost", 2000, None)?;
    let mut world = client.world()?;
    println!("Map: {}", world.map()?.name());

    let original = world.settings()?;
    let mut settings = original.clone();
    settings.synchronous_mode = true;
    settings.fixed_delta_seconds = Some(0.05);
    world.apply_settings(&settings, Duration::from_secs(5))?;

    let library = world.blueprint_library()?;
    let mut bp = library
        .find("sensor.other.v2x_custom")?
        .ok_or("sensor.other.v2x_custom not found (CARLA 0.9.16 required)")?;
    for (key, value) in [("path_loss_model", "winner"), ("scenario", "urban")] {
        if !bp.set_attribute(key, value) {
            return Err(format!("cannot set {key}").into());
        }
    }

    // Custom V2X sensors deliver only between sensors attached to actors:
    // three parked cars at the first spawn points, an antenna on each roof.
    let car_bp = library
        .find("vehicle.lincoln.mkz_2020")?
        .ok_or("vehicle.lincoln.mkz_2020 not found")?;
    let spawn_points = world.map()?.recommended_spawn_points()?;
    let mut cars = Vec::new();
    let mut sensors = Vec::new();
    for i in 0..spawn_points.len() {
        if cars.len() == 3 {
            break;
        }
        let Some(tf) = spawn_points.get(i) else {
            continue;
        };
        let Ok(car) = world.spawn_actor(&car_bp, tf) else {
            continue;
        };
        car.set_simulate_physics(false)?;
        let roof = Transform {
            location: Location::new(0.0, 0.0, 1.6),
            rotation: Rotation::new(0.0, 0.0, 0.0),
        };
        let antenna = world.spawn_actor_attached(&bp, &roof, &car, AttachmentType::Rigid)?;
        sensors.push(Sensor::try_from(antenna).map_err(|_| "not a sensor")?);
        cars.push(car);
    }

    let received: Arc<Mutex<Vec<(u32, f32, String)>>> = Arc::default();
    for s in &sensors {
        let id = s.id();
        let sink = Arc::clone(&received);
        s.listen(move |data| {
            if let Ok(event) = CustomV2XEvent::try_from(data) {
                let mut sink = sink.lock().unwrap();
                for m in event.messages() {
                    sink.push((id, m.power_dbm, m.message));
                }
            }
        })?;
    }

    // Let the sensor streams subscribe before the first message.
    for _ in 0..3 {
        world.tick()?;
    }
    std::thread::sleep(Duration::from_millis(200));

    for tick in 0..5 {
        for s in &sensors {
            s.send(&format!("hello from {} at tick {tick}", s.id()))?;
        }
        world.tick()?;
        std::thread::sleep(Duration::from_millis(100));
        for (rx, power, text) in received.lock().unwrap().drain(..) {
            println!("tick {tick}: sensor {rx} got {power:.1} dBm: {text}");
        }
    }

    for s in &sensors {
        s.stop()?;
        s.destroy()?;
    }
    for c in &cars {
        c.destroy()?;
    }
    world.tick()?;
    world.apply_settings(&original, Duration::from_secs(5))?;
    Ok(())
}

#[cfg(not(carla_version_0916))]
fn main() {
    eprintln!("The custom V2X sensor needs CARLA 0.9.16 (CARLA_VERSION=0.9.16).");
}
