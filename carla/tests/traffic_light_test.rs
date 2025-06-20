//! Integration tests for traffic light functionality

use carla::{
    actor::{TrafficLight, TrafficLightState},
    client::Client,
    traits::ActorT,
};
use std::time::Duration;

/// Test that requires a running CARLA server
/// Run with: cargo test --features test-carla-server
#[test]
#[cfg(feature = "test-carla-server")]
fn test_traffic_light_basic_operations() -> anyhow::Result<()> {
    // Connect to CARLA server
    let client = Client::new("localhost", 2000, None)?;
    let world = client.world()?;

    // Get all actors in the world
    // This will fail with todo!() until actor list iteration is implemented
    let actors = world.actors()?;

    // Find traffic lights by checking actor type
    let traffic_lights: Vec<TrafficLight> = actors
        .into_iter()
        .filter_map(|actor| TrafficLight::from_actor(actor).ok().flatten())
        .collect();

    if traffic_lights.is_empty() {
        eprintln!("No traffic lights found in the current map. Skipping test.");
        return Ok(());
    }

    let traffic_light = &traffic_lights[0];
    println!("Found traffic light with ID: {}", traffic_light.id());

    // Test basic state operations
    let initial_state = traffic_light.state();
    println!("Initial state: {:?}", initial_state);

    // Set to red
    traffic_light.set_state(TrafficLightState::Red)?;
    assert_eq!(traffic_light.state(), TrafficLightState::Red);

    // Set to green
    traffic_light.set_state(TrafficLightState::Green)?;
    assert_eq!(traffic_light.state(), TrafficLightState::Green);

    // Test time settings
    traffic_light.set_green_time(Duration::from_secs(10))?;
    traffic_light.set_yellow_time(Duration::from_secs(3))?;
    traffic_light.set_red_time(Duration::from_secs(15))?;

    assert_eq!(traffic_light.green_time(), Duration::from_secs(10));
    assert_eq!(traffic_light.yellow_time(), Duration::from_secs(3));
    assert_eq!(traffic_light.red_time(), Duration::from_secs(15));

    // Test freeze functionality
    traffic_light.freeze(true)?;
    assert!(traffic_light.is_frozen());

    traffic_light.freeze(false)?;
    assert!(!traffic_light.is_frozen());

    // Test new methods (these will fail with todo!() until FFI functions are implemented)

    // Test pole index
    let pole_index = traffic_light.pole_index();
    println!("Pole index: {}", pole_index);

    // Test affected lane waypoints
    let waypoints = traffic_light.affected_lane_waypoints();
    println!("Affected lanes: {} waypoints", waypoints.len());

    // Test group traffic lights
    let group_ids = traffic_light.group_traffic_lights();
    println!("Traffic light group contains {} lights", group_ids.len());

    // Restore initial state
    traffic_light.set_state(initial_state)?;

    Ok(())
}

/// Test traffic light type conversions
#[test]
fn test_traffic_light_state_conversions() {
    use carla::actor::TrafficLightState;
    use carla_sys::TrafficLightState as CxxState;

    // Test from_cxx
    assert_eq!(
        TrafficLightState::from_cxx(CxxState::Red),
        TrafficLightState::Red
    );
    assert_eq!(
        TrafficLightState::from_cxx(CxxState::Yellow),
        TrafficLightState::Yellow
    );
    assert_eq!(
        TrafficLightState::from_cxx(CxxState::Green),
        TrafficLightState::Green
    );
    assert_eq!(
        TrafficLightState::from_cxx(CxxState::Off),
        TrafficLightState::Off
    );
    assert_eq!(
        TrafficLightState::from_cxx(CxxState::Unknown),
        TrafficLightState::Unknown
    );

    // Test to_cxx
    assert_eq!(TrafficLightState::Red.to_cxx(), CxxState::Red);
    assert_eq!(TrafficLightState::Yellow.to_cxx(), CxxState::Yellow);
    assert_eq!(TrafficLightState::Green.to_cxx(), CxxState::Green);
    assert_eq!(TrafficLightState::Off.to_cxx(), CxxState::Off);
    assert_eq!(TrafficLightState::Unknown.to_cxx(), CxxState::Unknown);
}

/// Test that traffic light implements ActorT trait correctly
#[test]
#[cfg(feature = "test-carla-server")]
fn test_traffic_light_actor_trait() -> anyhow::Result<()> {
    let client = Client::new("localhost", 2000, None)?;
    let world = client.world()?;

    // Get all actors - this will fail with todo!() until implemented
    let actors = world.actors()?;
    let traffic_lights: Vec<TrafficLight> = actors
        .into_iter()
        .filter_map(|actor| TrafficLight::from_actor(actor).ok().flatten())
        .collect();

    if traffic_lights.is_empty() {
        eprintln!("No traffic lights found. Skipping test.");
        return Ok(());
    }

    let traffic_light = &traffic_lights[0];

    // Test ActorT methods
    let _id = traffic_light.id();
    let _type_id = traffic_light.type_id();
    let _transform = traffic_light.transform();
    let _velocity = traffic_light.velocity();
    let _angular_velocity = traffic_light.angular_velocity();
    let _acceleration = traffic_light.acceleration();
    let _is_alive = traffic_light.is_alive();
    let _bbox = traffic_light.bounding_box();

    println!("Traffic light ActorT trait methods work correctly");

    Ok(())
}
