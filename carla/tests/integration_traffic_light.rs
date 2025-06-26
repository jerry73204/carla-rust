//! Integration tests for traffic light functionality

use carla::actor::{ActorExt, TrafficLight, TrafficLightState};
use carla_test_server::with_carla_server;
use std::time::Duration;

/// Test that requires a running CARLA server
#[with_carla_server]
fn test_traffic_light_basic_operations(client: &carla::client::Client) {
    let world = client.world().expect("Failed to get world");

    // Get all actors in the world
    // This will fail with todo!() until actor list iteration is implemented
    let actors = world.actors().expect("Failed to get actors");

    // Find traffic lights by checking actor type
    let traffic_lights: Vec<TrafficLight> = actors
        .iter()
        .filter_map(|actor| TrafficLight::from_actor(actor).ok())
        .collect();

    if traffic_lights.is_empty() {
        eprintln!("No traffic lights found in the current map. Skipping test.");
        return;
    }

    let traffic_light = &traffic_lights[0];
    println!("Found traffic light with ID: {}", traffic_light.id());

    // Test basic state operations
    let initial_state = traffic_light.state();
    println!("Initial state: {:?}", initial_state);

    // Set to red
    traffic_light
        .set_state(TrafficLightState::Red)
        .expect("Failed to set state to red");
    assert_eq!(traffic_light.state(), TrafficLightState::Red);

    // Set to green
    traffic_light
        .set_state(TrafficLightState::Green)
        .expect("Failed to set state to green");
    assert_eq!(traffic_light.state(), TrafficLightState::Green);

    // Test time settings
    traffic_light
        .set_green_time(Duration::from_secs(10))
        .expect("Failed to set green time");
    traffic_light
        .set_yellow_time(Duration::from_secs(3))
        .expect("Failed to set yellow time");
    traffic_light
        .set_red_time(Duration::from_secs(15))
        .expect("Failed to set red time");

    assert_eq!(traffic_light.green_time(), Duration::from_secs(10));
    assert_eq!(traffic_light.yellow_time(), Duration::from_secs(3));
    assert_eq!(traffic_light.red_time(), Duration::from_secs(15));

    // Test freeze functionality
    traffic_light
        .freeze(true)
        .expect("Failed to freeze traffic light");
    assert!(traffic_light.is_frozen());

    traffic_light
        .freeze(false)
        .expect("Failed to unfreeze traffic light");
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
    traffic_light
        .set_state(initial_state)
        .expect("Failed to restore initial state");
}

/// Test that traffic light implements ActorT trait correctly
#[with_carla_server]
fn test_traffic_light_actor_trait(client: &carla::client::Client) {
    let world = client.world().expect("Failed to get world");

    // Get all actors - this will fail with todo!() until implemented
    let actors = world.actors().expect("Failed to get actors");
    let traffic_lights: Vec<TrafficLight> = actors
        .iter()
        .filter_map(|actor| TrafficLight::from_actor(actor).ok())
        .collect();

    if traffic_lights.is_empty() {
        eprintln!("No traffic lights found. Skipping test.");
        return;
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
}
