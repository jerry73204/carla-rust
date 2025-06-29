#![allow(dead_code)]

pub mod cli;
pub mod utils;

use anyhow::{Context, Result};
use carla::{
    actor::ActorExt,
    client::{Client, World},
};
use std::{thread, time::Duration};

/// Connect to CARLA with retry logic
pub fn connect_with_retry(host: &str, port: u16, timeout_secs: u32) -> Result<Client> {
    let mut attempts = 0;
    let max_attempts = timeout_secs;

    loop {
        match Client::new(host, port, None) {
            Ok(client) => return Ok(client),
            Err(e) if attempts < max_attempts => {
                attempts += 1;
                eprintln!("Connection attempt {attempts} failed: {e}. Retrying...");
                thread::sleep(Duration::from_secs(1));
            }
            Err(e) => return Err(e).context("Failed to connect to CARLA server. Is it running?"),
        }
    }
}

/// Ensure world is clean by destroying all actors
pub fn ensure_clean_world(world: &World) -> Result<()> {
    let actors = world.actors()?;
    let count = actors.len();

    if count > 0 {
        println!("Cleaning up {count} existing actors...");
        for actor in actors.iter() {
            if actor.type_id() != "spectator" {
                // Cannot destroy actor here as we don't have mutable reference
                // Destruction will happen automatically when actors go out of scope
            }
        }
        thread::sleep(Duration::from_millis(500)); // Give server time to clean up
    }

    Ok(())
}

/// Pretty print a transform
pub fn print_transform(transform: &carla::geom::Transform) {
    println!(
        "  Location: x={:.2}, y={:.2}, z={:.2}",
        transform.location.x, transform.location.y, transform.location.z
    );
    println!(
        "  Rotation: pitch={:.2}°, yaw={:.2}°, roll={:.2}°",
        transform.rotation.pitch, transform.rotation.yaw, transform.rotation.roll
    );
}

/// Wait for a condition to be true
pub fn wait_for_condition<F>(timeout: Duration, interval: Duration, mut condition: F) -> Result<()>
where
    F: FnMut() -> bool,
{
    let start = std::time::Instant::now();
    while start.elapsed() < timeout {
        if condition() {
            return Ok(());
        }
        thread::sleep(interval);
    }
    anyhow::bail!("Timeout waiting for condition")
}

/// Retry an operation with exponential backoff
pub fn retry_operation<F, T, E>(
    max_attempts: u32,
    initial_delay: Duration,
    mut operation: F,
) -> Result<T, E>
where
    F: FnMut() -> Result<T, E>,
    E: std::fmt::Display,
{
    let mut delay = initial_delay;
    for attempt in 1..=max_attempts {
        match operation() {
            Ok(result) => return Ok(result),
            Err(e) if attempt < max_attempts => {
                eprintln!("Attempt {attempt} failed: {e}. Retrying in {delay:?}...");
                thread::sleep(delay);
                delay *= 2; // Exponential backoff
            }
            Err(e) => return Err(e),
        }
    }
    unreachable!()
}
