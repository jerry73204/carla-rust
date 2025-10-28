//! Basic CARLA simulator connection example
//!
//! This example demonstrates how to connect to a running CARLA simulator
//! and retrieve the server version.
//!
//! Prerequisites:
//! - CARLA simulator must be running (default: localhost:2000)
//!
//! Run with:
//! ```bash
//! cargo run --example connect
//! ```

use carla::client::Client;

fn main() {
    println!("Connecting to CARLA simulator...");

    // Connect to CARLA simulator (default: localhost:2000)
    let client = Client::connect("localhost", 2000, None);

    // Get and display server version
    let version = client.server_version();
    println!("Successfully connected!");
    println!("CARLA Server version: {}", version);
}
