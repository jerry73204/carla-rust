//! Test getting world from client

fn main() {
    println!("Creating CARLA client...");

    match carla::client::Client::new("localhost", 2000, None::<usize>) {
        Ok(client) => {
            println!("Client created successfully!");

            // Try to get server version first
            match client.server_version() {
                Ok(version) => {
                    println!("Server version: {}", version);
                }
                Err(e) => {
                    println!("Failed to get server version: {:?}", e);
                    return;
                }
            }

            // Now try to get world
            println!("Getting world...");
            match client.world() {
                Ok(world) => {
                    println!("Got world successfully!");
                    println!("World ID: {}", world.id());
                }
                Err(e) => {
                    println!("Failed to get world: {:?}", e);
                }
            }
        }
        Err(e) => {
            println!("Failed to create client: {:?}", e);
        }
    }
}
