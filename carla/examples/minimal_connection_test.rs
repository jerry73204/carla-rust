//! Minimal test to debug connection issues

fn main() {
    println!("Creating CARLA client...");

    match carla::client::Client::new("localhost", 2000, None::<usize>) {
        Ok(client) => {
            println!("Client created successfully!");

            // Try to get server version
            match client.server_version() {
                Ok(version) => {
                    println!("Server version: {}", version);
                }
                Err(e) => {
                    println!("Failed to get server version: {:?}", e);
                }
            }
        }
        Err(e) => {
            println!("Failed to create client: {:?}", e);
        }
    }
}
