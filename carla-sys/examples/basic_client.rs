use carla_sys::ClientWrapper;

fn main() -> anyhow::Result<()> {
    println!("Testing carla-sys framework...");

    // Try to connect to CARLA server (this will fail if server is not running)
    match ClientWrapper::new("localhost", 2000) {
        Ok(mut client) => {
            println!("Connected to CARLA server!");

            let version = client.get_server_version();
            println!("Server version: {version}");

            // Set timeout to 10 seconds
            client.set_timeout(std::time::Duration::from_secs(10));

            // Get timeout
            let timeout = client.get_timeout();
            println!("Timeout: {timeout:?}");
        }
        Err(e) => {
            println!("Failed to connect to CARLA server: {e}");
            println!("Make sure CARLA simulator is running on localhost:2000");
        }
    }

    Ok(())
}
