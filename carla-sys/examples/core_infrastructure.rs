use carla_sys::{
    ActorBlueprintExt, ClientWrapper, SimpleLocation, SimpleRotation, SimpleTransform,
};

fn main() -> anyhow::Result<()> {
    println!("Testing carla-sys Core Client Infrastructure...");

    // Try to connect to CARLA server
    match ClientWrapper::new("localhost", 2000) {
        Ok(client) => {
            println!("✓ Connected to CARLA server!");

            // Get server version
            let version = client.get_server_version();
            println!("✓ Server version: {}", version);

            // Get world
            let world = client.get_world()?;
            println!("✓ Got world with ID: {}", world.get_id());

            // Get blueprint library
            let library = world.get_blueprint_library()?;
            println!("✓ Blueprint library size: {}", library.size());

            // Find a vehicle blueprint
            if let Some(vehicle_bp) = library.find("vehicle.tesla.model3") {
                println!("✓ Found Tesla Model 3 blueprint");
                println!("  - ID: {}", vehicle_bp.get_id());
                println!("  - Tags: {:?}", vehicle_bp.get_tags());

                // TODO: Set attributes on blueprints requires a mutable reference
                // which is not available through SharedPtr. Need to work around this.
                if vehicle_bp.contains_attribute("color") {
                    println!("  - Has color attribute");
                }

                // Get spectator
                if let Some(spectator) = world.get_spectator() {
                    println!("✓ Got spectator actor");
                    println!("  - ID: {}", spectator.get_id());
                    println!("  - Type: {}", spectator.get_type_id());
                    println!("  - Location: {:?}", spectator.get_location());

                    // Try to spawn a vehicle near the spectator
                    let spawn_transform = SimpleTransform {
                        location: SimpleLocation {
                            x: spectator.get_location().x + 10.0,
                            y: spectator.get_location().y,
                            z: spectator.get_location().z + 0.5,
                        },
                        rotation: SimpleRotation {
                            pitch: 0.0,
                            yaw: 0.0,
                            roll: 0.0,
                        },
                    };

                    match world.spawn_actor(&vehicle_bp, &spawn_transform, None) {
                        Ok(vehicle) => {
                            println!("✓ Spawned vehicle!");
                            println!("  - ID: {}", vehicle.get_id());
                            println!("  - Type: {}", vehicle.get_type_id());
                            println!("  - Display ID: {}", vehicle.get_display_id());
                            println!("  - Alive: {}", vehicle.is_alive());
                        }
                        Err(e) => {
                            println!("✗ Failed to spawn vehicle: {}", e);
                        }
                    }
                } else {
                    println!("✗ No spectator found");
                }

                // Tick the world
                let frame_id = world.tick(std::time::Duration::from_secs(1));
                println!("✓ World ticked to frame: {}", frame_id);
            } else {
                println!("✗ Tesla Model 3 blueprint not found");
                println!(
                    "  Blueprint library contains {} blueprints total",
                    library.size()
                );

                // Try another common vehicle
                if let Some(_vehicle_bp) = library.find("vehicle.audi.a2") {
                    println!("  Found Audi A2 blueprint instead");
                }
            }
        }
        Err(e) => {
            println!("✗ Failed to connect to CARLA server: {}", e);
            println!("Make sure CARLA simulator is running on localhost:2000");
        }
    }

    Ok(())
}
