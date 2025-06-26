//! Test vehicle casting

use carla::actor::ActorExt;

fn main() {
    println!("Creating CARLA client...");

    match carla::client::Client::new("localhost", 2000, None::<usize>) {
        Ok(client) => {
            println!("Client created successfully!");

            // Get world
            let world = match client.world() {
                Ok(w) => w,
                Err(e) => {
                    println!("Failed to get world: {:?}", e);
                    return;
                }
            };

            println!("Got world successfully!");

            // Get blueprint library
            let blueprint_library = match world.blueprint_library() {
                Ok(bl) => bl,
                Err(e) => {
                    println!("Failed to get blueprint library: {:?}", e);
                    return;
                }
            };

            // Find a vehicle blueprint
            let vehicle_bp = match blueprint_library.find("vehicle.dodge.charger") {
                Ok(Some(bp)) => bp,
                Ok(None) => {
                    println!("Vehicle blueprint not found");
                    return;
                }
                Err(e) => {
                    println!("Failed to find blueprint: {:?}", e);
                    return;
                }
            };

            println!("Found vehicle blueprint: {}", vehicle_bp.id());

            // Get spawn points
            let spawn_points = match world.map() {
                Ok(map) => map.spawn_points(),
                Err(e) => {
                    println!("Failed to get map: {:?}", e);
                    return;
                }
            };

            let spawn_point = spawn_points
                .get(0)
                .unwrap_or_else(carla::geom::Transform::default);

            println!("Spawning vehicle at: {:?}", spawn_point);

            // Spawn the actor
            let actor = match world.try_spawn_actor(&vehicle_bp, &spawn_point, None) {
                Ok(Some(a)) => a,
                Ok(None) => {
                    println!("Failed to spawn actor");
                    return;
                }
                Err(e) => {
                    println!("Error spawning actor: {:?}", e);
                    return;
                }
            };

            println!("Spawned actor with ID: {}", actor.id());
            println!("Actor type: {}", actor.type_id());

            // Try to cast to vehicle
            println!("Attempting to cast to vehicle...");
            match actor.into_vehicle() {
                Ok(vehicle) => {
                    println!("Successfully cast to vehicle!");
                    println!("Vehicle ID: {}", vehicle.id());
                    println!("Vehicle type: {}", vehicle.type_id());
                    println!("Vehicle transform: {:?}", vehicle.transform());
                }
                Err(actor) => {
                    println!("Failed to cast to vehicle, actor type: {}", actor.type_id());
                }
            }
        }
        Err(e) => {
            println!("Failed to create client: {:?}", e);
        }
    }
}
