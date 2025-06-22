//! Advanced world features demonstration for carla-sys.
//!
//! This example shows how to use advanced world features including:
//! - Map layer management for performance optimization
//! - Environment object queries and manipulation
//! - Texture and material application
//! - Advanced traffic light control

use anyhow::Result;
use carla_sys::{
    texture_utils, AdvancedWorldExt, CityObjectLabel, ClientWrapper, MapLayers, MaterialParameter,
    TextureColorBuilder,
};
use std::time::Duration;

fn main() -> Result<()> {
    println!("🌍 CARLA Advanced World Features Demo");
    println!("=====================================");

    // Connect to CARLA server
    println!("\n🔌 Connecting to CARLA server...");
    let mut client = ClientWrapper::new("localhost", 2000)?;
    println!(
        "✅ Connected! Server version: {}",
        client.get_server_version()
    );

    client.set_timeout(Duration::from_secs(10));

    let world = client.get_world()?;
    println!("🌍 World ID: {}", world.get_id());

    println!("\n🚀 Starting advanced world features demonstrations...");

    // === DEMO 1: Map Layer Management ===
    demo_map_layers(&world)?;
    std::thread::sleep(Duration::from_secs(2));

    // === DEMO 2: Environment Object Queries ===
    demo_environment_objects(&world)?;
    std::thread::sleep(Duration::from_secs(2));

    // === DEMO 3: Traffic Light Management ===
    demo_traffic_lights(&world)?;
    std::thread::sleep(Duration::from_secs(2));

    // === DEMO 4: Texture Application ===
    demo_textures(&world)?;
    std::thread::sleep(Duration::from_secs(2));

    // === DEMO 5: Performance Optimization ===
    demo_performance_optimization(&world)?;

    println!("\n✅ Advanced world features demonstration completed!");

    Ok(())
}

/// Demonstrate map layer management
fn demo_map_layers(world: &carla_sys::WorldWrapper) -> Result<()> {
    println!("\n📊 Demo 1: Map Layer Management");
    println!("-------------------------------");

    // Unload non-essential layers for better performance
    println!("🔧 Unloading decorative map layers for performance...");
    let decorative_layers = MapLayers::DECALS
        .union(MapLayers::PARTICLES)
        .union(MapLayers::FOLIAGE);

    world.unload_map_layers(decorative_layers);
    println!("✅ Unloaded: Decals, Particles, and Foliage");

    std::thread::sleep(Duration::from_secs(3));

    // Load specific layers
    println!("\n🔧 Loading only essential layers...");
    let essential_layers = MapLayers::BUILDINGS
        .union(MapLayers::ROADS)
        .union(MapLayers::SIDEWALKS)
        .union(MapLayers::TRAFFIC_SIGNS);

    world.load_map_layers(essential_layers);
    println!("✅ Loaded: Buildings, Roads, Sidewalks, and Traffic Signs");

    std::thread::sleep(Duration::from_secs(3));

    // Restore all layers
    println!("\n🔧 Restoring all map layers...");
    world.load_map_layers(MapLayers::ALL);
    println!("✅ All map layers restored");

    Ok(())
}

/// Demonstrate environment object queries
fn demo_environment_objects(world: &carla_sys::WorldWrapper) -> Result<()> {
    println!("\n🏗️ Demo 2: Environment Object Queries");
    println!("-------------------------------------");

    // Query building bounding boxes
    println!("🏢 Querying building bounding boxes...");
    let building_bbs = world.get_level_bounding_boxes(CityObjectLabel::Buildings);
    println!("✅ Found {} building bounding boxes", building_bbs.len());

    if !building_bbs.is_empty() {
        let bb = &building_bbs[0];
        println!(
            "   First building: Origin({:.2}, {:.2}, {:.2}), Extent({:.2}, {:.2}, {:.2})",
            bb.origin.x, bb.origin.y, bb.origin.z, bb.extent.x, bb.extent.y, bb.extent.z
        );
    }

    // Query traffic sign objects
    println!("\n🚦 Querying traffic sign objects...");
    let traffic_signs = world.query_environment_objects(CityObjectLabel::TrafficSigns);
    println!("✅ Found {} traffic sign objects", traffic_signs.len());

    for (i, sign) in traffic_signs.iter().take(3).enumerate() {
        println!(
            "   Sign {}: ID={}, Name='{}', Location({:.2}, {:.2}, {:.2})",
            i + 1,
            sign.id,
            sign.name,
            sign.transform.location.x,
            sign.transform.location.y,
            sign.transform.location.z
        );
    }

    // Query vegetation
    println!("\n🌳 Querying vegetation objects...");
    let vegetation = world.query_environment_objects(CityObjectLabel::Vegetation);
    println!("✅ Found {} vegetation objects", vegetation.len());

    // Disable some vegetation for performance
    if vegetation.len() > 10 {
        println!("\n🔧 Disabling 10 vegetation objects for performance...");
        let ids_to_disable: Vec<u64> = vegetation.iter().take(10).map(|v| v.id).collect();
        world.set_environment_objects_enabled(&ids_to_disable, false);
        println!("✅ Disabled {} vegetation objects", ids_to_disable.len());

        std::thread::sleep(Duration::from_secs(3));

        // Re-enable them
        println!("🔧 Re-enabling vegetation objects...");
        world.set_environment_objects_enabled(&ids_to_disable, true);
        println!("✅ Re-enabled vegetation objects");
    }

    Ok(())
}

/// Demonstrate advanced traffic light management
fn demo_traffic_lights(world: &carla_sys::WorldWrapper) -> Result<()> {
    println!("\n🚦 Demo 3: Traffic Light Management");
    println!("-----------------------------------");

    // Freeze all traffic lights
    println!("❄️  Freezing all traffic lights...");
    world.freeze_all_traffic_lights(true);
    println!("✅ All traffic lights frozen");

    std::thread::sleep(Duration::from_secs(3));

    // Unfreeze traffic lights
    println!("\n🔥 Unfreezing traffic lights...");
    world.freeze_all_traffic_lights(false);
    println!("✅ Traffic lights unfrozen");

    std::thread::sleep(Duration::from_secs(2));

    // Reset all traffic lights
    println!("\n🔄 Resetting all traffic lights to initial state...");
    world.reset_all_traffic_lights();
    println!("✅ Traffic lights reset");

    // Get vehicle light states
    println!("\n💡 Querying vehicle light states...");
    let vehicle_lights = world.get_all_vehicle_light_states();
    println!(
        "✅ Found {} vehicles with light states",
        vehicle_lights.len()
    );

    for (actor_id, light_state) in vehicle_lights.iter().take(5) {
        println!(
            "   Vehicle {}: Light state = {:#010b}",
            actor_id, light_state
        );
    }

    Ok(())
}

/// Demonstrate texture and material application
fn demo_textures(world: &carla_sys::WorldWrapper) -> Result<()> {
    println!("\n🎨 Demo 4: Texture and Material Application");
    println!("------------------------------------------");

    // Get texturable objects
    println!("🔍 Getting list of texturable objects...");
    let object_names = world.get_texturable_object_names();
    println!("✅ Found {} texturable objects", object_names.len());

    if object_names.is_empty() {
        println!("⚠️  No texturable objects found in this map");
        return Ok(());
    }

    // Show first few object names
    for (i, name) in object_names.iter().take(5).enumerate() {
        println!("   {}: {}", i + 1, name);
    }

    // Apply textures to first object
    if let Some(first_object) = object_names.first() {
        println!("\n🎨 Applying textures to '{}'...", first_object);

        // Create a red diffuse texture
        println!("🔴 Creating red diffuse texture...");
        let red_texture = texture_utils::solid_color(256, 256, 255, 0, 0);
        world.apply_color_texture(first_object, &red_texture, MaterialParameter::Diffuse);
        println!("✅ Applied red diffuse texture");

        std::thread::sleep(Duration::from_secs(2));

        // Create a checkerboard texture
        println!("\n🏁 Creating checkerboard texture...");
        let checker_texture = texture_utils::checkerboard(
            256,
            256,
            32,
            (255, 255, 255), // White
            (0, 0, 0),       // Black
        );
        world.apply_color_texture(first_object, &checker_texture, MaterialParameter::Diffuse);
        println!("✅ Applied checkerboard texture");

        std::thread::sleep(Duration::from_secs(2));

        // Create a gradient texture
        println!("\n🌈 Creating gradient texture...");
        let gradient_texture = texture_utils::gradient(
            256,
            256,
            (255, 0, 0), // Red
            (0, 0, 255), // Blue
            true,        // Horizontal
        );
        world.apply_color_texture(first_object, &gradient_texture, MaterialParameter::Diffuse);
        println!("✅ Applied gradient texture");

        // Create custom emissive texture
        println!("\n✨ Creating custom emissive texture...");
        let mut emissive_builder = TextureColorBuilder::new(128, 128);

        // Create a glowing pattern
        for y in 0..128 {
            for x in 0..128 {
                let intensity = ((x as f32 / 128.0 * std::f32::consts::PI * 4.0).sin()
                    * (y as f32 / 128.0 * std::f32::consts::PI * 4.0).sin()
                    * 255.0)
                    .abs() as u8;
                emissive_builder.set_pixel(x, y, intensity, intensity / 2, 0, 255);
            }
        }

        let emissive_texture = emissive_builder.build();
        world.apply_color_texture(first_object, &emissive_texture, MaterialParameter::Emissive);
        println!("✅ Applied custom emissive texture");
    }

    Ok(())
}

/// Demonstrate performance optimization techniques
fn demo_performance_optimization(world: &carla_sys::WorldWrapper) -> Result<()> {
    println!("\n⚡ Demo 5: Performance Optimization");
    println!("-----------------------------------");

    // Set pedestrian navigation seed for consistent behavior
    println!("🚶 Setting pedestrian navigation seed...");
    world.set_pedestrian_navigation_seed(42);
    println!("✅ Pedestrian navigation seed set to 42");

    // Optimize by unloading heavy layers
    println!("\n🏃 Optimizing for high performance mode...");
    let heavy_layers = MapLayers::FOLIAGE
        .union(MapLayers::PARTICLES)
        .union(MapLayers::DECALS)
        .union(MapLayers::PARKED_VEHICLES);

    world.unload_map_layers(heavy_layers);
    println!("✅ Unloaded heavy map layers");

    // Query and disable non-essential objects
    println!("\n🔧 Disabling non-essential environment objects...");

    let props = world.query_environment_objects(CityObjectLabel::Other);
    if !props.is_empty() {
        let prop_ids: Vec<u64> = props.iter().take(20).map(|p| p.id).collect();
        world.set_environment_objects_enabled(&prop_ids, false);
        println!("✅ Disabled {} prop objects", prop_ids.len());
    }

    println!("\n💡 Performance optimization tips:");
    println!("   - Unload unnecessary map layers");
    println!("   - Disable non-essential environment objects");
    println!("   - Freeze traffic lights when not needed");
    println!("   - Use pedestrian seed for deterministic behavior");
    println!("   - Apply simple textures to reduce memory usage");

    Ok(())
}
