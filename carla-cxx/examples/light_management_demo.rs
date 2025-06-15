//! Light Management Demonstration
//!
//! This example demonstrates how to use CARLA's light management system
//! to control lights in the simulation world.

use anyhow::Result;
use carla_cxx::{ClientWrapper, LightGroup, LightState};

fn main() -> Result<()> {
    println!("🔆 CARLA Light Management Demo");
    println!("===============================");

    // Connect to CARLA server
    let client = ClientWrapper::new("localhost", 2000)?;
    println!("✅ Connected to CARLA server");

    let world = client.get_world();
    println!("🌍 Retrieved world instance");

    // Get the light manager
    let light_manager = world.get_light_manager();
    println!("💡 Retrieved light manager");

    // Get all lights in the world
    let all_lights = light_manager.get_all_lights(LightGroup::None);
    println!("🔍 Found {} lights in the world", all_lights.len());

    // Show light groups breakdown
    let street_lights = light_manager.get_all_lights(LightGroup::Street);
    let building_lights = light_manager.get_all_lights(LightGroup::Building);
    let vehicle_lights = light_manager.get_all_lights(LightGroup::Vehicle);
    let other_lights = light_manager.get_all_lights(LightGroup::Other);

    println!("  🛣️  Street lights: {}", street_lights.len());
    println!("  🏢 Building lights: {}", building_lights.len());
    println!("  🚗 Vehicle lights: {}", vehicle_lights.len());
    println!("  ❓ Other lights: {}", other_lights.len());

    // Demonstrate light control operations
    if !all_lights.is_empty() {
        println!("\n🎮 Demonstrating light control operations...");

        // Get IDs of first few lights for demonstration
        let demo_light_ids: Vec<u32> = all_lights
            .iter()
            .take(5.min(all_lights.len()))
            .map(|light| light.id)
            .collect();

        println!(
            "  🎯 Using {} lights for demo: {:?}",
            demo_light_ids.len(),
            demo_light_ids
        );

        // Turn off demo lights
        println!("  🔴 Turning off demo lights...");
        light_manager.turn_off_lights(&demo_light_ids);

        // Wait a moment (in a real application you might tick the world)
        std::thread::sleep(std::time::Duration::from_millis(500));

        // Turn them back on
        println!("  🟢 Turning demo lights back on...");
        light_manager.turn_on_lights(&demo_light_ids);

        // Change colors to red
        println!("  🔴 Setting demo lights to red...");
        light_manager.set_light_colors(&demo_light_ids, (255, 0, 0)); // Red

        // Change intensity
        println!("  🔆 Setting demo lights to high intensity...");
        light_manager.set_light_intensities(&demo_light_ids, 1000.0); // High intensity

        // Restore original state (white, normal intensity)
        println!("  ⚪ Restoring demo lights to white with normal intensity...");
        let normal_state = LightState {
            intensity: 100.0,
            color: (255, 255, 255), // White
            group: LightGroup::None,
            active: true,
        };
        light_manager.set_light_states(&demo_light_ids, normal_state);
    }

    // Demonstrate day/night cycle
    println!("\n🌅 Demonstrating day/night cycle...");
    println!("  🌙 Enabling day/night cycle (lights will be controlled automatically)");
    light_manager.set_day_night_cycle(true);

    // Show some light details
    if let Some(first_light) = all_lights.first() {
        println!("\n📋 Example light details:");
        println!("  ID: {}", first_light.id);
        println!(
            "  Location: ({:.1}, {:.1}, {:.1})",
            first_light.location.0, first_light.location.1, first_light.location.2
        );
        println!("  Intensity: {:.1} lumens", first_light.state.intensity);
        println!(
            "  Color: RGB({}, {}, {})",
            first_light.state.color.0, first_light.state.color.1, first_light.state.color.2
        );
        println!("  Group: {:?}", first_light.state.group);
        println!("  Active: {}", first_light.state.active);
    }

    println!("\n✨ Light management demo completed!");
    println!("💡 The light management system allows you to:");
    println!("   • Get all lights or filter by group (Street, Building, Vehicle, Other)");
    println!("   • Control multiple lights efficiently with bulk operations");
    println!("   • Turn lights on/off, change colors, set intensity");
    println!("   • Set complete light states in one operation");
    println!("   • Enable automatic day/night cycle control");

    Ok(())
}
