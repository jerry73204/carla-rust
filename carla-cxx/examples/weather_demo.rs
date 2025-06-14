//! Weather control demo for carla-cxx
//!
//! This example demonstrates weather parameter control in CARLA simulator.
//! It shows how to get current weather, set different weather presets,
//! and create custom weather conditions.

use carla_cxx::{ClientWrapper, SimpleWeatherParameters};
use std::time::Duration;

fn main() -> anyhow::Result<()> {
    println!("🌤️  Weather Control Demo for carla-cxx");
    println!("=======================================");

    // Try to connect to CARLA server
    match ClientWrapper::new("localhost", 2000) {
        Ok(client) => {
            println!("✅ Connected to CARLA server");

            let world = client.get_world();

            // Check if weather is enabled
            if world.is_weather_enabled() {
                println!("🌦️  Weather simulation is enabled");
            } else {
                println!("⚠️  Weather simulation is disabled");
            }

            // Get current weather
            let current_weather = world.get_weather();
            println!("\n📊 Current weather parameters:");
            print_weather_info(&current_weather);

            // Demonstrate different weather presets
            println!("\n🌞 Setting Clear Noon weather...");
            let clear_noon = SimpleWeatherParameters::clear_noon();
            world.set_weather(&clear_noon);
            std::thread::sleep(Duration::from_secs(2));

            println!("🌧️  Setting Hard Rain weather...");
            let hard_rain = SimpleWeatherParameters::hard_rain_noon();
            world.set_weather(&hard_rain);
            std::thread::sleep(Duration::from_secs(2));

            println!("🌙 Setting Clear Night weather...");
            let clear_night = SimpleWeatherParameters::clear_night();
            world.set_weather(&clear_night);
            std::thread::sleep(Duration::from_secs(2));

            println!("🌪️  Setting Dust Storm weather...");
            let dust_storm = SimpleWeatherParameters::dust_storm();
            world.set_weather(&dust_storm);
            std::thread::sleep(Duration::from_secs(2));

            // Create custom weather
            println!("⛅ Setting custom weather (partly cloudy sunset)...");
            let custom_weather = SimpleWeatherParameters {
                cloudiness: 40.0,
                precipitation: 5.0,
                precipitation_deposits: 0.0,
                wind_intensity: 25.0,
                sun_azimuth_angle: 270.0, // West
                sun_altitude_angle: 15.0, // Sunset
                fog_density: 15.0,
                fog_distance: 50.0,
                fog_falloff: 0.8,
                wetness: 10.0,
                scattering_intensity: 1.2,
                mie_scattering_scale: 0.05,
                rayleigh_scattering_scale: 0.0331,
                dust_storm: 0.0,
            };
            world.set_weather(&custom_weather);

            println!("\n📊 Final weather parameters:");
            let final_weather = world.get_weather();
            print_weather_info(&final_weather);

            println!("\n✨ Weather demo completed successfully!");
        }
        Err(e) => {
            println!("❌ Failed to connect to CARLA server: {}", e);
            println!("💡 Make sure CARLA simulator is running on localhost:2000");
            println!("   Example: ./CarlaUE4.sh -windowed -ResX=800 -ResY=600");
            return Err(e);
        }
    }

    Ok(())
}

fn print_weather_info(weather: &SimpleWeatherParameters) {
    println!("   Cloudiness: {:.1}%", weather.cloudiness);
    println!("   Precipitation: {:.1}%", weather.precipitation);
    println!("   Wind Intensity: {:.1}%", weather.wind_intensity);
    println!(
        "   Sun Altitude: {:.1}° (45°=noon, 15°=sunset, -90°=night)",
        weather.sun_altitude_angle
    );
    println!("   Fog Density: {:.1}%", weather.fog_density);
    println!("   Wetness: {:.1}%", weather.wetness);
    if weather.dust_storm > 0.0 {
        println!("   Dust Storm: {:.1}%", weather.dust_storm);
    }
}
