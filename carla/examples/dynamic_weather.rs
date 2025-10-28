//! Dynamic Weather Example
//!
//! This example demonstrates:
//! - Weather parameter control
//! - Animating sun position over time
//! - Simulating weather cycles (clear, cloudy, rainy, stormy)
//!
//! Prerequisites:
//! - CARLA simulator must be running
//!
//! Run with:
//! ```bash
//! cargo run --example dynamic_weather
//! ```

use carla::{client::Client, rpc::WeatherParameters};
use std::{thread, time::Duration};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== CARLA Dynamic Weather Example ===\n");

    // Connect to CARLA
    println!("Connecting to CARLA simulator...");
    let client = Client::connect("localhost", 2000, None);
    let mut world = client.world();
    println!("✓ Connected! Current map: {}\n", world.map().name());

    // Get current weather
    let initial_weather = world.weather();
    println!("Initial weather:");
    print_weather(&initial_weather);
    println!();

    // Demo 1: Sun Animation
    println!("=== Demo 1: Sun Animation ===");
    println!("Animating sun position over 24-hour cycle...\n");

    for hour in 0..24 {
        let sun_altitude_angle = -30.0 + (hour as f32 * 5.0); // -30 to 85 degrees
        let sun_azimuth_angle = hour as f32 * 15.0; // 0 to 345 degrees

        let mut weather = world.weather();
        weather.sun_altitude_angle = sun_altitude_angle;
        weather.sun_azimuth_angle = sun_azimuth_angle;
        world.set_weather(&weather);

        println!(
            "Hour {:02}:00 - Altitude: {:.1}°, Azimuth: {:.1}°",
            hour, sun_altitude_angle, sun_azimuth_angle
        );

        thread::sleep(Duration::from_millis(500));
    }

    println!("\n✓ Sun animation complete\n");

    // Demo 2: Weather Cycle Simulation
    println!("=== Demo 2: Weather Cycle Simulation ===");
    println!("Cycling through different weather conditions...\n");

    // Define weather presets
    let weather_presets = vec![
        (
            "Clear Day",
            WeatherParameters {
                cloudiness: 10.0,
                precipitation: 0.0,
                precipitation_deposits: 0.0,
                wind_intensity: 5.0,
                sun_azimuth_angle: 0.0,
                sun_altitude_angle: 75.0,
                fog_density: 0.0,
                fog_distance: 0.0,
                wetness: 0.0,
                fog_falloff: 0.0,
                scattering_intensity: 0.0,
                mie_scattering_scale: 0.0,
                rayleigh_scattering_scale: 0.331,
                dust_storm: 0.0,
            },
        ),
        (
            "Partly Cloudy",
            WeatherParameters {
                cloudiness: 50.0,
                precipitation: 0.0,
                precipitation_deposits: 0.0,
                wind_intensity: 15.0,
                sun_azimuth_angle: 0.0,
                sun_altitude_angle: 75.0,
                fog_density: 0.0,
                fog_distance: 0.0,
                wetness: 0.0,
                fog_falloff: 0.0,
                scattering_intensity: 0.0,
                mie_scattering_scale: 0.0,
                rayleigh_scattering_scale: 0.331,
                dust_storm: 0.0,
            },
        ),
        (
            "Overcast",
            WeatherParameters {
                cloudiness: 90.0,
                precipitation: 0.0,
                precipitation_deposits: 0.0,
                wind_intensity: 25.0,
                sun_azimuth_angle: 0.0,
                sun_altitude_angle: 45.0,
                fog_density: 10.0,
                fog_distance: 50.0,
                wetness: 0.0,
                fog_falloff: 2.0,
                scattering_intensity: 0.0,
                mie_scattering_scale: 0.0,
                rayleigh_scattering_scale: 0.331,
                dust_storm: 0.0,
            },
        ),
        (
            "Light Rain",
            WeatherParameters {
                cloudiness: 80.0,
                precipitation: 30.0,
                precipitation_deposits: 20.0,
                wind_intensity: 30.0,
                sun_azimuth_angle: 0.0,
                sun_altitude_angle: 45.0,
                fog_density: 15.0,
                fog_distance: 50.0,
                wetness: 50.0,
                fog_falloff: 2.0,
                scattering_intensity: 0.0,
                mie_scattering_scale: 0.0,
                rayleigh_scattering_scale: 0.331,
                dust_storm: 0.0,
            },
        ),
        (
            "Heavy Rain",
            WeatherParameters {
                cloudiness: 100.0,
                precipitation: 80.0,
                precipitation_deposits: 70.0,
                wind_intensity: 50.0,
                sun_azimuth_angle: 0.0,
                sun_altitude_angle: 30.0,
                fog_density: 30.0,
                fog_distance: 30.0,
                wetness: 100.0,
                fog_falloff: 1.0,
                scattering_intensity: 0.0,
                mie_scattering_scale: 0.0,
                rayleigh_scattering_scale: 0.331,
                dust_storm: 0.0,
            },
        ),
        (
            "Storm",
            WeatherParameters {
                cloudiness: 100.0,
                precipitation: 100.0,
                precipitation_deposits: 90.0,
                wind_intensity: 100.0,
                sun_azimuth_angle: 0.0,
                sun_altitude_angle: 15.0,
                fog_density: 50.0,
                fog_distance: 20.0,
                wetness: 100.0,
                fog_falloff: 0.5,
                scattering_intensity: 0.0,
                mie_scattering_scale: 0.0,
                rayleigh_scattering_scale: 0.331,
                dust_storm: 0.0,
            },
        ),
    ];

    for (name, weather) in weather_presets.iter() {
        println!("--- {} ---", name);
        print_weather(weather);
        world.set_weather(weather);
        println!("Displaying for 5 seconds...\n");
        thread::sleep(Duration::from_secs(5));
    }

    println!("✓ Weather cycle complete\n");

    // Demo 3: Smooth Transitions
    println!("=== Demo 3: Smooth Weather Transitions ===");
    println!("Smoothly transitioning from clear to stormy...\n");

    let steps = 50;
    for i in 0..=steps {
        let t = i as f32 / steps as f32; // 0.0 to 1.0

        let mut weather = world.weather();
        weather.cloudiness = lerp(10.0, 100.0, t);
        weather.precipitation = lerp(0.0, 100.0, t);
        weather.wind_intensity = lerp(5.0, 100.0, t);
        weather.fog_density = lerp(0.0, 50.0, t);
        weather.wetness = lerp(0.0, 100.0, t);
        weather.sun_altitude_angle = lerp(75.0, 15.0, t);

        world.set_weather(&weather);

        if i % 10 == 0 {
            println!(
                "Progress: {:3}% - Cloudiness: {:.1}, Precipitation: {:.1}",
                (t * 100.0) as u32,
                weather.cloudiness,
                weather.precipitation
            );
        }

        thread::sleep(Duration::from_millis(100));
    }

    println!("\n✓ Smooth transition complete\n");

    // Restore initial weather
    println!("Restoring initial weather...");
    world.set_weather(&initial_weather);
    println!("✓ Weather restored");

    println!("\n=== Dynamic Weather Demo Complete ===");

    Ok(())
}

/// Linear interpolation between two values
fn lerp(start: f32, end: f32, t: f32) -> f32 {
    start + (end - start) * t
}

/// Print weather parameters in a readable format
fn print_weather(weather: &WeatherParameters) {
    println!("  Cloudiness: {:.1}%", weather.cloudiness);
    println!("  Precipitation: {:.1}%", weather.precipitation);
    println!("  Wind Intensity: {:.1}%", weather.wind_intensity);
    println!("  Sun Altitude: {:.1}°", weather.sun_altitude_angle);
    println!("  Sun Azimuth: {:.1}°", weather.sun_azimuth_angle);
    println!("  Fog Density: {:.1}%", weather.fog_density);
    println!("  Wetness: {:.1}%", weather.wetness);
}
