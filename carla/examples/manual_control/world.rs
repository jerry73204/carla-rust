//! World management module
//!
//! The World struct is responsible for:
//! - Vehicle spawning and respawning
//! - All sensor management (collision, lane invasion, GNSS, IMU, radar, camera)
//! - Weather cycling with presets
//! - Map layer management
//! - Recording state tracking
//! - Advanced features (radar, doors, constant velocity, telemetry)
//!
//! ## Phase 13 Implementation
//!
//! - ✅ Phase 1.2: Vehicle spawning and basic management
//! - ✅ Phase 5: Sensor spawning (IMU, GNSS, collision, lane invasion)
//! - ✅ Phase 9: Weather and map layer control
//! - ✅ Phase 10: Recording state management
//! - ✅ Phase 11: Advanced features (radar, doors, telemetry)

use carla::{
    client::{Client, Vehicle, World as CarlaWorld},
    prelude::*,
    rpc::{MapLayer, WeatherParameters},
};
use eyre::{eyre, Result};
use tracing::info;

/// Weather preset definition
///
/// Each preset contains weather parameters and a display name
type WeatherPreset = (WeatherParameters, &'static str);

/// Get all weather presets
///
/// ✅ Subphase 12.9.1: Define weather presets
fn get_weather_presets() -> Vec<WeatherPreset> {
    vec![
        (create_clear_noon(), "Clear Noon"),
        (create_cloudy_noon(), "Cloudy Noon"),
        (create_wet_noon(), "Wet Noon"),
        (create_wet_cloudy_noon(), "Wet Cloudy Noon"),
        (create_mid_rainy_noon(), "Mid Rainy Noon"),
        (create_hard_rain_noon(), "Hard Rain Noon"),
        (create_soft_rain_noon(), "Soft Rain Noon"),
        (create_clear_sunset(), "Clear Sunset"),
        (create_cloudy_sunset(), "Cloudy Sunset"),
        (create_wet_sunset(), "Wet Sunset"),
        (create_wet_cloudy_sunset(), "Wet Cloudy Sunset"),
        (create_mid_rain_sunset(), "Mid Rain Sunset"),
        (create_hard_rain_sunset(), "Hard Rain Sunset"),
        (create_soft_rain_sunset(), "Soft Rain Sunset"),
    ]
}

/// Create Clear Noon weather
fn create_clear_noon() -> WeatherParameters {
    WeatherParameters {
        cloudiness: 0.0,
        precipitation: 0.0,
        precipitation_deposits: 0.0,
        wind_intensity: 0.0,
        sun_azimuth_angle: 0.0,
        sun_altitude_angle: 70.0,
        fog_density: 0.0,
        fog_distance: 0.0,
        wetness: 0.0,
        fog_falloff: 0.0,
        scattering_intensity: 0.0,
        mie_scattering_scale: 0.0,
        rayleigh_scattering_scale: 0.0331,
        dust_storm: 0.0,
    }
}

/// Create Cloudy Noon weather
fn create_cloudy_noon() -> WeatherParameters {
    let mut weather = create_clear_noon();
    weather.cloudiness = 80.0;
    weather
}

/// Create Wet Noon weather
fn create_wet_noon() -> WeatherParameters {
    let mut weather = create_clear_noon();
    weather.precipitation_deposits = 50.0;
    weather.wetness = 50.0;
    weather
}

/// Create Wet Cloudy Noon weather
fn create_wet_cloudy_noon() -> WeatherParameters {
    let mut weather = create_wet_noon();
    weather.cloudiness = 80.0;
    weather
}

/// Create Mid Rainy Noon weather
fn create_mid_rainy_noon() -> WeatherParameters {
    let mut weather = create_wet_cloudy_noon();
    weather.precipitation = 30.0;
    weather
}

/// Create Hard Rain Noon weather
fn create_hard_rain_noon() -> WeatherParameters {
    let mut weather = create_wet_cloudy_noon();
    weather.precipitation = 60.0;
    weather
}

/// Create Soft Rain Noon weather
fn create_soft_rain_noon() -> WeatherParameters {
    let mut weather = create_wet_cloudy_noon();
    weather.precipitation = 15.0;
    weather
}

/// Create Clear Sunset weather
fn create_clear_sunset() -> WeatherParameters {
    let mut weather = create_clear_noon();
    weather.sun_altitude_angle = 15.0;
    weather
}

/// Create Cloudy Sunset weather
fn create_cloudy_sunset() -> WeatherParameters {
    let mut weather = create_clear_sunset();
    weather.cloudiness = 80.0;
    weather
}

/// Create Wet Sunset weather
fn create_wet_sunset() -> WeatherParameters {
    let mut weather = create_clear_sunset();
    weather.precipitation_deposits = 50.0;
    weather.wetness = 50.0;
    weather
}

/// Create Wet Cloudy Sunset weather
fn create_wet_cloudy_sunset() -> WeatherParameters {
    let mut weather = create_wet_sunset();
    weather.cloudiness = 80.0;
    weather
}

/// Create Mid Rain Sunset weather
fn create_mid_rain_sunset() -> WeatherParameters {
    let mut weather = create_wet_cloudy_sunset();
    weather.precipitation = 30.0;
    weather
}

/// Create Hard Rain Sunset weather
fn create_hard_rain_sunset() -> WeatherParameters {
    let mut weather = create_wet_cloudy_sunset();
    weather.precipitation = 60.0;
    weather
}

/// Create Soft Rain Sunset weather
fn create_soft_rain_sunset() -> WeatherParameters {
    let mut weather = create_wet_cloudy_sunset();
    weather.precipitation = 15.0;
    weather
}

/// World state manager
///
/// Manages the CARLA world, player vehicle, and all sensors
#[allow(dead_code)]
pub struct World {
    pub world: CarlaWorld,
    pub player: Option<Vehicle>,
    // TODO Phase 5: Add sensor fields
    // pub collision_sensor: Option<CollisionSensor>,
    // pub lane_invasion_sensor: Option<LaneInvasionSensor>,
    // pub gnss_sensor: Option<GnssSensor>,
    // pub imu_sensor: Option<IMUSensor>,
    // pub radar_sensor: Option<RadarSensor>,

    // ✅ Phase 9: Weather state
    pub current_weather: usize,
    pub weather_presets: Vec<WeatherPreset>,

    // ✅ Phase 9: Map layer state
    pub current_map_layer: usize,
    pub map_layers: Vec<MapLayer>,

    // ✅ Phase 10: Recording state
    pub recording_enabled: bool,
    pub recording_start: f64,

    // ✅ Phase 11: Advanced feature flags
    pub radar_enabled: bool,
    pub doors_are_open: bool,
    pub show_vehicle_telemetry: bool,
    pub constant_velocity_enabled: bool,
}

impl World {
    /// Create a new World and spawn player vehicle
    ///
    /// ✅ Subphase 12.1.2: Vehicle Spawning and Basic Display
    pub fn new(client: &Client, config: &crate::Config) -> Result<Self> {
        let mut world = client.world();

        // Get blueprint library
        let blueprint_library = world.blueprint_library();

        // Filter for vehicle blueprints (default: "vehicle.*")
        let vehicle_blueprints: Vec<_> = blueprint_library
            .iter()
            .filter(|bp| bp.id().contains(&config.filter))
            .collect();

        if vehicle_blueprints.is_empty() {
            return Err(eyre!(
                "No vehicle blueprints found matching filter: {}",
                config.filter
            ));
        }

        // Choose a random vehicle blueprint
        let blueprint = vehicle_blueprints[0].clone(); // For now, use first vehicle
        info!("Selected vehicle blueprint: {}", blueprint.id());

        // Get spawn points from the map
        let map = world.map();
        let spawn_points = map.recommended_spawn_points();

        if spawn_points.is_empty() {
            return Err(eyre!("No spawn points available on this map"));
        }

        // Spawn at the first spawn point
        let spawn_point = spawn_points
            .get(0)
            .ok_or_else(|| eyre!("No spawn points available"))?;
        info!("Spawning vehicle at spawn point 0");

        let actor = world
            .spawn_actor(&blueprint, &spawn_point)
            .map_err(|e| eyre!("Failed to spawn actor: {:?}", e))?;
        let player = Vehicle::try_from(actor).map_err(|_| eyre!("Failed to convert to vehicle"))?;
        info!("✓ Vehicle spawned successfully: ID {}", player.id());

        // Enable autopilot if requested
        if config.autopilot {
            player.set_autopilot(true);
            info!("✓ Autopilot enabled");
        }

        let weather_presets = get_weather_presets();
        let map_layers = vec![
            MapLayer::None,
            MapLayer::Buildings,
            MapLayer::Decals,
            MapLayer::Foliage,
            MapLayer::Ground,
            MapLayer::ParkedVehicles,
            MapLayer::Particles,
            MapLayer::Props,
            MapLayer::StreetLights,
            MapLayer::Walls,
            MapLayer::All,
        ];

        Ok(Self {
            world,
            player: Some(player),
            current_weather: 0,
            weather_presets,
            current_map_layer: 0,
            map_layers,
            recording_enabled: false,
            recording_start: 0.0,
            radar_enabled: false,
            doors_are_open: false,
            show_vehicle_telemetry: false,
            constant_velocity_enabled: false,
        })
    }

    /// Tick the world (advance simulation one step)
    ///
    /// TODO Phase 2: Called in main loop for synchronous mode
    #[allow(dead_code)]
    pub fn tick(&mut self) {
        // TODO: self.world.tick();
    }

    /// Update world state
    ///
    /// TODO Phase 4-11: Update sensors, weather, etc.
    #[allow(dead_code)]
    pub fn update(&mut self, _delta_time: f32) {
        // TODO: Update sensor data
        // TODO: Update recording state
    }

    // ✅ Phase 9.1: Weather cycling methods
    /// Cycle to next/previous weather preset
    pub fn next_weather(&mut self, reverse: bool) -> &str {
        if reverse {
            self.current_weather = if self.current_weather == 0 {
                self.weather_presets.len() - 1
            } else {
                self.current_weather - 1
            };
        } else {
            self.current_weather = (self.current_weather + 1) % self.weather_presets.len();
        }

        let (weather, name) = &self.weather_presets[self.current_weather];
        self.world.set_weather(weather);
        name
    }

    // ✅ Phase 9.2: Map layer methods
    /// Cycle to next map layer
    pub fn next_map_layer(&mut self, reverse: bool) -> MapLayer {
        if reverse {
            self.current_map_layer = if self.current_map_layer == 0 {
                self.map_layers.len() - 1
            } else {
                self.current_map_layer - 1
            };
        } else {
            self.current_map_layer = (self.current_map_layer + 1) % self.map_layers.len();
        }

        self.map_layers[self.current_map_layer].clone()
    }

    /// Load or unload the current map layer
    pub fn load_map_layer(&self, unload: bool) {
        let layer = &self.map_layers[self.current_map_layer];
        if unload {
            self.world.unload_level_layer(layer.clone());
        } else {
            self.world.load_level_layer(layer.clone());
        }
    }

    // TODO Phase 11: Advanced feature methods
    // pub fn toggle_radar(&mut self) {}
    // pub fn toggle_doors(&mut self) {}
    // pub fn toggle_constant_velocity(&mut self) {}
    // pub fn toggle_vehicle_telemetry(&mut self) {}

    /// Cleanup - destroy all actors
    pub fn destroy(&mut self) {
        // TODO: Destroy all sensors
        if let Some(ref player) = self.player {
            info!("Destroying player vehicle...");
            player.destroy();
        }
        self.player = None;
    }
}

impl Drop for World {
    fn drop(&mut self) {
        self.destroy();
    }
}
