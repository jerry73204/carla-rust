//! Dynamic Weather GUI - Weather Transition Demo
//!
//! Demonstrates smooth weather parameter interpolation with real-time visual feedback.
//! Includes 8-hour day/night cycle and 14 weather presets.
//!
//! # Features
//! - Smooth weather parameter interpolation
//! - 8-hour day/night cycle simulation
//! - 14 weather presets (Clear, Cloudy, Wet, Rain, Storm, etc.)
//! - Real-time weather parameter display
//! - Time of day control
//!
//! # Controls
//! - **C**: Next weather preset
//! - **Shift+C**: Previous weather preset
//! - **T**: Toggle time progression
//! - **+/-**: Speed up/slow down time
//! - **1-9**: Jump to time of day
//! - **H**: Toggle help overlay
//! - **ESC/Q**: Quit
//!
//! # Usage
//! ```bash
//! cargo run --example dynamic_weather_gui --profile dev-release
//! ```

use anyhow::Result;
use carla::{
    client::{ActorBase, Client, Sensor, Vehicle, World as CarlaWorld},
    geom::Location,
    rpc::WeatherParameters,
    sensor::data::Image as CarlaImage,
};
use macroquad::prelude::*;
use std::sync::{Arc, Mutex};

const WINDOW_WIDTH: u32 = 1280;
const WINDOW_HEIGHT: u32 = 720;
const CAMERA_WIDTH: u32 = 1280;
const CAMERA_HEIGHT: u32 = 720;

fn window_conf() -> Conf {
    Conf {
        window_title: "CARLA - Dynamic Weather Demo".to_owned(),
        window_width: WINDOW_WIDTH as i32,
        window_height: WINDOW_HEIGHT as i32,
        window_resizable: false,
        ..Default::default()
    }
}

struct CameraManager {
    sensor: Sensor,
    texture: Texture2D,
    latest_image: Arc<Mutex<Option<CarlaImage>>>,
}

impl CameraManager {
    fn new(world: &mut CarlaWorld, vehicle: &Vehicle) -> Result<Self> {
        let blueprint_library = world.blueprint_library();
        let camera_bp = blueprint_library
            .find("sensor.camera.rgb")
            .ok_or_else(|| anyhow::anyhow!("Camera blueprint not found"))?;

        let mut camera_bp = camera_bp;
        let _ = camera_bp.set_attribute("image_size_x", &CAMERA_WIDTH.to_string());
        let _ = camera_bp.set_attribute("image_size_y", &CAMERA_HEIGHT.to_string());
        let _ = camera_bp.set_attribute("fov", "110");

        // Spawn camera behind vehicle
        let mut camera_transform = carla::geom::Transform {
            location: Location::new(0.0, 0.0, 0.0),
            rotation: carla::geom::Rotation::new(0.0, 0.0, 0.0),
        };
        camera_transform.location.x = -5.5;
        camera_transform.location.z = 2.5;

        let camera = world.spawn_actor_attached(
            &camera_bp,
            &camera_transform,
            vehicle,
            carla::rpc::AttachmentType::SpringArm,
        )?;

        let sensor =
            Sensor::try_from(camera).map_err(|_| anyhow::anyhow!("Failed to cast to Sensor"))?;

        let texture = Texture2D::from_rgba8(
            CAMERA_WIDTH as u16,
            CAMERA_HEIGHT as u16,
            &vec![0u8; (CAMERA_WIDTH * CAMERA_HEIGHT * 4) as usize],
        );
        texture.set_filter(FilterMode::Linear);

        let latest_image = Arc::new(Mutex::new(None));
        let latest_image_clone = Arc::clone(&latest_image);

        sensor.listen(move |data| {
            if let Ok(image) = CarlaImage::try_from(data) {
                if let Ok(mut img) = latest_image_clone.lock() {
                    *img = Some(image);
                }
            }
        });

        Ok(Self {
            sensor,
            texture,
            latest_image,
        })
    }

    fn update(&mut self) -> bool {
        if let Ok(mut img_opt) = self.latest_image.lock() {
            if let Some(image) = img_opt.take() {
                let raw_data = image.as_slice();
                let mut rgba_data = Vec::with_capacity((CAMERA_WIDTH * CAMERA_HEIGHT * 4) as usize);

                for color in raw_data {
                    rgba_data.push(color.r);
                    rgba_data.push(color.g);
                    rgba_data.push(color.b);
                    rgba_data.push(color.a);
                }

                self.texture =
                    Texture2D::from_rgba8(CAMERA_WIDTH as u16, CAMERA_HEIGHT as u16, &rgba_data);
                self.texture.set_filter(FilterMode::Linear);
                return true;
            }
        }
        false
    }

    fn render(&self) {
        draw_texture_ex(
            &self.texture,
            0.0,
            0.0,
            WHITE,
            DrawTextureParams {
                dest_size: Some(vec2(WINDOW_WIDTH as f32, WINDOW_HEIGHT as f32)),
                ..Default::default()
            },
        );
    }
}

struct WeatherPreset {
    name: &'static str,
}

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

fn create_cloudy_noon() -> WeatherParameters {
    let mut weather = create_clear_noon();
    weather.cloudiness = 80.0;
    weather
}

fn create_wet_noon() -> WeatherParameters {
    let mut weather = create_clear_noon();
    weather.precipitation_deposits = 50.0;
    weather.wetness = 50.0;
    weather
}

fn create_wet_cloudy_noon() -> WeatherParameters {
    let mut weather = create_wet_noon();
    weather.cloudiness = 80.0;
    weather
}

fn create_soft_rain_noon() -> WeatherParameters {
    let mut weather = create_wet_cloudy_noon();
    weather.precipitation = 10.0;
    weather
}

fn create_mid_rain_noon() -> WeatherParameters {
    let mut weather = create_wet_cloudy_noon();
    weather.precipitation = 30.0;
    weather
}

fn create_hard_rain_noon() -> WeatherParameters {
    let mut weather = create_wet_cloudy_noon();
    weather.precipitation = 80.0;
    weather
}

fn create_clear_sunset() -> WeatherParameters {
    let mut weather = create_clear_noon();
    weather.sun_altitude_angle = 0.5;
    weather
}

fn create_cloudy_sunset() -> WeatherParameters {
    let mut weather = create_clear_sunset();
    weather.cloudiness = 80.0;
    weather
}

fn create_wet_sunset() -> WeatherParameters {
    let mut weather = create_clear_sunset();
    weather.precipitation_deposits = 50.0;
    weather.wetness = 50.0;
    weather
}

fn create_wet_cloudy_sunset() -> WeatherParameters {
    let mut weather = create_wet_sunset();
    weather.cloudiness = 80.0;
    weather
}

fn create_soft_rain_sunset() -> WeatherParameters {
    let mut weather = create_wet_cloudy_sunset();
    weather.precipitation = 10.0;
    weather
}

fn create_mid_rain_sunset() -> WeatherParameters {
    let mut weather = create_wet_cloudy_sunset();
    weather.precipitation = 30.0;
    weather
}

fn create_hard_rain_sunset() -> WeatherParameters {
    let mut weather = create_wet_cloudy_sunset();
    weather.precipitation = 80.0;
    weather
}

impl WeatherPreset {
    fn all_presets() -> Vec<WeatherPreset> {
        vec![
            WeatherPreset { name: "Clear Noon" },
            WeatherPreset {
                name: "Cloudy Noon",
            },
            WeatherPreset { name: "Wet Noon" },
            WeatherPreset {
                name: "Wet Cloudy Noon",
            },
            WeatherPreset {
                name: "Soft Rain Noon",
            },
            WeatherPreset {
                name: "Mid Rain Noon",
            },
            WeatherPreset {
                name: "Hard Rain Noon",
            },
            WeatherPreset {
                name: "Clear Sunset",
            },
            WeatherPreset {
                name: "Cloudy Sunset",
            },
            WeatherPreset { name: "Wet Sunset" },
            WeatherPreset {
                name: "Wet Cloudy Sunset",
            },
            WeatherPreset {
                name: "Soft Rain Sunset",
            },
            WeatherPreset {
                name: "Mid Rain Sunset",
            },
            WeatherPreset {
                name: "Hard Rain Sunset",
            },
        ]
    }
}

struct WeatherState {
    current: WeatherParameters,
    target: WeatherParameters,
    transition_progress: f32,
    transition_speed: f32,
}

impl WeatherState {
    fn new(initial: WeatherParameters) -> Self {
        // Create target as a copy of initial
        let target = WeatherParameters {
            cloudiness: initial.cloudiness,
            precipitation: initial.precipitation,
            precipitation_deposits: initial.precipitation_deposits,
            wind_intensity: initial.wind_intensity,
            sun_azimuth_angle: initial.sun_azimuth_angle,
            sun_altitude_angle: initial.sun_altitude_angle,
            fog_density: initial.fog_density,
            fog_distance: initial.fog_distance,
            wetness: initial.wetness,
            fog_falloff: initial.fog_falloff,
            scattering_intensity: initial.scattering_intensity,
            mie_scattering_scale: initial.mie_scattering_scale,
            rayleigh_scattering_scale: initial.rayleigh_scattering_scale,
            dust_storm: initial.dust_storm,
        };
        Self {
            current: initial,
            target,
            transition_progress: 1.0,
            transition_speed: 0.5, // 2 seconds transition at 60 FPS
        }
    }

    fn set_target(&mut self, target: WeatherParameters) {
        self.target = target;
        self.transition_progress = 0.0;
    }

    fn update(&mut self, delta_time: f32) -> &WeatherParameters {
        if self.transition_progress < 1.0 {
            self.transition_progress =
                (self.transition_progress + self.transition_speed * delta_time).min(1.0);

            // Linear interpolation
            let t = self.transition_progress;
            self.current = WeatherParameters {
                cloudiness: lerp(self.current.cloudiness, self.target.cloudiness, t),
                precipitation: lerp(self.current.precipitation, self.target.precipitation, t),
                precipitation_deposits: lerp(
                    self.current.precipitation_deposits,
                    self.target.precipitation_deposits,
                    t,
                ),
                wind_intensity: lerp(self.current.wind_intensity, self.target.wind_intensity, t),
                sun_azimuth_angle: lerp(
                    self.current.sun_azimuth_angle,
                    self.target.sun_azimuth_angle,
                    t,
                ),
                sun_altitude_angle: lerp(
                    self.current.sun_altitude_angle,
                    self.target.sun_altitude_angle,
                    t,
                ),
                fog_density: lerp(self.current.fog_density, self.target.fog_density, t),
                fog_distance: lerp(self.current.fog_distance, self.target.fog_distance, t),
                wetness: lerp(self.current.wetness, self.target.wetness, t),
                fog_falloff: lerp(self.current.fog_falloff, self.target.fog_falloff, t),
                scattering_intensity: lerp(
                    self.current.scattering_intensity,
                    self.target.scattering_intensity,
                    t,
                ),
                mie_scattering_scale: lerp(
                    self.current.mie_scattering_scale,
                    self.target.mie_scattering_scale,
                    t,
                ),
                rayleigh_scattering_scale: lerp(
                    self.current.rayleigh_scattering_scale,
                    self.target.rayleigh_scattering_scale,
                    t,
                ),
                dust_storm: lerp(self.current.dust_storm, self.target.dust_storm, t),
            };
        }

        &self.current
    }
}

fn lerp(a: f32, b: f32, t: f32) -> f32 {
    a + (b - a) * t
}

struct Hud {
    show: bool,
}

impl Hud {
    fn new() -> Self {
        Self { show: true }
    }

    fn toggle(&mut self) {
        self.show = !self.show;
    }

    fn render(
        &self,
        preset_name: &str,
        weather: &WeatherParameters,
        simulation_time: f32,
        time_speed: f32,
        time_paused: bool,
    ) {
        if !self.show {
            return;
        }

        let y_offset = 20.0;
        let line_height = 20.0;
        let mut y = y_offset;

        // Calculate time of day
        let hours = (simulation_time / 3600.0) as i32 % 24;
        let minutes = ((simulation_time % 3600.0) / 60.0) as i32;

        let texts = vec![
            "CARLA - Dynamic Weather Demo".to_string(),
            "".to_string(),
            format!("Weather: {}", preset_name),
            format!(
                "Time: {:02}:{:02} {}",
                hours,
                minutes,
                if time_paused { "(PAUSED)" } else { "" }
            ),
            format!("Time Speed: {:.1}x", time_speed),
            "".to_string(),
            "Weather Parameters:".to_string(),
            format!("  Cloudiness: {:.1}%", weather.cloudiness),
            format!("  Precipitation: {:.1}%", weather.precipitation),
            format!(
                "  Precipitation Deposits: {:.1}%",
                weather.precipitation_deposits
            ),
            format!("  Wind Intensity: {:.1}%", weather.wind_intensity),
            format!("  Fog Density: {:.1}%", weather.fog_density),
            format!("  Wetness: {:.1}%", weather.wetness),
            "".to_string(),
            "Sun Position:".to_string(),
            format!("  Altitude: {:.1}°", weather.sun_altitude_angle),
            format!("  Azimuth: {:.1}°", weather.sun_azimuth_angle),
            "".to_string(),
            "Controls:".to_string(),
            "  C / Shift+C - Cycle weather presets".to_string(),
            "  T - Toggle time progression".to_string(),
            "  +/- - Adjust time speed".to_string(),
            "  1-9 - Jump to time of day".to_string(),
            "  H - Toggle HUD".to_string(),
            "  ESC/Q - Quit".to_string(),
        ];

        for text in texts {
            let text_width = measure_text(&text, None, 20, 1.0).width;
            draw_rectangle(
                10.0,
                y - 15.0,
                text_width + 10.0,
                line_height,
                Color::from_rgba(0, 0, 0, 180),
            );
            draw_text(&text, 15.0, y, 20.0, WHITE);
            y += line_height;
        }
    }
}

struct HelpOverlay {
    show: bool,
}

impl HelpOverlay {
    fn new() -> Self {
        Self { show: false }
    }

    fn toggle(&mut self) {
        self.show = !self.show;
    }

    fn render(&self) {
        if !self.show {
            return;
        }

        draw_rectangle(
            0.0,
            0.0,
            WINDOW_WIDTH as f32,
            WINDOW_HEIGHT as f32,
            Color::from_rgba(0, 0, 0, 200),
        );

        let center_x = WINDOW_WIDTH as f32 / 2.0;
        let start_y = 50.0;
        let line_height = 30.0;

        let help_text = vec![
            ("DYNAMIC WEATHER DEMO", 30.0),
            ("", 20.0),
            (
                "Demonstrates smooth weather parameter interpolation with day/night cycle.",
                20.0,
            ),
            ("", 20.0),
            ("WEATHER CONTROLS:", 25.0),
            ("", 20.0),
            ("C              Next weather preset", 20.0),
            ("Shift+C        Previous weather preset", 20.0),
            ("", 20.0),
            ("TIME CONTROLS:", 25.0),
            ("", 20.0),
            ("T              Toggle time progression", 20.0),
            ("+              Speed up time (max 10x)", 20.0),
            ("-              Slow down time (min 0.1x)", 20.0),
            ("1-9            Jump to time of day (1=1am, 9=9am)", 20.0),
            ("", 20.0),
            ("OTHER:", 25.0),
            ("", 20.0),
            ("H              Toggle this help", 20.0),
            ("F1             Toggle HUD", 20.0),
            ("ESC / Q        Quit", 20.0),
            ("", 20.0),
            ("WEATHER PRESETS:", 25.0),
            ("", 20.0),
            ("Clear/Cloudy/Wet/Rain (Soft/Mid/Hard) at Noon/Sunset", 20.0),
            ("", 20.0),
            ("Press H to close this help", 20.0),
        ];

        let mut y = start_y;
        for (text, size) in help_text {
            let text_dims = measure_text(text, None, size as u16, 1.0);
            let x = center_x - text_dims.width / 2.0;
            draw_text(text, x, y, size, WHITE);
            y += line_height;
        }
    }
}

#[macroquad::main(window_conf)]
async fn main() -> Result<()> {
    println!("Dynamic Weather GUI - Connecting to CARLA...");

    let client = Client::connect("localhost", 2000, 0);
    let mut world = client.world();

    println!("Connected! Setting up scene...");

    // Get spawn points
    let map = world.map();
    let spawn_points = map.recommended_spawn_points();

    if spawn_points.is_empty() {
        return Err(anyhow::anyhow!("No spawn points available"));
    }

    // Spawn vehicle
    let blueprint_library = world.blueprint_library();
    let vehicle_bp = blueprint_library
        .filter("vehicle.*")
        .iter()
        .next()
        .ok_or_else(|| anyhow::anyhow!("No vehicle blueprints found"))?;

    let spawn_point = &spawn_points.as_slice()[0];
    let vehicle_actor = world.spawn_actor(&vehicle_bp, spawn_point)?;
    let vehicle = Vehicle::try_from(vehicle_actor)
        .map_err(|_| anyhow::anyhow!("Failed to cast to Vehicle"))?;

    // Enable autopilot
    vehicle.set_autopilot(true);

    println!("Vehicle spawned: {}", vehicle.type_id());

    // Create camera
    let mut camera = CameraManager::new(&mut world, &vehicle)?;
    println!("Camera attached");

    // Initialize weather
    let presets = WeatherPreset::all_presets();
    let mut current_preset_idx = 0;
    let initial_weather = create_clear_noon();
    let mut weather_state = WeatherState::new(initial_weather);

    // Apply initial weather
    world.set_weather(&weather_state.current);

    // Create UI
    let mut hud = Hud::new();
    let mut help = HelpOverlay::new();

    // Time simulation (8-hour cycle)
    let mut simulation_time: f32 = 12.0 * 3600.0; // Start at noon
    let mut time_speed: f32 = 1.0;
    let mut time_paused = false;
    let cycle_duration: f32 = 8.0 * 3600.0; // 8 hours

    println!("Starting main loop...");
    println!("Press H for help, C to cycle weather presets");

    let mut last_frame_time = get_time();

    loop {
        let current_time = get_time();
        let delta_time = (current_time - last_frame_time) as f32;
        last_frame_time = current_time;

        // Handle input
        if is_key_pressed(KeyCode::Escape) || is_key_pressed(KeyCode::Q) {
            break;
        }

        if is_key_pressed(KeyCode::H) {
            help.toggle();
        }

        if is_key_pressed(KeyCode::F1) {
            hud.toggle();
        }

        if is_key_pressed(KeyCode::T) {
            time_paused = !time_paused;
            println!("Time {}", if time_paused { "PAUSED" } else { "RESUMED" });
        }

        if is_key_pressed(KeyCode::Equal) || is_key_pressed(KeyCode::KpAdd) {
            time_speed = (time_speed + 0.5).min(10.0);
            println!("Time speed: {:.1}x", time_speed);
        }

        if is_key_pressed(KeyCode::Minus) || is_key_pressed(KeyCode::KpSubtract) {
            time_speed = (time_speed - 0.5).max(0.1);
            println!("Time speed: {:.1}x", time_speed);
        }

        // Weather preset cycling
        if is_key_pressed(KeyCode::C) {
            if is_key_down(KeyCode::LeftShift) || is_key_down(KeyCode::RightShift) {
                // Previous preset
                current_preset_idx = if current_preset_idx == 0 {
                    presets.len() - 1
                } else {
                    current_preset_idx - 1
                };
            } else {
                // Next preset
                current_preset_idx = (current_preset_idx + 1) % presets.len();
            }
            // Get new preset params
            let new_params = match current_preset_idx {
                0 => create_clear_noon(),
                1 => create_cloudy_noon(),
                2 => create_wet_noon(),
                3 => create_wet_cloudy_noon(),
                4 => create_soft_rain_noon(),
                5 => create_mid_rain_noon(),
                6 => create_hard_rain_noon(),
                7 => create_clear_sunset(),
                8 => create_cloudy_sunset(),
                9 => create_wet_sunset(),
                10 => create_wet_cloudy_sunset(),
                11 => create_soft_rain_sunset(),
                12 => create_mid_rain_sunset(),
                13 => create_hard_rain_sunset(),
                _ => create_clear_noon(),
            };
            weather_state.set_target(new_params);
            println!("Switching to: {}", presets[current_preset_idx].name);
        }

        // Jump to time of day (1-9 keys)
        if is_key_pressed(KeyCode::Key1) {
            simulation_time = 1.0 * 3600.0;
            println!("Jumped to 01:00");
        }
        if is_key_pressed(KeyCode::Key2) {
            simulation_time = 2.0 * 3600.0;
            println!("Jumped to 02:00");
        }
        if is_key_pressed(KeyCode::Key3) {
            simulation_time = 3.0 * 3600.0;
            println!("Jumped to 03:00");
        }
        if is_key_pressed(KeyCode::Key4) {
            simulation_time = 4.0 * 3600.0;
            println!("Jumped to 04:00");
        }
        if is_key_pressed(KeyCode::Key5) {
            simulation_time = 5.0 * 3600.0;
            println!("Jumped to 05:00");
        }
        if is_key_pressed(KeyCode::Key6) {
            simulation_time = 6.0 * 3600.0;
            println!("Jumped to 06:00");
        }
        if is_key_pressed(KeyCode::Key7) {
            simulation_time = 7.0 * 3600.0;
            println!("Jumped to 07:00");
        }
        if is_key_pressed(KeyCode::Key8) {
            simulation_time = 8.0 * 3600.0;
            println!("Jumped to 08:00");
        }
        if is_key_pressed(KeyCode::Key9) {
            simulation_time = 9.0 * 3600.0;
            println!("Jumped to 09:00");
        }

        // Update time simulation
        if !time_paused {
            simulation_time += delta_time * time_speed * 100.0; // Accelerated time
            simulation_time %= cycle_duration;

            // Update sun position based on time
            let time_fraction = simulation_time / cycle_duration;
            let sun_altitude = (time_fraction * 180.0) - 90.0; // -90° to +90°
            let sun_azimuth = time_fraction * 360.0; // 0° to 360°

            weather_state.target.sun_altitude_angle = sun_altitude;
            weather_state.target.sun_azimuth_angle = sun_azimuth;
        }

        // Update weather interpolation
        let current_weather = weather_state.update(delta_time);
        world.set_weather(current_weather);

        // Update camera
        camera.update();

        // Render
        clear_background(BLACK);
        camera.render();
        hud.render(
            presets[current_preset_idx].name,
            current_weather,
            simulation_time,
            time_speed,
            time_paused,
        );
        help.render();

        next_frame().await;
    }

    println!("Cleaning up...");
    vehicle.destroy();
    camera.sensor.destroy();
    println!("Done!");

    Ok(())
}
