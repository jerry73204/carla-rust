//! Manual Control with Steering Wheel Support
//!
//! Demonstrates manual vehicle control using a racing wheel (Logitech G29/G920) or keyboard fallback.
//! Provides realistic driving experience with analog steering, throttle, and brake inputs.
//!
//! # Features
//! - Logitech G29/G920 steering wheel support (via gilrs)
//! - Analog steering input (-1.0 to 1.0)
//! - Analog throttle and brake pedals (0.0 to 1.0)
//! - Force feedback effects (if hardware supports)
//! - Button mapping for lights, gears, camera switching
//! - Visual input display on HUD (steering angle, pedal positions)
//! - Keyboard fallback when no wheel detected
//!
//! # Controls (Steering Wheel)
//! - **Steering Wheel**: Steer vehicle
//! - **Throttle Pedal**: Accelerate
//! - **Brake Pedal**: Brake
//! - **Button 0 (X/A)**: Toggle reverse gear
//! - **Button 1 (Circle/B)**: Toggle headlights
//! - **Button 2 (Square/X)**: Switch camera
//! - **Button 3 (Triangle/Y)**: Handbrake
//! - **D-Pad Up/Down**: Adjust force feedback strength
//!
//! # Controls (Keyboard Fallback)
//! - **W/S**: Throttle/Brake
//! - **A/D**: Steering left/right
//! - **Space**: Handbrake
//! - **L**: Toggle headlights
//! - **C**: Switch camera
//! - **R**: Toggle reverse gear
//! - **H**: Toggle help overlay
//! - **ESC/Q**: Quit
//!
//! # Usage
//! ```bash
//! cargo run --example manual_control_steeringwheel --profile dev-release
//! ```

use anyhow::Result;
use carla::{
    client::{ActorBase, Client, Sensor, Vehicle, World as CarlaWorld},
    geom::{Location, Rotation, Transform},
    rpc::{VehicleControl, VehicleLightState},
    sensor::data::Image as CarlaImage,
};
use gilrs::{
    ff::{BaseEffect, BaseEffectType, Effect, EffectBuilder, Replay},
    Axis, Button, Event, EventType, Gilrs,
};
use macroquad::prelude::*;
use std::sync::{Arc, Mutex};

const WINDOW_WIDTH: u32 = 1280;
const WINDOW_HEIGHT: u32 = 720;
const CAMERA_WIDTH: u32 = 1280;
const CAMERA_HEIGHT: u32 = 720;

// Dead zone for analog inputs to prevent drift
const DEAD_ZONE: f32 = 0.05;
// Steering sensitivity curve (1.0 = linear, <1.0 = less sensitive in center)
const STEERING_CURVE: f32 = 2.0;

fn window_conf() -> Conf {
    Conf {
        window_title: "CARLA - Manual Control (Steering Wheel)".to_owned(),
        window_width: WINDOW_WIDTH as i32,
        window_height: WINDOW_HEIGHT as i32,
        window_resizable: false,
        ..Default::default()
    }
}

/// Input state from gamepad or keyboard
#[derive(Debug, Clone)]
struct InputState {
    steering: f32, // -1.0 (left) to 1.0 (right)
    throttle: f32, // 0.0 to 1.0
    brake: f32,    // 0.0 to 1.0
    handbrake: bool,
    reverse: bool,
    toggle_lights: bool,
    switch_camera: bool,
}

impl Default for InputState {
    fn default() -> Self {
        Self {
            steering: 0.0,
            throttle: 0.0,
            brake: 0.0,
            handbrake: false,
            reverse: false,
            toggle_lights: false,
            switch_camera: false,
        }
    }
}

/// Gamepad controller with force feedback support
struct GamepadController {
    gilrs: Gilrs,
    gamepad_id: Option<gilrs::GamepadId>,
    force_feedback_strength: f32,
    ff_effect_id: Option<Effect>,
}

impl GamepadController {
    fn new() -> Result<Self> {
        let gilrs =
            Gilrs::new().map_err(|e| anyhow::anyhow!("Failed to initialize gilrs: {}", e))?;

        // Find first connected gamepad
        let gamepad_id = gilrs
            .gamepads()
            .find(|(_, gp)| gp.is_connected())
            .map(|(id, _)| id);

        Ok(Self {
            gilrs,
            gamepad_id,
            force_feedback_strength: 0.5,
            ff_effect_id: None,
        })
    }

    fn has_gamepad(&self) -> bool {
        self.gamepad_id.is_some()
    }

    fn gamepad_name(&self) -> Option<String> {
        self.gamepad_id
            .map(|id| self.gilrs.gamepad(id).name().to_string())
    }

    /// Process gamepad events and update input state
    fn update(&mut self, input: &mut InputState) {
        // Process events
        while let Some(Event { id, event, .. }) = self.gilrs.next_event() {
            // Update current gamepad if disconnected
            if self.gamepad_id.is_none() || self.gamepad_id == Some(id) {
                self.gamepad_id = Some(id);
            }

            match event {
                EventType::ButtonPressed(button, _) => {
                    match button {
                        Button::South => input.reverse = !input.reverse, // X/A
                        Button::East => input.toggle_lights = true,      // Circle/B
                        Button::West => input.switch_camera = true,      // Square/X
                        Button::North => input.handbrake = true,         // Triangle/Y
                        Button::DPadUp => {
                            self.force_feedback_strength =
                                (self.force_feedback_strength + 0.1).min(1.0);
                        }
                        Button::DPadDown => {
                            self.force_feedback_strength =
                                (self.force_feedback_strength - 0.1).max(0.0);
                        }
                        _ => {}
                    }
                }
                EventType::ButtonReleased(Button::North, _) => {
                    input.handbrake = false;
                }
                _ => {}
            }
        }

        // Read analog axes
        if let Some(id) = self.gamepad_id {
            if let Some(gamepad) = self.gilrs.connected_gamepad(id) {
                // Steering (LeftStickX or first available axis)
                let steering_raw = gamepad
                    .axis_data(Axis::LeftStickX)
                    .map(|a| a.value())
                    .unwrap_or(0.0);
                input.steering =
                    Self::apply_dead_zone_and_curve(steering_raw, DEAD_ZONE, STEERING_CURVE);

                // Throttle (RightTrigger2 or RightZ)
                let throttle_raw = gamepad
                    .axis_data(Axis::RightZ)
                    .or_else(|| gamepad.axis_data(Axis::RightStickY))
                    .map(|a| (a.value() + 1.0) / 2.0) // Convert -1..1 to 0..1
                    .unwrap_or(0.0);
                input.throttle = Self::apply_dead_zone(throttle_raw, DEAD_ZONE).clamp(0.0, 1.0);

                // Brake (LeftTrigger2 or LeftZ)
                let brake_raw = gamepad
                    .axis_data(Axis::LeftZ)
                    .or_else(|| gamepad.axis_data(Axis::LeftStickY))
                    .map(|a| (a.value() + 1.0) / 2.0) // Convert -1..1 to 0..1
                    .unwrap_or(0.0);
                input.brake = Self::apply_dead_zone(brake_raw, DEAD_ZONE).clamp(0.0, 1.0);
            }
        }
    }

    /// Apply dead zone to prevent drift
    fn apply_dead_zone(value: f32, dead_zone: f32) -> f32 {
        if value.abs() < dead_zone {
            0.0
        } else {
            // Scale the value so dead zone is smooth
            let sign = value.signum();
            let magnitude = (value.abs() - dead_zone) / (1.0 - dead_zone);
            sign * magnitude
        }
    }

    /// Apply dead zone and sensitivity curve
    fn apply_dead_zone_and_curve(value: f32, dead_zone: f32, curve: f32) -> f32 {
        let dz_value = Self::apply_dead_zone(value, dead_zone);
        let sign = dz_value.signum();
        let magnitude = dz_value.abs();
        sign * magnitude.powf(curve)
    }

    /// Apply force feedback effect based on vehicle speed
    fn apply_force_feedback(&mut self, speed_kmh: f32) {
        if let Some(id) = self.gamepad_id {
            if let Some(gamepad) = self.gilrs.connected_gamepad(id) {
                if !gamepad.is_ff_supported() {
                    return;
                }

                // Remove old effect
                if let Some(_effect) = self.ff_effect_id.take() {
                    // Effect is automatically dropped
                }

                // Calculate resistance based on speed (stronger at higher speeds)
                let resistance = (speed_kmh / 100.0).min(1.0) * self.force_feedback_strength;

                if resistance > 0.01 {
                    // Create constant force effect
                    let duration = gilrs::ff::Ticks::from_ms(100);
                    let effect = EffectBuilder::new()
                        .add_effect(BaseEffect {
                            kind: BaseEffectType::Strong {
                                magnitude: (resistance * u16::MAX as f32) as u16,
                            },
                            scheduling: Replay {
                                play_for: duration,
                                with_delay: gilrs::ff::Ticks::from_ms(0),
                                ..Default::default()
                            },
                            ..Default::default()
                        })
                        .gamepads(&[id])
                        .finish(&mut self.gilrs);

                    if let Ok(effect) = effect {
                        let _ = effect.play();
                        self.ff_effect_id = Some(effect);
                    }
                }
            }
        }
    }
}

/// Keyboard input handler (fallback when no gamepad)
struct KeyboardInput {
    // Keyboard state tracking for smooth acceleration/deceleration
    throttle_input: f32,
    brake_input: f32,
    steering_input: f32,
}

impl KeyboardInput {
    fn new() -> Self {
        Self {
            throttle_input: 0.0,
            brake_input: 0.0,
            steering_input: 0.0,
        }
    }

    fn update(&mut self, input: &mut InputState, dt: f32) {
        // Acceleration/deceleration rate for smooth keyboard input
        let accel_rate = 2.0 * dt;
        let steer_rate = 3.0 * dt;

        // Throttle/Brake (W/S)
        if is_key_down(KeyCode::W) {
            self.throttle_input = (self.throttle_input + accel_rate).min(1.0);
            self.brake_input = 0.0;
        } else if is_key_down(KeyCode::S) {
            self.brake_input = (self.brake_input + accel_rate).min(1.0);
            self.throttle_input = 0.0;
        } else {
            // Decay inputs when no key pressed
            self.throttle_input = (self.throttle_input - accel_rate * 2.0).max(0.0);
            self.brake_input = (self.brake_input - accel_rate * 2.0).max(0.0);
        }

        input.throttle = self.throttle_input;
        input.brake = self.brake_input;

        // Steering (A/D)
        if is_key_down(KeyCode::A) {
            self.steering_input = (self.steering_input - steer_rate).max(-1.0);
        } else if is_key_down(KeyCode::D) {
            self.steering_input = (self.steering_input + steer_rate).min(1.0);
        } else {
            // Return to center when no key pressed
            if self.steering_input > 0.0 {
                self.steering_input = (self.steering_input - steer_rate * 2.0).max(0.0);
            } else if self.steering_input < 0.0 {
                self.steering_input = (self.steering_input + steer_rate * 2.0).min(0.0);
            }
        }

        input.steering = self.steering_input;

        // Buttons
        input.handbrake = is_key_down(KeyCode::Space);

        if is_key_pressed(KeyCode::L) {
            input.toggle_lights = true;
        }
        if is_key_pressed(KeyCode::C) {
            input.switch_camera = true;
        }
        if is_key_pressed(KeyCode::R) {
            input.reverse = !input.reverse;
        }
    }
}

struct CameraManager {
    sensor: Sensor,
    texture: Texture2D,
    latest_image: Arc<Mutex<Option<CarlaImage>>>,
    current_view: usize,
}

impl CameraManager {
    fn new(world: &mut CarlaWorld, vehicle: &Vehicle, view: usize) -> Result<Self> {
        let sensor = Self::spawn_camera(world, vehicle, view)?;

        // Create texture
        let texture = Texture2D::from_rgba8(
            CAMERA_WIDTH as u16,
            CAMERA_HEIGHT as u16,
            &vec![0u8; (CAMERA_WIDTH * CAMERA_HEIGHT * 4) as usize],
        );
        texture.set_filter(FilterMode::Linear);

        // Setup listener
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
            current_view: view,
        })
    }

    fn spawn_camera(world: &mut CarlaWorld, vehicle: &Vehicle, view: usize) -> Result<Sensor> {
        let blueprint_library = world.blueprint_library();
        let camera_bp = blueprint_library
            .find("sensor.camera.rgb")
            .ok_or_else(|| anyhow::anyhow!("Camera blueprint not found"))?;

        let mut camera_bp = camera_bp;
        let _ = camera_bp.set_attribute("image_size_x", &CAMERA_WIDTH.to_string());
        let _ = camera_bp.set_attribute("image_size_y", &CAMERA_HEIGHT.to_string());
        let _ = camera_bp.set_attribute("fov", "110");

        // Different camera views
        let transform = match view {
            0 => Transform {
                location: Location::new(-5.5, 0.0, 2.5), // Behind vehicle
                rotation: Rotation::new(0.0, 0.0, 0.0),
            },
            1 => Transform {
                location: Location::new(1.5, 0.0, 1.2), // Hood view
                rotation: Rotation::new(0.0, 0.0, 0.0),
            },
            2 => Transform {
                location: Location::new(0.0, 0.0, 50.0), // Top-down view
                rotation: Rotation::new(-90.0, 0.0, 0.0),
            },
            _ => Transform {
                location: Location::new(-5.5, 0.0, 2.5),
                rotation: Rotation::new(0.0, 0.0, 0.0),
            },
        };

        let camera = world.spawn_actor_attached(
            &camera_bp,
            &transform,
            vehicle,
            carla::rpc::AttachmentType::SpringArm,
        )?;

        Sensor::try_from(camera).map_err(|_| anyhow::anyhow!("Failed to cast to Sensor"))
    }

    fn switch_view(&mut self, world: &mut CarlaWorld, vehicle: &Vehicle) -> Result<()> {
        // Destroy old camera
        self.sensor.destroy();

        // Cycle through views
        self.current_view = (self.current_view + 1) % 3;

        // Spawn new camera
        self.sensor = Self::spawn_camera(world, vehicle, self.current_view)?;

        // Setup new listener
        let latest_image_clone = Arc::clone(&self.latest_image);
        self.sensor.listen(move |data| {
            if let Ok(image) = CarlaImage::try_from(data) {
                if let Ok(mut img) = latest_image_clone.lock() {
                    *img = Some(image);
                }
            }
        });

        Ok(())
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

struct Hud {
    show: bool,
}

impl Hud {
    fn new() -> Self {
        Self { show: true }
    }

    fn render(
        &self,
        vehicle: &Vehicle,
        input: &InputState,
        gamepad_name: Option<&str>,
        ff_strength: f32,
        camera_view: usize,
    ) {
        if !self.show {
            return;
        }

        let y_offset = 20.0;
        let line_height = 20.0;
        let mut y = y_offset;

        // Get vehicle data
        let transform = vehicle.transform();
        let location = transform.location;
        let velocity = vehicle.velocity();
        let speed_ms = velocity.length();
        let speed_kmh = speed_ms * 3.6;

        // Get vehicle control state
        let control = vehicle.control();

        // Camera view name
        let camera_name = match camera_view {
            0 => "Third Person",
            1 => "Hood View",
            2 => "Top-Down",
            _ => "Unknown",
        };

        // Input device
        let input_device = gamepad_name.unwrap_or("Keyboard (Fallback)");

        // Build HUD text
        let texts = vec![
            "CARLA - Manual Control (Steering Wheel)".to_string(),
            "".to_string(),
            format!("Input Device: {}", input_device),
            format!("Camera: {}", camera_name),
            format!("Speed: {:.1} km/h", speed_kmh),
            format!(
                "Location: ({:.1}, {:.1}, {:.1})",
                location.x, location.y, location.z
            ),
            "".to_string(),
            format!("--- Input State ---"),
            format!(
                "Steering: {:.2} {}",
                input.steering,
                Self::make_bar(input.steering, -1.0, 1.0, 20)
            ),
            format!(
                "Throttle: {:.2} {}",
                input.throttle,
                Self::make_bar(input.throttle, 0.0, 1.0, 20)
            ),
            format!(
                "Brake:    {:.2} {}",
                input.brake,
                Self::make_bar(input.brake, 0.0, 1.0, 20)
            ),
            format!("Handbrake: {}", if input.handbrake { "ON" } else { "OFF" }),
            format!("Reverse: {}", if input.reverse { "ON" } else { "OFF" }),
            "".to_string(),
            format!("--- Vehicle Control ---"),
            format!("Actual Steer: {:.2}", control.steer),
            format!("Actual Throttle: {:.2}", control.throttle),
            format!("Actual Brake: {:.2}", control.brake),
            format!("Gear: {}", if control.reverse { "R" } else { "D" }),
        ];

        // Add force feedback info if gamepad present
        let mut all_texts = texts;
        if gamepad_name.is_some() {
            all_texts.push("".to_string());
            all_texts.push(format!("Force Feedback: {:.0}%", ff_strength * 100.0));
        }

        for text in all_texts {
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

    /// Create a simple bar visualization
    fn make_bar(value: f32, min: f32, max: f32, width: usize) -> String {
        let normalized = ((value - min) / (max - min)).clamp(0.0, 1.0);
        let filled = (normalized * width as f32) as usize;
        let empty = width - filled;

        format!("[{}{}]", "=".repeat(filled), " ".repeat(empty))
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

    fn render(&self, has_gamepad: bool) {
        if !self.show {
            return;
        }

        let panel_width = 600.0;
        let panel_height = if has_gamepad { 500.0 } else { 400.0 };
        let x = (WINDOW_WIDTH as f32 - panel_width) / 2.0;
        let y = (WINDOW_HEIGHT as f32 - panel_height) / 2.0;

        // Semi-transparent background
        draw_rectangle(
            x,
            y,
            panel_width,
            panel_height,
            Color::from_rgba(0, 0, 0, 220),
        );
        draw_rectangle_lines(x, y, panel_width, panel_height, 2.0, WHITE);

        let mut text_y = y + 30.0;
        let line_height = 25.0;

        draw_text("CONTROLS", x + 20.0, text_y, 30.0, YELLOW);
        text_y += line_height * 1.5;

        if has_gamepad {
            let controls = vec![
                "Steering Wheel - Steer vehicle",
                "Throttle Pedal - Accelerate",
                "Brake Pedal - Brake",
                "Button X/A - Toggle reverse gear",
                "Button O/B - Toggle headlights",
                "Button □/X - Switch camera view",
                "Button △/Y - Handbrake",
                "D-Pad Up/Down - Adjust force feedback strength",
                "",
                "H - Toggle this help",
                "ESC/Q - Quit",
            ];

            for control in controls {
                draw_text(control, x + 20.0, text_y, 20.0, WHITE);
                text_y += line_height;
            }
        } else {
            let controls = vec![
                "KEYBOARD FALLBACK MODE",
                "",
                "W/S - Throttle / Brake",
                "A/D - Steer left / right",
                "Space - Handbrake",
                "L - Toggle headlights",
                "C - Switch camera view",
                "R - Toggle reverse gear",
                "",
                "H - Toggle this help",
                "ESC/Q - Quit",
            ];

            for control in controls {
                draw_text(control, x + 20.0, text_y, 20.0, WHITE);
                text_y += line_height;
            }
        }
    }
}

#[macroquad::main(window_conf)]
async fn main() -> Result<()> {
    println!("=== CARLA Manual Control (Steering Wheel) ===\n");

    // Initialize gamepad controller
    let mut gamepad = GamepadController::new()?;
    let mut keyboard = KeyboardInput::new();

    if gamepad.has_gamepad() {
        println!(
            "Gamepad detected: {}",
            gamepad
                .gamepad_name()
                .unwrap_or_else(|| "Unknown".to_string())
        );
    } else {
        println!("No gamepad detected - using keyboard fallback");
    }

    // Connect to CARLA
    let client = Client::connect("127.0.0.1", 2000, None);
    let mut world = client.world();

    println!("Connected to CARLA server");

    // Spawn vehicle
    let blueprint_library = world.blueprint_library();
    let vehicle_bp = blueprint_library
        .find("vehicle.tesla.model3")
        .ok_or_else(|| anyhow::anyhow!("Vehicle blueprint not found"))?;

    let spawn_points = world.map().recommended_spawn_points();
    let spawn_point = spawn_points
        .get(0)
        .ok_or_else(|| anyhow::anyhow!("No spawn points available"))?;

    let vehicle_actor = world.spawn_actor(&vehicle_bp, spawn_point)?;
    let vehicle = Vehicle::try_from(vehicle_actor)
        .map_err(|_| anyhow::anyhow!("Failed to cast to Vehicle"))?;

    println!("Spawned vehicle at spawn point");

    // Setup camera
    let mut camera = CameraManager::new(&mut world, &vehicle, 0)?;

    // Initialize HUD and help overlay
    let hud = Hud::new();
    let mut help_overlay = HelpOverlay::new();

    // Vehicle state
    let mut lights_on = false;
    let mut input_state = InputState::default();

    println!("Starting control loop...");

    let mut last_frame_time = get_time();

    loop {
        // Calculate delta time
        let current_time = get_time();
        let dt = (current_time - last_frame_time) as f32;
        last_frame_time = current_time;

        // Clear previous frame button states
        input_state.toggle_lights = false;
        input_state.switch_camera = false;

        // Update input
        if gamepad.has_gamepad() {
            gamepad.update(&mut input_state);
        } else {
            keyboard.update(&mut input_state, dt);
        }

        // Handle UI controls
        if is_key_pressed(KeyCode::H) {
            help_overlay.toggle();
        }
        if is_key_pressed(KeyCode::Escape) || is_key_pressed(KeyCode::Q) {
            break;
        }

        // Handle camera switching
        if input_state.switch_camera {
            if let Err(e) = camera.switch_view(&mut world, &vehicle) {
                eprintln!("Failed to switch camera: {}", e);
            }
        }

        // Handle light toggle
        if input_state.toggle_lights {
            lights_on = !lights_on;
            let light_state = if lights_on {
                VehicleLightState::LOW_BEAM | VehicleLightState::POSITION
            } else {
                VehicleLightState::NONE
            };
            vehicle.set_light_state(&light_state);
        }

        // Apply vehicle control
        let control = VehicleControl {
            throttle: input_state.throttle,
            steer: input_state.steering,
            brake: input_state.brake,
            hand_brake: input_state.handbrake,
            reverse: input_state.reverse,
            manual_gear_shift: false,
            gear: 0,
        };
        vehicle.apply_control(&control);

        // Apply force feedback
        if gamepad.has_gamepad() {
            let velocity = vehicle.velocity();
            let speed_kmh = velocity.length() * 3.6;
            gamepad.apply_force_feedback(speed_kmh);
        }

        // Update camera texture
        camera.update();

        // Render
        clear_background(BLACK);
        camera.render();

        let gamepad_name = gamepad.gamepad_name();
        hud.render(
            &vehicle,
            &input_state,
            gamepad_name.as_deref(),
            gamepad.force_feedback_strength,
            camera.current_view,
        );
        help_overlay.render(gamepad.has_gamepad());

        next_frame().await;
    }

    println!("\nCleaning up...");

    // Cleanup
    camera.sensor.destroy();
    vehicle.destroy();

    println!("Done!");
    Ok(())
}
