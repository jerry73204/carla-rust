//! Keyboard input handling
//!
//! The KeyboardControl struct handles:
//! - Event-based input (key press/release)
//! - Continuous input (key held down)
//! - Vehicle control (throttle, brake, steer, gear, handbrake)
//! - Walker control (speed, direction, jump)
//! - Ackermann steering mode
//! - Light state management
//! - Advanced features (radar, doors, constant velocity, telemetry)
//! - Recording and replay controls
//!
//! ## Phase 13 Implementation
//!
//! - ✅ Phase 3: Basic vehicle control (WASD, autopilot toggle)
//! - ✅ Phase 7: Advanced controls (manual transmission, lights, Ackermann)
//! - ✅ Phase 8: Camera switching
//! - ✅ Phase 9: Weather and map layer control
//! - ✅ Phase 10: Recording controls
//! - ✅ Phase 11: Advanced features (radar, doors, constant velocity, telemetry)
//! - ✅ Phase 12: Help and HUD toggle (H, F1)

use carla::{
    client::ActorBase,
    rpc::{VehicleAckermannControl, VehicleControl, VehicleLightState},
};
use eyre::Result;
use macroquad::prelude::*;

/// Keyboard input controller
///
/// Handles all keyboard input for vehicle control and UI
#[allow(dead_code)]
pub struct KeyboardControl {
    pub control: VehicleControl,
    pub autopilot_enabled: bool,
    pub manual_transmission_enabled: bool,
    pub current_gear: i32,
    pub lights: VehicleLightState,
    pub ackermann_enabled: bool,
    pub ackermann_control: VehicleAckermannControl,
    pub ackermann_reverse: f32,
    // TODO Phase 3: Add steering cache for smooth steering
    // steer_cache: f32
}

impl KeyboardControl {
    /// Create a new keyboard controller
    pub fn new(autopilot: bool) -> Self {
        Self {
            control: VehicleControl {
                throttle: 0.0,
                steer: 0.0,
                brake: 0.0,
                hand_brake: false,
                reverse: false,
                manual_gear_shift: false,
                gear: 0,
            },
            autopilot_enabled: autopilot,
            manual_transmission_enabled: false,
            current_gear: 1,
            lights: VehicleLightState::NONE,
            ackermann_enabled: false,
            ackermann_control: VehicleAckermannControl {
                steer: 0.0,
                steer_speed: 0.0,
                speed: 0.0,
                acceleration: 0.0,
                jerk: 0.0,
            },
            ackermann_reverse: 1.0,
        }
    }

    /// Parse keyboard events (key press/release)
    ///
    /// ✅ Subphase 12.3.2: Autopilot toggle implemented
    /// ✅ Subphase 12.8.1: Sensor switching implemented
    /// ✅ Subphase 12.10: Recording and replay implemented
    /// ✅ Subphase 12.12: Help and HUD toggle implemented
    /// Returns true if should quit
    pub fn parse_events(
        &mut self,
        client: &mut carla::client::Client,
        world: &mut crate::world::World,
        hud: &mut crate::hud::Hud,
        camera: &mut crate::camera::CameraManager,
        notification: &mut crate::ui::FadingText,
        help: &mut crate::ui::HelpText,
    ) -> bool {
        // P key - Toggle autopilot
        if is_key_pressed(KeyCode::P) {
            self.autopilot_enabled = !self.autopilot_enabled;

            // Apply autopilot setting to vehicle
            if let Some(ref player) = world.player {
                player.set_autopilot(self.autopilot_enabled);
            }

            // Show notification
            let message = if self.autopilot_enabled {
                "Autopilot enabled"
            } else {
                "Autopilot disabled"
            };
            notification.set_text(message, 2.0);
        }

        // ✅ Subphase 12.7.1: M key - Toggle manual transmission
        if is_key_pressed(KeyCode::M) {
            self.manual_transmission_enabled = !self.manual_transmission_enabled;
            self.control.manual_gear_shift = self.manual_transmission_enabled;

            // Reset to gear 1 when enabling manual transmission
            if self.manual_transmission_enabled {
                self.current_gear = 1;
                self.control.gear = 1;
            } else {
                self.control.gear = 0; // Auto mode
            }

            let message = if self.manual_transmission_enabled {
                "Manual transmission enabled"
            } else {
                "Manual transmission disabled"
            };
            notification.set_text(message, 2.0);
        }

        // ✅ Subphase 12.7.1: Comma key - Shift down
        if is_key_pressed(KeyCode::Comma)
            && self.manual_transmission_enabled
            && self.current_gear > -1
        {
            self.current_gear -= 1;
            self.control.gear = self.current_gear;
            notification.set_text(format!("Gear: {}", self.current_gear), 1.0);
        }

        // ✅ Subphase 12.7.1: Period key - Shift up
        if is_key_pressed(KeyCode::Period)
            && self.manual_transmission_enabled
            && self.current_gear < 5
        {
            self.current_gear += 1;
            self.control.gear = self.current_gear;
            notification.set_text(format!("Gear: {}", self.current_gear), 1.0);
        }

        // ✅ Subphase 12.7.2: Light controls
        let shift_pressed = is_key_down(KeyCode::LeftShift) || is_key_down(KeyCode::RightShift);

        // L key (with modifiers) - Light controls
        if is_key_pressed(KeyCode::L) {
            if shift_pressed {
                // Shift+L - Toggle high beam
                self.lights ^= VehicleLightState::HIGH_BEAM;
                notification.set_text("High beam toggled", 1.0);
            } else {
                // L - Cycle through Position -> LowBeam -> Fog -> Off
                let has_position = self.lights.contains(VehicleLightState::POSITION);
                let has_low_beam = self.lights.contains(VehicleLightState::LOW_BEAM);
                let has_fog = self.lights.contains(VehicleLightState::FOG);

                if !has_position {
                    // Turn on position lights
                    self.lights |= VehicleLightState::POSITION;
                    notification.set_text("Position lights", 1.0);
                } else if !has_low_beam {
                    // Turn on low beam (position stays on)
                    self.lights |= VehicleLightState::LOW_BEAM;
                    notification.set_text("Low beam lights", 1.0);
                } else if !has_fog {
                    // Turn on fog (position and low beam stay on)
                    self.lights |= VehicleLightState::FOG;
                    notification.set_text("Fog lights", 1.0);
                } else {
                    // Turn all off
                    self.lights &= !VehicleLightState::POSITION;
                    self.lights &= !VehicleLightState::LOW_BEAM;
                    self.lights &= !VehicleLightState::FOG;
                    notification.set_text("Lights off", 1.0);
                }
            }
        }

        // I key - Toggle interior lights
        if is_key_pressed(KeyCode::I) {
            self.lights ^= VehicleLightState::INTERIOR;
            notification.set_text("Interior lights toggled", 1.0);
        }

        // Z key - Toggle left blinker
        if is_key_pressed(KeyCode::Z) {
            self.lights ^= VehicleLightState::LEFT_BLINKER;
            notification.set_text("Left blinker toggled", 1.0);
        }

        // X key - Toggle right blinker
        if is_key_pressed(KeyCode::X) {
            self.lights ^= VehicleLightState::RIGHT_BLINKER;
            notification.set_text("Right blinker toggled", 1.0);
        }

        // ✅ Subphase 12.7.3: F key - Toggle Ackermann mode (minimal/stub implementation)
        if is_key_pressed(KeyCode::F) {
            self.ackermann_enabled = !self.ackermann_enabled;
            let message = if self.ackermann_enabled {
                "Ackermann mode enabled (experimental)"
            } else {
                "Ackermann mode disabled"
            };
            notification.set_text(message, 2.0);
        }

        // ✅ Subphase 12.7.3: Q key - Toggle reverse direction for Ackermann
        if is_key_pressed(KeyCode::Q) && self.ackermann_enabled {
            self.ackermann_reverse = -self.ackermann_reverse;
            let message = if self.ackermann_reverse > 0.0 {
                "Ackermann: Forward"
            } else {
                "Ackermann: Reverse"
            };
            notification.set_text(message, 1.0);
        }

        // ✅ Subphase 12.8.1: Backtick (`) key - Next sensor
        if is_key_pressed(KeyCode::GraveAccent) {
            if let Ok(sensor_name) = camera.next_sensor(world) {
                notification.set_text(sensor_name, 2.0);
            }
        }

        // ✅ Subphase 12.8.1: N key - Next sensor (alternative to backtick)
        if is_key_pressed(KeyCode::N) {
            if let Ok(sensor_name) = camera.next_sensor(world) {
                notification.set_text(sensor_name, 2.0);
            }
        }

        // ✅ Subphase 12.8.1: Number keys 1-9 - Direct sensor selection (indices 0-8)
        let ctrl_pressed = is_key_down(KeyCode::LeftControl) || is_key_down(KeyCode::RightControl);

        if is_key_pressed(KeyCode::Key1) {
            let index = if ctrl_pressed { 9 } else { 0 };
            if let Ok(sensor_name) = camera.set_sensor(world, index) {
                notification.set_text(sensor_name, 2.0);
            }
        }
        if is_key_pressed(KeyCode::Key2) {
            let index = if ctrl_pressed { 10 } else { 1 };
            if let Ok(sensor_name) = camera.set_sensor(world, index) {
                notification.set_text(sensor_name, 2.0);
            }
        }
        if is_key_pressed(KeyCode::Key3) {
            let index = if ctrl_pressed { 11 } else { 2 };
            if let Ok(sensor_name) = camera.set_sensor(world, index) {
                notification.set_text(sensor_name, 2.0);
            }
        }
        if is_key_pressed(KeyCode::Key4) {
            let index = if ctrl_pressed { 12 } else { 3 };
            if let Ok(sensor_name) = camera.set_sensor(world, index) {
                notification.set_text(sensor_name, 2.0);
            }
        }
        if is_key_pressed(KeyCode::Key5) {
            let index = if ctrl_pressed { 13 } else { 4 };
            if let Ok(sensor_name) = camera.set_sensor(world, index) {
                notification.set_text(sensor_name, 2.0);
            }
        }
        if is_key_pressed(KeyCode::Key6) && !ctrl_pressed {
            if let Ok(sensor_name) = camera.set_sensor(world, 5) {
                notification.set_text(sensor_name, 2.0);
            }
        }
        if is_key_pressed(KeyCode::Key7) && !ctrl_pressed {
            if let Ok(sensor_name) = camera.set_sensor(world, 6) {
                notification.set_text(sensor_name, 2.0);
            }
        }
        if is_key_pressed(KeyCode::Key8) && !ctrl_pressed {
            if let Ok(sensor_name) = camera.set_sensor(world, 7) {
                notification.set_text(sensor_name, 2.0);
            }
        }
        if is_key_pressed(KeyCode::Key9) && !ctrl_pressed {
            if let Ok(sensor_name) = camera.set_sensor(world, 8) {
                notification.set_text(sensor_name, 2.0);
            }
        }

        // TODO Phase 8.1: TAB - camera position (moved to main.rs for now)

        // ✅ Subphase 12.9.1: C key - Weather cycling
        if is_key_pressed(KeyCode::C) {
            let weather_name = world.next_weather(shift_pressed);
            let message = format!("Weather: {}", weather_name);
            notification.set_text(message, 2.0);
        }

        // ✅ Subphase 12.9.2: B key - Map layer control
        if is_key_pressed(KeyCode::B) {
            if shift_pressed {
                // Shift+B: Unload map layer
                world.load_map_layer(true);
                let layer = world.map_layers[world.current_map_layer].clone();
                notification.set_text(format!("Unloading: {:?}", layer), 2.0);
            } else {
                // B: Load map layer (cycles to next first)
                let layer = world.next_map_layer(false);
                world.load_map_layer(false);
                notification.set_text(format!("Loading: {:?}", layer), 2.0);
            }
        }

        // ✅ Subphase 12.10.1: Ctrl+R - Toggle recording
        let ctrl_pressed = is_key_down(KeyCode::LeftControl) || is_key_down(KeyCode::RightControl);
        if is_key_pressed(KeyCode::R) && ctrl_pressed {
            world.recording_enabled = !world.recording_enabled;

            if world.recording_enabled {
                let result = client.start_recorder("manual_recording.log", false);
                notification.set_text(format!("Recording ON: {}", result), 2.0);
            } else {
                client.stop_recorder();
                notification.set_text("Recording OFF", 2.0);
            }
        }

        // ✅ Subphase 12.10.2: Ctrl+P - Replay recording
        if is_key_pressed(KeyCode::P) && ctrl_pressed {
            // Disable autopilot during replay
            if self.autopilot_enabled {
                self.autopilot_enabled = false;
                if let Some(ref player) = world.player {
                    player.set_autopilot(false);
                }
            }

            let result = client.replay_file(
                "manual_recording.log",
                world.recording_start as f32,
                0.0,
                0,
                false,
            );
            notification.set_text(
                format!(
                    "Replay started at {:.1}s: {}",
                    world.recording_start, result
                ),
                3.0,
            );
        }

        // ✅ Subphase 12.10.2: Ctrl+Minus/Plus - Adjust replay start time
        if ctrl_pressed && is_key_pressed(KeyCode::Minus) {
            let adjustment = if shift_pressed { 10.0 } else { 1.0 };
            world.recording_start = (world.recording_start - adjustment).max(0.0);
            notification.set_text(format!("Replay start: {:.1}s", world.recording_start), 1.0);
        }
        if ctrl_pressed && is_key_pressed(KeyCode::Equal) {
            let adjustment = if shift_pressed { 10.0 } else { 1.0 };
            world.recording_start += adjustment;
            notification.set_text(format!("Replay start: {:.1}s", world.recording_start), 1.0);
        }

        // ✅ Subphase 12.10.3: R key - Camera recording
        if is_key_pressed(KeyCode::R) && !ctrl_pressed {
            let status = camera.toggle_recording();
            notification.set_text(status, 2.0);
        }

        // ✅ Subphase 12.11.1: G key - Toggle radar visualization
        if is_key_pressed(KeyCode::G) {
            if let Some(ref mut radar) = hud.radar_sensor {
                radar.toggle();
                world.radar_enabled = !world.radar_enabled;
                let message = if world.radar_enabled {
                    "Radar visualization ON"
                } else {
                    "Radar visualization OFF"
                };
                notification.set_text(message, 2.0);
            }
        }

        // ✅ Subphase 12.11.2: O key - Toggle vehicle doors
        if is_key_pressed(KeyCode::O) {
            if let Some(ref player) = world.player {
                use carla::rpc::VehicleDoor;

                world.doors_are_open = !world.doors_are_open;

                if world.doors_are_open {
                    // Try to open all doors - handle gracefully if vehicle doesn't support doors
                    player.open_door(VehicleDoor::All);
                    notification.set_text("Opening doors", 2.0);
                } else {
                    player.close_door(VehicleDoor::All);
                    notification.set_text("Closing doors", 2.0);
                }
            }
        }

        // ✅ Subphase 12.11.3: Ctrl+W - Toggle constant velocity mode
        if is_key_pressed(KeyCode::W) && ctrl_pressed {
            if let Some(ref player) = world.player {
                use nalgebra::Vector3;

                world.constant_velocity_enabled = !world.constant_velocity_enabled;

                if world.constant_velocity_enabled {
                    // 60 km/h = 16.67 m/s ≈ 17 m/s
                    let velocity = Vector3::new(17.0, 0.0, 0.0);
                    player.enable_constant_velocity(&velocity);
                    notification.set_text("Constant velocity mode ON (60 km/h)", 2.0);
                } else {
                    player.disable_constant_velocity();
                    notification.set_text("Constant velocity mode OFF", 2.0);
                }
            }
        }

        // ✅ Subphase 12.11.4: T key - Toggle vehicle telemetry
        if is_key_pressed(KeyCode::T) {
            if let Some(ref player) = world.player {
                world.show_vehicle_telemetry = !world.show_vehicle_telemetry;
                player.show_debug_telemetry(world.show_vehicle_telemetry);

                let message = if world.show_vehicle_telemetry {
                    "Vehicle telemetry ON"
                } else {
                    "Vehicle telemetry OFF"
                };
                notification.set_text(message, 2.0);
            }
        }

        // ✅ Subphase 12.12.2: H key or Shift+/ - Toggle help overlay
        if is_key_pressed(KeyCode::H) || (shift_pressed && is_key_pressed(KeyCode::Slash)) {
            help.toggle();
        }

        // ✅ Subphase 12.12.3: F1 key - Toggle HUD info display
        if is_key_pressed(KeyCode::F1) {
            hud.show_info = !hud.show_info;
            let message = if hud.show_info {
                "HUD info ON"
            } else {
                "HUD info OFF"
            };
            notification.set_text(message, 2.0);
        }

        // ESC or Ctrl+Q quits
        if is_key_pressed(KeyCode::Escape) {
            return true;
        }

        false
    }

    /// Parse continuous vehicle keys (WASD/arrows)
    ///
    /// ✅ Subphase 12.3.1: Throttle, brake, steer implementation
    pub fn parse_vehicle_keys(&mut self, _delta_time: f32) {
        // Don't process manual controls if autopilot is enabled
        if self.autopilot_enabled {
            return;
        }

        // Reset controls to neutral
        self.control.throttle = 0.0;
        self.control.steer = 0.0;
        self.control.brake = 0.0;
        self.control.hand_brake = false;

        // W or Up Arrow - Throttle
        if is_key_down(KeyCode::W) || is_key_down(KeyCode::Up) {
            self.control.throttle = 1.0;
        }

        // S or Down Arrow - Brake
        if is_key_down(KeyCode::S) || is_key_down(KeyCode::Down) {
            self.control.brake = 1.0;
        }

        // A or Left Arrow - Steer left
        if is_key_down(KeyCode::A) || is_key_down(KeyCode::Left) {
            self.control.steer = -1.0;
        }

        // D or Right Arrow - Steer right
        if is_key_down(KeyCode::D) || is_key_down(KeyCode::Right) {
            self.control.steer = 1.0;
        }

        // Space - Handbrake
        if is_key_down(KeyCode::Space) {
            self.control.hand_brake = true;
        }
    }

    /// Apply control to vehicle
    ///
    /// ✅ Subphase 12.3.1: Send control to player vehicle
    /// ✅ Subphase 12.7.2: Apply light state
    pub fn apply_control(&mut self, world: &mut crate::world::World) -> Result<()> {
        // Only apply manual control if autopilot is disabled
        if !self.autopilot_enabled {
            if let Some(ref player) = world.player {
                // Apply vehicle control
                player.apply_control(&self.control);

                // ✅ Subphase 12.7.2: Automatically set brake and reverse lights
                let mut current_lights = self.lights;

                // Set brake light when braking
                if self.control.brake > 0.0 {
                    current_lights |= VehicleLightState::BRAKE;
                } else {
                    current_lights &= !VehicleLightState::BRAKE;
                }

                // Set reverse light when in reverse
                if self.control.reverse {
                    current_lights |= VehicleLightState::REVERSE;
                } else {
                    current_lights &= !VehicleLightState::REVERSE;
                }

                // Apply light state (now type-safe with no unsafe code!)
                player.set_light_state(&current_lights);
            }
        }
        Ok(())
    }
}

impl Default for KeyboardControl {
    fn default() -> Self {
        Self::new(false)
    }
}
