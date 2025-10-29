//! Keyboard input handling
//!
//! The KeyboardControl struct handles:
//! - Event-based input (key press/release)
//! - Continuous input (key held down)
//! - Vehicle control (throttle, brake, steer, gear, handbrake)
//! - Walker control (speed, direction, jump)
//! - Ackermann steering mode
//! - Light state management
//!
//! ## Phase 13 Implementation
//!
//! - Phase 3: Basic vehicle control (WASD, autopilot toggle)
//! - Phase 7: Advanced controls (manual transmission, lights, Ackermann)
//! - Phase 8: Camera switching
//! - Phase 9: Weather and map layer control
//! - Phase 10: Recording controls

use carla::rpc::VehicleControl;
use eyre::Result;
use macroquad::prelude::*;

/// Keyboard input controller
///
/// Handles all keyboard input for vehicle control and UI
pub struct KeyboardControl {
    pub control: VehicleControl,
    pub autopilot_enabled: bool,
    // TODO Phase 3: Add steering cache for smooth steering
    // steer_cache: f32,

    // TODO Phase 7.2: Add light state
    // lights: u32,  // VehicleLightState bitflags

    // TODO Phase 7.3: Add Ackermann controller state
    // ackermann_enabled: bool,
    // ackermann_control: VehicleAckermannControl,
    // ackermann_reverse: f32,
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
        }
    }

    /// Parse keyboard events (key press/release)
    ///
    /// TODO Phase 3-11: Implement all keyboard shortcuts
    /// Returns true if should quit
    pub fn parse_events(
        &mut self,
        _world: &mut crate::world::World,
        _hud: &mut crate::hud::HUD,
        _camera: &mut crate::camera::CameraManager,
    ) -> bool {
        // TODO Phase 3.2: P key - toggle autopilot
        // TODO Phase 7.1: M key - manual transmission
        // TODO Phase 7.1: Comma/Period - shift gears
        // TODO Phase 7.2: L/I/Z/X keys - lights
        // TODO Phase 7.3: F key - Ackermann mode
        // TODO Phase 7.3: Q key - reverse direction
        // TODO Phase 8.1: TAB - camera position
        // TODO Phase 8.1: Backtick/N - next sensor
        // TODO Phase 8.1: Number keys - select sensor
        // TODO Phase 9.1: C/Shift+C - weather
        // TODO Phase 9.2: B/Shift+B - map layers
        // TODO Phase 10.1: Ctrl+R - recording
        // TODO Phase 10.2: Ctrl+P - replay
        // TODO Phase 10.2: Ctrl+Minus/Plus - replay time
        // TODO Phase 10.3: R - camera recording
        // TODO Phase 11: G - radar, O - doors, Ctrl+W - velocity, T - telemetry
        // TODO Phase 12: F1 - toggle HUD, H - help

        // ESC or Ctrl+Q quits
        if is_key_pressed(KeyCode::Escape) {
            return true;
        }

        false
    }

    /// Parse continuous vehicle keys (WASD/arrows)
    ///
    /// TODO Phase 3.1: Implement throttle, brake, steer
    pub fn parse_vehicle_keys(&mut self, _delta_time: f32) {
        // TODO: W/Up - throttle
        // TODO: S/Down - brake
        // TODO: A/Left - steer left
        // TODO: D/Right - steer right
        // TODO: Space - handbrake
    }

    /// Apply control to vehicle
    ///
    /// TODO Phase 3.1: Send control to player vehicle
    pub fn apply_control(&mut self, _world: &mut crate::world::World) -> Result<()> {
        // TODO: If not autopilot, apply self.control or ackermann_control
        Ok(())
    }
}

impl Default for KeyboardControl {
    fn default() -> Self {
        Self::new(false)
    }
}
