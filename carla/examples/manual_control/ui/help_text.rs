//! Help text overlay
//!
//! Displays a full-screen overlay with all keyboard controls.
//!
//! ## Phase 13 Implementation
//!
//! - Phase 12.2: Help overlay with all 40+ key bindings
//! - Phase 12.2: Toggle with H or Shift+/

use macroquad::prelude::*;

/// Help text overlay
///
/// Shows all keyboard controls in a centered overlay
pub struct HelpText {
    pub visible: bool,
    pub width: f32,
    pub height: f32,
}

impl HelpText {
    /// Create a new help overlay
    pub fn new(screen_width: f32, screen_height: f32) -> Self {
        Self {
            visible: false,
            width: screen_width,
            height: screen_height,
        }
    }

    /// Toggle help visibility
    ///
    /// TODO Phase 12.2: H key or Shift+/ functionality
    pub fn toggle(&mut self) {
        self.visible = !self.visible;
    }

    /// Render help overlay
    ///
    /// TODO Phase 12.2: Draw centered overlay with all controls
    pub fn render(&self) {
        if !self.visible {
            return;
        }

        // TODO: Draw semi-transparent background (alpha 220)
        // TODO: Draw help text in columns:
        //   BASIC CONTROLS:
        //     W/Up - Throttle
        //     S/Down - Brake
        //     A/Left, D/Right - Steer
        //     Space - Handbrake
        //     P - Toggle autopilot
        //   CAMERA:
        //     TAB - Camera position
        //     `, N - Next sensor
        //     1-9 - Select sensor
        //     R - Toggle recording
        //   TRANSMISSION:
        //     M - Manual/Automatic
        //     , . - Shift up/down
        //     Q - Reverse
        //   LIGHTS:
        //     L - Cycle lights
        //     Shift+L - High beam
        //     Ctrl+L - Special lights
        //     I - Interior
        //     Z, X - Turn signals
        //   WEATHER:
        //     C, Shift+C - Change weather
        //   MAP:
        //     B, Shift+B - Map layers
        //   RECORDING:
        //     Ctrl+R - Record/Stop
        //     Ctrl+P - Replay
        //     Ctrl+-, Ctrl++ - Adjust replay time
        //   ADVANCED:
        //     F - Ackermann mode
        //     G - Toggle radar
        //     O - Toggle doors
        //     T - Vehicle telemetry
        //     Ctrl+W - Constant velocity
        //   UI:
        //     F1 - Toggle HUD
        //     H, Shift+/ - This help
        //     ESC - Quit
    }
}

impl Default for HelpText {
    fn default() -> Self {
        Self::new(1280.0, 720.0)
    }
}
