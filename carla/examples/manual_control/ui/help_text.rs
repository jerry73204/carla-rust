//! Help text overlay
//!
//! Displays a full-screen overlay with all keyboard controls.
//!
//! ## Phase 13 Implementation
//!
//! - ✅ Phase 12.12.2: Help overlay with all 40+ key bindings
//! - ✅ Phase 12.12.2: Toggle with H or Shift+/

use macroquad::prelude::*;

/// Help text overlay
///
/// Shows all keyboard controls in a centered overlay
#[allow(dead_code)]
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
    #[allow(dead_code)]
    pub fn toggle(&mut self) {
        self.visible = !self.visible;
    }

    /// Render help overlay
    ///
    /// ✅ Subphase 12.12.2: Draw centered overlay with all controls
    pub fn render(&self) {
        if !self.visible {
            return;
        }

        // Semi-transparent background (alpha 220/255 ≈ 0.86)
        let bg_color = Color::new(0.0, 0.0, 0.0, 0.86);
        draw_rectangle(0.0, 0.0, self.width, self.height, bg_color);

        // Centered overlay box (780 pixels wide as per spec)
        let overlay_width = 780.0;
        let overlay_x = (self.width - overlay_width) / 2.0;
        let start_y = 60.0;
        let line_height = 22.0;
        let font_size = 18.0;
        let column_width = overlay_width / 2.0;

        // Title
        let title = "CARLA MANUAL CONTROL - KEYBOARD CONTROLS";
        draw_text(
            title,
            overlay_x + (overlay_width - measure_text(title, None, 24, 1.0).width) / 2.0,
            start_y,
            24.0,
            YELLOW,
        );

        let mut y = start_y + 40.0;

        // Helper function to draw a section
        let mut draw_section = |title: &str, controls: &[(&str, &str)]| {
            draw_text(title, overlay_x, y, font_size, SKYBLUE);
            y += line_height;
            for (key, desc) in controls {
                draw_text(key, overlay_x + 10.0, y, font_size, WHITE);
                draw_text(desc, overlay_x + 180.0, y, font_size, LIGHTGRAY);
                y += line_height;
            }
            y += 5.0; // Extra spacing between sections
        };

        // Left column
        draw_section(
            "BASIC CONTROLS:",
            &[
                ("W / Up", "Throttle"),
                ("S / Down", "Brake"),
                ("A / Left, D / Right", "Steer"),
                ("Space", "Handbrake"),
                ("P", "Toggle autopilot"),
            ],
        );

        draw_section(
            "CAMERA:",
            &[
                ("TAB", "Camera position"),
                ("`, N", "Next sensor"),
                ("1-9, Ctrl+1-5", "Select sensor"),
            ],
        );

        draw_section(
            "TRANSMISSION:",
            &[
                ("M", "Manual/Automatic"),
                (", .", "Shift down/up"),
                ("Q (Ackermann)", "Reverse direction"),
            ],
        );

        // Right column
        let column2_x = overlay_x + column_width;
        y = start_y + 40.0;

        let mut draw_section_col2 = |title: &str, controls: &[(&str, &str)]| {
            draw_text(title, column2_x, y, font_size, SKYBLUE);
            y += line_height;
            for (key, desc) in controls {
                draw_text(key, column2_x + 10.0, y, font_size, WHITE);
                draw_text(desc, column2_x + 180.0, y, font_size, LIGHTGRAY);
                y += line_height;
            }
            y += 5.0;
        };

        draw_section_col2(
            "LIGHTS:",
            &[
                ("L", "Cycle lights"),
                ("Shift+L", "High beam"),
                ("I", "Interior"),
                ("Z, X", "Turn signals"),
            ],
        );

        draw_section_col2(
            "WORLD:",
            &[
                ("C, Shift+C", "Change weather"),
                ("B, Shift+B", "Map layers"),
            ],
        );

        draw_section_col2(
            "RECORDING:",
            &[
                ("Ctrl+R", "Record/Stop"),
                ("Ctrl+P", "Replay"),
                ("Ctrl+-, Ctrl+=", "Adjust replay time"),
                ("R", "Camera recording"),
            ],
        );

        draw_section_col2(
            "ADVANCED:",
            &[
                ("F", "Ackermann mode"),
                ("G", "Toggle radar"),
                ("O", "Toggle doors"),
                ("T", "Vehicle telemetry"),
                ("Ctrl+W", "Constant velocity"),
            ],
        );

        draw_section_col2(
            "UI:",
            &[
                ("F1", "Toggle HUD"),
                ("H, Shift+/", "This help"),
                ("ESC", "Quit"),
            ],
        );

        // Bottom hint
        let hint = "Press H or Shift+/ to close this help";
        draw_text(
            hint,
            overlay_x
                + (overlay_width - measure_text(hint, None, font_size as u16, 1.0).width) / 2.0,
            self.height - 40.0,
            font_size,
            YELLOW,
        );
    }
}

impl Default for HelpText {
    fn default() -> Self {
        Self::new(1280.0, 720.0)
    }
}
