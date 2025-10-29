//! Fading text notification system
//!
//! Displays temporary notifications that fade out after a configurable duration.
//!
//! ## Phase 13 Implementation
//!
//! - Phase 12.1: Notification system with alpha fade
//! - Phase 12.1: 2-second display with smooth fade out

use macroquad::prelude::*;

/// Fading text notification
///
/// Displays a temporary message at the bottom of the screen that fades out
pub struct FadingText {
    pub text: String,
    pub color: Color,
    pub seconds_left: f32,
    pub max_seconds: f32,
    pub x: f32,
    pub y: f32,
}

impl FadingText {
    /// Create a new fading text display
    pub fn new(x: f32, y: f32) -> Self {
        Self {
            text: String::new(),
            color: WHITE,
            seconds_left: 0.0,
            max_seconds: 2.0,
            x,
            y,
        }
    }

    /// Set text to display
    ///
    /// TODO Phase 12.1: Called from keyboard/world events
    pub fn set_text(&mut self, text: impl Into<String>, seconds: f32) {
        self.text = text.into();
        self.seconds_left = seconds;
        self.max_seconds = seconds;
        self.color = WHITE;
    }

    /// Set text with custom color
    pub fn set_text_colored(&mut self, text: impl Into<String>, color: Color, seconds: f32) {
        self.text = text.into();
        self.seconds_left = seconds;
        self.max_seconds = seconds;
        self.color = color;
    }

    /// Update fade timer
    ///
    /// TODO Phase 12.1: Called each frame
    pub fn update(&mut self, delta_time: f32) {
        if self.seconds_left > 0.0 {
            self.seconds_left -= delta_time;
            if self.seconds_left < 0.0 {
                self.seconds_left = 0.0;
            }
        }
    }

    /// Render notification
    ///
    /// TODO Phase 12.1: Draw with alpha based on seconds_left
    pub fn render(&self) {
        if self.seconds_left <= 0.0 || self.text.is_empty() {
            return;
        }

        // Calculate alpha fade (0.0 = transparent, 1.0 = opaque)
        let alpha = (self.seconds_left / self.max_seconds * 500.0).min(1.0);
        let color_with_alpha = Color::new(self.color.r, self.color.g, self.color.b, alpha);

        // TODO: Draw semi-transparent background
        // TODO: Draw text with faded color
        let _ = color_with_alpha; // Use it to avoid warning
    }
}

impl Default for FadingText {
    fn default() -> Self {
        Self::new(20.0, 680.0) // Bottom-left corner
    }
}
