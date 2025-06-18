//! Debug visualization utilities for CARLA simulator.
//!
//! This module provides high-level utilities for drawing debug shapes in the CARLA world,
//! making it easier to visualize paths, waypoints, bounding boxes, and other debugging information.

use crate::{
    ffi::bridge::{
        SimpleColor, SimpleDebugArrow, SimpleDebugBox, SimpleDebugLine, SimpleDebugPoint,
        SimpleDebugString,
    },
    SimpleBoundingBox, SimpleLocation, SimpleRotation, SimpleTransform, SimpleVector3D,
    WorldWrapper,
};

/// Commonly used debug colors
pub mod colors {
    use crate::ffi::bridge::SimpleColor;

    pub const RED: SimpleColor = SimpleColor { r: 255, g: 0, b: 0 };
    pub const GREEN: SimpleColor = SimpleColor { r: 0, g: 255, b: 0 };
    pub const BLUE: SimpleColor = SimpleColor { r: 0, g: 0, b: 255 };
    pub const YELLOW: SimpleColor = SimpleColor {
        r: 255,
        g: 255,
        b: 0,
    };
    pub const CYAN: SimpleColor = SimpleColor {
        r: 0,
        g: 255,
        b: 255,
    };
    pub const MAGENTA: SimpleColor = SimpleColor {
        r: 255,
        g: 0,
        b: 255,
    };
    pub const WHITE: SimpleColor = SimpleColor {
        r: 255,
        g: 255,
        b: 255,
    };
    pub const BLACK: SimpleColor = SimpleColor { r: 0, g: 0, b: 0 };
    pub const ORANGE: SimpleColor = SimpleColor {
        r: 255,
        g: 165,
        b: 0,
    };
    pub const PURPLE: SimpleColor = SimpleColor {
        r: 128,
        g: 0,
        b: 128,
    };
    pub const PINK: SimpleColor = SimpleColor {
        r: 255,
        g: 192,
        b: 203,
    };
    pub const GRAY: SimpleColor = SimpleColor {
        r: 128,
        g: 128,
        b: 128,
    };
}

/// A builder for creating debug visualizations with fluent API
pub struct DebugDrawBuilder<'a> {
    world: &'a WorldWrapper,
    color: SimpleColor,
    life_time: f32,
    persistent_lines: bool,
    thickness: f32,
}

impl<'a> DebugDrawBuilder<'a> {
    /// Create a new debug draw builder
    pub fn new(world: &'a WorldWrapper) -> Self {
        Self {
            world,
            color: colors::RED,
            life_time: -1.0, // Persistent by default
            persistent_lines: true,
            thickness: 0.1,
        }
    }

    /// Set the color for subsequent drawings
    pub fn color(mut self, color: SimpleColor) -> Self {
        self.color = color;
        self
    }

    /// Set the color using RGB values
    pub fn rgb(mut self, r: u8, g: u8, b: u8) -> Self {
        self.color = SimpleColor { r, g, b };
        self
    }

    /// Set the lifetime in seconds (-1.0 for persistent)
    pub fn life_time(mut self, seconds: f32) -> Self {
        self.life_time = seconds;
        self
    }

    /// Set whether lines persist across frames
    pub fn persistent(mut self, persistent: bool) -> Self {
        self.persistent_lines = persistent;
        self
    }

    /// Set the thickness for lines, arrows, and boxes
    pub fn thickness(mut self, thickness: f32) -> Self {
        self.thickness = thickness;
        self
    }

    /// Draw a point at the specified location
    pub fn point(&self, location: &SimpleLocation, size: f32) {
        let point = SimpleDebugPoint {
            location: *location,
            size,
            color: self.color,
            life_time: self.life_time,
            persistent_lines: self.persistent_lines,
        };
        self.world.draw_debug_point(&point);
    }

    /// Draw a line between two points
    pub fn line(&self, begin: &SimpleLocation, end: &SimpleLocation) {
        let line = SimpleDebugLine {
            begin: *begin,
            end: *end,
            thickness: self.thickness,
            color: self.color,
            life_time: self.life_time,
            persistent_lines: self.persistent_lines,
        };
        self.world.draw_debug_line(&line);
    }

    /// Draw an arrow from begin to end
    pub fn arrow(&self, begin: &SimpleLocation, end: &SimpleLocation, arrow_size: f32) {
        let arrow = SimpleDebugArrow {
            begin: *begin,
            end: *end,
            thickness: self.thickness,
            arrow_size,
            color: self.color,
            life_time: self.life_time,
            persistent_lines: self.persistent_lines,
        };
        self.world.draw_debug_arrow(&arrow);
    }

    /// Draw a box with the specified bounding box and rotation
    pub fn box_shape(&self, bbox: &SimpleBoundingBox, rotation: &SimpleRotation) {
        let box_shape = SimpleDebugBox {
            bbox: *bbox,
            rotation: *rotation,
            thickness: self.thickness,
            color: self.color,
            life_time: self.life_time,
            persistent_lines: self.persistent_lines,
        };
        self.world.draw_debug_box(&box_shape);
    }

    /// Draw text at the specified location
    pub fn text(&self, location: &SimpleLocation, text: &str, draw_shadow: bool) {
        let string = SimpleDebugString {
            location: *location,
            text: text.to_string(),
            draw_shadow,
            color: self.color,
            life_time: self.life_time,
            persistent_lines: self.persistent_lines,
        };
        self.world.draw_debug_string(&string);
    }

    /// Draw a path as a series of connected lines
    pub fn path(&self, points: &[SimpleLocation]) {
        if points.len() < 2 {
            return;
        }

        for window in points.windows(2) {
            self.line(&window[0], &window[1]);
        }
    }

    /// Draw a transform with coordinate axes
    pub fn transform(&self, transform: &SimpleTransform, axis_length: f32) {
        let origin = &transform.location;

        // Calculate axis directions from transform
        let forward = transform_forward_vector(transform);
        let right = transform_right_vector(transform);
        let up = transform_up_vector(transform);

        // Draw X axis (forward) in red
        let x_end = SimpleLocation {
            x: origin.x + forward.x * axis_length as f64,
            y: origin.y + forward.y * axis_length as f64,
            z: origin.z + forward.z * axis_length as f64,
        };
        let x_arrow = SimpleDebugArrow {
            begin: *origin,
            end: x_end,
            thickness: self.thickness,
            arrow_size: axis_length * 0.1,
            color: colors::RED,
            life_time: self.life_time,
            persistent_lines: self.persistent_lines,
        };
        self.world.draw_debug_arrow(&x_arrow);

        // Draw Y axis (right) in green
        let y_end = SimpleLocation {
            x: origin.x + right.x * axis_length as f64,
            y: origin.y + right.y * axis_length as f64,
            z: origin.z + right.z * axis_length as f64,
        };
        let y_arrow = SimpleDebugArrow {
            begin: *origin,
            end: y_end,
            thickness: self.thickness,
            arrow_size: axis_length * 0.1,
            color: colors::GREEN,
            life_time: self.life_time,
            persistent_lines: self.persistent_lines,
        };
        self.world.draw_debug_arrow(&y_arrow);

        // Draw Z axis (up) in blue
        let z_end = SimpleLocation {
            x: origin.x + up.x * axis_length as f64,
            y: origin.y + up.y * axis_length as f64,
            z: origin.z + up.z * axis_length as f64,
        };
        let z_arrow = SimpleDebugArrow {
            begin: *origin,
            end: z_end,
            thickness: self.thickness,
            arrow_size: axis_length * 0.1,
            color: colors::BLUE,
            life_time: self.life_time,
            persistent_lines: self.persistent_lines,
        };
        self.world.draw_debug_arrow(&z_arrow);
    }

    /// Draw a velocity vector
    pub fn velocity(&self, location: &SimpleLocation, velocity: &SimpleVector3D, scale: f32) {
        let end = SimpleLocation {
            x: location.x + velocity.x * scale as f64,
            y: location.y + velocity.y * scale as f64,
            z: location.z + velocity.z * scale as f64,
        };
        self.arrow(location, &end, 0.2);
    }

    /// Draw a sphere approximation using circles
    pub fn sphere(&self, center: &SimpleLocation, radius: f32, segments: u32) {
        // Draw three circles to approximate a sphere
        let angle_step = 2.0 * std::f32::consts::PI / segments as f32;

        // XY plane circle
        for i in 0..segments {
            let angle1 = i as f32 * angle_step;
            let angle2 = (i + 1) as f32 * angle_step;

            let p1 = SimpleLocation {
                x: center.x + (radius * angle1.cos()) as f64,
                y: center.y + (radius * angle1.sin()) as f64,
                z: center.z,
            };
            let p2 = SimpleLocation {
                x: center.x + (radius * angle2.cos()) as f64,
                y: center.y + (radius * angle2.sin()) as f64,
                z: center.z,
            };
            self.line(&p1, &p2);
        }

        // XZ plane circle
        for i in 0..segments {
            let angle1 = i as f32 * angle_step;
            let angle2 = (i + 1) as f32 * angle_step;

            let p1 = SimpleLocation {
                x: center.x + (radius * angle1.cos()) as f64,
                y: center.y,
                z: center.z + (radius * angle1.sin()) as f64,
            };
            let p2 = SimpleLocation {
                x: center.x + (radius * angle2.cos()) as f64,
                y: center.y,
                z: center.z + (radius * angle2.sin()) as f64,
            };
            self.line(&p1, &p2);
        }

        // YZ plane circle
        for i in 0..segments {
            let angle1 = i as f32 * angle_step;
            let angle2 = (i + 1) as f32 * angle_step;

            let p1 = SimpleLocation {
                x: center.x,
                y: center.y + (radius * angle1.cos()) as f64,
                z: center.z + (radius * angle1.sin()) as f64,
            };
            let p2 = SimpleLocation {
                x: center.x,
                y: center.y + (radius * angle2.cos()) as f64,
                z: center.z + (radius * angle2.sin()) as f64,
            };
            self.line(&p1, &p2);
        }
    }
}

// Helper functions for transform calculations
fn transform_forward_vector(transform: &SimpleTransform) -> SimpleVector3D {
    let pitch = transform.rotation.pitch.to_radians();
    let yaw = transform.rotation.yaw.to_radians();

    SimpleVector3D {
        x: pitch.cos() * yaw.cos(),
        y: pitch.cos() * yaw.sin(),
        z: pitch.sin(),
    }
}

fn transform_right_vector(transform: &SimpleTransform) -> SimpleVector3D {
    let yaw = transform.rotation.yaw.to_radians();

    SimpleVector3D {
        x: yaw.sin(),
        y: -yaw.cos(),
        z: 0.0,
    }
}

fn transform_up_vector(transform: &SimpleTransform) -> SimpleVector3D {
    let pitch = transform.rotation.pitch.to_radians();
    let yaw = transform.rotation.yaw.to_radians();
    let roll = transform.rotation.roll.to_radians();

    SimpleVector3D {
        x: -pitch.sin() * yaw.cos() * roll.cos() - yaw.sin() * roll.sin(),
        y: -pitch.sin() * yaw.sin() * roll.cos() + yaw.cos() * roll.sin(),
        z: pitch.cos() * roll.cos(),
    }
}

/// Extension trait for WorldWrapper to add debug drawing functionality
pub trait DebugDrawExt {
    /// Create a debug draw builder for fluent API
    fn debug_draw(&self) -> DebugDrawBuilder;

    /// Quick method to draw a point
    fn debug_point(&self, location: &SimpleLocation, color: SimpleColor, size: f32);

    /// Quick method to draw a line
    fn debug_line(
        &self,
        begin: &SimpleLocation,
        end: &SimpleLocation,
        color: SimpleColor,
        thickness: f32,
    );

    /// Quick method to draw text
    fn debug_text(&self, location: &SimpleLocation, text: &str, color: SimpleColor);
}

impl DebugDrawExt for WorldWrapper {
    fn debug_draw(&self) -> DebugDrawBuilder {
        DebugDrawBuilder::new(self)
    }

    fn debug_point(&self, location: &SimpleLocation, color: SimpleColor, size: f32) {
        let point = SimpleDebugPoint {
            location: *location,
            size,
            color,
            life_time: -1.0,
            persistent_lines: true,
        };
        self.draw_debug_point(&point);
    }

    fn debug_line(
        &self,
        begin: &SimpleLocation,
        end: &SimpleLocation,
        color: SimpleColor,
        thickness: f32,
    ) {
        let line = SimpleDebugLine {
            begin: *begin,
            end: *end,
            thickness,
            color,
            life_time: -1.0,
            persistent_lines: true,
        };
        self.draw_debug_line(&line);
    }

    fn debug_text(&self, location: &SimpleLocation, text: &str, color: SimpleColor) {
        let string = SimpleDebugString {
            location: *location,
            text: text.to_string(),
            draw_shadow: false,
            color,
            life_time: -1.0,
            persistent_lines: true,
        };
        self.draw_debug_string(&string);
    }
}
