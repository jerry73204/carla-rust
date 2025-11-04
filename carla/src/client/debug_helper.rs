//! Debug drawing utilities for visualizing simulation data.
//!
//! The [`DebugHelper`] provides methods to draw geometric primitives (points, lines,
//! boxes) and text in the CARLA simulator for debugging and visualization purposes.
//!
//! # Examples
//!
//! ```no_run
//! # use carla::client::Client;
//! # use carla::geom::{Location, Rotation};
//! # use carla::rpc::Color;
//! #
//! # let mut client = Client::connect("localhost", 2000, None).unwrap();
//! # let mut world = client.world();
//! let debug = world.debug();
//!
//! // Draw a red point at origin
//! debug.draw_point(Location::new(0.0, 0.0, 0.0), 0.5, Color::RED, 5.0, false);
//!
//! // Draw a green line
//! debug.draw_line(
//!     Location::new(0.0, 0.0, 0.0),
//!     Location::new(10.0, 0.0, 0.0),
//!     0.1,
//!     Color::GREEN,
//!     5.0,
//!     false,
//! );
//!
//! // Draw a text label
//! debug.draw_string(
//!     Location::new(0.0, 0.0, 5.0),
//!     "Hello, CARLA!",
//!     false,
//!     Color::WHITE,
//!     5.0,
//!     false,
//! );
//! ```

use crate::{
    geom::{BoundingBox, FfiRotation, Location, Rotation},
    rpc::Color,
};
use carla_sys::carla_rust::client::{
    FfiDebugHelper, FfiDebugHelper_DrawArrow, FfiDebugHelper_DrawBox, FfiDebugHelper_DrawLine,
    FfiDebugHelper_DrawPoint, FfiDebugHelper_DrawString,
};

/// Debug drawing helper for visualizing simulation data.
///
/// Provides methods to draw geometric primitives, arrows, and text in the CARLA
/// simulator. All drawing operations are sent to the server and rendered in the
/// simulation viewport.
///
/// Drawing parameters:
/// - `life_time`: Duration in seconds the drawing persists (0.0 = one frame)
/// - `persistent_lines`: If true, drawing persists until manually cleared
///
/// Obtain via [`crate::client::World::debug`].
pub struct DebugHelper {
    pub(crate) inner: cxx::UniquePtr<FfiDebugHelper>,
}

impl DebugHelper {
    /// Draws a point in the simulation.
    ///
    /// # Arguments
    ///
    /// * `location` - World position of the point
    /// * `size` - Size of the point in meters
    /// * `color` - Color of the point
    /// * `life_time` - Duration in seconds (0.0 = one frame)
    /// * `persistent_lines` - Whether the point persists until cleared
    ///
    /// # Examples
    ///
    /// ```no_run
    /// # use carla::client::Client;
    /// # use carla::geom::Location;
    /// # use carla::rpc::Color;
    /// #
    /// # let mut client = Client::connect("localhost", 2000, None).unwrap();
    /// # let mut world = client.world();
    /// let debug = world.debug();
    ///
    /// // Draw a large red point at origin for 5 seconds
    /// debug.draw_point(Location::new(0.0, 0.0, 0.0), 1.0, Color::RED, 5.0, false);
    /// ```
    pub fn draw_point(
        &self,
        location: Location,
        size: f32,
        color: Color,
        life_time: f32,
        persistent_lines: bool,
    ) {
        let ffi_color = color.into();
        FfiDebugHelper_DrawPoint(
            &self.inner,
            location.as_ffi(),
            size,
            &ffi_color,
            life_time,
            persistent_lines,
        )
    }

    /// Draws a line between two points.
    ///
    /// # Arguments
    ///
    /// * `begin` - Start position of the line
    /// * `end` - End position of the line
    /// * `thickness` - Line thickness in centimeters
    /// * `color` - Color of the line
    /// * `life_time` - Duration in seconds (0.0 = one frame)
    /// * `persistent_lines` - Whether the line persists until cleared
    ///
    /// # Examples
    ///
    /// ```no_run
    /// # use carla::client::Client;
    /// # use carla::geom::Location;
    /// # use carla::rpc::Color;
    /// #
    /// # let mut client = Client::connect("localhost", 2000, None).unwrap();
    /// # let mut world = client.world();
    /// let debug = world.debug();
    ///
    /// // Draw a green line along the X axis
    /// debug.draw_line(
    ///     Location::new(0.0, 0.0, 0.0),
    ///     Location::new(10.0, 0.0, 0.0),
    ///     10.0,
    ///     Color::GREEN,
    ///     5.0,
    ///     false,
    /// );
    /// ```
    pub fn draw_line(
        &self,
        begin: Location,
        end: Location,
        thickness: f32,
        color: Color,
        life_time: f32,
        persistent_lines: bool,
    ) {
        let ffi_color = color.into();
        FfiDebugHelper_DrawLine(
            &self.inner,
            begin.as_ffi(),
            end.as_ffi(),
            thickness,
            &ffi_color,
            life_time,
            persistent_lines,
        )
    }

    /// Draws an arrow from begin to end.
    ///
    /// # Arguments
    ///
    /// * `begin` - Start position of the arrow
    /// * `end` - End position (arrow points here)
    /// * `thickness` - Arrow line thickness in centimeters
    /// * `arrow_size` - Size of the arrowhead in centimeters
    /// * `color` - Color of the arrow
    /// * `life_time` - Duration in seconds (0.0 = one frame)
    /// * `persistent_lines` - Whether the arrow persists until cleared
    ///
    /// # Examples
    ///
    /// ```no_run
    /// # use carla::client::Client;
    /// # use carla::geom::Location;
    /// # use carla::rpc::Color;
    /// #
    /// # let mut client = Client::connect("localhost", 2000, None).unwrap();
    /// # let mut world = client.world();
    /// let debug = world.debug();
    ///
    /// // Draw a blue arrow pointing up
    /// debug.draw_arrow(
    ///     Location::new(0.0, 0.0, 0.0),
    ///     Location::new(0.0, 0.0, 5.0),
    ///     10.0,
    ///     20.0,
    ///     Color::BLUE,
    ///     5.0,
    ///     false,
    /// );
    /// ```
    #[allow(clippy::too_many_arguments)]
    pub fn draw_arrow(
        &self,
        begin: Location,
        end: Location,
        thickness: f32,
        arrow_size: f32,
        color: Color,
        life_time: f32,
        persistent_lines: bool,
    ) {
        let ffi_color = color.into();
        FfiDebugHelper_DrawArrow(
            &self.inner,
            begin.as_ffi(),
            end.as_ffi(),
            thickness,
            arrow_size,
            &ffi_color,
            life_time,
            persistent_lines,
        )
    }

    /// Draws a bounding box.
    ///
    /// # Arguments
    ///
    /// * `bbox` - The bounding box to draw
    /// * `rotation` - Rotation of the box
    /// * `thickness` - Line thickness in centimeters
    /// * `color` - Color of the box
    /// * `life_time` - Duration in seconds (0.0 = one frame)
    /// * `persistent_lines` - Whether the box persists until cleared
    ///
    /// # Examples
    ///
    /// ```no_run
    /// # use carla::client::Client;
    /// # use carla::geom::{BoundingBox, Location, Rotation, Vector3D};
    /// # use carla::rpc::Color;
    /// #
    /// # let mut client = Client::connect("localhost", 2000, None).unwrap();
    /// # let mut world = client.world();
    /// let debug = world.debug();
    ///
    /// // Draw a rotated bounding box
    /// let bbox = BoundingBox::new(Location::new(0.0, 0.0, 1.0), Vector3D::new(2.0, 1.0, 1.0));
    /// debug.draw_box(
    ///     &bbox,
    ///     Rotation::new(0.0, 45.0, 0.0),
    ///     10.0,
    ///     Color::YELLOW,
    ///     5.0,
    ///     false,
    /// );
    /// ```
    pub fn draw_box(
        &self,
        bbox: &BoundingBox,
        rotation: Rotation,
        thickness: f32,
        color: Color,
        life_time: f32,
        persistent_lines: bool,
    ) {
        let ffi_color = color.into();
        let native_bbox = bbox.to_native();
        // SAFETY: FfiRotation and carla::geom::Rotation have identical memory layout
        let cpp_rotation = unsafe {
            &*(rotation.as_ffi() as *const FfiRotation as *const carla_sys::carla::geom::Rotation)
        };
        FfiDebugHelper_DrawBox(
            &self.inner,
            &native_bbox,
            cpp_rotation,
            thickness,
            &ffi_color,
            life_time,
            persistent_lines,
        )
    }

    /// Draws a text string at a location.
    ///
    /// # Arguments
    ///
    /// * `location` - World position for the text
    /// * `text` - The text to display
    /// * `draw_shadow` - Whether to draw a shadow behind the text
    /// * `color` - Color of the text
    /// * `life_time` - Duration in seconds (0.0 = one frame)
    /// * `persistent_lines` - Whether the text persists until cleared
    ///
    /// # Examples
    ///
    /// ```no_run
    /// # use carla::client::Client;
    /// # use carla::geom::Location;
    /// # use carla::rpc::Color;
    /// #
    /// # let mut client = Client::connect("localhost", 2000, None).unwrap();
    /// # let mut world = client.world();
    /// let debug = world.debug();
    ///
    /// // Draw text label with shadow
    /// debug.draw_string(
    ///     Location::new(0.0, 0.0, 5.0),
    ///     "Spawn Point",
    ///     true,
    ///     Color::WHITE,
    ///     5.0,
    ///     false,
    /// );
    /// ```
    pub fn draw_string(
        &self,
        location: Location,
        text: &str,
        draw_shadow: bool,
        color: Color,
        life_time: f32,
        persistent_lines: bool,
    ) {
        let ffi_color = color.into();
        cxx::let_cxx_string!(text_cxx = text);
        FfiDebugHelper_DrawString(
            &self.inner,
            location.as_ffi(),
            &text_cxx,
            draw_shadow,
            &ffi_color,
            life_time,
            persistent_lines,
        )
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn test_debug_helper_methods_compile() {
        // This test ensures the API compiles correctly
        // Actual functionality tests require a running CARLA simulator
    }
}
