//! Walker control commands.

use crate::geom::Vector3D;

/// Walker (pedestrian) control commands.
#[derive(Debug, Clone, PartialEq)]
pub struct WalkerControl {
    /// Direction vector (normalized)
    pub direction: Vector3D,
    /// Speed in m/s
    pub speed: f32,
    /// Jump command
    pub jump: bool,
}

impl Default for WalkerControl {
    fn default() -> Self {
        Self {
            direction: Vector3D::zero(),
            speed: 0.0,
            jump: false,
        }
    }
}
