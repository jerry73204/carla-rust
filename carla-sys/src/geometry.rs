//! Basic constructors for CARLA geometry types.

use crate::ffi::{
    SimpleBoundingBox, SimpleLocation, SimpleRotation, SimpleTransform, SimpleVector2D,
    SimpleVector3D,
};

// Basic constructors for FFI types
impl SimpleVector2D {
    pub fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }
}

impl SimpleVector3D {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }
}

impl SimpleLocation {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }
}

impl SimpleRotation {
    pub const ZERO: Self = Self {
        pitch: 0.0,
        yaw: 0.0,
        roll: 0.0,
    };

    pub fn new(pitch: f64, yaw: f64, roll: f64) -> Self {
        Self { pitch, yaw, roll }
    }
}

impl SimpleTransform {
    pub fn new(location: SimpleLocation, rotation: SimpleRotation) -> Self {
        Self { location, rotation }
    }
}

impl SimpleBoundingBox {
    pub fn new(location: SimpleLocation, extent: SimpleVector3D) -> Self {
        Self { location, extent }
    }
}
