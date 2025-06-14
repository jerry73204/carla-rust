//! Geometry types with mathematical operations for CARLA.

use crate::ffi::{
    SimpleBoundingBox, SimpleLocation, SimpleRotation, SimpleTransform, SimpleVector2D,
    SimpleVector3D,
};

// Vector2D Implementation
impl SimpleVector2D {
    pub const ZERO: Self = Self { x: 0.0, y: 0.0 };
    pub const UNIT_X: Self = Self { x: 1.0, y: 0.0 };
    pub const UNIT_Y: Self = Self { x: 0.0, y: 1.0 };

    pub fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    pub fn length(&self) -> f64 {
        crate::ffi::Vector2D_Length(self)
    }

    pub fn length_squared(&self) -> f64 {
        crate::ffi::bridge::Vector2D_SquaredLength(self)
    }

    pub fn distance_to(&self, other: &Self) -> f64 {
        crate::ffi::bridge::Vector2D_Distance(self, other)
    }

    pub fn distance_squared_to(&self, other: &Self) -> f64 {
        crate::ffi::bridge::Vector2D_DistanceSquared(self, other)
    }

    pub fn dot(&self, other: &Self) -> f64 {
        crate::ffi::bridge::Vector2D_Dot(self, other)
    }

    pub fn normalized(&self) -> Self {
        let len = self.length();
        if len > 0.0 {
            Self {
                x: self.x / len,
                y: self.y / len,
            }
        } else {
            *self
        }
    }

    pub fn scale(&self, factor: f64) -> Self {
        Self {
            x: self.x * factor,
            y: self.y * factor,
        }
    }
}

impl std::ops::Add for SimpleVector2D {
    type Output = Self;
    fn add(self, other: Self) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
        }
    }
}

impl std::ops::Sub for SimpleVector2D {
    type Output = Self;
    fn sub(self, other: Self) -> Self {
        Self {
            x: self.x - other.x,
            y: self.y - other.y,
        }
    }
}

impl std::ops::Mul<f64> for SimpleVector2D {
    type Output = Self;
    fn mul(self, scalar: f64) -> Self {
        Self {
            x: self.x * scalar,
            y: self.y * scalar,
        }
    }
}

// Vector3D Implementation
impl SimpleVector3D {
    pub const ZERO: Self = Self {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };
    pub const UNIT_X: Self = Self {
        x: 1.0,
        y: 0.0,
        z: 0.0,
    };
    pub const UNIT_Y: Self = Self {
        x: 0.0,
        y: 1.0,
        z: 0.0,
    };
    pub const UNIT_Z: Self = Self {
        x: 0.0,
        y: 0.0,
        z: 1.0,
    };

    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    pub fn length(&self) -> f64 {
        crate::ffi::bridge::Vector3D_Length(self)
    }

    pub fn length_squared(&self) -> f64 {
        crate::ffi::bridge::Vector3D_SquaredLength(self)
    }

    pub fn distance_to(&self, other: &Self) -> f64 {
        crate::ffi::bridge::Vector3D_Distance(self, other)
    }

    pub fn distance_squared_to(&self, other: &Self) -> f64 {
        crate::ffi::bridge::Vector3D_DistanceSquared(self, other)
    }

    pub fn dot(&self, other: &Self) -> f64 {
        crate::ffi::bridge::Vector3D_Dot(self, other)
    }

    pub fn cross(&self, other: &Self) -> Self {
        crate::ffi::bridge::Vector3D_Cross(self, other)
    }

    pub fn normalized(&self) -> Self {
        let len = self.length();
        if len > 0.0 {
            Self {
                x: self.x / len,
                y: self.y / len,
                z: self.z / len,
            }
        } else {
            *self
        }
    }

    pub fn scale(&self, factor: f64) -> Self {
        Self {
            x: self.x * factor,
            y: self.y * factor,
            z: self.z * factor,
        }
    }

    pub fn to_2d(&self) -> SimpleVector2D {
        SimpleVector2D {
            x: self.x,
            y: self.y,
        }
    }
}

impl std::ops::Add for SimpleVector3D {
    type Output = Self;
    fn add(self, other: Self) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
            z: self.z + other.z,
        }
    }
}

impl std::ops::Sub for SimpleVector3D {
    type Output = Self;
    fn sub(self, other: Self) -> Self {
        Self {
            x: self.x - other.x,
            y: self.y - other.y,
            z: self.z - other.z,
        }
    }
}

impl std::ops::Mul<f64> for SimpleVector3D {
    type Output = Self;
    fn mul(self, scalar: f64) -> Self {
        Self {
            x: self.x * scalar,
            y: self.y * scalar,
            z: self.z * scalar,
        }
    }
}

// Location Implementation
impl SimpleLocation {
    pub const ZERO: Self = Self {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };

    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    pub fn distance_to(&self, other: &Self) -> f64 {
        crate::ffi::bridge::Location_Distance(self, other)
    }

    pub fn distance_squared_to(&self, other: &Self) -> f64 {
        crate::ffi::bridge::Location_DistanceSquared(self, other)
    }

    pub fn to_vector(&self) -> SimpleVector3D {
        SimpleVector3D {
            x: self.x,
            y: self.y,
            z: self.z,
        }
    }

    pub fn translate(&self, offset: &SimpleVector3D) -> Self {
        Self {
            x: self.x + offset.x,
            y: self.y + offset.y,
            z: self.z + offset.z,
        }
    }
}

impl std::ops::Add<SimpleVector3D> for SimpleLocation {
    type Output = Self;
    fn add(self, vector: SimpleVector3D) -> Self {
        Self {
            x: self.x + vector.x,
            y: self.y + vector.y,
            z: self.z + vector.z,
        }
    }
}

impl std::ops::Sub<SimpleVector3D> for SimpleLocation {
    type Output = Self;
    fn sub(self, vector: SimpleVector3D) -> Self {
        Self {
            x: self.x - vector.x,
            y: self.y - vector.y,
            z: self.z - vector.z,
        }
    }
}

impl std::ops::Sub for SimpleLocation {
    type Output = SimpleVector3D;
    fn sub(self, other: Self) -> SimpleVector3D {
        SimpleVector3D {
            x: self.x - other.x,
            y: self.y - other.y,
            z: self.z - other.z,
        }
    }
}

// Rotation Implementation
impl SimpleRotation {
    pub const ZERO: Self = Self {
        pitch: 0.0,
        yaw: 0.0,
        roll: 0.0,
    };

    pub fn new(pitch: f64, yaw: f64, roll: f64) -> Self {
        Self { pitch, yaw, roll }
    }

    pub fn from_degrees(pitch: f64, yaw: f64, roll: f64) -> Self {
        Self { pitch, yaw, roll }
    }

    pub fn from_radians(pitch: f64, yaw: f64, roll: f64) -> Self {
        Self {
            pitch: pitch.to_degrees(),
            yaw: yaw.to_degrees(),
            roll: roll.to_degrees(),
        }
    }

    pub fn to_radians(&self) -> (f64, f64, f64) {
        (
            self.pitch.to_radians(),
            self.yaw.to_radians(),
            self.roll.to_radians(),
        )
    }
}

// Transform Implementation
impl SimpleTransform {
    pub const IDENTITY: Self = Self {
        location: SimpleLocation::ZERO,
        rotation: SimpleRotation::ZERO,
    };

    pub fn new(location: SimpleLocation, rotation: SimpleRotation) -> Self {
        Self { location, rotation }
    }

    pub fn from_location(location: SimpleLocation) -> Self {
        Self {
            location,
            rotation: SimpleRotation::ZERO,
        }
    }

    pub fn from_rotation(rotation: SimpleRotation) -> Self {
        Self {
            location: SimpleLocation::ZERO,
            rotation,
        }
    }

    pub fn transform_point(&self, point: &SimpleLocation) -> SimpleLocation {
        crate::ffi::bridge::Transform_TransformPoint(self, point)
    }

    pub fn inverse_transform_point(&self, point: &SimpleLocation) -> SimpleLocation {
        crate::ffi::bridge::Transform_InverseTransformPoint(self, point)
    }

    pub fn get_forward_vector(&self) -> SimpleVector3D {
        crate::ffi::bridge::Transform_GetForwardVector(self)
    }

    pub fn get_right_vector(&self) -> SimpleVector3D {
        crate::ffi::bridge::Transform_GetRightVector(self)
    }

    pub fn get_up_vector(&self) -> SimpleVector3D {
        crate::ffi::bridge::Transform_GetUpVector(self)
    }

    pub fn translate(&self, offset: &SimpleVector3D) -> Self {
        Self {
            location: self.location.translate(offset),
            rotation: self.rotation,
        }
    }

    pub fn rotate(&self, additional_rotation: &SimpleRotation) -> Self {
        Self {
            location: self.location,
            rotation: SimpleRotation {
                pitch: self.rotation.pitch + additional_rotation.pitch,
                yaw: self.rotation.yaw + additional_rotation.yaw,
                roll: self.rotation.roll + additional_rotation.roll,
            },
        }
    }
}

// BoundingBox Implementation
impl SimpleBoundingBox {
    pub fn new(location: SimpleLocation, extent: SimpleVector3D) -> Self {
        Self { location, extent }
    }

    pub fn contains(&self, point: &SimpleLocation) -> bool {
        crate::ffi::bridge::BoundingBox_Contains(self, point)
    }

    pub fn get_vertices(&self) -> Vec<SimpleLocation> {
        crate::ffi::bridge::BoundingBox_GetVertices(self)
    }

    pub fn get_center(&self) -> SimpleLocation {
        self.location
    }

    pub fn get_extent(&self) -> SimpleVector3D {
        self.extent
    }

    pub fn get_min(&self) -> SimpleLocation {
        SimpleLocation {
            x: self.location.x - self.extent.x,
            y: self.location.y - self.extent.y,
            z: self.location.z - self.extent.z,
        }
    }

    pub fn get_max(&self) -> SimpleLocation {
        SimpleLocation {
            x: self.location.x + self.extent.x,
            y: self.location.y + self.extent.y,
            z: self.location.z + self.extent.z,
        }
    }

    pub fn expand(&self, expansion: f64) -> Self {
        Self {
            location: self.location,
            extent: SimpleVector3D {
                x: self.extent.x + expansion,
                y: self.extent.y + expansion,
                z: self.extent.z + expansion,
            },
        }
    }
}

// Conversion traits to work with other geometry libraries
impl From<SimpleVector3D> for SimpleLocation {
    fn from(vector: SimpleVector3D) -> Self {
        Self {
            x: vector.x,
            y: vector.y,
            z: vector.z,
        }
    }
}

impl From<SimpleLocation> for SimpleVector3D {
    fn from(location: SimpleLocation) -> Self {
        Self {
            x: location.x,
            y: location.y,
            z: location.z,
        }
    }
}

impl From<(f64, f64)> for SimpleVector2D {
    fn from((x, y): (f64, f64)) -> Self {
        Self { x, y }
    }
}

impl From<(f64, f64, f64)> for SimpleVector3D {
    fn from((x, y, z): (f64, f64, f64)) -> Self {
        Self { x, y, z }
    }
}

impl From<(f64, f64, f64)> for SimpleLocation {
    fn from((x, y, z): (f64, f64, f64)) -> Self {
        Self { x, y, z }
    }
}

impl From<(f64, f64, f64)> for SimpleRotation {
    fn from((pitch, yaw, roll): (f64, f64, f64)) -> Self {
        Self { pitch, yaw, roll }
    }
}
