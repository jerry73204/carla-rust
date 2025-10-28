//! Geometry types and utilities.
//!
//! This module provides geometry primitives used throughout CARLA:
//! - [`Location`] - 3D position (x, y, z)
//! - [`Rotation`] - 3D rotation (pitch, yaw, roll in degrees)
//! - [`Transform`] - Combined position and rotation
//! - [`Vector2D`] / [`Vector3D`] - 2D and 3D vectors
//! - [`BoundingBox`] - Axis-aligned or oriented bounding boxes
//! - [`GeoLocation`] - GPS coordinates (latitude, longitude, altitude)
//!
//! # Coordinate System
//!
//! CARLA uses Unreal Engine's left-handed Z-up coordinate system:
//! - **X-axis**: Forward
//! - **Y-axis**: Right
//! - **Z-axis**: Up
//!
//! # nalgebra Integration
//!
//! Extension traits provide conversions to/from [nalgebra](https://nalgebra.org) types:
//! - [`LocationExt`] - Convert Location ↔ `Translation3` / `Point3`
//! - [`RotationExt`] - Convert Rotation ↔ `UnitQuaternion`
//! - [`TransformExt`] - Convert Transform ↔ `Isometry3`
//! - [`Vector2DExt`] / [`Vector3DExt`] - Convert vectors
//!
//! # Examples
//!
//! ```
//! use carla::geom::{Location, LocationExt, Transform, TransformExt};
//! use nalgebra::{Isometry3, Translation3, UnitQuaternion};
//!
//! // Create a location from nalgebra
//! let translation = Translation3::new(1.0, 2.0, 3.0);
//! let location = Location::from_na_translation(&translation);
//!
//! // Create a transform
//! let transform = Isometry3::from_parts(translation, UnitQuaternion::identity());
//! let carla_transform = Transform::from_na(&transform);
//! ```

use carla_sys::carla_rust::geom::FfiBoundingBox as NativeBoundingBox;
use nalgebra::{Isometry3, Point3, Translation3, UnitQuaternion, Vector2, Vector3};
use static_assertions::assert_impl_all;

pub use carla_sys::{
    carla::geom::{GeoLocation, Rotation, Vector2D, Vector3D},
    carla_rust::geom::{FfiLocation as Location, FfiTransform as Transform},
};

/// Extension trait for converting between [`Vector2D`] and nalgebra's `Vector2<f32>`.
///
/// Enables seamless integration with nalgebra for 2D vector mathematics.
pub trait Vector2DExt {
    /// Creates a CARLA vector from a nalgebra vector.
    fn from_na(from: &Vector2<f32>) -> Self;
    /// Converts to a nalgebra vector.
    fn to_na(&self) -> Vector2<f32>;
}

impl Vector2DExt for Vector2D {
    fn from_na(from: &Vector2<f32>) -> Self {
        Self {
            x: from.x,
            y: from.y,
        }
    }

    fn to_na(&self) -> Vector2<f32> {
        let Self { x, y } = *self;
        Vector2::new(x, y)
    }
}

/// Extension trait for converting between [`Vector3D`] and nalgebra's `Vector3<f32>`.
///
/// Enables seamless integration with nalgebra for 3D vector mathematics.
pub trait Vector3DExt {
    /// Creates a CARLA vector from a nalgebra vector.
    fn from_na(from: &Vector3<f32>) -> Self;
    /// Converts to a nalgebra vector.
    fn to_na(&self) -> Vector3<f32>;
}

impl Vector3DExt for Vector3D {
    fn from_na(from: &Vector3<f32>) -> Self {
        Self {
            x: from.x,
            y: from.y,
            z: from.z,
        }
    }

    fn to_na(&self) -> Vector3<f32> {
        let Self { x, y, z } = *self;
        Vector3::new(x, y, z)
    }
}

/// Extension trait for converting between [`Location`] and nalgebra types.
///
/// Provides conversions to/from `Translation3` and `Point3` for integration
/// with nalgebra's linear algebra operations.
pub trait LocationExt {
    /// Creates a new location with the given coordinates.
    ///
    /// # Arguments
    ///
    /// * `x` - Forward coordinate
    /// * `y` - Right coordinate
    /// * `z` - Up coordinate
    fn new(x: f32, y: f32, z: f32) -> Self;
    /// Creates a location from a nalgebra translation.
    fn from_na_translation(from: &Translation3<f32>) -> Self;
    /// Creates a location from a nalgebra point.
    fn from_na_point(from: &Point3<f32>) -> Self;
    /// Converts to a nalgebra translation.
    fn to_na_translation(&self) -> Translation3<f32>;
    /// Converts to a nalgebra point.
    fn to_na_point(&self) -> Point3<f32>;
}

impl LocationExt for Location {
    #[inline]
    fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    fn from_na_translation(from: &Translation3<f32>) -> Self {
        Self {
            x: from.x,
            y: from.y,
            z: from.z,
        }
    }

    fn from_na_point(from: &Point3<f32>) -> Self {
        Self {
            x: from.x,
            y: from.y,
            z: from.z,
        }
    }

    fn to_na_translation(&self) -> Translation3<f32> {
        let Self { x, y, z } = *self;
        Translation3::new(x, y, z)
    }

    fn to_na_point(&self) -> Point3<f32> {
        let Self { x, y, z } = *self;
        Point3::new(x, y, z)
    }
}

/// Extension trait for converting between [`Rotation`] and nalgebra's `UnitQuaternion`.
///
/// CARLA rotations use degrees (pitch, yaw, roll), while nalgebra uses radians.
/// This trait handles the conversion automatically.
pub trait RotationExt {
    /// Creates a rotation from a nalgebra unit quaternion.
    fn from_na(from: &UnitQuaternion<f32>) -> Self;
    /// Converts to a nalgebra unit quaternion.
    fn to_na(&self) -> UnitQuaternion<f32>;
}

impl RotationExt for Rotation {
    fn from_na(from: &UnitQuaternion<f32>) -> Self {
        let (roll, pitch, yaw) = from.euler_angles();
        Self {
            roll: roll.to_degrees(),
            pitch: pitch.to_degrees(),
            yaw: yaw.to_degrees(),
        }
    }

    fn to_na(&self) -> UnitQuaternion<f32> {
        let Self { roll, pitch, yaw } = *self;
        UnitQuaternion::from_euler_angles(roll.to_radians(), pitch.to_radians(), yaw.to_radians())
    }
}

/// Extension trait for converting between [`Transform`] and nalgebra's `Isometry3`.
///
/// Transforms combine position ([`Location`]) and rotation ([`Rotation`]) into
/// a single rigid-body transformation.
pub trait TransformExt {
    /// Creates a transform from a nalgebra isometry (position + rotation).
    fn from_na(pose: &Isometry3<f32>) -> Self;
    /// Converts to a nalgebra isometry.
    fn to_na(&self) -> Isometry3<f32>;
}

impl TransformExt for Transform {
    fn from_na(pose: &Isometry3<f32>) -> Self {
        let Isometry3 {
            rotation,
            translation,
        } = pose;
        let location = Location::from_na_translation(translation);
        let rotation = Rotation::from_na(rotation);
        Self { location, rotation }
    }

    fn to_na(&self) -> Isometry3<f32> {
        Isometry3 {
            rotation: self.rotation.to_na(),
            translation: self.location.to_na_translation(),
        }
    }
}

/// A 3D bounding box for collision detection and spatial queries.
///
/// Bounding boxes can be axis-aligned or oriented (rotated) and are used
/// for vehicle bounds, sensor ranges, and spatial queries.
///
/// # Examples
///
/// ```
/// use carla::geom::BoundingBox;
/// use nalgebra::{Isometry3, Vector3};
///
/// let bbox = BoundingBox {
///     transform: Isometry3::identity(),
///     extent: Vector3::new(2.0, 1.0, 0.5), // Half-extents (width, height, depth)
/// };
/// ```
#[derive(Debug, Clone)]
pub struct BoundingBox<T> {
    /// The center position and orientation of the bounding box.
    pub transform: Isometry3<T>,
    /// Half-extents along each axis (half-width, half-height, half-depth).
    pub extent: Vector3<T>,
}

impl BoundingBox<f32> {
    /// Creates a new bounding box with a given center location and half-extents.
    ///
    /// # Arguments
    ///
    /// * `location` - Center position of the bounding box
    /// * `extent` - Half-extents along each axis (half-width, half-height, half-depth)
    ///
    /// # Examples
    ///
    /// ```
    /// use carla::geom::{BoundingBox, Location, LocationExt, Vector3D};
    ///
    /// let bbox = BoundingBox::new(
    ///     Location::new(0.0, 0.0, 1.0),
    ///     Vector3D {
    ///         x: 2.0,
    ///         y: 1.0,
    ///         z: 1.0,
    ///     },
    /// );
    /// ```
    pub fn new(location: Location, extent: Vector3D) -> Self {
        Self {
            transform: Isometry3 {
                rotation: UnitQuaternion::identity(),
                translation: location.to_na_translation(),
            },
            extent: extent.to_na(),
        }
    }

    /// Converts to CARLA's native C++ bounding box type.
    pub fn to_native(&self) -> NativeBoundingBox {
        let Self { transform, extent } = self;
        NativeBoundingBox {
            location: Location::from_na_translation(&transform.translation),
            rotation: Rotation::from_na(&transform.rotation),
            extent: Vector3D::from_na(extent),
            #[cfg(carla_0916)]
            actor_id: 0,
        }
    }

    /// Creates from CARLA's native C++ bounding box type.
    pub fn from_native(bbox: &NativeBoundingBox) -> Self {
        #[cfg(carla_0916)]
        let NativeBoundingBox {
            location,
            extent,
            rotation,
            actor_id: _,
        } = bbox;
        #[cfg(not(carla_0916))]
        let NativeBoundingBox {
            location,
            extent,
            rotation,
        } = bbox;
        Self {
            transform: Isometry3 {
                rotation: rotation.to_na(),
                translation: location.to_na_translation(),
            },
            extent: extent.to_na(),
        }
    }

    /// Tests if a point is inside the bounding box.
    ///
    /// # Arguments
    ///
    /// * `in_world_point` - Point position in world coordinates
    /// * `in_bbox_to_world_transform` - Transform from bounding box space to world space
    pub fn contains(
        &self,
        in_world_point: &Translation3<f32>,
        in_bbox_to_world_transform: &Isometry3<f32>,
    ) -> bool {
        self.to_native().Contains(
            &Location::from_na_translation(in_world_point),
            &Transform::from_na(in_bbox_to_world_transform),
        )
    }

    /// Returns the 8 corner vertices in local (bounding box) coordinates.
    pub fn local_vertices(&self) -> Vec<Translation3<f32>> {
        self.to_native()
            .GetLocalVertices()
            .iter()
            .map(|loc| loc.to_na_translation())
            .collect()
    }

    /// Returns the 8 corner vertices in world coordinates.
    ///
    /// # Arguments
    ///
    /// * `in_bbox_to_world_tr` - Transform from bounding box space to world space
    pub fn world_vertices(&self, in_bbox_to_world_tr: &Isometry3<f32>) -> Vec<Translation3<f32>> {
        self.to_native()
            .GetWorldVertices(&Transform::from_na(in_bbox_to_world_tr))
            .iter()
            .map(|loc| loc.to_na_translation())
            .collect()
    }
}

assert_impl_all!(Vector2D: Send, Sync);
assert_impl_all!(Vector3D: Send, Sync);
assert_impl_all!(Location: Send, Sync);
assert_impl_all!(Rotation: Send, Sync);
assert_impl_all!(Transform: Send, Sync);
assert_impl_all!(NativeBoundingBox: Send, Sync);
