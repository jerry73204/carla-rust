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
//! CARLA uses Unreal Engine's **left-handed Z-up** coordinate system:
//! - **X-axis**: Forward (+X points forward)
//! - **Y-axis**: Right (+Y points right)
//! - **Z-axis**: Up (+Z points up)
//!
//! ## Left-Handed vs Right-Handed
//!
//! In a left-handed coordinate system, if you point your left thumb along +X (forward)
//! and your index finger along +Y (right), your middle finger points along +Z (up).
//!
//! This differs from right-handed systems (common in mathematics and some graphics APIs)
//! where +Y often points left instead of right.
//!
//! ## Rotation Convention
//!
//! CARLA uses intrinsic Euler angles in degrees:
//! - **Roll**: Rotation around X-axis (forward) - positive rolls right wing down
//! - **Pitch**: Rotation around Y-axis (right) - positive pitches nose up
//! - **Yaw**: Rotation around Z-axis (up) - positive yaws counter-clockwise when looking down
//!
//! # nalgebra Integration
//!
//! Extension traits provide conversions to/from [nalgebra](https://nalgebra.org) types:
//! - [`LocationExt`] - Convert Location ↔ `Translation3` / `Point3`
//! - [`RotationExt`] - Convert Rotation ↔ `UnitQuaternion`
//! - [`Transform`] - Provides `from_na()` and `to_na()` methods for `Isometry3` conversion
//! - [`Vector2DExt`] / [`Vector3DExt`] - Convert vectors
//!
//! **IMPORTANT**: Converted types (Isometry3, Translation3, UnitQuaternion) still represent
//! CARLA's left-handed coordinate system. nalgebra is used as a math library;
//! coordinate semantics are preserved.
//!
//! ## Transform Composition
//!
//! For composing transforms (e.g., sensor mounting), use the `*` operator:
//!
//! ```ignore
//! use carla::geom::{Location, Rotation, Transform};
//!
//! // Vehicle transform
//! let vehicle = Transform {
//!     location: Location { x: 10.0, y: 0.0, z: 0.0 },
//!     rotation: Rotation { pitch: 0.0, yaw: 0.0, roll: 0.0 }
//! };
//!
//! // Sensor offset (2m forward, 1m right)
//! let sensor_offset = Transform {
//!     location: Location { x: 2.0, y: 1.0, z: 0.0 },
//!     rotation: Rotation { pitch: 0.0, yaw: 0.0, roll: 0.0 }
//! };
//!
//! // Compose to get sensor world position
//! let sensor_world = vehicle * sensor_offset;
//! ```
//!
//! You can also compose transforms via nalgebra (requires understanding coordinate systems):
//!
//! ```ignore
//! use carla::geom::Transform;
//! let result = Transform::from_na(&(t1.to_na() * t2.to_na()));  // Advanced
//! ```
//!
//! ## Handedness-Dependent Operations
//!
//! Be cautious with operations that depend on coordinate system handedness:
//!
//! ### Cross Products
//! Vector cross products depend on handedness:
//! - Left-handed: `forward × right = up`
//! - Right-handed: `forward × right = -up`
//!
//! nalgebra's `cross()` method assumes right-handed convention by default.
//! Use CARLA types for geometric operations to ensure correctness.
//!
//! ### Rotation Directions
//! Positive rotations differ between systems:
//! - In CARLA (left-handed): Positive yaw rotates counter-clockwise looking down
//! - In right-handed: Positive yaw rotates counter-clockwise looking up
//!
//! ### Best Practices
//!
//! - **Prefer CARLA types** for geometry operations: [`Transform`], [`Location`], [`Rotation`]
//! - **Use nalgebra** for linear algebra: matrix ops, linear solvers, decompositions
//! - **Convert at boundaries**: Get data as nalgebra, compute, convert back to CARLA
//! - **Test with simulator**: Verify spatial relationships match expectations
//!
//! # Examples
//!
//! ```
//! use carla::geom::{Location, LocationExt, Transform};
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

use carla_sys::carla_rust::geom::FfiTransform;

pub use carla_sys::{
    carla::geom::{GeoLocation, Rotation, Vector2D, Vector3D},
    carla_rust::geom::FfiLocation as Location,
};

/// CARLA transform combining position and rotation.
///
/// This is a zero-cost wrapper around the FFI type [`FfiTransform`] with identical
/// memory layout. The wrapper enables implementing Rust traits like `std::ops::Mul`
/// while maintaining perfect compatibility with the C++ CARLA client library.
///
/// # Memory Layout
///
/// This type uses `#[repr(transparent)]` to guarantee it has the exact same memory
/// layout as `FfiTransform` (24 bytes: 6 × f32). This allows zero-cost conversion
/// via transmutation at FFI boundaries.
///
/// # Transform Composition
///
/// Transforms can be composed using the `*` operator:
///
/// ```no_run
/// use carla::geom::{Location, Rotation, Transform};
///
/// // Vehicle transform
/// let vehicle = Transform {
///     location: Location {
///         x: 10.0,
///         y: 5.0,
///         z: 0.3,
///     },
///     rotation: Rotation {
///         pitch: 0.0,
///         yaw: 90.0,
///         roll: 0.0,
///     },
/// };
///
/// // Sensor offset (mounted 2m forward, 0.5m up)
/// let sensor_offset = Transform {
///     location: Location {
///         x: 2.0,
///         y: 0.0,
///         z: 0.5,
///     },
///     rotation: Rotation {
///         pitch: 0.0,
///         yaw: 0.0,
///         roll: 0.0,
///     },
/// };
///
/// // Compute sensor world position
/// let sensor_world = vehicle * sensor_offset;
/// ```
///
/// # Coordinate System
///
/// CARLA uses a **left-handed, Z-up** coordinate system:
/// - X: forward
/// - Y: right
/// - Z: up
///
/// This wrapper preserves CARLA's coordinate semantics. See module-level documentation
/// for detailed coordinate system information.
///
/// # Memory Layout
///
/// This type uses `#[repr(C)]` to guarantee C-compatible layout with public fields.
/// The layout is verified at compile time on both C++ and Rust sides to ensure
/// perfect compatibility with `FfiTransform` for zero-cost transmutation.
#[repr(C)]
#[derive(Debug, Clone)]
pub struct Transform {
    /// Position component (X, Y, Z coordinates).
    pub location: Location,
    /// Rotation component (pitch, yaw, roll in degrees).
    pub rotation: Rotation,
}

impl Transform {
    /// Returns a reference to the underlying FFI type.
    ///
    /// Use this when passing transforms to FFI functions that expect `&FfiTransform`.
    ///
    /// # Safety
    ///
    /// This is a zero-cost operation using transmute. The memory layout is verified
    /// at compile time via static_assertions on both C++ and Rust sides.
    #[inline(always)]
    pub fn as_ffi(&self) -> &FfiTransform {
        // Safety: repr(C) with verified layout (size, alignment, field offsets)
        unsafe { std::mem::transmute(self) }
    }

    /// Converts from the FFI type.
    ///
    /// This is a zero-cost operation using transmute.
    ///
    /// # Safety
    ///
    /// The memory layout is verified at compile time via static_assertions
    /// on both C++ and Rust sides.
    #[inline(always)]
    pub fn from_ffi(ffi: FfiTransform) -> Self {
        // Safety: repr(C) with verified layout (size, alignment, field offsets)
        unsafe { std::mem::transmute(ffi) }
    }

    /// Converts into the FFI type.
    ///
    /// This is a zero-cost operation using transmute.
    ///
    /// # Safety
    ///
    /// The memory layout is verified at compile time via static_assertions
    /// on both C++ and Rust sides.
    #[inline(always)]
    pub fn into_ffi(self) -> FfiTransform {
        // Safety: repr(C) with verified layout (size, alignment, field offsets)
        unsafe { std::mem::transmute(self) }
    }

    /// Creates a transform from a nalgebra isometry (position + rotation).
    pub fn from_na(pose: &Isometry3<f32>) -> Self {
        let Isometry3 {
            rotation,
            translation,
        } = pose;
        let location = Location::from_na_translation(translation);
        let rotation = Rotation::from_na(rotation);
        Self { location, rotation }
    }

    /// Converts to nalgebra `Isometry3`.
    ///
    /// **IMPORTANT**: The resulting `Isometry3` represents the same
    /// coordinate system as CARLA (left-handed, Z-up). nalgebra is
    /// used purely as a math library; coordinate semantics are preserved.
    ///
    /// # Coordinate System Preservation
    ///
    /// The conversion preserves CARLA's coordinate system:
    /// - **Position** (x, y, z): Copied directly without sign changes
    /// - **Rotation**: Converted from Euler angles (degrees) to quaternion (radians)
    /// - **Handedness**: Left-handed system preserved
    /// - **Up axis**: Z-up preserved
    ///
    /// # Transform Composition
    ///
    /// Multiplication via nalgebra works correctly for transforms in
    /// CARLA's coordinate system:
    ///
    /// ```ignore
    /// use carla::geom::Transform;
    ///
    /// let vehicle_to_world = vehicle.transform();  // Isometry3
    /// let sensor_to_vehicle = sensor_offset.to_na();
    /// let sensor_to_world = vehicle_to_world * sensor_to_vehicle;
    /// let sensor_transform = Transform::from_na(&sensor_to_world);
    /// ```
    ///
    /// This is equivalent to `vehicle * sensor_offset` using the Mul operator.
    ///
    /// # Example
    ///
    /// ```
    /// use carla::geom::{Location, Rotation, Transform};
    ///
    /// // Vehicle at (10, 0, 0) facing forward
    /// let vehicle = Transform {
    ///     location: Location {
    ///         x: 10.0,
    ///         y: 0.0,
    ///         z: 0.0,
    ///     },
    ///     rotation: Rotation {
    ///         pitch: 0.0,
    ///         yaw: 0.0,
    ///         roll: 0.0,
    ///     },
    /// };
    ///
    /// // Sensor mounted 2 meters in front
    /// let sensor_offset = Transform {
    ///     location: Location {
    ///         x: 2.0,
    ///         y: 0.0,
    ///         z: 0.0,
    ///     },
    ///     rotation: Rotation {
    ///         pitch: 0.0,
    ///         yaw: 0.0,
    ///         roll: 0.0,
    ///     },
    /// };
    ///
    /// // Compose using multiplication
    /// let sensor_world = &vehicle * &sensor_offset;
    /// let iso = sensor_world.to_na();
    ///
    /// // Sensor should be at (12, 0, 0) in world coordinates
    /// assert!((iso.translation.x - 12.0).abs() < 0.01);
    /// ```
    pub fn to_na(&self) -> Isometry3<f32> {
        Isometry3 {
            rotation: self.rotation.to_na(),
            translation: self.location.to_na_translation(),
        }
    }

    /// Composes two transforms (internal implementation for multiplication operator).
    ///
    /// Prefer using the `*` operator instead of calling this method directly:
    /// ```ignore
    /// let result = &transform_a * &transform_b;  // Preferred
    /// ```
    fn compose(&self, other: &Self) -> Self {
        Transform::from_na(&(self.to_na() * other.to_na()))
    }
}

// Compile-time layout verification
static_assertions::assert_eq_size!(Transform, FfiTransform);
static_assertions::assert_eq_align!(Transform, FfiTransform);
static_assertions::assert_eq_size!(Transform, [f32; 6]);

// Field offset verification (ensures same field layout as FfiTransform)
static_assertions::const_assert_eq!(memoffset::offset_of!(Transform, location), 0);
static_assertions::const_assert_eq!(memoffset::offset_of!(Transform, rotation), 12); // sizeof(Location) = 3 × f32 = 12 bytes

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

    /// Converts to nalgebra `Translation3`.
    ///
    /// **IMPORTANT**: The resulting `Translation3` represents the same
    /// coordinate system as CARLA (left-handed, Z-up). Coordinates are
    /// copied directly without sign changes:
    /// - **X-axis**: Forward (same in both systems)
    /// - **Y-axis**: Right (CARLA convention preserved)
    /// - **Z-axis**: Up (same in both systems)
    ///
    /// nalgebra is used purely as a math library; coordinate semantics
    /// are preserved from CARLA.
    ///
    /// # Coordinate System
    ///
    /// CARLA uses Unreal Engine's **left-handed, Z-up** coordinate system:
    /// - +X points forward
    /// - +Y points right
    /// - +Z points up
    ///
    /// This coordinate system is preserved in the nalgebra representation.
    fn to_na_translation(&self) -> Translation3<f32>;

    /// Converts to nalgebra `Point3`.
    ///
    /// **IMPORTANT**: Same coordinate system notes as [`to_na_translation`](LocationExt::to_na_translation) apply.
    /// The point represents a position in CARLA's left-handed, Z-up coordinate system.
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

    /// Converts to nalgebra `UnitQuaternion`.
    ///
    /// **IMPORTANT**: The resulting quaternion represents rotations in
    /// CARLA's **left-handed** coordinate system. CARLA's Euler angles
    /// (in degrees) are converted to radians and then to a quaternion
    /// using intrinsic XYZ rotation order.
    ///
    /// # Coordinate System
    ///
    /// - **Handedness**: Left-handed (CARLA/Unreal Engine convention)
    /// - **Up axis**: Z-up
    /// - **Rotation Order**: Intrinsic XYZ (roll around X, pitch around Y, yaw around Z)
    /// - **Angular Units**: Radians (converted from CARLA's degrees)
    ///
    /// # Rotation Convention
    ///
    /// CARLA uses intrinsic rotations in the order: roll (X) → pitch (Y) → yaw (Z).
    /// This is equivalent to extrinsic rotations: yaw (Z) → pitch (Y) → roll (X).
    ///
    /// In a left-handed system:
    /// - Positive yaw rotates counter-clockwise when looking down (-Z direction)
    /// - Positive pitch rotates nose up
    /// - Positive roll rotates right wing down
    ///
    /// # Note on Transform Operations
    ///
    /// Quaternion multiplication via nalgebra works correctly for CARLA's
    /// coordinate system. However, be cautious with operations that depend on
    /// handedness (like vector cross products).
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

// Implement multiplication operators for Transform composition.
// These implementations are now possible because Transform is a newtype wrapper
// defined in this crate, not a type alias to a foreign type.

impl std::ops::Mul for Transform {
    type Output = Transform;

    fn mul(self, rhs: Transform) -> Transform {
        self.compose(&rhs)
    }
}

impl std::ops::Mul<&Transform> for Transform {
    type Output = Transform;

    fn mul(self, rhs: &Transform) -> Transform {
        self.compose(rhs)
    }
}

impl std::ops::Mul<Transform> for &Transform {
    type Output = Transform;

    fn mul(self, rhs: Transform) -> Transform {
        self.compose(&rhs)
    }
}

impl std::ops::Mul<&Transform> for &Transform {
    type Output = Transform;

    fn mul(self, rhs: &Transform) -> Transform {
        self.compose(rhs)
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
            Transform::from_na(in_bbox_to_world_transform).as_ffi(),
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
            .GetWorldVertices(Transform::from_na(in_bbox_to_world_tr).as_ffi())
            .iter()
            .map(|loc| loc.to_na_translation())
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper to compare floats with tolerance
    fn assert_approx_eq(a: f32, b: f32, epsilon: f32) {
        assert!(
            (a - b).abs() < epsilon,
            "assertion failed: `{} ≈ {}` (diff: {})",
            a,
            b,
            (a - b).abs()
        );
    }

    // === Layout Verification Tests ===
    // These tests verify the newtype wrapper has identical layout to FfiTransform

    #[test]
    fn test_transform_size() {
        assert_eq!(
            std::mem::size_of::<Transform>(),
            std::mem::size_of::<FfiTransform>()
        );
        assert_eq!(std::mem::size_of::<Transform>(), 24); // 6 × f32
    }

    #[test]
    fn test_transform_alignment() {
        assert_eq!(
            std::mem::align_of::<Transform>(),
            std::mem::align_of::<FfiTransform>()
        );
        assert_eq!(std::mem::align_of::<Transform>(), 4); // f32 alignment
    }

    #[test]
    fn test_transform_round_trip_conversion() {
        let ffi = FfiTransform {
            location: Location {
                x: 10.5,
                y: 20.3,
                z: 5.7,
            },
            rotation: Rotation {
                pitch: 15.0,
                yaw: 45.0,
                roll: -10.0,
            },
        };

        // Convert FFI → Transform → FFI
        let ffi_copy = ffi.clone();
        let transform = Transform::from_ffi(ffi);
        let ffi_back = transform.into_ffi();

        // Should be identical
        assert_eq!(ffi_back.location.x, ffi_copy.location.x);
        assert_eq!(ffi_back.location.y, ffi_copy.location.y);
        assert_eq!(ffi_back.location.z, ffi_copy.location.z);
        assert_eq!(ffi_back.rotation.pitch, ffi_copy.rotation.pitch);
        assert_eq!(ffi_back.rotation.yaw, ffi_copy.rotation.yaw);
        assert_eq!(ffi_back.rotation.roll, ffi_copy.rotation.roll);
    }

    #[test]
    fn test_transform_accessors() {
        let transform = Transform {
            location: Location {
                x: 1.0,
                y: 2.0,
                z: 3.0,
            },
            rotation: Rotation {
                pitch: 10.0,
                yaw: 20.0,
                roll: 30.0,
            },
        };

        // Test immutable accessors
        assert_eq!(transform.location.x, 1.0);
        assert_eq!(transform.location.y, 2.0);
        assert_eq!(transform.location.z, 3.0);
        assert_eq!(transform.rotation.pitch, 10.0);
        assert_eq!(transform.rotation.yaw, 20.0);
        assert_eq!(transform.rotation.roll, 30.0);

        // Test mutable accessors
        let mut transform_mut = transform;
        transform_mut.location.x = 5.0;
        transform_mut.rotation.yaw = 45.0;
        assert_eq!(transform_mut.location.x, 5.0);
        assert_eq!(transform_mut.rotation.yaw, 45.0);
    }

    #[test]
    fn test_transform_as_ffi_reference() {
        let transform = Transform {
            location: Location {
                x: 7.0,
                y: 8.0,
                z: 9.0,
            },
            rotation: Rotation {
                pitch: 5.0,
                yaw: 15.0,
                roll: 25.0,
            },
        };

        // Get FFI reference
        let ffi_ref: &FfiTransform = transform.as_ffi();

        // Verify it points to the same data
        assert_eq!(ffi_ref.location.x, 7.0);
        assert_eq!(ffi_ref.location.y, 8.0);
        assert_eq!(ffi_ref.location.z, 9.0);
        assert_eq!(ffi_ref.rotation.pitch, 5.0);
        assert_eq!(ffi_ref.rotation.yaw, 15.0);
        assert_eq!(ffi_ref.rotation.roll, 25.0);
    }

    // === Transform Multiplication Tests ===

    #[test]
    fn test_transform_mul_identity() {
        let transform = Transform {
            location: Location {
                x: 10.0,
                y: 5.0,
                z: 2.0,
            },
            rotation: Rotation {
                pitch: 10.0,
                yaw: 20.0,
                roll: 5.0,
            },
        };

        let identity = Transform {
            location: Location {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            rotation: Rotation {
                pitch: 0.0,
                yaw: 0.0,
                roll: 0.0,
            },
        };

        // Multiplying with identity should not change the transform
        let result = &transform * &identity;

        assert_approx_eq(result.location.x, transform.location.x, 0.001);
        assert_approx_eq(result.location.y, transform.location.y, 0.001);
        assert_approx_eq(result.location.z, transform.location.z, 0.001);
        assert_approx_eq(result.rotation.pitch, transform.rotation.pitch, 0.01);
        assert_approx_eq(result.rotation.yaw, transform.rotation.yaw, 0.01);
        assert_approx_eq(result.rotation.roll, transform.rotation.roll, 0.01);
    }

    #[test]
    fn test_transform_mul_translation_only() {
        let t1 = Transform {
            location: Location {
                x: 10.0,
                y: 5.0,
                z: 2.0,
            },
            rotation: Rotation {
                pitch: 0.0,
                yaw: 0.0,
                roll: 0.0,
            },
        };

        let t2 = Transform {
            location: Location {
                x: 3.0,
                y: 2.0,
                z: 1.0,
            },
            rotation: Rotation {
                pitch: 0.0,
                yaw: 0.0,
                roll: 0.0,
            },
        };

        // Pure translation composition
        let result = t1 * t2;

        assert_approx_eq(result.location.x, 13.0, 0.001);
        assert_approx_eq(result.location.y, 7.0, 0.001);
        assert_approx_eq(result.location.z, 3.0, 0.001);
        assert_approx_eq(result.rotation.pitch, 0.0, 0.01);
        assert_approx_eq(result.rotation.yaw, 0.0, 0.01);
        assert_approx_eq(result.rotation.roll, 0.0, 0.01);
    }

    #[test]
    fn test_transform_mul_rotation_90deg_yaw() {
        // Parent rotated 90° yaw (left-handed: counter-clockwise looking down)
        let parent = Transform {
            location: Location {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            rotation: Rotation {
                pitch: 0.0,
                yaw: 90.0,
                roll: 0.0,
            },
        };

        // Child offset forward (in local frame)
        let child = Transform {
            location: Location {
                x: 5.0,
                y: 0.0,
                z: 0.0,
            },
            rotation: Rotation {
                pitch: 0.0,
                yaw: 0.0,
                roll: 0.0,
            },
        };

        // After 90° yaw, local forward (x+) becomes world left (y+)
        let result = parent * child;

        // In left-handed system, 90° yaw makes X→Y, Y→-X
        // So (5, 0, 0) rotated 90° yaw → (0, 5, 0)
        assert_approx_eq(result.location.x, 0.0, 0.01);
        assert_approx_eq(result.location.y, 5.0, 0.01);
        assert_approx_eq(result.location.z, 0.0, 0.01);
        assert_approx_eq(result.rotation.yaw, 90.0, 0.1);
    }

    #[test]
    fn test_transform_mul_rotation_180deg_yaw() {
        // Parent rotated 180° yaw
        let parent = Transform {
            location: Location {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            rotation: Rotation {
                pitch: 0.0,
                yaw: 180.0,
                roll: 0.0,
            },
        };

        // Child offset forward
        let child = Transform {
            location: Location {
                x: 5.0,
                y: 0.0,
                z: 0.0,
            },
            rotation: Rotation {
                pitch: 0.0,
                yaw: 0.0,
                roll: 0.0,
            },
        };

        // After 180° yaw, forward becomes backward
        let result = parent * child;

        // (5, 0, 0) rotated 180° → (-5, 0, 0)
        assert_approx_eq(result.location.x, -5.0, 0.01);
        assert_approx_eq(result.location.y, 0.0, 0.01);
        assert_approx_eq(result.location.z, 0.0, 0.01);
        assert_approx_eq(result.rotation.yaw.abs(), 180.0, 0.1);
    }

    #[test]
    fn test_transform_mul_combined_translation_rotation() {
        // Vehicle at (10, 0, 0) facing forward
        let vehicle = Transform {
            location: Location {
                x: 10.0,
                y: 0.0,
                z: 0.0,
            },
            rotation: Rotation {
                pitch: 0.0,
                yaw: 0.0,
                roll: 0.0,
            },
        };

        // Sensor mounted 2m forward, 1m right, 0.5m up
        let sensor_offset = Transform {
            location: Location {
                x: 2.0,
                y: 1.0,
                z: 0.5,
            },
            rotation: Rotation {
                pitch: 0.0,
                yaw: 0.0,
                roll: 0.0,
            },
        };

        // Compose transforms
        let sensor_world = vehicle * sensor_offset;

        // Expected: vehicle position + offset (no rotation)
        assert_approx_eq(sensor_world.location.x, 12.0, 0.001);
        assert_approx_eq(sensor_world.location.y, 1.0, 0.001);
        assert_approx_eq(sensor_world.location.z, 0.5, 0.001);
    }

    #[test]
    fn test_transform_mul_sensor_mounting_rotated() {
        // Vehicle at (10, 0, 0) rotated 90° yaw
        let vehicle = Transform {
            location: Location {
                x: 10.0,
                y: 0.0,
                z: 0.0,
            },
            rotation: Rotation {
                pitch: 0.0,
                yaw: 90.0,
                roll: 0.0,
            },
        };

        // Sensor mounted 2m forward (in vehicle's local frame)
        let sensor_offset = Transform {
            location: Location {
                x: 2.0,
                y: 0.0,
                z: 0.0,
            },
            rotation: Rotation {
                pitch: 0.0,
                yaw: 0.0,
                roll: 0.0,
            },
        };

        // Compose transforms
        let sensor_world = vehicle * sensor_offset;

        // Vehicle's "forward" is now world "left" (+Y)
        // So sensor is at (10, 2, 0) in world coordinates
        assert_approx_eq(sensor_world.location.x, 10.0, 0.01);
        assert_approx_eq(sensor_world.location.y, 2.0, 0.01);
        assert_approx_eq(sensor_world.location.z, 0.0, 0.01);
        assert_approx_eq(sensor_world.rotation.yaw, 90.0, 0.1);
    }

    #[test]
    fn test_transform_mul_associativity() {
        let t1 = Transform {
            location: Location {
                x: 1.0,
                y: 2.0,
                z: 3.0,
            },
            rotation: Rotation {
                pitch: 10.0,
                yaw: 20.0,
                roll: 5.0,
            },
        };

        let t2 = Transform {
            location: Location {
                x: 4.0,
                y: 5.0,
                z: 6.0,
            },
            rotation: Rotation {
                pitch: 15.0,
                yaw: 30.0,
                roll: 10.0,
            },
        };

        let t3 = Transform {
            location: Location {
                x: 7.0,
                y: 8.0,
                z: 9.0,
            },
            rotation: Rotation {
                pitch: 5.0,
                yaw: 10.0,
                roll: 15.0,
            },
        };

        // Test associativity: (t1 * t2) * t3 == t1 * (t2 * t3)
        let left = (&t1 * &t2) * &t3;
        let right = &t1 * (&t2 * &t3);

        assert_approx_eq(left.location.x, right.location.x, 0.001);
        assert_approx_eq(left.location.y, right.location.y, 0.001);
        assert_approx_eq(left.location.z, right.location.z, 0.001);
        assert_approx_eq(left.rotation.pitch, right.rotation.pitch, 0.01);
        assert_approx_eq(left.rotation.yaw, right.rotation.yaw, 0.01);
        assert_approx_eq(left.rotation.roll, right.rotation.roll, 0.01);
    }

    #[test]
    fn test_transform_mul_reference_variants() {
        let t1 = Transform {
            location: Location {
                x: 10.0,
                y: 5.0,
                z: 2.0,
            },
            rotation: Rotation {
                pitch: 0.0,
                yaw: 0.0,
                roll: 0.0,
            },
        };

        let t2 = Transform {
            location: Location {
                x: 3.0,
                y: 2.0,
                z: 1.0,
            },
            rotation: Rotation {
                pitch: 0.0,
                yaw: 0.0,
                roll: 0.0,
            },
        };

        // Test reference variant: &Transform * &Transform
        let result = &t1 * &t2;

        // Original transforms should still be available
        assert_eq!(t1.location.x, 10.0);
        assert_eq!(t2.location.x, 3.0);

        // Result should be correct
        assert_approx_eq(result.location.x, 13.0, 0.001);
        assert_approx_eq(result.location.y, 7.0, 0.001);
        assert_approx_eq(result.location.z, 3.0, 0.001);
    }

    #[test]
    fn test_transform_mul_rotation_composition() {
        // First rotation: 45° yaw
        let rot1 = Transform {
            location: Location {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            rotation: Rotation {
                pitch: 0.0,
                yaw: 45.0,
                roll: 0.0,
            },
        };

        // Second rotation: another 45° yaw
        let rot2 = Transform {
            location: Location {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            rotation: Rotation {
                pitch: 0.0,
                yaw: 45.0,
                roll: 0.0,
            },
        };

        // Compose rotations
        let result = rot1 * rot2;

        // Should result in 90° yaw
        assert_approx_eq(result.rotation.yaw, 90.0, 0.1);
        assert_approx_eq(result.rotation.pitch, 0.0, 0.1);
        assert_approx_eq(result.rotation.roll, 0.0, 0.1);
    }
}

assert_impl_all!(Vector2D: Send, Sync);
assert_impl_all!(Vector3D: Send, Sync);
assert_impl_all!(Location: Send, Sync);
assert_impl_all!(Rotation: Send, Sync);
assert_impl_all!(Transform: Send, Sync);
assert_impl_all!(NativeBoundingBox: Send, Sync);
