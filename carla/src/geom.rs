//! Geometry types and utilities.
//!
//! This module provides geometry primitives used throughout CARLA:
//! - [`Location`] - 3D position (x, y, z)
//! - [`Rotation`] - 3D rotation (pitch, yaw, roll in degrees)
//! - [`Transform`] - Combined position and rotation
//! - [`Vector2D`] - 2D vector
//! - [`Vector3D`] - 3D vector
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
//! Geometry types provide conversions to/from [nalgebra](https://nalgebra.org) types:
//! - [`Location`] - Provides `from_na_*()` and `to_na_*()` methods for `Translation3` / `Point3`
//! - [`Rotation`] - Provides `from_na()` and `to_na()` methods for `UnitQuaternion` conversion
//! - [`Transform`] - Provides `from_na()` and `to_na()` methods for `Isometry3` conversion
//! - [`Vector2D`] - Provides `from_na()` and `to_na()` methods for `Vector2` conversion
//! - [`Vector3D`] - Provides `from_na()` and `to_na()` methods for `Vector3` conversion
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
//! use carla::geom::{Location, Transform};
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

pub use carla_sys::carla_rust::geom::{
    FfiGeoLocation, FfiLocation, FfiRotation, FfiTransform, FfiVector2D, FfiVector3D,
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

/// 3D location in CARLA's left-handed Z-up coordinate system.
///
/// Represents a position in 3D space with floating-point precision.
/// This type is compatible with the C++ `carla::geom::Location` type
/// and can be safely transmuted for zero-cost FFI conversion.
///
/// # Examples
///
/// ```
/// use carla::geom::Location;
///
/// let loc1 = Location::new(10.0, 20.0, 1.5);
/// let loc2 = Location::new(5.0, 3.0, 0.5);
///
/// // Vector arithmetic
/// let sum = loc1 + loc2;
/// let diff = loc1 - loc2;
///
/// // Distance calculation
/// let distance = loc1.distance(&loc2);
/// ```
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Location {
    /// X coordinate (forward/backward in vehicle-relative coordinates)
    pub x: f32,
    /// Y coordinate (left/right in vehicle-relative coordinates)
    pub y: f32,
    /// Z coordinate (up/down, altitude)
    pub z: f32,
}

impl Location {
    /// Creates a new location with the given coordinates.
    #[inline]
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    /// Returns the zero location (origin).
    #[inline]
    pub fn zero() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }
    }

    /// Calculates the Euclidean distance to another location.
    pub fn distance(&self, other: &Location) -> f32 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        let dz = self.z - other.z;
        (dx * dx + dy * dy + dz * dz).sqrt()
    }

    /// Calculates the squared distance (avoids sqrt for performance).
    pub fn distance_squared(&self, other: &Location) -> f32 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        let dz = self.z - other.z;
        dx * dx + dy * dy + dz * dz
    }

    /// Calculates the 2D distance (ignoring Z) to another location.
    pub fn distance_2d(&self, other: &Location) -> f32 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        (dx * dx + dy * dy).sqrt()
    }

    /// Creates from nalgebra Translation3.
    pub fn from_na_translation(from: &Translation3<f32>) -> Self {
        Self {
            x: from.x,
            y: from.y,
            z: from.z,
        }
    }

    /// Creates from nalgebra Point3.
    pub fn from_na_point(from: &Point3<f32>) -> Self {
        Self {
            x: from.x,
            y: from.y,
            z: from.z,
        }
    }

    /// Converts to nalgebra Translation3.
    pub fn to_na_translation(&self) -> Translation3<f32> {
        Translation3::new(self.x, self.y, self.z)
    }

    /// Converts to nalgebra Point3.
    pub fn to_na_point(&self) -> Point3<f32> {
        Point3::new(self.x, self.y, self.z)
    }

    /// Returns a reference to the underlying FFI type.
    #[inline(always)]
    pub fn as_ffi(&self) -> &FfiLocation {
        // Safety: repr(C) with verified layout
        unsafe { std::mem::transmute(self) }
    }

    /// Converts from the FFI type.
    #[inline(always)]
    pub fn from_ffi(ffi: FfiLocation) -> Self {
        // Safety: repr(C) with verified layout
        unsafe { std::mem::transmute(ffi) }
    }

    /// Converts into the FFI type.
    #[inline(always)]
    pub fn into_ffi(self) -> FfiLocation {
        // Safety: repr(C) with verified layout
        unsafe { std::mem::transmute(self) }
    }
}

// Compile-time layout verification
static_assertions::assert_eq_size!(Location, FfiLocation);
static_assertions::assert_eq_align!(Location, FfiLocation);
static_assertions::assert_eq_size!(Location, [f32; 3]);

// Field offset verification
static_assertions::const_assert_eq!(memoffset::offset_of!(Location, x), 0);
static_assertions::const_assert_eq!(memoffset::offset_of!(Location, y), 4);
static_assertions::const_assert_eq!(memoffset::offset_of!(Location, z), 8);

// Arithmetic operators for Location
use std::ops::{Add, AddAssign, Sub, SubAssign};

// Addition: Location + Location
impl Add for Location {
    type Output = Location;
    fn add(self, rhs: Location) -> Location {
        Location {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
        }
    }
}

impl AddAssign for Location {
    fn add_assign(&mut self, rhs: Location) {
        self.x += rhs.x;
        self.y += rhs.y;
        self.z += rhs.z;
    }
}

// Subtraction: Location - Location
impl Sub for Location {
    type Output = Location;
    fn sub(self, rhs: Location) -> Location {
        Location {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
        }
    }
}

impl SubAssign for Location {
    fn sub_assign(&mut self, rhs: Location) {
        self.x -= rhs.x;
        self.y -= rhs.y;
        self.z -= rhs.z;
    }
}

// Reference variants for borrowing
impl Add<&Location> for &Location {
    type Output = Location;
    fn add(self, rhs: &Location) -> Location {
        Location {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
        }
    }
}

impl Sub<&Location> for &Location {
    type Output = Location;
    fn sub(self, rhs: &Location) -> Location {
        Location {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
        }
    }
}

/// 3D rotation using Euler angles in degrees.
///
/// CARLA uses intrinsic Euler angles with the rotation order: roll → pitch → yaw.
/// All angles are in degrees, following Unreal Engine convention.
///
/// # Examples
///
/// ```
/// use carla::geom::Rotation;
///
/// let rot = Rotation::new(0.0, 90.0, 0.0); // 90° yaw
/// ```
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Rotation {
    /// Pitch angle in degrees (rotation around Y-axis).
    /// Positive pitch rotates nose up.
    pub pitch: f32,
    /// Yaw angle in degrees (rotation around Z-axis).
    /// Positive yaw rotates counter-clockwise when looking down.
    pub yaw: f32,
    /// Roll angle in degrees (rotation around X-axis).
    /// Positive roll rotates right wing down.
    pub roll: f32,
}

impl Rotation {
    /// Creates a new rotation with the given angles in degrees.
    #[inline]
    pub fn new(pitch: f32, yaw: f32, roll: f32) -> Self {
        Self { pitch, yaw, roll }
    }

    /// Returns the identity rotation (all zeros).
    #[inline]
    pub fn identity() -> Self {
        Self {
            pitch: 0.0,
            yaw: 0.0,
            roll: 0.0,
        }
    }

    /// Gets the forward vector for this rotation.
    ///
    /// **C++ API:** `GetForwardVector()`
    pub fn forward_vector(&self) -> Vector3D {
        // SAFETY: carla::geom::Vector3D and FfiVector3D have identical memory layout
        unsafe {
            let cpp_vec = self.as_ffi().GetForwardVector();
            Vector3D::from_ffi((&cpp_vec as *const _ as *const FfiVector3D).read())
        }
    }

    /// Gets the right vector for this rotation.
    ///
    /// **C++ API:** `GetRightVector()`
    pub fn right_vector(&self) -> Vector3D {
        // SAFETY: carla::geom::Vector3D and FfiVector3D have identical memory layout
        unsafe {
            let cpp_vec = self.as_ffi().GetRightVector();
            Vector3D::from_ffi((&cpp_vec as *const _ as *const FfiVector3D).read())
        }
    }

    /// Gets the up vector for this rotation.
    ///
    /// **C++ API:** `GetUpVector()`
    pub fn up_vector(&self) -> Vector3D {
        // SAFETY: carla::geom::Vector3D and FfiVector3D have identical memory layout
        unsafe {
            let cpp_vec = self.as_ffi().GetUpVector();
            Vector3D::from_ffi((&cpp_vec as *const _ as *const FfiVector3D).read())
        }
    }

    /// Rotates a vector by this rotation.
    ///
    /// Applies the rotation transformation: Rz(yaw) * Ry(pitch) * Rx(roll)
    ///
    /// **C++ API:** `RotateVector(Vector3D&)` and `RotateVector(const Vector3D&) const`
    pub fn rotate_vector(&self, point: &Vector3D) -> Vector3D {
        // SAFETY: FfiVector3D and carla::geom::Vector3D have identical memory layout
        unsafe {
            let cpp_input =
                &*(point.as_ffi() as *const FfiVector3D as *const carla_sys::carla::geom::Vector3D);
            let cpp_result = self.as_ffi().RotateVector(cpp_input);
            Vector3D::from_ffi((&cpp_result as *const _ as *const FfiVector3D).read())
        }
    }

    /// Applies the inverse rotation to a vector.
    ///
    /// Uses the transpose of the rotation matrix (the rotation inverse).
    ///
    /// **C++ API:** `InverseRotateVector(Vector3D&)`
    pub fn inverse_rotate_vector(&self, point: &Vector3D) -> Vector3D {
        // SAFETY: FfiVector3D and carla::geom::Vector3D have identical memory layout
        unsafe {
            let cpp_input =
                &*(point.as_ffi() as *const FfiVector3D as *const carla_sys::carla::geom::Vector3D);
            let cpp_result = self.as_ffi().InverseRotateVector(cpp_input);
            Vector3D::from_ffi((&cpp_result as *const _ as *const FfiVector3D).read())
        }
    }

    /// Creates from nalgebra UnitQuaternion.
    pub fn from_na(from: &UnitQuaternion<f32>) -> Self {
        let (roll, pitch, yaw) = from.euler_angles();
        Self {
            pitch: pitch.to_degrees(),
            yaw: yaw.to_degrees(),
            roll: roll.to_degrees(),
        }
    }

    /// Converts to nalgebra `UnitQuaternion`.
    ///
    /// **IMPORTANT**: The resulting quaternion represents rotations in
    /// CARLA's **left-handed** coordinate system. CARLA's Euler angles
    /// (in degrees) are converted to radians and then to a quaternion
    /// using intrinsic XYZ rotation order.
    pub fn to_na(&self) -> UnitQuaternion<f32> {
        UnitQuaternion::from_euler_angles(
            self.roll.to_radians(),
            self.pitch.to_radians(),
            self.yaw.to_radians(),
        )
    }

    /// Returns a reference to the underlying FFI type.
    #[inline(always)]
    pub fn as_ffi(&self) -> &FfiRotation {
        // Safety: repr(C) with verified layout
        unsafe { std::mem::transmute(self) }
    }

    /// Converts from the FFI type.
    #[inline(always)]
    pub fn from_ffi(ffi: FfiRotation) -> Self {
        // Safety: repr(C) with verified layout
        unsafe { std::mem::transmute(ffi) }
    }

    /// Converts into the FFI type.
    #[inline(always)]
    pub fn into_ffi(self) -> FfiRotation {
        // Safety: repr(C) with verified layout
        unsafe { std::mem::transmute(self) }
    }
}

// Compile-time layout verification
static_assertions::assert_eq_size!(Rotation, FfiRotation);
static_assertions::assert_eq_align!(Rotation, FfiRotation);
static_assertions::assert_eq_size!(Rotation, [f32; 3]);

// Field offset verification
static_assertions::const_assert_eq!(memoffset::offset_of!(Rotation, pitch), 0);
static_assertions::const_assert_eq!(memoffset::offset_of!(Rotation, yaw), 4);
static_assertions::const_assert_eq!(memoffset::offset_of!(Rotation, roll), 8);

/// 3D vector for directions and velocities.
///
/// Unlike Location, Vector3D represents directions, offsets, and velocities
/// rather than absolute positions. Supports full arithmetic operations including
/// scalar multiplication and division.
///
/// # Examples
///
/// ```
/// use carla::geom::Vector3D;
///
/// let v1 = Vector3D::new(1.0, 2.0, 3.0);
/// let v2 = Vector3D::new(4.0, 5.0, 6.0);
///
/// // Vector arithmetic
/// let sum = v1 + v2;
/// let scaled = v1 * 2.0;
/// let normalized = v1.normalize();
/// ```
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vector3D {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vector3D {
    /// Creates a new vector with the given components.
    #[inline]
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    /// Returns the zero vector.
    #[inline]
    pub fn zero() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }
    }

    /// Calculates the length (magnitude) of the vector.
    pub fn length(&self) -> f32 {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    /// Calculates the squared length (avoids sqrt for performance).
    pub fn length_squared(&self) -> f32 {
        self.x * self.x + self.y * self.y + self.z * self.z
    }

    /// Returns a normalized (unit length) version of this vector.
    pub fn normalize(&self) -> Vector3D {
        let len = self.length();
        if len > 0.0 {
            *self / len
        } else {
            *self
        }
    }

    /// Calculates the dot product with another vector.
    pub fn dot(&self, other: &Vector3D) -> f32 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    /// Calculates the cross product with another vector.
    /// Note: Result depends on coordinate system handedness.
    pub fn cross(&self, other: &Vector3D) -> Vector3D {
        Vector3D {
            x: self.y * other.z - self.z * other.y,
            y: self.z * other.x - self.x * other.z,
            z: self.x * other.y - self.y * other.x,
        }
    }

    /// Returns absolute value of each component.
    pub fn abs(&self) -> Vector3D {
        Vector3D {
            x: self.x.abs(),
            y: self.y.abs(),
            z: self.z.abs(),
        }
    }

    /// Creates from nalgebra Vector3.
    pub fn from_na(from: &Vector3<f32>) -> Self {
        Self {
            x: from.x,
            y: from.y,
            z: from.z,
        }
    }

    /// Converts to nalgebra Vector3.
    pub fn to_na(&self) -> Vector3<f32> {
        Vector3::new(self.x, self.y, self.z)
    }

    /// Returns a reference to the underlying FFI type.
    #[inline(always)]
    pub fn as_ffi(&self) -> &FfiVector3D {
        // Safety: repr(C) with verified layout
        unsafe { std::mem::transmute(self) }
    }

    /// Converts from the FFI type.
    #[inline(always)]
    pub fn from_ffi(ffi: FfiVector3D) -> Self {
        // Safety: repr(C) with verified layout
        unsafe { std::mem::transmute(ffi) }
    }

    /// Converts into the FFI type.
    #[inline(always)]
    pub fn into_ffi(self) -> FfiVector3D {
        // Safety: repr(C) with verified layout
        unsafe { std::mem::transmute(self) }
    }
}

// Compile-time layout verification
static_assertions::assert_eq_size!(Vector3D, FfiVector3D);
static_assertions::assert_eq_align!(Vector3D, FfiVector3D);
static_assertions::assert_eq_size!(Vector3D, [f32; 3]);

// Field offset verification
static_assertions::const_assert_eq!(memoffset::offset_of!(Vector3D, x), 0);
static_assertions::const_assert_eq!(memoffset::offset_of!(Vector3D, y), 4);
static_assertions::const_assert_eq!(memoffset::offset_of!(Vector3D, z), 8);

// Arithmetic operators for Vector3D
use std::ops::{Div, DivAssign, Mul, MulAssign};

// Vector + Vector
impl Add for Vector3D {
    type Output = Vector3D;
    fn add(self, rhs: Vector3D) -> Vector3D {
        Vector3D {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
        }
    }
}

impl AddAssign for Vector3D {
    fn add_assign(&mut self, rhs: Vector3D) {
        self.x += rhs.x;
        self.y += rhs.y;
        self.z += rhs.z;
    }
}

// Vector - Vector
impl Sub for Vector3D {
    type Output = Vector3D;
    fn sub(self, rhs: Vector3D) -> Vector3D {
        Vector3D {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
        }
    }
}

impl SubAssign for Vector3D {
    fn sub_assign(&mut self, rhs: Vector3D) {
        self.x -= rhs.x;
        self.y -= rhs.y;
        self.z -= rhs.z;
    }
}

// Vector - scalar
impl Sub<f32> for Vector3D {
    type Output = Vector3D;
    fn sub(self, scalar: f32) -> Vector3D {
        Vector3D {
            x: self.x - scalar,
            y: self.y - scalar,
            z: self.z - scalar,
        }
    }
}

impl SubAssign<f32> for Vector3D {
    fn sub_assign(&mut self, scalar: f32) {
        self.x -= scalar;
        self.y -= scalar;
        self.z -= scalar;
    }
}

// Vector * scalar
impl Mul<f32> for Vector3D {
    type Output = Vector3D;
    fn mul(self, scalar: f32) -> Vector3D {
        Vector3D {
            x: self.x * scalar,
            y: self.y * scalar,
            z: self.z * scalar,
        }
    }
}

// scalar * Vector (commutative)
impl Mul<Vector3D> for f32 {
    type Output = Vector3D;
    fn mul(self, vector: Vector3D) -> Vector3D {
        vector * self
    }
}

impl MulAssign<f32> for Vector3D {
    fn mul_assign(&mut self, scalar: f32) {
        self.x *= scalar;
        self.y *= scalar;
        self.z *= scalar;
    }
}

// Vector / scalar
impl Div<f32> for Vector3D {
    type Output = Vector3D;
    fn div(self, scalar: f32) -> Vector3D {
        Vector3D {
            x: self.x / scalar,
            y: self.y / scalar,
            z: self.z / scalar,
        }
    }
}

// scalar / Vector
impl Div<Vector3D> for f32 {
    type Output = Vector3D;
    fn div(self, vector: Vector3D) -> Vector3D {
        Vector3D {
            x: self / vector.x,
            y: self / vector.y,
            z: self / vector.z,
        }
    }
}

impl DivAssign<f32> for Vector3D {
    fn div_assign(&mut self, scalar: f32) {
        self.x /= scalar;
        self.y /= scalar;
        self.z /= scalar;
    }
}

// Reference variants for ergonomics
impl Add<&Vector3D> for &Vector3D {
    type Output = Vector3D;
    fn add(self, rhs: &Vector3D) -> Vector3D {
        Vector3D {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
        }
    }
}

impl Sub<&Vector3D> for &Vector3D {
    type Output = Vector3D;
    fn sub(self, rhs: &Vector3D) -> Vector3D {
        Vector3D {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
        }
    }
}

impl Mul<f32> for &Vector3D {
    type Output = Vector3D;
    fn mul(self, scalar: f32) -> Vector3D {
        Vector3D {
            x: self.x * scalar,
            y: self.y * scalar,
            z: self.z * scalar,
        }
    }
}

impl Div<f32> for &Vector3D {
    type Output = Vector3D;
    fn div(self, scalar: f32) -> Vector3D {
        Vector3D {
            x: self.x / scalar,
            y: self.y / scalar,
            z: self.z / scalar,
        }
    }
}

/// 2D vector for 2D positions and directions.
///
/// Similar to Vector3D but for 2D space. Supports full arithmetic operations.
///
/// # Examples
///
/// ```
/// use carla::geom::Vector2D;
///
/// let v1 = Vector2D::new(3.0, 4.0);
/// let v2 = Vector2D::new(1.0, 2.0);
///
/// let sum = v1 + v2;
/// let scaled = v1 * 2.0;
/// let length = v1.length();
/// ```
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vector2D {
    pub x: f32,
    pub y: f32,
}

impl Vector2D {
    /// Creates a new vector with the given components.
    #[inline]
    pub fn new(x: f32, y: f32) -> Self {
        Self { x, y }
    }

    /// Returns the zero vector.
    #[inline]
    pub fn zero() -> Self {
        Self { x: 0.0, y: 0.0 }
    }

    /// Calculates the length (magnitude) of the vector.
    pub fn length(&self) -> f32 {
        (self.x * self.x + self.y * self.y).sqrt()
    }

    /// Calculates the squared length (avoids sqrt for performance).
    pub fn length_squared(&self) -> f32 {
        self.x * self.x + self.y * self.y
    }

    /// Returns a normalized (unit length) version of this vector.
    pub fn normalize(&self) -> Vector2D {
        let len = self.length();
        if len > 0.0 {
            *self / len
        } else {
            *self
        }
    }

    /// Calculates the dot product with another vector.
    pub fn dot(&self, other: &Vector2D) -> f32 {
        self.x * other.x + self.y * other.y
    }

    /// Creates from nalgebra Vector2.
    pub fn from_na(from: &Vector2<f32>) -> Self {
        Self {
            x: from.x,
            y: from.y,
        }
    }

    /// Converts to nalgebra Vector2.
    pub fn to_na(&self) -> Vector2<f32> {
        Vector2::new(self.x, self.y)
    }

    /// Returns a reference to the underlying FFI type.
    #[inline(always)]
    pub fn as_ffi(&self) -> &FfiVector2D {
        // Safety: repr(C) with verified layout
        unsafe { std::mem::transmute(self) }
    }

    /// Converts from the FFI type.
    #[inline(always)]
    pub fn from_ffi(ffi: FfiVector2D) -> Self {
        // Safety: repr(C) with verified layout
        unsafe { std::mem::transmute(ffi) }
    }

    /// Converts into the FFI type.
    #[inline(always)]
    pub fn into_ffi(self) -> FfiVector2D {
        // Safety: repr(C) with verified layout
        unsafe { std::mem::transmute(self) }
    }
}

// Compile-time layout verification
static_assertions::assert_eq_size!(Vector2D, FfiVector2D);
static_assertions::assert_eq_align!(Vector2D, FfiVector2D);
static_assertions::assert_eq_size!(Vector2D, [f32; 2]);

// Field offset verification
static_assertions::const_assert_eq!(memoffset::offset_of!(Vector2D, x), 0);
static_assertions::const_assert_eq!(memoffset::offset_of!(Vector2D, y), 4);

// Arithmetic operators for Vector2D (same pattern as Vector3D)

// Vector + Vector
impl Add for Vector2D {
    type Output = Vector2D;
    fn add(self, rhs: Vector2D) -> Vector2D {
        Vector2D {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
        }
    }
}

impl AddAssign for Vector2D {
    fn add_assign(&mut self, rhs: Vector2D) {
        self.x += rhs.x;
        self.y += rhs.y;
    }
}

// Vector - Vector
impl Sub for Vector2D {
    type Output = Vector2D;
    fn sub(self, rhs: Vector2D) -> Vector2D {
        Vector2D {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
        }
    }
}

impl SubAssign for Vector2D {
    fn sub_assign(&mut self, rhs: Vector2D) {
        self.x -= rhs.x;
        self.y -= rhs.y;
    }
}

// Vector - scalar
impl Sub<f32> for Vector2D {
    type Output = Vector2D;
    fn sub(self, scalar: f32) -> Vector2D {
        Vector2D {
            x: self.x - scalar,
            y: self.y - scalar,
        }
    }
}

impl SubAssign<f32> for Vector2D {
    fn sub_assign(&mut self, scalar: f32) {
        self.x -= scalar;
        self.y -= scalar;
    }
}

// Vector * scalar
impl Mul<f32> for Vector2D {
    type Output = Vector2D;
    fn mul(self, scalar: f32) -> Vector2D {
        Vector2D {
            x: self.x * scalar,
            y: self.y * scalar,
        }
    }
}

// scalar * Vector (commutative)
impl Mul<Vector2D> for f32 {
    type Output = Vector2D;
    fn mul(self, vector: Vector2D) -> Vector2D {
        vector * self
    }
}

impl MulAssign<f32> for Vector2D {
    fn mul_assign(&mut self, scalar: f32) {
        self.x *= scalar;
        self.y *= scalar;
    }
}

// Vector / scalar
impl Div<f32> for Vector2D {
    type Output = Vector2D;
    fn div(self, scalar: f32) -> Vector2D {
        Vector2D {
            x: self.x / scalar,
            y: self.y / scalar,
        }
    }
}

// scalar / Vector
impl Div<Vector2D> for f32 {
    type Output = Vector2D;
    fn div(self, vector: Vector2D) -> Vector2D {
        Vector2D {
            x: self / vector.x,
            y: self / vector.y,
        }
    }
}

impl DivAssign<f32> for Vector2D {
    fn div_assign(&mut self, scalar: f32) {
        self.x /= scalar;
        self.y /= scalar;
    }
}

// Reference variants for ergonomics
impl Add<&Vector2D> for &Vector2D {
    type Output = Vector2D;
    fn add(self, rhs: &Vector2D) -> Vector2D {
        Vector2D {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
        }
    }
}

impl Sub<&Vector2D> for &Vector2D {
    type Output = Vector2D;
    fn sub(self, rhs: &Vector2D) -> Vector2D {
        Vector2D {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
        }
    }
}

impl Mul<f32> for &Vector2D {
    type Output = Vector2D;
    fn mul(self, scalar: f32) -> Vector2D {
        Vector2D {
            x: self.x * scalar,
            y: self.y * scalar,
        }
    }
}

impl Div<f32> for &Vector2D {
    type Output = Vector2D;
    fn div(self, scalar: f32) -> Vector2D {
        Vector2D {
            x: self.x / scalar,
            y: self.y / scalar,
        }
    }
}

/// Geographic coordinates (GPS location).
///
/// Represents a position using latitude, longitude, and altitude.
/// Used for geo-referencing and map transformations.
///
/// # Examples
///
/// ```
/// use carla::geom::GeoLocation;
///
/// let geo = GeoLocation::new(37.7749, -122.4194, 10.0);
/// ```
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GeoLocation {
    /// Latitude in degrees (North-South position).
    pub latitude: f64,
    /// Longitude in degrees (East-West position).
    pub longitude: f64,
    /// Altitude in meters above sea level.
    pub altitude: f64,
}

impl GeoLocation {
    /// Creates a new geographic location.
    #[inline]
    pub fn new(latitude: f64, longitude: f64, altitude: f64) -> Self {
        Self {
            latitude,
            longitude,
            altitude,
        }
    }

    /// Returns a reference to the underlying FFI type.
    #[inline(always)]
    pub fn as_ffi(&self) -> &FfiGeoLocation {
        // Safety: repr(C) with verified layout
        unsafe { std::mem::transmute(self) }
    }

    /// Converts from the FFI type.
    #[inline(always)]
    pub fn from_ffi(ffi: FfiGeoLocation) -> Self {
        // Safety: repr(C) with verified layout
        unsafe { std::mem::transmute(ffi) }
    }

    /// Converts into the FFI type.
    #[inline(always)]
    pub fn into_ffi(self) -> FfiGeoLocation {
        // Safety: repr(C) with verified layout
        unsafe { std::mem::transmute(self) }
    }
}

// Compile-time layout verification
static_assertions::assert_eq_size!(GeoLocation, FfiGeoLocation);
static_assertions::assert_eq_align!(GeoLocation, FfiGeoLocation);
static_assertions::assert_eq_size!(GeoLocation, [f64; 3]);

// Field offset verification
static_assertions::const_assert_eq!(memoffset::offset_of!(GeoLocation, latitude), 0);
static_assertions::const_assert_eq!(memoffset::offset_of!(GeoLocation, longitude), 8);
static_assertions::const_assert_eq!(memoffset::offset_of!(GeoLocation, altitude), 16);

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
    /// use carla::geom::{BoundingBox, Location, Vector3D};
    ///
    /// let bbox = BoundingBox::new(Location::new(0.0, 0.0, 1.0), Vector3D::new(2.0, 1.0, 1.0));
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
            location: Location::from_na_translation(&transform.translation).into_ffi(),
            rotation: Rotation::from_na(&transform.rotation).into_ffi(),
            extent: Vector3D::from_na(extent).into_ffi(),
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
                rotation: Rotation::from_ffi(rotation.clone()).to_na(),
                translation: Location::from_ffi(location.clone()).to_na_translation(),
            },
            extent: Vector3D::from_ffi(extent.clone()).to_na(),
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
            Location::from_na_translation(in_world_point).as_ffi(),
            Transform::from_na(in_bbox_to_world_transform).as_ffi(),
        )
    }

    /// Returns the 8 corner vertices in local (bounding box) coordinates.
    pub fn local_vertices(&self) -> Vec<Location> {
        self.to_native()
            .GetLocalVertices()
            .iter()
            .map(|loc| Location::from_ffi(loc.clone()))
            .collect()
    }

    /// Returns the 8 corner vertices in world coordinates.
    ///
    /// # Arguments
    ///
    /// * `transform` - Transform from bounding box space to world space
    pub fn world_vertices(&self, transform: &Transform) -> Vec<Location> {
        self.to_native()
            .GetWorldVertices(transform.as_ffi())
            .iter()
            .map(|loc| Location::from_ffi(loc.clone()))
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
            location: FfiLocation {
                x: 10.5,
                y: 20.3,
                z: 5.7,
            },
            rotation: FfiRotation {
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

assert_impl_all!(Location: Send, Sync);
assert_impl_all!(Rotation: Send, Sync);
assert_impl_all!(Vector2D: Send, Sync);
assert_impl_all!(Vector3D: Send, Sync);
assert_impl_all!(GeoLocation: Send, Sync);
assert_impl_all!(Transform: Send, Sync);
assert_impl_all!(NativeBoundingBox: Send, Sync);
