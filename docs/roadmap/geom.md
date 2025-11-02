# Geometry Types Migration

**[← Back to Roadmap Index](../roadmap.md)**

This document tracks the migration of CARLA geometry types from FFI exports to native Rust types with compile-time memory layout verification.

## Overview

Currently, most geometry types in `carla/src/geom.rs` are direct re-exports of FFI types from `carla-sys`. This creates several issues:

1. **No Rust trait implementations**: Cannot implement `std::ops` traits (Add, Sub, Mul, etc.) on foreign types due to orphan rules
2. **Limited type safety**: FFI types expose raw C++ semantics without Rust idiomatic wrappers
3. **Poor user experience**: Users work directly with FFI types instead of clean Rust APIs
4. **No compile-time guarantees**: Layout compatibility is assumed, not verified

The Transform type migration (completed) demonstrates the proper approach:
- Define native Rust struct with `#[repr(C)]` and public fields
- Add comprehensive static_asserts on C++ side (size, alignment, field offsets)
- Add Rust static_assertions using `memoffset` crate for field offset verification
- Use `transmute` for zero-cost FFI conversion with verified identical layouts
- Implement Rust traits (Mul, Add, etc.) on owned types

## Migration Status

| Type        | Status          | C++ Fields                    | Size     | Priority | Notes                                            |
|-------------|-----------------|-------------------------------|----------|----------|--------------------------------------------------|
| Transform   | ✅ **COMPLETE** | location, rotation            | 24 bytes | -        | Migration complete with full layout verification |
| Location    | 📋 **PLANNED**  | x, y, z                       | 12 bytes | HIGH     | Already has FfiLocation wrapper, needs Rust type |
| Rotation    | 📋 **PLANNED**  | pitch, yaw, roll              | 12 bytes | HIGH     | Direct FFI export, needs migration               |
| Vector3D    | 📋 **PLANNED**  | x, y, z                       | 12 bytes | MEDIUM   | Direct FFI export, needs migration               |
| Vector2D    | 📋 **PLANNED**  | x, y                          | 8 bytes  | MEDIUM   | Direct FFI export, needs migration               |
| GeoLocation | 📋 **PLANNED**  | latitude, longitude, altitude | 24 bytes | LOW      | GPS coordinates, less frequently used            |

## Current State

### Direct FFI Exports (Need Migration)

```rust
// carla/src/geom.rs
pub use carla_sys::{
    carla::geom::{GeoLocation, Rotation, Vector2D, Vector3D},
    carla_rust::geom::FfiLocation as Location,
};
```

**Problems:**
- `GeoLocation`, `Rotation`, `Vector2D`, `Vector3D` are foreign types from `carla_sys`
- Cannot implement Rust traits like `Add`, `Sub`, `Mul`, `Div`, `Neg` due to orphan rules
- Users work directly with C++ types, not idiomatic Rust
- No compile-time layout verification

### FfiLocation (Partial Migration)

`FfiLocation` exists in C++ but is still exported as-is:
```cpp
// carla-sys/csrc/carla_rust/geom.hpp
class FfiLocation {
public:
    float x;
    float y;
    float z;
    // ... has layout verification
};
```

This needs to become a native Rust type like Transform.

## Migration Phases

### Phase A: Location Type Migration

**Priority:** HIGH
**Dependencies:** None
**Estimated Effort:** 3-4 hours

#### A.1: C++ Layout Verification ✅ **COMPLETE**

FfiLocation already has static_asserts in `carla-sys/csrc/carla_rust/geom.hpp`:
- ✅ Size verification
- ✅ Alignment verification
- ✅ Field offset verification (x at 0, y at 4, z at 8)

#### A.2: Define Native Rust Location Type

**File:** `carla/src/geom.rs`

Replace FFI export with native Rust struct:

```rust
/// 3D location in CARLA's left-handed Z-up coordinate system.
///
/// Represents a position in 3D space with floating-point precision.
/// This type is compatible with the C++ `carla::geom::Location` type
/// and can be safely transmuted for zero-cost FFI conversion.
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
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    /// Returns the zero location (origin).
    pub fn zero() -> Self {
        Self { x: 0.0, y: 0.0, z: 0.0 }
    }

    /// Calculates the Euclidean distance to another location.
    pub fn distance(&self, other: &Location) -> f32 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        let dz = self.z - other.z;
        (dx * dx + dy * dy + dz * dz).sqrt()
    }

    /// Calculates the 2D distance (ignoring Z) to another location.
    pub fn distance_2d(&self, other: &Location) -> f32 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        (dx * dx + dy * dy).sqrt()
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
```

#### A.3: Implement Arithmetic Operators

**Based on:** C++ `carla::geom::Location` and Python `carla.Location` APIs

The C++ and Python APIs support the following operations:

**C++ API (Location inherits from Vector3D):**
- `operator+` (Location + Location)
- `operator-` (Location - Location)
- `operator+=` (Location += Location)
- `operator-=` (Location -= Location)
- **Note:** Scalar multiplication/division NOT supported on Location (only on Vector3D)

**Python API (`carla.Location`):**
- `__add__` - Addition operator
- `__sub__` - Subtraction operator
- `__mul__` - Multiplication operator (possibly scalar only)
- `__div__` / `__truediv__` - Division operator
- `__neg__` - Negation operator
- `__abs__` - Absolute value operator

**Rust Implementation:**

```rust
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
```

**Note:** Location in C++ does NOT support scalar multiplication/division. Those operations are only on Vector3D. We follow C++ semantics to maintain API consistency.

#### A.4: Migrate LocationExt Trait

Move methods from trait to impl block (similar to Transform):

```rust
impl Location {
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
}
```

Remove LocationExt trait and update all imports.

#### A.5: Update All Usages

Update all code that uses Location:
- Update FFI boundaries to use `as_ffi()`, `from_ffi()`, `into_ffi()`
- Update imports (remove LocationExt from prelude)
- Test all arithmetic operations
- Verify layout with tests

**Files to update:**
- `carla/src/geom.rs` - Core implementation
- `carla/src/lib.rs` - Remove from prelude
- `carla/src/client/*.rs` - Update FFI conversions
- `carla/src/rpc/*.rs` - Update FFI conversions
- `carla/examples/*.rs` - Update example code

### Phase B: Rotation Type Migration

**Priority:** HIGH
**Dependencies:** Phase A (for consistency)
**Estimated Effort:** 3-4 hours

#### B.1: C++ Wrapper and Verification

Create `FfiRotation` wrapper in `carla-sys/csrc/carla_rust/geom.hpp`:

```cpp
// Rotation
class FfiRotation {
public:
    float pitch;
    float yaw;
    float roll;

    FfiRotation(Rotation&& base) {
        *this = std::move(reinterpret_cast<FfiRotation&&>(base));
    }

    FfiRotation(const Rotation& base) {
        *this = reinterpret_cast<const FfiRotation&>(base);
    }

    const Rotation& as_native() const {
        return reinterpret_cast<const Rotation&>(*this);
    }
};

static_assert(sizeof(FfiRotation) == sizeof(Rotation),
              "FfiRotation and Rotation size mismatch");
static_assert(alignof(FfiRotation) == alignof(Rotation),
              "FfiRotation and Rotation alignment mismatch");
static_assert(alignof(FfiRotation) == 4,
              "FfiRotation must have 4-byte alignment");
static_assert(offsetof(FfiRotation, pitch) == 0,
              "FfiRotation pitch must be first field");
static_assert(offsetof(FfiRotation, yaw) == 4,
              "FfiRotation yaw must be second field");
static_assert(offsetof(FfiRotation, roll) == 8,
              "FfiRotation roll must be third field");
```

#### B.2: Define Native Rust Rotation Type

**File:** `carla/src/geom.rs`

```rust
/// 3D rotation using Euler angles in degrees.
///
/// CARLA uses intrinsic Euler angles with the rotation order: roll → pitch → yaw.
/// All angles are in degrees, following Unreal Engine convention.
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
    pub fn new(pitch: f32, yaw: f32, roll: f32) -> Self {
        Self { pitch, yaw, roll }
    }

    /// Returns the identity rotation (all zeros).
    pub fn identity() -> Self {
        Self { pitch: 0.0, yaw: 0.0, roll: 0.0 }
    }

    /// Gets the forward vector for this rotation.
    ///
    /// **C++ API:** `GetForwardVector()`
    pub fn forward_vector(&self) -> Vector3D {
        // Delegates to C++ Math::GetForwardVector via FFI
        // This is complex trigonometry best kept in C++
        todo!("Call FFI method")
    }

    /// Gets the right vector for this rotation.
    ///
    /// **C++ API:** `GetRightVector()`
    pub fn right_vector(&self) -> Vector3D {
        // Delegates to C++ Math::GetRightVector via FFI
        todo!("Call FFI method")
    }

    /// Gets the up vector for this rotation.
    ///
    /// **C++ API:** `GetUpVector()`
    pub fn up_vector(&self) -> Vector3D {
        // Delegates to C++ Math::GetUpVector via FFI
        todo!("Call FFI method")
    }

    /// Rotates a vector by this rotation.
    ///
    /// Applies the rotation transformation: Rz(yaw) * Ry(pitch) * Rx(roll)
    ///
    /// **C++ API:** `RotateVector(Vector3D&)` and `RotateVector(const Vector3D&) const`
    pub fn rotate_vector(&self, point: &Vector3D) -> Vector3D {
        // Delegates to C++ RotateVector method via FFI
        // This is complex matrix math best kept in C++
        todo!("Call FFI method")
    }

    /// Applies the inverse rotation to a vector.
    ///
    /// Uses the transpose of the rotation matrix (the rotation inverse).
    ///
    /// **C++ API:** `InverseRotateVector(Vector3D&)`
    pub fn inverse_rotate_vector(&self, point: &Vector3D) -> Vector3D {
        // Delegates to C++ InverseRotateVector method via FFI
        todo!("Call FFI method")
    }

    /// Converts from nalgebra UnitQuaternion.
    pub fn from_na(from: &UnitQuaternion<f32>) -> Self {
        let (roll, pitch, yaw) = from.euler_angles();
        Self {
            pitch: pitch.to_degrees(),
            yaw: yaw.to_degrees(),
            roll: roll.to_degrees(),
        }
    }

    /// Converts to nalgebra UnitQuaternion.
    pub fn to_na(&self) -> UnitQuaternion<f32> {
        UnitQuaternion::from_euler_angles(
            self.roll.to_radians(),
            self.pitch.to_radians(),
            self.yaw.to_radians(),
        )
    }

    // FFI conversion methods (as_ffi, from_ffi, into_ffi)
    // ... similar to Location
}

// Static assertions for layout verification
// ... similar to Location
```

**Note on Rotation:** The C++ `Rotation` class does NOT have arithmetic operators (no `operator+` or `operator-`). Rotation composition is handled via quaternions or Transform multiplication. We follow this design in Rust.

#### B.3: Implement Rotation Helper Methods via FFI

The following methods require C++ implementation and should be exposed via FFI:

**In `carla-sys/csrc/carla_rust/geom.hpp`:**

```cpp
class FfiRotation {
    // ... existing fields and methods ...

    Vector3D GetForwardVector() const {
        return as_native().GetForwardVector();
    }

    Vector3D GetRightVector() const {
        return as_native().GetRightVector();
    }

    Vector3D GetUpVector() const {
        return as_native().GetUpVector();
    }

    Vector3D RotateVector(const Vector3D& point) const {
        return as_native().RotateVector(point);
    }

    Vector3D InverseRotateVector(const Vector3D& point) const {
        Vector3D result = point;
        as_native().InverseRotateVector(result);
        return result;
    }
};
```

These methods must be declared in `carla-sys/src/ffi.rs` autocxx bindings.

#### B.4: Migrate RotationExt Trait

Move methods to impl block and remove trait.

#### B.4: Update Transform Type

Update Transform to use the new native Rotation:

```rust
#[repr(C)]
#[derive(Debug, Clone)]
pub struct Transform {
    pub location: Location,
    pub rotation: Rotation,  // Now uses native Rust type
}
```

#### B.5: Update All Usages

Similar to Location migration.

### Phase C: Vector3D Type Migration

**Priority:** MEDIUM
**Dependencies:** Phase A, B (for consistency)
**Estimated Effort:** 2-3 hours

#### C.1: C++ Wrapper and Verification

Create `FfiVector3D` in `carla-sys/csrc/carla_rust/geom.hpp` with static_asserts.

#### C.2: Define Native Rust Vector3D Type

```rust
/// 3D vector for directions and velocities.
///
/// Unlike Location, Vector3D represents directions, offsets, and velocities
/// rather than absolute positions.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vector3D {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vector3D {
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    pub fn zero() -> Self {
        Self { x: 0.0, y: 0.0, z: 0.0 }
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

    // nalgebra conversion methods
    pub fn from_na(from: &Vector3<f32>) -> Self {
        Self { x: from.x, y: from.y, z: from.z }
    }

    pub fn to_na(&self) -> Vector3<f32> {
        Vector3::new(self.x, self.y, self.z)
    }

    // FFI conversion methods
    // ...
}

// Arithmetic operators based on C++ API
// C++ supports: +, -, *, / (with both vector and scalar operands)
use std::ops::{Add, AddAssign, Sub, SubAssign, Mul, MulAssign, Div, DivAssign};

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

// Vector - scalar (C++ supports this: vector -= float)
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

// scalar / Vector (C++ supports this)
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

// Static assertions
// ...
```

#### C.3: Migrate Vector3DExt Trait

Move to impl block and remove trait.

#### C.4: Update All Usages

Update imports, FFI boundaries, and remove from prelude.

### Phase D: Vector2D Type Migration

**Priority:** MEDIUM
**Dependencies:** Phase C (for consistency)
**Estimated Effort:** 2-3 hours

Similar to Vector3D but with only x and y fields. The C++ API supports the same operations as Vector3D.

**C++ API (`carla::geom::Vector2D`):**
- `operator+`, `operator+=` (Vector + Vector)
- `operator-`, `operator-=` (Vector - Vector)
- `operator*`, `operator*=` (Vector * scalar, scalar * Vector)
- `operator/`, `operator/=` (Vector / scalar, scalar / Vector)
- `operator==`, `operator!=` (comparison)
- `Length()`, `SquaredLength()` (magnitude)
- `MakeUnitVector()` (normalization)

```rust
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vector2D {
    pub x: f32,
    pub y: f32,
}

impl Vector2D {
    pub fn new(x: f32, y: f32) -> Self {
        Self { x, y }
    }

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

    // nalgebra conversion methods
    pub fn from_na(from: &Vector2<f32>) -> Self {
        Self { x: from.x, y: from.y }
    }

    pub fn to_na(&self) -> Vector2<f32> {
        Vector2::new(self.x, self.y)
    }

    // FFI conversion methods
    // ...
}

// Arithmetic operators - same pattern as Vector3D
// Add, AddAssign, Sub, SubAssign, Mul, MulAssign, Div, DivAssign
// Support for: Vector ± Vector, Vector * scalar, scalar * Vector, Vector / scalar, scalar / Vector
// (See Vector3D implementation for full details)
```

### Phase E: GeoLocation Type Migration

**Priority:** LOW
**Dependencies:** Phase A (uses Location internally)
**Estimated Effort:** 3-4 hours

#### E.1: C++ Wrapper and Verification

Create `FfiGeoLocation` with static_asserts for:
- Size: 24 bytes (3 × f64)
- Alignment: 8 bytes
- Field offsets: latitude (0), longitude (8), altitude (16)

#### E.2: Define Native Rust GeoLocation Type

```rust
/// Geographic coordinates (GPS location).
///
/// Represents a position using latitude, longitude, and altitude.
/// Used for geo-referencing and map transformations.
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
    pub fn new(latitude: f64, longitude: f64, altitude: f64) -> Self {
        Self { latitude, longitude, altitude }
    }

    // Keep existing C++ methods as FFI calls:
    // - Transform(location) -> GeoLocation
    // - GeoLocationToTransform(lat, lon, alt) -> Location
    // These require complex math from C++ side

    // FFI conversion methods
    // ...
}

// Static assertions
// ...
```

**Note:** GeoLocation has complex methods that call C++ implementations. These should remain as FFI calls to the native methods rather than being reimplemented in Rust.

## Summary of Arithmetic Operations

This table summarizes the arithmetic operators to implement for each geometry type, based on the C++ API:

| Type       | Add (`+`)      | Sub (`-`)      | Mul (`*`)       | Div (`/`)       | Assign Ops | Notes                                    |
|------------|----------------|----------------|-----------------|-----------------|------------|------------------------------------------|
| Location   | ✅ Loc + Loc   | ✅ Loc - Loc   | ❌              | ❌              | ✅ +=, -= | No scalar ops (use Vector3D for that)    |
| Vector3D   | ✅ Vec + Vec   | ✅ Vec - Vec   | ✅ Vec * scalar | ✅ Vec / scalar | ✅ All     | Also: Vec - scalar, scalar * Vec, scalar / Vec |
|            |                | ✅ Vec - scalar | ✅ scalar * Vec | ✅ scalar / Vec |            | Supports commutative scalar ops         |
| Vector2D   | ✅ Vec + Vec   | ✅ Vec - Vec   | ✅ Vec * scalar | ✅ Vec / scalar | ✅ All     | Same as Vector3D but 2D                  |
|            |                |                | ✅ scalar * Vec | ✅ scalar / Vec |            |                                          |
| Rotation   | ❌             | ❌             | ❌              | ❌              | ❌         | No arithmetic operators                   |
|            |                |                |                 |                 |            | Use quaternions or Transform for composition |
| Transform  | ❌             | ❌             | ✅ Trans * Trans | ❌             | ❌         | Only multiplication (composition)        |
| GeoLocation| ❌             | ❌             | ❌              | ❌              | ❌         | Specialized GPS operations only          |

**Key Points:**

1. **Location**: Vector addition/subtraction only. No scalar multiplication (that's Vector3D's job).

2. **Vector3D/Vector2D**: Full vector arithmetic
   - Vector-vector operations: `v1 + v2`, `v1 - v2`
   - Vector-scalar operations: `v * 2.0`, `v / 2.0`, `v - 1.0`
   - Scalar-vector operations: `2.0 * v`, `2.0 / v` (commutative)
   - Assignment variants: `+=`, `-=`, `*=`, `/=`

3. **Rotation**: NO arithmetic operators
   - Rotation composition is done via quaternions (`to_na()` → multiply → `from_na()`)
   - Or via Transform multiplication

4. **Transform**: Only multiplication for composition
   - `transform1 * transform2` composes transforms
   - Already implemented ✅

5. **Reference Variants**: Implement for all operators to support borrowing
   - Example: `impl Add<&Vector3D> for &Vector3D`
   - Allows `&v1 + &v2` without moving values

## Testing Strategy

For each phase:

1. **Compile-time verification:**
   - C++ static_asserts ensure layout compatibility
   - Rust static_assertions verify size, alignment, field offsets
   - Build must fail if layouts diverge

2. **Unit tests:**
   - Test arithmetic operators (Add, Sub, Mul, Div, Neg)
   - Test comparison operators (PartialEq, Eq if applicable)
   - Test conversion methods (from_na, to_na)
   - Test FFI round-trip conversion (from_ffi . into_ffi = identity)

3. **Integration tests:**
   - Update existing examples to use new types
   - Run examples against CARLA simulator
   - Verify spatial relationships match expectations

4. **Regression tests:**
   - Ensure all existing functionality still works
   - Check performance (should be identical - zero-cost)
   - Verify memory layout in debugger

## Benefits After Migration

1. **Type Safety:** Native Rust types with Rust semantics
2. **Trait Implementations:** Can implement Add, Sub, Mul, etc. on owned types
3. **Better API:** Users work with idiomatic Rust, not FFI types
4. **Compile-time Guarantees:** Layout verified at compile time on both sides
5. **Documentation:** Better docs with examples using native Rust types
6. **Zero Cost:** Transmute-based conversion maintains zero-cost FFI

## Migration Checklist

- [ ] Phase A: Location Type Migration
  - [ ] A.1: Verify C++ layout checks ✅ **COMPLETE**
  - [ ] A.2: Define native Rust Location
  - [ ] A.3: Implement arithmetic operators
  - [ ] A.4: Migrate LocationExt trait
  - [ ] A.5: Update all usages

- [ ] Phase B: Rotation Type Migration
  - [ ] B.1: C++ wrapper and verification
  - [ ] B.2: Define native Rust Rotation
  - [ ] B.3: Migrate RotationExt trait
  - [ ] B.4: Update Transform type
  - [ ] B.5: Update all usages

- [ ] Phase C: Vector3D Type Migration
  - [ ] C.1: C++ wrapper and verification
  - [ ] C.2: Define native Rust Vector3D
  - [ ] C.3: Migrate Vector3DExt trait
  - [ ] C.4: Update all usages

- [ ] Phase D: Vector2D Type Migration
  - [ ] D.1: C++ wrapper and verification
  - [ ] D.2: Define native Rust Vector2D
  - [ ] D.3: Migrate Vector2DExt trait
  - [ ] D.4: Update all usages

- [ ] Phase E: GeoLocation Type Migration
  - [ ] E.1: C++ wrapper and verification
  - [ ] E.2: Define native Rust GeoLocation
  - [ ] E.3: Handle complex FFI method calls
  - [ ] E.4: Update all usages

- [ ] Final Verification
  - [ ] All tests pass
  - [ ] All examples run successfully
  - [ ] Linter passes (make lint-rust)
  - [ ] Documentation updated
  - [ ] Performance benchmarks (verify zero-cost)

## Notes

- **Order matters:** Location → Rotation → Vector3D → Vector2D → GeoLocation
  - Transform already uses Location and Rotation
  - GeoLocation methods return Location

- **Breaking changes:** This is a breaking API change
  - Users must update imports
  - Extension traits will be removed
  - Methods move from traits to impl blocks
  - Should be done in a major version bump

- **Performance:** Zero-cost abstraction via transmute
  - No runtime overhead
  - Same memory layout as C++ types
  - Compile-time verified

- **Reference:** Transform migration (PR #XXX) as template for all types
