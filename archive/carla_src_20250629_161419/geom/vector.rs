//! Vector types for 2D and 3D operations.

use super::{FromCxx, ToCxx};
use nalgebra::{Vector2, Vector3};
use std::ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Neg, Sub, SubAssign};

/// Represents a 2D vector.
///
/// This is equivalent to `carla::geom::Vector2D` in the C++ API.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct Vector2D {
    /// X component
    pub x: f32,
    /// Y component
    pub y: f32,
}

impl Vector2D {
    /// Create a new 2D vector.
    pub fn new(x: f32, y: f32) -> Self {
        Self { x, y }
    }

    /// Create a zero vector.
    pub fn zero() -> Self {
        Self::new(0.0, 0.0)
    }

    /// Create a unit vector along the X axis.
    pub fn unit_x() -> Self {
        Self::new(1.0, 0.0)
    }

    /// Create a unit vector along the Y axis.
    pub fn unit_y() -> Self {
        Self::new(0.0, 1.0)
    }

    /// Calculate the length (magnitude) of the vector.
    pub fn length(&self) -> f32 {
        (self.x * self.x + self.y * self.y).sqrt()
    }

    /// Calculate the squared length (faster than length()).
    pub fn length_squared(&self) -> f32 {
        self.x * self.x + self.y * self.y
    }

    /// Normalize the vector to unit length.
    pub fn normalize(&mut self) {
        let len = self.length();
        if len > 0.0 {
            self.x /= len;
            self.y /= len;
        }
    }

    /// Return a normalized copy of the vector.
    pub fn normalized(&self) -> Self {
        let mut result = *self;
        result.normalize();
        result
    }

    /// Calculate the dot product with another vector.
    pub fn dot(&self, other: &Vector2D) -> f32 {
        self.x * other.x + self.y * other.y
    }

    /// Calculate the cross product (returns scalar in 2D).
    pub fn cross(&self, other: &Vector2D) -> f32 {
        self.x * other.y - self.y * other.x
    }

    /// Calculate the distance to another vector.
    pub fn distance(&self, other: &Vector2D) -> f32 {
        (*self - *other).length()
    }

    /// Calculate the squared distance to another vector.
    pub fn distance_squared(&self, other: &Vector2D) -> f32 {
        (*self - *other).length_squared()
    }

    /// Linear interpolation between two vectors.
    pub fn lerp(&self, other: &Vector2D, t: f32) -> Vector2D {
        Vector2D::new(
            self.x + t * (other.x - self.x),
            self.y + t * (other.y - self.y),
        )
    }

    /// Check if vector is approximately equal to another.
    pub fn approx_eq(&self, other: &Vector2D, epsilon: f32) -> bool {
        (self.x - other.x).abs() < epsilon && (self.y - other.y).abs() < epsilon
    }

    /// Convert to nalgebra Vector2.
    pub fn to_vector2(&self) -> Vector2<f32> {
        Vector2::new(self.x, self.y)
    }

    /// Create from nalgebra Vector2.
    pub fn from_vector2(vector: &Vector2<f32>) -> Self {
        Self::new(vector.x, vector.y)
    }

    /// Get the angle of this vector in radians.
    pub fn angle(&self) -> f32 {
        self.y.atan2(self.x)
    }

    /// Get the angle of this vector in degrees.
    pub fn angle_degrees(&self) -> f32 {
        self.angle().to_degrees()
    }

    /// Create a vector from angle and magnitude.
    pub fn from_angle(angle: f32, magnitude: f32) -> Self {
        Self::new(angle.cos() * magnitude, angle.sin() * magnitude)
    }
}

/// Represents a 3D vector.
///
/// This is equivalent to `carla::geom::Vector3D` in the C++ API.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct Vector3D {
    /// X component
    pub x: f32,
    /// Y component
    pub y: f32,
    /// Z component
    pub z: f32,
}

impl Vector3D {
    /// Create a new 3D vector.
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    /// Create a zero vector.
    pub fn zero() -> Self {
        Self::new(0.0, 0.0, 0.0)
    }

    /// Create a unit vector along the X axis.
    pub fn unit_x() -> Self {
        Self::new(1.0, 0.0, 0.0)
    }

    /// Create a unit vector along the Y axis.
    pub fn unit_y() -> Self {
        Self::new(0.0, 1.0, 0.0)
    }

    /// Create a unit vector along the Z axis.
    pub fn unit_z() -> Self {
        Self::new(0.0, 0.0, 1.0)
    }

    /// Calculate the length (magnitude) of the vector.
    pub fn length(&self) -> f32 {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    /// Calculate the squared length (faster than length()).
    pub fn length_squared(&self) -> f32 {
        self.x * self.x + self.y * self.y + self.z * self.z
    }

    /// Calculate the 2D length (ignoring Z component).
    pub fn length_2d(&self) -> f32 {
        (self.x * self.x + self.y * self.y).sqrt()
    }

    /// Calculate the squared 2D length.
    pub fn length_2d_squared(&self) -> f32 {
        self.x * self.x + self.y * self.y
    }

    /// Normalize the vector to unit length.
    pub fn normalize(&mut self) {
        let len = self.length();
        if len > 0.0 {
            self.x /= len;
            self.y /= len;
            self.z /= len;
        }
    }

    /// Return a normalized copy of the vector.
    pub fn normalized(&self) -> Self {
        let mut result = *self;
        result.normalize();
        result
    }

    /// Calculate the dot product with another vector.
    pub fn dot(&self, other: &Vector3D) -> f32 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    /// Calculate the cross product with another vector.
    pub fn cross(&self, other: &Vector3D) -> Vector3D {
        Vector3D::new(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x,
        )
    }

    /// Calculate the distance to another vector.
    pub fn distance(&self, other: &Vector3D) -> f32 {
        (*self - *other).length()
    }

    /// Calculate the squared distance to another vector.
    pub fn distance_squared(&self, other: &Vector3D) -> f32 {
        (*self - *other).length_squared()
    }

    /// Calculate the 2D distance to another vector (ignoring Z).
    pub fn distance_2d(&self, other: &Vector3D) -> f32 {
        ((self.x - other.x).powi(2) + (self.y - other.y).powi(2)).sqrt()
    }

    /// Linear interpolation between two vectors.
    pub fn lerp(&self, other: &Vector3D, t: f32) -> Vector3D {
        Vector3D::new(
            self.x + t * (other.x - self.x),
            self.y + t * (other.y - self.y),
            self.z + t * (other.z - self.z),
        )
    }

    /// Check if vector is approximately equal to another.
    pub fn approx_eq(&self, other: &Vector3D, epsilon: f32) -> bool {
        (self.x - other.x).abs() < epsilon
            && (self.y - other.y).abs() < epsilon
            && (self.z - other.z).abs() < epsilon
    }

    /// Convert to nalgebra Vector3.
    pub fn to_vector3(&self) -> Vector3<f32> {
        Vector3::new(self.x, self.y, self.z)
    }

    /// Create from nalgebra Vector3.
    pub fn from_vector3(vector: &Vector3<f32>) -> Self {
        Self::new(vector.x, vector.y, vector.z)
    }

    /// Get the XY components as a 2D vector.
    pub fn xy(&self) -> Vector2D {
        Vector2D::new(self.x, self.y)
    }

    /// Create from 2D vector with Z component.
    pub fn from_vector2d(vector: &Vector2D, z: f32) -> Self {
        Self::new(vector.x, vector.y, z)
    }
}

// Default implementations
impl Default for Vector2D {
    fn default() -> Self {
        Self::zero()
    }
}

impl Default for Vector3D {
    fn default() -> Self {
        Self::zero()
    }
}

// Conversion implementations for Vector2D
impl From<(f32, f32)> for Vector2D {
    fn from((x, y): (f32, f32)) -> Self {
        Self::new(x, y)
    }
}

impl From<Vector2D> for (f32, f32) {
    fn from(vec: Vector2D) -> Self {
        (vec.x, vec.y)
    }
}

impl From<[f32; 2]> for Vector2D {
    fn from([x, y]: [f32; 2]) -> Self {
        Self::new(x, y)
    }
}

impl From<Vector2D> for [f32; 2] {
    fn from(vec: Vector2D) -> Self {
        [vec.x, vec.y]
    }
}

impl From<Vector2<f32>> for Vector2D {
    fn from(vec: Vector2<f32>) -> Self {
        Self::from_vector2(&vec)
    }
}

impl From<Vector2D> for Vector2<f32> {
    fn from(vec: Vector2D) -> Self {
        vec.to_vector2()
    }
}

// Conversion implementations for Vector3D
impl From<(f32, f32, f32)> for Vector3D {
    fn from((x, y, z): (f32, f32, f32)) -> Self {
        Self::new(x, y, z)
    }
}

impl From<Vector3D> for (f32, f32, f32) {
    fn from(vec: Vector3D) -> Self {
        (vec.x, vec.y, vec.z)
    }
}

impl From<[f32; 3]> for Vector3D {
    fn from([x, y, z]: [f32; 3]) -> Self {
        Self::new(x, y, z)
    }
}

impl From<Vector3D> for [f32; 3] {
    fn from(vec: Vector3D) -> Self {
        [vec.x, vec.y, vec.z]
    }
}

impl From<Vector3<f32>> for Vector3D {
    fn from(vec: Vector3<f32>) -> Self {
        Self::from_vector3(&vec)
    }
}

impl From<Vector3D> for Vector3<f32> {
    fn from(vec: Vector3D) -> Self {
        vec.to_vector3()
    }
}

impl From<Vector2D> for Vector3D {
    fn from(vec: Vector2D) -> Self {
        Self::from_vector2d(&vec, 0.0)
    }
}

// Arithmetic operations for Vector2D
impl Add for Vector2D {
    type Output = Vector2D;
    fn add(self, rhs: Vector2D) -> Self::Output {
        Vector2D::new(self.x + rhs.x, self.y + rhs.y)
    }
}

impl Sub for Vector2D {
    type Output = Vector2D;
    fn sub(self, rhs: Vector2D) -> Self::Output {
        Vector2D::new(self.x - rhs.x, self.y - rhs.y)
    }
}

impl Mul<f32> for Vector2D {
    type Output = Vector2D;
    fn mul(self, rhs: f32) -> Self::Output {
        Vector2D::new(self.x * rhs, self.y * rhs)
    }
}

impl Div<f32> for Vector2D {
    type Output = Vector2D;
    fn div(self, rhs: f32) -> Self::Output {
        Vector2D::new(self.x / rhs, self.y / rhs)
    }
}

impl Neg for Vector2D {
    type Output = Vector2D;
    fn neg(self) -> Self::Output {
        Vector2D::new(-self.x, -self.y)
    }
}

impl AddAssign for Vector2D {
    fn add_assign(&mut self, rhs: Vector2D) {
        self.x += rhs.x;
        self.y += rhs.y;
    }
}

impl SubAssign for Vector2D {
    fn sub_assign(&mut self, rhs: Vector2D) {
        self.x -= rhs.x;
        self.y -= rhs.y;
    }
}

impl MulAssign<f32> for Vector2D {
    fn mul_assign(&mut self, rhs: f32) {
        self.x *= rhs;
        self.y *= rhs;
    }
}

impl DivAssign<f32> for Vector2D {
    fn div_assign(&mut self, rhs: f32) {
        self.x /= rhs;
        self.y /= rhs;
    }
}

// Arithmetic operations for Vector3D
impl Add for Vector3D {
    type Output = Vector3D;
    fn add(self, rhs: Vector3D) -> Self::Output {
        Vector3D::new(self.x + rhs.x, self.y + rhs.y, self.z + rhs.z)
    }
}

impl Sub for Vector3D {
    type Output = Vector3D;
    fn sub(self, rhs: Vector3D) -> Self::Output {
        Vector3D::new(self.x - rhs.x, self.y - rhs.y, self.z - rhs.z)
    }
}

impl Mul<f32> for Vector3D {
    type Output = Vector3D;
    fn mul(self, rhs: f32) -> Self::Output {
        Vector3D::new(self.x * rhs, self.y * rhs, self.z * rhs)
    }
}

impl Div<f32> for Vector3D {
    type Output = Vector3D;
    fn div(self, rhs: f32) -> Self::Output {
        Vector3D::new(self.x / rhs, self.y / rhs, self.z / rhs)
    }
}

impl Neg for Vector3D {
    type Output = Vector3D;
    fn neg(self) -> Self::Output {
        Vector3D::new(-self.x, -self.y, -self.z)
    }
}

impl AddAssign for Vector3D {
    fn add_assign(&mut self, rhs: Vector3D) {
        self.x += rhs.x;
        self.y += rhs.y;
        self.z += rhs.z;
    }
}

impl SubAssign for Vector3D {
    fn sub_assign(&mut self, rhs: Vector3D) {
        self.x -= rhs.x;
        self.y -= rhs.y;
        self.z -= rhs.z;
    }
}

impl MulAssign<f32> for Vector3D {
    fn mul_assign(&mut self, rhs: f32) {
        self.x *= rhs;
        self.y *= rhs;
        self.z *= rhs;
    }
}

impl DivAssign<f32> for Vector3D {
    fn div_assign(&mut self, rhs: f32) {
        self.x /= rhs;
        self.y /= rhs;
        self.z /= rhs;
    }
}

// Display implementations
impl std::fmt::Display for Vector2D {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Vector2D(x={:.3}, y={:.3})", self.x, self.y)
    }
}

impl std::fmt::Display for Vector3D {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "Vector3D(x={:.3}, y={:.3}, z={:.3})",
            self.x, self.y, self.z
        )
    }
}

// Conversion to/from carla-sys types
impl FromCxx<carla_sys::SimpleVector2D> for Vector2D {
    fn from_cxx(value: carla_sys::SimpleVector2D) -> Self {
        Self::new(value.x as f32, value.y as f32)
    }
}

impl ToCxx<carla_sys::SimpleVector2D> for Vector2D {
    fn to_cxx(&self) -> carla_sys::SimpleVector2D {
        carla_sys::SimpleVector2D {
            x: self.x as f64,
            y: self.y as f64,
        }
    }
}

impl FromCxx<carla_sys::SimpleVector3D> for Vector3D {
    fn from_cxx(value: carla_sys::SimpleVector3D) -> Self {
        Self::new(value.x as f32, value.y as f32, value.z as f32)
    }
}

impl ToCxx<carla_sys::SimpleVector3D> for Vector3D {
    fn to_cxx(&self) -> carla_sys::SimpleVector3D {
        carla_sys::SimpleVector3D {
            x: self.x as f64,
            y: self.y as f64,
            z: self.z as f64,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    // Tests corresponding to C++ test_geom.cpp and test_vector3D.cpp

    #[test]
    fn test_vector_operations() {
        // Test vector operations from test_geom.cpp
        let v1 = Vector3D::new(1.0, 2.0, 3.0);
        let v2 = Vector3D::new(4.0, 5.0, 6.0);

        // Test dot product
        let dot = v1.dot(&v2);
        assert_relative_eq!(dot, 32.0, epsilon = 0.001);

        // Test cross product
        let cross = v1.cross(&v2);
        assert_relative_eq!(cross.x, -3.0, epsilon = 0.001);
        assert_relative_eq!(cross.y, 6.0, epsilon = 0.001);
        assert_relative_eq!(cross.z, -3.0, epsilon = 0.001);
    }

    #[test]
    fn test_vector_unit_vectors() {
        // Test unit vector creation
        let unit_x = Vector3D::unit_x();
        assert_relative_eq!(unit_x.x, 1.0, epsilon = 0.001);
        assert_relative_eq!(unit_x.y, 0.0, epsilon = 0.001);
        assert_relative_eq!(unit_x.z, 0.0, epsilon = 0.001);

        let unit_y = Vector3D::unit_y();
        assert_relative_eq!(unit_y.x, 0.0, epsilon = 0.001);
        assert_relative_eq!(unit_y.y, 1.0, epsilon = 0.001);
        assert_relative_eq!(unit_y.z, 0.0, epsilon = 0.001);

        let unit_z = Vector3D::unit_z();
        assert_relative_eq!(unit_z.x, 0.0, epsilon = 0.001);
        assert_relative_eq!(unit_z.y, 0.0, epsilon = 0.001);
        assert_relative_eq!(unit_z.z, 1.0, epsilon = 0.001);
    }

    #[test]
    fn test_vector_normalization() {
        // Test vector normalization
        let mut v = Vector3D::new(3.0, 4.0, 0.0);
        v.normalize();

        assert_relative_eq!(v.length(), 1.0, epsilon = 0.001);
        assert_relative_eq!(v.x, 0.6, epsilon = 0.001);
        assert_relative_eq!(v.y, 0.8, epsilon = 0.001);
        assert_relative_eq!(v.z, 0.0, epsilon = 0.001);

        // Test normalized() method
        let v2 = Vector3D::new(1.0, 1.0, 1.0);
        let normalized = v2.normalized();
        let expected_component = 1.0 / 3.0_f32.sqrt();

        assert_relative_eq!(normalized.length(), 1.0, epsilon = 0.001);
        assert_relative_eq!(normalized.x, expected_component, epsilon = 0.001);
        assert_relative_eq!(normalized.y, expected_component, epsilon = 0.001);
        assert_relative_eq!(normalized.z, expected_component, epsilon = 0.001);
    }

    #[test]
    fn test_vector_arithmetic() {
        // Test vector arithmetic operations
        let v1 = Vector3D::new(1.0, 2.0, 3.0);
        let v2 = Vector3D::new(4.0, 5.0, 6.0);

        // Addition
        let sum = v1 + v2;
        assert_relative_eq!(sum.x, 5.0, epsilon = 0.001);
        assert_relative_eq!(sum.y, 7.0, epsilon = 0.001);
        assert_relative_eq!(sum.z, 9.0, epsilon = 0.001);

        // Subtraction
        let diff = v2 - v1;
        assert_relative_eq!(diff.x, 3.0, epsilon = 0.001);
        assert_relative_eq!(diff.y, 3.0, epsilon = 0.001);
        assert_relative_eq!(diff.z, 3.0, epsilon = 0.001);

        // Scalar multiplication
        let scaled = v1 * 2.0;
        assert_relative_eq!(scaled.x, 2.0, epsilon = 0.001);
        assert_relative_eq!(scaled.y, 4.0, epsilon = 0.001);
        assert_relative_eq!(scaled.z, 6.0, epsilon = 0.001);

        // Scalar division
        let divided = v2 / 2.0;
        assert_relative_eq!(divided.x, 2.0, epsilon = 0.001);
        assert_relative_eq!(divided.y, 2.5, epsilon = 0.001);
        assert_relative_eq!(divided.z, 3.0, epsilon = 0.001);

        // Negation
        let neg = -v1;
        assert_relative_eq!(neg.x, -1.0, epsilon = 0.001);
        assert_relative_eq!(neg.y, -2.0, epsilon = 0.001);
        assert_relative_eq!(neg.z, -3.0, epsilon = 0.001);
    }

    #[test]
    fn test_vector_length() {
        // Test vector length calculations
        let v = Vector3D::new(3.0, 4.0, 0.0);
        assert_relative_eq!(v.length(), 5.0, epsilon = 0.001);
        assert_relative_eq!(v.length_squared(), 25.0, epsilon = 0.001);
        assert_relative_eq!(v.length_2d(), 5.0, epsilon = 0.001);

        let v2 = Vector3D::new(1.0, 0.0, 0.0);
        assert_relative_eq!(v2.length(), 1.0, epsilon = 0.001);
        assert_relative_eq!(v2.length_squared(), 1.0, epsilon = 0.001);
    }

    #[test]
    fn test_vector_distance() {
        // Test distance calculations
        let v1 = Vector3D::new(0.0, 0.0, 0.0);
        let v2 = Vector3D::new(3.0, 4.0, 0.0);

        assert_relative_eq!(v1.distance(&v2), 5.0, epsilon = 0.001);
        assert_relative_eq!(v1.distance_squared(&v2), 25.0, epsilon = 0.001);
        assert_relative_eq!(v1.distance_2d(&v2), 5.0, epsilon = 0.001);
    }

    #[test]
    fn test_vector_2d_operations() {
        // Test 2D vector operations
        let v1 = Vector2D::new(3.0, 4.0);
        let v2 = Vector2D::new(1.0, 0.0);

        // Length
        assert_relative_eq!(v1.length(), 5.0, epsilon = 0.001);
        assert_relative_eq!(v2.length(), 1.0, epsilon = 0.001);

        // Dot product
        let dot = v1.dot(&v2);
        assert_relative_eq!(dot, 3.0, epsilon = 0.001);

        // Cross product (returns scalar in 2D)
        let cross = v1.cross(&v2);
        assert_relative_eq!(cross, -4.0, epsilon = 0.001);

        // Angle
        let angle = v2.angle();
        assert_relative_eq!(angle, 0.0, epsilon = 0.001);

        let v3 = Vector2D::new(0.0, 1.0);
        let angle3 = v3.angle();
        assert_relative_eq!(angle3, std::f32::consts::PI / 2.0, epsilon = 0.001);
    }
}
