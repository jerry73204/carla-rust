//! Location type for 3D positions.

use super::{FromCxx, ToCxx};
use nalgebra::{Point3, Vector3};
use std::ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Sub, SubAssign};

/// Represents a 3D location in the world.
///
/// This is equivalent to `carla::geom::Location` in the C++ API.
/// Coordinates are in meters, using CARLA's coordinate system:
/// - X: Forward (North)
/// - Y: Right (East)  
/// - Z: Up
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct Location {
    /// X coordinate (forward/north) in meters
    pub x: f64,
    /// Y coordinate (right/east) in meters  
    pub y: f64,
    /// Z coordinate (up) in meters
    pub z: f64,
}

impl Location {
    /// Create a new location.
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    /// Create a location at the origin.
    pub fn zero() -> Self {
        Self::new(0.0, 0.0, 0.0)
    }

    /// Calculate the distance to another location.
    pub fn distance(&self, other: &Location) -> f64 {
        ((other.x - self.x).powi(2) + (other.y - self.y).powi(2) + (other.z - self.z).powi(2))
            .sqrt()
    }

    /// Calculate the 2D distance to another location (ignoring Z).
    pub fn distance_2d(&self, other: &Location) -> f64 {
        ((other.x - self.x).powi(2) + (other.y - self.y).powi(2)).sqrt()
    }

    /// Calculate the squared distance to another location (faster than distance).
    pub fn distance_squared(&self, other: &Location) -> f64 {
        (other.x - self.x).powi(2) + (other.y - self.y).powi(2) + (other.z - self.z).powi(2)
    }

    /// Convert to nalgebra Point3.
    pub fn to_point3(&self) -> Point3<f64> {
        Point3::new(self.x, self.y, self.z)
    }

    /// Convert to nalgebra Vector3.
    pub fn to_vector3(&self) -> Vector3<f64> {
        Vector3::new(self.x, self.y, self.z)
    }

    /// Create from nalgebra Point3.
    pub fn from_point3(point: &Point3<f64>) -> Self {
        Self::new(point.x, point.y, point.z)
    }

    /// Create from nalgebra Vector3.
    pub fn from_vector3(vector: &Vector3<f64>) -> Self {
        Self::new(vector.x, vector.y, vector.z)
    }

    /// Linear interpolation between two locations.
    pub fn lerp(&self, other: &Location, t: f64) -> Location {
        Location::new(
            self.x + t * (other.x - self.x),
            self.y + t * (other.y - self.y),
            self.z + t * (other.z - self.z),
        )
    }

    /// Check if location is approximately equal to another (within epsilon).
    pub fn approx_eq(&self, other: &Location, epsilon: f64) -> bool {
        (self.x - other.x).abs() < epsilon
            && (self.y - other.y).abs() < epsilon
            && (self.z - other.z).abs() < epsilon
    }
}

impl Default for Location {
    fn default() -> Self {
        Self::zero()
    }
}

impl From<(f64, f64, f64)> for Location {
    fn from((x, y, z): (f64, f64, f64)) -> Self {
        Self::new(x, y, z)
    }
}

impl From<Location> for (f64, f64, f64) {
    fn from(loc: Location) -> Self {
        (loc.x, loc.y, loc.z)
    }
}

impl From<[f64; 3]> for Location {
    fn from([x, y, z]: [f64; 3]) -> Self {
        Self::new(x, y, z)
    }
}

impl From<Location> for [f64; 3] {
    fn from(loc: Location) -> Self {
        [loc.x, loc.y, loc.z]
    }
}

impl From<Point3<f64>> for Location {
    fn from(point: Point3<f64>) -> Self {
        Self::from_point3(&point)
    }
}

impl From<Location> for Point3<f64> {
    fn from(loc: Location) -> Self {
        loc.to_point3()
    }
}

impl From<Vector3<f64>> for Location {
    fn from(vector: Vector3<f64>) -> Self {
        Self::from_vector3(&vector)
    }
}

impl From<Location> for Vector3<f64> {
    fn from(loc: Location) -> Self {
        loc.to_vector3()
    }
}

// Arithmetic operations
impl Add for Location {
    type Output = Location;

    fn add(self, rhs: Location) -> Self::Output {
        Location::new(self.x + rhs.x, self.y + rhs.y, self.z + rhs.z)
    }
}

impl Sub for Location {
    type Output = Location;

    fn sub(self, rhs: Location) -> Self::Output {
        Location::new(self.x - rhs.x, self.y - rhs.y, self.z - rhs.z)
    }
}

impl Mul<f64> for Location {
    type Output = Location;

    fn mul(self, rhs: f64) -> Self::Output {
        Location::new(self.x * rhs, self.y * rhs, self.z * rhs)
    }
}

impl Div<f64> for Location {
    type Output = Location;

    fn div(self, rhs: f64) -> Self::Output {
        Location::new(self.x / rhs, self.y / rhs, self.z / rhs)
    }
}

impl AddAssign for Location {
    fn add_assign(&mut self, rhs: Location) {
        self.x += rhs.x;
        self.y += rhs.y;
        self.z += rhs.z;
    }
}

impl SubAssign for Location {
    fn sub_assign(&mut self, rhs: Location) {
        self.x -= rhs.x;
        self.y -= rhs.y;
        self.z -= rhs.z;
    }
}

impl MulAssign<f64> for Location {
    fn mul_assign(&mut self, rhs: f64) {
        self.x *= rhs;
        self.y *= rhs;
        self.z *= rhs;
    }
}

impl DivAssign<f64> for Location {
    fn div_assign(&mut self, rhs: f64) {
        self.x /= rhs;
        self.y /= rhs;
        self.z /= rhs;
    }
}

impl std::fmt::Display for Location {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "Location(x={:.3}, y={:.3}, z={:.3})",
            self.x, self.y, self.z
        )
    }
}

// Conversion to/from carla-cxx types
impl FromCxx<carla_cxx::SimpleLocation> for Location {
    fn from_cxx(value: carla_cxx::SimpleLocation) -> Self {
        Self::new(value.x, value.y, value.z)
    }
}

impl ToCxx<carla_cxx::SimpleLocation> for Location {
    fn to_cxx(&self) -> carla_cxx::SimpleLocation {
        carla_cxx::SimpleLocation {
            x: self.x,
            y: self.y,
            z: self.z,
        }
    }
}
