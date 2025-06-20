//! Rotation type for 3D rotations.

use super::{FromCxx, ToCxx};
use nalgebra::{UnitQuaternion, Vector3};
use std::ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Sub, SubAssign};

/// Represents a 3D rotation using Euler angles.
///
/// This is equivalent to `carla::geom::Rotation` in the C++ API.
/// All angles are in degrees, using CARLA's coordinate system:
/// - Pitch: rotation around Y-axis (up/down)
/// - Yaw: rotation around Z-axis (left/right)  
/// - Roll: rotation around X-axis (banking)
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct Rotation {
    /// Pitch angle in degrees (rotation around Y-axis)
    pub pitch: f32,
    /// Yaw angle in degrees (rotation around Z-axis)
    pub yaw: f32,
    /// Roll angle in degrees (rotation around X-axis)
    pub roll: f32,
}

impl Rotation {
    /// Create a new rotation from Euler angles in degrees.
    pub fn new(pitch: f32, yaw: f32, roll: f32) -> Self {
        Self { pitch, yaw, roll }
    }

    /// Create a rotation with no rotation (identity).
    pub fn zero() -> Self {
        Self::new(0.0, 0.0, 0.0)
    }

    /// Create a rotation from pitch and yaw only (roll = 0).
    pub fn from_pitch_yaw(pitch: f32, yaw: f32) -> Self {
        Self::new(pitch, yaw, 0.0)
    }

    /// Create a rotation from yaw only (pitch = 0, roll = 0).
    pub fn from_yaw(yaw: f32) -> Self {
        Self::new(0.0, yaw, 0.0)
    }

    /// Convert angles to radians.
    pub fn to_radians(&self) -> (f32, f32, f32) {
        (
            self.pitch.to_radians(),
            self.yaw.to_radians(),
            self.roll.to_radians(),
        )
    }

    /// Create rotation from angles in radians.
    pub fn from_radians(pitch: f32, yaw: f32, roll: f32) -> Self {
        Self::new(pitch.to_degrees(), yaw.to_degrees(), roll.to_degrees())
    }

    /// Convert to nalgebra UnitQuaternion.
    pub fn to_quaternion(&self) -> UnitQuaternion<f32> {
        let (pitch, yaw, roll) = self.to_radians();
        UnitQuaternion::from_euler_angles(roll, pitch, yaw)
    }

    /// Create from nalgebra UnitQuaternion.
    pub fn from_quaternion(quaternion: &UnitQuaternion<f32>) -> Self {
        let (roll, pitch, yaw) = quaternion.euler_angles();
        Self::from_radians(pitch, yaw, roll)
    }

    /// Get the forward direction vector (X-axis after rotation).
    pub fn forward_vector(&self) -> Vector3<f32> {
        let (pitch, yaw, _) = self.to_radians();
        Vector3::new(
            pitch.cos() * yaw.cos(),
            pitch.cos() * yaw.sin(),
            pitch.sin(),
        )
    }

    /// Get the right direction vector (Y-axis after rotation).
    pub fn right_vector(&self) -> Vector3<f32> {
        let (_, yaw, roll) = self.to_radians();
        Vector3::new(-yaw.sin() * roll.cos(), yaw.cos() * roll.cos(), roll.sin())
    }

    /// Get the up direction vector (Z-axis after rotation).
    pub fn up_vector(&self) -> Vector3<f32> {
        let (pitch, yaw, roll) = self.to_radians();
        Vector3::new(
            -pitch.sin() * yaw.cos() * roll.cos() + yaw.sin() * roll.sin(),
            -pitch.sin() * yaw.sin() * roll.cos() - yaw.cos() * roll.sin(),
            pitch.cos() * roll.cos(),
        )
    }

    /// Normalize angles to [-180, 180] degrees.
    pub fn normalize(&mut self) {
        self.pitch = super::utils::clamp_angle(self.pitch as f64) as f32;
        self.yaw = super::utils::clamp_angle(self.yaw as f64) as f32;
        self.roll = super::utils::clamp_angle(self.roll as f64) as f32;
    }

    /// Return normalized copy of this rotation.
    pub fn normalized(&self) -> Self {
        let mut result = *self;
        result.normalize();
        result
    }

    /// Linear interpolation between two rotations.
    ///
    /// Note: This performs linear interpolation on Euler angles, which may not
    /// be the most natural rotation interpolation. Consider using quaternions
    /// for smoother rotation interpolation.
    pub fn lerp(&self, other: &Rotation, t: f32) -> Rotation {
        Rotation::new(
            self.pitch + t * (other.pitch - self.pitch),
            self.yaw + t * (other.yaw - self.yaw),
            self.roll + t * (other.roll - self.roll),
        )
    }

    /// Spherical linear interpolation between rotations using quaternions.
    pub fn slerp(&self, other: &Rotation, t: f32) -> Rotation {
        let q1 = self.to_quaternion();
        let q2 = other.to_quaternion();
        let interpolated = q1.slerp(&q2, t);
        Self::from_quaternion(&interpolated)
    }

    /// Check if rotation is approximately equal to another (within epsilon degrees).
    pub fn approx_eq(&self, other: &Rotation, epsilon: f32) -> bool {
        (self.pitch - other.pitch).abs() < epsilon
            && (self.yaw - other.yaw).abs() < epsilon
            && (self.roll - other.roll).abs() < epsilon
    }

    /// Invert the rotation.
    pub fn inverse(&self) -> Self {
        Self::new(-self.pitch, -self.yaw, -self.roll)
    }
}

impl Default for Rotation {
    fn default() -> Self {
        Self::zero()
    }
}

impl From<(f32, f32, f32)> for Rotation {
    fn from((pitch, yaw, roll): (f32, f32, f32)) -> Self {
        Self::new(pitch, yaw, roll)
    }
}

impl From<Rotation> for (f32, f32, f32) {
    fn from(rot: Rotation) -> Self {
        (rot.pitch, rot.yaw, rot.roll)
    }
}

impl From<[f32; 3]> for Rotation {
    fn from([pitch, yaw, roll]: [f32; 3]) -> Self {
        Self::new(pitch, yaw, roll)
    }
}

impl From<Rotation> for [f32; 3] {
    fn from(rot: Rotation) -> Self {
        [rot.pitch, rot.yaw, rot.roll]
    }
}

impl From<UnitQuaternion<f32>> for Rotation {
    fn from(quaternion: UnitQuaternion<f32>) -> Self {
        Self::from_quaternion(&quaternion)
    }
}

impl From<Rotation> for UnitQuaternion<f32> {
    fn from(rot: Rotation) -> Self {
        rot.to_quaternion()
    }
}

// Arithmetic operations
impl Add for Rotation {
    type Output = Rotation;

    fn add(self, rhs: Rotation) -> Self::Output {
        Rotation::new(
            self.pitch + rhs.pitch,
            self.yaw + rhs.yaw,
            self.roll + rhs.roll,
        )
    }
}

impl Sub for Rotation {
    type Output = Rotation;

    fn sub(self, rhs: Rotation) -> Self::Output {
        Rotation::new(
            self.pitch - rhs.pitch,
            self.yaw - rhs.yaw,
            self.roll - rhs.roll,
        )
    }
}

impl Mul<f32> for Rotation {
    type Output = Rotation;

    fn mul(self, rhs: f32) -> Self::Output {
        Rotation::new(self.pitch * rhs, self.yaw * rhs, self.roll * rhs)
    }
}

impl Div<f32> for Rotation {
    type Output = Rotation;

    fn div(self, rhs: f32) -> Self::Output {
        Rotation::new(self.pitch / rhs, self.yaw / rhs, self.roll / rhs)
    }
}

impl AddAssign for Rotation {
    fn add_assign(&mut self, rhs: Rotation) {
        self.pitch += rhs.pitch;
        self.yaw += rhs.yaw;
        self.roll += rhs.roll;
    }
}

impl SubAssign for Rotation {
    fn sub_assign(&mut self, rhs: Rotation) {
        self.pitch -= rhs.pitch;
        self.yaw -= rhs.yaw;
        self.roll -= rhs.roll;
    }
}

impl MulAssign<f32> for Rotation {
    fn mul_assign(&mut self, rhs: f32) {
        self.pitch *= rhs;
        self.yaw *= rhs;
        self.roll *= rhs;
    }
}

impl DivAssign<f32> for Rotation {
    fn div_assign(&mut self, rhs: f32) {
        self.pitch /= rhs;
        self.yaw /= rhs;
        self.roll /= rhs;
    }
}

impl std::fmt::Display for Rotation {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "Rotation(pitch={:.1}°, yaw={:.1}°, roll={:.1}°)",
            self.pitch, self.yaw, self.roll
        )
    }
}

// Conversion to/from carla-sys types
impl FromCxx<carla_sys::SimpleRotation> for Rotation {
    fn from_cxx(value: carla_sys::SimpleRotation) -> Self {
        Self::new(value.pitch as f32, value.yaw as f32, value.roll as f32)
    }
}

impl ToCxx<carla_sys::SimpleRotation> for Rotation {
    fn to_cxx(&self) -> carla_sys::SimpleRotation {
        carla_sys::SimpleRotation {
            pitch: self.pitch as f64,
            yaw: self.yaw as f64,
            roll: self.roll as f64,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    // Tests corresponding to C++ test_geom.cpp

    #[test]
    fn test_forward_vector() {
        // Corresponds to C++ TEST(geom, forward_vector)

        // Test case 1: Identity rotation (0,0,0) -> forward is (1,0,0)
        let rotation = Rotation::new(0.0, 0.0, 0.0);
        let forward = rotation.forward_vector();
        assert_relative_eq!(forward.x, 1.0, epsilon = 0.001);
        assert_relative_eq!(forward.y, 0.0, epsilon = 0.001);
        assert_relative_eq!(forward.z, 0.0, epsilon = 0.001);

        // Test case 2: 90 degree yaw -> forward is (0,1,0)
        let rotation = Rotation::new(0.0, 90.0, 0.0);
        let forward = rotation.forward_vector();
        assert_relative_eq!(forward.x, 0.0, epsilon = 0.001);
        assert_relative_eq!(forward.y, 1.0, epsilon = 0.001);
        assert_relative_eq!(forward.z, 0.0, epsilon = 0.001);

        // Test case 3: -90 degree yaw -> forward is (0,-1,0)
        let rotation = Rotation::new(0.0, -90.0, 0.0);
        let forward = rotation.forward_vector();
        assert_relative_eq!(forward.x, 0.0, epsilon = 0.001);
        assert_relative_eq!(forward.y, -1.0, epsilon = 0.001);
        assert_relative_eq!(forward.z, 0.0, epsilon = 0.001);

        // Test case 4: 90 degree pitch -> forward is (0,0,1)
        let rotation = Rotation::new(90.0, 0.0, 0.0);
        let forward = rotation.forward_vector();
        assert_relative_eq!(forward.x, 0.0, epsilon = 0.001);
        assert_relative_eq!(forward.y, 0.0, epsilon = 0.001);
        assert_relative_eq!(forward.z, 1.0, epsilon = 0.001);

        // Test case 5: 180 degree yaw -> forward is (-1,0,0)
        let rotation = Rotation::new(0.0, 180.0, 0.0);
        let forward = rotation.forward_vector();
        assert_relative_eq!(forward.x, -1.0, epsilon = 0.001);
        assert_relative_eq!(forward.y, 0.0, epsilon = 0.001);
        assert_relative_eq!(forward.z, 0.0, epsilon = 0.001);

        // Test case 6: 45 degree pitch -> forward is (sqrt(2)/2, 0, sqrt(2)/2)
        let rotation = Rotation::new(45.0, 0.0, 0.0);
        let forward = rotation.forward_vector();
        let sqrt2_2 = std::f32::consts::SQRT_2 / 2.0;
        assert_relative_eq!(forward.x, sqrt2_2, epsilon = 0.001);
        assert_relative_eq!(forward.y, 0.0, epsilon = 0.001);
        assert_relative_eq!(forward.z, sqrt2_2, epsilon = 0.001);

        // Test case 7: 45 degree yaw -> forward is (sqrt(2)/2, sqrt(2)/2, 0)
        let rotation = Rotation::new(0.0, 45.0, 0.0);
        let forward = rotation.forward_vector();
        assert_relative_eq!(forward.x, sqrt2_2, epsilon = 0.001);
        assert_relative_eq!(forward.y, sqrt2_2, epsilon = 0.001);
        assert_relative_eq!(forward.z, 0.0, epsilon = 0.001);

        // Test case 8: Combined rotation (pitch=30, yaw=60)
        let rotation = Rotation::new(30.0, 60.0, 0.0);
        let forward = rotation.forward_vector();
        // Expected values based on the formula:
        // x = cos(pitch) * cos(yaw) = cos(30°) * cos(60°) = 0.866 * 0.5 = 0.433
        // y = cos(pitch) * sin(yaw) = cos(30°) * sin(60°) = 0.866 * 0.866 = 0.75
        // z = sin(pitch) = sin(30°) = 0.5
        assert_relative_eq!(forward.x, 0.433, epsilon = 0.01);
        assert_relative_eq!(forward.y, 0.75, epsilon = 0.01);
        assert_relative_eq!(forward.z, 0.5, epsilon = 0.01);
    }

    #[test]
    fn test_rotation_normalization() {
        // Test that angles are properly normalized to [-180, 180] range
        let mut rotation = Rotation::new(270.0, 370.0, -200.0);
        rotation.normalize();

        assert_relative_eq!(rotation.pitch, -90.0, epsilon = 0.001);
        assert_relative_eq!(rotation.yaw, 10.0, epsilon = 0.001);
        assert_relative_eq!(rotation.roll, 160.0, epsilon = 0.001);
    }

    #[test]
    fn test_rotation_inverse() {
        // Test rotation inverse
        let rotation = Rotation::new(30.0, 45.0, 60.0);
        let inverse = rotation.inverse();

        assert_relative_eq!(inverse.pitch, -30.0, epsilon = 0.001);
        assert_relative_eq!(inverse.yaw, -45.0, epsilon = 0.001);
        assert_relative_eq!(inverse.roll, -60.0, epsilon = 0.001);
    }
}
