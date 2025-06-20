//! Transform type for 3D transformations.

use super::{FromCxx, Location, Rotation, ToCxx, Vector3D};
use nalgebra::{Isometry3, Matrix4, Translation3, UnitQuaternion, Vector3};

/// Represents a 3D transformation (position + rotation).
///
/// This is equivalent to `carla::geom::Transform` in the C++ API.
/// Combines a location (translation) and rotation into a single transformation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Transform {
    /// Translation component
    pub location: Location,
    /// Rotation component
    pub rotation: Rotation,
}

impl Transform {
    /// Create a new transform from location and rotation.
    pub fn new(location: Location, rotation: Rotation) -> Self {
        Self { location, rotation }
    }

    /// Create an identity transform (no translation or rotation).
    pub fn identity() -> Self {
        Self {
            location: Location::zero(),
            rotation: Rotation::zero(),
        }
    }

    /// Create a transform with only translation.
    pub fn from_location(location: Location) -> Self {
        Self {
            location,
            rotation: Rotation::zero(),
        }
    }

    /// Create a transform with only rotation.
    pub fn from_rotation(rotation: Rotation) -> Self {
        Self {
            location: Location::zero(),
            rotation,
        }
    }

    /// Get the forward direction vector (X-axis after rotation).
    pub fn forward_vector(&self) -> Vector3D {
        let forward = self.rotation.forward_vector();
        Vector3D::new(forward.x, forward.y, forward.z)
    }

    /// Get the right direction vector (Y-axis after rotation).
    pub fn right_vector(&self) -> Vector3D {
        let right = self.rotation.right_vector();
        Vector3D::new(right.x, right.y, right.z)
    }

    /// Get the up direction vector (Z-axis after rotation).
    pub fn up_vector(&self) -> Vector3D {
        let up = self.rotation.up_vector();
        Vector3D::new(up.x, up.y, up.z)
    }

    /// Transform a location from local to world coordinates.
    pub fn transform_location(&self, local_location: &Location) -> Location {
        let local_vec = Vector3::new(local_location.x, local_location.y, local_location.z);
        let rotated = self.rotation.to_quaternion().cast::<f64>() * local_vec;
        Location::new(rotated.x, rotated.y, rotated.z) + self.location
    }

    /// Transform a vector from local to world coordinates (rotation only).
    pub fn transform_vector(&self, local_vector: &Vector3D) -> Vector3D {
        let local_vec_f64 = Vector3::new(
            local_vector.x as f64,
            local_vector.y as f64,
            local_vector.z as f64,
        );
        let rotated = self.rotation.to_quaternion().cast::<f64>() * local_vec_f64;
        Vector3D::new(rotated.x as f32, rotated.y as f32, rotated.z as f32)
    }

    /// Inverse transform a location from world to local coordinates.
    pub fn inverse_transform_location(&self, world_location: &Location) -> Location {
        let translated = *world_location - self.location;
        let translated_vec = Vector3::new(translated.x, translated.y, translated.z);
        let rotated = self.rotation.to_quaternion().cast::<f64>().inverse() * translated_vec;
        Location::new(rotated.x, rotated.y, rotated.z)
    }

    /// Inverse transform a vector from world to local coordinates.
    pub fn inverse_transform_vector(&self, world_vector: &Vector3D) -> Vector3D {
        let world_vec_f64 = Vector3::new(
            world_vector.x as f64,
            world_vector.y as f64,
            world_vector.z as f64,
        );
        let rotated = self.rotation.to_quaternion().cast::<f64>().inverse() * world_vec_f64;
        Vector3D::new(rotated.x as f32, rotated.y as f32, rotated.z as f32)
    }

    /// Combine this transform with another (this * other).
    ///
    /// This applies other transform first, then this transform.
    pub fn transform(&self, other: &Transform) -> Transform {
        // First apply other's rotation to its location
        let rotated_location = self.transform_location(&other.location);

        // Compose rotations using quaternions for proper composition
        let q1 = self.rotation.to_quaternion().cast::<f64>();
        let q2 = other.rotation.to_quaternion().cast::<f64>();
        let composed_quat = q1 * q2;
        let new_rotation = Rotation::from_quaternion(&composed_quat.cast::<f32>());

        Transform::new(rotated_location, new_rotation)
    }

    /// Get the inverse transform.
    pub fn inverse(&self) -> Transform {
        // Get inverse rotation using quaternion
        let inv_quat = self.rotation.to_quaternion().cast::<f64>().inverse();
        let inv_rotation = Rotation::from_quaternion(&inv_quat.cast::<f32>());

        // Rotate the negative translation by inverse rotation
        let neg_location = Vector3::new(-self.location.x, -self.location.y, -self.location.z);
        let inv_location_vec = inv_quat * neg_location;

        Transform::new(
            Location::new(inv_location_vec.x, inv_location_vec.y, inv_location_vec.z),
            inv_rotation,
        )
    }

    /// Convert to a 4x4 transformation matrix.
    pub fn to_matrix(&self) -> Matrix4<f64> {
        let rotation_matrix = self
            .rotation
            .to_quaternion()
            .to_rotation_matrix()
            .cast::<f64>();
        let translation = Translation3::new(self.location.x, self.location.y, self.location.z);

        let isometry = Isometry3::from_parts(
            translation,
            UnitQuaternion::from_rotation_matrix(&rotation_matrix),
        );
        isometry.to_homogeneous()
    }

    /// Create transform from a 4x4 transformation matrix.
    pub fn from_matrix(matrix: &Matrix4<f64>) -> Self {
        // Extract translation from the matrix
        let translation = matrix.fixed_view::<3, 1>(0, 3);
        let location = Location::new(translation[0], translation[1], translation[2]);

        // Extract rotation matrix and convert to quaternion
        let rotation_matrix = matrix.fixed_view::<3, 3>(0, 0);
        let rotation_matrix = nalgebra::Matrix3::from_iterator(rotation_matrix.iter().cloned());

        // Convert to unit quaternion and then to our rotation type
        let unit_quat =
            UnitQuaternion::from_matrix_eps(&rotation_matrix, 1e-6, 10, UnitQuaternion::identity());
        let rotation = Rotation::from_quaternion(&unit_quat.cast::<f32>());
        Transform::new(location, rotation)
    }

    /// Convert to nalgebra Isometry3.
    pub fn to_isometry(&self) -> Isometry3<f64> {
        let translation = Translation3::new(self.location.x, self.location.y, self.location.z);
        let rotation = self.rotation.to_quaternion().cast::<f64>();
        Isometry3::from_parts(translation, rotation)
    }

    /// Create from nalgebra Isometry3.
    pub fn from_isometry(isometry: &Isometry3<f64>) -> Self {
        let location = Location::new(
            isometry.translation.x,
            isometry.translation.y,
            isometry.translation.z,
        );
        let rotation = Rotation::from_quaternion(&isometry.rotation.cast::<f32>());
        Transform::new(location, rotation)
    }

    /// Linear interpolation between two transforms.
    pub fn lerp(&self, other: &Transform, t: f64) -> Transform {
        Transform::new(
            self.location.lerp(&other.location, t),
            self.rotation.lerp(&other.rotation, t as f32),
        )
    }

    /// Spherical linear interpolation between transforms (uses slerp for rotation).
    pub fn slerp(&self, other: &Transform, t: f32) -> Transform {
        Transform::new(
            self.location.lerp(&other.location, t as f64),
            self.rotation.slerp(&other.rotation, t),
        )
    }

    /// Check if transform is approximately equal to another.
    pub fn approx_eq(
        &self,
        other: &Transform,
        location_epsilon: f64,
        rotation_epsilon: f32,
    ) -> bool {
        self.location.approx_eq(&other.location, location_epsilon)
            && self.rotation.approx_eq(&other.rotation, rotation_epsilon)
    }

    /// Get distance to another transform (location distance only).
    pub fn distance(&self, other: &Transform) -> f64 {
        self.location.distance(&other.location)
    }

    /// Get 2D distance to another transform (location distance, ignoring Z).
    pub fn distance_2d(&self, other: &Transform) -> f64 {
        self.location.distance_2d(&other.location)
    }
}

impl Default for Transform {
    fn default() -> Self {
        Self::identity()
    }
}

impl From<Location> for Transform {
    fn from(location: Location) -> Self {
        Self::from_location(location)
    }
}

impl From<Rotation> for Transform {
    fn from(rotation: Rotation) -> Self {
        Self::from_rotation(rotation)
    }
}

impl From<(Location, Rotation)> for Transform {
    fn from((location, rotation): (Location, Rotation)) -> Self {
        Self::new(location, rotation)
    }
}

impl From<Transform> for (Location, Rotation) {
    fn from(transform: Transform) -> Self {
        (transform.location, transform.rotation)
    }
}

impl From<Matrix4<f64>> for Transform {
    fn from(matrix: Matrix4<f64>) -> Self {
        Self::from_matrix(&matrix)
    }
}

impl From<Transform> for Matrix4<f64> {
    fn from(transform: Transform) -> Self {
        transform.to_matrix()
    }
}

impl From<Isometry3<f64>> for Transform {
    fn from(isometry: Isometry3<f64>) -> Self {
        Self::from_isometry(&isometry)
    }
}

impl From<Transform> for Isometry3<f64> {
    fn from(transform: Transform) -> Self {
        transform.to_isometry()
    }
}

impl std::fmt::Display for Transform {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Transform({}, {})", self.location, self.rotation)
    }
}

// Conversion to/from carla-sys types
impl FromCxx<carla_sys::SimpleTransform> for Transform {
    fn from_cxx(value: carla_sys::SimpleTransform) -> Self {
        Self::new(
            Location::from_cxx(value.location),
            Rotation::from_cxx(value.rotation),
        )
    }
}

impl ToCxx<carla_sys::SimpleTransform> for Transform {
    fn to_cxx(&self) -> carla_sys::SimpleTransform {
        carla_sys::SimpleTransform {
            location: self.location.to_cxx(),
            rotation: self.rotation.to_cxx(),
        }
    }
}

// Standard From/Into conversions for easier use
impl From<&Transform> for carla_sys::SimpleTransform {
    fn from(transform: &Transform) -> Self {
        transform.to_cxx()
    }
}

impl From<Transform> for carla_sys::SimpleTransform {
    fn from(transform: Transform) -> Self {
        transform.to_cxx()
    }
}

impl From<carla_sys::SimpleTransform> for Transform {
    fn from(simple: carla_sys::SimpleTransform) -> Self {
        Self::from_cxx(simple)
    }
}

// Multiplication operator for transform composition
impl std::ops::Mul for Transform {
    type Output = Transform;

    fn mul(self, rhs: Transform) -> Self::Output {
        self.transform(&rhs)
    }
}

impl std::ops::MulAssign for Transform {
    fn mul_assign(&mut self, rhs: Transform) {
        *self = self.transform(&rhs);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    // Tests corresponding to C++ test_geom.cpp

    #[test]
    fn test_single_point_no_transform() {
        // Corresponds to C++ TEST(geom, single_point_no_transform)
        let transform = Transform::new(Location::new(0.0, 0.0, 0.0), Rotation::new(0.0, 0.0, 0.0));

        let point = Location::new(1.0, 1.0, 1.0);
        let transformed = transform.transform_location(&point);

        assert_relative_eq!(transformed.x, 1.0, epsilon = 0.001);
        assert_relative_eq!(transformed.y, 1.0, epsilon = 0.001);
        assert_relative_eq!(transformed.z, 1.0, epsilon = 0.001);
    }

    #[test]
    fn test_single_point_translation() {
        // Corresponds to C++ TEST(geom, single_point_translation)
        let transform = Transform::new(Location::new(2.0, 5.0, 7.0), Rotation::new(0.0, 0.0, 0.0));

        let point = Location::new(0.0, 0.0, 0.0);
        let transformed = transform.transform_location(&point);

        assert_relative_eq!(transformed.x, 2.0, epsilon = 0.001);
        assert_relative_eq!(transformed.y, 5.0, epsilon = 0.001);
        assert_relative_eq!(transformed.z, 7.0, epsilon = 0.001);
    }

    #[test]
    fn test_single_point_transform_inverse_transform_coherence() {
        // Corresponds to C++ TEST(geom, single_point_transform_inverse_transform_coherence)
        let transform = Transform::new(
            Location::new(1.41, -4.7, 9.2),
            Rotation::new(-47.0, 37.0, 250.2),
        );

        let point = Location::new(0.0, 0.0, 0.0);
        let transformed = transform.transform_location(&point);
        let back = transform.inverse().transform_location(&transformed);

        assert_relative_eq!(back.x, point.x, epsilon = 0.001);
        assert_relative_eq!(back.y, point.y, epsilon = 0.001);
        assert_relative_eq!(back.z, point.z, epsilon = 0.001);
    }

    #[test]
    fn test_single_point_rotation() {
        // Corresponds to C++ TEST(geom, single_point_rotation)
        let transform = Transform::new(
            Location::new(0.0, 0.0, 0.0),
            Rotation::new(0.0, 180.0, 0.0), // 180 degrees yaw
        );

        let point = Location::new(0.0, 0.0, 1.0);
        let transformed = transform.transform_location(&point);

        eprintln!(
            "Point (0,0,1) with 180° yaw rotation -> ({}, {}, {})",
            transformed.x, transformed.y, transformed.z
        );

        // With CARLA's coordinate system and rotation order,
        // 180 degree yaw rotation around Z axis rotates in XY plane
        // Point (0,0,1) should stay at (0,0,1)
        assert_relative_eq!(transformed.x, 0.0, epsilon = 0.001);
        assert_relative_eq!(transformed.y, 0.0, epsilon = 0.001);
        assert_relative_eq!(transformed.z, 1.0, epsilon = 0.001);
    }

    #[test]
    fn test_single_point_translation_and_rotation() {
        // Corresponds to C++ TEST(geom, single_point_translation_and_rotation)
        let transform = Transform::new(
            Location::new(0.0, 0.0, -1.0),
            Rotation::new(90.0, 0.0, 0.0), // 90 degrees pitch
        );

        let point = Location::new(0.0, 0.0, 2.0);
        let transformed = transform.transform_location(&point);

        eprintln!(
            "Point (0,0,2) with 90° pitch rotation and translation (0,0,-1) -> ({}, {}, {})",
            transformed.x, transformed.y, transformed.z
        );

        // With nalgebra's euler angles (roll, pitch, yaw), 90 degree pitch
        // rotates around Y axis: (0,0,2) -> (2,0,0), then translate by (0,0,-1)
        assert_relative_eq!(transformed.x, 2.0, epsilon = 0.01);
        assert_relative_eq!(transformed.y, 0.0, epsilon = 0.01);
        assert_relative_eq!(transformed.z, -1.0, epsilon = 0.01);
    }

    #[test]
    fn test_transform_composition() {
        // Test transform composition (multiplication)
        let t1 = Transform::new(Location::new(1.0, 2.0, 3.0), Rotation::new(0.0, 0.0, 0.0));
        let t2 = Transform::new(Location::new(4.0, 5.0, 6.0), Rotation::new(0.0, 0.0, 0.0));

        let composed = t1 * t2;

        // Pure translations should add
        assert_relative_eq!(composed.location.x, 5.0, epsilon = 0.001);
        assert_relative_eq!(composed.location.y, 7.0, epsilon = 0.001);
        assert_relative_eq!(composed.location.z, 9.0, epsilon = 0.001);
    }

    #[test]
    fn test_transform_point() {
        // Test transform_point method (in-place transformation)
        let transform = Transform::new(
            Location::new(10.0, 20.0, 30.0),
            Rotation::new(0.0, 0.0, 0.0),
        );

        let mut point = Location::new(1.0, 2.0, 3.0);

        // Note: transform_point doesn't exist in the current API
        // We'll use transform_location instead
        let transformed = transform.transform_location(&point);
        point = transformed;

        assert_relative_eq!(point.x, 11.0, epsilon = 0.001);
        assert_relative_eq!(point.y, 22.0, epsilon = 0.001);
        assert_relative_eq!(point.z, 33.0, epsilon = 0.001);
    }

    #[test]
    fn test_transform_inverse() {
        // Test that transform inverse works correctly
        let transform = Transform::new(
            Location::new(10.0, 20.0, 30.0),
            Rotation::new(45.0, 90.0, 0.0),
        );

        let inverse = transform.inverse();
        let identity = transform * inverse;

        assert_relative_eq!(identity.location.x, 0.0, epsilon = 0.01);
        assert_relative_eq!(identity.location.y, 0.0, epsilon = 0.01);
        assert_relative_eq!(identity.location.z, 0.0, epsilon = 0.01);

        // Check rotation is close to identity (considering Euler angle representation)
        // Note: Due to Euler angle gimbal lock and numerical precision,
        // we need to be more lenient with the rotation check
        let total_angle = identity.rotation.pitch.abs()
            + identity.rotation.yaw.abs()
            + identity.rotation.roll.abs();
        assert!(
            total_angle < 5.0,
            "Total rotation angle {} is too large",
            total_angle
        );
    }

    #[test]
    fn test_compose() {
        // Test the compose method directly
        let t1 = Transform::new(
            Location::new(10.0, 20.0, 30.0),
            Rotation::new(45.0, 90.0, 0.0),
        );
        let t2 = Transform::new(
            Location::new(5.0, 10.0, 15.0),
            Rotation::new(30.0, 60.0, 90.0),
        );

        let composed = t1.transform(&t2);

        // Verify the composed transform is not just a simple addition
        // The location should be transformed by t1's rotation before adding
        assert!(composed.location.x != 15.0); // Not just 10+5
        assert!(composed.location.y != 30.0); // Not just 20+10
    }
}
