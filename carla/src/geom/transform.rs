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
    pub fn get_forward_vector(&self) -> Vector3D {
        let forward = self.rotation.get_forward_vector();
        Vector3D::new(forward.x, forward.y, forward.z)
    }

    /// Get the right direction vector (Y-axis after rotation).
    pub fn get_right_vector(&self) -> Vector3D {
        let right = self.rotation.get_right_vector();
        Vector3D::new(right.x, right.y, right.z)
    }

    /// Get the up direction vector (Z-axis after rotation).
    pub fn get_up_vector(&self) -> Vector3D {
        let up = self.rotation.get_up_vector();
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
    /// This applies this transform first, then the other transform.
    pub fn transform(&self, other: &Transform) -> Transform {
        let new_location = self.transform_location(&other.location);
        let new_rotation = self.rotation + other.rotation; // Note: Euler angle composition
        Transform::new(new_location, new_rotation)
    }

    /// Get the inverse transform.
    pub fn inverse(&self) -> Transform {
        let inv_rotation = self.rotation.inverse();
        let neg_location = Vector3::new(-self.location.x, -self.location.y, -self.location.z);
        let inv_location_vec = inv_rotation.to_quaternion().cast::<f64>() * neg_location;
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

// Conversion to/from carla-cxx types
impl FromCxx<carla_cxx::SimpleTransform> for Transform {
    fn from_cxx(value: carla_cxx::SimpleTransform) -> Self {
        Self::new(
            Location::from_cxx(value.location),
            Rotation::from_cxx(value.rotation),
        )
    }
}

impl ToCxx<carla_cxx::SimpleTransform> for Transform {
    fn to_cxx(&self) -> carla_cxx::SimpleTransform {
        carla_cxx::SimpleTransform {
            location: self.location.to_cxx(),
            rotation: self.rotation.to_cxx(),
        }
    }
}

// Standard From/Into conversions for easier use
impl From<&Transform> for carla_cxx::SimpleTransform {
    fn from(transform: &Transform) -> Self {
        transform.to_cxx()
    }
}

impl From<Transform> for carla_cxx::SimpleTransform {
    fn from(transform: Transform) -> Self {
        transform.to_cxx()
    }
}

impl From<carla_cxx::SimpleTransform> for Transform {
    fn from(simple: carla_cxx::SimpleTransform) -> Self {
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
