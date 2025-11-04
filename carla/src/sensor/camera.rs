//! Camera projection and coordinate transformation utilities.
//!
//! This module provides utilities for:
//! - Building camera projection matrices (pinhole camera model)
//! - Transforming 3D world coordinates to camera space
//! - Projecting 3D points to 2D image coordinates
//!
//! These utilities are essential for sensor fusion tasks like projecting
//! LiDAR points onto camera images.

use crate::geom::{Location, Transform, Vector3D};
use nalgebra::Matrix3;

/// Builds a camera projection matrix (intrinsic matrix) using the pinhole camera model.
///
/// The projection matrix K transforms 3D camera coordinates to 2D image coordinates:
/// ```text
/// K = [[f_x,   0,  c_x],
///      [  0, f_y,  c_y],
///      [  0,   0,    1]]
/// ```
///
/// Where:
/// - `f_x`, `f_y`: Focal lengths in pixels (equal for square pixels)
/// - `c_x`, `c_y`: Principal point (image center)
///
/// # Arguments
///
/// * `width` - Image width in pixels
/// * `height` - Image height in pixels
/// * `fov` - Horizontal field of view in degrees
///
/// # Returns
///
/// A 3x3 projection matrix
///
/// # Examples
///
/// ```
/// use carla::sensor::camera;
///
/// // Build projection matrix for 800x600 image with 90° FOV
/// let k = camera::build_projection_matrix(800, 600, 90.0);
/// ```
///
/// # Reference
///
/// Based on [CARLA's lidar_to_camera.py example](https://github.com/carla-simulator/carla/blob/master/PythonAPI/examples/lidar_to_camera.py)
pub fn build_projection_matrix(width: u32, height: u32, fov: f32) -> Matrix3<f32> {
    // Calculate focal length from FOV
    // focal = width / (2.0 * tan(fov * π / 360.0))
    let focal = width as f32 / (2.0 * (fov * std::f32::consts::PI / 360.0).tan());

    // Principal point at image center
    let c_x = width as f32 / 2.0;
    let c_y = height as f32 / 2.0;

    // Build intrinsic matrix
    Matrix3::new(focal, 0.0, c_x, 0.0, focal, c_y, 0.0, 0.0, 1.0)
}

/// Transforms a 3D point from world coordinates to camera coordinates.
///
/// This function:
/// 1. Transforms the point from world space to camera space using the inverse camera transform
/// 2. Converts from Unreal Engine 4 coordinate system to standard camera coordinates
///
/// # Coordinate System Conversion
///
/// UE4 uses (x=forward, y=right, z=up) while standard cameras use (x=right, y=down, z=forward):
/// ```text
/// UE4:              Standard Camera:
/// ^ z                    . z
/// |                     /
/// |              =>    +-------> x
/// | . x                |
/// |/                   |
/// +-------> y          v y
/// ```
///
/// The conversion is: `(x, y, z) -> (y, -z, x)`
///
/// # Arguments
///
/// * `point` - 3D point in world coordinates
/// * `camera_transform` - Camera's world transform (use `camera.get_transform()`)
///
/// # Returns
///
/// 3D point in camera coordinates (x=right, y=down, z=forward/depth)
///
/// # Examples
///
/// ```no_run
/// use carla::{geom::Location, sensor::camera};
/// # use carla::client::Client;
/// # let client = Client::default();
/// # let world = client.world();
/// # let camera = world.get_actors().iter().next().unwrap();
///
/// let world_point = Location {
///     x: 10.0,
///     y: 5.0,
///     z: 2.0,
/// };
/// let camera_transform = camera.get_transform();
/// let camera_point = camera::world_to_camera(&world_point, &camera_transform);
/// ```
pub fn world_to_camera(point: &Location, camera_transform: &Transform) -> Vector3D {
    // Convert Transform to nalgebra for inverse calculation
    let na_transform = camera_transform.to_na();

    // Convert Location to homogeneous coordinates (x, y, z, 1)
    let world_point = nalgebra::Vector4::new(point.x, point.y, point.z, 1.0);

    // Get inverse camera transform (world to camera)
    let world_to_camera = na_transform.inverse();

    // Convert Isometry3 to 4x4 transformation matrix
    let world_to_camera_matrix = world_to_camera.to_homogeneous();

    // Transform to camera space
    let camera_point = world_to_camera_matrix * world_point;

    // Convert from UE4 coordinate system to standard camera coordinates
    // UE4: (x=forward, y=right, z=up)
    // Camera: (x=right, y=down, z=forward)
    // Conversion: (x, y, z) -> (y, -z, x)
    Vector3D::new(
        camera_point.y,  // right
        -camera_point.z, // down (inverted up)
        camera_point.x,  // forward (depth)
    )
}

/// Projects a 3D point in camera coordinates to 2D image coordinates.
///
/// Uses the camera projection matrix to transform 3D camera space coordinates
/// to 2D pixel coordinates. Points are normalized by their depth (z-coordinate).
///
/// # Arguments
///
/// * `point_3d` - 3D point in camera coordinates (from [`world_to_camera`])
/// * `k_matrix` - Camera projection matrix (from [`build_projection_matrix`])
///
/// # Returns
///
/// Tuple of (x, y) pixel coordinates. Note that these may be outside the image bounds
/// or behind the camera (negative depth). Caller should validate:
/// - `0 <= x < image_width`
/// - `0 <= y < image_height`
/// - `z > 0` (point in front of camera)
///
/// # Examples
///
/// ```
/// use carla::{geom::Vector3D, sensor::camera};
///
/// let k = camera::build_projection_matrix(800, 600, 90.0);
/// let camera_point = Vector3D::new(2.0, 1.0, 10.0); // x, y, z(depth)
/// let (x, y) = camera::project_to_2d(&camera_point, &k);
///
/// // Check if point is visible in image
/// if x >= 0.0 && x < 800.0 && y >= 0.0 && y < 600.0 && camera_point.z > 0.0 {
///     println!("Pixel coordinates: ({}, {})", x as u32, y as u32);
/// }
/// ```
pub fn project_to_2d(point_3d: &Vector3D, k_matrix: &Matrix3<f32>) -> (f32, f32) {
    // Convert to nalgebra for matrix multiplication
    let na_point = nalgebra::Vector3::new(point_3d.x, point_3d.y, point_3d.z);

    // Project 3D point to 2D using projection matrix
    let projected = k_matrix * na_point;

    // Normalize by depth (z-coordinate)
    let x = projected.x / projected.z;
    let y = projected.y / projected.z;

    (x, y)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_build_projection_matrix() {
        let k = build_projection_matrix(800, 600, 90.0);

        // Check principal point (image center)
        assert_relative_eq!(k[(0, 2)], 400.0, epsilon = 1e-5);
        assert_relative_eq!(k[(1, 2)], 300.0, epsilon = 1e-5);

        // Check focal lengths are equal (square pixels)
        assert_relative_eq!(k[(0, 0)], k[(1, 1)], epsilon = 1e-5);

        // Check bottom-right is 1
        assert_relative_eq!(k[(2, 2)], 1.0, epsilon = 1e-5);

        // Check off-diagonal elements are 0
        assert_relative_eq!(k[(0, 1)], 0.0, epsilon = 1e-5);
        assert_relative_eq!(k[(1, 0)], 0.0, epsilon = 1e-5);
        assert_relative_eq!(k[(2, 0)], 0.0, epsilon = 1e-5);
        assert_relative_eq!(k[(2, 1)], 0.0, epsilon = 1e-5);
    }

    #[test]
    fn test_project_to_2d_center() {
        let k = build_projection_matrix(800, 600, 90.0);

        // Point straight ahead should project to image center
        let point = Vector3D::new(0.0, 0.0, 10.0); // at camera center, 10m ahead
        let (x, y) = project_to_2d(&point, &k);

        assert_relative_eq!(x, 400.0, epsilon = 1.0);
        assert_relative_eq!(y, 300.0, epsilon = 1.0);
    }

    #[test]
    fn test_project_to_2d_offset() {
        let k = build_projection_matrix(800, 600, 90.0);

        // Point to the right and up should project accordingly
        let point = Vector3D::new(5.0, -2.0, 10.0); // right=5, up=2 (y is down), depth=10
        let (x, y) = project_to_2d(&point, &k);

        // Should be to the right of center
        assert!(x > 400.0, "x={} should be > 400", x);
        // Should be above center (y is down, so negative y means up)
        assert!(y < 300.0, "y={} should be < 300", y);
    }

    #[test]
    fn test_world_to_camera_identity() {
        // Camera at origin with identity rotation
        use crate::geom::Rotation;
        let camera_transform = Transform {
            location: Location::new(0.0, 0.0, 0.0),
            rotation: Rotation::new(0.0, 0.0, 0.0),
        };
        let world_point = Location {
            x: 10.0,
            y: 5.0,
            z: 2.0,
        };

        let camera_point = world_to_camera(&world_point, &camera_transform);

        // With identity transform, should just do coordinate conversion
        // UE4 (x=10, y=5, z=2) -> Camera (y=5, -z=-2, x=10)
        assert_relative_eq!(camera_point.x, 5.0, epsilon = 1e-5); // y -> x (right)
        assert_relative_eq!(camera_point.y, -2.0, epsilon = 1e-5); // -z -> y (down)
        assert_relative_eq!(camera_point.z, 10.0, epsilon = 1e-5); // x -> z (forward)
    }

    #[test]
    fn test_full_pipeline() {
        // Test complete projection pipeline
        use crate::geom::Rotation;
        let width = 800;
        let height = 600;
        let fov = 90.0;

        let k = build_projection_matrix(width, height, fov);
        let camera_transform = Transform {
            location: Location::new(0.0, 0.0, 0.0),
            rotation: Rotation::new(0.0, 0.0, 0.0),
        };

        // Point 10m ahead of camera at world coordinates
        let world_point = Location {
            x: 10.0,
            y: 0.0,
            z: 0.0,
        };

        // Transform to camera coordinates
        let camera_point = world_to_camera(&world_point, &camera_transform);

        // Project to 2D
        let (x, y) = project_to_2d(&camera_point, &k);

        // Point straight ahead should be at image center
        assert_relative_eq!(x, 400.0, epsilon = 1.0);
        assert_relative_eq!(y, 300.0, epsilon = 1.0);
    }
}
