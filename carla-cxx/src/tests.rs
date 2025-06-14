//! Unit tests for carla-cxx functionality.

#[cfg(test)]
mod tests {
    use crate::{
        ffi::{
            SimpleBoundingBox, SimpleLocation, SimpleRotation, SimpleTransform, SimpleVector2D,
            SimpleVector3D,
        },
        geometry::*,
    };

    #[test]
    fn test_location_creation() {
        let loc = SimpleLocation {
            x: 1.0,
            y: 2.0,
            z: 3.0,
        };
        assert_eq!(loc.x, 1.0);
        assert_eq!(loc.y, 2.0);
        assert_eq!(loc.z, 3.0);
    }

    #[test]
    fn test_rotation_creation() {
        let rot = SimpleRotation {
            pitch: 10.0,
            yaw: 20.0,
            roll: 30.0,
        };
        assert_eq!(rot.pitch, 10.0);
        assert_eq!(rot.yaw, 20.0);
        assert_eq!(rot.roll, 30.0);
    }

    #[test]
    fn test_transform_creation() {
        let transform = SimpleTransform {
            location: SimpleLocation {
                x: 1.0,
                y: 2.0,
                z: 3.0,
            },
            rotation: SimpleRotation {
                pitch: 10.0,
                yaw: 20.0,
                roll: 30.0,
            },
        };
        assert_eq!(transform.location.x, 1.0);
        assert_eq!(transform.location.y, 2.0);
        assert_eq!(transform.location.z, 3.0);
        assert_eq!(transform.rotation.pitch, 10.0);
        assert_eq!(transform.rotation.yaw, 20.0);
        assert_eq!(transform.rotation.roll, 30.0);
    }

    #[test]
    fn test_vector2d_operations() {
        let v1 = SimpleVector2D::new(3.0, 4.0);
        let v2 = SimpleVector2D::new(1.0, 2.0);

        // Test basic properties
        assert_eq!(v1.x, 3.0);
        assert_eq!(v1.y, 4.0);

        // Test arithmetic operations
        let sum = v1 + v2;
        assert_eq!(sum.x, 4.0);
        assert_eq!(sum.y, 6.0);

        let diff = v1 - v2;
        assert_eq!(diff.x, 2.0);
        assert_eq!(diff.y, 2.0);

        let scaled = v1 * 2.0;
        assert_eq!(scaled.x, 6.0);
        assert_eq!(scaled.y, 8.0);

        // Test constants
        assert_eq!(SimpleVector2D::ZERO, SimpleVector2D { x: 0.0, y: 0.0 });
        assert_eq!(SimpleVector2D::UNIT_X, SimpleVector2D { x: 1.0, y: 0.0 });
        assert_eq!(SimpleVector2D::UNIT_Y, SimpleVector2D { x: 0.0, y: 1.0 });
    }

    #[test]
    fn test_vector3d_operations() {
        let v1 = SimpleVector3D::new(1.0, 2.0, 3.0);
        let v2 = SimpleVector3D::new(4.0, 5.0, 6.0);

        // Test basic properties
        assert_eq!(v1.x, 1.0);
        assert_eq!(v1.y, 2.0);
        assert_eq!(v1.z, 3.0);

        // Test arithmetic operations
        let sum = v1 + v2;
        assert_eq!(sum.x, 5.0);
        assert_eq!(sum.y, 7.0);
        assert_eq!(sum.z, 9.0);

        let diff = v2 - v1;
        assert_eq!(diff.x, 3.0);
        assert_eq!(diff.y, 3.0);
        assert_eq!(diff.z, 3.0);

        let scaled = v1 * 2.0;
        assert_eq!(scaled.x, 2.0);
        assert_eq!(scaled.y, 4.0);
        assert_eq!(scaled.z, 6.0);

        // Test conversions
        let v2d = v1.to_2d();
        assert_eq!(v2d.x, 1.0);
        assert_eq!(v2d.y, 2.0);

        // Test constants
        assert_eq!(
            SimpleVector3D::ZERO,
            SimpleVector3D {
                x: 0.0,
                y: 0.0,
                z: 0.0
            }
        );
        assert_eq!(
            SimpleVector3D::UNIT_X,
            SimpleVector3D {
                x: 1.0,
                y: 0.0,
                z: 0.0
            }
        );
        assert_eq!(
            SimpleVector3D::UNIT_Y,
            SimpleVector3D {
                x: 0.0,
                y: 1.0,
                z: 0.0
            }
        );
        assert_eq!(
            SimpleVector3D::UNIT_Z,
            SimpleVector3D {
                x: 0.0,
                y: 0.0,
                z: 1.0
            }
        );
    }

    #[test]
    fn test_location_operations() {
        let loc1 = SimpleLocation::new(1.0, 2.0, 3.0);
        let loc2 = SimpleLocation::new(4.0, 5.0, 6.0);
        let vec = SimpleVector3D::new(1.0, 1.0, 1.0);

        // Test basic properties
        assert_eq!(loc1.x, 1.0);
        assert_eq!(loc1.y, 2.0);
        assert_eq!(loc1.z, 3.0);

        // Test operations with vectors
        let translated = loc1 + vec;
        assert_eq!(translated.x, 2.0);
        assert_eq!(translated.y, 3.0);
        assert_eq!(translated.z, 4.0);

        let moved_back = translated - vec;
        assert_eq!(moved_back, loc1);

        // Test location difference
        let diff = loc2 - loc1;
        assert_eq!(diff.x, 3.0);
        assert_eq!(diff.y, 3.0);
        assert_eq!(diff.z, 3.0);

        // Test conversions
        let as_vector = loc1.to_vector();
        assert_eq!(as_vector.x, 1.0);
        assert_eq!(as_vector.y, 2.0);
        assert_eq!(as_vector.z, 3.0);

        // Test constants
        assert_eq!(
            SimpleLocation::ZERO,
            SimpleLocation {
                x: 0.0,
                y: 0.0,
                z: 0.0
            }
        );
    }

    #[test]
    fn test_rotation_advanced() {
        let rot = SimpleRotation::new(10.0, 20.0, 30.0);
        assert_eq!(rot.pitch, 10.0);
        assert_eq!(rot.yaw, 20.0);
        assert_eq!(rot.roll, 30.0);

        // Test degree/radian conversions
        let rot_rad = SimpleRotation::from_radians(
            90.0_f64.to_radians(),
            180.0_f64.to_radians(),
            270.0_f64.to_radians(),
        );
        assert!((rot_rad.pitch - 90.0).abs() < 1e-10);
        assert!((rot_rad.yaw - 180.0).abs() < 1e-10);
        assert!((rot_rad.roll - 270.0).abs() < 1e-10);

        let (pitch_rad, yaw_rad, roll_rad) = rot_rad.to_radians();
        assert!((pitch_rad - 90.0_f64.to_radians()).abs() < 1e-10);
        assert!((yaw_rad - 180.0_f64.to_radians()).abs() < 1e-10);
        assert!((roll_rad - 270.0_f64.to_radians()).abs() < 1e-10);

        // Test constants
        assert_eq!(
            SimpleRotation::ZERO,
            SimpleRotation {
                pitch: 0.0,
                yaw: 0.0,
                roll: 0.0
            }
        );
    }

    #[test]
    fn test_transform_operations() {
        let location = SimpleLocation::new(1.0, 2.0, 3.0);
        let rotation = SimpleRotation::new(10.0, 20.0, 30.0);
        let transform = SimpleTransform::new(location, rotation);

        // Test construction methods
        let loc_only = SimpleTransform::from_location(location);
        assert_eq!(loc_only.location, location);
        assert_eq!(loc_only.rotation, SimpleRotation::ZERO);

        let rot_only = SimpleTransform::from_rotation(rotation);
        assert_eq!(rot_only.location, SimpleLocation::ZERO);
        assert_eq!(rot_only.rotation, rotation);

        // Test translation
        let offset = SimpleVector3D::new(5.0, 6.0, 7.0);
        let translated = transform.translate(&offset);
        assert_eq!(translated.location.x, 6.0);
        assert_eq!(translated.location.y, 8.0);
        assert_eq!(translated.location.z, 10.0);
        assert_eq!(translated.rotation, rotation);

        // Test rotation addition
        let additional_rot = SimpleRotation::new(5.0, 10.0, 15.0);
        let rotated = transform.rotate(&additional_rot);
        assert_eq!(rotated.location, location);
        assert_eq!(rotated.rotation.pitch, 15.0);
        assert_eq!(rotated.rotation.yaw, 30.0);
        assert_eq!(rotated.rotation.roll, 45.0);

        // Test constants
        assert_eq!(SimpleTransform::IDENTITY.location, SimpleLocation::ZERO);
        assert_eq!(SimpleTransform::IDENTITY.rotation, SimpleRotation::ZERO);
    }

    #[test]
    fn test_bounding_box() {
        let center = SimpleLocation::new(0.0, 0.0, 0.0);
        let extent = SimpleVector3D::new(5.0, 10.0, 2.0);
        let bbox = SimpleBoundingBox::new(center, extent);

        // Test basic properties
        assert_eq!(bbox.get_center(), center);
        assert_eq!(bbox.get_extent(), extent);

        // Test min/max calculations
        let min = bbox.get_min();
        assert_eq!(min.x, -5.0);
        assert_eq!(min.y, -10.0);
        assert_eq!(min.z, -2.0);

        let max = bbox.get_max();
        assert_eq!(max.x, 5.0);
        assert_eq!(max.y, 10.0);
        assert_eq!(max.z, 2.0);

        // Test expansion
        let expanded = bbox.expand(1.0);
        assert_eq!(expanded.extent.x, 6.0);
        assert_eq!(expanded.extent.y, 11.0);
        assert_eq!(expanded.extent.z, 3.0);
    }

    #[test]
    fn test_conversions() {
        // Test tuple conversions
        let v2d: SimpleVector2D = (3.0, 4.0).into();
        assert_eq!(v2d.x, 3.0);
        assert_eq!(v2d.y, 4.0);

        let v3d: SimpleVector3D = (1.0, 2.0, 3.0).into();
        assert_eq!(v3d.x, 1.0);
        assert_eq!(v3d.y, 2.0);
        assert_eq!(v3d.z, 3.0);

        let loc: SimpleLocation = (5.0, 6.0, 7.0).into();
        assert_eq!(loc.x, 5.0);
        assert_eq!(loc.y, 6.0);
        assert_eq!(loc.z, 7.0);

        let rot: SimpleRotation = (10.0, 20.0, 30.0).into();
        assert_eq!(rot.pitch, 10.0);
        assert_eq!(rot.yaw, 20.0);
        assert_eq!(rot.roll, 30.0);

        // Test vector/location conversions
        let vector = SimpleVector3D::new(1.0, 2.0, 3.0);
        let location: SimpleLocation = vector.into();
        assert_eq!(location.x, 1.0);
        assert_eq!(location.y, 2.0);
        assert_eq!(location.z, 3.0);

        let back_to_vector: SimpleVector3D = location.into();
        assert_eq!(back_to_vector.x, 1.0);
        assert_eq!(back_to_vector.y, 2.0);
        assert_eq!(back_to_vector.z, 3.0);
    }
}
