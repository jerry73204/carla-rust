//! Unit tests for carla-sys functionality.

#[cfg(test)]
mod tests {
    use crate::ffi::{
        SimpleBoundingBox, SimpleLocation, SimpleRotation, SimpleTransform, SimpleVector2D,
        SimpleVector3D,
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
    fn test_location_new() {
        let loc = SimpleLocation::new(5.0, 10.0, 15.0);
        assert_eq!(loc.x, 5.0);
        assert_eq!(loc.y, 10.0);
        assert_eq!(loc.z, 15.0);
    }

    #[test]
    fn test_rotation_creation() {
        let rot = SimpleRotation {
            pitch: 90.0,
            yaw: 180.0,
            roll: 270.0,
        };
        assert_eq!(rot.pitch, 90.0);
        assert_eq!(rot.yaw, 180.0);
        assert_eq!(rot.roll, 270.0);
    }

    #[test]
    fn test_rotation_new() {
        let rot = SimpleRotation::new(45.0, 90.0, 135.0);
        assert_eq!(rot.pitch, 45.0);
        assert_eq!(rot.yaw, 90.0);
        assert_eq!(rot.roll, 135.0);
    }

    #[test]
    fn test_rotation_zero() {
        let zero = SimpleRotation::ZERO;
        assert_eq!(zero.pitch, 0.0);
        assert_eq!(zero.yaw, 0.0);
        assert_eq!(zero.roll, 0.0);
    }

    #[test]
    fn test_vector2d_creation() {
        let v = SimpleVector2D::new(3.0, 4.0);
        assert_eq!(v.x, 3.0);
        assert_eq!(v.y, 4.0);
    }

    #[test]
    fn test_vector3d_creation() {
        let v = SimpleVector3D::new(1.0, 2.0, 3.0);
        assert_eq!(v.x, 1.0);
        assert_eq!(v.y, 2.0);
        assert_eq!(v.z, 3.0);
    }

    #[test]
    fn test_transform_creation() {
        let location = SimpleLocation::new(10.0, 20.0, 30.0);
        let rotation = SimpleRotation::new(0.0, 0.0, 0.0);

        let transform = SimpleTransform::new(location, rotation);
        assert_eq!(transform.location.x, 10.0);
        assert_eq!(transform.location.y, 20.0);
        assert_eq!(transform.location.z, 30.0);
        assert_eq!(transform.rotation.pitch, 0.0);
        assert_eq!(transform.rotation.yaw, 0.0);
        assert_eq!(transform.rotation.roll, 0.0);
    }

    #[test]
    fn test_bounding_box_creation() {
        let location = SimpleLocation::new(0.0, 0.0, 0.0);
        let extent = SimpleVector3D::new(2.0, 1.0, 1.5);

        let bbox = SimpleBoundingBox::new(location, extent);
        assert_eq!(bbox.location.x, 0.0);
        assert_eq!(bbox.extent.x, 2.0);
        assert_eq!(bbox.extent.y, 1.0);
        assert_eq!(bbox.extent.z, 1.5);
    }
}
