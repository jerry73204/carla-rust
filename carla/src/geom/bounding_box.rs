//! Bounding box geometry types.

use super::transform::{Location, LocationExt, Rotation, RotationExt, Transform};
use nalgebra::{Isometry3, Vector3};

// For compatibility with existing code that expects these types
pub type BoundingBox = carla_sys::carla_transform_t; // Placeholder until we have proper bounding box

/// A nalgebra-based bounding box geometry object that provides
/// convenient mathematical operations on CARLA's native bounding box.
#[derive(Debug, Clone)]
pub struct NalgebraBoundingBox<T> {
    /// The pose of the center point.
    pub transform: Isometry3<T>,
    /// The lengths of box sides.
    pub extent: Vector3<T>,
}

impl NalgebraBoundingBox<f32> {
    /// Convert to a CARLA's native C transform.
    /// Note: BoundingBox is currently a placeholder using Transform.
    /// This will be updated when proper bounding box C struct is available.
    pub fn to_native(&self) -> BoundingBox {
        let Self { transform, .. } = self;
        BoundingBox {
            location: Location::from_na_translation(&transform.translation),
            rotation: Rotation::from_na(&transform.rotation),
        }
    }

    /// Create from a CARLA's native C transform.
    /// Note: BoundingBox is currently a placeholder using Transform.
    /// This will be updated when proper bounding box C struct is available.
    pub fn from_native(bbox: &BoundingBox) -> Self {
        let BoundingBox { location, rotation } = bbox;
        Self {
            transform: Isometry3 {
                rotation: rotation.to_na(),
                translation: location.to_na_translation(),
            },
            extent: Vector3::new(1.0, 1.0, 1.0), // Default extent
        }
    }
}

/// Extension methods for the native C BoundingBox type (placeholder implementation)
pub trait BoundingBoxExt {
    /// Check if a point is contained within the bounding box
    fn contains_point(&self, point: &Location, bbox_transform: &Transform) -> bool;
}

impl BoundingBoxExt for BoundingBox {
    fn contains_point(&self, _point: &Location, _bbox_transform: &Transform) -> bool {
        // TODO: Implement when proper bounding box C functions are available
        false
    }
}
