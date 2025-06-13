//! Geometry types and utilities.
//!
//! This module provides geometry types and utilities for working with
//! CARLA's coordinate system, including transforms, vectors, and bounding boxes.
//!
//! The module is organized into logical submodules:
//! - `transform` - Transform, Location, and Rotation types and extensions
//! - `vector` - Vector2D and Vector3D types and utilities  
//! - `bounding_box` - Bounding box geometry and operations

// Organize geometry types into submodules
pub mod bounding_box;
pub mod transform;
pub mod vector;

// Re-export commonly used types at the top level for convenience
pub use bounding_box::{BoundingBox, BoundingBoxExt, NalgebraBoundingBox};
pub use transform::{Location, LocationExt, Rotation, RotationExt, Transform, TransformExt};
pub use vector::{Vector2D, Vector2DExt, Vector3D, Vector3DExt};

// Thread safety assertions
use static_assertions::assert_impl_all;
assert_impl_all!(Vector2D: Send, Sync);
assert_impl_all!(Vector3D: Send, Sync);
assert_impl_all!(Location: Send, Sync);
assert_impl_all!(Rotation: Send, Sync);
assert_impl_all!(Transform: Send, Sync);
assert_impl_all!(BoundingBox: Send, Sync);
assert_impl_all!(NalgebraBoundingBox<f32>: Send, Sync);
