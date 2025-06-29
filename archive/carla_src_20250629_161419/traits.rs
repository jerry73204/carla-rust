//! Common traits for CARLA actors and objects.
//!
//! This module provides trait-based abstractions that capture common behaviors
//! across different CARLA object types, enabling generic programming patterns.

use crate::{
    geom::{BoundingBox, Transform},
    ActorExt,
};

/// Trait for objects that can be collected into lists.
///
/// Provides iteration and filtering capabilities for actor collections.
pub trait CollectionT<T> {
    /// Get the number of items in the collection.
    fn len(&self) -> usize;

    /// Check if the collection is empty.
    fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Get an item by index.
    fn get(&self, index: usize) -> Option<&T>;

    /// Find an item by predicate.
    fn find<P>(&self, predicate: P) -> Option<&T>
    where
        P: Fn(&T) -> bool;

    /// Filter items by predicate.
    fn filter<P>(&self, predicate: P) -> Vec<&T>
    where
        P: Fn(&T) -> bool;

    /// Convert to a vector.
    fn to_vec(&self) -> Vec<T>
    where
        T: Clone;
}

/// Marker trait for types that can be spawned in the world.
pub trait SpawnableT {
    /// The type returned after spawning.
    type SpawnedType;

    /// Get the blueprint ID for spawning.
    fn blueprint_id(&self) -> &str;
}

/// Trait for objects that have a bounding box.
pub trait BoundingBoxT {
    /// Get the object's bounding box in local coordinates.
    fn get_bounding_box(&self) -> BoundingBox;

    /// Get the object's bounding box in world coordinates.
    fn get_world_bounding_box(&self) -> BoundingBox {
        let _bbox = self.get_bounding_box();
        let _transform = if let Some(actor) = self.as_actor() {
            actor.transform()
        } else {
            Transform::default()
        };

        // Transform bounding box to world coordinates
        todo!("Implement bounding box transformation")
    }

    /// Attempt to cast to an actor (for transform access).
    fn as_actor(&self) -> Option<&dyn ActorExt> {
        None
    }
}
