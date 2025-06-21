//! Actor list implementation that wraps the C++ vector of actors.

use crate::{actor::Actor, traits::ActorT};
use carla_sys::{ffi::bridge::SimpleActorList, WorldWrapper};
use std::sync::Arc;

/// A list of actors that efficiently wraps the C++ vector.
///
/// This type provides Vec-like operations while avoiding unnecessary conversions
/// from the underlying C++ representation.
#[derive(Debug, Clone)]
pub struct ActorList {
    /// The underlying actor ID list from C++
    inner: SimpleActorList,
    /// Reference to the world for actor lookups
    world: Arc<WorldWrapper>,
}

impl ActorList {
    /// Create a new ActorList from the FFI representation.
    pub(crate) fn new(inner: SimpleActorList, world: Arc<WorldWrapper>) -> Self {
        Self { inner, world }
    }

    /// Get the number of actors in the list.
    pub fn len(&self) -> usize {
        self.inner.actor_ids.len()
    }

    /// Check if the list is empty.
    pub fn is_empty(&self) -> bool {
        self.inner.actor_ids.is_empty()
    }

    /// Get the actor IDs without fetching full actor data.
    pub fn ids(&self) -> &[u32] {
        &self.inner.actor_ids
    }

    /// Get an actor by index.
    pub fn get(&self, index: usize) -> Option<Actor> {
        self.inner.actor_ids.get(index).and_then(|&id| {
            self.world
                .get_actor(id)
                .map(|actor_wrapper| Actor::from_cxx(actor_wrapper))
        })
    }

    /// Find actors matching a type pattern.
    pub fn find_by_type(&self, pattern: &str) -> Vec<Actor> {
        self.iter()
            .filter(|actor| {
                let type_id = actor.type_id();
                pattern == "*" || type_id.contains(pattern) || type_id.starts_with(pattern)
            })
            .collect()
    }

    /// Find an actor by ID.
    pub fn find_by_id(&self, id: u32) -> Option<Actor> {
        if self.inner.actor_ids.contains(&id) {
            self.world
                .get_actor(id)
                .map(|actor_wrapper| Actor::from_cxx(actor_wrapper))
        } else {
            None
        }
    }

    /// Convert to a Vec<Actor>.
    ///
    /// This performs the full conversion, fetching all actor data.
    pub fn to_vec(&self) -> Vec<Actor> {
        self.iter().collect()
    }

    /// Apply a function to each actor.
    pub fn for_each<F>(&self, mut f: F)
    where
        F: FnMut(&Actor),
    {
        for actor in self.iter() {
            f(&actor);
        }
    }

    /// Filter actors using a predicate.
    pub fn filter<F>(&self, predicate: F) -> Vec<Actor>
    where
        F: Fn(&Actor) -> bool,
    {
        self.iter().filter(|actor| predicate(actor)).collect()
    }

    /// Check if any actor matches the predicate.
    pub fn any<F>(&self, predicate: F) -> bool
    where
        F: Fn(&Actor) -> bool,
    {
        self.iter().any(|actor| predicate(&actor))
    }

    /// Check if all actors match the predicate.
    pub fn all<F>(&self, predicate: F) -> bool
    where
        F: Fn(&Actor) -> bool,
    {
        self.iter().all(|actor| predicate(&actor))
    }

    /// Get an iterator over the actors.
    pub fn iter(&self) -> ActorListIter {
        ActorListIter {
            actor_ids: &self.inner.actor_ids,
            world: &self.world,
            index: 0,
        }
    }
}

/// Iterator over actors in an ActorList.
pub struct ActorListIter<'a> {
    actor_ids: &'a [u32],
    world: &'a Arc<WorldWrapper>,
    index: usize,
}

impl<'a> Iterator for ActorListIter<'a> {
    type Item = Actor;

    fn next(&mut self) -> Option<Self::Item> {
        while self.index < self.actor_ids.len() {
            let id = self.actor_ids[self.index];
            self.index += 1;

            if let Some(actor_wrapper) = self.world.get_actor(id) {
                return Some(Actor::from_cxx(actor_wrapper));
            }
            // Skip if actor no longer exists
        }
        None
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        let remaining = self.actor_ids.len() - self.index;
        (0, Some(remaining)) // Lower bound is 0 since actors might have been destroyed
    }
}

impl IntoIterator for ActorList {
    type Item = Actor;
    type IntoIter = ActorListIntoIter;

    fn into_iter(self) -> Self::IntoIter {
        ActorListIntoIter {
            actor_ids: self.inner.actor_ids,
            world: self.world,
            index: 0,
        }
    }
}

/// Consuming iterator over actors in an ActorList.
pub struct ActorListIntoIter {
    actor_ids: Vec<u32>,
    world: Arc<WorldWrapper>,
    index: usize,
}

impl Iterator for ActorListIntoIter {
    type Item = Actor;

    fn next(&mut self) -> Option<Self::Item> {
        while self.index < self.actor_ids.len() {
            let id = self.actor_ids[self.index];
            self.index += 1;

            if let Some(actor_wrapper) = self.world.get_actor(id) {
                return Some(Actor::from_cxx(actor_wrapper));
            }
            // Skip if actor no longer exists
        }
        None
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        let remaining = self.actor_ids.len() - self.index;
        (0, Some(remaining))
    }
}

impl<'a> IntoIterator for &'a ActorList {
    type Item = Actor;
    type IntoIter = ActorListIter<'a>;

    fn into_iter(self) -> Self::IntoIter {
        self.iter()
    }
}

// Implement common traits
impl std::ops::Index<usize> for ActorList {
    type Output = Actor;

    fn index(&self, index: usize) -> &Self::Output {
        panic!("ActorList does not support indexing with references. Use .get() instead.");
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_actor_list_operations() {
        // This would require a mock world/actor setup
        // For now, just verify the types compile
        let _ = |list: ActorList| {
            let _len = list.len();
            let _empty = list.is_empty();
            let _ids = list.ids();
            let _vec = list.to_vec();

            for actor in &list {
                let _ = actor.id();
            }
        };
    }
}
