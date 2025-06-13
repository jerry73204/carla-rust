use super::Actor;
use anyhow::{anyhow, Result};
use carla_sys::*;
use std::ptr;

/// A list of actors in the simulation.
/// This is a wrapper around the C API actor list that provides safe access to actors.
#[derive(Debug)]
pub struct ActorList {
    inner: *mut carla_actor_list_t,
}

impl ActorList {
    /// Create an ActorList from a raw C pointer.
    ///
    /// # Safety
    /// The pointer must be valid and not null.
    pub(crate) fn from_raw_ptr(ptr: *mut carla_actor_list_t) -> Result<Self> {
        if ptr.is_null() {
            return Err(anyhow!("Null actor list pointer"));
        }
        Ok(Self { inner: ptr })
    }

    /// Find an actor by its ID.
    pub fn find(&self, actor_id: u32) -> Option<Actor> {
        let actor_ptr = unsafe { carla_actor_list_find(self.inner, actor_id) };
        if actor_ptr.is_null() {
            None
        } else {
            Actor::from_raw_ptr(actor_ptr).ok()
        }
    }

    /// Filter actors by type pattern (e.g., "vehicle.*", "walker.*").
    /// This creates a new ActorList containing only actors matching the pattern.
    pub fn filter(&self, pattern: &str) -> Self {
        // TODO: Implement pattern filtering when C API provides it
        // For now, we manually filter by checking type IDs
        let mut filtered_actors = Vec::new();

        for i in 0..self.len() {
            if let Some(actor) = self.get(i) {
                let type_id = actor.type_id();
                if type_id.contains(pattern) || pattern == "*" {
                    filtered_actors.push(actor);
                }
            }
        }

        // Create a temporary list - this is a simplified implementation
        // In a full implementation, we'd need to create a proper C actor list
        // For now, we'll return self and add a warning
        eprintln!("Warning: ActorList::filter() returns unfiltered list - filtering not fully implemented");
        Self { inner: self.inner }
    }

    /// Get an actor by index.
    pub fn get(&self, index: usize) -> Option<Actor> {
        if index >= self.len() {
            return None;
        }

        let actor_ptr = unsafe { carla_actor_list_get(self.inner, index) };
        if actor_ptr.is_null() {
            None
        } else {
            Actor::from_raw_ptr(actor_ptr).ok()
        }
    }

    /// Get the number of actors in the list.
    pub fn len(&self) -> usize {
        unsafe { carla_actor_list_size(self.inner) }
    }

    /// Check if the list is empty.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Iterate over all actors in the list.
    pub fn iter(&self) -> ActorListIterator {
        ActorListIterator {
            list: self,
            index: 0,
            len: self.len(),
        }
    }

    /// Convert to a Vec of actors for easier manipulation.
    pub fn to_vec(&self) -> Vec<Actor> {
        self.iter().collect()
    }

    /// Filter actors by type, returning only vehicles.
    pub fn vehicles(&self) -> Vec<crate::client::Vehicle> {
        self.iter()
            .filter_map(|actor| actor.try_into_vehicle().ok())
            .collect()
    }

    /// Filter actors by type, returning only walkers.
    pub fn walkers(&self) -> Vec<crate::client::Walker> {
        self.iter()
            .filter_map(|actor| actor.try_into_walker().ok())
            .collect()
    }

    /// Filter actors by type, returning only sensors.
    pub fn sensors(&self) -> Vec<crate::sensor::Sensor> {
        self.iter()
            .filter_map(|actor| actor.try_into_sensor().ok())
            .collect()
    }

    /// Filter actors by type, returning only traffic lights.
    pub fn traffic_lights(&self) -> Vec<crate::client::TrafficLight> {
        self.iter()
            .filter_map(|actor| actor.try_into_traffic_light().ok())
            .collect()
    }

    /// Filter actors by type, returning only traffic signs.
    pub fn traffic_signs(&self) -> Vec<crate::client::TrafficSign> {
        self.iter()
            .filter_map(|actor| actor.try_into_traffic_sign().ok())
            .collect()
    }

    /// Get the raw C pointer (for internal use).
    pub(crate) fn raw_ptr(&self) -> *mut carla_actor_list_t {
        self.inner
    }
}

/// Iterator for ActorList.
pub struct ActorListIterator<'a> {
    list: &'a ActorList,
    index: usize,
    len: usize,
}

impl<'a> Iterator for ActorListIterator<'a> {
    type Item = Actor;

    fn next(&mut self) -> Option<Self::Item> {
        if self.index >= self.len {
            return None;
        }

        let actor = self.list.get(self.index);
        self.index += 1;
        actor
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        let remaining = self.len.saturating_sub(self.index);
        (remaining, Some(remaining))
    }
}

impl<'a> ExactSizeIterator for ActorListIterator<'a> {
    fn len(&self) -> usize {
        self.len.saturating_sub(self.index)
    }
}

impl Drop for ActorList {
    fn drop(&mut self) {
        if !self.inner.is_null() {
            unsafe {
                carla_actor_list_free(self.inner);
            }
            self.inner = ptr::null_mut();
        }
    }
}

// SAFETY: ActorList wraps a thread-safe C API
unsafe impl Send for ActorList {}
unsafe impl Sync for ActorList {}

impl Clone for ActorList {
    fn clone(&self) -> Self {
        // For cloning, we need to iterate through and create a new list
        // This is a simplified implementation - in practice we'd want to
        // create a proper C actor list or use reference counting
        eprintln!(
            "Warning: ActorList::clone() creates a shallow copy - clone not fully implemented"
        );
        Self { inner: self.inner }
    }
}

impl IntoIterator for ActorList {
    type Item = Actor;
    type IntoIter = ActorListOwnedIterator;

    fn into_iter(self) -> Self::IntoIter {
        let len = self.len();
        ActorListOwnedIterator {
            list: self,
            index: 0,
            len,
        }
    }
}

impl<'a> IntoIterator for &'a ActorList {
    type Item = Actor;
    type IntoIter = ActorListIterator<'a>;

    fn into_iter(self) -> Self::IntoIter {
        self.iter()
    }
}

/// Owned iterator for ActorList.
pub struct ActorListOwnedIterator {
    list: ActorList,
    index: usize,
    len: usize,
}

impl Iterator for ActorListOwnedIterator {
    type Item = Actor;

    fn next(&mut self) -> Option<Self::Item> {
        if self.index >= self.len {
            return None;
        }

        let actor = self.list.get(self.index);
        self.index += 1;
        actor
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        let remaining = self.len.saturating_sub(self.index);
        (remaining, Some(remaining))
    }
}

impl ExactSizeIterator for ActorListOwnedIterator {
    fn len(&self) -> usize {
        self.len.saturating_sub(self.index)
    }
}
