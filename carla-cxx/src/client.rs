//! High-level wrapper types for CARLA client functionality.

use crate::ffi::{self, Actor, ActorBlueprint, BlueprintLibrary, Client, SimpleTransform, World};
use anyhow::Result;
use cxx::{SharedPtr, UniquePtr};
use std::time::Duration;

/// High-level wrapper for CARLA Client
pub struct ClientWrapper {
    inner: UniquePtr<Client>,
}

impl ClientWrapper {
    /// Create a new CARLA client connection
    pub fn new(host: &str, port: u16) -> Result<Self> {
        let inner = ffi::create_client(host, port, 0);
        if inner.is_null() {
            anyhow::bail!("Failed to create CARLA client");
        }
        Ok(Self { inner })
    }

    /// Get the server version string
    pub fn get_server_version(&self) -> String {
        ffi::Client_GetServerVersion(&self.inner)
    }

    /// Set client timeout duration
    pub fn set_timeout(&mut self, timeout: Duration) {
        ffi::Client_SetTimeout(self.inner.pin_mut(), timeout.as_secs_f64());
    }

    /// Get current client timeout duration
    pub fn get_timeout(&mut self) -> Duration {
        Duration::from_secs_f64(ffi::Client_GetTimeout(self.inner.pin_mut()))
    }

    /// Get the simulation world
    pub fn get_world(&self) -> WorldWrapper {
        let world_ptr = ffi::Client_GetWorld(&self.inner);
        WorldWrapper { inner: world_ptr }
    }
}

/// High-level wrapper for CARLA World
pub struct WorldWrapper {
    inner: SharedPtr<World>,
}

impl WorldWrapper {
    /// Get the world ID
    pub fn get_id(&self) -> u64 {
        ffi::World_GetId(&self.inner)
    }

    /// Get the blueprint library for spawning actors
    pub fn get_blueprint_library(&self) -> BlueprintLibraryWrapper {
        let library_ptr = ffi::World_GetBlueprintLibrary(&self.inner);
        BlueprintLibraryWrapper { inner: library_ptr }
    }

    /// Get the spectator actor (camera)
    pub fn get_spectator(&self) -> Option<ActorWrapper> {
        let actor_ptr = ffi::World_GetSpectator(&self.inner);
        if actor_ptr.is_null() {
            None
        } else {
            Some(ActorWrapper { inner: actor_ptr })
        }
    }

    /// Advance the simulation by one tick
    pub fn tick(&self, timeout: Duration) -> u64 {
        ffi::World_Tick(&self.inner, timeout.as_secs_f64())
    }

    /// Spawn an actor in the world (panics if failed)
    pub fn spawn_actor(
        &self,
        blueprint: &ActorBlueprint,
        transform: &SimpleTransform,
        parent: Option<&Actor>,
    ) -> Result<ActorWrapper> {
        let parent_ptr = parent
            .map(|p| p as *const Actor)
            .unwrap_or(std::ptr::null());
        let actor_ptr =
            unsafe { ffi::World_SpawnActor(&self.inner, blueprint, transform, parent_ptr) };
        if actor_ptr.is_null() {
            anyhow::bail!("Failed to spawn actor");
        }
        Ok(ActorWrapper { inner: actor_ptr })
    }

    /// Try to spawn an actor in the world (returns None if failed)
    pub fn try_spawn_actor(
        &self,
        blueprint: &ActorBlueprint,
        transform: &SimpleTransform,
        parent: Option<&Actor>,
    ) -> Option<ActorWrapper> {
        let parent_ptr = parent
            .map(|p| p as *const Actor)
            .unwrap_or(std::ptr::null());
        let actor_ptr =
            unsafe { ffi::World_TrySpawnActor(&self.inner, blueprint, transform, parent_ptr) };
        if actor_ptr.is_null() {
            None
        } else {
            Some(ActorWrapper { inner: actor_ptr })
        }
    }
}

/// High-level wrapper for CARLA Actor
pub struct ActorWrapper {
    inner: SharedPtr<Actor>,
}

impl ActorWrapper {
    /// Get reference to the inner Actor for casting
    pub fn get_actor(&self) -> &Actor {
        &self.inner
    }

    /// Get the actor's unique ID
    pub fn get_id(&self) -> u32 {
        ffi::Actor_GetId(&self.inner)
    }

    /// Get the actor's type ID string
    pub fn get_type_id(&self) -> String {
        ffi::Actor_GetTypeId(&self.inner)
    }

    /// Get the actor's display ID string
    pub fn get_display_id(&self) -> String {
        ffi::Actor_GetDisplayId(&self.inner)
    }

    /// Get the actor's current location
    pub fn get_location(&self) -> crate::ffi::SimpleLocation {
        ffi::Actor_GetLocation(&self.inner)
    }

    /// Get the actor's current transform
    pub fn get_transform(&self) -> SimpleTransform {
        ffi::Actor_GetTransform(&self.inner)
    }

    /// Set the actor's location
    pub fn set_location(&self, location: &crate::ffi::SimpleLocation) {
        ffi::Actor_SetLocation(&self.inner, location);
    }

    /// Set the actor's transform
    pub fn set_transform(&self, transform: &SimpleTransform) {
        ffi::Actor_SetTransform(&self.inner, transform);
    }

    /// Destroy the actor
    pub fn destroy(&self) -> bool {
        ffi::Actor_Destroy(&self.inner)
    }

    /// Check if the actor is still alive
    pub fn is_alive(&self) -> bool {
        ffi::Actor_IsAlive(&self.inner)
    }
}

/// High-level wrapper for CARLA BlueprintLibrary
pub struct BlueprintLibraryWrapper {
    inner: SharedPtr<BlueprintLibrary>,
}

impl BlueprintLibraryWrapper {
    /// Find a blueprint by ID
    pub fn find(&self, id: &str) -> Option<SharedPtr<ActorBlueprint>> {
        let blueprint_ptr = ffi::BlueprintLibrary_Find(&self.inner, id);
        if blueprint_ptr.is_null() {
            None
        } else {
            Some(blueprint_ptr)
        }
    }

    // TODO: Implement filter method - CXX doesn't support Vec<SharedPtr<T>>
    // pub fn filter(&self, wildcard_pattern: &str) -> Vec<SharedPtr<ActorBlueprint>> {
    //     ffi::BlueprintLibrary_Filter(&self.inner, wildcard_pattern)
    // }

    /// Get the number of blueprints in the library
    pub fn size(&self) -> usize {
        ffi::BlueprintLibrary_Size(&self.inner)
    }
}
