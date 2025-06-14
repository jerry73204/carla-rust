#![allow(dead_code)]

#[cxx::bridge]
mod ffi {
    // Since we can't directly map CARLA's complex geometry types, let's use our own simple ones
    #[derive(Debug, Clone, Copy)]
    struct SimpleLocation {
        x: f64,
        y: f64,
        z: f64,
    }

    #[derive(Debug, Clone, Copy)]
    struct SimpleRotation {
        pitch: f64,
        yaw: f64,
        roll: f64,
    }

    #[derive(Debug, Clone, Copy)]
    struct SimpleTransform {
        location: SimpleLocation,
        rotation: SimpleRotation,
    }

    #[namespace = "carla::client"]
    unsafe extern "C++" {
        include!("carla_cxx_bridge.h");

        // Client type
        type Client;

        // World type
        type World;

        // Actor type
        type Actor;

        // BlueprintLibrary type
        type BlueprintLibrary;

        // ActorBlueprint type
        type ActorBlueprint;

        // Client creation
        fn create_client(host: &str, port: u16, worker_threads: usize) -> UniquePtr<Client>;

        // Client methods
        fn Client_GetServerVersion(client: &Client) -> String;
        fn Client_SetTimeout(client: Pin<&mut Client>, timeout_seconds: f64);
        fn Client_GetTimeout(client: Pin<&mut Client>) -> f64;
        fn Client_GetWorld(client: &Client) -> SharedPtr<World>;

        // World methods
        fn World_GetId(world: &World) -> u64;
        fn World_GetBlueprintLibrary(world: &World) -> SharedPtr<BlueprintLibrary>;
        fn World_GetSpectator(world: &World) -> SharedPtr<Actor>;
        fn World_Tick(world: &World, timeout_seconds: f64) -> u64;
        unsafe fn World_SpawnActor(
            world: &World,
            blueprint: &ActorBlueprint,
            transform: &SimpleTransform,
            parent: *const Actor,
        ) -> SharedPtr<Actor>;
        unsafe fn World_TrySpawnActor(
            world: &World,
            blueprint: &ActorBlueprint,
            transform: &SimpleTransform,
            parent: *const Actor,
        ) -> SharedPtr<Actor>;

        // Actor methods
        fn Actor_GetId(actor: &Actor) -> u32;
        fn Actor_GetTypeId(actor: &Actor) -> String;
        fn Actor_GetDisplayId(actor: &Actor) -> String;
        fn Actor_GetLocation(actor: &Actor) -> SimpleLocation;
        fn Actor_GetTransform(actor: &Actor) -> SimpleTransform;
        fn Actor_SetLocation(actor: &Actor, location: &SimpleLocation);
        fn Actor_SetTransform(actor: &Actor, transform: &SimpleTransform);
        fn Actor_Destroy(actor: &Actor) -> bool;
        fn Actor_IsAlive(actor: &Actor) -> bool;

        // BlueprintLibrary methods
        fn BlueprintLibrary_Find(library: &BlueprintLibrary, id: &str)
            -> SharedPtr<ActorBlueprint>;
        fn BlueprintLibrary_Size(library: &BlueprintLibrary) -> usize;

        // ActorBlueprint methods
        fn ActorBlueprint_GetId(blueprint: &ActorBlueprint) -> String;
        fn ActorBlueprint_GetTags(blueprint: &ActorBlueprint) -> Vec<String>;
        fn ActorBlueprint_MatchTags(blueprint: &ActorBlueprint, wildcard_pattern: &str) -> bool;
        fn ActorBlueprint_ContainsTag(blueprint: &ActorBlueprint, tag: &str) -> bool;
        fn ActorBlueprint_ContainsAttribute(blueprint: &ActorBlueprint, id: &str) -> bool;
        fn ActorBlueprint_SetAttribute(blueprint: &ActorBlueprint, id: &str, value: &str);
    }
}

// Re-export types
pub use ffi::{
    Actor, ActorBlueprint, BlueprintLibrary, Client, SimpleLocation, SimpleRotation,
    SimpleTransform, World,
};

// High-level wrapper types
pub struct ClientWrapper {
    inner: cxx::UniquePtr<Client>,
}

impl ClientWrapper {
    pub fn new(host: &str, port: u16) -> anyhow::Result<Self> {
        let inner = ffi::create_client(host, port, 0);
        if inner.is_null() {
            anyhow::bail!("Failed to create CARLA client");
        }
        Ok(Self { inner })
    }

    pub fn get_server_version(&self) -> String {
        ffi::Client_GetServerVersion(&self.inner)
    }

    pub fn set_timeout(&mut self, timeout: std::time::Duration) {
        ffi::Client_SetTimeout(self.inner.pin_mut(), timeout.as_secs_f64());
    }

    pub fn get_timeout(&mut self) -> std::time::Duration {
        std::time::Duration::from_secs_f64(ffi::Client_GetTimeout(self.inner.pin_mut()))
    }

    pub fn get_world(&self) -> WorldWrapper {
        let world_ptr = ffi::Client_GetWorld(&self.inner);
        WorldWrapper { inner: world_ptr }
    }
}

pub struct WorldWrapper {
    inner: cxx::SharedPtr<World>,
}

impl WorldWrapper {
    pub fn get_id(&self) -> u64 {
        ffi::World_GetId(&self.inner)
    }

    pub fn get_blueprint_library(&self) -> BlueprintLibraryWrapper {
        let library_ptr = ffi::World_GetBlueprintLibrary(&self.inner);
        BlueprintLibraryWrapper { inner: library_ptr }
    }

    pub fn get_spectator(&self) -> Option<ActorWrapper> {
        let actor_ptr = ffi::World_GetSpectator(&self.inner);
        if actor_ptr.is_null() {
            None
        } else {
            Some(ActorWrapper { inner: actor_ptr })
        }
    }

    pub fn tick(&self, timeout: std::time::Duration) -> u64 {
        ffi::World_Tick(&self.inner, timeout.as_secs_f64())
    }

    pub fn spawn_actor(
        &self,
        blueprint: &ActorBlueprint,
        transform: &SimpleTransform,
        parent: Option<&Actor>,
    ) -> anyhow::Result<ActorWrapper> {
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

pub struct ActorWrapper {
    inner: cxx::SharedPtr<Actor>,
}

impl ActorWrapper {
    pub fn get_id(&self) -> u32 {
        ffi::Actor_GetId(&self.inner)
    }

    pub fn get_type_id(&self) -> String {
        ffi::Actor_GetTypeId(&self.inner)
    }

    pub fn get_display_id(&self) -> String {
        ffi::Actor_GetDisplayId(&self.inner)
    }

    pub fn get_location(&self) -> SimpleLocation {
        ffi::Actor_GetLocation(&self.inner)
    }

    pub fn get_transform(&self) -> SimpleTransform {
        ffi::Actor_GetTransform(&self.inner)
    }

    pub fn set_location(&self, location: &SimpleLocation) {
        ffi::Actor_SetLocation(&self.inner, location);
    }

    pub fn set_transform(&self, transform: &SimpleTransform) {
        ffi::Actor_SetTransform(&self.inner, transform);
    }

    pub fn destroy(&self) -> bool {
        ffi::Actor_Destroy(&self.inner)
    }

    pub fn is_alive(&self) -> bool {
        ffi::Actor_IsAlive(&self.inner)
    }
}

pub struct BlueprintLibraryWrapper {
    inner: cxx::SharedPtr<BlueprintLibrary>,
}

impl BlueprintLibraryWrapper {
    pub fn find(&self, id: &str) -> Option<cxx::SharedPtr<ActorBlueprint>> {
        let blueprint_ptr = ffi::BlueprintLibrary_Find(&self.inner, id);
        if blueprint_ptr.is_null() {
            None
        } else {
            Some(blueprint_ptr)
        }
    }

    // TODO: Implement filter method - CXX doesn't support Vec<SharedPtr<T>>
    // pub fn filter(&self, wildcard_pattern: &str) -> Vec<cxx::SharedPtr<ActorBlueprint>> {
    //     ffi::BlueprintLibrary_Filter(&self.inner, wildcard_pattern)
    // }

    pub fn size(&self) -> usize {
        ffi::BlueprintLibrary_Size(&self.inner)
    }
}

// Extension methods for ActorBlueprint
pub trait ActorBlueprintExt {
    fn get_id(&self) -> String;
    fn get_tags(&self) -> Vec<String>;
    fn match_tags(&self, wildcard_pattern: &str) -> bool;
    fn contains_tag(&self, tag: &str) -> bool;
    fn contains_attribute(&self, id: &str) -> bool;
    fn set_attribute(&mut self, id: &str, value: &str);
}

impl ActorBlueprintExt for ActorBlueprint {
    fn get_id(&self) -> String {
        ffi::ActorBlueprint_GetId(self)
    }

    fn get_tags(&self) -> Vec<String> {
        ffi::ActorBlueprint_GetTags(self)
    }

    fn match_tags(&self, wildcard_pattern: &str) -> bool {
        ffi::ActorBlueprint_MatchTags(self, wildcard_pattern)
    }

    fn contains_tag(&self, tag: &str) -> bool {
        ffi::ActorBlueprint_ContainsTag(self, tag)
    }

    fn contains_attribute(&self, id: &str) -> bool {
        ffi::ActorBlueprint_ContainsAttribute(self, id)
    }

    fn set_attribute(&mut self, id: &str, value: &str) {
        ffi::ActorBlueprint_SetAttribute(self, id, value);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
}
