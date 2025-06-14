//! FFI bridge definitions for CARLA C++ integration using CXX.

#[cxx::bridge]
pub mod bridge {
    // Geometry types - Simple versions to avoid conflicts with CARLA's complex types
    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct SimpleVector2D {
        pub x: f64,
        pub y: f64,
    }

    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct SimpleVector3D {
        pub x: f64,
        pub y: f64,
        pub z: f64,
    }

    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct SimpleLocation {
        pub x: f64,
        pub y: f64,
        pub z: f64,
    }

    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct SimpleRotation {
        pub pitch: f64,
        pub yaw: f64,
        pub roll: f64,
    }

    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct SimpleTransform {
        pub location: SimpleLocation,
        pub rotation: SimpleRotation,
    }

    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct SimpleBoundingBox {
        pub location: SimpleLocation,
        pub extent: SimpleVector3D,
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

        // Geometry utility functions
        fn Vector2D_Length(vector: &SimpleVector2D) -> f64;
        fn Vector2D_SquaredLength(vector: &SimpleVector2D) -> f64;
        fn Vector2D_Distance(a: &SimpleVector2D, b: &SimpleVector2D) -> f64;
        fn Vector2D_DistanceSquared(a: &SimpleVector2D, b: &SimpleVector2D) -> f64;
        fn Vector2D_Dot(a: &SimpleVector2D, b: &SimpleVector2D) -> f64;

        fn Vector3D_Length(vector: &SimpleVector3D) -> f64;
        fn Vector3D_SquaredLength(vector: &SimpleVector3D) -> f64;
        fn Vector3D_Distance(a: &SimpleVector3D, b: &SimpleVector3D) -> f64;
        fn Vector3D_DistanceSquared(a: &SimpleVector3D, b: &SimpleVector3D) -> f64;
        fn Vector3D_Dot(a: &SimpleVector3D, b: &SimpleVector3D) -> f64;
        fn Vector3D_Cross(a: &SimpleVector3D, b: &SimpleVector3D) -> SimpleVector3D;

        fn Location_Distance(a: &SimpleLocation, b: &SimpleLocation) -> f64;
        fn Location_DistanceSquared(a: &SimpleLocation, b: &SimpleLocation) -> f64;

        fn Transform_TransformPoint(
            transform: &SimpleTransform,
            point: &SimpleLocation,
        ) -> SimpleLocation;
        fn Transform_InverseTransformPoint(
            transform: &SimpleTransform,
            point: &SimpleLocation,
        ) -> SimpleLocation;
        fn Transform_GetForwardVector(transform: &SimpleTransform) -> SimpleVector3D;
        fn Transform_GetRightVector(transform: &SimpleTransform) -> SimpleVector3D;
        fn Transform_GetUpVector(transform: &SimpleTransform) -> SimpleVector3D;

        fn BoundingBox_Contains(bbox: &SimpleBoundingBox, point: &SimpleLocation) -> bool;
        fn BoundingBox_GetVertices(bbox: &SimpleBoundingBox) -> Vec<SimpleLocation>;
    }
}

// Re-export bridge types for easier access
pub use bridge::{
    // Re-export FFI functions
    create_client,
    Actor,
    ActorBlueprint,
    ActorBlueprint_ContainsAttribute,
    ActorBlueprint_ContainsTag,
    ActorBlueprint_GetId,
    ActorBlueprint_GetTags,
    ActorBlueprint_MatchTags,
    ActorBlueprint_SetAttribute,
    Actor_Destroy,
    Actor_GetDisplayId,
    Actor_GetId,
    Actor_GetLocation,
    Actor_GetTransform,
    Actor_GetTypeId,
    Actor_IsAlive,
    Actor_SetLocation,
    Actor_SetTransform,
    BlueprintLibrary,
    BlueprintLibrary_Find,
    BlueprintLibrary_Size,
    BoundingBox_Contains,
    BoundingBox_GetVertices,
    Client,
    Client_GetServerVersion,
    Client_GetTimeout,
    Client_GetWorld,
    Client_SetTimeout,
    Location_Distance,
    Location_DistanceSquared,
    SimpleBoundingBox,
    SimpleLocation,
    SimpleRotation,
    SimpleTransform,
    SimpleVector2D,
    SimpleVector3D,
    Transform_GetForwardVector,
    Transform_GetRightVector,
    Transform_GetUpVector,
    Transform_InverseTransformPoint,
    Transform_TransformPoint,
    Vector2D_Distance,
    Vector2D_DistanceSquared,
    Vector2D_Dot,
    Vector2D_Length,
    Vector2D_SquaredLength,
    Vector3D_Cross,
    Vector3D_Distance,
    Vector3D_DistanceSquared,
    Vector3D_Dot,
    Vector3D_Length,
    Vector3D_SquaredLength,
    World,
    World_GetBlueprintLibrary,
    World_GetId,
    World_GetSpectator,
    World_SpawnActor,
    World_Tick,
    World_TrySpawnActor,
};
