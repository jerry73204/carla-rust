pub use ffi_::*;

#[cxx::bridge]
mod ffi_ {
    unsafe extern "C++" {
        include!("csrc/carla_rust.hpp");

        // time_duration
        type time_duration;
        fn milliseconds(self: &time_duration) -> usize;

        // Client
        type Client;
        fn client_new(host: &CxxString, port: u16) -> UniquePtr<Client>;
        fn client_get_client_version(client: &Client) -> String;
        fn client_get_server_version(client: &Client) -> String;
        fn client_load_world(client: &Client, map_name: &CxxString) -> UniquePtr<World>;
        fn client_get_world(client: &Client) -> UniquePtr<World>;
        fn client_get_timeout_millis(client: Pin<&mut Client>) -> usize;
        fn client_set_timeout_millis(client: Pin<&mut Client>, millis: usize);

        // World
        type World;
        #[rust_name = "get_id"]
        fn GetId(self: &World) -> u64;
        fn world_get_map(world: &World) -> UniquePtr<SharedMap>;
        fn world_get_blueprint_library(world: &World) -> UniquePtr<SharedBlueprintLibrary>;
        fn world_get_spectator(world: &World) -> UniquePtr<SharedActor>;

        // Map
        type SharedMap;

        // BlueprintLibrary
        type BlueprintLibrary;
        type SharedBlueprintLibrary;

        fn bp_filter(
            bp: &SharedBlueprintLibrary,
            wildcard_pattern: &CxxString,
        ) -> UniquePtr<SharedBlueprintLibrary>;
        fn bp_to_raw(bp: &SharedBlueprintLibrary) -> *const BlueprintLibrary;

        fn Find(self: &BlueprintLibrary, key: &CxxString) -> *const ActorBlueprint;
        fn size(self: &BlueprintLibrary) -> usize;
        fn at(self: &BlueprintLibrary, pos: usize) -> &ActorBlueprint;
        fn empty(self: &BlueprintLibrary) -> bool;

        // Actor
        type SharedActor;

        // ActorBlueprint
        type ActorBlueprint;

        fn actor_bp_copy(actor_bp: &ActorBlueprint) -> UniquePtr<ActorBlueprint>;
        fn ContainsAttribute(self: &ActorBlueprint, id: &CxxString) -> bool;
        fn actor_bp_set_attribute(
            actor_bp: Pin<&mut ActorBlueprint>,
            id: &CxxString,
            value: &CxxString,
        );
        fn size(self: &ActorBlueprint) -> usize;

        // ActorAttribute
        type ActorAttribute;

        // Transform
        type Transform;

        // Location
        type Location;

        // Rotation
        type Rotation;

        // Vector2D
        type Vector2D;

        // Vector3D
        type Vector3D;
    }
}
