pub use ffi_::*;

#[cxx::bridge]
mod ffi_ {
    unsafe extern "C++" {
        include!("csrc/carla_rust.hpp");

        // Client
        type Client;
        fn client_new(host: &CxxString, port: u16) -> UniquePtr<Client>;
        fn client_get_client_version(client: &Client) -> String;
        fn client_get_server_version(client: &Client) -> String;
        fn client_load_world(client: &Client, map_name: &CxxString) -> UniquePtr<World>;
        fn client_get_world(client: &Client) -> UniquePtr<World>;

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
        type SharedBlueprintLibrary;
        fn bp_filter(
            bp: &SharedBlueprintLibrary,
            wildcard_pattern: &CxxString,
        ) -> UniquePtr<SharedBlueprintLibrary>;

        // Actor
        type SharedActor;
    }
}
