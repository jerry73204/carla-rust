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
        unsafe fn world_try_spawn_actor(
            world: Pin<&mut World>,
            blueprint: &ActorBlueprint,
            transform: &Transform,
            parent: *const SharedActor,
        ) -> UniquePtr<SharedActor>;

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
        type Actor;
        type SharedActor;

        fn actor_get_location(actor: &SharedActor) -> UniquePtr<Location>;
        fn actor_get_transform(actor: &SharedActor) -> UniquePtr<Transform>;
        fn actor_get_velocity(actor: &SharedActor) -> UniquePtr<Vector3D>;
        fn actor_get_angular_velocity(actor: &SharedActor) -> UniquePtr<Vector3D>;
        fn actor_get_acceleration(actor: &SharedActor) -> UniquePtr<Vector3D>;

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

        // Location
        type Location;
        fn location_from_xyz(x: f32, y: f32, z: f32) -> UniquePtr<Location>;
        fn location_as_vec3d(loc: &Location) -> &Vector3D;

        // Rotation
        type Rotation;
        fn rotation_from_pitch_yaw_roll(p: f32, r: f32, y: f32) -> UniquePtr<Rotation>;
        fn rotation_roll(rot: &Rotation) -> f32;
        fn rotation_pitch(rot: &Rotation) -> f32;
        fn rotation_yaw(rot: &Rotation) -> f32;

        // Transform
        type Transform;
        fn transform_new(loc: &Location, rot: &Rotation) -> UniquePtr<Transform>;
        fn transform_get_location(trans: &Transform) -> &Location;
        fn transform_get_rotation(trans: &Transform) -> &Rotation;

        // Vector2D
        type Vector2D;
        fn vec2d_new(x: f32, y: f32) -> UniquePtr<Vector2D>;
        fn vec2d_x(v: &Vector2D) -> f32;
        fn vec2d_y(v: &Vector2D) -> f32;

        // Vector3D
        type Vector3D;
        fn vec3d_new(x: f32, y: f32, z: f32) -> UniquePtr<Vector3D>;
        fn vec3d_x(v: &Vector3D) -> f32;
        fn vec3d_y(v: &Vector3D) -> f32;
        fn vec3d_z(v: &Vector3D) -> f32;
    }
}
