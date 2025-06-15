//! FFI bridge definitions for CARLA C++ integration using CXX.

// Map layer enum values (outside of bridge for CXX compatibility)
pub const MAP_LAYER_NONE: u8 = 0;
pub const MAP_LAYER_BUILDINGS: u8 = 1;
pub const MAP_LAYER_DECALS: u8 = 2;
pub const MAP_LAYER_FOLIAGE: u8 = 4;
pub const MAP_LAYER_GROUND: u8 = 8;
pub const MAP_LAYER_PARKED_VEHICLES: u8 = 16;
pub const MAP_LAYER_PARTICLES: u8 = 32;
pub const MAP_LAYER_PROPS: u8 = 64;
pub const MAP_LAYER_STREET_LIGHTS: u8 = 128;
pub const MAP_LAYER_WALLS: u8 = 255;
pub const MAP_LAYER_ALL: u8 = 255;

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

    // Vehicle control structures
    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct SimpleVehicleControl {
        pub throttle: f32,
        pub steer: f32,
        pub brake: f32,
        pub hand_brake: bool,
        pub reverse: bool,
        pub manual_gear_shift: bool,
        pub gear: i32,
    }

    // Ackermann vehicle control
    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct SimpleAckermannControl {
        pub steer: f32,
        pub steer_speed: f32,
        pub speed: f32,
        pub acceleration: f32,
        pub jerk: f32,
    }

    // Vehicle physics control
    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct SimpleVehiclePhysicsControl {
        pub torque_curve_max_rpm: f32,
        pub torque_curve_max_torque_nm: f32,
        pub max_rpm: f32,
        pub moi: f32,
        pub damping_rate_full_throttle: f32,
        pub damping_rate_zero_throttle_clutch_engaged: f32,
        pub damping_rate_zero_throttle_clutch_disengaged: f32,
        pub use_gear_autobox: bool,
        pub gear_switch_time: f32,
        pub clutch_strength: f32,
        pub final_ratio: f32,
        pub mass: f32,
        pub drag_coefficient: f32,
        pub center_of_mass_x: f32,
        pub center_of_mass_y: f32,
        pub center_of_mass_z: f32,
        pub steering_curve_0: f32,
        pub steering_curve_1: f32,
        pub use_sweep_wheel_collision: bool,
    }

    // Wheel physics control
    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct SimpleWheelPhysicsControl {
        pub tire_friction: f32,
        pub damping_rate: f32,
        pub max_steer_angle: f32,
        pub radius: f32,
        pub max_brake_torque: f32,
        pub max_handbrake_torque: f32,
        pub position_x: f32,
        pub position_y: f32,
        pub position_z: f32,
    }

    // Gear physics control
    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct SimpleGearPhysicsControl {
        pub ratio: f32,
        pub down_ratio: f32,
        pub up_ratio: f32,
    }

    // Vehicle door state
    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct SimpleVehicleDoor {
        pub door_type: u32, // VehicleDoor enum
        pub is_open: bool,
    }

    // Vehicle telemetry data
    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct SimpleVehicleTelemetryData {
        pub speed: f32,
        pub rpm: f32,
        pub gear: i32,
        pub engine_temperature: f32,
        pub fuel_level: f32,
    }

    // Walker control structures
    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct SimpleWalkerControl {
        pub direction: SimpleVector3D,
        pub speed: f32,
        pub jump: bool,
    }

    // Walker AI controller destination
    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct SimpleWalkerDestination {
        pub x: f64,
        pub y: f64,
        pub z: f64,
    }

    // Traffic light states
    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct SimpleTrafficLightState {
        pub red_time: f32,
        pub yellow_time: f32,
        pub green_time: f32,
        pub elapsed_time: f32,
    }

    // Sensor data structures
    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct SimpleLiDARPoint {
        pub x: f32,
        pub y: f32,
        pub z: f32,
        pub intensity: f32,
    }

    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct SimpleRadarDetection {
        pub velocity: f32,
        pub azimuth: f32,
        pub altitude: f32,
        pub depth: f32,
    }

    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct SimpleIMUData {
        pub accelerometer_x: f64,
        pub accelerometer_y: f64,
        pub accelerometer_z: f64,
        pub gyroscope_x: f64,
        pub gyroscope_y: f64,
        pub gyroscope_z: f64,
        pub compass: f64,
    }

    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct SimpleGNSSData {
        pub latitude: f64,
        pub longitude: f64,
        pub altitude: f64,
    }

    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct SimpleImageData {
        pub width: u32,
        pub height: u32,
        pub fov: f32,
        pub data_size: usize,
    }

    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct SimpleCollisionData {
        pub other_actor_id: u32,
        pub normal_impulse: SimpleVector3D,
    }

    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct SimpleCrossedLaneMarking {
        pub lane_type: u32,  // LaneMarkingType enum
        pub color: u32,      // LaneMarkingColor enum
        pub lane_change: u8, // LaneChange enum
    }

    // Advanced sensor data structures
    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct SimpleDVSEvent {
        pub x: u16,    // X pixel coordinate
        pub y: u16,    // Y pixel coordinate
        pub t: i64,    // Timestamp in nanoseconds
        pub pol: bool, // Polarity (true=positive, false=negative)
    }

    #[derive(Debug, Clone, PartialEq)]
    pub struct SimpleDVSEventArray {
        pub width: u32,                  // Image width in pixels
        pub height: u32,                 // Image height in pixels
        pub fov_angle: f32,              // Horizontal field of view
        pub events: Vec<SimpleDVSEvent>, // Array of DVS events
    }

    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct SimpleObstacleDetectionEvent {
        pub self_actor_id: u32,  // Sensor's parent actor ID
        pub other_actor_id: u32, // Detected obstacle actor ID
        pub distance: f32,       // Distance to obstacle in meters
    }

    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct SimpleSemanticLidarDetection {
        pub point: SimpleLocation, // 3D point location (x, y, z)
        pub cos_inc_angle: f32,    // Cosine of incidence angle
        pub object_idx: u32,       // Object instance index
        pub object_tag: u32,       // Semantic tag/label
    }

    #[derive(Debug, Clone, PartialEq)]
    pub struct SimpleSemanticLidarData {
        pub horizontal_angle: f32, // Current horizontal rotation angle
        pub channel_count: u32,    // Number of laser channels
        pub detections: Vec<SimpleSemanticLidarDetection>, // Array of detections
    }

    #[derive(Debug, Clone, PartialEq)]
    pub struct SimpleRssResponse {
        pub response_valid: bool,          // RSS calculation validity
        pub proper_response: String,       // RSS proper response (serialized)
        pub rss_state_snapshot: String,    // Current RSS state (serialized)
        pub situation_snapshot: String,    // Situation analysis (serialized)
        pub world_model: String,           // World model used (serialized)
        pub ego_dynamics_on_route: String, // Ego vehicle dynamics (serialized)
    }

    // Light management data structures
    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct SimpleLightState {
        pub intensity: f32,     // Light intensity in lumens
        pub color: SimpleColor, // RGB color values
        pub group: u8,          // LightGroup enum as u8
        pub active: bool,       // On/off state
    }

    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct SimpleLight {
        pub id: u32,                  // Light ID
        pub location: SimpleLocation, // 3D position
        pub state: SimpleLightState,  // Current light state
    }

    #[derive(Debug, Clone, PartialEq)]
    pub struct SimpleLightManager {
        pub day_night_cycle: bool, // Day/night cycle enabled
        pub lights_count: u32,     // Total number of lights
    }

    // ROS2 integration data structures (simplified)
    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct SimpleVehicleControlCommand {
        pub throttle: f32,
        pub steer: f32,
        pub brake: f32,
        pub hand_brake: bool,
        pub reverse: bool,
        pub gear: i32,
        pub manual_gear_shift: bool,
    }

    // Debug visualization structures
    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct SimpleColor {
        pub r: u8,
        pub g: u8,
        pub b: u8,
    }

    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct SimpleDebugPoint {
        pub location: SimpleLocation,
        pub size: f32,
        pub color: SimpleColor,
        pub life_time: f32,
        pub persistent_lines: bool,
    }

    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct SimpleDebugLine {
        pub begin: SimpleLocation,
        pub end: SimpleLocation,
        pub thickness: f32,
        pub color: SimpleColor,
        pub life_time: f32,
        pub persistent_lines: bool,
    }

    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct SimpleDebugArrow {
        pub begin: SimpleLocation,
        pub end: SimpleLocation,
        pub thickness: f32,
        pub arrow_size: f32,
        pub color: SimpleColor,
        pub life_time: f32,
        pub persistent_lines: bool,
    }

    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct SimpleDebugBox {
        pub bbox: SimpleBoundingBox,
        pub rotation: SimpleRotation,
        pub thickness: f32,
        pub color: SimpleColor,
        pub life_time: f32,
        pub persistent_lines: bool,
    }

    #[derive(Debug, Clone, PartialEq)]
    pub struct SimpleDebugString {
        pub location: SimpleLocation,
        pub text: String,
        pub draw_shadow: bool,
        pub color: SimpleColor,
        pub life_time: f32,
        pub persistent_lines: bool,
    }

    // Time and timestamp types
    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct SimpleTimestamp {
        pub frame: u64,
        pub elapsed_seconds: f64,
        pub delta_seconds: f64,
        pub platform_timestamp: f64,
    }

    // Episode and world settings
    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct SimpleEpisodeSettings {
        pub synchronous_mode: bool,
        pub no_rendering_mode: bool,
        pub fixed_delta_seconds: f64, // Use 0.0 for None/variable time step
        pub substepping: bool,
        pub max_substep_delta_time: f64,
        pub max_substeps: i32,
        pub max_culling_distance: f32,
        pub deterministic_ragdolls: bool,
        pub tile_stream_distance: f32,
        pub actor_active_distance: f32,
        pub spectator_as_ego: bool,
    }

    // Weather parameters
    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct SimpleWeatherParameters {
        pub cloudiness: f32,
        pub precipitation: f32,
        pub precipitation_deposits: f32,
        pub wind_intensity: f32,
        pub sun_azimuth_angle: f32,
        pub sun_altitude_angle: f32,
        pub fog_density: f32,
        pub fog_distance: f32,
        pub fog_falloff: f32,
        pub wetness: f32,
        pub scattering_intensity: f32,
        pub mie_scattering_scale: f32,
        pub rayleigh_scattering_scale: f32,
        pub dust_storm: f32,
    }

    // Recording and playback types
    #[derive(Debug, Clone, PartialEq)]
    pub struct SimpleRecorderInfo {
        pub version: u16,
        pub magic: String,
        pub date: u64, // Unix timestamp
        pub mapfile: String,
        pub frame_count: u32,
        pub duration_seconds: f64,
        pub total_size_mb: f32,
    }

    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct SimpleRecorderCollision {
        pub id: u32,
        pub database_id1: u32,
        pub database_id2: u32,
        pub is_actor1_hero: bool,
        pub is_actor2_hero: bool,
        pub frame: u32,
        pub time_seconds: f64,
    }

    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct SimpleRecorderPosition {
        pub database_id: u32,
        pub location: SimpleLocation,
        pub rotation: SimpleRotation,
        pub frame: u32,
        pub time_seconds: f64,
    }

    // Map and navigation types
    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct SimpleLaneMarking {
        pub lane_marking_type: u32, // LaneMarking::Type enum
        pub color: u8,              // LaneMarking::Color enum
        pub lane_change: u8,        // LaneMarking::LaneChange enum
        pub width: f64,
    }

    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct SimpleWaypointInfo {
        pub id: u64,
        pub road_id: u32,
        pub section_id: u32,
        pub lane_id: i32,
        pub s: f64,
        pub transform: SimpleTransform,
        pub is_junction: bool,
        pub lane_width: f64,
        pub lane_type: u32,  // Lane::LaneType enum
        pub lane_change: u8, // LaneMarking::LaneChange enum
    }

    // Traffic Light info structures
    #[derive(Debug, Clone, PartialEq)]
    pub struct SimpleTrafficLightInfo {
        pub actor_id: u32,
        pub type_id: String,
        pub transform: SimpleTransform,
        pub state: u32, // TrafficLightState as u32
        pub pole_index: u32,
    }

    // World interaction types for ray casting and queries
    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct SimpleLabelledPoint {
        pub location: SimpleLocation,
        pub label: u8, // CityObjectLabel as u8
    }

    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct SimpleOptionalLabelledPoint {
        pub has_value: bool,
        pub value: SimpleLabelledPoint,
    }

    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct SimpleOptionalLocation {
        pub has_value: bool,
        pub value: SimpleLocation,
    }

    // Actor collection for queries
    #[derive(Debug, Clone, PartialEq)]
    pub struct SimpleActorId {
        pub id: u32,
    }

    #[derive(Debug, Clone, PartialEq)]
    pub struct SimpleActorList {
        pub actor_ids: Vec<u32>,
    }

    // Batch operation types
    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct SimpleBatchCommand {
        pub command_type: u8, // CommandType enum as u8
        pub actor_id: u32,
        pub data1: f64,       // Multi-purpose data field 1
        pub data2: f64,       // Multi-purpose data field 2
        pub data3: f64,       // Multi-purpose data field 3
        pub data4: f64,       // Multi-purpose data field 4
        pub data5: f64,       // Multi-purpose data field 5
        pub data6: f64,       // Multi-purpose data field 6
        pub bool_flag1: bool, // Multi-purpose boolean flag 1
        pub bool_flag2: bool, // Multi-purpose boolean flag 2
        pub int_flag1: i32,   // Multi-purpose integer flag 1
        pub int_flag2: i32,   // Multi-purpose integer flag 2
    }

    #[derive(Debug, Clone, PartialEq)]
    pub struct SimpleBatchResponse {
        pub has_error: bool,
        pub error_message: String,
        pub actor_id: u32, // Result actor ID (for spawn commands)
    }

    // Advanced world feature types
    #[derive(Debug, Clone, PartialEq)]
    pub struct SimpleEnvironmentObject {
        pub id: u64,
        pub name: String,
        pub transform: SimpleTransform,
        pub bounding_box: SimpleBoundingBox,
    }

    #[derive(Debug, Clone, PartialEq)]
    pub struct SimpleLandmark {
        pub id: String,
        pub name: String,
        pub type_: String,
        pub distance: f64,
        pub s: f64,
        pub is_dynamic: bool,
        pub orientation: i32,
        pub z_offset: f64,
        pub country: String,
        pub type_code: String,
        pub sub_type: String,
        pub value: f64,
        pub unit: String,
        pub height: f64,
        pub width: f64,
        pub text: String,
        pub h_offset: f64,
        pub pitch: f64,
        pub roll: f64,
    }

    #[derive(Debug, Clone, PartialEq)]
    pub struct SimpleTextureFloatColor {
        pub width: u32,
        pub height: u32,
        pub data: Vec<f32>,
    }

    #[derive(Debug, Clone, PartialEq)]
    pub struct SimpleTextureColor {
        pub width: u32,
        pub height: u32,
        pub data: Vec<u8>,
    }

    #[derive(Debug, Clone, PartialEq)]
    pub struct SimpleLevelBoundingBox {
        pub origin: SimpleLocation,
        pub extent: SimpleVector3D,
        pub rotation: SimpleRotation,
        pub tag: u8,
    }

    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct SimpleJunction {
        pub id: u32,
        pub bounding_box: SimpleBoundingBox,
    }

    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct SimpleGeoLocation {
        pub latitude: f64,
        pub longitude: f64,
        pub altitude: f64,
    }

    // Traffic Manager types
    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct SimpleTrafficManagerConfig {
        pub global_speed_percentage_difference: f32,
        pub global_lane_offset: f32,
        pub global_distance_to_leading_vehicle: f32,
        pub synchronous_mode: bool,
        pub synchronous_mode_timeout_ms: f64,
        pub hybrid_physics_mode: bool,
        pub hybrid_physics_radius: f32,
        pub respawn_dormant_vehicles: bool,
        pub respawn_lower_bound: f32,
        pub respawn_upper_bound: f32,
        pub random_device_seed: u64,
        pub osm_mode: bool,
        pub port: u16,
    }

    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct SimpleTrafficManagerVehicleConfig {
        pub speed_percentage_difference: f32,
        pub desired_speed: f32,
        pub lane_offset: f32,
        pub distance_to_leading_vehicle: f32,
        pub auto_lane_change: bool,
        pub force_lane_change_direction: bool,
        pub force_lane_change_active: bool,
        pub keep_right_percentage: f32,
        pub random_left_lane_change_percentage: f32,
        pub random_right_lane_change_percentage: f32,
        pub percentage_running_light: f32,
        pub percentage_running_sign: f32,
        pub percentage_ignore_walkers: f32,
        pub percentage_ignore_vehicles: f32,
        pub update_vehicle_lights: bool,
        pub collision_detection_enabled: bool,
    }

    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct SimpleTrafficManagerAction {
        pub road_option: u32, // RoadOption enum
        pub waypoint_id: u64, // Waypoint ID (0 if no waypoint)
    }

    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct SimpleTrafficManagerStats {
        pub total_registered_vehicles: u32,
        pub active_vehicle_count: u32,
        pub total_ticks: u32,
        pub average_tick_time_ms: f64,
        pub collision_count: u32,
        pub lane_change_count: u32,
        pub traffic_light_violations: u32,
        pub stop_sign_violations: u32,
        pub total_simulation_time_seconds: f64,
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

        // Specific actor types
        type Vehicle;
        type Walker;
        type WalkerAIController;
        type Sensor;
        type TrafficLight;
        type TrafficSign;

        // BlueprintLibrary type
        type BlueprintLibrary;

        // ActorBlueprint type
        type ActorBlueprint;

        // Map and navigation types
        type Map;
        type Waypoint;
        type Junction;
        type Landmark;

        // Light management types
        type LightManager;

        // ROS2 integration types (simplified - no ROS2 singleton exposure)

        // Client creation
        fn create_client(host: &str, port: u16, worker_threads: usize) -> UniquePtr<Client>;

        // Client methods
        fn Client_GetServerVersion(client: &Client) -> String;
        fn Client_SetTimeout(client: Pin<&mut Client>, timeout_seconds: f64);
        fn Client_GetTimeout(client: Pin<&mut Client>) -> f64;
        fn Client_GetWorld(client: &Client) -> SharedPtr<World>;

        // World/Map management methods
        fn Client_GetAvailableMaps(client: &Client) -> Vec<String>;
        fn Client_LoadWorld(client: &Client, map_name: &str) -> SharedPtr<World>;
        fn Client_ReloadWorld(client: &Client, reset_settings: bool) -> SharedPtr<World>;
        fn Client_GenerateOpenDriveWorld(client: &Client, opendrive: &str) -> SharedPtr<World>;

        // Recording methods
        fn Client_StartRecorder(client: &Client, filename: &str, additional_data: bool) -> String;
        fn Client_StopRecorder(client: &Client);
        fn Client_ShowRecorderFileInfo(client: &Client, filename: &str, show_all: bool) -> String;
        fn Client_ShowRecorderCollisions(
            client: &Client,
            filename: &str,
            type1: u8,
            type2: u8,
        ) -> String;
        fn Client_ShowRecorderActorsBlocked(
            client: &Client,
            filename: &str,
            min_time: f64,
            min_distance: f64,
        ) -> String;

        // Playback methods
        fn Client_ReplayFile(
            client: &Client,
            filename: &str,
            start_time: f64,
            duration: f64,
            follow_id: u32,
            replay_sensors: bool,
        ) -> String;
        fn Client_StopReplayer(client: &Client, keep_actors: bool);
        fn Client_SetReplayerTimeFactor(client: &Client, time_factor: f64);
        fn Client_SetReplayerIgnoreHero(client: &Client, ignore_hero: bool);
        fn Client_SetReplayerIgnoreSpectator(client: &Client, ignore_spectator: bool);

        // Batch operation methods
        fn Client_ApplyBatch(client: &Client, commands: &[SimpleBatchCommand], do_tick_cue: bool);
        fn Client_ApplyBatchSync(
            client: &Client,
            commands: &[SimpleBatchCommand],
            do_tick_cue: bool,
        ) -> Vec<SimpleBatchResponse>;

        // Debug drawing methods - on World, not Client
        fn World_DrawDebugPoint(world: &World, point: &SimpleDebugPoint);
        fn World_DrawDebugLine(world: &World, line: &SimpleDebugLine);
        fn World_DrawDebugArrow(world: &World, arrow: &SimpleDebugArrow);
        fn World_DrawDebugBox(world: &World, box_shape: &SimpleDebugBox);
        fn World_DrawDebugString(world: &World, string: &SimpleDebugString);

        // World methods
        fn World_GetId(world: &World) -> u64;
        fn World_GetBlueprintLibrary(world: &World) -> SharedPtr<BlueprintLibrary>;
        fn World_GetSpectator(world: &World) -> SharedPtr<Actor>;
        fn World_Tick(world: &World, timeout_seconds: f64) -> u64;
        fn World_GetSnapshot(world: &World) -> SimpleTimestamp;
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
        fn World_GetMap(world: &World) -> SharedPtr<Map>;
        fn World_GetSettings(world: &World) -> SimpleEpisodeSettings;
        fn World_ApplySettings(
            world: &World,
            settings: &SimpleEpisodeSettings,
            timeout_seconds: f64,
        ) -> u64;

        // World interaction methods
        // Ray casting functionality
        fn World_CastRay(
            world: &World,
            start_location: &SimpleLocation,
            end_location: &SimpleLocation,
        ) -> Vec<SimpleLabelledPoint>;
        fn World_ProjectPoint(
            world: &World,
            location: &SimpleLocation,
            direction: &SimpleVector3D,
            search_distance: f32,
        ) -> SimpleOptionalLabelledPoint;
        fn World_GroundProjection(
            world: &World,
            location: &SimpleLocation,
            search_distance: f32,
        ) -> SimpleOptionalLabelledPoint;

        // Traffic light queries
        fn World_GetTrafficLightsFromWaypoint(
            world: &World,
            waypoint: &Waypoint,
            distance: f64,
        ) -> SimpleActorList;
        fn World_GetTrafficLightsInJunction(world: &World, junction_id: i32) -> SimpleActorList;

        // Pedestrian navigation
        fn World_GetRandomLocationFromNavigation(world: &World) -> SimpleOptionalLocation;
        fn World_SetPedestriansCrossFactor(world: &World, percentage: f32);

        // Actor query methods
        fn World_GetActors(world: &World) -> SimpleActorList;
        fn World_GetActorsByIds(world: &World, actor_ids: &[u32]) -> SimpleActorList;
        fn World_GetActor(world: &World, actor_id: u32) -> SharedPtr<Actor>;

        // Advanced world features - Map layer management
        fn World_LoadLevelLayer(world: &World, map_layers: u8);
        fn World_UnloadLevelLayer(world: &World, map_layers: u8);

        // Environment object queries
        fn World_GetLevelBBs(world: &World, queried_tag: u8) -> Vec<SimpleLevelBoundingBox>;
        fn World_GetEnvironmentObjects(
            world: &World,
            queried_tag: u8,
        ) -> Vec<SimpleEnvironmentObject>;
        fn World_EnableEnvironmentObjects(world: &World, env_objects_ids: &[u64], enable: bool);

        // Advanced traffic light management
        fn World_ResetAllTrafficLights(world: &World);
        fn World_FreezeAllTrafficLights(world: &World, frozen: bool);
        fn World_GetVehiclesLightStates(world: &World) -> Vec<SimpleBatchCommand>; // Reusing struct for vehicle light states

        // Texture and material application
        fn World_ApplyColorTextureToObject(
            world: &World,
            object_name: &str,
            texture: &SimpleTextureColor,
            material_type: u8,
        );
        fn World_ApplyFloatColorTextureToObject(
            world: &World,
            object_name: &str,
            texture: &SimpleTextureFloatColor,
            material_type: u8,
        );
        fn World_GetNamesOfAllObjects(world: &World) -> Vec<String>;

        // Pedestrian navigation
        fn World_SetPedestriansSeed(world: &World, seed: u32);

        // Light management functions
        fn World_GetLightManager(world: &World) -> SharedPtr<LightManager>;
        fn LightManager_GetAllLights(light_manager: &LightManager, group: u8) -> Vec<SimpleLight>;
        fn LightManager_SetDayNightCycle(light_manager: &LightManager, active: bool);
        fn LightManager_TurnOnLights(light_manager: &LightManager, light_ids: &[u32]);
        fn LightManager_TurnOffLights(light_manager: &LightManager, light_ids: &[u32]);
        fn LightManager_SetLightIntensities(
            light_manager: &LightManager,
            light_ids: &[u32],
            intensity: f32,
        );
        fn LightManager_SetLightColors(
            light_manager: &LightManager,
            light_ids: &[u32],
            color: SimpleColor,
        );
        fn LightManager_SetLightStates(
            light_manager: &LightManager,
            light_ids: &[u32],
            state: SimpleLightState,
        );

        // Landmark and signal management functions
        fn Map_GetAllLandmarks(map: &Map) -> Vec<SimpleLandmark>;
        fn Map_GetLandmarksFromId(map: &Map, id: &str) -> Vec<SimpleLandmark>;
        fn Map_GetAllLandmarksOfType(map: &Map, type_: &str) -> Vec<SimpleLandmark>;
        // fn Map_GetLandmarkGroup(map: &Map, landmark: &Landmark) -> Vec<SimpleLandmark>; // Requires Landmark param
        fn Waypoint_GetAllLandmarksInDistance(
            waypoint: &Waypoint,
            distance: f64,
            stop_at_junction: bool,
        ) -> Vec<SimpleLandmark>;
        fn Waypoint_GetLandmarksOfTypeInDistance(
            waypoint: &Waypoint,
            distance: f64,
            filter_type: &str,
            stop_at_junction: bool,
        ) -> Vec<SimpleLandmark>;

        // Individual landmark property functions (commented out due to CXX SharedPtr limitations)
        // Use collection-based approaches via Map_GetAllLandmarks and Waypoint_GetAllLandmarksInDistance instead
        // fn Landmark_GetWaypoint(landmark: &Landmark) -> SharedPtr<Waypoint>;
        // fn Landmark_GetTransform(landmark: &Landmark) -> SimpleTransform;
        // ... (other individual landmark functions are available in C++ but not exposed via FFI)

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

        // Actor type checking and casting
        fn Actor_CastToVehicle(actor: &Actor) -> SharedPtr<Vehicle>;
        fn Vehicle_CastToActor(vehicle: &Vehicle) -> SharedPtr<Actor>;
        fn Actor_CastToWalker(actor: &Actor) -> SharedPtr<Walker>;
        fn Walker_CastToActor(walker: &Walker) -> SharedPtr<Actor>;
        fn Actor_CastToWalkerAIController(actor: &Actor) -> SharedPtr<WalkerAIController>;
        fn Actor_CastToSensor(actor: &Actor) -> SharedPtr<Sensor>;
        fn Sensor_CastToActor(sensor: &Sensor) -> SharedPtr<Actor>;
        fn Actor_CastToTrafficLight(actor: &Actor) -> SharedPtr<TrafficLight>;
        fn Actor_CastToTrafficSign(actor: &Actor) -> SharedPtr<TrafficSign>;

        // Vehicle methods
        fn Vehicle_ApplyControl(vehicle: &Vehicle, control: &SimpleVehicleControl);
        fn Vehicle_GetControl(vehicle: &Vehicle) -> SimpleVehicleControl;
        fn Vehicle_SetAutopilot(vehicle: &Vehicle, enabled: bool, tm_port: u16);
        fn Vehicle_GetSpeed(vehicle: &Vehicle) -> f32;
        fn Vehicle_GetSpeedLimit(vehicle: &Vehicle) -> f32;
        fn Vehicle_SetLightState(vehicle: &Vehicle, light_state: u32);
        fn Vehicle_GetLightState(vehicle: &Vehicle) -> u32;

        // Vehicle telemetry data
        fn Vehicle_GetTelemetryData(vehicle: &Vehicle) -> SimpleVehicleTelemetryData;

        // Vehicle ID and basic actor properties
        fn Vehicle_GetId(vehicle: &Vehicle) -> u32;
        fn Vehicle_GetTypeId(vehicle: &Vehicle) -> String;
        fn Vehicle_GetTransform(vehicle: &Vehicle) -> SimpleTransform;
        fn Vehicle_SetTransform(vehicle: &Vehicle, transform: &SimpleTransform);
        fn Vehicle_IsAlive(vehicle: &Vehicle) -> bool;
        fn Vehicle_Destroy(vehicle: &Vehicle) -> bool;

        // Advanced vehicle control
        fn Vehicle_ApplyAckermannControl(vehicle: &Vehicle, control: &SimpleAckermannControl);
        fn Vehicle_GetAckermannControl(vehicle: &Vehicle) -> SimpleAckermannControl;
        fn Vehicle_ApplyPhysicsControl(vehicle: &Vehicle, control: &SimpleVehiclePhysicsControl);
        fn Vehicle_GetPhysicsControl(vehicle: &Vehicle) -> SimpleVehiclePhysicsControl;

        // Vehicle telemetry
        fn Vehicle_GetVelocity(vehicle: &Vehicle) -> SimpleVector3D;
        fn Vehicle_GetAngularVelocity(vehicle: &Vehicle) -> SimpleVector3D;
        fn Vehicle_GetAcceleration(vehicle: &Vehicle) -> SimpleVector3D;
        fn Vehicle_GetTireFriction(vehicle: &Vehicle) -> f32;
        fn Vehicle_GetEngineRpm(vehicle: &Vehicle) -> f32;
        fn Vehicle_GetGearRatio(vehicle: &Vehicle) -> f32;

        // Vehicle doors (CARLA 0.10.0)
        fn Vehicle_OpenDoor(vehicle: &Vehicle, door_type: u32);
        fn Vehicle_CloseDoor(vehicle: &Vehicle, door_type: u32);
        fn Vehicle_IsDoorOpen(vehicle: &Vehicle, door_type: u32) -> bool;
        fn Vehicle_GetDoorStates(vehicle: &Vehicle) -> Vec<SimpleVehicleDoor>;

        // Wheel physics
        fn Vehicle_GetWheelPhysicsControls(vehicle: &Vehicle) -> Vec<SimpleWheelPhysicsControl>;
        fn Vehicle_SetWheelPhysicsControls(vehicle: &Vehicle, wheels: &[SimpleWheelPhysicsControl]);

        // Gear physics
        fn Vehicle_GetGearPhysicsControls(vehicle: &Vehicle) -> Vec<SimpleGearPhysicsControl>;
        fn Vehicle_SetGearPhysicsControls(vehicle: &Vehicle, gears: &[SimpleGearPhysicsControl]);

        // Wheel steer angle
        fn Vehicle_GetWheelSteerAngle(vehicle: &Vehicle, wheel_location: u32) -> f32;

        // Actor physics methods (from Actor base class)
        fn Vehicle_SetSimulatePhysics(vehicle: &Vehicle, enabled: bool);
        fn Vehicle_AddImpulse(vehicle: &Vehicle, impulse: &SimpleVector3D);
        fn Vehicle_AddImpulseAtLocation(
            vehicle: &Vehicle,
            impulse: &SimpleVector3D,
            location: &SimpleVector3D,
        );
        fn Vehicle_AddForce(vehicle: &Vehicle, force: &SimpleVector3D);
        fn Vehicle_AddForceAtLocation(
            vehicle: &Vehicle,
            force: &SimpleVector3D,
            location: &SimpleVector3D,
        );
        fn Vehicle_AddTorque(vehicle: &Vehicle, torque: &SimpleVector3D);

        // Walker methods
        fn Walker_ApplyControl(walker: &Walker, control: &SimpleWalkerControl);
        fn Walker_GetControl(walker: &Walker) -> SimpleWalkerControl;
        fn Walker_GetSpeed(walker: &Walker) -> f32;

        // Walker pose control methods (simplified - no bone transforms)
        fn Walker_BlendPose(walker: &Walker, blend: f32);
        fn Walker_ShowPose(walker: &Walker);
        fn Walker_HidePose(walker: &Walker);
        fn Walker_GetPoseFromAnimation(walker: &Walker);

        // Walker Actor interface methods
        fn Walker_GetTypeId(walker: &Walker) -> String;
        fn Walker_GetTransform(walker: &Walker) -> SimpleTransform;
        fn Walker_SetTransform(walker: &Walker, transform: &SimpleTransform);
        fn Walker_GetVelocity(walker: &Walker) -> SimpleVector3D;
        fn Walker_GetAngularVelocity(walker: &Walker) -> SimpleVector3D;
        fn Walker_GetAcceleration(walker: &Walker) -> SimpleVector3D;
        fn Walker_IsAlive(walker: &Walker) -> bool;
        fn Walker_Destroy(walker: &Walker) -> bool;
        fn Walker_SetSimulatePhysics(walker: &Walker, enabled: bool);
        fn Walker_AddImpulse(walker: &Walker, impulse: &SimpleVector3D);
        fn Walker_AddForce(walker: &Walker, force: &SimpleVector3D);
        fn Walker_AddTorque(walker: &Walker, torque: &SimpleVector3D);

        // Walker AI Controller methods
        fn WalkerAIController_Start(controller: &WalkerAIController);
        fn WalkerAIController_Stop(controller: &WalkerAIController);
        fn WalkerAIController_SetMaxSpeed(controller: &WalkerAIController, max_speed: f32);
        fn WalkerAIController_GoToLocation(
            controller: &WalkerAIController,
            destination: &SimpleWalkerDestination,
        );
        fn WalkerAIController_GetRandomLocation(
            controller: &WalkerAIController,
        ) -> SimpleWalkerDestination;
        fn WalkerAIController_HasValidDestination(controller: &WalkerAIController) -> bool;

        // Sensor methods
        fn Sensor_Stop(sensor: &Sensor);
        fn Sensor_IsListening(sensor: &Sensor) -> bool;
        fn Sensor_Listen(sensor: &Sensor);

        // Sensor Actor interface methods
        fn Sensor_GetTypeId(sensor: &Sensor) -> String;
        fn Sensor_GetTransform(sensor: &Sensor) -> SimpleTransform;
        fn Sensor_SetTransform(sensor: &Sensor, transform: &SimpleTransform);
        fn Sensor_GetVelocity(sensor: &Sensor) -> SimpleVector3D;
        fn Sensor_GetAngularVelocity(sensor: &Sensor) -> SimpleVector3D;
        fn Sensor_GetAcceleration(sensor: &Sensor) -> SimpleVector3D;
        fn Sensor_IsAlive(sensor: &Sensor) -> bool;
        fn Sensor_Destroy(sensor: &Sensor) -> bool;
        fn Sensor_SetSimulatePhysics(sensor: &Sensor, enabled: bool);
        fn Sensor_AddImpulse(sensor: &Sensor, impulse: &SimpleVector3D);
        fn Sensor_AddForce(sensor: &Sensor, force: &SimpleVector3D);
        fn Sensor_AddTorque(sensor: &Sensor, torque: &SimpleVector3D);

        // Sensor data retrieval - polling approach
        fn Sensor_GetLastImageData(sensor: &Sensor) -> SimpleImageData;
        fn Sensor_GetImageDataBuffer(sensor: &Sensor, buffer: &mut [u8]) -> bool;
        fn Sensor_GetLastLiDARData(sensor: &Sensor) -> Vec<SimpleLiDARPoint>;
        fn Sensor_GetLastRadarData(sensor: &Sensor) -> Vec<SimpleRadarDetection>;
        fn Sensor_GetLastIMUData(sensor: &Sensor) -> SimpleIMUData;
        fn Sensor_GetLastGNSSData(sensor: &Sensor) -> SimpleGNSSData;
        fn Sensor_GetLastCollisionData(sensor: &Sensor) -> SimpleCollisionData;
        fn Sensor_GetLastLaneInvasionData(sensor: &Sensor) -> Vec<SimpleCrossedLaneMarking>;
        fn Sensor_HasNewData(sensor: &Sensor) -> bool;

        // Advanced sensor data retrieval functions
        fn Sensor_GetLastDVSData(sensor: &Sensor) -> SimpleDVSEventArray;
        fn Sensor_GetLastObstacleDetectionData(sensor: &Sensor) -> SimpleObstacleDetectionEvent;
        fn Sensor_GetLastSemanticLidarData(sensor: &Sensor) -> SimpleSemanticLidarData;
        fn Sensor_GetLastRssData(sensor: &Sensor) -> SimpleRssResponse;

        // Traffic Light methods
        fn TrafficLight_GetState(traffic_light: &TrafficLight) -> u32; // TrafficLightState enum
        fn TrafficLight_SetState(traffic_light: &TrafficLight, state: u32);
        fn TrafficLight_GetElapsedTime(traffic_light: &TrafficLight) -> f32;
        fn TrafficLight_SetRedTime(traffic_light: &TrafficLight, red_time: f32);
        fn TrafficLight_SetYellowTime(traffic_light: &TrafficLight, yellow_time: f32);
        fn TrafficLight_SetGreenTime(traffic_light: &TrafficLight, green_time: f32);
        fn TrafficLight_GetRedTime(traffic_light: &TrafficLight) -> f32;
        fn TrafficLight_GetYellowTime(traffic_light: &TrafficLight) -> f32;
        fn TrafficLight_GetGreenTime(traffic_light: &TrafficLight) -> f32;
        fn TrafficLight_Freeze(traffic_light: &TrafficLight, freeze: bool);
        fn TrafficLight_IsFrozen(traffic_light: &TrafficLight) -> bool;

        // Traffic Light Actor interface methods
        fn TrafficLight_GetTypeId(traffic_light: &TrafficLight) -> String;
        fn TrafficLight_GetTransform(traffic_light: &TrafficLight) -> SimpleTransform;
        fn TrafficLight_SetTransform(traffic_light: &TrafficLight, transform: &SimpleTransform);
        fn TrafficLight_GetVelocity(traffic_light: &TrafficLight) -> SimpleVector3D;
        fn TrafficLight_GetAngularVelocity(traffic_light: &TrafficLight) -> SimpleVector3D;
        fn TrafficLight_GetAcceleration(traffic_light: &TrafficLight) -> SimpleVector3D;
        fn TrafficLight_IsAlive(traffic_light: &TrafficLight) -> bool;
        fn TrafficLight_Destroy(traffic_light: &TrafficLight) -> bool;
        fn TrafficLight_SetSimulatePhysics(traffic_light: &TrafficLight, enabled: bool);
        fn TrafficLight_AddImpulse(traffic_light: &TrafficLight, impulse: &SimpleVector3D);
        fn TrafficLight_AddForce(traffic_light: &TrafficLight, force: &SimpleVector3D);
        fn TrafficLight_AddTorque(traffic_light: &TrafficLight, torque: &SimpleVector3D);

        // Traffic Light advanced methods
        // NOTE: These FFI functions exist in C++ implementation but CXX bridge integration needs debugging
        // fn TrafficLight_GetAffectedLaneWaypoints(traffic_light: &TrafficLight) -> Vec<SimpleWaypointInfo>;
        // fn TrafficLight_GetPoleIndex(traffic_light: &TrafficLight) -> u32;
        // fn TrafficLight_GetGroupTrafficLights(traffic_light: &TrafficLight) -> Vec<SimpleTrafficLightInfo>;

        // Traffic Sign methods
        fn TrafficSign_GetSignId(traffic_sign: &TrafficSign) -> String;
        fn TrafficSign_GetTriggerVolume(traffic_sign: &TrafficSign) -> SimpleBoundingBox;

        // Traffic Manager methods
        fn TrafficManager_GetInstance(client: &Client, port: u16) -> SharedPtr<TrafficManager>;
        fn TrafficManager_RegisterVehicles(tm: &TrafficManager, vehicles: &[*const Vehicle]);
        fn TrafficManager_UnregisterVehicles(tm: &TrafficManager, vehicles: &[*const Vehicle]);
        fn TrafficManager_SetSynchronousMode(tm: &TrafficManager, mode: bool);
        fn TrafficManager_SynchronousTick(tm: &TrafficManager) -> bool;
        fn TrafficManager_SetSynchronousModeTimeout(tm: &TrafficManager, timeout_ms: f64);

        // Global traffic manager configuration
        fn TrafficManager_SetGlobalSpeedPercentage(tm: &TrafficManager, percentage: f32);
        fn TrafficManager_SetGlobalLaneOffset(tm: &TrafficManager, offset: f32);
        fn TrafficManager_SetGlobalDistanceToLeadingVehicle(tm: &TrafficManager, distance: f32);
        fn TrafficManager_SetRandomDeviceSeed(tm: &TrafficManager, seed: u64);
        fn TrafficManager_SetOSMMode(tm: &TrafficManager, mode: bool);

        // Vehicle-specific configuration
        fn TrafficManager_SetVehicleSpeedPercentage(
            tm: &TrafficManager,
            vehicle: &Vehicle,
            percentage: f32,
        );
        fn TrafficManager_SetVehicleDesiredSpeed(
            tm: &TrafficManager,
            vehicle: &Vehicle,
            speed: f32,
        );
        fn TrafficManager_SetVehicleLaneOffset(tm: &TrafficManager, vehicle: &Vehicle, offset: f32);
        fn TrafficManager_SetVehicleAutoLaneChange(
            tm: &TrafficManager,
            vehicle: &Vehicle,
            enable: bool,
        );
        fn TrafficManager_ForceVehicleLaneChange(
            tm: &TrafficManager,
            vehicle: &Vehicle,
            direction: bool,
        );
        fn TrafficManager_SetVehicleDistanceToLeadingVehicle(
            tm: &TrafficManager,
            vehicle: &Vehicle,
            distance: f32,
        );

        // Traffic rule compliance
        fn TrafficManager_SetVehiclePercentageRunningLight(
            tm: &TrafficManager,
            vehicle: &Vehicle,
            percentage: f32,
        );
        fn TrafficManager_SetVehiclePercentageRunningSign(
            tm: &TrafficManager,
            vehicle: &Vehicle,
            percentage: f32,
        );
        fn TrafficManager_SetVehiclePercentageIgnoreWalkers(
            tm: &TrafficManager,
            vehicle: &Vehicle,
            percentage: f32,
        );
        fn TrafficManager_SetVehiclePercentageIgnoreVehicles(
            tm: &TrafficManager,
            vehicle: &Vehicle,
            percentage: f32,
        );

        // Advanced features
        fn TrafficManager_SetHybridPhysicsMode(tm: &TrafficManager, mode: bool);
        fn TrafficManager_SetHybridPhysicsRadius(tm: &TrafficManager, radius: f32);
        fn TrafficManager_SetCollisionDetection(
            tm: &TrafficManager,
            vehicle1: &Vehicle,
            vehicle2: &Vehicle,
            detect: bool,
        );
        fn TrafficManager_SetVehicleUpdateLights(
            tm: &TrafficManager,
            vehicle: &Vehicle,
            update: bool,
        );

        // Lane behavior percentages
        fn TrafficManager_SetVehicleKeepRightPercentage(
            tm: &TrafficManager,
            vehicle: &Vehicle,
            percentage: f32,
        );
        fn TrafficManager_SetVehicleRandomLeftLaneChangePercentage(
            tm: &TrafficManager,
            vehicle: &Vehicle,
            percentage: f32,
        );
        fn TrafficManager_SetVehicleRandomRightLaneChangePercentage(
            tm: &TrafficManager,
            vehicle: &Vehicle,
            percentage: f32,
        );

        // Respawn configuration
        fn TrafficManager_SetRespawnDormantVehicles(tm: &TrafficManager, enable: bool);
        fn TrafficManager_SetRespawnBoundaries(
            tm: &TrafficManager,
            lower_bound: f32,
            upper_bound: f32,
        );
        fn TrafficManager_SetMaxBoundaries(tm: &TrafficManager, lower: f32, upper: f32);

        // Statistics and monitoring
        fn TrafficManager_GetConfig(tm: &TrafficManager) -> SimpleTrafficManagerConfig;
        fn TrafficManager_GetVehicleConfig(
            tm: &TrafficManager,
            vehicle: &Vehicle,
        ) -> SimpleTrafficManagerVehicleConfig;
        fn TrafficManager_GetStats(tm: &TrafficManager) -> SimpleTrafficManagerStats;
        fn TrafficManager_GetNextAction(
            tm: &TrafficManager,
            vehicle: &Vehicle,
        ) -> SimpleTrafficManagerAction;
        fn TrafficManager_IsVehicleRegistered(tm: &TrafficManager, vehicle: &Vehicle) -> bool;
        fn TrafficManager_GetPort(tm: &TrafficManager) -> u16;

        // Lifecycle management
        fn TrafficManager_Shutdown(tm: &TrafficManager);
        fn TrafficManager_Reset();
        fn TrafficManager_Release();

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

        // Map methods
        fn Map_GetName(map: &Map) -> String;
        fn Map_GetOpenDrive(map: &Map) -> String;
        fn Map_GetRecommendedSpawnPoints(map: &Map) -> Vec<SimpleTransform>;
        fn Map_GetWaypoint(
            map: &Map,
            location: &SimpleLocation,
            project_to_road: bool,
            lane_type: i32,
        ) -> SharedPtr<Waypoint>;
        fn Map_GetWaypointXODR(
            map: &Map,
            road_id: u32,
            lane_id: i32,
            s: f64,
        ) -> SharedPtr<Waypoint>;
        fn Map_GenerateWaypoints(map: &Map, distance: f64) -> Vec<SimpleWaypointInfo>;
        fn Map_GetGeoReference(map: &Map) -> SimpleGeoLocation;
        fn Map_GetAllCrosswalkZones(map: &Map) -> Vec<SimpleLocation>;
        fn Map_GetJunction(map: &Map, waypoint: &Waypoint) -> SharedPtr<Junction>;
        // Note: Due to CXX limitations, these return simplified structs instead of Vec<SharedPtr<T>>
        fn Map_GetTopology(map: &Map) -> Vec<SimpleWaypointInfo>;

        // Waypoint methods
        fn Waypoint_GetId(waypoint: &Waypoint) -> u64;
        fn Waypoint_GetRoadId(waypoint: &Waypoint) -> u32;
        fn Waypoint_GetSectionId(waypoint: &Waypoint) -> u32;
        fn Waypoint_GetLaneId(waypoint: &Waypoint) -> i32;
        fn Waypoint_GetDistance(waypoint: &Waypoint) -> f64;
        fn Waypoint_GetTransform(waypoint: &Waypoint) -> SimpleTransform;
        fn Waypoint_GetJunctionId(waypoint: &Waypoint) -> u32;
        fn Waypoint_IsJunction(waypoint: &Waypoint) -> bool;
        fn Waypoint_GetJunction(waypoint: &Waypoint) -> SharedPtr<Junction>;
        fn Waypoint_GetLaneWidth(waypoint: &Waypoint) -> f64;
        fn Waypoint_GetType(waypoint: &Waypoint) -> u32;
        fn Waypoint_GetNext(waypoint: &Waypoint, distance: f64) -> SharedPtr<Waypoint>;
        fn Waypoint_GetPrevious(waypoint: &Waypoint, distance: f64) -> SharedPtr<Waypoint>;
        fn Waypoint_GetRight(waypoint: &Waypoint) -> SharedPtr<Waypoint>;
        fn Waypoint_GetLeft(waypoint: &Waypoint) -> SharedPtr<Waypoint>;
        fn Waypoint_GetRightLaneMarking(waypoint: &Waypoint) -> SimpleLaneMarking;
        fn Waypoint_GetLeftLaneMarking(waypoint: &Waypoint) -> SimpleLaneMarking;
        fn Waypoint_GetLaneChange(waypoint: &Waypoint) -> u8;
        // Junction methods
        fn Junction_GetId(junction: &Junction) -> u32;
        fn Junction_GetBoundingBox(junction: &Junction) -> SimpleBoundingBox;

    }

    // Global namespace functions (primarily bridge utilities)
    unsafe extern "C++" {
        include!("carla_cxx_bridge.h");

        // Weather functions - use global namespace since they're bridge functions
        fn World_GetWeather(world: &World) -> SimpleWeatherParameters;
        fn World_SetWeather(world: &World, weather: &SimpleWeatherParameters);
        fn World_IsWeatherEnabled(world: &World) -> bool;

        // ROS2 integration functions (simplified sensor control only)
        fn Sensor_EnableForROS(sensor: &Sensor);
        fn Sensor_DisableForROS(sensor: &Sensor);
        fn Sensor_IsEnabledForROS(sensor: &Sensor) -> bool;
    }

    #[namespace = "carla::traffic_manager"]
    unsafe extern "C++" {
        include!("carla_cxx_bridge.h");

        // Traffic Manager type
        type TrafficManager;
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
    Actor_CastToSensor,
    Actor_CastToTrafficLight,
    Actor_CastToTrafficSign,
    // Actor casting functions
    Actor_CastToVehicle,
    Actor_CastToWalker,
    Actor_CastToWalkerAIController,
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
    Client_GenerateOpenDriveWorld,
    Client_GetAvailableMaps,
    Client_GetServerVersion,
    Client_GetTimeout,
    Client_GetWorld,
    Client_LoadWorld,
    Client_ReloadWorld,
    Client_ReplayFile,
    Client_SetReplayerIgnoreHero,
    Client_SetReplayerIgnoreSpectator,
    Client_SetReplayerTimeFactor,
    Client_SetTimeout,
    Client_ShowRecorderActorsBlocked,
    Client_ShowRecorderCollisions,
    Client_ShowRecorderFileInfo,
    Client_StartRecorder,
    Client_StopRecorder,
    Client_StopReplayer,
    Junction,
    Junction_GetBoundingBox,
    // Junction methods
    Junction_GetId,
    Location_Distance,
    Location_DistanceSquared,
    // Map and navigation types
    Map,
    Map_GenerateWaypoints,
    Map_GetAllCrosswalkZones,
    Map_GetGeoReference,
    Map_GetJunction,
    // Map methods
    Map_GetName,
    Map_GetOpenDrive,
    Map_GetRecommendedSpawnPoints,
    Map_GetTopology,
    Map_GetWaypoint,
    Map_GetWaypointXODR,
    Sensor,
    // Sensor methods
    Sensor_CastToActor,
    // Sensor data retrieval
    Sensor_GetImageDataBuffer,
    Sensor_GetLastCollisionData,
    Sensor_GetLastGNSSData,
    Sensor_GetLastIMUData,
    Sensor_GetLastImageData,
    Sensor_GetLastLaneInvasionData,
    Sensor_GetLastLiDARData,
    Sensor_GetLastRadarData,
    Sensor_HasNewData,
    Sensor_IsListening,
    Sensor_Listen,
    Sensor_Stop,
    // Control structures
    SimpleAckermannControl,
    SimpleActorId,
    SimpleActorList,
    SimpleBoundingBox,
    SimpleCollisionData,
    // Debug visualization structures
    SimpleColor,
    SimpleCrossedLaneMarking,
    SimpleDebugArrow,
    SimpleDebugBox,
    SimpleDebugLine,
    SimpleDebugPoint,
    SimpleDebugString,
    SimpleEpisodeSettings,
    SimpleGNSSData,
    SimpleGearPhysicsControl,
    SimpleGeoLocation,
    SimpleIMUData,
    SimpleImageData,
    SimpleJunction,
    SimpleLabelledPoint,
    SimpleLaneMarking,
    SimpleLiDARPoint,
    SimpleLocation,
    SimpleOptionalLabelledPoint,
    SimpleOptionalLocation,
    SimpleRadarDetection,
    SimpleRecorderCollision,
    SimpleRecorderInfo,
    SimpleRecorderPosition,
    SimpleRotation,
    SimpleTimestamp,
    SimpleTrafficLightInfo,
    SimpleTrafficLightState,
    SimpleTrafficManagerAction,
    SimpleTrafficManagerConfig,
    SimpleTrafficManagerStats,
    SimpleTrafficManagerVehicleConfig,
    SimpleTransform,
    SimpleVector2D,
    SimpleVector3D,
    SimpleVehicleControl,
    SimpleVehicleDoor,
    SimpleVehiclePhysicsControl,
    SimpleVehicleTelemetryData,
    SimpleWalkerControl,
    SimpleWalkerDestination,
    SimpleWaypointInfo,
    SimpleWeatherParameters,
    SimpleWheelPhysicsControl,
    TrafficLight,
    TrafficLight_AddForce,
    TrafficLight_AddImpulse,
    TrafficLight_AddTorque,
    TrafficLight_Destroy,
    TrafficLight_Freeze,
    TrafficLight_GetAcceleration,
    TrafficLight_GetAngularVelocity,
    TrafficLight_GetElapsedTime,
    TrafficLight_GetGreenTime,
    TrafficLight_GetRedTime,
    // Traffic Light methods
    TrafficLight_GetState,
    TrafficLight_GetTransform,
    // Traffic Light Actor interface methods
    TrafficLight_GetTypeId,
    TrafficLight_GetVelocity,
    TrafficLight_GetYellowTime,
    TrafficLight_IsAlive,
    TrafficLight_IsFrozen,
    TrafficLight_SetGreenTime,
    TrafficLight_SetRedTime,
    TrafficLight_SetSimulatePhysics,
    TrafficLight_SetState,
    TrafficLight_SetTransform,
    TrafficLight_SetYellowTime,
    // Traffic Manager type and methods
    TrafficManager,
    TrafficManager_ForceVehicleLaneChange,
    TrafficManager_GetConfig,
    TrafficManager_GetInstance,
    TrafficManager_GetNextAction,
    TrafficManager_GetPort,
    TrafficManager_GetStats,
    TrafficManager_GetVehicleConfig,
    TrafficManager_IsVehicleRegistered,
    TrafficManager_RegisterVehicles,
    TrafficManager_Release,
    TrafficManager_Reset,
    TrafficManager_SetCollisionDetection,
    TrafficManager_SetGlobalDistanceToLeadingVehicle,
    TrafficManager_SetGlobalLaneOffset,
    TrafficManager_SetGlobalSpeedPercentage,
    TrafficManager_SetHybridPhysicsMode,
    TrafficManager_SetHybridPhysicsRadius,
    TrafficManager_SetMaxBoundaries,
    TrafficManager_SetOSMMode,
    TrafficManager_SetRandomDeviceSeed,
    TrafficManager_SetRespawnBoundaries,
    TrafficManager_SetRespawnDormantVehicles,
    TrafficManager_SetSynchronousMode,
    TrafficManager_SetSynchronousModeTimeout,
    TrafficManager_SetVehicleAutoLaneChange,
    TrafficManager_SetVehicleDesiredSpeed,
    TrafficManager_SetVehicleDistanceToLeadingVehicle,
    TrafficManager_SetVehicleKeepRightPercentage,
    TrafficManager_SetVehicleLaneOffset,
    TrafficManager_SetVehiclePercentageIgnoreVehicles,
    TrafficManager_SetVehiclePercentageIgnoreWalkers,
    TrafficManager_SetVehiclePercentageRunningLight,
    TrafficManager_SetVehiclePercentageRunningSign,
    TrafficManager_SetVehicleRandomLeftLaneChangePercentage,
    TrafficManager_SetVehicleRandomRightLaneChangePercentage,
    TrafficManager_SetVehicleSpeedPercentage,
    TrafficManager_SetVehicleUpdateLights,
    TrafficManager_Shutdown,
    TrafficManager_SynchronousTick,
    // Traffic Sign type and methods
    TrafficSign,
    TrafficSign_GetSignId,
    TrafficSign_GetTriggerVolume,
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
    // Specific actor types
    Vehicle,
    Vehicle_AddForce,
    Vehicle_AddForceAtLocation,
    Vehicle_AddImpulse,
    Vehicle_AddImpulseAtLocation,
    Vehicle_AddTorque,
    // Vehicle methods
    Vehicle_ApplyAckermannControl,
    Vehicle_ApplyControl,
    Vehicle_ApplyPhysicsControl,
    Vehicle_CastToActor,
    Vehicle_CloseDoor,
    Vehicle_Destroy,
    Vehicle_GetAcceleration,
    Vehicle_GetAckermannControl,
    Vehicle_GetAngularVelocity,
    Vehicle_GetControl,
    Vehicle_GetDoorStates,
    Vehicle_GetEngineRpm,
    Vehicle_GetGearPhysicsControls,
    Vehicle_GetGearRatio,
    Vehicle_GetId,
    Vehicle_GetLightState,
    Vehicle_GetPhysicsControl,
    Vehicle_GetSpeed,
    Vehicle_GetSpeedLimit,
    Vehicle_GetTelemetryData,
    Vehicle_GetTireFriction,
    Vehicle_GetTransform,
    Vehicle_GetTypeId,
    Vehicle_GetVelocity,
    Vehicle_GetWheelPhysicsControls,
    Vehicle_GetWheelSteerAngle,
    Vehicle_IsAlive,
    Vehicle_IsDoorOpen,
    Vehicle_OpenDoor,
    Vehicle_SetAutopilot,
    Vehicle_SetGearPhysicsControls,
    Vehicle_SetLightState,
    Vehicle_SetSimulatePhysics,
    Vehicle_SetTransform,
    Vehicle_SetWheelPhysicsControls,
    Walker,
    WalkerAIController,
    WalkerAIController_GetRandomLocation,
    WalkerAIController_GoToLocation,
    WalkerAIController_HasValidDestination,
    WalkerAIController_SetMaxSpeed,
    WalkerAIController_Start,
    WalkerAIController_Stop,
    // Walker methods
    Walker_AddForce,
    Walker_AddImpulse,
    Walker_AddTorque,
    Walker_ApplyControl,
    Walker_BlendPose,
    Walker_CastToActor,
    Walker_Destroy,
    Walker_GetAcceleration,
    Walker_GetAngularVelocity,
    Walker_GetControl,
    Walker_GetPoseFromAnimation,
    Walker_GetSpeed,
    Walker_GetTransform,
    Walker_GetTypeId,
    Walker_GetVelocity,
    Walker_HidePose,
    Walker_IsAlive,
    Walker_SetSimulatePhysics,
    Walker_SetTransform,
    Walker_ShowPose,
    Waypoint,
    Waypoint_GetDistance,
    // Waypoint methods
    Waypoint_GetId,
    Waypoint_GetJunction,
    Waypoint_GetJunctionId,
    Waypoint_GetLaneChange,
    Waypoint_GetLaneId,
    Waypoint_GetLaneWidth,
    Waypoint_GetLeft,
    Waypoint_GetLeftLaneMarking,
    Waypoint_GetNext,
    Waypoint_GetPrevious,
    Waypoint_GetRight,
    Waypoint_GetRightLaneMarking,
    Waypoint_GetRoadId,
    Waypoint_GetSectionId,
    Waypoint_GetTransform,
    Waypoint_GetType,
    Waypoint_IsJunction,
    World,
    World_ApplyColorTextureToObject,
    World_ApplyFloatColorTextureToObject,
    World_ApplySettings,
    // World interaction functions
    World_CastRay,
    World_DrawDebugArrow,
    World_DrawDebugBox,
    World_DrawDebugLine,
    // Debug drawing functions
    World_DrawDebugPoint,
    World_DrawDebugString,
    World_EnableEnvironmentObjects,
    World_FreezeAllTrafficLights,
    World_GetActor,
    World_GetActors,
    World_GetActorsByIds,
    World_GetBlueprintLibrary,
    World_GetEnvironmentObjects,
    World_GetId,
    World_GetLevelBBs,
    World_GetMap,
    World_GetNamesOfAllObjects,
    World_GetRandomLocationFromNavigation,
    World_GetSettings,
    World_GetSnapshot,
    World_GetSpectator,
    World_GetTrafficLightsFromWaypoint,
    World_GetTrafficLightsInJunction,
    World_GetVehiclesLightStates,
    World_GetWeather,
    World_GroundProjection,
    World_IsWeatherEnabled,
    // Advanced world features
    World_LoadLevelLayer,
    World_ProjectPoint,
    World_ResetAllTrafficLights,
    World_SetPedestriansCrossFactor,
    World_SetPedestriansSeed,
    World_SetWeather,
    World_SpawnActor,
    World_Tick,
    World_TrySpawnActor,
    World_UnloadLevelLayer,
};
