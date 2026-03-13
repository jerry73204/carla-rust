# CARLA API Coverage Gap Analysis

This document compares the CARLA Python/C++ client API against the current
Rust `carla` crate bindings, listing missing features needed for 100% coverage.

The "Since" column shows the earliest CARLA version where the API is available.
All APIs are present in 0.9.14 unless noted otherwise.

## Legend

- **Have** = Already implemented in Rust crate
- **Missing** = Not yet implemented
- **Changed** = API changed in 0.10.0 vs 0.9.x (needs update)
- **0.9.16 only** = Present in 0.9.16 but removed in 0.10.0
- **Python-only** = Present in Python API but not in C++ headers

---

## 1. Client

### `Client`

| Method                            | Status      | Since  |
|-----------------------------------|-------------|--------|
| `connect()` / constructor         | Have        | 0.9.14 |
| `get_client_version()`            | Have        | 0.9.14 |
| `get_server_version()`            | Have        | 0.9.14 |
| `get_world()`                     | Have        | 0.9.14 |
| `set_timeout()`                   | Have        | 0.9.14 |
| `get_timeout()`                   | Have        | 0.9.14 |
| `load_world()`                    | Have        | 0.9.14 |
| `reload_world()`                  | Have        | 0.9.14 |
| `generate_opendrive_world()`      | Have        | 0.9.14 |
| `apply_batch_sync()`              | Have        | 0.9.14 |
| `apply_batch()`                   | Have        | 0.9.14 |
| `get_trafficmanager()`            | Have        | 0.9.14 |
| `get_available_maps()`            | Have        | 0.9.14 |
| `set_files_base_folder()`         | Have        | 0.9.14 |
| `get_required_files()`            | Have        | 0.9.14 |
| `request_file()`                  | Have        | 0.9.14 |
| `start_recorder()`                | Have        | 0.9.14 |
| `stop_recorder()`                 | Have        | 0.9.14 |
| `show_recorder_file_info()`       | Have        | 0.9.14 |
| `show_recorder_collisions()`      | Have        | 0.9.14 |
| `show_recorder_actors_blocked()`  | Have        | 0.9.14 |
| `replay_file()`                   | Have        | 0.9.14 |
| `stop_replayer()`                 | Have        | 0.9.14 |
| `set_replayer_time_factor()`      | Have        | 0.9.14 |
| `set_replayer_ignore_hero()`      | Have        | 0.9.14 |
| `load_world_if_different()`       | Have        | 0.9.15 |
| `set_replayer_ignore_spectator()` | Have        | 0.9.15 |

### `World`

| Method                                    | Status      | Since  |
|-------------------------------------------|-------------|--------|
| `id()`                                    | Have        | 0.9.14 |
| `get_map()`                               | Have        | 0.9.14 |
| `get_blueprint_library()`                 | Have        | 0.9.14 |
| `get_spectator()`                         | Have        | 0.9.14 |
| `get_settings()` / `apply_settings()`     | Have        | 0.9.14 |
| `get_weather()` / `set_weather()`         | Have        | 0.9.14 |
| `get_snapshot()`                          | Have        | 0.9.14 |
| `get_actors()`                            | Have        | 0.9.14 |
| `spawn_actor()` / `try_spawn_actor()`     | Have        | 0.9.14 |
| `tick()` / `wait_for_tick()`              | Have        | 0.9.14 |
| `get_traffic_lights_from_waypoint()`      | Have        | 0.9.14 |
| `debug` (DebugHelper)                     | Have        | 0.9.14 |
| `get_light_manager()`                     | Have        | 0.9.14 |
| `load_map_layer()` / `unload_map_layer()` | **Missing** | 0.9.14 |
| `get_vehicles_light_states()`             | **Missing** | 0.9.14 |
| `get_random_location_from_navigation()`   | **Missing** | 0.9.14 |
| `on_tick()` / `remove_on_tick()`          | **Missing** | 0.9.14 |
| `set_pedestrians_cross_factor()`          | **Missing** | 0.9.14 |
| `set_pedestrians_seed()`                  | **Missing** | 0.9.14 |
| `get_traffic_sign(landmark)`              | **Missing** | 0.9.14 |
| `get_traffic_light(landmark)`             | **Missing** | 0.9.14 |
| `get_traffic_light_from_opendrive_id()`   | **Missing** | 0.9.14 |
| `get_traffic_lights_in_junction()`        | **Missing** | 0.9.14 |
| `reset_all_traffic_lights()`              | **Missing** | 0.9.14 |
| `freeze_all_traffic_lights()`             | **Missing** | 0.9.14 |
| `get_level_bbs()`                         | **Missing** | 0.9.14 |
| `get_environment_objects()`               | **Missing** | 0.9.14 |
| `enable_environment_objects()`            | **Missing** | 0.9.14 |
| `cast_ray()`                              | **Missing** | 0.9.14 |
| `project_point()`                         | **Missing** | 0.9.14 |
| `ground_projection()`                     | **Missing** | 0.9.14 |
| `get_names_of_all_objects()`              | **Missing** | 0.9.14 |
| `apply_color_texture_to_object()`         | **Missing** | 0.9.14 |
| `apply_float_color_texture_to_object()`   | **Missing** | 0.9.14 |
| `apply_textures_to_object()`              | **Missing** | 0.9.14 |
| `get_actor(id)` (single actor lookup)     | **Missing** | 0.9.14 |
| `is_weather_enabled()`                    | **Missing** | 0.10.0 |

### `EpisodeSettings`

| Field                        | Status      | Since  |
|------------------------------|-------------|--------|
| `synchronous_mode`           | Have        | 0.9.14 |
| `no_rendering_mode`          | Have        | 0.9.14 |
| `fixed_delta_seconds`        | Have        | 0.9.14 |
| `substepping`                | Have        | 0.9.14 |
| `max_substep_delta_time`     | Have        | 0.9.14 |
| `max_substeps`               | Have        | 0.9.14 |
| `max_actor_shapes`           | Have        | 0.9.14 |
| `deterministic_ragdolls`     | Have        | 0.9.14 |
| `enable_environment_objects` | Have        | 0.9.14 |
| `max_culling_distance`       | **Missing** | 0.9.14 |
| `tile_stream_distance`       | **Missing** | 0.9.14 |
| `actor_active_distance`      | **Missing** | 0.9.14 |
| `spectator_as_ego`           | **Missing** | 0.9.15 |

---

## 2. Actor

### `Actor` (base)

| Method                                                 | Status      | Since  |
|--------------------------------------------------------|-------------|--------|
| `id()`                                                 | Have        | 0.9.14 |
| `type_id()`                                            | Have        | 0.9.14 |
| `parent()`                                             | Have        | 0.9.14 |
| `semantic_tags()`                                      | Have        | 0.9.14 |
| `attributes()`                                         | Have        | 0.9.14 |
| `world()`                                              | Have        | 0.9.14 |
| `location()` / `transform()`                           | Have        | 0.9.14 |
| `velocity()` / `angular_velocity()` / `acceleration()` | Have        | 0.9.14 |
| `set_transform()` / `set_velocity()`                   | Have        | 0.9.14 |
| `add_impulse()` / `add_force()`                        | Have        | 0.9.14 |
| `set_simulate_physics()`                               | Have        | 0.9.14 |
| `set_enable_gravity()`                                 | Have        | 0.9.14 |
| `set_actor_dead()`                                     | Have        | 0.9.14 |
| `is_alive()`                                           | Have        | 0.9.14 |
| `set_location()` (without rotation)                    | Have        | 0.9.14 |
| `set_target_velocity()`                                | Have        | 0.9.14 |
| `set_target_angular_velocity()`                        | Have        | 0.9.14 |
| `enable_constant_velocity()`                           | Have        | 0.9.14 |
| `disable_constant_velocity()`                          | Have        | 0.9.14 |
| `add_impulse(impulse, location)` (at point)            | Have        | 0.9.14 |
| `add_force(force, location)` (at point)                | Have        | 0.9.14 |
| `add_angular_impulse()`                                | Have        | 0.9.14 |
| `add_torque()`                                         | Have        | 0.9.14 |
| `destroy()`                                            | Have        | 0.9.14 |
| `get_bounding_box()`                                   | Have        | 0.9.14 |
| `get_actor_state()`                                    | Have        | 0.9.14 |
| `is_dormant()` / `is_active()`                         | Have        | 0.9.14 |
| `set_collisions()`                                     | Have        | 0.9.15 |
| `get_actor_name()`                                     | Have        | 0.10.0 |
| `get_actor_class_name()`                               | Have        | 0.10.0 |
| `apply_texture()`                                      | **Missing** | 0.10.0 |

### `Vehicle`

| Method                                                    | Status      | Since  |
|-----------------------------------------------------------|-------------|--------|
| `apply_control()` / `control()`                           | Have        | 0.9.14 |
| `apply_ackermann_control()` / `ackermann_control()`       | Have        | 0.9.14 |
| `apply_physics_control()` / `physics_control()`           | Have        | 0.9.14 |
| `set_autopilot()`                                         | Have        | 0.9.14 |
| `show_debug_telemetry()`                                  | Have        | 0.9.14 |
| `get_speed_limit()`                                       | Have        | 0.9.14 |
| `get_traffic_light_state()`                               | Have        | 0.9.14 |
| `is_at_traffic_light()`                                   | Have        | 0.9.14 |
| `set_light_state()` / `light_state()`                     | Have        | 0.9.14 |
| `open_door()` / `close_door()`                            | Have        | 0.9.14 |
| `enable_carsim()`                                         | Have        | 0.9.14 |
| `get_failure_state()`                                     | Have        | 0.9.14 |
| `set_wheel_steer_direction()` / `get_wheel_steer_angle()` | Have        | 0.9.14 |
| `get_traffic_light()` (returns TrafficLight)              | **Missing** | 0.9.14 |
| `apply_ackermann_controller_settings()`                   | **Missing** | 0.9.14 |
| `get_ackermann_controller_settings()`                     | **Missing** | 0.9.14 |
| `use_carsim_road()`                                       | **Missing** | 0.9.14 |
| `enable_chrono_physics()`                                 | **Missing** | 0.9.14 |

### `Walker`

| Method                                         | Status      | Since  |
|------------------------------------------------|-------------|--------|
| `apply_control()` / `control()`                | Have        | 0.9.14 |
| `set_bones()` / `get_bones_transform()`        | Have        | 0.9.14 |
| `blend_pose()` / `show_pose()` / `hide_pose()` | Have        | 0.9.14 |
| `get_pose_from_animation()`                    | **Missing** | 0.9.14 |

### `WalkerAIController`

All methods implemented. 100% coverage.

### `TrafficLight`

| Method                                                      | Status      | Since  |
|-------------------------------------------------------------|-------------|--------|
| `state()` / `set_state()`                                   | Have        | 0.9.14 |
| `green_time()` / `yellow_time()` / `red_time()`             | Have        | 0.9.14 |
| `elapsed_time()`                                            | Have        | 0.9.14 |
| `freeze()` / `is_frozen()`                                  | Have        | 0.9.14 |
| `get_group_traffic_lights()`                                | Have        | 0.9.14 |
| `get_affected_lane_waypoints()`                             | Have        | 0.9.14 |
| `get_opendrive_id()`                                        | Have        | 0.9.14 |
| `sign_id()` / `trigger_volume()`                            | Have        | 0.9.14 |
| `set_green_time()` / `set_yellow_time()` / `set_red_time()` | **Missing** | 0.9.14 |
| `get_pole_index()`                                          | **Missing** | 0.9.14 |
| `reset_group()`                                             | **Missing** | 0.9.14 |
| `get_light_boxes()`                                         | **Missing** | 0.9.14 |
| `get_stop_waypoints()`                                      | **Missing** | 0.9.14 |

### `Sensor` / `ServerSideSensor`

| Method                   | Status      | Since  |
|--------------------------|-------------|--------|
| `listen()`               | Have        | 0.9.14 |
| `stop()`                 | Have        | 0.9.14 |
| `is_listening()`         | Have        | 0.9.14 |
| `listen_to_gbuffer()`    | **Missing** | 0.9.14 |
| `is_listening_gbuffer()` | **Missing** | 0.9.14 |
| `stop_gbuffer()`         | **Missing** | 0.9.14 |
| `enable_for_ros()`       | **Missing** | 0.9.15 |
| `disable_for_ros()`      | **Missing** | 0.9.15 |
| `is_enabled_for_ros()`   | **Missing** | 0.9.15 |

---

## 3. Map & Navigation

### `Map`

| Method                                               | Status      | Since       |
|------------------------------------------------------|-------------|-------------|
| `name()`                                             | Have        | 0.9.14      |
| `to_opendrive()`                                     | Have        | 0.9.14      |
| `recommended_spawn_points()`                         | Have        | 0.9.14      |
| `waypoint_at()`                                      | Have        | 0.9.14      |
| `generate_waypoints()`                               | Have        | 0.9.14      |
| `all_landmarks()`                                    | Have        | 0.9.14      |
| `landmarks_of_type()`                                | Have        | 0.9.14      |
| `get_topology()`                                     | Have        | 0.9.14      |
| `get_landmarks_from_waypoint()`                      | Have        | 0.9.14      |
| `get_waypoint_xodr()`                                | **Missing** | 0.9.14      |
| `get_geo_reference()` / `transform_to_geolocation()` | **Missing** | 0.9.14      |
| `get_crosswalks()`                                   | **Missing** | 0.9.14      |
| `get_all_landmarks_from_id()`                        | **Missing** | 0.9.14      |
| `get_landmark_group()`                               | **Missing** | 0.9.14      |
| `cook_in_memory_map()`                               | **Missing** | 0.9.14      |
| `save_to_disk()`                                     | **Missing** | Python-only |

### `Waypoint`

| Method                                                      | Status      | Since  |
|-------------------------------------------------------------|-------------|--------|
| `id()` / `road_id()` / `section_id()` / `lane_id()` / `s()` | Have        | 0.9.14 |
| `lane_width()` / `lane_type()`                              | Have        | 0.9.14 |
| `is_junction()`                                             | Have        | 0.9.14 |
| `right_lane_marking()` / `left_lane_marking()`              | Have        | 0.9.14 |
| `transform()`                                               | Have        | 0.9.14 |
| `next()` / `previous()`                                     | Have        | 0.9.14 |
| `get_right_lane()` / `get_left_lane()`                      | Have        | 0.9.14 |
| `get_junction()`                                            | Have        | 0.9.14 |
| `lane_change()`                                             | Have        | 0.9.14 |
| `all_landmarks_in_distance()`                               | Have        | 0.9.14 |
| `get_landmarks_of_type_in_distance()`                       | Have        | 0.9.14 |
| `next_until_lane_end()`                                     | **Missing** | 0.9.14 |
| `previous_until_lane_start()`                               | **Missing** | 0.9.14 |
| `junction_id` (property)                                    | **Missing** | 0.9.14 |

### `Landmark`

| Method                               | Status      | Since  |
|--------------------------------------|-------------|--------|
| `waypoint()` / `transform()`         | Have        | 0.9.14 |
| `road_id()` / `sign_id()` / `name()` | Have        | 0.9.14 |
| `type()` / `value()` / `unit()`      | Have        | 0.9.14 |
| `height()` / `width()`               | Have        | 0.9.14 |
| `s()` / `t()` / `z_offset()`         | Have        | 0.9.14 |
| `is_dynamic()`                       | Have        | 0.9.14 |
| `country()`                          | **Missing** | 0.9.14 |
| `sub_type()`                         | **Missing** | 0.9.14 |
| `text()`                             | **Missing** | 0.9.14 |
| `h_offset()`                         | **Missing** | 0.9.14 |
| `pitch()` / `roll()`                 | **Missing** | 0.9.14 |
| `orientation()`                      | **Missing** | 0.9.14 |
| `get_lane_validities()`              | **Missing** | 0.9.14 |

### `Junction`

All methods implemented. 100% coverage.

---

## 4. Geometry

### `Transform`

| Method                         | Status      | Since  |
|--------------------------------|-------------|--------|
| `location` / `rotation` fields | Have        | 0.9.14 |
| Composition (`Mul`)            | Have        | 0.9.14 |
| nalgebra conversions           | Have        | 0.9.14 |
| `get_forward_vector()`         | **Missing** | 0.9.14 |
| `get_right_vector()`           | **Missing** | 0.9.14 |
| `get_up_vector()`              | **Missing** | 0.9.14 |
| `transform_point()`            | **Missing** | 0.9.14 |
| `transform_vector()`           | **Missing** | 0.9.14 |
| `inverse_transform_point()`    | **Missing** | 0.9.14 |
| `get_matrix()`                 | **Missing** | 0.9.14 |
| `get_inverse_matrix()`         | **Missing** | 0.9.14 |

### `Rotation`

| Method | Status | Since |
|--------|--------|-------|
| `pitch` / `yaw` / `roll` fields | Have | 0.9.14 |
| `forward()` / `right()` / `up()` | Have | 0.9.14 |
| `get_normalized()` | **Missing** | 0.10.0 |

### `Vector3D`

| Method                                             | Status      | Since  |
|----------------------------------------------------|-------------|--------|
| Fields, arithmetic, `length()`, `dot()`, `cross()` | Have        | 0.9.14 |
| `normalize()`                                      | Have        | 0.9.14 |
| `abs()`                                            | **Missing** | 0.9.14 |
| `make_unit_vector()` / `make_safe_unit_vector()`   | **Missing** | 0.9.14 |
| `squared_length_2d()` / `length_2d()`              | **Missing** | 0.9.15 |
| `distance()` / `distance_squared()` (on Location)  | **Missing** | 0.9.15 |
| `distance_2d()` / `distance_squared_2d()` (Math)   | **Missing** | 0.9.14 |
| `get_vector_angle()` (Math)                        | **Missing** | 0.9.14 |

### `BoundingBox`

All methods implemented. 100% coverage.

---

## 5. RPC Types

### `VehiclePhysicsControl` — **Changed in 0.10.0**

The 0.10.0 `VehiclePhysicsControl` has a completely different layout from 0.9.x.
The 0.9.x layout is fully bound; the 0.10.0 layout uses a compatibility shim.

| 0.10.0 Field                | Status      | Notes                                  |
|-----------------------------|-------------|----------------------------------------|
| `torque_curve`              | Have        | Same in both                           |
| `max_torque`                | **Missing** | New in 0.10.0                          |
| `max_rpm`                   | Have        | Same in both                           |
| `idle_rpm`                  | **Missing** | New in 0.10.0                          |
| `brake_effect`              | **Missing** | New in 0.10.0                          |
| `rev_up_moi`                | **Missing** | New in 0.10.0 (replaces `moi`)         |
| `rev_down_rate`             | **Missing** | New in 0.10.0                          |
| `differential_type`         | **Missing** | New in 0.10.0                          |
| `front_rear_split`          | **Missing** | New in 0.10.0                          |
| `use_automatic_gears`       | **Missing** | New in 0.10.0 (was `use_gear_autobox`) |
| `gear_change_time`          | Have        | Was `gear_switch_time`                 |
| `final_ratio`               | Have        | Same in both                           |
| `forward_gear_ratios`       | **Missing** | New in 0.10.0 (was `forward_gears`)    |
| `reverse_gear_ratios`       | **Missing** | New in 0.10.0                          |
| `change_up_rpm`             | **Missing** | New in 0.10.0                          |
| `change_down_rpm`           | **Missing** | New in 0.10.0                          |
| `transmission_efficiency`   | **Missing** | New in 0.10.0                          |
| `mass`                      | Have        | Same in both                           |
| `drag_coefficient`          | Have        | Same in both                           |
| `center_of_mass`            | Have        | Same in both                           |
| `chassis_width`             | **Missing** | New in 0.10.0                          |
| `chassis_height`            | **Missing** | New in 0.10.0                          |
| `downforce_coefficient`     | **Missing** | New in 0.10.0                          |
| `drag_area`                 | **Missing** | New in 0.10.0                          |
| `inertia_tensor_scale`      | **Missing** | New in 0.10.0                          |
| `sleep_threshold`           | **Missing** | New in 0.10.0                          |
| `sleep_slope_limit`         | **Missing** | New in 0.10.0                          |
| `steering_curve`            | Have        | Same in both                           |
| `wheels`                    | Have        | Layout changed in 0.10.0               |
| `use_sweep_wheel_collision` | Have        | Same in both                           |

**Removed in 0.10.0:** `moi`, `damping_rate_full_throttle`,
`damping_rate_zero_throttle_clutch_engaged`,
`damping_rate_zero_throttle_clutch_disengaged`, `clutch_strength`,
`forward_gears` (GearPhysicsControl vec), `GearPhysicsControl` type

### `WheelPhysicsControl` — **Changed in 0.10.0**

0.10.0 has a vastly expanded `WheelPhysicsControl` with ~40 fields vs ~7 in 0.9.x.
Currently we use a compatibility shim providing 0.9.x layout for all versions.

### `VehicleTelemetryData` / `WheelTelemetryData`

| Type                   | Status | Since                           |
|------------------------|--------|---------------------------------|
| `VehicleTelemetryData` | Have   | 0.9.16 only (removed in 0.10.0) |
| `WheelTelemetryData`   | Have   | 0.9.16 only (removed in 0.10.0) |

### `WeatherParameters`

| Feature                                         | Status      | Since  |
|-------------------------------------------------|-------------|--------|
| All 14 fields                                   | Have        | 0.9.14 |
| Static presets (23: ClearNoon, DustStorm, etc.) | **Missing** | 0.9.14 |

### `OpendriveGenerationParameters`

| Field                                                     | Status                    | Since  |
|-----------------------------------------------------------|---------------------------|--------|
| `vertex_distance`, `max_road_length`, `wall_height`, etc. | **Missing** (entire type) | 0.9.14 |

### `Command` variants

| Variant                        | Status      | Since  |
|--------------------------------|-------------|--------|
| `SpawnActor`                   | Have        | 0.9.14 |
| `DestroyActor`                 | Have        | 0.9.14 |
| `ApplyVehicleControl`          | Have        | 0.9.14 |
| `ApplyVehicleAckermannControl` | Have        | 0.9.14 |
| `ApplyWalkerControl`           | Have        | 0.9.14 |
| `ApplyVehiclePhysicsControl`   | Have        | 0.9.14 |
| `ApplyTransform`               | Have        | 0.9.14 |
| `SetAutopilot`                 | Have        | 0.9.14 |
| `SetVehicleLightState`         | Have        | 0.9.14 |
| `SetTrafficLightState`         | Have        | 0.9.14 |
| `ApplyWalkerState`             | **Missing** | 0.9.14 |
| `ApplyTargetVelocity`          | **Missing** | 0.9.14 |
| `ApplyTargetAngularVelocity`   | **Missing** | 0.9.14 |
| `ApplyImpulse`                 | **Missing** | 0.9.14 |
| `ApplyForce`                   | **Missing** | 0.9.14 |
| `ApplyAngularImpulse`          | **Missing** | 0.9.14 |
| `ApplyTorque`                  | **Missing** | 0.9.14 |
| `SetSimulatePhysics`           | **Missing** | 0.9.14 |
| `SetEnableGravity`             | **Missing** | 0.9.14 |
| `ShowDebugTelemetry`           | **Missing** | 0.9.14 |
| `ConsoleCommand`               | **Missing** | 0.9.14 |
| `SpawnActor.then()` chaining   | **Missing** | 0.9.14 |

### Other missing RPC types

These types are used in public method signatures of Client, World, Actor, etc.

| Type                                               | Status      | Since  | Used by                                          |
|----------------------------------------------------|-------------|--------|--------------------------------------------------|
| `ActorState` enum (`Invalid`, `Active`, `Dormant`) | Have        | 0.9.14 | `Actor::GetActorState()`                         |
| `FloatColor` struct                                | **Missing** | 0.9.14 | `TextureFloatColor` / texture methods            |
| `TextureColor` / `TextureFloatColor`               | **Missing** | 0.9.14 | `World::ApplyColorTextureToObject()` etc.        |
| `MaterialParameter` enum                           | **Missing** | 0.9.14 | `World::ApplyColorTextureToObject()` parameter   |
| `LabelledPoint` struct                             | **Missing** | 0.9.14 | `World::CastRay()`, `ProjectPoint()` return type |
| `CityObjectLabel` / `ObjectLabel` enum             | **Missing** | 0.9.14 | `LabelledPoint` field, `World::GetLevelBBs()`    |
| `MapLayer` enum (load/unload support)              | Partial     | 0.9.14 | `World::LoadLevelLayer()` parameter              |
| `SignalOrientation` enum                           | **Missing** | 0.9.14 | `Landmark::GetOrientation()` return type         |
| `ActorAttributeType` enum                          | **Missing** | 0.9.14 | `ActorAttributeValueAccess::GetType()`           |
| `AckermannControllerSettings` struct               | **Missing** | 0.9.14 | `Vehicle::GetAckermannControllerSettings()`      |

---

## 6. Sensor Data

| Feature                                          | Status      | Since       |
|--------------------------------------------------|-------------|-------------|
| `frame()` / `timestamp()` / `sensor_transform()` | Have        | 0.9.14      |
| `Image` (width, height, fov, raw_data)           | Have        | 0.9.14      |
| `OpticalFlowImage`                               | Have        | 0.9.14      |
| `LidarMeasurement` (point count, detections)     | Have        | 0.9.14      |
| `SemanticLidarMeasurement`                       | Have        | 0.9.14      |
| `RadarMeasurement` / `RadarDetection`            | Have        | 0.9.14      |
| `CollisionEvent`                                 | Have        | 0.9.14      |
| `LaneInvasionEvent`                              | Have        | 0.9.14      |
| `ObstacleDetectionEvent`                         | Have        | 0.9.14      |
| `DVSEventArray` / `DVSEvent`                     | Have        | 0.9.14      |
| `GnssMeasurement`                                | Have        | 0.9.14      |
| `ImuMeasurement`                                 | Have        | 0.9.14      |
| `NormalsImage`                                   | **Missing** | 0.9.14      |
| `horizontal_angle` / `channels` (LiDAR props)    | **Missing** | 0.9.14      |
| `Image::save_to_disk()`                          | **Missing** | Python-only |
| `Image::convert()` (ColorConverter)              | **Missing** | Python-only |

---

## 7. Traffic Manager

| Method                                                      | Status      | Since                                         |
|-------------------------------------------------------------|-------------|-----------------------------------------------|
| `get_port()`                                                | Have        | 0.9.14                                        |
| `register_vehicles()` / `unregister_vehicles()`             | Have        | 0.9.14                                        |
| `set_desired_speed()`                                       | Have        | 0.9.14                                        |
| `set_global_percentage_speed_difference()`                  | Have        | 0.9.14                                        |
| `set_percentage_speed_difference()`                         | Have        | 0.9.14                                        |
| `set_synchronous_mode()`                                    | Have        | 0.9.14                                        |
| `set_hybrid_physics_mode()` / `set_hybrid_physics_radius()` | Have        | 0.9.14                                        |
| `set_lane_offset()`                                         | Have        | 0.9.14                                        |
| `set_auto_lane_change()`                                    | Have        | 0.9.14                                        |
| `set_distance_to_leading_vehicle()`                         | Have        | 0.9.14                                        |
| `set_collision_detection()`                                 | Have        | 0.9.14                                        |
| `ignore_lights_percentage()`                                | Have        | 0.9.14                                        |
| `ignore_vehicles_percentage()`                              | Have        | 0.9.14                                        |
| `ignore_walkers_percentage()`                               | Have        | 0.9.14                                        |
| `shut_down()`                                               | Have        | 0.9.14                                        |
| `global_lane_offset()`                                      | **Missing** | 0.9.14                                        |
| `update_vehicle_lights()`                                   | **Missing** | 0.9.14                                        |
| `force_lane_change()`                                       | **Missing** | 0.9.14                                        |
| `ignore_signs_percentage()`                                 | **Missing** | 0.9.14                                        |
| `set_global_distance_to_leading_vehicle()`                  | **Missing** | 0.9.14                                        |
| `random_left_lanechange_percentage()`                       | **Missing** | 0.9.14                                        |
| `random_right_lanechange_percentage()`                      | **Missing** | 0.9.14                                        |
| `set_random_device_seed()`                                  | **Missing** | 0.9.14                                        |
| `set_osm_mode()`                                            | **Missing** | 0.9.14                                        |
| `set_path()` / `set_route()`                                | **Missing** | 0.9.14                                        |
| `set_respawn_dormant_vehicles()`                            | **Missing** | 0.9.14                                        |
| `set_boundaries_respawn_dormant_vehicles()`                 | **Missing** | 0.9.14                                        |
| `get_next_action()` / `get_all_actions()`                   | **Missing** | 0.9.14                                        |
| `synchronous_tick()`                                        | **Missing** | 0.9.14                                        |
| `keep_right_rule_percentage()`                              | **Missing** | 0.9.14 (absent in 0.9.16, restored in 0.10.0) |

---

## 8. Blueprint

| Method                                                              | Status      | Since  |
|---------------------------------------------------------------------|-------------|--------|
| `filter()`                                                          | Have        | 0.9.14 |
| `find()`                                                            | Have        | 0.9.14 |
| `len()` / `iter()` / `get()`                                        | Have        | 0.9.14 |
| `id()` / `type()` / `is_modifiable()` / `value()`                   | Have        | 0.9.14 |
| `recommended_values()`                                              | Have        | 0.9.14 |
| `filter_by_attribute()`                                             | **Missing** | 0.9.15 |
| `as_bool()` / `as_int()` / `as_float()` / `as_str()` / `as_color()` | **Missing** | 0.9.14 |

---

## 9. Lighting

| Method                                          | Status      | Since  |
|-------------------------------------------------|-------------|--------|
| Basic light operations                          | Have        | 0.9.14 |
| `set_daylight_cycle()`                          | Have        | 0.9.14 |
| Batch `SetColor/SetIntensity` (multiple values) | **Missing** | 0.9.14 |
| `GetAllLights(type)` filter by group            | **Missing** | 0.9.14 |
| `GetTurnedOnLights()` / `GetTurnedOffLights()`  | **Missing** | 0.9.14 |

---

## 10. Debug Helper

All methods implemented. 100% coverage.

---

## Version-Gated API Summary

Most missing APIs have been present since 0.9.14. Only these require `#[cfg]` guards:

| API                                                                       | Introduced | Removed                  |
|---------------------------------------------------------------------------|------------|--------------------------|
| `Client::load_world_if_different()`                                       | 0.9.15     | —                        |
| `Client::set_replayer_ignore_spectator()`                                 | 0.9.15     | —                        |
| `Actor::set_collisions()`                                                 | 0.9.15     | —                        |
| `Sensor::enable_for_ros()` / `disable_for_ros()` / `is_enabled_for_ros()` | 0.9.15     | —                        |
| `EpisodeSettings::spectator_as_ego`                                       | 0.9.15     | —                        |
| `BlueprintLibrary::filter_by_attribute()`                                 | 0.9.15     | —                        |
| `Vector3D::squared_length_2d()` / `length_2d()`                           | 0.9.15     | —                        |
| `Location::distance_squared()`                                            | 0.9.15     | —                        |
| `VehicleTelemetryData` / `WheelTelemetryData`                             | 0.9.16     | 0.10.0                   |
| `Rotation::get_normalized()`                                              | 0.10.0     | —                        |
| `Actor::get_actor_name()` / `get_actor_class_name()`                      | 0.10.0     | —                        |
| `Actor::apply_texture()`                                                  | 0.10.0     | —                        |
| `World::is_weather_enabled()`                                             | 0.10.0     | —                        |
| `VehiclePhysicsControl` (new layout)                                      | 0.10.0     | —                        |
| `WheelPhysicsControl` (new layout)                                        | 0.10.0     | —                        |
| `GearPhysicsControl` type                                                 | 0.9.14     | 0.10.0                   |
| `TrafficManager::keep_right_rule_percentage()`                            | 0.9.14     | 0.9.16 (restored 0.10.0) |
