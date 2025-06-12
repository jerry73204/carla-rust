/*!
 * Temporary stub types for C API functions and types that are not yet implemented.
 * These will be removed when the actual C API is integrated.
 */

// TODO: Remove all these stubs when actual C API types are available

// Sensor data types
#[repr(C)]
pub struct carla_collision_event_data_t {
    pub _placeholder: u8,
}

#[repr(C)]
pub struct carla_obstacle_detection_event_data_t {
    pub _placeholder: u8,
}

#[repr(C)]
pub struct carla_lane_invasion_event_data_t {
    pub _placeholder: u8,
}

// Walker types
pub type carla_walker_control_t = carla_sys::carla_vehicle_control_t;

// Actor types
pub type carla_attachment_type_t = u32;

// OpenDRIVE types
#[repr(C)]
pub struct carla_opendrive_generator_t {
    pub _placeholder: u8,
}

#[repr(C)]
pub struct carla_opendrive_waypoint_t {
    pub _placeholder: u8,
}

#[repr(C)]
pub struct carla_opendrive_generation_settings_t {
    pub _placeholder: u8,
}

#[repr(C)]
pub struct carla_opendrive_road_info_t {
    pub _placeholder: u8,
}

// ROS2 types
#[repr(C)]
pub struct carla_ros2_node_t {
    pub _placeholder: u8,
}

#[repr(C)]
pub struct carla_ros2_publisher_t {
    pub _placeholder: u8,
}

#[repr(C)]
pub struct carla_ros2_subscriber_t {
    pub _placeholder: u8,
}

#[repr(C)]
pub struct carla_ros2_message_t {
    pub _placeholder: u8,
}

// Stub functions that do nothing or return defaults
pub unsafe extern "C" fn carla_walker_apply_control(
    _actor: *mut carla_sys::carla_actor_t,
    _control: *const carla_walker_control_t,
) -> carla_sys::carla_error_t {
    carla_sys::carla_error_t_CARLA_ERROR_NONE
}

pub unsafe extern "C" fn carla_actor_attach_to(
    _actor: *mut carla_sys::carla_actor_t,
    _other: *mut carla_sys::carla_actor_t,
    _attachment_type: carla_attachment_type_t,
) -> carla_sys::carla_error_t {
    carla_sys::carla_error_t_CARLA_ERROR_NONE
}

pub unsafe extern "C" fn carla_collision_event_get_actor(
    _event: *mut carla_collision_event_data_t,
) -> *mut carla_sys::carla_actor_t {
    std::ptr::null_mut()
}

pub unsafe extern "C" fn carla_collision_event_get_other_actor(
    _event: *mut carla_collision_event_data_t,
) -> *mut carla_sys::carla_actor_t {
    std::ptr::null_mut()
}

pub unsafe extern "C" fn carla_collision_event_get_normal_impulse(
    _event: *mut carla_collision_event_data_t,
) -> carla_sys::carla_vector3d_t {
    carla_sys::carla_vector3d_t {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    }
}

pub unsafe extern "C" fn carla_gnss_data_get_longitude(
    _data: *mut carla_sys::carla_gnss_data_t,
) -> f64 {
    0.0
}

pub unsafe extern "C" fn carla_gnss_data_get_latitude(
    _data: *mut carla_sys::carla_gnss_data_t,
) -> f64 {
    0.0
}

pub unsafe extern "C" fn carla_gnss_data_get_altitude(
    _data: *mut carla_sys::carla_gnss_data_t,
) -> f64 {
    0.0
}

pub unsafe extern "C" fn carla_obstacle_detection_event_get_actor(
    _event: *mut carla_obstacle_detection_event_data_t,
) -> *mut carla_sys::carla_actor_t {
    std::ptr::null_mut()
}

pub unsafe extern "C" fn carla_obstacle_detection_event_get_other_actor(
    _event: *mut carla_obstacle_detection_event_data_t,
) -> *mut carla_sys::carla_actor_t {
    std::ptr::null_mut()
}

pub unsafe extern "C" fn carla_obstacle_detection_event_get_distance(
    _event: *mut carla_obstacle_detection_event_data_t,
) -> f64 {
    0.0
}

pub unsafe extern "C" fn carla_lane_invasion_event_get_actor(
    _event: *mut carla_lane_invasion_event_data_t,
) -> *mut carla_sys::carla_actor_t {
    std::ptr::null_mut()
}

pub unsafe extern "C" fn carla_lane_invasion_event_get_crossed_lane_markings(
    _event: *mut carla_lane_invasion_event_data_t,
    _out_count: *mut usize,
) -> *mut carla_sys::carla_lane_marking_t {
    if !_out_count.is_null() {
        *_out_count = 0;
    }
    std::ptr::null_mut()
}

pub unsafe extern "C" fn carla_lane_marking_get_type(
    _marking: *mut carla_sys::carla_lane_marking_t,
) -> LaneMarking_Type {
    // Return a default value - we'll need to define this enum
    0 // Represents "None" type
}

pub unsafe extern "C" fn carla_lane_marking_get_color(
    _marking: *mut carla_sys::carla_lane_marking_t,
) -> LaneMarking_Color {
    0 // Represents "Standard" color
}

pub unsafe extern "C" fn carla_lane_marking_get_lane_change(
    _marking: *mut carla_sys::carla_lane_marking_t,
) -> LaneMarking_LaneChange {
    0 // Represents "None" lane change
}

pub unsafe extern "C" fn carla_lane_marking_get_width(
    _marking: *mut carla_sys::carla_lane_marking_t,
) -> f64 {
    0.0
}

// Enum stub types
pub type LaneMarking_Type = u32;
pub type LaneMarking_Color = u32;
pub type LaneMarking_LaneChange = u32;

// Image analysis types
#[repr(C)]
pub struct carla_image_analyzer_t {
    pub _placeholder: u8,
}

#[repr(C)]
pub struct carla_semantic_analysis_result_t {
    pub _placeholder: u8,
}

#[repr(C)]
pub struct carla_instance_analysis_result_t {
    pub _placeholder: u8,
}

// Missing sensor data type constants
pub const carla_sensor_data_type_t_CARLA_SENSOR_DATA_DVS_EVENTS:
    carla_sys::carla_sensor_data_type_t = 999;
pub const carla_sensor_data_type_t_CARLA_SENSOR_DATA_OPTICAL_FLOW:
    carla_sys::carla_sensor_data_type_t = 998;

// Actor type checking functions (needed for conversion methods)
pub unsafe extern "C" fn carla_actor_is_walker(_actor: *const carla_sys::carla_actor_t) -> bool {
    // TODO: Implement when C API is available
    false
}

pub unsafe extern "C" fn carla_actor_is_traffic_light(
    _actor: *const carla_sys::carla_actor_t,
) -> bool {
    // TODO: Implement when C API is available
    // For now, check based on type ID string comparison
    false
}

pub unsafe extern "C" fn carla_actor_is_traffic_sign(
    _actor: *const carla_sys::carla_actor_t,
) -> bool {
    // TODO: Implement when C API is available
    // For now, check based on type ID string comparison
    false
}

// Missing actor functions
pub unsafe extern "C" fn carla_actor_get_parent(
    _actor: *const carla_sys::carla_actor_t,
) -> *mut carla_sys::carla_actor_t {
    // TODO: Implement when C API is available
    std::ptr::null_mut()
}

// Wrapper function to provide error handling for carla_actor_set_transform
pub unsafe fn carla_actor_set_transform_checked(
    actor: *mut carla_sys::carla_actor_t,
    transform: *const carla_sys::carla_transform_t,
) -> carla_sys::carla_error_t {
    carla_sys::carla_actor_set_transform(actor, transform);
    carla_sys::carla_error_t_CARLA_ERROR_NONE
}

// Wrapper function to provide error handling for carla_actor_destroy
pub unsafe fn carla_actor_destroy_checked(
    actor: *mut carla_sys::carla_actor_t,
) -> carla_sys::carla_error_t {
    carla_sys::carla_actor_destroy(actor);
    carla_sys::carla_error_t_CARLA_ERROR_NONE
}
