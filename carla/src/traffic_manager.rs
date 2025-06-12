//! The traffic manager manages groups of autopilot vehicles with
//! user-customized options.

mod traffic_manager;

pub use traffic_manager::*;

// Re-export traffic manager types from carla-sys
pub use carla_sys::{
    carla_road_option_t as RoadOption, carla_traffic_manager_action_buffer_t as ActionBuffer,
    carla_traffic_manager_action_t as Action, carla_traffic_manager_config_t as Config,
    carla_traffic_manager_info_t as Info, carla_traffic_manager_path_t as Path,
    carla_traffic_manager_route_t as Route, carla_traffic_manager_stats_t as Stats,
    carla_traffic_manager_vehicle_config_t as VehicleConfig, CARLA_TM_DEFAULT_PORT as DEFAULT_PORT,
};
