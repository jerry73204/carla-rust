//! Defines message types used for communication.

mod environment_object;
mod episode_settings;
mod light_id;
mod map_layer;
mod vehicle_light_state_list;
mod vehicle_physics_control;

pub use carla_sys::{
    carla::rpc::{
        AckermannControllerSettings, AttachmentType, GearPhysicsControl,
        OpendriveGenerationParameters, TrafficLightState, VehicleAckermannControl, VehicleControl,
        VehicleDoor, VehicleLightState_LightState as VehicleLightState, VehicleWheelLocation,
        WeatherParameters, WheelPhysicsControl,
    },
    carla_rust::rpc::{
        FfiActorId as ActorId, FfiLabelledPoint as LabelledPoint, FfiRpcLightGroup as LightGroup,
        FfiRpcLightState as LightState,
    },
};

// Extension trait for WeatherParameters to support C FFI conversion
impl WeatherParameters {
    pub fn from_c_weather(weather: carla_sys::carla_weather_parameters_t) -> Self {
        WeatherParameters {
            cloudiness: weather.cloudiness,
            precipitation: weather.precipitation,
            precipitation_deposits: weather.precipitation_deposits,
            wind_intensity: weather.wind_intensity,
            sun_azimuth_angle: weather.sun_azimuth_angle,
            sun_altitude_angle: weather.sun_altitude_angle,
            fog_density: weather.fog_density,
            fog_distance: weather.fog_distance,
            wetness: weather.wetness,
            fog_falloff: weather.fog_falloff,
            scattering_intensity: weather.scattering_intensity,
            mie_scattering_scale: weather.mie_scattering_scale,
            rayleigh_scattering_scale: weather.rayleigh_scattering_scale,
        }
    }

    pub fn to_c_weather(&self) -> carla_sys::carla_weather_parameters_t {
        carla_sys::carla_weather_parameters_t {
            cloudiness: self.cloudiness,
            precipitation: self.precipitation,
            precipitation_deposits: self.precipitation_deposits,
            wind_intensity: self.wind_intensity,
            sun_azimuth_angle: self.sun_azimuth_angle,
            sun_altitude_angle: self.sun_altitude_angle,
            fog_density: self.fog_density,
            fog_distance: self.fog_distance,
            wetness: self.wetness,
            fog_falloff: self.fog_falloff,
            scattering_intensity: self.scattering_intensity,
            mie_scattering_scale: self.mie_scattering_scale,
            rayleigh_scattering_scale: self.rayleigh_scattering_scale,
        }
    }
}

// Extension methods for vehicle control types
impl VehicleControl {
    pub fn to_c_control(&self) -> carla_sys::carla_vehicle_control_t {
        carla_sys::carla_vehicle_control_t {
            throttle: self.throttle,
            steer: self.steer,
            brake: self.brake,
            hand_brake: self.hand_brake,
            reverse: self.reverse,
            manual_gear_shift: self.manual_gear_shift,
            gear: self.gear,
        }
    }
    
    pub fn from_c_control(control: carla_sys::carla_vehicle_control_t) -> Self {
        VehicleControl {
            throttle: control.throttle,
            steer: control.steer,
            brake: control.brake,
            hand_brake: control.hand_brake,
            reverse: control.reverse,
            manual_gear_shift: control.manual_gear_shift,
            gear: control.gear,
        }
    }
}

impl VehicleAckermannControl {
    pub fn to_c_control(&self) -> carla_sys::carla_vehicle_ackermann_control_t {
        carla_sys::carla_vehicle_ackermann_control_t {
            steer: self.steer,
            steer_speed: self.steer_speed,
            speed: self.speed,
            acceleration: self.acceleration,
            jerk: self.jerk,
        }
    }
    
    pub fn from_c_control(control: carla_sys::carla_vehicle_ackermann_control_t) -> Self {
        VehicleAckermannControl {
            steer: control.steer,
            steer_speed: control.steer_speed,
            speed: control.speed,
            acceleration: control.acceleration,
            jerk: control.jerk,
        }
    }
}

impl AckermannControllerSettings {
    pub fn to_c_settings(&self) -> carla_sys::carla_ackermann_controller_settings_t {
        carla_sys::carla_ackermann_controller_settings_t {
            speed_kp: self.speed_kp,
            speed_ki: self.speed_ki,
            speed_kd: self.speed_kd,
            accel_kp: self.accel_kp,
            accel_ki: self.accel_ki,
            accel_kd: self.accel_kd,
        }
    }
    
    pub fn from_c_settings(settings: carla_sys::carla_ackermann_controller_settings_t) -> Self {
        AckermannControllerSettings {
            speed_kp: settings.speed_kp,
            speed_ki: settings.speed_ki,
            speed_kd: settings.speed_kd,
            accel_kp: settings.accel_kp,
            accel_ki: settings.accel_ki,
            accel_kd: settings.accel_kd,
        }
    }
}

impl VehicleLightState {
    pub fn to_c_light_state(&self) -> carla_sys::carla_vehicle_light_state_t {
        // Convert bitflags to C enum value
        // This is a simplified implementation - actual bitflags conversion depends on the VehicleLightState structure
        self.bits() as carla_sys::carla_vehicle_light_state_t
    }
    
    pub fn from_c_light_state(light_state: carla_sys::carla_vehicle_light_state_t) -> Self {
        // Convert C enum value to bitflags
        // This is a simplified implementation - actual bitflags conversion depends on the VehicleLightState structure
        VehicleLightState::from_bits_truncate(light_state as u32)
    }
}

impl TrafficLightState {
    pub fn from_c_state(state: carla_sys::carla_traffic_light_state_t) -> Self {
        match state {
            carla_sys::carla_traffic_light_state_t_CARLA_TRAFFIC_LIGHT_RED => TrafficLightState::Red,
            carla_sys::carla_traffic_light_state_t_CARLA_TRAFFIC_LIGHT_YELLOW => TrafficLightState::Yellow,
            carla_sys::carla_traffic_light_state_t_CARLA_TRAFFIC_LIGHT_GREEN => TrafficLightState::Green,
            carla_sys::carla_traffic_light_state_t_CARLA_TRAFFIC_LIGHT_OFF => TrafficLightState::Off,
            _ => TrafficLightState::Unknown,
        }
    }
    
    pub fn to_c_state(&self) -> carla_sys::carla_traffic_light_state_t {
        match self {
            TrafficLightState::Red => carla_sys::carla_traffic_light_state_t_CARLA_TRAFFIC_LIGHT_RED,
            TrafficLightState::Yellow => carla_sys::carla_traffic_light_state_t_CARLA_TRAFFIC_LIGHT_YELLOW,
            TrafficLightState::Green => carla_sys::carla_traffic_light_state_t_CARLA_TRAFFIC_LIGHT_GREEN,
            TrafficLightState::Off => carla_sys::carla_traffic_light_state_t_CARLA_TRAFFIC_LIGHT_OFF,
            _ => carla_sys::carla_traffic_light_state_t_CARLA_TRAFFIC_LIGHT_OFF,
        }
    }
}
pub use environment_object::*;
pub use episode_settings::*;
pub use light_id::*;
pub use map_layer::*;
pub use vehicle_light_state_list::*;
pub use vehicle_physics_control::*;
