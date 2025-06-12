use super::{Actor, ActorBase};
use crate::{
    rpc::{
        AckermannControllerSettings, TrafficLightState, VehicleAckermannControl, VehicleControl,
        VehicleDoor, VehicleLightState, VehiclePhysicsControl, VehicleWheelLocation,
    },
    traffic_manager::constants::Networking::TM_DEFAULT_PORT,
    utils::check_carla_error,
};
use anyhow::{anyhow, Result};
use carla_sys::*;
use std::ptr;

/// Represents a vehicle in the simulation, corresponding to
/// `carla.Vehicle` in Python API.
#[derive(Clone, Debug)]
pub struct Vehicle {
    inner: *mut carla_vehicle_t, // carla_vehicle_t is typedef for carla_actor_t
}

impl Vehicle {
    /// Create a Vehicle from a raw C pointer.
    /// 
    /// # Safety
    /// The pointer must be valid and not null.
    pub(crate) fn from_raw_ptr(ptr: *mut carla_vehicle_t) -> Result<Self> {
        if ptr.is_null() {
            return Err(anyhow!("Null vehicle pointer"));
        }
        Ok(Self { inner: ptr })
    }

    pub fn set_autopilot(&self, enabled: bool) -> Result<()> {
        self.set_autopilot_opt(enabled, TM_DEFAULT_PORT)
    }

    pub fn set_autopilot_opt(&self, enabled: bool, tm_port: u16) -> Result<()> {
        let error = unsafe { carla_vehicle_set_autopilot(self.inner, enabled, tm_port) };
        check_carla_error(error)
    }

    pub fn show_debug_telemetry(&self, enabled: bool) -> Result<()> {
        let error = unsafe { carla_vehicle_show_debug_telemetry(self.inner, enabled) };
        check_carla_error(error)
    }

    pub fn apply_control(&self, control: &VehicleControl) -> Result<()> {
        let c_control = control.to_c_control();
        let error = unsafe { carla_vehicle_apply_control(self.inner, &c_control) };
        check_carla_error(error)
    }

    pub fn control(&self) -> VehicleControl {
        let c_control = unsafe { carla_vehicle_get_control(self.inner) };
        VehicleControl::from_c_control(c_control)
    }

    pub fn apply_physics_control(&self, control: &VehiclePhysicsControl) {
        let control = control.to_cxx();
        self.inner.ApplyPhysicsControl(&control);
    }

    pub fn physics_control(&self) -> VehiclePhysicsControl {
        VehiclePhysicsControl::from_cxx(&self.inner.GetPhysicsControl().within_unique_ptr())
    }

    pub fn apply_ackermann_control(&self, control: &VehicleAckermannControl) -> Result<()> {
        let c_control = control.to_c_control();
        let error = unsafe { carla_vehicle_apply_ackermann_control(self.inner, &c_control) };
        check_carla_error(error)
    }

    pub fn apply_ackermann_controller_settings(&self, settings: &AckermannControllerSettings) -> Result<()> {
        let c_settings = settings.to_c_settings();
        let error = unsafe { carla_vehicle_apply_ackermann_controller_settings(self.inner, &c_settings) };
        check_carla_error(error)
    }

    pub fn ackermann_controller_settings(&self) -> AckermannControllerSettings {
        let c_settings = unsafe { carla_vehicle_get_ackermann_controller_settings(self.inner) };
        AckermannControllerSettings::from_c_settings(c_settings)
    }

    pub fn open_door(&self, door: VehicleDoor) -> Result<()> {
        let c_door = door as carla_vehicle_door_t;
        let error = unsafe { carla_vehicle_open_door(self.inner, c_door) };
        check_carla_error(error)
    }

    pub fn close_door(&self, door: VehicleDoor) -> Result<()> {
        let c_door = door as carla_vehicle_door_t;
        let error = unsafe { carla_vehicle_close_door(self.inner, c_door) };
        check_carla_error(error)
    }

    pub fn set_light_state(&self, light_state: &VehicleLightState) -> Result<()> {
        let c_light_state = light_state.to_c_light_state();
        let error = unsafe { carla_vehicle_set_light_state(self.inner, c_light_state) };
        check_carla_error(error)
    }

    pub fn set_wheel_steer_direction(&self, wheel_location: VehicleWheelLocation, degrees: f32) -> Result<()> {
        let c_wheel_location = wheel_location as carla_vehicle_wheel_location_t;
        let error = unsafe { carla_vehicle_set_wheel_steer_direction(self.inner, c_wheel_location, degrees) };
        check_carla_error(error)
    }

    pub fn wheel_steer_angle(&self, wheel_location: VehicleWheelLocation) -> f32 {
        let c_wheel_location = wheel_location as carla_vehicle_wheel_location_t;
        unsafe { carla_vehicle_get_wheel_steer_angle(self.inner, c_wheel_location) }
    }

    pub fn light_state(&self) -> VehicleLightState {
        let c_light_state = unsafe { carla_vehicle_get_light_state(self.inner) };
        VehicleLightState::from_c_light_state(c_light_state)
    }

    pub fn traffic_light_state(&self) -> TrafficLightState {
        let c_traffic_light_state = unsafe { carla_vehicle_get_traffic_light_state(self.inner) };
        TrafficLightState::from_c_state(c_traffic_light_state)
    }

    pub fn is_at_traffic_light(&self) -> bool {
        unsafe { carla_vehicle_is_at_traffic_light(self.inner) }
    }

    pub fn enable_car_sim(&self, simfile_path: &str) -> Result<()> {
        // Note: CarSim integration needs to be implemented in C wrapper
        Err(anyhow!("CarSim integration not yet implemented in C wrapper"))
    }

    pub fn use_car_sim_road(&self, enabled: bool) -> Result<()> {
        // Note: CarSim road usage needs to be implemented in C wrapper
        Err(anyhow!("CarSim road usage not yet implemented in C wrapper"))
    }

    pub fn enable_chrono_physics(
        &self,
        max_substeps: u64,
        max_substep_delta_time: f32,
        vehicle_json: &str,
        powertrain_json: &str,
        tire_json: &str,
        base_json_path: &str,
    ) -> Result<()> {
        // Note: Chrono physics integration needs to be implemented in C wrapper
        Err(anyhow!("Chrono physics integration not yet implemented in C wrapper"))
    }

    pub fn into_actor(self) -> Result<Actor> {
        Actor::from_raw_ptr(self.inner)
    }

    pub fn speed_limit(&self) -> f32 {
        unsafe { carla_vehicle_get_speed_limit(self.inner) }
    }
    
    pub fn get_traffic_light(&self) -> Option<Actor> {
        let traffic_light_ptr = unsafe { carla_vehicle_get_traffic_light(self.inner) };
        if traffic_light_ptr.is_null() {
            None
        } else {
            Actor::from_raw_ptr(traffic_light_ptr).ok()
        }
    }

    // TODO: Implement CARLA 0.10.0 vehicle features
    // These require corresponding C API functions to be implemented

    /// Get vehicle telemetry data.
    pub fn get_telemetry_data(&self) -> Result<VehicleTelemetryData> {
        // TODO: Implement with C API: carla_vehicle_get_telemetry_data()
        todo!("Implement vehicle telemetry data retrieval")
    }

    /// Get vehicle physics control settings.
    pub fn get_physics_control(&self) -> Result<VehiclePhysicsControl> {
        // TODO: Implement with C API: carla_vehicle_get_physics_control()
        todo!("Implement physics control retrieval")
    }

    /// Check door state.
    pub fn get_door_state(&self, door: VehicleDoor) -> Result<DoorState> {
        // TODO: Implement with C API: carla_vehicle_get_door_state()
        todo!("Implement door state query")
    }

    /// Get vehicle failure state.
    pub fn get_failure_state(&self) -> Result<VehicleFailureState> {
        // TODO: Implement with C API: carla_vehicle_get_failure_state()
        todo!("Implement failure state retrieval")
    }

    /// Set vehicle damage parameters.
    pub fn set_damage(&self, damage: &VehicleDamage) -> Result<()> {
        // TODO: Implement with C API: carla_vehicle_set_damage()
        todo!("Implement vehicle damage system")
    }

    /// Get wheel physics properties.
    pub fn get_wheel_physics(&self, wheel: VehicleWheelLocation) -> Result<WheelPhysics> {
        // TODO: Implement with C API: carla_vehicle_get_wheel_physics()
        todo!("Implement wheel physics query")
    }

    /// Enable/disable specific vehicle subsystems.
    pub fn set_subsystem_enabled(&self, subsystem: VehicleSubsystem, enabled: bool) -> Result<()> {
        // TODO: Implement with C API: carla_vehicle_set_subsystem_enabled()
        todo!("Implement vehicle subsystem control")
    }
}

// TODO: Define these vehicle-related types when implementing the above methods

/// Vehicle telemetry data.
#[derive(Debug, Clone)]
pub struct VehicleTelemetryData {
    pub speed: f32,
    pub engine_rpm: f32,
    pub gear: i32,
    pub fuel_level: f32,
    pub engine_temperature: f32,
    // TODO: Add more telemetry fields
}

/// Door state information.
#[derive(Debug, Clone, PartialEq)]
pub enum DoorState {
    Closed,
    Open,
    Opening,
    Closing,
}

/// Vehicle damage parameters.
#[derive(Debug, Clone)]
pub struct VehicleDamage {
    pub engine_damage: f32,
    pub tire_damage: [f32; 4],
    pub body_damage: f32,
    // TODO: Add more damage types
}

/// Wheel physics properties.
#[derive(Debug, Clone)]
pub struct WheelPhysics {
    pub friction: f32,
    pub suspension_stiffness: f32,
    pub suspension_damping: f32,
    pub max_brake_torque: f32,
    // TODO: Add more wheel physics properties
}

/// Vehicle subsystems that can be controlled.
#[derive(Debug, Clone, PartialEq)]
pub enum VehicleSubsystem {
    Engine,
    Brakes,
    Steering,
    Transmission,
    ABS,
    ESP,
    // TODO: Add more subsystems
}

impl ActorBase for Vehicle {
    fn raw_ptr(&self) -> *mut carla_actor_t {
        self.inner
    }
}

impl TryFrom<Actor> for Vehicle {
    type Error = Actor;

    fn try_from(value: Actor) -> std::result::Result<Self, Self::Error> {
        // Check if the actor is actually a vehicle
        let is_vehicle = unsafe { carla_actor_is_vehicle(value.inner) };
        if is_vehicle {
            let vehicle_ptr = unsafe { carla_actor_as_vehicle(value.inner) };
            if !vehicle_ptr.is_null() {
                return Ok(Vehicle { inner: vehicle_ptr });
            }
        }
        Err(value)
    }
}

impl Drop for Vehicle {
    fn drop(&mut self) {
        // Note: Vehicle uses the same pointer as Actor, so we don't need to free it separately
        // The Actor destructor will handle the cleanup
        self.inner = ptr::null_mut();
    }
}

// SAFETY: Vehicle wraps a thread-safe C API
unsafe impl Send for Vehicle {}
unsafe impl Sync for Vehicle {}
