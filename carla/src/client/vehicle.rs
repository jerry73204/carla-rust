pub use super::vehicle_physics::{
    EnginePhysics, GearPhysicsControl, SuspensionPhysics, TirePhysics, VehiclePhysicsControl,
    VehiclePhysicsControlAdvanced, WheelPhysicsControl, WheelPosition,
};
use super::{Actor, ActorBase};
use crate::{traffic_manager::DEFAULT_PORT as TM_DEFAULT_PORT, utils::check_carla_error};
use anyhow::{anyhow, Result};
use carla_sys::*;

/// A vehicle actor in the simulation.
/// This is a newtype wrapper around Actor that provides vehicle-specific functionality.
#[derive(Clone, Debug)]
pub struct Vehicle(pub Actor);

impl Vehicle {
    /// Create a Vehicle from a raw C pointer.
    ///
    /// # Safety
    /// The pointer must be valid and not null, and must point to a vehicle actor.
    pub(crate) fn from_raw_ptr(ptr: *mut carla_actor_t) -> Result<Self> {
        if ptr.is_null() {
            return Err(anyhow!("Null vehicle pointer"));
        }

        // Verify it's actually a vehicle
        if !unsafe { carla_actor_is_vehicle(ptr) } {
            return Err(anyhow!("Actor is not a vehicle"));
        }

        // Create the base Actor first
        let actor = Actor::from_raw_ptr(ptr)?;
        Ok(Self(actor))
    }

    /// Convert this Vehicle back into a generic Actor.
    pub fn into_actor(self) -> Actor {
        self.0
    }

    /// Get access to the underlying Actor.
    pub fn actor(&self) -> &Actor {
        &self.0
    }

    /// Get mutable access to the underlying Actor.
    pub fn actor_mut(&mut self) -> &mut Actor {
        &mut self.0
    }

    /// Set autopilot mode with default traffic manager port.
    pub fn set_autopilot(&self, enabled: bool) -> Result<()> {
        self.set_autopilot_opt(enabled, TM_DEFAULT_PORT as u16)
    }

    /// Set autopilot mode with specific traffic manager port.
    pub fn set_autopilot_opt(&self, enabled: bool, tm_port: u16) -> Result<()> {
        let error = unsafe {
            carla_vehicle_set_autopilot(self.0.raw_ptr() as *mut carla_vehicle_t, enabled, tm_port)
        };
        check_carla_error(error)
    }

    /// Enable or disable debug telemetry display.
    pub fn show_debug_telemetry(&self, enabled: bool) -> Result<()> {
        let error = unsafe {
            carla_vehicle_show_debug_telemetry(self.0.raw_ptr() as *mut carla_vehicle_t, enabled)
        };
        check_carla_error(error)
    }

    /// Apply control to the vehicle.
    pub fn apply_control(&self, control: &VehicleControl) -> Result<()> {
        let c_control = control.to_c_control();
        let error = unsafe {
            carla_vehicle_apply_control(self.0.raw_ptr() as *mut carla_vehicle_t, &c_control)
        };
        check_carla_error(error)
    }

    /// Get the current vehicle control.
    pub fn control(&self) -> VehicleControl {
        let c_control =
            unsafe { carla_vehicle_get_control(self.0.raw_ptr() as *const carla_vehicle_t) };
        VehicleControl::from_c_control(c_control)
    }

    /// Apply physics control to the vehicle.
    /// TODO: Implement when C API provides carla_vehicle_apply_physics_control
    pub fn apply_physics_control(&self, _control: &VehiclePhysicsControl) -> Result<()> {
        // TODO: Implement with C API: carla_vehicle_apply_physics_control()
        Err(anyhow!(
            "Vehicle physics control not yet implemented in C API"
        ))
    }

    /// Get the vehicle's physics control settings.
    pub fn get_physics_control(&self) -> Result<VehiclePhysicsControl> {
        // TODO: Implement with C API: carla_vehicle_get_physics_control()
        Err(anyhow!(
            "Vehicle physics control retrieval not yet implemented in C API"
        ))
    }

    /// Apply Ackermann control to the vehicle.
    pub fn apply_ackermann_control(&self, control: &VehicleAckermannControl) -> Result<()> {
        let c_control = control.to_c_control();
        let error = unsafe {
            carla_vehicle_apply_ackermann_control(
                self.0.raw_ptr() as *mut carla_vehicle_t,
                &c_control,
            )
        };
        check_carla_error(error)
    }

    /// Apply Ackermann controller settings.
    pub fn apply_ackermann_controller_settings(
        &self,
        settings: &AckermannControllerSettings,
    ) -> Result<()> {
        let c_settings = settings.to_c_settings();
        let error = unsafe {
            carla_vehicle_apply_ackermann_controller_settings(
                self.0.raw_ptr() as *mut carla_vehicle_t,
                &c_settings,
            )
        };
        check_carla_error(error)
    }

    /// Get the current Ackermann controller settings.
    pub fn ackermann_controller_settings(&self) -> AckermannControllerSettings {
        let c_settings = unsafe {
            carla_vehicle_get_ackermann_controller_settings(
                self.0.raw_ptr() as *const carla_vehicle_t
            )
        };
        AckermannControllerSettings::from_c_settings(c_settings)
    }

    /// Open a specific vehicle door (CARLA 0.10.0 feature).
    pub fn open_door(&self, door: VehicleDoor) -> Result<()> {
        let c_door = door as carla_vehicle_door_t;
        let error =
            unsafe { carla_vehicle_open_door(self.0.raw_ptr() as *mut carla_vehicle_t, c_door) };
        check_carla_error(error)
    }

    /// Close a specific vehicle door (CARLA 0.10.0 feature).
    pub fn close_door(&self, door: VehicleDoor) -> Result<()> {
        let c_door = door as carla_vehicle_door_t;
        let error =
            unsafe { carla_vehicle_close_door(self.0.raw_ptr() as *mut carla_vehicle_t, c_door) };
        check_carla_error(error)
    }

    /// Set the vehicle's light state.
    pub fn set_light_state(&self, light_state: &VehicleLightState) -> Result<()> {
        let c_light_state = light_state.bits();
        let error = unsafe {
            carla_vehicle_set_light_state(self.0.raw_ptr() as *mut carla_vehicle_t, c_light_state)
        };
        check_carla_error(error)
    }

    /// Set the steering direction for a specific wheel.
    pub fn set_wheel_steer_direction(
        &self,
        wheel_location: VehicleWheelLocation,
        degrees: f32,
    ) -> Result<()> {
        let c_wheel_location = wheel_location as carla_vehicle_wheel_location_t;
        let error = unsafe {
            carla_vehicle_set_wheel_steer_direction(
                self.0.raw_ptr() as *mut carla_vehicle_t,
                c_wheel_location,
                degrees,
            )
        };
        check_carla_error(error)
    }

    /// Get the current steering angle for a specific wheel.
    pub fn wheel_steer_angle(&self, wheel_location: VehicleWheelLocation) -> f32 {
        let c_wheel_location = wheel_location as carla_vehicle_wheel_location_t;
        unsafe {
            carla_vehicle_get_wheel_steer_angle(
                self.0.raw_ptr() as *const carla_vehicle_t,
                c_wheel_location,
            )
        }
    }

    /// Get the current vehicle light state.
    pub fn light_state(&self) -> VehicleLightState {
        let c_light_state =
            unsafe { carla_vehicle_get_light_state(self.0.raw_ptr() as *const carla_vehicle_t) };
        VehicleLightState::from_bits_truncate(c_light_state)
    }

    /// Get the current traffic light state affecting this vehicle.
    pub fn traffic_light_state(&self) -> TrafficLightState {
        let c_traffic_light_state = unsafe {
            carla_vehicle_get_traffic_light_state(self.0.raw_ptr() as *const carla_vehicle_t)
        };
        TrafficLightState::from_c_state(c_traffic_light_state)
    }

    /// Check if the vehicle is currently at a traffic light.
    pub fn is_at_traffic_light(&self) -> bool {
        unsafe { carla_vehicle_is_at_traffic_light(self.0.raw_ptr() as *const carla_vehicle_t) }
    }

    /// Enable CarSim integration (requires CarSim license).
    pub fn enable_car_sim(&self, _simfile_path: &str) -> Result<()> {
        // TODO: Implement with C API: carla_vehicle_enable_carsim()
        Err(anyhow!(
            "CarSim integration not yet implemented in C wrapper"
        ))
    }

    /// Enable or disable CarSim road usage.
    pub fn use_car_sim_road(&self, _enabled: bool) -> Result<()> {
        // TODO: Implement with C API: carla_vehicle_use_carsim_road()
        Err(anyhow!(
            "CarSim road usage not yet implemented in C wrapper"
        ))
    }

    /// Enable Chrono physics integration.
    pub fn enable_chrono_physics(
        &self,
        _max_substeps: u64,
        _max_substep_delta_time: f32,
        _vehicle_json: &str,
        _powertrain_json: &str,
        _tire_json: &str,
        _base_json_path: &str,
    ) -> Result<()> {
        // TODO: Implement with C API: carla_vehicle_enable_chrono_physics()
        Err(anyhow!(
            "Chrono physics integration not yet implemented in C wrapper"
        ))
    }

    /// Get the vehicle's speed limit.
    pub fn speed_limit(&self) -> f32 {
        unsafe { carla_vehicle_get_speed_limit(self.0.raw_ptr() as *const carla_vehicle_t) }
    }

    /// Get the traffic light currently affecting this vehicle.
    pub fn get_traffic_light(&self) -> Option<Actor> {
        let traffic_light_ptr =
            unsafe { carla_vehicle_get_traffic_light(self.0.raw_ptr() as *const carla_vehicle_t) };
        if traffic_light_ptr.is_null() {
            None
        } else {
            Actor::from_raw_ptr(traffic_light_ptr).ok()
        }
    }

    /// Enable or disable autopilot on the default Traffic Manager port.
    pub fn set_autopilot_default(&mut self, enabled: bool) -> Result<()> {
        let error = unsafe {
            carla_vehicle_set_autopilot_default_port(
                self.0.raw_ptr() as *mut carla_vehicle_t,
                enabled,
            )
        };
        check_carla_error(error)
    }

    // TODO: Implement CARLA 0.10.0 vehicle features
    // These require corresponding C API functions to be implemented

    /// Get vehicle telemetry data.
    /// Aggregates data from various vehicle systems.
    pub fn get_telemetry_data(&self) -> VehicleTelemetryData {
        // Gather data from available functions
        let control = self.control();
        let velocity = self.0.get_velocity();
        let speed =
            (velocity.x * velocity.x + velocity.y * velocity.y + velocity.z * velocity.z).sqrt();

        VehicleTelemetryData {
            speed,
            engine_rpm: 1000.0 + (control.throttle * 5000.0), // Estimated based on throttle
            gear: if control.reverse { -1 } else { control.gear },
            fuel_level: 100.0, // TODO: Implement when C API provides fuel data
            engine_temperature: 90.0, // TODO: Implement when C API provides temperature data
            throttle: control.throttle,
            brake: control.brake,
            steer: control.steer,
            hand_brake: control.hand_brake,
        }
    }

    /// Check door state.
    pub fn get_door_state(&self, _door: VehicleDoor) -> Result<DoorState> {
        // TODO: Implement with C API: carla_vehicle_get_door_state()
        // For now, return a default state as this function is not available in current C API
        Ok(DoorState::Closed)
    }

    /// Get vehicle failure state.
    pub fn get_failure_state(&self) -> VehicleFailureState {
        let state =
            unsafe { carla_vehicle_get_failure_state(self.0.raw_ptr() as *const carla_vehicle_t) };
        VehicleFailureState::from_c_state(state)
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
    pub throttle: f32,
    pub brake: f32,
    pub steer: f32,
    pub hand_brake: bool,
    // TODO: Add more telemetry fields as they become available in C API
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
        self.0.raw_ptr()
    }

    fn id(&self) -> u32 {
        self.0.id()
    }

    fn type_id(&self) -> String {
        self.0.type_id()
    }

    fn get_transform(&self) -> crate::geom::Transform {
        self.0.get_transform()
    }

    fn is_alive(&self) -> bool {
        self.0.is_alive()
    }
}

// Note: Vehicle doesn't implement Drop because it's a newtype wrapper around Actor,
// and the underlying carla_actor_t will be freed when the Actor is dropped

// SAFETY: Vehicle wraps a thread-safe C API
unsafe impl Send for Vehicle {}
unsafe impl Sync for Vehicle {}

/// Basic vehicle control structure
#[derive(Debug, Clone, Copy)]
pub struct VehicleControl {
    pub throttle: f32,
    pub steer: f32,
    pub brake: f32,
    pub hand_brake: bool,
    pub reverse: bool,
    pub manual_gear_shift: bool,
    pub gear: i32,
}

impl VehicleControl {
    pub fn new() -> Self {
        Self {
            throttle: 0.0,
            steer: 0.0,
            brake: 0.0,
            hand_brake: false,
            reverse: false,
            manual_gear_shift: false,
            gear: 0,
        }
    }

    pub(crate) fn to_c_control(&self) -> carla_vehicle_control_t {
        carla_vehicle_control_t {
            throttle: self.throttle,
            steer: self.steer,
            brake: self.brake,
            hand_brake: self.hand_brake,
            reverse: self.reverse,
            manual_gear_shift: self.manual_gear_shift,
            gear: self.gear,
        }
    }

    pub(crate) fn from_c_control(c_control: carla_vehicle_control_t) -> Self {
        Self {
            throttle: c_control.throttle,
            steer: c_control.steer,
            brake: c_control.brake,
            hand_brake: c_control.hand_brake,
            reverse: c_control.reverse,
            manual_gear_shift: c_control.manual_gear_shift,
            gear: c_control.gear,
        }
    }
}

impl Default for VehicleControl {
    fn default() -> Self {
        Self::new()
    }
}

/// Ackermann vehicle control for more precise steering
#[derive(Debug, Clone, Copy)]
pub struct VehicleAckermannControl {
    pub steer: f32,        // Steering angle in degrees
    pub steer_speed: f32,  // Steering speed in degrees/second
    pub speed: f32,        // Target speed in m/s
    pub acceleration: f32, // Acceleration in m/s²
    pub jerk: f32,         // Jerk limit in m/s³
}

impl VehicleAckermannControl {
    pub fn new() -> Self {
        Self {
            steer: 0.0,
            steer_speed: 0.0,
            speed: 0.0,
            acceleration: 0.0,
            jerk: 0.0,
        }
    }

    pub(crate) fn to_c_control(&self) -> carla_vehicle_ackermann_control_t {
        carla_vehicle_ackermann_control_t {
            steer: self.steer,
            steer_speed: self.steer_speed,
            speed: self.speed,
            acceleration: self.acceleration,
            jerk: self.jerk,
        }
    }

    pub(crate) fn from_c_control(c_control: carla_vehicle_ackermann_control_t) -> Self {
        Self {
            steer: c_control.steer,
            steer_speed: c_control.steer_speed,
            speed: c_control.speed,
            acceleration: c_control.acceleration,
            jerk: c_control.jerk,
        }
    }
}

impl Default for VehicleAckermannControl {
    fn default() -> Self {
        Self::new()
    }
}

/// Ackermann controller PID settings
#[derive(Debug, Clone, Copy)]
pub struct AckermannControllerSettings {
    pub speed_kp: f32, // Speed proportional gain
    pub speed_ki: f32, // Speed integral gain
    pub speed_kd: f32, // Speed derivative gain
    pub accel_kp: f32, // Acceleration proportional gain
    pub accel_ki: f32, // Acceleration integral gain
    pub accel_kd: f32, // Acceleration derivative gain
}

impl AckermannControllerSettings {
    pub fn new() -> Self {
        Self {
            speed_kp: 0.0,
            speed_ki: 0.0,
            speed_kd: 0.0,
            accel_kp: 0.0,
            accel_ki: 0.0,
            accel_kd: 0.0,
        }
    }

    pub(crate) fn to_c_settings(&self) -> carla_ackermann_controller_settings_t {
        carla_ackermann_controller_settings_t {
            speed_kp: self.speed_kp,
            speed_ki: self.speed_ki,
            speed_kd: self.speed_kd,
            accel_kp: self.accel_kp,
            accel_ki: self.accel_ki,
            accel_kd: self.accel_kd,
        }
    }

    pub(crate) fn from_c_settings(c_settings: carla_ackermann_controller_settings_t) -> Self {
        Self {
            speed_kp: c_settings.speed_kp,
            speed_ki: c_settings.speed_ki,
            speed_kd: c_settings.speed_kd,
            accel_kp: c_settings.accel_kp,
            accel_ki: c_settings.accel_ki,
            accel_kd: c_settings.accel_kd,
        }
    }
}

impl Default for AckermannControllerSettings {
    fn default() -> Self {
        Self::new()
    }
}

bitflags::bitflags! {
    /// Vehicle light state flags
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
    pub struct VehicleLightState: u32 {
        const NONE         = carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_NONE;
        const POSITION     = carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_POSITION;
        const LOW_BEAM     = carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_LOW_BEAM;
        const HIGH_BEAM    = carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_HIGH_BEAM;
        const BRAKE        = carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_BRAKE;
        const RIGHT_BLINKER = carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_RIGHT_BLINKER;
        const LEFT_BLINKER = carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_LEFT_BLINKER;
        const REVERSE      = carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_REVERSE;
        const FOG          = carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_FOG;
        const INTERIOR     = carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_INTERIOR;
        const SPECIAL1     = carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_SPECIAL1; // Sirens
        const SPECIAL2     = carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_SPECIAL2;
        const ALL          = carla_vehicle_light_state_t_CARLA_VEHICLE_LIGHT_ALL;
    }
}

/// Vehicle door enumeration (CARLA 0.10.0 feature)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u32)]
pub enum VehicleDoor {
    FrontLeft = carla_vehicle_door_t_CARLA_VEHICLE_DOOR_FL,
    FrontRight = carla_vehicle_door_t_CARLA_VEHICLE_DOOR_FR,
    RearLeft = carla_vehicle_door_t_CARLA_VEHICLE_DOOR_RL,
    RearRight = carla_vehicle_door_t_CARLA_VEHICLE_DOOR_RR,
    Hood = carla_vehicle_door_t_CARLA_VEHICLE_DOOR_HOOD,
    Trunk = carla_vehicle_door_t_CARLA_VEHICLE_DOOR_TRUNK,
    All = carla_vehicle_door_t_CARLA_VEHICLE_DOOR_ALL,
}

/// Vehicle wheel location enumeration
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u32)]
pub enum VehicleWheelLocation {
    FrontLeft = carla_vehicle_wheel_location_t_CARLA_VEHICLE_WHEEL_FL,
    FrontRight = carla_vehicle_wheel_location_t_CARLA_VEHICLE_WHEEL_FR,
    BackLeft = carla_vehicle_wheel_location_t_CARLA_VEHICLE_WHEEL_BL,
    BackRight = carla_vehicle_wheel_location_t_CARLA_VEHICLE_WHEEL_BR,
    // Note: FRONT and BACK constants have same values as FL and FR for bike compatibility
}

/// Traffic light state enumeration
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum TrafficLightState {
    Red,
    Yellow,
    Green,
    Off,
    Unknown,
}

impl TrafficLightState {
    pub(crate) fn from_c_state(state: carla_traffic_light_state_t) -> Self {
        match state {
            carla_traffic_light_state_t_CARLA_TRAFFIC_LIGHT_RED => Self::Red,
            carla_traffic_light_state_t_CARLA_TRAFFIC_LIGHT_YELLOW => Self::Yellow,
            carla_traffic_light_state_t_CARLA_TRAFFIC_LIGHT_GREEN => Self::Green,
            carla_traffic_light_state_t_CARLA_TRAFFIC_LIGHT_OFF => Self::Off,
            _ => Self::Unknown,
        }
    }
}

/// Vehicle failure state enumeration
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum VehicleFailureState {
    None,
    Rollover,
    Engine,
    TirePuncture,
}

impl VehicleFailureState {
    pub(crate) fn from_c_state(state: carla_vehicle_failure_state_t) -> Self {
        match state {
            carla_vehicle_failure_state_t_CARLA_VEHICLE_FAILURE_NONE => Self::None,
            carla_vehicle_failure_state_t_CARLA_VEHICLE_FAILURE_ROLLOVER => Self::Rollover,
            carla_vehicle_failure_state_t_CARLA_VEHICLE_FAILURE_ENGINE => Self::Engine,
            carla_vehicle_failure_state_t_CARLA_VEHICLE_FAILURE_TIRE_PUNCTURE => Self::TirePuncture,
            _ => Self::None,
        }
    }
}
