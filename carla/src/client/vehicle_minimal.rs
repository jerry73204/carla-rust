use crate::utils::check_carla_error;
use anyhow::{anyhow, Result};
use carla_sys::*;

/// A vehicle actor in the CARLA simulation.
#[derive(Debug)]
pub struct Vehicle {
    pub(crate) inner: *mut carla_vehicle_t,
}

impl Vehicle {
    /// Create a Vehicle from a raw C pointer.
    ///
    /// # Safety
    /// The pointer must be valid and not null, and must point to a vehicle actor.
    pub(crate) fn from_raw_ptr(ptr: *mut carla_vehicle_t) -> Result<Self> {
        if ptr.is_null() {
            return Err(anyhow!("Null vehicle pointer"));
        }

        // Verify it's actually a vehicle
        if !unsafe { carla_actor_is_vehicle(ptr as *const carla_actor_t) } {
            return Err(anyhow!("Actor is not a vehicle"));
        }

        Ok(Self { inner: ptr })
    }

    /// Try to create a Vehicle from an Actor.
    pub fn from_actor(actor: &super::Actor) -> Option<Self> {
        let vehicle_ptr = unsafe { carla_actor_as_vehicle(actor.raw_ptr()) };
        if vehicle_ptr.is_null() {
            None
        } else {
            Self::from_raw_ptr(vehicle_ptr).ok()
        }
    }

    pub(crate) fn raw_ptr(&self) -> *mut carla_vehicle_t {
        self.inner
    }

    /// Apply vehicle control (throttle, brake, steering, etc.)
    pub fn apply_control(&mut self, control: &VehicleControl) -> Result<()> {
        let c_control = control.to_c_control();
        let error = unsafe { carla_vehicle_apply_control(self.inner, &c_control) };
        check_carla_error(error)
    }

    /// Apply Ackermann vehicle control (more precise steering model)
    pub fn apply_ackermann_control(&mut self, control: &VehicleAckermannControl) -> Result<()> {
        let c_control = control.to_c_control();
        let error = unsafe { carla_vehicle_apply_ackermann_control(self.inner, &c_control) };
        check_carla_error(error)
    }

    /// Get current vehicle control state
    pub fn control(&self) -> VehicleControl {
        let c_control = unsafe { carla_vehicle_get_control(self.inner) };
        VehicleControl::from_c_control(c_control)
    }

    /// Enable or disable autopilot on a specific Traffic Manager port
    pub fn set_autopilot(&mut self, enabled: bool, tm_port: u16) -> Result<()> {
        let error = unsafe { carla_vehicle_set_autopilot(self.inner, enabled, tm_port) };
        check_carla_error(error)
    }

    /// Enable or disable autopilot on the default Traffic Manager port
    pub fn set_autopilot_default(&mut self, enabled: bool) -> Result<()> {
        let error = unsafe { carla_vehicle_set_autopilot_default_port(self.inner, enabled) };
        check_carla_error(error)
    }

    /// Set vehicle light state (headlights, brake lights, etc.)
    pub fn set_light_state(&mut self, light_state: VehicleLightState) -> Result<()> {
        let error = unsafe { carla_vehicle_set_light_state(self.inner, light_state.bits()) };
        check_carla_error(error)
    }

    /// Get current vehicle light state
    pub fn light_state(&self) -> VehicleLightState {
        let state = unsafe { carla_vehicle_get_light_state(self.inner) };
        VehicleLightState::from_bits_truncate(state)
    }

    /// Open a vehicle door (CARLA 0.10.0 feature)
    pub fn open_door(&mut self, door: VehicleDoor) -> Result<()> {
        let error = unsafe { carla_vehicle_open_door(self.inner, door as u32) };
        check_carla_error(error)
    }

    /// Close a vehicle door (CARLA 0.10.0 feature)
    pub fn close_door(&mut self, door: VehicleDoor) -> Result<()> {
        let error = unsafe { carla_vehicle_close_door(self.inner, door as u32) };
        check_carla_error(error)
    }

    /// Set wheel steering angle
    pub fn set_wheel_steer_direction(
        &mut self,
        wheel: VehicleWheelLocation,
        angle_degrees: f32,
    ) -> Result<()> {
        let error = unsafe {
            carla_vehicle_set_wheel_steer_direction(self.inner, wheel as u32, angle_degrees)
        };
        check_carla_error(error)
    }

    /// Get wheel steering angle
    pub fn wheel_steer_angle(&self, wheel: VehicleWheelLocation) -> f32 {
        unsafe { carla_vehicle_get_wheel_steer_angle(self.inner, wheel as u32) }
    }

    /// Apply Ackermann controller settings
    pub fn apply_ackermann_controller_settings(
        &mut self,
        settings: &AckermannControllerSettings,
    ) -> Result<()> {
        let c_settings = settings.to_c_settings();
        let error =
            unsafe { carla_vehicle_apply_ackermann_controller_settings(self.inner, &c_settings) };
        check_carla_error(error)
    }

    /// Get Ackermann controller settings
    pub fn ackermann_controller_settings(&self) -> AckermannControllerSettings {
        let c_settings = unsafe { carla_vehicle_get_ackermann_controller_settings(self.inner) };
        AckermannControllerSettings::from_c_settings(c_settings)
    }

    /// Get current speed limit
    pub fn speed_limit(&self) -> f32 {
        unsafe { carla_vehicle_get_speed_limit(self.inner) }
    }

    /// Get traffic light state affecting this vehicle
    pub fn traffic_light_state(&self) -> TrafficLightState {
        let state = unsafe { carla_vehicle_get_traffic_light_state(self.inner) };
        TrafficLightState::from_c_state(state)
    }

    /// Check if vehicle is at a traffic light
    pub fn is_at_traffic_light(&self) -> bool {
        unsafe { carla_vehicle_is_at_traffic_light(self.inner) }
    }

    /// Get the traffic light affecting this vehicle
    pub fn traffic_light(&self) -> Option<super::Actor> {
        let actor_ptr = unsafe { carla_vehicle_get_traffic_light(self.inner) };
        if actor_ptr.is_null() {
            None
        } else {
            super::Actor::from_raw_ptr(actor_ptr).ok()
        }
    }

    /// Get vehicle failure state
    pub fn failure_state(&self) -> VehicleFailureState {
        let state = unsafe { carla_vehicle_get_failure_state(self.inner) };
        VehicleFailureState::from_c_state(state)
    }

    /// Enable or disable debug telemetry display
    pub fn show_debug_telemetry(&mut self, enabled: bool) -> Result<()> {
        let error = unsafe { carla_vehicle_show_debug_telemetry(self.inner, enabled) };
        check_carla_error(error)
    }
}

// Note: Vehicle doesn't implement Drop because it's a specialized Actor,
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
