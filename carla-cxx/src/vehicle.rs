//! Vehicle actor implementation for CARLA.

use crate::ffi::{
    self, Actor, SimpleAckermannControl, SimpleGearPhysicsControl, SimpleVector3D,
    SimpleVehicleControl, SimpleVehiclePhysicsControl, SimpleWheelPhysicsControl, Vehicle,
};
use anyhow::Result;
use cxx::SharedPtr;

/// High-level wrapper for CARLA Vehicle
pub struct VehicleWrapper {
    inner: SharedPtr<Vehicle>,
}

impl VehicleWrapper {
    /// Create a VehicleWrapper from an Actor (performs cast)
    pub fn from_actor(actor: &Actor) -> Option<Self> {
        let vehicle_ptr = ffi::Actor_CastToVehicle(actor);
        if vehicle_ptr.is_null() {
            None
        } else {
            Some(Self { inner: vehicle_ptr })
        }
    }

    /// Apply vehicle control (throttle, brake, steering)
    pub fn apply_control(&self, control: &VehicleControl) -> Result<()> {
        let simple_control = SimpleVehicleControl {
            throttle: control.throttle,
            steer: control.steer,
            brake: control.brake,
            hand_brake: control.hand_brake,
            reverse: control.reverse,
            manual_gear_shift: control.manual_gear_shift,
            gear: control.gear,
        };
        ffi::Vehicle_ApplyControl(&self.inner, &simple_control);
        Ok(())
    }

    /// Get current vehicle control settings
    pub fn get_control(&self) -> VehicleControl {
        let simple_control = ffi::Vehicle_GetControl(&self.inner);
        VehicleControl {
            throttle: simple_control.throttle,
            steer: simple_control.steer,
            brake: simple_control.brake,
            hand_brake: simple_control.hand_brake,
            reverse: simple_control.reverse,
            manual_gear_shift: simple_control.manual_gear_shift,
            gear: simple_control.gear,
        }
    }

    /// Enable or disable autopilot
    pub fn set_autopilot(&self, enabled: bool) {
        ffi::Vehicle_SetAutopilot(&self.inner, enabled);
    }

    /// Get current vehicle speed in m/s
    pub fn get_speed(&self) -> f32 {
        ffi::Vehicle_GetSpeed(&self.inner)
    }

    /// Get speed limit for the current lane in m/s
    pub fn get_speed_limit(&self) -> f32 {
        ffi::Vehicle_GetSpeedLimit(&self.inner)
    }

    /// Set vehicle light state
    pub fn set_light_state(&self, light_state: VehicleLightState) {
        ffi::Vehicle_SetLightState(&self.inner, light_state.bits());
    }

    /// Get current vehicle light state
    pub fn get_light_state(&self) -> VehicleLightState {
        let bits = ffi::Vehicle_GetLightState(&self.inner);
        VehicleLightState::from_bits_truncate(bits)
    }

    /// Apply Ackermann control for more precise vehicle control
    pub fn apply_ackermann_control(&self, control: &AckermannControl) -> Result<()> {
        let simple_control = SimpleAckermannControl {
            steer: control.steer,
            steer_speed: control.steer_speed,
            speed: control.speed,
            acceleration: control.acceleration,
            jerk: control.jerk,
        };
        ffi::Vehicle_ApplyAckermannControl(&self.inner, &simple_control);
        Ok(())
    }

    /// Get current Ackermann control settings
    pub fn get_ackermann_control(&self) -> AckermannControl {
        let simple_control = ffi::Vehicle_GetAckermannControl(&self.inner);
        AckermannControl {
            steer: simple_control.steer,
            steer_speed: simple_control.steer_speed,
            speed: simple_control.speed,
            acceleration: simple_control.acceleration,
            jerk: simple_control.jerk,
        }
    }

    /// Apply physics control for advanced vehicle physics
    pub fn apply_physics_control(&self, control: &VehiclePhysicsControl) -> Result<()> {
        let simple_control = SimpleVehiclePhysicsControl {
            torque_curve_max_rpm: control.engine.torque_curve_max_rpm,
            torque_curve_max_torque_nm: control.engine.torque_curve_max_torque_nm,
            max_rpm: control.engine.max_rpm,
            moi: control.engine.moi,
            damping_rate_full_throttle: control.engine.damping_rate_full_throttle,
            damping_rate_zero_throttle_clutch_engaged: control
                .engine
                .damping_rate_zero_throttle_clutch_engaged,
            damping_rate_zero_throttle_clutch_disengaged: control
                .engine
                .damping_rate_zero_throttle_clutch_disengaged,
            use_gear_autobox: control.transmission.use_gear_autobox,
            gear_switch_time: control.transmission.gear_switch_time,
            clutch_strength: control.transmission.clutch_strength,
            final_ratio: control.transmission.final_ratio,
            mass: control.mass,
            drag_coefficient: control.drag_coefficient,
            center_of_mass_x: control.center_of_mass.x as f32,
            center_of_mass_y: control.center_of_mass.y as f32,
            center_of_mass_z: control.center_of_mass.z as f32,
            steering_curve_0: control.steering.curve_at_zero,
            steering_curve_1: control.steering.curve_at_one,
            use_sweep_wheel_collision: control.use_sweep_wheel_collision,
        };
        ffi::Vehicle_ApplyPhysicsControl(&self.inner, &simple_control);
        Ok(())
    }

    /// Get current physics control settings
    pub fn get_physics_control(&self) -> VehiclePhysicsControl {
        let simple_control = ffi::Vehicle_GetPhysicsControl(&self.inner);
        VehiclePhysicsControl {
            engine: EnginePhysics {
                torque_curve_max_rpm: simple_control.torque_curve_max_rpm,
                torque_curve_max_torque_nm: simple_control.torque_curve_max_torque_nm,
                max_rpm: simple_control.max_rpm,
                moi: simple_control.moi,
                damping_rate_full_throttle: simple_control.damping_rate_full_throttle,
                damping_rate_zero_throttle_clutch_engaged: simple_control
                    .damping_rate_zero_throttle_clutch_engaged,
                damping_rate_zero_throttle_clutch_disengaged: simple_control
                    .damping_rate_zero_throttle_clutch_disengaged,
            },
            transmission: TransmissionPhysics {
                use_gear_autobox: simple_control.use_gear_autobox,
                gear_switch_time: simple_control.gear_switch_time,
                clutch_strength: simple_control.clutch_strength,
                final_ratio: simple_control.final_ratio,
            },
            mass: simple_control.mass,
            drag_coefficient: simple_control.drag_coefficient,
            center_of_mass: SimpleVector3D {
                x: simple_control.center_of_mass_x as f64,
                y: simple_control.center_of_mass_y as f64,
                z: simple_control.center_of_mass_z as f64,
            },
            steering: SteeringPhysics {
                curve_at_zero: simple_control.steering_curve_0,
                curve_at_one: simple_control.steering_curve_1,
            },
            use_sweep_wheel_collision: simple_control.use_sweep_wheel_collision,
        }
    }

    /// Get vehicle velocity vector
    pub fn get_velocity(&self) -> SimpleVector3D {
        ffi::Vehicle_GetVelocity(&self.inner)
    }

    /// Get vehicle angular velocity vector
    pub fn get_angular_velocity(&self) -> SimpleVector3D {
        ffi::Vehicle_GetAngularVelocity(&self.inner)
    }

    /// Get vehicle acceleration vector
    pub fn get_acceleration(&self) -> SimpleVector3D {
        ffi::Vehicle_GetAcceleration(&self.inner)
    }

    /// Get average tire friction
    pub fn get_tire_friction(&self) -> f32 {
        ffi::Vehicle_GetTireFriction(&self.inner)
    }

    /// Get engine RPM
    pub fn get_engine_rpm(&self) -> f32 {
        ffi::Vehicle_GetEngineRpm(&self.inner)
    }

    /// Get current gear ratio
    pub fn get_gear_ratio(&self) -> f32 {
        ffi::Vehicle_GetGearRatio(&self.inner)
    }

    /// Open a vehicle door (CARLA 0.10.0 feature)
    pub fn open_door(&self, door_type: VehicleDoorType) {
        ffi::Vehicle_OpenDoor(&self.inner, door_type as u32);
    }

    /// Close a vehicle door (CARLA 0.10.0 feature)
    pub fn close_door(&self, door_type: VehicleDoorType) {
        ffi::Vehicle_CloseDoor(&self.inner, door_type as u32);
    }

    /// Check if a vehicle door is open (CARLA 0.10.0 feature)
    ///
    /// **Note**: CARLA 0.10.0 does not provide a direct API to query door states.
    /// This function always returns false as a placeholder.
    pub fn is_door_open(&self, door_type: VehicleDoorType) -> bool {
        ffi::Vehicle_IsDoorOpen(&self.inner, door_type as u32)
    }

    /// Get all door states
    ///
    /// **Note**: CARLA 0.10.0 does not provide a direct API to query door states.
    /// This function returns an empty vector as the functionality is not available.
    pub fn get_door_states(&self) -> Vec<VehicleDoorState> {
        let simple_doors = ffi::Vehicle_GetDoorStates(&self.inner);
        simple_doors
            .into_iter()
            .map(|door| VehicleDoorState {
                door_type: VehicleDoorType::from_u32(door.door_type),
                is_open: door.is_open,
            })
            .collect()
    }

    /// Get wheel physics controls
    pub fn get_wheel_physics_controls(&self) -> Vec<WheelPhysicsControl> {
        let simple_wheels = ffi::Vehicle_GetWheelPhysicsControls(&self.inner);
        simple_wheels
            .into_iter()
            .map(|wheel| WheelPhysicsControl {
                tire_friction: wheel.tire_friction,
                damping_rate: wheel.damping_rate,
                max_steer_angle: wheel.max_steer_angle,
                radius: wheel.radius,
                max_brake_torque: wheel.max_brake_torque,
                max_handbrake_torque: wheel.max_handbrake_torque,
                position: SimpleVector3D {
                    x: wheel.position_x as f64,
                    y: wheel.position_y as f64,
                    z: wheel.position_z as f64,
                },
            })
            .collect()
    }

    /// Set wheel physics controls
    pub fn set_wheel_physics_controls(&self, wheels: &[WheelPhysicsControl]) -> Result<()> {
        let simple_wheels: Vec<SimpleWheelPhysicsControl> = wheels
            .iter()
            .map(|wheel| SimpleWheelPhysicsControl {
                tire_friction: wheel.tire_friction,
                damping_rate: wheel.damping_rate,
                max_steer_angle: wheel.max_steer_angle,
                radius: wheel.radius,
                max_brake_torque: wheel.max_brake_torque,
                max_handbrake_torque: wheel.max_handbrake_torque,
                position_x: wheel.position.x as f32,
                position_y: wheel.position.y as f32,
                position_z: wheel.position.z as f32,
            })
            .collect();
        ffi::Vehicle_SetWheelPhysicsControls(&self.inner, &simple_wheels);
        Ok(())
    }

    /// Get gear physics controls
    pub fn get_gear_physics_controls(&self) -> Vec<GearPhysicsControl> {
        let simple_gears = ffi::Vehicle_GetGearPhysicsControls(&self.inner);
        simple_gears
            .into_iter()
            .map(|gear| GearPhysicsControl {
                ratio: gear.ratio,
                down_ratio: gear.down_ratio,
                up_ratio: gear.up_ratio,
            })
            .collect()
    }

    /// Set gear physics controls
    pub fn set_gear_physics_controls(&self, gears: &[GearPhysicsControl]) -> Result<()> {
        let simple_gears: Vec<SimpleGearPhysicsControl> = gears
            .iter()
            .map(|gear| SimpleGearPhysicsControl {
                ratio: gear.ratio,
                down_ratio: gear.down_ratio,
                up_ratio: gear.up_ratio,
            })
            .collect();
        ffi::Vehicle_SetGearPhysicsControls(&self.inner, &simple_gears);
        Ok(())
    }
}

/// Vehicle control structure
#[derive(Debug, Clone, PartialEq)]
pub struct VehicleControl {
    /// Throttle input [0.0, 1.0]
    pub throttle: f32,
    /// Steering input [-1.0, 1.0]
    pub steer: f32,
    /// Brake input [0.0, 1.0]
    pub brake: f32,
    /// Hand brake enabled
    pub hand_brake: bool,
    /// Reverse gear enabled
    pub reverse: bool,
    /// Manual gear shift mode
    pub manual_gear_shift: bool,
    /// Current gear (-1=reverse, 0=neutral, 1+=forward)
    pub gear: i32,
}

impl Default for VehicleControl {
    fn default() -> Self {
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
}

bitflags::bitflags! {
    /// Vehicle light state flags
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
    pub struct VehicleLightState: u32 {
        const NONE = 0;
        const POSITION = 1;
        const LOW_BEAM = 2;
        const HIGH_BEAM = 4;
        const BRAKE = 8;
        const RIGHT_BLINKER = 16;
        const LEFT_BLINKER = 32;
        const REVERSE = 64;
        const FOG = 128;
        const INTERIOR = 256;
        const SPECIAL1 = 512;
        const SPECIAL2 = 1024;
        const ALL = 0xFFFFFFFF;
    }
}

impl VehicleControl {
    /// Create a new VehicleControl with default values
    pub fn new() -> Self {
        Self::default()
    }

    /// Set throttle value [0.0, 1.0]
    pub fn throttle(mut self, throttle: f32) -> Self {
        self.throttle = throttle.clamp(0.0, 1.0);
        self
    }

    /// Set steering value [-1.0, 1.0]
    pub fn steer(mut self, steer: f32) -> Self {
        self.steer = steer.clamp(-1.0, 1.0);
        self
    }

    /// Set brake value [0.0, 1.0]
    pub fn brake(mut self, brake: f32) -> Self {
        self.brake = brake.clamp(0.0, 1.0);
        self
    }

    /// Set hand brake
    pub fn hand_brake(mut self, hand_brake: bool) -> Self {
        self.hand_brake = hand_brake;
        self
    }

    /// Set reverse gear
    pub fn reverse(mut self, reverse: bool) -> Self {
        self.reverse = reverse;
        self
    }

    /// Set manual gear shift mode
    pub fn manual_gear_shift(mut self, manual: bool) -> Self {
        self.manual_gear_shift = manual;
        self
    }

    /// Set gear
    pub fn gear(mut self, gear: i32) -> Self {
        self.gear = gear;
        self
    }
}

/// Ackermann vehicle control for more precise steering
#[derive(Debug, Clone, PartialEq)]
pub struct AckermannControl {
    /// Steering angle in radians
    pub steer: f32,
    /// Steering speed in rad/s
    pub steer_speed: f32,
    /// Target speed in m/s
    pub speed: f32,
    /// Target acceleration in m/s²
    pub acceleration: f32,
    /// Target jerk in m/s³
    pub jerk: f32,
}

impl Default for AckermannControl {
    fn default() -> Self {
        Self {
            steer: 0.0,
            steer_speed: 0.0,
            speed: 0.0,
            acceleration: 0.0,
            jerk: 0.0,
        }
    }
}

impl AckermannControl {
    /// Create a new AckermannControl with default values
    pub fn new() -> Self {
        Self::default()
    }

    /// Set steering angle
    pub fn steer(mut self, steer: f32) -> Self {
        self.steer = steer;
        self
    }

    /// Set steering speed
    pub fn steer_speed(mut self, steer_speed: f32) -> Self {
        self.steer_speed = steer_speed;
        self
    }

    /// Set target speed
    pub fn speed(mut self, speed: f32) -> Self {
        self.speed = speed;
        self
    }

    /// Set target acceleration
    pub fn acceleration(mut self, acceleration: f32) -> Self {
        self.acceleration = acceleration;
        self
    }

    /// Set target jerk
    pub fn jerk(mut self, jerk: f32) -> Self {
        self.jerk = jerk;
        self
    }
}

/// Complete vehicle physics control
#[derive(Debug, Clone, PartialEq)]
pub struct VehiclePhysicsControl {
    /// Engine physics parameters
    pub engine: EnginePhysics,
    /// Transmission physics parameters
    pub transmission: TransmissionPhysics,
    /// Vehicle mass in kg
    pub mass: f32,
    /// Drag coefficient
    pub drag_coefficient: f32,
    /// Center of mass position
    pub center_of_mass: SimpleVector3D,
    /// Steering physics parameters
    pub steering: SteeringPhysics,
    /// Use sweep wheel collision detection
    pub use_sweep_wheel_collision: bool,
}

/// Engine physics parameters
#[derive(Debug, Clone, PartialEq)]
pub struct EnginePhysics {
    /// Torque curve maximum RPM
    pub torque_curve_max_rpm: f32,
    /// Torque curve maximum torque in Nm
    pub torque_curve_max_torque_nm: f32,
    /// Maximum RPM
    pub max_rpm: f32,
    /// Moment of inertia in kg·m²
    pub moi: f32,
    /// Damping rate at full throttle
    pub damping_rate_full_throttle: f32,
    /// Damping rate at zero throttle with clutch engaged
    pub damping_rate_zero_throttle_clutch_engaged: f32,
    /// Damping rate at zero throttle with clutch disengaged
    pub damping_rate_zero_throttle_clutch_disengaged: f32,
}

/// Transmission physics parameters
#[derive(Debug, Clone, PartialEq)]
pub struct TransmissionPhysics {
    /// Use automatic gear box
    pub use_gear_autobox: bool,
    /// Gear switch time in seconds
    pub gear_switch_time: f32,
    /// Clutch strength
    pub clutch_strength: f32,
    /// Final gear ratio
    pub final_ratio: f32,
}

/// Steering physics parameters
#[derive(Debug, Clone, PartialEq)]
pub struct SteeringPhysics {
    /// Steering curve value at 0 speed
    pub curve_at_zero: f32,
    /// Steering curve value at maximum speed
    pub curve_at_one: f32,
}

/// Individual wheel physics control
#[derive(Debug, Clone, PartialEq)]
pub struct WheelPhysicsControl {
    /// Tire friction coefficient
    pub tire_friction: f32,
    /// Damping rate
    pub damping_rate: f32,
    /// Maximum steering angle in radians
    pub max_steer_angle: f32,
    /// Wheel radius in meters
    pub radius: f32,
    /// Maximum brake torque in Nm
    pub max_brake_torque: f32,
    /// Maximum handbrake torque in Nm
    pub max_handbrake_torque: f32,
    /// Wheel position relative to vehicle center
    pub position: SimpleVector3D,
}

/// Individual gear physics control
#[derive(Debug, Clone, PartialEq)]
pub struct GearPhysicsControl {
    /// Gear ratio
    pub ratio: f32,
    /// Down shift ratio threshold
    pub down_ratio: f32,
    /// Up shift ratio threshold
    pub up_ratio: f32,
}

/// Vehicle door types (CARLA 0.10.0)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u32)]
pub enum VehicleDoorType {
    /// Front left door
    FrontLeft = 0,
    /// Front right door
    FrontRight = 1,
    /// Rear left door
    RearLeft = 2,
    /// Rear right door
    RearRight = 3,
    /// All doors
    All = 4,
}

impl VehicleDoorType {
    /// Convert from u32 value
    pub fn from_u32(value: u32) -> Self {
        match value {
            0 => VehicleDoorType::FrontLeft,
            1 => VehicleDoorType::FrontRight,
            2 => VehicleDoorType::RearLeft,
            3 => VehicleDoorType::RearRight,
            4 => VehicleDoorType::All,
            _ => VehicleDoorType::FrontLeft, // Default fallback
        }
    }
}

/// Vehicle door state
#[derive(Debug, Clone, PartialEq)]
pub struct VehicleDoorState {
    /// Door type
    pub door_type: VehicleDoorType,
    /// Whether the door is open
    pub is_open: bool,
}

impl Default for VehiclePhysicsControl {
    fn default() -> Self {
        Self {
            engine: EnginePhysics {
                torque_curve_max_rpm: 5000.0,
                torque_curve_max_torque_nm: 300.0,
                max_rpm: 5000.0,
                moi: 1.0,
                damping_rate_full_throttle: 0.15,
                damping_rate_zero_throttle_clutch_engaged: 2.0,
                damping_rate_zero_throttle_clutch_disengaged: 0.35,
            },
            transmission: TransmissionPhysics {
                use_gear_autobox: true,
                gear_switch_time: 0.5,
                clutch_strength: 10.0,
                final_ratio: 4.0,
            },
            mass: 1000.0,
            drag_coefficient: 0.3,
            center_of_mass: SimpleVector3D {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            steering: SteeringPhysics {
                curve_at_zero: 1.0,
                curve_at_one: 1.0,
            },
            use_sweep_wheel_collision: false,
        }
    }
}

impl Default for WheelPhysicsControl {
    fn default() -> Self {
        Self {
            tire_friction: 2.0,
            damping_rate: 0.25,
            max_steer_angle: 0.6108,
            radius: 0.3,
            max_brake_torque: 1500.0,
            max_handbrake_torque: 3000.0,
            position: SimpleVector3D {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        }
    }
}

impl Default for GearPhysicsControl {
    fn default() -> Self {
        Self {
            ratio: 1.0,
            down_ratio: 0.5,
            up_ratio: 0.65,
        }
    }
}
