//! Vehicle control commands and physics.

use crate::geom::Vector3D;

/// Vehicle control commands.
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
    /// Manual gear shift enabled
    pub manual_gear_shift: bool,
    /// Gear selection (when manual_gear_shift is true)
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

/// Vehicle physics control parameters.
#[derive(Debug, Clone, PartialEq)]
pub struct VehiclePhysicsControl {
    /// Torque curve points
    pub torque_curve: Vec<(f32, f32)>,
    /// Maximum RPM
    pub max_rpm: f32,
    /// Moment of inertia
    pub moi: f32,
    /// Damping rate (full throttle)
    pub damping_rate_full_throttle: f32,
    /// Damping rate (zero throttle)
    pub damping_rate_zero_throttle_clutch_engaged: f32,
    /// Damping rate (zero throttle, clutch disengaged)
    pub damping_rate_zero_throttle_clutch_disengaged: f32,
    /// Use gear auto box
    pub use_gear_autobox: bool,
    /// Gear switch time
    pub gear_switch_time: f32,
    /// Clutch strength
    pub clutch_strength: f32,
    /// Final ratio
    pub final_ratio: f32,
    /// Forward gears
    pub forward_gears: Vec<f32>,
    /// Mass in kg
    pub mass: f32,
    /// Drag coefficient
    pub drag_coefficient: f32,
    /// Center of mass
    pub center_of_mass: Vector3D,
}

impl Default for VehiclePhysicsControl {
    fn default() -> Self {
        Self {
            torque_curve: vec![(0.0, 500.0), (5000.0, 500.0)],
            max_rpm: 5000.0,
            moi: 1.0,
            damping_rate_full_throttle: 0.15,
            damping_rate_zero_throttle_clutch_engaged: 2.0,
            damping_rate_zero_throttle_clutch_disengaged: 0.35,
            use_gear_autobox: true,
            gear_switch_time: 0.5,
            clutch_strength: 10.0,
            final_ratio: 4.0,
            forward_gears: vec![2.85, 1.99, 1.51, 1.17, 1.0],
            mass: 1000.0,
            drag_coefficient: 0.3,
            center_of_mass: Vector3D::zero(),
        }
    }
}

/// Vehicle light state flags.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct VehicleLightState {
    /// Bitfield representing active lights
    pub lights: u32,
}

impl VehicleLightState {
    /// No lights
    pub const NONE: u32 = 0;
    /// Position lights
    pub const POSITION: u32 = 1;
    /// Low beam
    pub const LOW_BEAM: u32 = 2;
    /// High beam
    pub const HIGH_BEAM: u32 = 4;
    /// Brake lights
    pub const BRAKE: u32 = 8;
    /// Reverse lights
    pub const REVERSE: u32 = 16;
    /// Fog lights
    pub const FOG: u32 = 32;
    /// Interior lights
    pub const INTERIOR: u32 = 64;
    /// Special 1
    pub const SPECIAL1: u32 = 128;
    /// Special 2
    pub const SPECIAL2: u32 = 256;
    /// All lights
    pub const ALL: u32 = 0xFFFFFFFF;
}

impl Default for VehicleLightState {
    fn default() -> Self {
        Self { lights: Self::NONE }
    }
}

/// Vehicle door types (CARLA 0.10.0).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VehicleDoorType {
    /// Front left door
    FrontLeft,
    /// Front right door
    FrontRight,
    /// Rear left door
    RearLeft,
    /// Rear right door
    RearRight,
    /// All doors
    All,
}

/// Vehicle telemetry data.
#[derive(Debug, Clone, PartialEq)]
pub struct VehicleTelemetryData {
    /// Current speed in km/h
    pub speed: f32,
    /// Current RPM
    pub rpm: f32,
    /// Current gear
    pub gear: i32,
    /// Engine temperature
    pub engine_temperature: f32,
    /// Fuel level [0.0, 1.0]
    pub fuel_level: f32,
}

impl Default for VehicleTelemetryData {
    fn default() -> Self {
        Self {
            speed: 0.0,
            rpm: 0.0,
            gear: 0,
            engine_temperature: 90.0,
            fuel_level: 1.0,
        }
    }
}
