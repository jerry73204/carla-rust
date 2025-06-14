//! Vehicle actor implementation for CARLA.

use crate::ffi::{self, Actor, SimpleVehicleControl, Vehicle};
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
