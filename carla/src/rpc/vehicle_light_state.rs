//! Vehicle light state bitflags.
//!
//! Provides a type-safe wrapper around CARLA's VehicleLightState enum, which
//! is used as bitflags to control multiple vehicle lights simultaneously.
//!
//! Corresponds to [`carla.VehicleLightState`](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.VehicleLightState) in the Python API.
//!
//! # Examples
//!
//! ```no_run
//! use carla::rpc::VehicleLightState;
//!
//! // Turn on position and low beam lights
//! let lights = VehicleLightState::POSITION | VehicleLightState::LOW_BEAM;
//!
//! // Check if a specific light is on
//! if lights.contains(VehicleLightState::LOW_BEAM) {
//!     println!("Low beams are on");
//! }
//!
//! // Toggle high beam
//! let lights = lights ^ VehicleLightState::HIGH_BEAM;
//!
//! // Turn off position light
//! let lights = lights & !VehicleLightState::POSITION;
//! ```

use bitflags::bitflags;
use carla_sys::carla::rpc::VehicleLightState_LightState as FfiVehicleLightState;

bitflags! {
    /// Vehicle light state flags.
    ///
    /// Each flag corresponds to a specific vehicle light. Multiple flags can be
    /// combined using bitwise operations to control multiple lights simultaneously.
    ///
    /// # FFI Safety
    ///
    /// This type uses `#[repr(transparent)]` (via bitflags macro) to ensure it has
    /// the same memory layout as the underlying `u32`, making it safe to convert
    /// to/from the FFI type `VehicleLightState_LightState`.
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
    pub struct VehicleLightState: u32 {
        /// No lights (all off)
        const NONE         = 0x0;

        /// Position/parking lights
        const POSITION     = 0x1;

        /// Low beam headlights
        const LOW_BEAM     = 0x2;

        /// High beam headlights
        const HIGH_BEAM    = 0x4;

        /// Brake lights
        const BRAKE        = 0x8;

        /// Right turn signal/blinker
        const RIGHT_BLINKER = 0x10;

        /// Left turn signal/blinker
        const LEFT_BLINKER = 0x20;

        /// Reverse lights
        const REVERSE      = 0x40;

        /// Fog lights
        const FOG          = 0x80;

        /// Interior lights
        const INTERIOR     = 0x100;

        /// Special light 1 (vehicle-specific)
        const SPECIAL1     = 0x200;

        /// Special light 2 (vehicle-specific)
        const SPECIAL2     = 0x400;

        /// All lights on (all flags set)
        const ALL          = 0xFFFFFFFF;
    }
}

impl VehicleLightState {
    /// Convert to FFI type for passing to C++ API.
    ///
    /// # Safety
    ///
    /// This function uses `MaybeUninit` to construct the FFI type without
    /// triggering Rust's enum validation, since `VehicleLightState_LightState`
    /// is a POD type that wraps a u32 bitfield.
    pub(crate) fn to_ffi(self) -> FfiVehicleLightState {
        unsafe {
            let mut uninit = std::mem::MaybeUninit::<FfiVehicleLightState>::uninit();
            std::ptr::write(uninit.as_mut_ptr() as *mut u32, self.bits());
            uninit.assume_init()
        }
    }

    /// Create from FFI type received from C++ API.
    ///
    /// Uses `from_bits_truncate` to handle any bits that don't correspond
    /// to defined flags (e.g., reserved or unknown bits).
    pub(crate) fn from_ffi(ffi: &FfiVehicleLightState) -> Self {
        unsafe {
            let bits = std::ptr::read(ffi as *const _ as *const u32);
            VehicleLightState::from_bits_truncate(bits)
        }
    }
}

impl From<VehicleLightState> for FfiVehicleLightState {
    fn from(state: VehicleLightState) -> Self {
        state.to_ffi()
    }
}

impl From<&VehicleLightState> for FfiVehicleLightState {
    fn from(state: &VehicleLightState) -> Self {
        (*state).to_ffi()
    }
}

impl From<FfiVehicleLightState> for VehicleLightState {
    fn from(ffi: FfiVehicleLightState) -> Self {
        VehicleLightState::from_ffi(&ffi)
    }
}

impl From<&FfiVehicleLightState> for VehicleLightState {
    fn from(ffi: &FfiVehicleLightState) -> Self {
        VehicleLightState::from_ffi(ffi)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bitflags_operations() {
        // Combine multiple lights
        let lights = VehicleLightState::POSITION | VehicleLightState::LOW_BEAM;
        assert!(lights.contains(VehicleLightState::POSITION));
        assert!(lights.contains(VehicleLightState::LOW_BEAM));
        assert!(!lights.contains(VehicleLightState::HIGH_BEAM));

        // Toggle a light
        let lights = lights ^ VehicleLightState::HIGH_BEAM;
        assert!(lights.contains(VehicleLightState::HIGH_BEAM));

        // Turn off a light
        let lights = lights & !VehicleLightState::POSITION;
        assert!(!lights.contains(VehicleLightState::POSITION));
        assert!(lights.contains(VehicleLightState::LOW_BEAM));
    }

    #[test]
    fn test_bit_values() {
        // Verify bit values match CARLA specification
        assert_eq!(VehicleLightState::NONE.bits(), 0x0);
        assert_eq!(VehicleLightState::POSITION.bits(), 0x1);
        assert_eq!(VehicleLightState::LOW_BEAM.bits(), 0x2);
        assert_eq!(VehicleLightState::HIGH_BEAM.bits(), 0x4);
        assert_eq!(VehicleLightState::BRAKE.bits(), 0x8);
        assert_eq!(VehicleLightState::RIGHT_BLINKER.bits(), 0x10);
        assert_eq!(VehicleLightState::LEFT_BLINKER.bits(), 0x20);
        assert_eq!(VehicleLightState::REVERSE.bits(), 0x40);
        assert_eq!(VehicleLightState::FOG.bits(), 0x80);
        assert_eq!(VehicleLightState::INTERIOR.bits(), 0x100);
        assert_eq!(VehicleLightState::SPECIAL1.bits(), 0x200);
        assert_eq!(VehicleLightState::SPECIAL2.bits(), 0x400);
        assert_eq!(VehicleLightState::ALL.bits(), 0xFFFFFFFF);
    }

    #[test]
    fn test_from_bits() {
        // Test creating from raw bits
        let lights = VehicleLightState::from_bits_truncate(0x3);
        assert!(lights.contains(VehicleLightState::POSITION));
        assert!(lights.contains(VehicleLightState::LOW_BEAM));
    }
}
