//! Vehicle actor implementation.

use crate::{
    actor::{Actor, ActorFfi, ActorId},
    error::CarlaResult,
    geom::Vector3D,
};

/// Vehicle actor.
#[derive(Debug)]
pub struct Vehicle {
    /// Actor ID
    id: ActorId,
    /// Internal vehicle wrapper for FFI calls
    inner: carla_sys::VehicleWrapper,
}

impl Vehicle {
    /// Create a vehicle from a carla-sys VehicleWrapper.
    pub(crate) fn from_cxx(vehicle_wrapper: carla_sys::VehicleWrapper) -> CarlaResult<Self> {
        // Get the vehicle's actor ID
        let id = vehicle_wrapper.get_id();

        Ok(Self {
            id,
            inner: vehicle_wrapper,
        })
    }

    /// Create a vehicle from an actor by casting.
    pub fn from_actor(actor: Actor) -> Result<Self, Actor> {
        if let Some(vehicle_wrapper) =
            carla_sys::VehicleWrapper::from_actor(actor.inner_wrapper().get_shared_ptr())
        {
            let id = vehicle_wrapper.get_id();

            Ok(Self {
                id,
                inner: vehicle_wrapper,
            })
        } else {
            Err(actor)
        }
    }

    /// Get the vehicle's actor ID.
    pub fn id(&self) -> ActorId {
        self.id
    }

    /// Get access to the underlying VehicleWrapper for FFI calls.
    pub(crate) fn inner(&self) -> &carla_sys::VehicleWrapper {
        &self.inner
    }

    /// Get access to the underlying FFI Vehicle type for direct FFI calls.
    pub(crate) fn as_ffi(&self) -> &carla_sys::ffi::Vehicle {
        self.inner.inner().as_ref().unwrap()
    }

    /// Get the vehicle's wheel physics parameters.
    pub fn wheel_steer_angle(&self, wheel_location: WheelLocation) -> f32 {
        use carla_sys::vehicle::VehicleWheelLocation;

        let cxx_wheel_location = match wheel_location {
            WheelLocation::FrontLeft => VehicleWheelLocation::FrontLeft,
            WheelLocation::FrontRight => VehicleWheelLocation::FrontRight,
            WheelLocation::RearLeft => VehicleWheelLocation::RearLeft,
            WheelLocation::RearRight => VehicleWheelLocation::RearRight,
        };
        self.inner.get_wheel_steer_angle(cxx_wheel_location)
    }

    /// Get the vehicle's forward speed in km/h.
    pub fn speed(&self) -> f32 {
        // VehicleWrapper returns speed in m/s, convert to km/h
        self.inner.get_speed() * 3.6
    }

    /// Convert this vehicle to a generic Actor.
    ///
    /// This creates a new Actor instance that represents the same vehicle.
    /// This is useful when you need to work with generic actor functionality.
    pub fn into_actor(self) -> Actor {
        let actor_wrapper = self.inner.as_actor_wrapper();
        Actor::from_cxx(actor_wrapper)
    }

    /// Open or close a specific door.
    pub fn open_door(&self, door_type: VehicleDoorType) -> CarlaResult<()> {
        self.set_door_state(door_type, true)
    }

    /// Close a specific door.
    pub fn close_door(&self, door_type: VehicleDoorType) -> CarlaResult<()> {
        self.set_door_state(door_type, false)
    }
}

impl ActorFfi for Vehicle {
    fn as_actor_ffi(&self) -> carla_sys::ActorWrapper {
        // Create ActorWrapper from VehicleWrapper on-demand
        self.inner.as_actor_wrapper()
    }
}

impl Vehicle {
    /// Apply control commands to the vehicle.
    ///
    /// # Arguments
    /// * `control` - The vehicle control configuration to apply
    ///
    /// # Returns
    /// Returns `Ok(())` if the control was applied successfully, or an error if it failed.
    pub fn apply_control(&self, control: &VehicleControl) -> CarlaResult<()> {
        // Convert high-level VehicleControl to carla-sys VehicleControl
        let cxx_control = carla_sys::VehicleControl {
            throttle: control.throttle,
            steer: control.steer,
            brake: control.brake,
            hand_brake: control.hand_brake,
            reverse: control.reverse,
            manual_gear_shift: control.manual_gear_shift,
            gear: control.gear,
        };
        self.inner.apply_control(&cxx_control).map_err(|e| {
            crate::error::CarlaError::Vehicle(crate::error::VehicleError::ControlFailed(
                e.to_string(),
            ))
        })
    }

    /// Get the current control configuration of the vehicle.
    ///
    /// # Returns
    /// Returns the current vehicle control state.
    pub fn control(&self) -> VehicleControl {
        let cxx_control = self.inner.get_control();
        VehicleControl {
            throttle: cxx_control.throttle,
            steer: cxx_control.steer,
            brake: cxx_control.brake,
            hand_brake: cxx_control.hand_brake,
            reverse: cxx_control.reverse,
            manual_gear_shift: cxx_control.manual_gear_shift,
            gear: cxx_control.gear,
        }
    }

    /// Enable or disable autopilot for this vehicle.
    ///
    /// # Arguments
    /// * `enabled` - Whether to enable (true) or disable (false) autopilot
    /// * `traffic_manager_port` - Optional port for the traffic manager (defaults to 8000)
    ///
    /// # Returns
    /// Returns `Ok(())` if autopilot was set successfully.
    pub fn set_autopilot(
        &self,
        enabled: bool,
        traffic_manager_port: Option<u16>,
    ) -> CarlaResult<()> {
        let tm_port = traffic_manager_port.unwrap_or(8000);
        self.inner.set_autopilot(enabled, tm_port);
        Ok(())
    }

    /// Try to enable or disable autopilot with Traffic Manager availability check.
    ///
    /// This is a safer version that handles cases where Traffic Manager is not available.
    ///
    /// # Arguments
    /// * `enabled` - Whether to enable (true) or disable (false) autopilot
    /// * `traffic_manager_port` - Optional port for the traffic manager (defaults to 8000)
    ///
    /// # Returns
    /// Returns `Ok(())` if autopilot was set successfully, or an error if Traffic Manager
    /// is not available or operation failed.
    pub fn try_set_autopilot(
        &self,
        enabled: bool,
        traffic_manager_port: Option<u16>,
    ) -> CarlaResult<()> {
        // If disabling, just call the normal method
        if !enabled {
            return self.set_autopilot(false, traffic_manager_port);
        }

        // For enabling, we need to be more careful
        // TODO: Add Traffic Manager availability check when FFI is available
        // For now, we'll try and catch any crashes
        eprintln!(
            "Warning: Enabling autopilot without Traffic Manager may crash CARLA 0.10.0 server"
        );

        // Try to enable with a warning
        match std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            let tm_port = traffic_manager_port.unwrap_or(8000);
            self.inner.set_autopilot(enabled, tm_port);
        })) {
            Ok(_) => Ok(()),
            Err(_) => Err(crate::error::CarlaError::Actor(
                crate::error::ActorError::TrafficManagerUnavailable,
            )),
        }
    }

    /// Get the current physics control configuration of the vehicle.
    ///
    /// # Returns
    /// Returns the current vehicle physics control parameters.
    pub fn physics_control(&self) -> VehiclePhysicsControl {
        let cxx_physics = self.inner.get_physics_control();

        // Convert carla-sys VehiclePhysicsControl to high-level VehiclePhysicsControl
        VehiclePhysicsControl {
            torque_curve: vec![
                (0.0, cxx_physics.engine.torque_curve_max_torque_nm),
                (
                    cxx_physics.engine.torque_curve_max_rpm,
                    cxx_physics.engine.torque_curve_max_torque_nm,
                ),
            ],
            max_rpm: cxx_physics.engine.max_rpm,
            moi: cxx_physics.engine.moi,
            damping_rate_full_throttle: cxx_physics.engine.damping_rate_full_throttle,
            damping_rate_zero_throttle_clutch_engaged: cxx_physics
                .engine
                .damping_rate_zero_throttle_clutch_engaged,
            damping_rate_zero_throttle_clutch_disengaged: cxx_physics
                .engine
                .damping_rate_zero_throttle_clutch_disengaged,
            use_gear_autobox: cxx_physics.transmission.use_gear_autobox,
            gear_switch_time: cxx_physics.transmission.gear_switch_time,
            clutch_strength: cxx_physics.transmission.clutch_strength,
            final_ratio: cxx_physics.transmission.final_ratio,
            forward_gears: vec![2.85, 1.99, 1.51, 1.17, 1.0], // Default values since not available in simple struct
            mass: cxx_physics.mass,
            drag_coefficient: cxx_physics.drag_coefficient,
            center_of_mass: crate::geom::Vector3D::new(
                cxx_physics.center_of_mass.x as f32,
                cxx_physics.center_of_mass.y as f32,
                cxx_physics.center_of_mass.z as f32,
            ),
        }
    }

    /// Apply physics control configuration to the vehicle.
    ///
    /// # Arguments
    /// * `physics_control` - The physics control configuration to apply
    ///
    /// # Returns
    /// Returns `Ok(())` if the physics control was applied successfully.
    pub fn apply_physics_control(
        &self,
        physics_control: &VehiclePhysicsControl,
    ) -> CarlaResult<()> {
        let cxx_physics = carla_sys::VehiclePhysicsControl {
            engine: carla_sys::EnginePhysics {
                torque_curve_max_rpm: physics_control.max_rpm,
                torque_curve_max_torque_nm: physics_control
                    .torque_curve
                    .last()
                    .map(|(_, torque)| *torque)
                    .unwrap_or(300.0),
                max_rpm: physics_control.max_rpm,
                moi: physics_control.moi,
                damping_rate_full_throttle: physics_control.damping_rate_full_throttle,
                damping_rate_zero_throttle_clutch_engaged: physics_control
                    .damping_rate_zero_throttle_clutch_engaged,
                damping_rate_zero_throttle_clutch_disengaged: physics_control
                    .damping_rate_zero_throttle_clutch_disengaged,
            },
            transmission: carla_sys::TransmissionPhysics {
                use_gear_autobox: physics_control.use_gear_autobox,
                gear_switch_time: physics_control.gear_switch_time,
                clutch_strength: physics_control.clutch_strength,
                final_ratio: physics_control.final_ratio,
            },
            mass: physics_control.mass,
            drag_coefficient: physics_control.drag_coefficient,
            center_of_mass: carla_sys::SimpleVector3D {
                x: physics_control.center_of_mass.x as f64,
                y: physics_control.center_of_mass.y as f64,
                z: physics_control.center_of_mass.z as f64,
            },
            steering: carla_sys::SteeringPhysics {
                curve_at_zero: 1.0,
                curve_at_one: 1.0,
            },
            use_sweep_wheel_collision: false,
        };

        self.inner.apply_physics_control(&cxx_physics).map_err(|e| {
            crate::error::CarlaError::Vehicle(crate::error::VehicleError::PhysicsControlFailed(
                e.to_string(),
            ))
        })
    }

    /// Set the light state of the vehicle.
    ///
    /// # Arguments
    /// * `light_state` - The light state configuration to apply
    ///
    /// # Returns
    /// Returns `Ok(())` if the light state was set successfully.
    pub fn set_light_state(&self, light_state: VehicleLightState) -> CarlaResult<()> {
        let cxx_light_state = carla_sys::VehicleLightState::from_bits_truncate(light_state.lights);
        self.inner.set_light_state(cxx_light_state);
        Ok(())
    }

    /// Get the current light state of the vehicle.
    ///
    /// # Returns
    /// Returns the current vehicle light state.
    pub fn light_state(&self) -> VehicleLightState {
        let cxx_light_state = self.inner.get_light_state();
        VehicleLightState {
            lights: cxx_light_state.bits(),
        }
    }

    /// Set the door state of the vehicle.
    ///
    /// # Arguments
    /// * `door_type` - The type of door to control
    /// * `is_open` - Whether to open (true) or close (false) the door
    ///
    /// # Returns
    /// Returns `Ok(())` if the door state was set successfully.
    pub fn set_door_state(&self, door_type: VehicleDoorType, is_open: bool) -> CarlaResult<()> {
        let cxx_door_type = match door_type {
            VehicleDoorType::FrontLeft => carla_sys::VehicleDoorType::FrontLeft,
            VehicleDoorType::FrontRight => carla_sys::VehicleDoorType::FrontRight,
            VehicleDoorType::RearLeft => carla_sys::VehicleDoorType::RearLeft,
            VehicleDoorType::RearRight => carla_sys::VehicleDoorType::RearRight,
            VehicleDoorType::All => carla_sys::VehicleDoorType::All,
        };

        if is_open {
            self.inner.open_door(cxx_door_type);
        } else {
            self.inner.close_door(cxx_door_type);
        }
        Ok(())
    }

    /// Get the current telemetry data from the vehicle.
    ///
    /// # Returns
    /// Returns the current vehicle telemetry data including speed, RPM, and gear information.
    pub fn telemetry_data(&self) -> VehicleTelemetryData {
        let cxx_telemetry = self.inner.get_telemetry_data();
        VehicleTelemetryData {
            speed: cxx_telemetry.speed,
            rpm: cxx_telemetry.rpm,
            gear: cxx_telemetry.gear,
            engine_temperature: cxx_telemetry.engine_temperature,
            fuel_level: cxx_telemetry.fuel_level,
        }
    }
}

impl Drop for Vehicle {
    fn drop(&mut self) {
        // Only destroy if the vehicle is still alive
        if self.inner.is_alive() {
            let _ = self.inner.destroy();
        }
    }
}

/// Vehicle wheel locations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WheelLocation {
    /// Front left wheel
    FrontLeft,
    /// Front right wheel
    FrontRight,
    /// Rear left wheel
    RearLeft,
    /// Rear right wheel
    RearRight,
}

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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geom::Vector3D;

    #[test]
    fn test_wheel_location_enum() {
        // Test that all wheel locations are defined
        let locations = [
            WheelLocation::FrontLeft,
            WheelLocation::FrontRight,
            WheelLocation::RearLeft,
            WheelLocation::RearRight,
        ];

        assert_eq!(locations.len(), 4);

        // Test Debug output
        assert_eq!(format!("{:?}", WheelLocation::FrontLeft), "FrontLeft");
        assert_eq!(format!("{:?}", WheelLocation::RearRight), "RearRight");
    }

    #[test]
    fn test_vehicle_control_default() {
        let control = VehicleControl::default();

        assert_eq!(control.throttle, 0.0);
        assert_eq!(control.steer, 0.0);
        assert_eq!(control.brake, 0.0);
        assert!(!control.hand_brake);
        assert!(!control.reverse);
        assert!(!control.manual_gear_shift);
        assert_eq!(control.gear, 0);
    }

    #[test]
    fn test_vehicle_control_custom() {
        let control = VehicleControl {
            throttle: 0.8,
            steer: -0.3,
            brake: 0.0,
            hand_brake: false,
            reverse: false,
            manual_gear_shift: true,
            gear: 3,
        };

        assert_eq!(control.throttle, 0.8);
        assert_eq!(control.steer, -0.3);
        assert_eq!(control.gear, 3);
        assert!(control.manual_gear_shift);
    }

    #[test]
    fn test_vehicle_physics_control_default() {
        let physics = VehiclePhysicsControl::default();

        assert_eq!(physics.max_rpm, 5000.0);
        assert_eq!(physics.mass, 1000.0);
        assert_eq!(physics.drag_coefficient, 0.3);
        assert!(physics.use_gear_autobox);
        assert_eq!(physics.forward_gears.len(), 5);
        assert_eq!(physics.forward_gears[0], 2.85);
        assert_eq!(physics.torque_curve.len(), 2);
    }

    #[test]
    fn test_vehicle_light_state_constants() {
        assert_eq!(VehicleLightState::NONE, 0);
        assert_eq!(VehicleLightState::POSITION, 1);
        assert_eq!(VehicleLightState::LOW_BEAM, 2);
        assert_eq!(VehicleLightState::HIGH_BEAM, 4);
        assert_eq!(VehicleLightState::BRAKE, 8);
        assert_eq!(VehicleLightState::REVERSE, 16);
        assert_eq!(VehicleLightState::FOG, 32);
        assert_eq!(VehicleLightState::INTERIOR, 64);
        assert_eq!(VehicleLightState::ALL, 0xFFFFFFFF);
    }

    #[test]
    fn test_vehicle_light_state_bitfield() {
        let light_state = VehicleLightState {
            lights: VehicleLightState::LOW_BEAM | VehicleLightState::POSITION,
        };

        assert_eq!(light_state.lights, 3); // 1 + 2

        // Test that we can check individual lights
        assert!(light_state.lights & VehicleLightState::LOW_BEAM != 0);
        assert!(light_state.lights & VehicleLightState::POSITION != 0);
        assert!(light_state.lights & VehicleLightState::HIGH_BEAM == 0);
    }

    #[test]
    fn test_vehicle_light_state_default() {
        let light_state = VehicleLightState::default();
        assert_eq!(light_state.lights, VehicleLightState::NONE);
    }

    #[test]
    fn test_vehicle_door_type_enum() {
        let door_types = [
            VehicleDoorType::FrontLeft,
            VehicleDoorType::FrontRight,
            VehicleDoorType::RearLeft,
            VehicleDoorType::RearRight,
            VehicleDoorType::All,
        ];

        assert_eq!(door_types.len(), 5);

        // Test Debug output
        assert_eq!(format!("{:?}", VehicleDoorType::FrontLeft), "FrontLeft");
        assert_eq!(format!("{:?}", VehicleDoorType::All), "All");
    }

    #[test]
    fn test_vehicle_telemetry_data_default() {
        let telemetry = VehicleTelemetryData::default();

        assert_eq!(telemetry.speed, 0.0);
        assert_eq!(telemetry.rpm, 0.0);
        assert_eq!(telemetry.gear, 0);
        assert_eq!(telemetry.engine_temperature, 90.0);
        assert_eq!(telemetry.fuel_level, 1.0);
    }

    #[test]
    fn test_vehicle_telemetry_data_custom() {
        let telemetry = VehicleTelemetryData {
            speed: 65.5,
            rpm: 2500.0,
            gear: 4,
            engine_temperature: 95.5,
            fuel_level: 0.8,
        };

        assert_eq!(telemetry.speed, 65.5);
        assert_eq!(telemetry.rpm, 2500.0);
        assert_eq!(telemetry.gear, 4);
        assert_eq!(telemetry.engine_temperature, 95.5);
        assert_eq!(telemetry.fuel_level, 0.8);
    }

    #[test]
    fn test_vehicle_physics_control_realistic_values() {
        // Test realistic sports car values
        let physics = VehiclePhysicsControl {
            mass: 1200.0,
            max_rpm: 7000.0,
            drag_coefficient: 0.25,
            torque_curve: vec![(0.0, 400.0), (4000.0, 600.0), (7000.0, 450.0)],
            ..Default::default()
        };

        assert_eq!(physics.mass, 1200.0);
        assert_eq!(physics.max_rpm, 7000.0);
        assert_eq!(physics.torque_curve.len(), 3);
        assert_eq!(physics.torque_curve[1], (4000.0, 600.0));
    }

    #[test]
    fn test_vector3d_center_of_mass() {
        let physics = VehiclePhysicsControl {
            center_of_mass: Vector3D::new(0.0, 0.0, -0.5),
            ..Default::default()
        };

        assert_eq!(physics.center_of_mass.x, 0.0);
        assert_eq!(physics.center_of_mass.y, 0.0);
        assert_eq!(physics.center_of_mass.z, -0.5);
    }

    #[test]
    fn test_vehicle_light_state_combinations() {
        // Test common light combinations
        let headlights = VehicleLightState {
            lights: VehicleLightState::POSITION | VehicleLightState::LOW_BEAM,
        };

        let brake_lights = VehicleLightState {
            lights: VehicleLightState::POSITION | VehicleLightState::BRAKE,
        };

        let hazard_lights = VehicleLightState {
            lights: VehicleLightState::POSITION | VehicleLightState::BRAKE | VehicleLightState::FOG,
        };

        assert_eq!(headlights.lights, 3); // 1 + 2
        assert_eq!(brake_lights.lights, 9); // 1 + 8
        assert_eq!(hazard_lights.lights, 41); // 1 + 8 + 32
    }
}
