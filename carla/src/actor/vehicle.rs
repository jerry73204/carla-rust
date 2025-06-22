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
        let actor_ref = actor.inner_actor();
        if let Some(vehicle_wrapper) = carla_sys::VehicleWrapper::from_actor(actor_ref) {
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

    pub fn set_autopilot(
        &self,
        enabled: bool,
        traffic_manager_port: Option<u16>,
    ) -> CarlaResult<()> {
        let tm_port = traffic_manager_port.unwrap_or(8000);
        self.inner.set_autopilot(enabled, tm_port);
        Ok(())
    }

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

    pub fn set_light_state(&self, light_state: VehicleLightState) -> CarlaResult<()> {
        let cxx_light_state = carla_sys::VehicleLightState::from_bits_truncate(light_state.lights);
        self.inner.set_light_state(cxx_light_state);
        Ok(())
    }

    pub fn light_state(&self) -> VehicleLightState {
        let cxx_light_state = self.inner.get_light_state();
        VehicleLightState {
            lights: cxx_light_state.bits(),
        }
    }

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
