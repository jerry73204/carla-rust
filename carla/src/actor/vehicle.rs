//! Vehicle actor implementation.

use crate::{
    actor::{Actor, ActorId},
    error::CarlaResult,
    geom::{FromCxx, ToCxx, Transform, Vector3D},
    rpc::{
        VehicleControl, VehicleDoorType, VehicleLightState, VehiclePhysicsControl,
        VehicleTelemetryData,
    },
    traits::{ActorT, VehicleT},
};

/// Vehicle actor.
#[derive(Debug)]
pub struct Vehicle {
    /// Actor ID
    id: ActorId,
    /// Internal vehicle wrapper for FFI calls
    inner: carla_cxx::VehicleWrapper,
}

impl Vehicle {
    /// Create a vehicle from a carla-cxx VehicleWrapper.
    pub fn from_cxx(vehicle_wrapper: carla_cxx::VehicleWrapper) -> CarlaResult<Self> {
        // Get the vehicle's actor ID
        let id = vehicle_wrapper.get_id();

        Ok(Self {
            id,
            inner: vehicle_wrapper,
        })
    }

    /// Create a vehicle from an actor by casting.
    pub fn from_actor(actor: Actor) -> CarlaResult<Option<Self>> {
        let actor_ref = actor.get_inner_actor();
        if let Some(vehicle_wrapper) = carla_cxx::VehicleWrapper::from_actor(actor_ref) {
            let id = vehicle_wrapper.get_id();
            Ok(Some(Self {
                id,
                inner: vehicle_wrapper,
            }))
        } else {
            Ok(None)
        }
    }

    /// Get the vehicle's actor ID.
    pub fn get_id(&self) -> ActorId {
        self.id
    }

    /// Get the vehicle's wheel physics parameters.
    pub fn get_wheel_steer_angle(&self, wheel_location: WheelLocation) -> f32 {
        use carla_cxx::vehicle::VehicleWheelLocation;

        let cxx_wheel_location = match wheel_location {
            WheelLocation::FrontLeft => VehicleWheelLocation::FrontLeft,
            WheelLocation::FrontRight => VehicleWheelLocation::FrontRight,
            WheelLocation::RearLeft => VehicleWheelLocation::RearLeft,
            WheelLocation::RearRight => VehicleWheelLocation::RearRight,
        };
        self.inner.get_wheel_steer_angle(cxx_wheel_location)
    }

    /// Get the vehicle's forward speed in km/h.
    pub fn get_speed(&self) -> f32 {
        // VehicleWrapper returns speed in m/s, convert to km/h
        self.inner.get_speed() * 3.6
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

impl ActorT for Vehicle {
    fn id(&self) -> ActorId {
        self.id
    }
    fn type_id(&self) -> String {
        self.inner.get_type_id()
    }
    fn transform(&self) -> Transform {
        let cxx_transform = self.inner.get_transform();
        Transform::from(cxx_transform)
    }
    fn set_transform(&self, transform: &Transform) -> CarlaResult<()> {
        let cxx_transform = transform.to_cxx();
        self.inner.set_transform(&cxx_transform);
        Ok(())
    }
    fn velocity(&self) -> Vector3D {
        let vel = self.inner.get_velocity();
        Vector3D::new(vel.x as f32, vel.y as f32, vel.z as f32)
    }
    fn angular_velocity(&self) -> Vector3D {
        let vel = self.inner.get_angular_velocity();
        Vector3D::new(vel.x as f32, vel.y as f32, vel.z as f32)
    }
    fn acceleration(&self) -> Vector3D {
        let acc = self.inner.get_acceleration();
        Vector3D::new(acc.x as f32, acc.y as f32, acc.z as f32)
    }
    fn is_alive(&self) -> bool {
        self.inner.is_alive()
    }
    fn set_simulate_physics(&self, enabled: bool) -> CarlaResult<()> {
        self.inner.set_simulate_physics(enabled);
        Ok(())
    }
    fn add_impulse(&self, impulse: &Vector3D) -> CarlaResult<()> {
        let cxx_impulse = carla_cxx::SimpleVector3D {
            x: impulse.x as f64,
            y: impulse.y as f64,
            z: impulse.z as f64,
        };
        self.inner.add_impulse(&cxx_impulse);
        Ok(())
    }
    fn add_force(&self, force: &Vector3D) -> CarlaResult<()> {
        let cxx_force = carla_cxx::SimpleVector3D {
            x: force.x as f64,
            y: force.y as f64,
            z: force.z as f64,
        };
        self.inner.add_force(&cxx_force);
        Ok(())
    }
    fn add_torque(&self, torque: &Vector3D) -> CarlaResult<()> {
        let cxx_torque = carla_cxx::SimpleVector3D {
            x: torque.x as f64,
            y: torque.y as f64,
            z: torque.z as f64,
        };
        self.inner.add_torque(&cxx_torque);
        Ok(())
    }

    fn bounding_box(&self) -> crate::geom::BoundingBox {
        // Vehicle doesn't have a direct GetBoundingBox method, need to cast to Actor
        let actor_ptr = self.inner.to_actor();
        if actor_ptr.is_null() {
            panic!("Internal error: Failed to cast Vehicle to Actor");
        }
        let simple_bbox = carla_cxx::ffi::Actor_GetBoundingBox(&actor_ptr);
        crate::geom::BoundingBox::from_cxx(simple_bbox)
    }
}

impl VehicleT for Vehicle {
    fn apply_control(&self, control: &VehicleControl) -> CarlaResult<()> {
        // Convert high-level VehicleControl to carla-cxx VehicleControl
        let cxx_control = carla_cxx::VehicleControl {
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

    fn get_control(&self) -> VehicleControl {
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

    fn set_autopilot(&self, enabled: bool, traffic_manager_port: Option<u16>) -> CarlaResult<()> {
        let tm_port = traffic_manager_port.unwrap_or(8000);
        self.inner.set_autopilot(enabled, tm_port);
        Ok(())
    }

    fn get_physics_control(&self) -> VehiclePhysicsControl {
        let cxx_physics = self.inner.get_physics_control();

        // Convert carla-cxx VehiclePhysicsControl to high-level VehiclePhysicsControl
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

    fn apply_physics_control(&self, physics_control: &VehiclePhysicsControl) -> CarlaResult<()> {
        let cxx_physics = carla_cxx::VehiclePhysicsControl {
            engine: carla_cxx::EnginePhysics {
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
            transmission: carla_cxx::TransmissionPhysics {
                use_gear_autobox: physics_control.use_gear_autobox,
                gear_switch_time: physics_control.gear_switch_time,
                clutch_strength: physics_control.clutch_strength,
                final_ratio: physics_control.final_ratio,
            },
            mass: physics_control.mass,
            drag_coefficient: physics_control.drag_coefficient,
            center_of_mass: carla_cxx::SimpleVector3D {
                x: physics_control.center_of_mass.x as f64,
                y: physics_control.center_of_mass.y as f64,
                z: physics_control.center_of_mass.z as f64,
            },
            steering: carla_cxx::SteeringPhysics {
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

    fn set_light_state(&self, light_state: VehicleLightState) -> CarlaResult<()> {
        let cxx_light_state = carla_cxx::VehicleLightState::from_bits_truncate(light_state.lights);
        self.inner.set_light_state(cxx_light_state);
        Ok(())
    }

    fn get_light_state(&self) -> VehicleLightState {
        let cxx_light_state = self.inner.get_light_state();
        VehicleLightState {
            lights: cxx_light_state.bits(),
        }
    }

    fn set_door_state(&self, door_type: VehicleDoorType, is_open: bool) -> CarlaResult<()> {
        let cxx_door_type = match door_type {
            VehicleDoorType::FrontLeft => carla_cxx::VehicleDoorType::FrontLeft,
            VehicleDoorType::FrontRight => carla_cxx::VehicleDoorType::FrontRight,
            VehicleDoorType::RearLeft => carla_cxx::VehicleDoorType::RearLeft,
            VehicleDoorType::RearRight => carla_cxx::VehicleDoorType::RearRight,
            VehicleDoorType::All => carla_cxx::VehicleDoorType::All,
        };

        if is_open {
            self.inner.open_door(cxx_door_type);
        } else {
            self.inner.close_door(cxx_door_type);
        }
        Ok(())
    }

    fn get_telemetry_data(&self) -> VehicleTelemetryData {
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
