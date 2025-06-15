//! Vehicle actor implementation.

use crate::{
    client::{Actor, ActorId},
    error::CarlaResult,
    geom::{Transform, Vector3D},
    rpc::{
        VehicleControl, VehicleDoorType, VehicleLightState, VehiclePhysicsControl,
        VehicleTelemetryData,
    },
    traits::{ActorT, VehicleT},
};

/// Vehicle actor.
#[derive(Debug)]
pub struct Vehicle {
    /// Base actor
    pub actor: Actor,
    // Additional vehicle-specific fields
}

impl Vehicle {
    /// Create a vehicle from a carla-cxx VehicleWrapper.
    /// TODO: Need Vehicle to Actor conversion in carla-cxx FFI
    pub fn new(_vehicle_wrapper: carla_cxx::VehicleWrapper) -> Self {
        todo!("Vehicle::new - need Vehicle to Actor conversion FFI function")
    }

    /// Create a vehicle from an actor.
    pub fn from_actor(actor: Actor) -> Self {
        Self { actor }
    }

    /// Get the underlying actor.
    pub fn as_actor(&self) -> &Actor {
        &self.actor
    }

    /// Get the maximum steering angle.
    pub fn get_max_steering_angle(&self) -> f32 {
        // TODO: Implement using carla-cxx FFI interface
        todo!("Vehicle::get_max_steering_angle not yet implemented with carla-cxx FFI")
    }

    /// Get the vehicle's wheel physics parameters.
    pub fn get_wheel_steer_angle(&self, wheel_location: WheelLocation) -> f32 {
        // TODO: Implement using carla-cxx FFI interface
        let _wheel_location = wheel_location;
        todo!("Vehicle::get_wheel_steer_angle not yet implemented with carla-cxx FFI")
    }

    /// Get the vehicle's forward speed in km/h.
    pub fn get_speed(&self) -> f32 {
        // Convert m/s to km/h
        self.get_velocity().length() * 3.6
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
    fn get_id(&self) -> ActorId {
        self.actor.get_id()
    }
    fn get_type_id(&self) -> String {
        self.actor.get_type_id()
    }
    fn get_transform(&self) -> Transform {
        self.actor.get_transform()
    }
    fn set_transform(&self, transform: &Transform) -> CarlaResult<()> {
        self.actor.set_transform(transform)
    }
    fn get_velocity(&self) -> Vector3D {
        self.actor.get_velocity()
    }
    fn get_angular_velocity(&self) -> Vector3D {
        self.actor.get_angular_velocity()
    }
    fn get_acceleration(&self) -> Vector3D {
        self.actor.get_acceleration()
    }
    fn is_alive(&self) -> bool {
        self.actor.is_alive()
    }
    fn destroy(&self) -> CarlaResult<()> {
        self.actor.destroy()
    }
    fn set_simulate_physics(&self, enabled: bool) -> CarlaResult<()> {
        self.actor.set_simulate_physics(enabled)
    }
    fn add_impulse(&self, impulse: &Vector3D) -> CarlaResult<()> {
        self.actor.add_impulse(impulse)
    }
    fn add_force(&self, force: &Vector3D) -> CarlaResult<()> {
        self.actor.add_force(force)
    }
    fn add_torque(&self, torque: &Vector3D) -> CarlaResult<()> {
        self.actor.add_torque(torque)
    }
}

impl VehicleT for Vehicle {
    fn apply_control(&self, control: &VehicleControl) -> CarlaResult<()> {
        // TODO: Implement using carla-cxx FFI interface
        let _control = control;
        todo!("Vehicle::apply_control not yet implemented with carla-cxx FFI")
    }

    fn get_control(&self) -> VehicleControl {
        // TODO: Implement using carla-cxx FFI interface
        todo!("Vehicle::get_control not yet implemented with carla-cxx FFI")
    }

    fn set_autopilot(&self, enabled: bool, traffic_manager_port: Option<u16>) -> CarlaResult<()> {
        // TODO: Implement using carla-cxx FFI interface
        let _enabled = enabled;
        let _traffic_manager_port = traffic_manager_port;
        todo!("Vehicle::set_autopilot not yet implemented with carla-cxx FFI")
    }

    fn get_physics_control(&self) -> VehiclePhysicsControl {
        // TODO: Implement using carla-cxx FFI interface
        todo!("Vehicle::get_physics_control not yet implemented with carla-cxx FFI")
    }

    fn apply_physics_control(&self, physics_control: &VehiclePhysicsControl) -> CarlaResult<()> {
        // TODO: Implement using carla-cxx FFI interface
        let _physics_control = physics_control;
        todo!("Vehicle::apply_physics_control not yet implemented with carla-cxx FFI")
    }

    fn set_light_state(&self, light_state: VehicleLightState) -> CarlaResult<()> {
        // TODO: Implement using carla-cxx FFI interface
        let _light_state = light_state;
        todo!("Vehicle::set_light_state not yet implemented with carla-cxx FFI")
    }

    fn get_light_state(&self) -> VehicleLightState {
        // TODO: Implement using carla-cxx FFI interface
        todo!("Vehicle::get_light_state not yet implemented with carla-cxx FFI")
    }

    fn set_door_state(&self, door_type: VehicleDoorType, is_open: bool) -> CarlaResult<()> {
        // TODO: Implement using carla-cxx FFI interface
        let _door_type = door_type;
        let _is_open = is_open;
        todo!("Vehicle::set_door_state not yet implemented with carla-cxx FFI")
    }

    fn get_telemetry_data(&self) -> VehicleTelemetryData {
        // TODO: Implement using carla-cxx FFI interface
        todo!("Vehicle::get_telemetry_data not yet implemented with carla-cxx FFI")
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
