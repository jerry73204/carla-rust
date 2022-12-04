use super::{Actor, ActorBase};
use crate::rpc::{
    TrafficLightState, VehicleControl, VehicleDoor, VehicleLightState_LightState,
    VehicleWheelLocation,
};
use autocxx::prelude::*;
use carla_sys::{
    carla::{rpc::VehiclePhysicsControl, traffic_manager::constants::Networking::TM_DEFAULT_PORT},
    carla_rust::client::{FfiActor, FfiVehicle},
};
use cxx::{SharedPtr, UniquePtr};

#[repr(transparent)]
pub struct Vehicle {
    inner: SharedPtr<FfiVehicle>,
}

impl Vehicle {
    pub fn set_autopilot(&mut self, enabled: bool) {
        self.set_autopilot_opt(enabled, TM_DEFAULT_PORT)
    }

    pub fn set_autopilot_opt(&mut self, enabled: bool, tm_port: u16) {
        self.inner.SetAutopilot(enabled, tm_port);
    }

    pub fn show_debug_telemetry(&mut self, enabled: bool) {
        self.inner.ShowDebugTelemetry(enabled);
    }

    pub fn apply_control(&mut self, control: &VehicleControl) {
        self.inner.ApplyControl(control);
    }

    pub fn open_door(&mut self, door: VehicleDoor) {
        self.inner.OpenDoor(door);
    }

    pub fn close_door(&mut self, door: VehicleDoor) {
        self.inner.CloseDoor(door);
    }

    pub fn set_light_state(&mut self, light_state: &VehicleLightState_LightState) {
        self.inner.SetLightState(light_state);
    }

    pub fn set_wheel_steer_direction(
        &mut self,
        wheel_location: VehicleWheelLocation,
        degrees: f32,
    ) {
        self.inner.SetWheelSteerDirection(wheel_location, degrees);
    }

    pub fn get_wheel_steer_angle(&mut self, wheel_location: VehicleWheelLocation) -> f32 {
        self.inner.GetWheelSteerAngle(wheel_location)
    }

    pub fn control(&self) -> VehicleControl {
        self.inner.GetControl()
    }

    pub fn physics_control(&self) -> UniquePtr<VehiclePhysicsControl> {
        self.inner.GetPhysicsControl().within_unique_ptr()
    }

    pub fn light_state(&self) -> VehicleLightState_LightState {
        self.inner.GetLightState()
    }

    pub fn traffic_light_state(&self) -> TrafficLightState {
        self.inner.GetTrafficLightState()
    }

    pub fn is_at_traffic_light(&mut self) -> bool {
        self.inner.IsAtTrafficLight()
    }

    pub fn enable_car_sim(&mut self, simfile_path: &str) {
        self.inner.EnableCarSim(simfile_path)
    }

    pub fn use_car_sim_road(&mut self, enabled: bool) {
        self.inner.UseCarSimRoad(enabled);
    }

    pub fn enable_chrono_physics(
        &mut self,
        max_substeps: u64,
        max_substep_delta_time: f32,
        vehicle_json: &str,
        powertrain_json: &str,
        tire_json: &str,
        base_json_path: &str,
    ) {
        self.inner.EnableChronoPhysics(
            max_substeps,
            max_substep_delta_time,
            vehicle_json,
            powertrain_json,
            tire_json,
            base_json_path,
        );
    }

    pub fn into_actor(self) -> Actor {
        let ptr = self.inner.to_actor();
        Actor::from_cxx(ptr).unwrap()
    }

    pub(crate) fn from_cxx(ptr: SharedPtr<FfiVehicle>) -> Option<Self> {
        if ptr.is_null() {
            None
        } else {
            Some(Self { inner: ptr })
        }
    }
}

impl ActorBase for Vehicle {
    fn cxx_actor(&self) -> SharedPtr<FfiActor> {
        self.inner.to_actor()
    }
}

impl TryFrom<Actor> for Vehicle {
    type Error = Actor;

    fn try_from(value: Actor) -> Result<Self, Self::Error> {
        let ptr = value.inner.to_vehicle();
        Self::from_cxx(ptr).ok_or(value)
    }
}
