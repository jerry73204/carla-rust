use autocxx::prelude::*;
use carla_sys::carla::{
    client::Vehicle as FfiVehicle,
    rpc::{TrafficLightState, VehicleControl, VehiclePhysicsControl},
    traffic_manager::constants::Networking::TM_DEFAULT_PORT,
};
use cxx::UniquePtr;

pub struct Vehicle {
    inner: UniquePtr<FfiVehicle>,
}

impl Vehicle {
    pub fn set_autopilot(&mut self, enabled: bool) {
        self.set_autopilot_opt(enabled, TM_DEFAULT_PORT)
    }

    pub fn set_autopilot_opt(&mut self, enabled: bool, tm_port: u16) {
        self.inner.pin_mut().SetAutopilot(enabled, tm_port);
    }

    pub fn show_debug_telemetry(&mut self, enabled: bool) {
        self.inner.pin_mut().ShowDebugTelemetry(enabled);
    }

    pub fn apply_control(&mut self, control: &VehicleControl) {
        self.inner.pin_mut().ApplyControl(control);
    }

    // pub fn open_door(&mut self, door: VehicleDoor) {
    //     self.inner.pin_mut().OpenDoor(door);
    // }

    // pub fn close_door(&mut self, door: VehicleDoor) {
    //     self.inner.pin_mut().CloseDoor(door);
    // }

    // pub fn set_light_state(&mut self, light_state: &TrafficLightState) {
    //     self.inner.pin_mut().SetLightState(light_state);
    // }

    // pub fn set_wheel_steer_direction(
    //     &mut self,
    //     wheel_location: VehicleWheelLocation,
    //     degrees: f32,
    // ) {
    //     self.inner
    //         .pin_mut()
    //         .SetWheelSteerDirection(wheel_location, degrees);
    // }

    // pub fn get_wheel_steer_angle(&mut self, wheel_location: VehicleWheelLocation) -> f32 {
    //     self.inner.pin_mut().GetWheelSteerAngle(wheel_location)
    // }

    pub fn control(&self) -> VehicleControl {
        self.inner.GetControl()
    }

    pub fn physics_control(&self) -> UniquePtr<VehiclePhysicsControl> {
        self.inner.GetPhysicsControl().within_unique_ptr()
    }

    // pub fn light_state(&self) -> VehicleLightState {
    //     self.inner.GetLightState()
    // }

    pub fn traffic_light_state(&self) -> TrafficLightState {
        self.inner.GetTrafficLightState()
    }

    pub fn is_at_traffic_light(&mut self) -> bool {
        self.inner.pin_mut().IsAtTrafficLight()
    }

    pub fn enable_car_sim(&mut self, simfile_path: &str) {
        self.inner.pin_mut().EnableCarSim(simfile_path)
    }

    pub fn use_car_sim_road(&mut self, enabled: bool) {
        self.inner.pin_mut().UseCarSimRoad(enabled);
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
        self.inner.pin_mut().EnableChronoPhysics(
            max_substeps,
            max_substep_delta_time,
            vehicle_json,
            powertrain_json,
            tire_json,
            base_json_path,
        );
    }
}
