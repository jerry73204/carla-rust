//! Batch operations utilities for efficient bulk operations in CARLA.

use crate::{
    ffi::bridge::{SimpleBatchCommand, SimpleBatchResponse},
    SimpleLocation, SimpleTransform, SimpleVector3D, SimpleVehicleControl,
};

/// Command types for batch operations
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum BatchCommandType {
    SpawnActor = 0,
    DestroyActor = 1,
    ApplyVehicleControl = 2,
    ApplyWalkerControl = 3,
    ApplyTransform = 4,
    ApplyLocation = 5,
    ApplyVehicleAckermannControl = 6,
    SetAutopilot = 7,
    SetVehicleLightState = 8,
    SetSimulatePhysics = 9,
    SetEnableGravity = 10,
    ApplyTargetVelocity = 11,
    ApplyTargetAngularVelocity = 12,
    ApplyImpulse = 13,
    ApplyForce = 14,
    ApplyAngularImpulse = 15,
    ApplyTorque = 16,
    SetTrafficLightState = 17,
    ApplyWalkerState = 18,
    ApplyVehiclePhysicsControl = 19,
    ShowDebugTelemetry = 20,
    ConsoleCommand = 21,
}

impl From<BatchCommandType> for u8 {
    fn from(cmd_type: BatchCommandType) -> u8 {
        cmd_type as u8
    }
}

/// Traffic light states for batch operations
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum TrafficLightState {
    Red = 0,
    Yellow = 1,
    Green = 2,
    Off = 3,
    Unknown = 4,
}

impl From<TrafficLightState> for u8 {
    fn from(state: TrafficLightState) -> u8 {
        state as u8
    }
}

/// Vehicle light state flags for batch operations
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct VehicleLightState {
    pub flags: u32,
}

impl VehicleLightState {
    pub const NONE: u32 = 0;
    pub const POSITION: u32 = 0x1;
    pub const LOW_BEAM: u32 = 0x1 << 1;
    pub const HIGH_BEAM: u32 = 0x1 << 2;
    pub const BRAKE: u32 = 0x1 << 3;
    pub const RIGHT_BLINKER: u32 = 0x1 << 4;
    pub const LEFT_BLINKER: u32 = 0x1 << 5;
    pub const REVERSE: u32 = 0x1 << 6;
    pub const FOG: u32 = 0x1 << 7;
    pub const INTERIOR: u32 = 0x1 << 8;
    pub const SPECIAL1: u32 = 0x1 << 9;
    pub const SPECIAL2: u32 = 0x1 << 10;
    pub const ALL: u32 = 0xFFFFFFFF;

    pub fn new(flags: u32) -> Self {
        Self { flags }
    }

    pub fn none() -> Self {
        Self::new(Self::NONE)
    }

    pub fn all() -> Self {
        Self::new(Self::ALL)
    }
}

/// Builder for batch commands
pub struct BatchCommandBuilder;

impl BatchCommandBuilder {
    /// Create a command to destroy an actor
    pub fn destroy_actor(actor_id: u32) -> SimpleBatchCommand {
        SimpleBatchCommand {
            command_type: BatchCommandType::DestroyActor.into(),
            actor_id,
            data1: 0.0,
            data2: 0.0,
            data3: 0.0,
            data4: 0.0,
            data5: 0.0,
            data6: 0.0,
            bool_flag1: false,
            bool_flag2: false,
            int_flag1: 0,
            int_flag2: 0,
        }
    }

    /// Create a command to apply vehicle control
    pub fn apply_vehicle_control(
        actor_id: u32,
        control: &SimpleVehicleControl,
    ) -> SimpleBatchCommand {
        SimpleBatchCommand {
            command_type: BatchCommandType::ApplyVehicleControl.into(),
            actor_id,
            data1: control.throttle as f64,
            data2: control.steer as f64,
            data3: control.brake as f64,
            data4: 0.0,
            data5: 0.0,
            data6: 0.0,
            bool_flag1: control.hand_brake,
            bool_flag2: control.reverse,
            int_flag1: if control.manual_gear_shift { 1 } else { 0 },
            int_flag2: control.gear,
        }
    }

    /// Create a command to apply walker control
    pub fn apply_walker_control(
        actor_id: u32,
        direction: &SimpleVector3D,
        speed: f32,
        jump: bool,
    ) -> SimpleBatchCommand {
        SimpleBatchCommand {
            command_type: BatchCommandType::ApplyWalkerControl.into(),
            actor_id,
            data1: direction.x,
            data2: direction.y,
            data3: direction.z,
            data4: speed as f64,
            data5: 0.0,
            data6: 0.0,
            bool_flag1: jump,
            bool_flag2: false,
            int_flag1: 0,
            int_flag2: 0,
        }
    }

    /// Create a command to apply transform
    pub fn apply_transform(actor_id: u32, transform: &SimpleTransform) -> SimpleBatchCommand {
        SimpleBatchCommand {
            command_type: BatchCommandType::ApplyTransform.into(),
            actor_id,
            data1: transform.location.x,
            data2: transform.location.y,
            data3: transform.location.z,
            data4: transform.rotation.pitch,
            data5: transform.rotation.yaw,
            data6: transform.rotation.roll,
            bool_flag1: false,
            bool_flag2: false,
            int_flag1: 0,
            int_flag2: 0,
        }
    }

    /// Create a command to apply location only
    pub fn apply_location(actor_id: u32, location: &SimpleLocation) -> SimpleBatchCommand {
        SimpleBatchCommand {
            command_type: BatchCommandType::ApplyLocation.into(),
            actor_id,
            data1: location.x,
            data2: location.y,
            data3: location.z,
            data4: 0.0,
            data5: 0.0,
            data6: 0.0,
            bool_flag1: false,
            bool_flag2: false,
            int_flag1: 0,
            int_flag2: 0,
        }
    }

    /// Create a command to set autopilot
    pub fn set_autopilot(actor_id: u32, enabled: bool, tm_port: u16) -> SimpleBatchCommand {
        SimpleBatchCommand {
            command_type: BatchCommandType::SetAutopilot.into(),
            actor_id,
            data1: 0.0,
            data2: 0.0,
            data3: 0.0,
            data4: 0.0,
            data5: 0.0,
            data6: 0.0,
            bool_flag1: enabled,
            bool_flag2: false,
            int_flag1: tm_port as i32,
            int_flag2: 0,
        }
    }

    /// Create a command to set vehicle light state
    pub fn set_vehicle_light_state(
        actor_id: u32,
        light_state: VehicleLightState,
    ) -> SimpleBatchCommand {
        SimpleBatchCommand {
            command_type: BatchCommandType::SetVehicleLightState.into(),
            actor_id,
            data1: 0.0,
            data2: 0.0,
            data3: 0.0,
            data4: 0.0,
            data5: 0.0,
            data6: 0.0,
            bool_flag1: false,
            bool_flag2: false,
            int_flag1: light_state.flags as i32,
            int_flag2: 0,
        }
    }

    /// Create a command to set physics simulation
    pub fn set_simulate_physics(actor_id: u32, enabled: bool) -> SimpleBatchCommand {
        SimpleBatchCommand {
            command_type: BatchCommandType::SetSimulatePhysics.into(),
            actor_id,
            data1: 0.0,
            data2: 0.0,
            data3: 0.0,
            data4: 0.0,
            data5: 0.0,
            data6: 0.0,
            bool_flag1: enabled,
            bool_flag2: false,
            int_flag1: 0,
            int_flag2: 0,
        }
    }

    /// Create a command to set gravity
    pub fn set_enable_gravity(actor_id: u32, enabled: bool) -> SimpleBatchCommand {
        SimpleBatchCommand {
            command_type: BatchCommandType::SetEnableGravity.into(),
            actor_id,
            data1: 0.0,
            data2: 0.0,
            data3: 0.0,
            data4: 0.0,
            data5: 0.0,
            data6: 0.0,
            bool_flag1: enabled,
            bool_flag2: false,
            int_flag1: 0,
            int_flag2: 0,
        }
    }

    /// Create a command to apply target velocity
    pub fn apply_target_velocity(actor_id: u32, velocity: &SimpleVector3D) -> SimpleBatchCommand {
        SimpleBatchCommand {
            command_type: BatchCommandType::ApplyTargetVelocity.into(),
            actor_id,
            data1: velocity.x,
            data2: velocity.y,
            data3: velocity.z,
            data4: 0.0,
            data5: 0.0,
            data6: 0.0,
            bool_flag1: false,
            bool_flag2: false,
            int_flag1: 0,
            int_flag2: 0,
        }
    }

    /// Create a command to apply target angular velocity
    pub fn apply_target_angular_velocity(
        actor_id: u32,
        angular_velocity: &SimpleVector3D,
    ) -> SimpleBatchCommand {
        SimpleBatchCommand {
            command_type: BatchCommandType::ApplyTargetAngularVelocity.into(),
            actor_id,
            data1: angular_velocity.x,
            data2: angular_velocity.y,
            data3: angular_velocity.z,
            data4: 0.0,
            data5: 0.0,
            data6: 0.0,
            bool_flag1: false,
            bool_flag2: false,
            int_flag1: 0,
            int_flag2: 0,
        }
    }

    /// Create a command to apply impulse
    pub fn apply_impulse(actor_id: u32, impulse: &SimpleVector3D) -> SimpleBatchCommand {
        SimpleBatchCommand {
            command_type: BatchCommandType::ApplyImpulse.into(),
            actor_id,
            data1: impulse.x,
            data2: impulse.y,
            data3: impulse.z,
            data4: 0.0,
            data5: 0.0,
            data6: 0.0,
            bool_flag1: false,
            bool_flag2: false,
            int_flag1: 0,
            int_flag2: 0,
        }
    }

    /// Create a command to apply force
    pub fn apply_force(actor_id: u32, force: &SimpleVector3D) -> SimpleBatchCommand {
        SimpleBatchCommand {
            command_type: BatchCommandType::ApplyForce.into(),
            actor_id,
            data1: force.x,
            data2: force.y,
            data3: force.z,
            data4: 0.0,
            data5: 0.0,
            data6: 0.0,
            bool_flag1: false,
            bool_flag2: false,
            int_flag1: 0,
            int_flag2: 0,
        }
    }

    /// Create a command to apply angular impulse
    pub fn apply_angular_impulse(actor_id: u32, impulse: &SimpleVector3D) -> SimpleBatchCommand {
        SimpleBatchCommand {
            command_type: BatchCommandType::ApplyAngularImpulse.into(),
            actor_id,
            data1: impulse.x,
            data2: impulse.y,
            data3: impulse.z,
            data4: 0.0,
            data5: 0.0,
            data6: 0.0,
            bool_flag1: false,
            bool_flag2: false,
            int_flag1: 0,
            int_flag2: 0,
        }
    }

    /// Create a command to apply torque
    pub fn apply_torque(actor_id: u32, torque: &SimpleVector3D) -> SimpleBatchCommand {
        SimpleBatchCommand {
            command_type: BatchCommandType::ApplyTorque.into(),
            actor_id,
            data1: torque.x,
            data2: torque.y,
            data3: torque.z,
            data4: 0.0,
            data5: 0.0,
            data6: 0.0,
            bool_flag1: false,
            bool_flag2: false,
            int_flag1: 0,
            int_flag2: 0,
        }
    }

    /// Create a command to set traffic light state
    pub fn set_traffic_light_state(actor_id: u32, state: TrafficLightState) -> SimpleBatchCommand {
        SimpleBatchCommand {
            command_type: BatchCommandType::SetTrafficLightState.into(),
            actor_id,
            data1: 0.0,
            data2: 0.0,
            data3: 0.0,
            data4: 0.0,
            data5: 0.0,
            data6: 0.0,
            bool_flag1: false,
            bool_flag2: false,
            int_flag1: state as i32,
            int_flag2: 0,
        }
    }

    /// Create a command to show debug telemetry
    pub fn show_debug_telemetry(actor_id: u32, enabled: bool) -> SimpleBatchCommand {
        SimpleBatchCommand {
            command_type: BatchCommandType::ShowDebugTelemetry.into(),
            actor_id,
            data1: 0.0,
            data2: 0.0,
            data3: 0.0,
            data4: 0.0,
            data5: 0.0,
            data6: 0.0,
            bool_flag1: enabled,
            bool_flag2: false,
            int_flag1: 0,
            int_flag2: 0,
        }
    }
}

/// Extension trait for SimpleBatchResponse
pub trait BatchResponseExt {
    /// Check if the command was successful
    fn is_success(&self) -> bool;

    /// Check if the command failed
    fn is_error(&self) -> bool;

    /// Get the error message if any
    fn get_error_message(&self) -> Option<&str>;

    /// Get the resulting actor ID (for spawn commands)
    fn get_actor_id(&self) -> Option<u32>;
}

impl BatchResponseExt for SimpleBatchResponse {
    fn is_success(&self) -> bool {
        !self.has_error
    }

    fn is_error(&self) -> bool {
        self.has_error
    }

    fn get_error_message(&self) -> Option<&str> {
        if self.has_error && !self.error_message.is_empty() {
            Some(&self.error_message)
        } else {
            None
        }
    }

    fn get_actor_id(&self) -> Option<u32> {
        if !self.has_error && self.actor_id != 0 {
            Some(self.actor_id)
        } else {
            None
        }
    }
}

/// Utility functions for batch operations
pub mod batch_utils {
    use super::*;

    /// Create multiple vehicle control commands at once
    pub fn create_vehicle_controls(
        actor_ids: &[u32],
        controls: &[SimpleVehicleControl],
    ) -> Vec<SimpleBatchCommand> {
        actor_ids
            .iter()
            .zip(controls.iter())
            .map(|(&actor_id, control)| {
                BatchCommandBuilder::apply_vehicle_control(actor_id, control)
            })
            .collect()
    }

    /// Create multiple destroy commands at once
    pub fn create_destroy_commands(actor_ids: &[u32]) -> Vec<SimpleBatchCommand> {
        actor_ids
            .iter()
            .map(|&actor_id| BatchCommandBuilder::destroy_actor(actor_id))
            .collect()
    }

    /// Create multiple autopilot commands at once
    pub fn create_autopilot_commands(
        actor_ids: &[u32],
        enabled: bool,
        tm_port: u16,
    ) -> Vec<SimpleBatchCommand> {
        actor_ids
            .iter()
            .map(|&actor_id| BatchCommandBuilder::set_autopilot(actor_id, enabled, tm_port))
            .collect()
    }

    /// Create multiple transform commands at once
    pub fn create_transform_commands(
        actor_ids: &[u32],
        transforms: &[SimpleTransform],
    ) -> Vec<SimpleBatchCommand> {
        actor_ids
            .iter()
            .zip(transforms.iter())
            .map(|(&actor_id, transform)| BatchCommandBuilder::apply_transform(actor_id, transform))
            .collect()
    }

    /// Create multiple location commands at once
    pub fn create_location_commands(
        actor_ids: &[u32],
        locations: &[SimpleLocation],
    ) -> Vec<SimpleBatchCommand> {
        actor_ids
            .iter()
            .zip(locations.iter())
            .map(|(&actor_id, location)| BatchCommandBuilder::apply_location(actor_id, location))
            .collect()
    }
}
