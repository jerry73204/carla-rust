//! Batch command types for executing multiple actor operations atomically.

use crate::{
    actor::{ActorId, VehicleControl, WalkerControl},
    geom::Transform,
};

/// Generic actor command.
#[derive(Debug, Clone)]
pub enum ActorCommand {
    /// Spawn an actor
    Spawn {
        /// Blueprint ID
        blueprint_id: String,
        /// Spawn transform
        transform: Transform,
        /// Parent actor ID (optional)
        parent_id: Option<ActorId>,
    },
    /// Destroy an actor
    Destroy {
        /// Actor ID to destroy
        actor_id: ActorId,
    },
    /// Apply control to an actor
    ApplyControl {
        /// Actor ID
        actor_id: ActorId,
        /// Control command
        control: ControlCommand,
    },
    /// Set actor transform
    SetTransform {
        /// Actor ID
        actor_id: ActorId,
        /// New transform
        transform: Transform,
    },
    /// Set actor physics enabled
    SetSimulatePhysics {
        /// Actor ID
        actor_id: ActorId,
        /// Physics enabled
        enabled: bool,
    },
}

/// Control command variants.
#[derive(Debug, Clone)]
pub enum ControlCommand {
    /// Vehicle control
    Vehicle(VehicleControl),
    /// Walker control
    Walker(WalkerControl),
}

/// Batch command for executing multiple commands atomically.
#[derive(Debug, Clone)]
pub struct BatchCommand {
    /// List of commands to execute
    pub commands: Vec<ActorCommand>,
    /// Whether to execute synchronously
    pub synchronous: bool,
}

impl BatchCommand {
    /// Create a new batch command.
    pub fn new(synchronous: bool) -> Self {
        Self {
            commands: Vec::new(),
            synchronous,
        }
    }

    /// Add a command to the batch.
    pub fn add_command(&mut self, command: ActorCommand) {
        self.commands.push(command);
    }

    /// Get the number of commands in the batch.
    pub fn len(&self) -> usize {
        self.commands.len()
    }

    /// Check if the batch is empty.
    pub fn is_empty(&self) -> bool {
        self.commands.is_empty()
    }
}
