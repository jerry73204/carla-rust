//! RPC data structures and types.
//!
//! This module contains data structures used for remote procedure calls
//! between the client and server, mirroring CARLA's `carla::rpc` namespace.

mod actor_id;
mod command;
mod vehicle_control;
mod walker_control;

pub use actor_id::*;
pub use command::*;
pub use vehicle_control::*;
pub use walker_control::*;
