//! Rust client library for Carla simulator.
//!
//! It works with CARLA simulator version 0.10.0.

pub mod client;
pub mod geom;
// pub mod road; // TODO: Update road module to use new C API functions
// pub mod rpc;
pub mod sensor_minimal;
pub mod traffic_manager;
mod utils;

// pub mod prelude {
//     pub use crate::{
//         client::{ActorBase as _, TimestampExt as _},
//         geom::{
//             LocationExt as _, RotationExt as _, TransformExt as _, Vector2DExt as _,
//             Vector3DExt as _,
//         },
//         sensor::SensorDataBase as _,
//     };
// }

pub use carla_sys;
