//! Rust client library for Carla simulator.
//!
//! It works with CARLA simulator version 0.9.14.

pub mod client;
pub mod geom;
pub mod road;
pub mod rpc;
pub mod sensor;
pub mod traffic_manager;
mod utils;

pub mod prelude {
    pub use crate::{
        client::{ActorBase as _, TimestampExt as _},
        geom::{
            LocationExt as _, RotationExt as _, TransformExt as _, Vector2DExt as _,
            Vector3DExt as _,
        },
        sensor::SensorDataBase as _,
    };
}

pub use carla_sys;
