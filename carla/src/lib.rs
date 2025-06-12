//! Rust client library for Carla simulator.
//!
//! It works with CARLA simulator version 0.10.0.
//!
//! This library provides a comprehensive Rust API that mirrors the C++ API structure,
//! offering safe and idiomatic access to all CARLA simulation capabilities.

pub mod client;
pub mod geom;
pub mod road; // TODO: Update road module to use new C API functions
pub mod ros2; // Native ROS2 integration for CARLA 0.10.0
pub mod sensor;
mod stubs; // TODO: Remove when real C API is integrated
pub mod traffic_manager;
mod utils;

pub mod prelude {
    //! Common traits and types for CARLA Rust API.
    pub use crate::{
        client::ActorBase as _,
        geom::{
            LocationExt as _, RotationExt as _, TransformExt as _, Vector2DExt as _,
            Vector3DExt as _,
        },
        // TODO: Re-enable when sensor trait is updated
        // sensor::SensorDataBase as _,
    };
}

pub use carla_sys;
