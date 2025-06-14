//! High-level Rust bindings for CARLA simulator using direct CXX FFI.
//!
//! This crate provides safe, idiomatic Rust bindings to CARLA's C++ client library
//! using the CXX bridge for direct C++/Rust interop without a C wrapper layer.
//!
//! # Architecture
//!
//! - **ffi** - Raw FFI bridge definitions using CXX
//! - **geometry** - Mathematical operations for CARLA geometry types
//! - **client** - High-level wrappers for Client, World, Actor management
//! - **blueprint** - Extension traits for ActorBlueprint functionality
//!
//! # Quick Start
//!
//! ```rust,no_run
//! use carla_cxx::{ClientWrapper, SimpleLocation, SimpleRotation, SimpleTransform};
//!
//! // Connect to CARLA server
//! let mut client = ClientWrapper::new("localhost", 2000)?;
//! let world = client.get_world();
//!
//! // Get blueprint library and spawn a vehicle
//! let blueprint_library = world.get_blueprint_library();
//! if let Some(vehicle_bp) = blueprint_library.find("vehicle.tesla.model3") {
//!     let spawn_point =
//!         SimpleTransform::new(SimpleLocation::new(0.0, 0.0, 0.5), SimpleRotation::ZERO);
//!
//!     if let Ok(vehicle) = world.spawn_actor(&vehicle_bp, &spawn_point, None) {
//!         println!("Spawned vehicle with ID: {}", vehicle.get_id());
//!     }
//! }
//! # Ok::<(), anyhow::Error>(())
//! ```

#![allow(dead_code)]

// Module declarations
pub mod blueprint;
pub mod client;
pub mod ffi;
pub mod geometry;

#[cfg(test)]
mod tests;

// Re-export the most commonly used types for convenience
pub use ffi::{
    Actor, ActorBlueprint, BlueprintLibrary, Client, SimpleBoundingBox, SimpleLocation,
    SimpleRotation, SimpleTransform, SimpleVector2D, SimpleVector3D, World,
};

pub use blueprint::ActorBlueprintExt;
pub use client::{ActorWrapper, BlueprintLibraryWrapper, ClientWrapper, WorldWrapper};

// Re-export geometry implementations (they're implemented as trait impls on the ffi types)
// The geometry module contains all the impl blocks for the Simple* types from ffi
