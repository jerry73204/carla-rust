//! Traffic manager for controlling autopilot vehicles with realistic behavior.
//!
//! The [`TrafficManager`] provides fine-grained control over groups of vehicles
//! in autopilot mode, enabling realistic urban traffic simulation with customizable
//! behaviors.
//!
//! # Key Features
//!
//! - **Speed control**: Set global or per-vehicle speed limits
//! - **Lane behavior**: Configure lane changes, lane offset, keep-right rules
//! - **Safety**: Collision detection, safe distances
//! - **Traffic rules**: Ignore pedestrians/vehicles, run red lights (probabilistic)
//! - **Routing**: Custom paths and imported routes
//! - **Performance**: Hybrid physics mode for large-scale simulations
//!
//! # Examples
//!
//! ```no_run
//! use carla::client::Client;
//!
//! let client = Client::default();
//! let mut world = client.world();
//! let mut tm = client.instance_tm(8000);
//!
//! // Spawn some vehicles
//! let bp_lib = world.blueprint_library();
//! let vehicle_bp = bp_lib.filter("vehicle.*").get(0).unwrap();
//! let spawn_points = world.map().recommended_spawn_points();
//! let vehicle1 = world
//!     .spawn_actor(&vehicle_bp, &spawn_points.get(0).unwrap())
//!     .unwrap();
//! let vehicle2 = world
//!     .spawn_actor(&vehicle_bp, &spawn_points.get(1).unwrap())
//!     .unwrap();
//!
//! // Register vehicles with traffic manager
//! let vehicles = [vehicle1.clone(), vehicle2.clone()];
//! tm.register_vehicles(&vehicles);
//!
//! // Configure behavior
//! tm.set_global_percentage_speed_difference(-20.0); // 20% faster
//! tm.set_auto_lane_change(&vehicle1, true);
//! tm.set_distance_to_leading_vehicle(&vehicle1, 5.0);
//!
//! // Enable autopilot (done on vehicle, not TM)
//! let vehicle1_typed: carla::client::Vehicle = vehicle1.try_into().unwrap();
//! vehicle1_typed.set_autopilot(true);
//! ```

mod action;
mod action_buffer;
mod tm;

pub use action::*;
pub use action_buffer::*;
pub use carla_sys::carla::traffic_manager::{constants, RoadOption};
pub use tm::*;
