//! CARLA navigation agents system.
//!
//! This module provides autonomous vehicle control through a layered architecture:
//! - **Navigation**: High-level route planning and trajectory following
//! - **Tools**: Utility functions for agent operations
//!
//! ## Component Hierarchy
//!
//! ```text
//! GlobalRoutePlanner (high-level path planning)
//!     ↓
//! LocalPlanner (trajectory following + PID control)
//!     ↓
//! VehiclePIDController (low-level vehicle control)
//!     ├─ PIDLongitudinalController (throttle/brake)
//!     └─ PIDLateralController (steering)
//! ```
//!
//! ## Usage Example
//!
//! ```no_run
//! use carla::client::Client;
//! use carla::agents::navigation::BasicAgent;
//!
//! # fn main() -> eyre::Result<()> {
//! let client = Client::new("localhost:2000", 10)?;
//! let mut world = client.world();
//!
//! // Spawn vehicle
//! let vehicle = /* spawn vehicle */;
//!
//! // Create agent
//! let mut agent = BasicAgent::new(&vehicle, 30.0)?;
//! agent.follow_speed_limits(true);
//!
//! // Set destination
//! let destination = /* some location */;
//! agent.set_destination(destination)?;
//!
//! // Control loop
//! while !agent.done() {
//!     world.tick()?;
//!     let control = agent.run_step()?;
//!     vehicle.apply_control(&control)?;
//! }
//! # Ok(())
//! # }
//! ```

pub mod navigation;
pub mod tools;
