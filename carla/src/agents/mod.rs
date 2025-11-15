//! CARLA navigation agents system for autonomous vehicle control.
//!
//! This module provides autonomous vehicle control through a layered architecture,
//! corresponding to the Python API's
//! [agents](https://carla.readthedocs.io/en/0.9.16/adv_agents/)
//! package.
//!
//! # Key Components
//!
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
//! # Python API Reference
//!
//! See the [CARLA Agents](https://carla.readthedocs.io/en/0.9.16/adv_agents/)
//! documentation for the Python equivalent.
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
