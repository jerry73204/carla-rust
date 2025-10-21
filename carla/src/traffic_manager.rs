//! The traffic manager manages groups of autopilot vehicles with
//! user-customized options.

mod action;
mod action_buffer;
mod tm;

pub use action::*;
pub use action_buffer::*;
pub use carla_sys::carla::traffic_manager::{constants, RoadOption};
pub use tm::*;
