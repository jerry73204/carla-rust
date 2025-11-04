//! Navigation components for autonomous vehicle control.
//!
//! This module provides:
//! - PID controllers for vehicle control
//! - Route planning and following
//! - Agent implementations (BasicAgent, BehaviorAgent, etc.)

pub mod agent_core;
pub mod basic_agent;
pub mod behavior_agent;
pub mod constant_velocity_agent;
pub mod controller;
pub mod global_route_planner;
pub mod local_planner;
pub mod pid;
pub mod types;

pub use agent_core::{
    AgentCore, AgentCoreConfig, LaneChangeDirection, ObstacleDetectionResult,
    TrafficLightDetectionResult,
};
pub use basic_agent::{BasicAgent, BasicAgentConfig};
pub use behavior_agent::{BehaviorAgent, BehaviorAgentConfig, BehaviorParams, BehaviorType};
pub use constant_velocity_agent::{ConstantVelocityAgent, ConstantVelocityAgentConfig};
pub use controller::VehiclePIDController;
pub use global_route_planner::GlobalRoutePlanner;
pub use local_planner::{LocalPlanner, LocalPlannerConfig};
pub use pid::{PIDLateralController, PIDLongitudinalController, PIDParams};
pub use types::{Agent, RoadOption};
