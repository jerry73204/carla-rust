//! Type definitions for navigation agents.

use crate::{client::Waypoint, geom::Location, rpc::VehicleControl};
use anyhow::Result;
use std::{fmt, str::FromStr};

/// Road maneuver options for route planning.
///
/// This enum represents different types of maneuvers that can be performed
/// along a route, used to annotate waypoint sequences.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum RoadOption {
    /// No specific maneuver (invalid/placeholder)
    Void = -1,
    /// Turn left at intersection
    Left = 1,
    /// Turn right at intersection
    Right = 2,
    /// Go straight through intersection
    Straight = 3,
    /// Continue following current lane
    LaneFollow = 4,
    /// Change to left lane
    ChangeLaneLeft = 5,
    /// Change to right lane
    ChangeLaneRight = 6,
}

impl RoadOption {
    /// Returns all valid road options (excluding Void).
    pub fn all() -> &'static [RoadOption] {
        &[
            RoadOption::Left,
            RoadOption::Right,
            RoadOption::Straight,
            RoadOption::LaneFollow,
            RoadOption::ChangeLaneLeft,
            RoadOption::ChangeLaneRight,
        ]
    }

    /// Converts integer value to RoadOption.
    pub fn from_i32(value: i32) -> Option<Self> {
        match value {
            -1 => Some(RoadOption::Void),
            1 => Some(RoadOption::Left),
            2 => Some(RoadOption::Right),
            3 => Some(RoadOption::Straight),
            4 => Some(RoadOption::LaneFollow),
            5 => Some(RoadOption::ChangeLaneLeft),
            6 => Some(RoadOption::ChangeLaneRight),
            _ => None,
        }
    }

    /// Converts RoadOption to integer value.
    pub fn as_i32(&self) -> i32 {
        *self as i32
    }
}

impl fmt::Display for RoadOption {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            RoadOption::Void => write!(f, "VOID"),
            RoadOption::Left => write!(f, "LEFT"),
            RoadOption::Right => write!(f, "RIGHT"),
            RoadOption::Straight => write!(f, "STRAIGHT"),
            RoadOption::LaneFollow => write!(f, "LANEFOLLOW"),
            RoadOption::ChangeLaneLeft => write!(f, "CHANGELANELEFT"),
            RoadOption::ChangeLaneRight => write!(f, "CHANGELANERIGHT"),
        }
    }
}

impl FromStr for RoadOption {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s.to_uppercase().as_str() {
            "VOID" => Ok(RoadOption::Void),
            "LEFT" => Ok(RoadOption::Left),
            "RIGHT" => Ok(RoadOption::Right),
            "STRAIGHT" => Ok(RoadOption::Straight),
            "LANEFOLLOW" => Ok(RoadOption::LaneFollow),
            "CHANGELANELEFT" => Ok(RoadOption::ChangeLaneLeft),
            "CHANGELANERIGHT" => Ok(RoadOption::ChangeLaneRight),
            _ => Err(format!("Invalid RoadOption: {}", s)),
        }
    }
}

/// Common interface for navigation agents.
///
/// This trait defines the core functionality that all agent types must provide,
/// enabling polymorphic usage of different agent implementations.
///
/// # Examples
///
/// ```no_run
/// use carla::{
///     agents::navigation::{Agent, BasicAgent, BasicAgentConfig},
///     client::Vehicle,
///     geom::Location,
/// };
///
/// fn run_agent_loop(agent: &mut dyn Agent) -> anyhow::Result<()> {
///     while !agent.done() {
///         let control = agent.run_step()?;
///         // Apply control to vehicle...
///     }
///     Ok(())
/// }
/// # fn example(vehicle: Vehicle) -> anyhow::Result<()> {
/// let config = BasicAgentConfig::default();
/// let mut agent = BasicAgent::new(vehicle, config, None, None)?;
/// let destination = Location::new(100.0, 50.0, 0.3);
/// agent.set_destination(destination, None, true)?;
/// run_agent_loop(&mut agent)?;
/// # Ok(())
/// # }
/// ```
pub trait Agent {
    /// Executes one navigation step and returns vehicle control.
    fn run_step(&mut self) -> Result<VehicleControl>;

    /// Checks if the destination has been reached.
    fn done(&self) -> bool;

    /// Sets a navigation destination.
    ///
    /// # Arguments
    /// * `end_location` - Target destination
    /// * `start_location` - Optional starting location (defaults to vehicle position)
    /// * `clean_queue` - Whether to clear existing route
    fn set_destination(
        &mut self,
        end_location: Location,
        start_location: Option<Location>,
        clean_queue: bool,
    ) -> Result<()>;

    /// Sets the target speed in km/h.
    fn set_target_speed(&mut self, speed: f32);

    /// Sets a pre-computed global plan.
    fn set_global_plan(
        &mut self,
        plan: Vec<(Waypoint, RoadOption)>,
        stop_waypoint_creation: bool,
        clean_queue: bool,
    );

    /// Computes a route between two waypoints.
    fn trace_route(
        &self,
        start_waypoint: &Waypoint,
        end_waypoint: &Waypoint,
    ) -> Result<Vec<(Waypoint, RoadOption)>>;

    /// Sets whether to ignore traffic lights.
    fn ignore_traffic_lights(&mut self, active: bool);

    /// Sets whether to ignore stop signs.
    fn ignore_stop_signs(&mut self, active: bool);

    /// Sets whether to ignore vehicles.
    fn ignore_vehicles(&mut self, active: bool);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_road_option_i32_conversion() {
        assert_eq!(RoadOption::Void.as_i32(), -1);
        assert_eq!(RoadOption::Left.as_i32(), 1);
        assert_eq!(RoadOption::Right.as_i32(), 2);
        assert_eq!(RoadOption::Straight.as_i32(), 3);
        assert_eq!(RoadOption::LaneFollow.as_i32(), 4);
        assert_eq!(RoadOption::ChangeLaneLeft.as_i32(), 5);
        assert_eq!(RoadOption::ChangeLaneRight.as_i32(), 6);
    }

    #[test]
    fn test_road_option_from_i32() {
        assert_eq!(RoadOption::from_i32(-1), Some(RoadOption::Void));
        assert_eq!(RoadOption::from_i32(1), Some(RoadOption::Left));
        assert_eq!(RoadOption::from_i32(2), Some(RoadOption::Right));
        assert_eq!(RoadOption::from_i32(3), Some(RoadOption::Straight));
        assert_eq!(RoadOption::from_i32(4), Some(RoadOption::LaneFollow));
        assert_eq!(RoadOption::from_i32(5), Some(RoadOption::ChangeLaneLeft));
        assert_eq!(RoadOption::from_i32(6), Some(RoadOption::ChangeLaneRight));
        assert_eq!(RoadOption::from_i32(99), None);
    }

    #[test]
    fn test_road_option_display() {
        assert_eq!(format!("{}", RoadOption::Void), "VOID");
        assert_eq!(format!("{}", RoadOption::Left), "LEFT");
        assert_eq!(format!("{}", RoadOption::Right), "RIGHT");
        assert_eq!(format!("{}", RoadOption::Straight), "STRAIGHT");
        assert_eq!(format!("{}", RoadOption::LaneFollow), "LANEFOLLOW");
        assert_eq!(format!("{}", RoadOption::ChangeLaneLeft), "CHANGELANELEFT");
        assert_eq!(
            format!("{}", RoadOption::ChangeLaneRight),
            "CHANGELANERIGHT"
        );
    }

    #[test]
    fn test_road_option_from_str() {
        assert_eq!("VOID".parse::<RoadOption>(), Ok(RoadOption::Void));
        assert_eq!("LEFT".parse::<RoadOption>(), Ok(RoadOption::Left));
        assert_eq!("right".parse::<RoadOption>(), Ok(RoadOption::Right));
        assert_eq!("Straight".parse::<RoadOption>(), Ok(RoadOption::Straight));
        assert!("INVALID".parse::<RoadOption>().is_err());
    }

    #[test]
    fn test_road_option_all() {
        let all = RoadOption::all();
        assert_eq!(all.len(), 6);
        assert!(!all.contains(&RoadOption::Void));
        assert!(all.contains(&RoadOption::Left));
        assert!(all.contains(&RoadOption::Right));
    }
}
