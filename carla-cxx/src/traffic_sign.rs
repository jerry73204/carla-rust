//! Traffic Sign actor implementation for CARLA.

use crate::ffi::{self, Actor, TrafficSign};
use cxx::SharedPtr;

/// High-level wrapper for CARLA TrafficSign
pub struct TrafficSignWrapper {
    inner: SharedPtr<TrafficSign>,
}

impl TrafficSignWrapper {
    /// Create a TrafficSignWrapper from an Actor (performs cast)
    pub fn from_actor(actor: &Actor) -> Option<Self> {
        let traffic_sign_ptr = ffi::Actor_CastToTrafficSign(actor);
        if traffic_sign_ptr.is_null() {
            None
        } else {
            Some(Self {
                inner: traffic_sign_ptr,
            })
        }
    }

    /// Get the sign ID (unique identifier for this traffic sign type)
    pub fn get_sign_id(&self) -> String {
        ffi::TrafficSign_GetSignId(&self.inner)
    }

    /// Get the trigger volume (bounding box) for this traffic sign
    pub fn get_trigger_volume(&self) -> crate::ffi::bridge::SimpleBoundingBox {
        ffi::TrafficSign_GetTriggerVolume(&self.inner)
    }

    /// Get the traffic sign's type ID
    pub fn get_type_id(&self) -> String {
        ffi::TrafficSign_GetTypeId(&self.inner)
    }

    /// Get the traffic sign's transform
    pub fn get_transform(&self) -> crate::ffi::bridge::SimpleTransform {
        ffi::TrafficSign_GetTransform(&self.inner)
    }

    /// Set the traffic sign's transform
    pub fn set_transform(&self, transform: &crate::ffi::bridge::SimpleTransform) {
        ffi::TrafficSign_SetTransform(&self.inner, transform)
    }

    /// Get the traffic sign's velocity
    pub fn get_velocity(&self) -> crate::ffi::bridge::SimpleVector3D {
        ffi::TrafficSign_GetVelocity(&self.inner)
    }

    /// Get the traffic sign's angular velocity
    pub fn get_angular_velocity(&self) -> crate::ffi::bridge::SimpleVector3D {
        ffi::TrafficSign_GetAngularVelocity(&self.inner)
    }

    /// Get the traffic sign's acceleration
    pub fn get_acceleration(&self) -> crate::ffi::bridge::SimpleVector3D {
        ffi::TrafficSign_GetAcceleration(&self.inner)
    }

    /// Check if the traffic sign is alive
    pub fn is_alive(&self) -> bool {
        ffi::TrafficSign_IsAlive(&self.inner)
    }

    /// Destroy the traffic sign
    pub fn destroy(&self) -> bool {
        ffi::TrafficSign_Destroy(&self.inner)
    }

    /// Set physics simulation for the traffic sign
    pub fn set_simulate_physics(&self, enabled: bool) {
        ffi::TrafficSign_SetSimulatePhysics(&self.inner, enabled)
    }

    /// Add impulse to the traffic sign
    pub fn add_impulse(&self, impulse: &crate::ffi::bridge::SimpleVector3D) {
        ffi::TrafficSign_AddImpulse(&self.inner, impulse)
    }

    /// Add force to the traffic sign
    pub fn add_force(&self, force: &crate::ffi::bridge::SimpleVector3D) {
        ffi::TrafficSign_AddForce(&self.inner, force)
    }

    /// Add torque to the traffic sign
    pub fn add_torque(&self, torque: &crate::ffi::bridge::SimpleVector3D) {
        ffi::TrafficSign_AddTorque(&self.inner, torque)
    }

    /// Get access to the inner TrafficSign for direct FFI calls
    pub fn get_inner_traffic_sign(&self) -> &cxx::SharedPtr<TrafficSign> {
        &self.inner
    }
}

impl std::fmt::Debug for TrafficSignWrapper {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("TrafficSignWrapper")
            .field("sign_id", &self.get_sign_id())
            .finish()
    }
}

/// Common traffic sign types found in CARLA
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum TrafficSignType {
    /// Speed limit signs
    SpeedLimit30,
    SpeedLimit40,
    SpeedLimit50,
    SpeedLimit60,
    SpeedLimit70,
    SpeedLimit80,
    SpeedLimit90,
    SpeedLimit100,
    SpeedLimit110,
    SpeedLimit120,
    SpeedLimitEnd,

    /// Stop and yield signs
    Stop,
    Yield,
    YieldToPedestrians,

    /// Direction signs
    TurnLeft,
    TurnRight,
    GoStraight,
    GoStraightOrTurnLeft,
    GoStraightOrTurnRight,
    KeepLeft,
    KeepRight,
    Roundabout,

    /// Prohibition signs
    NoEntry,
    NoTurnLeft,
    NoTurnRight,
    NoUTurn,
    NoOvertaking,
    NoPassing,
    NoParking,
    NoStopping,

    /// Warning signs
    DangerousCurveLeft,
    DangerousCurveRight,
    DangerousCurves,
    Crosswalk,
    SchoolZone,
    TrafficSignalsAhead,
    RoadWorks,
    Slippery,

    /// Information signs
    ParkingArea,
    Highway,
    HighwayEnd,
    OneWay,
    DeadEnd,

    /// Priority signs
    PriorityRoad,
    PriorityRoadEnd,
    GiveWay,

    /// Other
    Unknown,
}

impl TrafficSignType {
    /// Parse a traffic sign type from its CARLA sign ID string
    pub fn from_sign_id(sign_id: &str) -> Self {
        match sign_id {
            // Speed limits
            "traffic.speed_limit.30" => Self::SpeedLimit30,
            "traffic.speed_limit.40" => Self::SpeedLimit40,
            "traffic.speed_limit.50" => Self::SpeedLimit50,
            "traffic.speed_limit.60" => Self::SpeedLimit60,
            "traffic.speed_limit.70" => Self::SpeedLimit70,
            "traffic.speed_limit.80" => Self::SpeedLimit80,
            "traffic.speed_limit.90" => Self::SpeedLimit90,
            "traffic.speed_limit.100" => Self::SpeedLimit100,
            "traffic.speed_limit.110" => Self::SpeedLimit110,
            "traffic.speed_limit.120" => Self::SpeedLimit120,
            "traffic.speed_limit.end" => Self::SpeedLimitEnd,

            // Stop and yield
            "traffic.stop" => Self::Stop,
            "traffic.yield" => Self::Yield,
            "traffic.yield_to_pedestrians" => Self::YieldToPedestrians,

            // Direction
            "traffic.turn_left" => Self::TurnLeft,
            "traffic.turn_right" => Self::TurnRight,
            "traffic.go_straight" => Self::GoStraight,
            "traffic.go_straight_or_turn_left" => Self::GoStraightOrTurnLeft,
            "traffic.go_straight_or_turn_right" => Self::GoStraightOrTurnRight,
            "traffic.keep_left" => Self::KeepLeft,
            "traffic.keep_right" => Self::KeepRight,
            "traffic.roundabout" => Self::Roundabout,

            // Prohibition
            "traffic.no_entry" => Self::NoEntry,
            "traffic.no_turn_left" => Self::NoTurnLeft,
            "traffic.no_turn_right" => Self::NoTurnRight,
            "traffic.no_u_turn" => Self::NoUTurn,
            "traffic.no_overtaking" => Self::NoOvertaking,
            "traffic.no_passing" => Self::NoPassing,
            "traffic.no_parking" => Self::NoParking,
            "traffic.no_stopping" => Self::NoStopping,

            // Warning
            "traffic.dangerous_curve_left" => Self::DangerousCurveLeft,
            "traffic.dangerous_curve_right" => Self::DangerousCurveRight,
            "traffic.dangerous_curves" => Self::DangerousCurves,
            "traffic.crosswalk" => Self::Crosswalk,
            "traffic.school_zone" => Self::SchoolZone,
            "traffic.traffic_signals_ahead" => Self::TrafficSignalsAhead,
            "traffic.road_works" => Self::RoadWorks,
            "traffic.slippery" => Self::Slippery,

            // Information
            "traffic.parking_area" => Self::ParkingArea,
            "traffic.highway" => Self::Highway,
            "traffic.highway_end" => Self::HighwayEnd,
            "traffic.one_way" => Self::OneWay,
            "traffic.dead_end" => Self::DeadEnd,

            // Priority
            "traffic.priority_road" => Self::PriorityRoad,
            "traffic.priority_road_end" => Self::PriorityRoadEnd,
            "traffic.give_way" => Self::GiveWay,

            _ => Self::Unknown,
        }
    }

    /// Get the CARLA sign ID string for this traffic sign type
    pub fn to_sign_id(&self) -> &'static str {
        match self {
            // Speed limits
            Self::SpeedLimit30 => "traffic.speed_limit.30",
            Self::SpeedLimit40 => "traffic.speed_limit.40",
            Self::SpeedLimit50 => "traffic.speed_limit.50",
            Self::SpeedLimit60 => "traffic.speed_limit.60",
            Self::SpeedLimit70 => "traffic.speed_limit.70",
            Self::SpeedLimit80 => "traffic.speed_limit.80",
            Self::SpeedLimit90 => "traffic.speed_limit.90",
            Self::SpeedLimit100 => "traffic.speed_limit.100",
            Self::SpeedLimit110 => "traffic.speed_limit.110",
            Self::SpeedLimit120 => "traffic.speed_limit.120",
            Self::SpeedLimitEnd => "traffic.speed_limit.end",

            // Stop and yield
            Self::Stop => "traffic.stop",
            Self::Yield => "traffic.yield",
            Self::YieldToPedestrians => "traffic.yield_to_pedestrians",

            // Direction
            Self::TurnLeft => "traffic.turn_left",
            Self::TurnRight => "traffic.turn_right",
            Self::GoStraight => "traffic.go_straight",
            Self::GoStraightOrTurnLeft => "traffic.go_straight_or_turn_left",
            Self::GoStraightOrTurnRight => "traffic.go_straight_or_turn_right",
            Self::KeepLeft => "traffic.keep_left",
            Self::KeepRight => "traffic.keep_right",
            Self::Roundabout => "traffic.roundabout",

            // Prohibition
            Self::NoEntry => "traffic.no_entry",
            Self::NoTurnLeft => "traffic.no_turn_left",
            Self::NoTurnRight => "traffic.no_turn_right",
            Self::NoUTurn => "traffic.no_u_turn",
            Self::NoOvertaking => "traffic.no_overtaking",
            Self::NoPassing => "traffic.no_passing",
            Self::NoParking => "traffic.no_parking",
            Self::NoStopping => "traffic.no_stopping",

            // Warning
            Self::DangerousCurveLeft => "traffic.dangerous_curve_left",
            Self::DangerousCurveRight => "traffic.dangerous_curve_right",
            Self::DangerousCurves => "traffic.dangerous_curves",
            Self::Crosswalk => "traffic.crosswalk",
            Self::SchoolZone => "traffic.school_zone",
            Self::TrafficSignalsAhead => "traffic.traffic_signals_ahead",
            Self::RoadWorks => "traffic.road_works",
            Self::Slippery => "traffic.slippery",

            // Information
            Self::ParkingArea => "traffic.parking_area",
            Self::Highway => "traffic.highway",
            Self::HighwayEnd => "traffic.highway_end",
            Self::OneWay => "traffic.one_way",
            Self::DeadEnd => "traffic.dead_end",

            // Priority
            Self::PriorityRoad => "traffic.priority_road",
            Self::PriorityRoadEnd => "traffic.priority_road_end",
            Self::GiveWay => "traffic.give_way",

            Self::Unknown => "traffic.unknown",
        }
    }

    /// Check if this is a speed limit sign
    pub fn is_speed_limit(&self) -> bool {
        matches!(
            self,
            Self::SpeedLimit30
                | Self::SpeedLimit40
                | Self::SpeedLimit50
                | Self::SpeedLimit60
                | Self::SpeedLimit70
                | Self::SpeedLimit80
                | Self::SpeedLimit90
                | Self::SpeedLimit100
                | Self::SpeedLimit110
                | Self::SpeedLimit120
        )
    }

    /// Get the speed limit value in km/h (if this is a speed limit sign)
    pub fn get_speed_limit_kmh(&self) -> Option<u32> {
        match self {
            Self::SpeedLimit30 => Some(30),
            Self::SpeedLimit40 => Some(40),
            Self::SpeedLimit50 => Some(50),
            Self::SpeedLimit60 => Some(60),
            Self::SpeedLimit70 => Some(70),
            Self::SpeedLimit80 => Some(80),
            Self::SpeedLimit90 => Some(90),
            Self::SpeedLimit100 => Some(100),
            Self::SpeedLimit110 => Some(110),
            Self::SpeedLimit120 => Some(120),
            _ => None,
        }
    }

    /// Check if this is a regulatory sign (must be obeyed)
    pub fn is_regulatory(&self) -> bool {
        matches!(
            self,
            Self::Stop
                | Self::Yield
                | Self::YieldToPedestrians
                | Self::NoEntry
                | Self::NoTurnLeft
                | Self::NoTurnRight
                | Self::NoUTurn
                | Self::NoOvertaking
                | Self::NoPassing
                | Self::NoParking
                | Self::NoStopping
                | Self::OneWay
                | Self::GiveWay
        ) || self.is_speed_limit()
    }

    /// Check if this is a warning sign
    pub fn is_warning(&self) -> bool {
        matches!(
            self,
            Self::DangerousCurveLeft
                | Self::DangerousCurveRight
                | Self::DangerousCurves
                | Self::Crosswalk
                | Self::SchoolZone
                | Self::TrafficSignalsAhead
                | Self::RoadWorks
                | Self::Slippery
                | Self::DeadEnd
        )
    }

    /// Check if this is an informational sign
    pub fn is_informational(&self) -> bool {
        matches!(
            self,
            Self::ParkingArea
                | Self::Highway
                | Self::HighwayEnd
                | Self::PriorityRoad
                | Self::PriorityRoadEnd
        )
    }

    /// Check if this is a directional sign
    pub fn is_directional(&self) -> bool {
        matches!(
            self,
            Self::TurnLeft
                | Self::TurnRight
                | Self::GoStraight
                | Self::GoStraightOrTurnLeft
                | Self::GoStraightOrTurnRight
                | Self::KeepLeft
                | Self::KeepRight
                | Self::Roundabout
        )
    }
}

impl std::fmt::Display for TrafficSignType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            // Speed limits
            Self::SpeedLimit30 => write!(f, "🚗 Speed Limit 30 km/h"),
            Self::SpeedLimit40 => write!(f, "🚗 Speed Limit 40 km/h"),
            Self::SpeedLimit50 => write!(f, "🚗 Speed Limit 50 km/h"),
            Self::SpeedLimit60 => write!(f, "🚗 Speed Limit 60 km/h"),
            Self::SpeedLimit70 => write!(f, "🚗 Speed Limit 70 km/h"),
            Self::SpeedLimit80 => write!(f, "🚗 Speed Limit 80 km/h"),
            Self::SpeedLimit90 => write!(f, "🚗 Speed Limit 90 km/h"),
            Self::SpeedLimit100 => write!(f, "🚗 Speed Limit 100 km/h"),
            Self::SpeedLimit110 => write!(f, "🚗 Speed Limit 110 km/h"),
            Self::SpeedLimit120 => write!(f, "🚗 Speed Limit 120 km/h"),
            Self::SpeedLimitEnd => write!(f, "✅ End of Speed Limit"),

            // Stop and yield
            Self::Stop => write!(f, "🛑 STOP"),
            Self::Yield => write!(f, "⚠️ Yield"),
            Self::YieldToPedestrians => write!(f, "🚶 Yield to Pedestrians"),

            // Direction
            Self::TurnLeft => write!(f, "⬅️ Turn Left"),
            Self::TurnRight => write!(f, "➡️ Turn Right"),
            Self::GoStraight => write!(f, "⬆️ Go Straight"),
            Self::GoStraightOrTurnLeft => write!(f, "⬆️⬅️ Go Straight or Turn Left"),
            Self::GoStraightOrTurnRight => write!(f, "⬆️➡️ Go Straight or Turn Right"),
            Self::KeepLeft => write!(f, "⬅️ Keep Left"),
            Self::KeepRight => write!(f, "➡️ Keep Right"),
            Self::Roundabout => write!(f, "🔄 Roundabout"),

            // Prohibition
            Self::NoEntry => write!(f, "🚫 No Entry"),
            Self::NoTurnLeft => write!(f, "🚫⬅️ No Left Turn"),
            Self::NoTurnRight => write!(f, "🚫➡️ No Right Turn"),
            Self::NoUTurn => write!(f, "🚫↩️ No U-Turn"),
            Self::NoOvertaking => write!(f, "🚫🚗 No Overtaking"),
            Self::NoPassing => write!(f, "🚫 No Passing"),
            Self::NoParking => write!(f, "🚫🅿️ No Parking"),
            Self::NoStopping => write!(f, "🚫🛑 No Stopping"),

            // Warning
            Self::DangerousCurveLeft => write!(f, "⚠️↩️ Dangerous Curve Left"),
            Self::DangerousCurveRight => write!(f, "⚠️↪️ Dangerous Curve Right"),
            Self::DangerousCurves => write!(f, "⚠️〰️ Dangerous Curves"),
            Self::Crosswalk => write!(f, "⚠️🚶 Crosswalk"),
            Self::SchoolZone => write!(f, "⚠️🏫 School Zone"),
            Self::TrafficSignalsAhead => write!(f, "⚠️🚦 Traffic Signals Ahead"),
            Self::RoadWorks => write!(f, "⚠️🚧 Road Works"),
            Self::Slippery => write!(f, "⚠️🌧️ Slippery Road"),

            // Information
            Self::ParkingArea => write!(f, "🅿️ Parking Area"),
            Self::Highway => write!(f, "🛣️ Highway"),
            Self::HighwayEnd => write!(f, "🛣️✅ Highway End"),
            Self::OneWay => write!(f, "➡️ One Way"),
            Self::DeadEnd => write!(f, "🚧 Dead End"),

            // Priority
            Self::PriorityRoad => write!(f, "🔶 Priority Road"),
            Self::PriorityRoadEnd => write!(f, "🔶✅ Priority Road End"),
            Self::GiveWay => write!(f, "⚠️ Give Way"),

            Self::Unknown => write!(f, "❓ Unknown Sign"),
        }
    }
}
