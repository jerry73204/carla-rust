pub mod actor;
pub mod camera;
pub mod collision;
pub mod dvs;
pub mod gnss;
pub mod id;
pub mod imu;
pub mod lane_invasion;
pub mod lidar;
pub mod list;
pub mod obstacle_detection;
pub mod radar;
pub mod rss;
pub mod sensor;
pub mod traffic_light;
pub mod traffic_sign;
pub mod traits;
pub mod vehicle;
pub mod walker;

pub use actor::{Actor, ActorSnapshot};
pub use camera::{Camera, CameraType};
pub use collision::CollisionSensor;
pub use dvs::DVSCamera;
pub use gnss::GNSS;
pub use id::ActorId;
pub use imu::IMU;
pub use lane_invasion::LaneInvasionSensor;
pub use lidar::LiDAR;
pub use list::ActorList;
pub use obstacle_detection::ObstacleDetectionSensor;
pub use radar::Radar;
pub use rss::RSSSensor;
pub use sensor::Sensor;
pub use traffic_light::{TrafficLight, TrafficLightState};
pub use traffic_sign::TrafficSign;
pub use traits::{ActorExt, SensorExt};
pub(crate) use traits::{ActorFfi, SensorFfi};
pub use vehicle::{
    Vehicle, VehicleControl, VehicleDoorType, VehicleLightState, VehiclePhysicsControl,
    VehicleTelemetryData,
};
pub use walker::{Walker, WalkerControl};

// Macro for implementing sensor conversions
#[macro_export]
macro_rules! impl_sensor_conversions {
    ($sensor_type:ty, $check_method:ident) => {
        impl $sensor_type {
            /// Try to create this type from a generic Actor.
            /// Returns the original Actor if the conversion fails.
            pub fn from_actor(actor: $crate::actor::Actor) -> Result<Self, $crate::actor::Actor> {
                match $crate::actor::Sensor::from_actor(actor) {
                    Ok(sensor) => Self::from_sensor(sensor).map_err(|s| s.into_actor()),
                    Err(actor) => Err(actor),
                }
            }

            /// Convert this type into a generic Actor.
            pub fn into_actor(self) -> $crate::actor::Actor {
                self.0.into_actor()
            }

            /// Get a reference to this type as an Actor.
            pub fn as_actor(&self) -> &$crate::actor::Actor {
                // This is not ideal as it creates a new Actor each time,
                // but necessary due to the C++ shared pointer semantics
                // The returned reference is to a leaked box to maintain lifetime safety
                Box::leak(Box::new(self.0.as_actor()))
            }

            /// Try to create this type from a generic Sensor.
            /// Returns the original Sensor if the conversion fails.
            pub fn from_sensor(
                sensor: $crate::actor::Sensor,
            ) -> Result<Self, $crate::actor::Sensor> {
                if sensor.$check_method() {
                    Ok(Self(sensor))
                } else {
                    Err(sensor)
                }
            }

            /// Convert this type into a generic Sensor.
            pub fn into_sensor(self) -> $crate::actor::Sensor {
                self.0
            }

            /// Get a reference to this type as a Sensor.
            pub fn as_sensor(&self) -> &$crate::actor::Sensor {
                &self.0
            }
        }
    };
}
