use std::fmt::{self, Debug, Formatter};

// carla::client
// SAFETY: ActorBlueprint wraps a SharedPtr which provides thread-safe reference counting.
// The underlying C++ type is immutable after construction.
unsafe impl Send for crate::carla::client::ActorBlueprint {}
unsafe impl Sync for crate::carla::client::ActorBlueprint {}

// SAFETY: WorldSnapshot is a read-only snapshot of world state at a point in time.
// The underlying C++ object is immutable and can be safely shared across threads.
unsafe impl Send for crate::carla::client::WorldSnapshot {}

// carla_rust::client
// SAFETY: All FfiClient operations are thread-safe. The underlying C++ client uses
// thread-safe SharedPtr for reference counting and synchronizes access to network operations.
unsafe impl Send for crate::carla_rust::client::FfiClient {}
unsafe impl Sync for crate::carla_rust::client::FfiClient {}

// SAFETY: FfiWorld is a thread-safe handle to the simulation world. Operations are
// synchronized by the CARLA server, and the SharedPtr provides thread-safe ownership.
unsafe impl Send for crate::carla_rust::client::FfiWorld {}
unsafe impl Sync for crate::carla_rust::client::FfiWorld {}

// SAFETY: FfiActor represents an immutable handle to an actor in the simulation.
// The underlying SharedPtr provides thread-safe reference counting.
unsafe impl Send for crate::carla_rust::client::FfiActor {}
unsafe impl Sync for crate::carla_rust::client::FfiActor {}

// SAFETY: FfiBlueprintLibrary is a read-only collection of actor blueprints.
// The underlying data is immutable and SharedPtr ensures thread-safe ownership.
unsafe impl Send for crate::carla_rust::client::FfiBlueprintLibrary {}
unsafe impl Sync for crate::carla_rust::client::FfiBlueprintLibrary {}

// SAFETY: FfiLandmark represents an immutable map landmark (traffic sign, light, etc.).
// The underlying SharedPtr provides thread-safe reference counting.
unsafe impl Send for crate::carla_rust::client::FfiLandmark {}
unsafe impl Sync for crate::carla_rust::client::FfiLandmark {}

// SAFETY: FfiMap is a read-only representation of the simulation map.
// The map data is immutable and SharedPtr ensures thread-safe ownership.
unsafe impl Send for crate::carla_rust::client::FfiMap {}
unsafe impl Sync for crate::carla_rust::client::FfiMap {}

// SAFETY: FfiSensor wraps a SharedPtr to a sensor actor. While sensors can be configured,
// the handle itself is thread-safe and sensor operations are synchronized by the server.
unsafe impl Send for crate::carla_rust::client::FfiSensor {}
unsafe impl Sync for crate::carla_rust::client::FfiSensor {}

// SAFETY: FfiVehicle wraps a SharedPtr to a vehicle actor. The handle is thread-safe
// and vehicle control operations are synchronized by the CARLA server.
unsafe impl Send for crate::carla_rust::client::FfiVehicle {}
unsafe impl Sync for crate::carla_rust::client::FfiVehicle {}

// SAFETY: FfiTrafficSign wraps a SharedPtr to an immutable traffic sign actor.
// The underlying data is thread-safe.
unsafe impl Send for crate::carla_rust::client::FfiTrafficSign {}
unsafe impl Sync for crate::carla_rust::client::FfiTrafficSign {}

// SAFETY: FfiTrafficLight wraps a SharedPtr to a traffic light actor.
// Operations are synchronized by the CARLA server.
unsafe impl Send for crate::carla_rust::client::FfiTrafficLight {}
unsafe impl Sync for crate::carla_rust::client::FfiTrafficLight {}

// SAFETY: FfiWaypoint represents an immutable point on the road network.
// The waypoint data is read-only and SharedPtr ensures thread-safe ownership.
unsafe impl Send for crate::carla_rust::client::FfiWaypoint {}
unsafe impl Sync for crate::carla_rust::client::FfiWaypoint {}

// SAFETY: FfiActorList is a read-only container of actor SharedPtrs.
// The list itself is immutable and all contained actors are thread-safe.
unsafe impl Send for crate::carla_rust::client::FfiActorList {}
unsafe impl Sync for crate::carla_rust::client::FfiActorList {}

// SAFETY: FfiTransformList is a read-only container of transform data.
// All data is immutable POD types (position/rotation).
unsafe impl Send for crate::carla_rust::client::FfiTransformList {}
unsafe impl Sync for crate::carla_rust::client::FfiTransformList {}

// SAFETY: FfiLandmarkList is a read-only container of landmark SharedPtrs.
// The list and all landmarks are immutable.
unsafe impl Send for crate::carla_rust::client::FfiLandmarkList {}
unsafe impl Sync for crate::carla_rust::client::FfiLandmarkList {}

// SAFETY: FfiWorldSnapshot is an immutable snapshot of world state at a point in time.
// All data is read-only and can be safely shared across threads.
unsafe impl Send for crate::carla_rust::client::FfiWorldSnapshot {}
unsafe impl Sync for crate::carla_rust::client::FfiWorldSnapshot {}

// SAFETY: FfiWaypointList is a read-only container of waypoint pointers.
// All waypoints are immutable and the list itself is read-only.
unsafe impl Send for crate::carla_rust::client::FfiWaypointList {}
unsafe impl Sync for crate::carla_rust::client::FfiWaypointList {}

// SAFETY: FfiJunction represents an immutable junction in the road network.
// The junction data is read-only and SharedPtr ensures thread-safe ownership.
unsafe impl Send for crate::carla_rust::client::FfiJunction {}
unsafe impl Sync for crate::carla_rust::client::FfiJunction {}

// SAFETY: FfiTrafficLightList is a read-only container of traffic light pointers.
// All traffic lights are thread-safe and the list is immutable.
unsafe impl Send for crate::carla_rust::client::FfiTrafficLightList {}
unsafe impl Sync for crate::carla_rust::client::FfiTrafficLightList {}

// SAFETY: FfiBoundingBoxList is a read-only container of bounding box data.
// All data is POD types (geometry data) and the container is immutable.
unsafe impl Send for crate::carla_rust::client::FfiBoundingBoxList {}
unsafe impl Sync for crate::carla_rust::client::FfiBoundingBoxList {}

// SAFETY: FfiLabelledPointList is a read-only container of labelled point data.
// All data is POD types and the container is immutable.
unsafe impl Send for crate::carla_rust::client::FfiLabelledPointList {}
unsafe impl Sync for crate::carla_rust::client::FfiLabelledPointList {}

// SAFETY: FfiEnvironmentObjectList is a read-only container of environment object data.
// The container and all contained data are immutable.
unsafe impl Send for crate::carla_rust::client::FfiEnvironmentObjectList {}
unsafe impl Sync for crate::carla_rust::client::FfiEnvironmentObjectList {}

// SAFETY: FfiActorVec is a read-only vector of actor SharedPtrs.
// All actors are thread-safe and the vector is immutable.
unsafe impl Send for crate::carla_rust::client::FfiActorVec {}
unsafe impl Sync for crate::carla_rust::client::FfiActorVec {}

// SAFETY: FfiLightManager manages global light state and uses SharedPtr for thread-safe ownership.
// Light operations are synchronized by the CARLA server.
unsafe impl Send for crate::carla_rust::client::FfiLightManager {}
unsafe impl Sync for crate::carla_rust::client::FfiLightManager {}

// SAFETY: FfiLightList is a read-only container of light data.
// The container is immutable after creation.
unsafe impl Send for crate::carla_rust::client::FfiLightList {}
unsafe impl Sync for crate::carla_rust::client::FfiLightList {}

// carla::geom
// SAFETY: Vector2D is a POD type containing only two f32 fields (x, y).
// It has no interior mutability and is safe to send and share across threads.
unsafe impl Send for crate::carla::geom::Vector2D {}
unsafe impl Sync for crate::carla::geom::Vector2D {}

// SAFETY: Vector3D is a POD type containing only three f32 fields (x, y, z).
// It has no interior mutability and is safe to send and share across threads.
unsafe impl Send for crate::carla::geom::Vector3D {}
unsafe impl Sync for crate::carla::geom::Vector3D {}

// SAFETY: Rotation is a POD type containing only three f32 fields (pitch, yaw, roll).
// It has no interior mutability and is safe to send and share across threads.
unsafe impl Send for crate::carla::geom::Rotation {}
unsafe impl Sync for crate::carla::geom::Rotation {}

impl Debug for crate::carla::geom::Rotation {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        f.debug_struct("Rotation")
            .field("pitch", &self.pitch)
            .field("yaw", &self.yaw)
            .field("roll", &self.roll)
            .finish()
    }
}

impl Clone for crate::carla::geom::Rotation {
    fn clone(&self) -> Self {
        Self {
            pitch: self.pitch,
            yaw: self.yaw,
            roll: self.roll,
        }
    }
}

impl Debug for crate::carla::geom::Vector2D {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        f.debug_struct("Vector2D")
            .field("x", &self.x)
            .field("y", &self.y)
            .finish()
    }
}

impl Clone for crate::carla::geom::Vector2D {
    fn clone(&self) -> Self {
        Self {
            x: self.x,
            y: self.y,
        }
    }
}

impl Debug for crate::carla::geom::Vector3D {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        f.debug_struct("Vector3D")
            .field("x", &self.x)
            .field("y", &self.y)
            .field("z", &self.z)
            .finish()
    }
}

impl Clone for crate::carla::geom::Vector3D {
    fn clone(&self) -> Self {
        Self {
            x: self.x,
            y: self.y,
            z: self.z,
        }
    }
}

// carla_rust::geom
// SAFETY: FfiLocation is a POD type containing only three f32 fields (x, y, z).
// It wraps carla::geom::Location which is thread-safe.
unsafe impl Send for crate::carla_rust::geom::FfiLocation {}
unsafe impl Sync for crate::carla_rust::geom::FfiLocation {}

// SAFETY: FfiTransform contains only POD types (FfiLocation and Rotation).
// Both contained types are thread-safe.
unsafe impl Send for crate::carla_rust::geom::FfiTransform {}
unsafe impl Sync for crate::carla_rust::geom::FfiTransform {}

impl Debug for crate::carla_rust::geom::FfiLocation {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        f.debug_struct("FfiLocation")
            .field("x", &self.x)
            .field("y", &self.y)
            .field("z", &self.z)
            .finish()
    }
}

impl Clone for crate::carla_rust::geom::FfiLocation {
    fn clone(&self) -> Self {
        Self {
            x: self.x,
            y: self.y,
            z: self.z,
        }
    }
}

impl Debug for crate::carla_rust::geom::FfiTransform {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        f.debug_struct("FfiTransform")
            .field("location", &self.location)
            .field("rotation", &self.rotation)
            .finish()
    }
}

impl Clone for crate::carla_rust::geom::FfiTransform {
    fn clone(&self) -> Self {
        Self {
            location: self.location.clone(),
            rotation: self.rotation.clone(),
        }
    }
}

// carla::road

// carla_rust::road::element
// SAFETY: FfiLaneMarking is a read-only wrapper around lane marking data.
// All data is immutable and can be safely sent across threads.
unsafe impl Send for crate::carla_rust::road::element::FfiLaneMarking {}

// carla::rpc
// SAFETY: All carla::rpc types are Plain Old Data (POD) structs used for RPC communication.
// They contain only primitive types, vectors of primitives, or other POD types.
// None have interior mutability and all are safe to send across threads.

// EpisodeSettings contains only bool and numeric fields for simulation configuration.
unsafe impl Send for crate::carla::rpc::EpisodeSettings {}
unsafe impl Sync for crate::carla::rpc::EpisodeSettings {}

// GearPhysicsControl contains only f32 fields for gear ratios.
unsafe impl Send for crate::carla::rpc::GearPhysicsControl {}
unsafe impl Sync for crate::carla::rpc::GearPhysicsControl {}

// LabelledPoint contains geometry data (Location) and an enum label.
unsafe impl Send for crate::carla::rpc::LabelledPoint {}
unsafe impl Sync for crate::carla::rpc::LabelledPoint {}

// OpendriveGenerationParameters contains configuration data for map generation.
unsafe impl Send for crate::carla::rpc::OpendriveGenerationParameters {}
unsafe impl Sync for crate::carla::rpc::OpendriveGenerationParameters {}

// VehicleLightState is a bitfield enum representing light states.
unsafe impl Send for crate::carla::rpc::VehicleLightState {}
unsafe impl Sync for crate::carla::rpc::VehicleLightState {}

// VehiclePhysicsControl contains physics parameters (mass, drag, etc).
unsafe impl Send for crate::carla::rpc::VehiclePhysicsControl {}
unsafe impl Sync for crate::carla::rpc::VehiclePhysicsControl {}

// WalkerBoneControlIn contains bone animation input data.
unsafe impl Send for crate::carla::rpc::WalkerBoneControlIn {}
unsafe impl Sync for crate::carla::rpc::WalkerBoneControlIn {}

// WalkerBoneControlOut contains bone animation output data.
unsafe impl Send for crate::carla::rpc::WalkerBoneControlOut {}
unsafe impl Sync for crate::carla::rpc::WalkerBoneControlOut {}

// WalkerControl contains walker movement control parameters.
unsafe impl Send for crate::carla::rpc::WalkerControl {}
unsafe impl Sync for crate::carla::rpc::WalkerControl {}

// WheelPhysicsControl contains wheel physics parameters.
unsafe impl Send for crate::carla::rpc::WheelPhysicsControl {}
unsafe impl Sync for crate::carla::rpc::WheelPhysicsControl {}

// CityObjectLabel is a C-style enum for semantic segmentation labels.
unsafe impl Send for crate::carla::rpc::CityObjectLabel {}
unsafe impl Sync for crate::carla::rpc::CityObjectLabel {}

impl Debug for crate::carla::rpc::CityObjectLabel {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        match self {
            Self::None => write!(f, "None"),
            Self::Roads => write!(f, "Roads"),
            Self::Sidewalks => write!(f, "Sidewalks"),
            Self::Buildings => write!(f, "Buildings"),
            Self::Walls => write!(f, "Walls"),
            Self::Fences => write!(f, "Fences"),
            Self::Poles => write!(f, "Poles"),
            Self::TrafficLight => write!(f, "TrafficLight"),
            Self::TrafficSigns => write!(f, "TrafficSigns"),
            Self::Vegetation => write!(f, "Vegetation"),
            Self::Terrain => write!(f, "Terrain"),
            Self::Sky => write!(f, "Sky"),
            Self::Pedestrians => write!(f, "Pedestrians"),
            Self::Rider => write!(f, "Rider"),
            Self::Car => write!(f, "Car"),
            Self::Truck => write!(f, "Truck"),
            Self::Bus => write!(f, "Bus"),
            Self::Train => write!(f, "Train"),
            Self::Motorcycle => write!(f, "Motorcycle"),
            Self::Bicycle => write!(f, "Bicycle"),
            Self::Static => write!(f, "Static"),
            Self::Dynamic => write!(f, "Dynamic"),
            Self::Other => write!(f, "Other"),
            Self::Water => write!(f, "Water"),
            Self::RoadLines => write!(f, "RoadLines"),
            Self::Ground => write!(f, "Ground"),
            Self::Bridge => write!(f, "Bridge"),
            Self::RailTrack => write!(f, "RailTrack"),
            Self::GuardRail => write!(f, "GuardRail"),
            Self::Any => write!(f, "Any"),
        }
    }
}

// carla_rust::rpc
// SAFETY: FfiLabelledPoint contains only POD types (location and label enum).
// It is safe to send and share across threads.
unsafe impl Send for crate::carla_rust::rpc::FfiLabelledPoint {}
unsafe impl Sync for crate::carla_rust::rpc::FfiLabelledPoint {}

// SAFETY: FfiEpisodeSettings wraps carla::rpc::EpisodeSettings which is POD.
// It is safe to send and share across threads.
unsafe impl Send for crate::carla_rust::rpc::FfiEpisodeSettings {}
unsafe impl Sync for crate::carla_rust::rpc::FfiEpisodeSettings {}

impl Debug for crate::carla_rust::rpc::FfiLabelledPoint {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        f.debug_struct("FfiLabelledPoint")
            .field("location", &self.location)
            .field("label", &self.label)
            .finish()
    }
}

impl Clone for crate::carla_rust::rpc::FfiLabelledPoint {
    fn clone(&self) -> Self {
        Self {
            location: self.location.clone(),
            label: self.label.clone(),
        }
    }
}

impl Debug for crate::carla::rpc::GearPhysicsControl {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        f.debug_struct("GearPhysicsControl")
            .field("ratio", &self.ratio)
            .field("down_ratio", &self.down_ratio)
            .field("up_ratio", &self.up_ratio)
            .finish()
    }
}

impl Clone for crate::carla::rpc::GearPhysicsControl {
    fn clone(&self) -> Self {
        Self {
            ratio: self.ratio,
            down_ratio: self.down_ratio,
            up_ratio: self.up_ratio,
        }
    }
}

impl Debug for crate::carla::rpc::WheelPhysicsControl {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        f.debug_struct("WheelPhysicsControl")
            .field("tire_friction", &self.tire_friction)
            .field("damping_rate", &self.damping_rate)
            .field("max_steer_angle", &self.max_steer_angle)
            .field("radius", &self.radius)
            .field("max_brake_torque", &self.max_brake_torque)
            .field("max_handbrake_torque", &self.max_handbrake_torque)
            .field("lat_stiff_max_load", &self.lat_stiff_max_load)
            .field("lat_stiff_value", &self.lat_stiff_value)
            .field("long_stiff_value", &self.long_stiff_value)
            .field("position", &self.position)
            .finish()
    }
}

impl Clone for crate::carla::rpc::WheelPhysicsControl {
    fn clone(&self) -> Self {
        Self {
            tire_friction: self.tire_friction,
            damping_rate: self.damping_rate,
            max_steer_angle: self.max_steer_angle,
            radius: self.radius,
            max_brake_torque: self.max_brake_torque,
            max_handbrake_torque: self.max_handbrake_torque,
            lat_stiff_max_load: self.lat_stiff_max_load,
            lat_stiff_value: self.lat_stiff_value,
            long_stiff_value: self.long_stiff_value,
            position: self.position.clone(),
        }
    }
}

// carla_rust::sensor
// SAFETY: FfiSensorData is a SharedPtr wrapper to sensor data.
// The underlying data is immutable after creation and SharedPtr ensures thread-safe ownership.
unsafe impl Send for crate::carla_rust::sensor::FfiSensorData {}
unsafe impl Sync for crate::carla_rust::sensor::FfiSensorData {}

// carla_rust::sensor::data
impl Clone for crate::carla_rust::sensor::data::FfiColor {
    fn clone(&self) -> Self {
        Self {
            b: self.b,
            g: self.g,
            r: self.r,
            a: self.a,
        }
    }
}

// carla_rust::sensor::data
// SAFETY: All sensor data types are SharedPtr wrappers to immutable sensor measurements.
// The data is captured at a point in time and never modified. SharedPtr ensures thread-safe
// reference counting, making all sensor data types safe to send and share across threads.

// CollisionEvent contains immutable collision data (actor refs, impulse).
unsafe impl Send for crate::carla_rust::sensor::data::FfiCollisionEvent {}
unsafe impl Sync for crate::carla_rust::sensor::data::FfiCollisionEvent {}

// GnssMeasurement contains GPS coordinate data (latitude, longitude, altitude).
unsafe impl Send for crate::carla_rust::sensor::data::FfiGnssMeasurement {}
unsafe impl Sync for crate::carla_rust::sensor::data::FfiGnssMeasurement {}

// Image contains pixel data that is immutable after capture.
unsafe impl Send for crate::carla_rust::sensor::data::FfiImage {}
unsafe impl Sync for crate::carla_rust::sensor::data::FfiImage {}

// ImuMeasurement contains IMU sensor data (accelerometer, gyroscope, compass).
unsafe impl Send for crate::carla_rust::sensor::data::FfiImuMeasurement {}
unsafe impl Sync for crate::carla_rust::sensor::data::FfiImuMeasurement {}

// LaneInvasionEvent contains immutable lane invasion data.
unsafe impl Send for crate::carla_rust::sensor::data::FfiLaneInvasionEvent {}
unsafe impl Sync for crate::carla_rust::sensor::data::FfiLaneInvasionEvent {}

// LidarMeasurement contains immutable point cloud data.
unsafe impl Send for crate::carla_rust::sensor::data::FfiLidarMeasurement {}
unsafe impl Sync for crate::carla_rust::sensor::data::FfiLidarMeasurement {}

// ObstacleDetectionEvent contains immutable obstacle detection data.
unsafe impl Send for crate::carla_rust::sensor::data::FfiObstacleDetectionEvent {}
unsafe impl Sync for crate::carla_rust::sensor::data::FfiObstacleDetectionEvent {}

// RadarMeasurement contains immutable radar detection data.
unsafe impl Send for crate::carla_rust::sensor::data::FfiRadarMeasurement {}
unsafe impl Sync for crate::carla_rust::sensor::data::FfiRadarMeasurement {}

// SemanticLidarMeasurement contains immutable semantic point cloud data.
unsafe impl Send for crate::carla_rust::sensor::data::FfiSemanticLidarMeasurement {}
unsafe impl Sync for crate::carla_rust::sensor::data::FfiSemanticLidarMeasurement {}

// carla_rust::traffic_manager
// SAFETY: FfiTrafficManager is a thread-safe handle to the traffic manager.
// Operations are synchronized internally by the CARLA server.
unsafe impl Send for crate::carla_rust::traffic_manager::FfiTrafficManager {}
unsafe impl Sync for crate::carla_rust::traffic_manager::FfiTrafficManager {}

// SAFETY: FfiAction contains immutable traffic action data.
// It is safe to send and share across threads.
unsafe impl Send for crate::carla_rust::traffic_manager::FfiAction {}
unsafe impl Sync for crate::carla_rust::traffic_manager::FfiAction {}

// SAFETY: FfiActionBuffer is a read-only buffer of traffic actions.
// The buffer is immutable after creation and can be safely shared.
unsafe impl Send for crate::carla_rust::traffic_manager::FfiActionBuffer {}
unsafe impl Sync for crate::carla_rust::traffic_manager::FfiActionBuffer {}
