use std::fmt::{self, Debug, Formatter};

// carla::client
unsafe impl Send for crate::carla::client::ActorBlueprint {}
unsafe impl Sync for crate::carla::client::ActorBlueprint {}
unsafe impl Send for crate::carla::client::WorldSnapshot {}

// carla_rust::client
unsafe impl Send for crate::carla_rust::client::FfiClient {}
unsafe impl Send for crate::carla_rust::client::FfiWorld {}
unsafe impl Send for crate::carla_rust::client::FfiActor {}
unsafe impl Sync for crate::carla_rust::client::FfiActor {}
unsafe impl Send for crate::carla_rust::client::FfiBlueprintLibrary {}
unsafe impl Sync for crate::carla_rust::client::FfiBlueprintLibrary {}
unsafe impl Send for crate::carla_rust::client::FfiLandmark {}
unsafe impl Sync for crate::carla_rust::client::FfiLandmark {}
unsafe impl Send for crate::carla_rust::client::FfiMap {}
unsafe impl Sync for crate::carla_rust::client::FfiMap {}
unsafe impl Send for crate::carla_rust::client::FfiSensor {}
unsafe impl Sync for crate::carla_rust::client::FfiSensor {}
unsafe impl Send for crate::carla_rust::client::FfiVehicle {}
unsafe impl Sync for crate::carla_rust::client::FfiVehicle {}
unsafe impl Send for crate::carla_rust::client::FfiTrafficSign {}
unsafe impl Sync for crate::carla_rust::client::FfiTrafficSign {}
unsafe impl Send for crate::carla_rust::client::FfiTrafficLight {}
unsafe impl Sync for crate::carla_rust::client::FfiTrafficLight {}
unsafe impl Send for crate::carla_rust::client::FfiWaypoint {}
unsafe impl Sync for crate::carla_rust::client::FfiWaypoint {}
unsafe impl Send for crate::carla_rust::client::FfiActorList {}
unsafe impl Sync for crate::carla_rust::client::FfiActorList {}
unsafe impl Send for crate::carla_rust::client::FfiTransformList {}
unsafe impl Sync for crate::carla_rust::client::FfiTransformList {}
unsafe impl Send for crate::carla_rust::client::FfiLandmarkList {}
unsafe impl Sync for crate::carla_rust::client::FfiLandmarkList {}
unsafe impl Send for crate::carla_rust::client::FfiWorldSnapshot {}
unsafe impl Sync for crate::carla_rust::client::FfiWorldSnapshot {}

// carla::geom
unsafe impl Send for crate::carla::geom::Vector2D {}
unsafe impl Sync for crate::carla::geom::Vector2D {}
unsafe impl Send for crate::carla::geom::Vector3D {}
unsafe impl Sync for crate::carla::geom::Vector3D {}
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
            pitch: self.pitch.clone(),
            yaw: self.yaw.clone(),
            roll: self.roll.clone(),
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
unsafe impl Send for crate::carla_rust::geom::FfiLocation {}
unsafe impl Sync for crate::carla_rust::geom::FfiLocation {}
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
            x: self.x.clone(),
            y: self.y.clone(),
            z: self.z.clone(),
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
unsafe impl Send for crate::carla_rust::road::element::FfiLaneMarking {}

// carla::rpc
unsafe impl Send for crate::carla::rpc::EpisodeSettings {}
unsafe impl Send for crate::carla::rpc::GearPhysicsControl {}
unsafe impl Send for crate::carla::rpc::LabelledPoint {}
unsafe impl Send for crate::carla::rpc::OpendriveGenerationParameters {}
unsafe impl Send for crate::carla::rpc::VehicleLightState {}
unsafe impl Send for crate::carla::rpc::VehiclePhysicsControl {}
unsafe impl Send for crate::carla::rpc::WalkerBoneControlIn {}
unsafe impl Send for crate::carla::rpc::WalkerBoneControlOut {}
unsafe impl Send for crate::carla::rpc::WalkerControl {}
unsafe impl Send for crate::carla::rpc::WheelPhysicsControl {}
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
unsafe impl Send for crate::carla_rust::rpc::FfiLabelledPoint {}
unsafe impl Send for crate::carla_rust::rpc::FfiEpisodeSettings {}

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
unsafe impl Send for crate::carla_rust::sensor::data::FfiCollisionEvent {}
unsafe impl Sync for crate::carla_rust::sensor::data::FfiCollisionEvent {}
unsafe impl Send for crate::carla_rust::sensor::data::FfiGnssMeasurement {}
unsafe impl Sync for crate::carla_rust::sensor::data::FfiGnssMeasurement {}
unsafe impl Send for crate::carla_rust::sensor::data::FfiImage {}
unsafe impl Sync for crate::carla_rust::sensor::data::FfiImage {}
unsafe impl Send for crate::carla_rust::sensor::data::FfiImuMeasurement {}
unsafe impl Sync for crate::carla_rust::sensor::data::FfiImuMeasurement {}
unsafe impl Send for crate::carla_rust::sensor::data::FfiLaneInvasionEvent {}
unsafe impl Sync for crate::carla_rust::sensor::data::FfiLaneInvasionEvent {}
unsafe impl Send for crate::carla_rust::sensor::data::FfiLidarMeasurement {}
unsafe impl Sync for crate::carla_rust::sensor::data::FfiLidarMeasurement {}
unsafe impl Send for crate::carla_rust::sensor::data::FfiObstacleDetectionEvent {}
unsafe impl Sync for crate::carla_rust::sensor::data::FfiObstacleDetectionEvent {}
unsafe impl Send for crate::carla_rust::sensor::data::FfiRadarMeasurement {}
unsafe impl Sync for crate::carla_rust::sensor::data::FfiRadarMeasurement {}
unsafe impl Send for crate::carla_rust::sensor::data::FfiSemanticLidarMeasurement {}
unsafe impl Sync for crate::carla_rust::sensor::data::FfiSemanticLidarMeasurement {}

// carla_rust::traffic_manager
unsafe impl Send for crate::carla_rust::traffic_manager::FfiTrafficManager {}
unsafe impl Send for crate::carla_rust::traffic_manager::FfiAction {}
unsafe impl Send for crate::carla_rust::traffic_manager::FfiActionBuffer {}
