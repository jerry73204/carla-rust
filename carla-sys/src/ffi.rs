use autocxx::prelude::*;

include_cpp! {
    #include "carla_rust.hpp"

    #include "carla/Time.h"
    #include "carla/Memory.h"

    #include "carla/geom/Location.h"
    #include "carla/geom/Rotation.h"
    #include "carla/geom/Transform.h"
    #include "carla/geom/Vector2D.h"
    #include "carla/geom/Vector3D.h"

    #include "carla/rpc/AttachmentType.h"
    #include "carla/rpc/VehicleControl.h"
    #include "carla/rpc/VehiclePhysicsControl.h"
    #include "carla/rpc/VehicleLightState.h"
    #include "carla/rpc/VehicleDoor.h"
    #include "carla/rpc/VehicleWheels.h"
    #include "carla/rpc/OpendriveGenerationParameters.h"
    #include "carla/rpc/TrafficLightState.h"
    #include "carla/rpc/EpisodeSettings.h"

    #include "carla/trafficmanager/Constants.h"
    #include "carla/trafficmanager/TrafficManager.h"

    #include "carla/client/Waypoint.h"
    #include "carla/client/Sensor.h"
    #include "carla/client/Vehicle.h"
    #include "carla/client/Walker.h"
    #include "carla/client/TrafficLight.h"
    #include "carla/client/TrafficSign.h"
    #include "carla/client/Junction.h"
    #include "carla/client/ActorAttribute.h"
    #include "carla/client/ActorBlueprint.h"
    #include "carla/client/ActorList.h"
    #include "carla/client/BlueprintLibrary.h"
    #include "carla/client/Landmark.h"
    #include "carla/client/LaneInvasionSensor.h"
    #include "carla/client/Light.h"
    #include "carla/client/LightManager.h"
    #include "carla/client/Map.h"
    #include "carla/client/World.h"
    #include "carla/client/WorldSnapshot.h"

    #include "carla/sensor/SensorData.h"
    #include "carla/sensor/data/Image.h"
    #include "carla/sensor/data/RadarMeasurement.h"
    #include "carla/sensor/data/RadarData.h"
    #include "carla/sensor/data/LidarMeasurement.h"
    #include "carla/sensor/data/SemanticLidarMeasurement.h"
    #include "carla/sensor/data/LidarData.h"
    #include "carla/sensor/data/SemanticLidarData.h"
    #include "carla/sensor/data/ObstacleDetectionEvent.h"
    #include "carla/sensor/data/CollisionEvent.h"
    #include "carla/sensor/data/LaneInvasionEvent.h"

    safety!(unsafe_ffi)

    // carla_rust
    generate_ns!("carla_rust")
    generate_pod!("carla_rust::geom::FfiLocation")
    generate_pod!("carla_rust::geom::FfiTransform")
    generate_pod!("carla_rust::sensor::data::FfiColor")
    generate_pod!("carla_rust::sensor::data::FfiLidarDetection")
    generate_pod!("carla_rust::sensor::data::FfiSemanticLidarDetection")

    // carla
    generate!("carla::SharedPtr")
    generate!("carla::time_duration")

    // carla::geom
    generate_ns!("carla::geom")
    generate_pod!("carla::geom::Vector2D")
    generate_pod!("carla::geom::Vector3D")
    generate_pod!("carla::geom::Rotation")
    generate_pod!("carla::geom::GeoLocation")

    // carla::traffic_manager
    generate_ns!("carla::traffic_manager::constants")

    // carla::rpc::element
    generate_pod!("carla::road::element::LaneMarking_Type")
    generate_pod!("carla::road::element::LaneMarking_Color")
    generate_pod!("carla::road::element::LaneMarking_LaneChange")

    // carla::rpc
    generate!("carla::rpc::OpendriveGenerationParameters")
    generate_pod!("carla::rpc::AttachmentType")
    generate_pod!("carla::rpc::VehicleControl")
    generate!("carla::rpc::VehiclePhysicsControl")
    generate!("carla::rpc::VehicleLightState")
    generate_pod!("carla::rpc::VehicleDoor")
    generate_pod!("carla::rpc::VehicleWheelLocation")
    generate!("carla::rpc::TrafficLightState")
    generate!("carla::rpc::LabelledPoint")
    generate!("carla::rpc::EpisodeSettings")

    // carla::client
    generate!("carla::client::Waypoint")
    generate!("carla::client::Vehicle")
    generate!("carla::client::Walker")
    generate!("carla::client::TrafficLight")
    generate!("carla::client::TrafficSign")
    generate!("carla::client::Junction")
    generate!("carla::client::ActorAttribute")
    generate!("carla::client::ActorBlueprint")
    generate!("carla::client::ActorList")
    generate!("carla::client::Landmark")
    generate!("carla::client::Light")
    generate!("carla::client::LaneInvasionSensor")
    generate!("carla::client::WorldSnapshot")

    // carla::sensor
    generate!("carla::sensor::SensorData")

    // carla::sensor::data
    generate!("carla::sensor::data::Image")
    generate!("carla::sensor::data::RadarMeasurement")
    generate_pod!("carla::sensor::data::RadarDetection")
    generate!("carla::sensor::data::RadarData")
    generate!("carla::sensor::data::LidarMeasurement")
    generate!("carla::sensor::data::SemanticLidarMeasurement")
    generate!("carla::sensor::data::LidarDetection")
    generate!("carla::sensor::data::LidarData")
    generate!("carla::sensor::data::SemanticLidarDetection")
    generate!("carla::sensor::data::SemanticLidarData")
    generate!("carla::sensor::data::IMUMeasurement")
    generate!("carla::sensor::data::GnssMeasurement")

    // block offending types
    block!("carla::client::Sensor_CallbackFunctionType")

    // bad types
    // generate!("carla::client::Sensor")
    // generate!("carla::rpc::MapLayer")
    // generate!("carla::client::Map")
    // generate!("carla::client::World")
    // generate!("carla::client::BlueprintLibrary")
    // generate!("carla::client::LightManager")
    // generate!("carla::traffic_manager::TrafficManager")
    // generate!("carla::sensor::data::Color")
}