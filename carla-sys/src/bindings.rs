// Suppress clippy lints for autocxx-generated FFI bindings
#![allow(clippy::new_ret_no_self)]
#![allow(clippy::too_many_arguments)]
#![allow(clippy::excessive_precision)]
#![allow(clippy::missing_safety_doc)]
#![allow(clippy::needless_lifetimes)]
#![allow(clippy::should_implement_trait)]
#![allow(clippy::len_without_is_empty)]

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
    #include "carla/rpc/VehicleLightState.h"
    #include "carla/rpc/VehicleDoor.h"
    #include "carla/rpc/VehicleWheels.h"
    #include "carla/rpc/VehicleAckermannControl.h"
    #include "carla/rpc/WalkerControl.h"
    #include "carla/rpc/WalkerBoneControlIn.h"
    #include "carla/rpc/WalkerBoneControlOut.h"
    #include "carla/rpc/BoneTransformDataIn.h"
    #include "carla/rpc/BoneTransformDataOut.h"
    #include "carla/rpc/OpendriveGenerationParameters.h"
    #include "carla/rpc/TrafficLightState.h"
    #include "carla/rpc/GearPhysicsControl.h"
    #include "carla/rpc/WeatherParameters.h"
    #include "carla/rpc/ObjectLabel.h"
    #include "carla/rpc/VehicleFailureState.h"
    #include "carla/rpc/VehicleTelemetryData.h"
    #include "carla/rpc/WheelTelemetryData.h"

    #include "carla/trafficmanager/Constants.h"
    #include "carla/trafficmanager/TrafficManager.h"
    #include "carla/trafficmanager/SimpleWaypoint.h"

    #include "carla/client/Waypoint.h"
    #include "carla/client/Sensor.h"
    #include "carla/client/Vehicle.h"
    #include "carla/client/Walker.h"
    #include "carla/client/WalkerAIController.h"
    #include "carla/client/TrafficLight.h"
    #include "carla/client/TrafficSign.h"
    #include "carla/client/Junction.h"
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
    #include "carla/client/Timestamp.h"
    #include "carla/client/Light.h"

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
    generate!("carla_rust::client::FfiWalker")
    generate!("carla_rust::client::FfiWalkerAIController")
    generate!("carla_rust::client::FfiDebugHelper")
    generate!("carla_rust::client::FfiDebugHelper_DrawPoint")
    generate!("carla_rust::client::FfiDebugHelper_DrawLine")
    generate!("carla_rust::client::FfiDebugHelper_DrawArrow")
    generate!("carla_rust::client::FfiDebugHelper_DrawBox")
    generate!("carla_rust::client::FfiDebugHelper_DrawString")
    generate!("carla_rust::client::FfiCommandBatch")
    generate!("carla_rust::client::FfiCommandResponse")
    generate!("carla_rust::client::FfiWaypointPair")
    generate!("carla_rust::client::FfiActorSnapshot")
    generate!("carla_rust::client::FfiActorSnapshotList")
    generate_pod!("carla_rust::client::FfiClientLightState")
    generate_pod!("carla_rust::geom::FfiLocation")
    generate_pod!("carla_rust::geom::FfiRotation")
    generate_pod!("carla_rust::geom::FfiVector2D")
    generate_pod!("carla_rust::geom::FfiVector3D")
    generate_pod!("carla_rust::geom::FfiGeoLocation")
    generate_pod!("carla_rust::geom::FfiTransform")
    generate_pod!("carla_rust::geom::FfiBoundingBox")
    generate_pod!("carla_rust::sensor::data::FfiColor")
    generate_pod!("carla_rust::sensor::data::FfiLidarDetection")
    generate_pod!("carla_rust::sensor::data::FfiSemanticLidarDetection")
    generate_pod!("carla_rust::sensor::data::FfiOpticalFlowPixel")
    generate!("carla_rust::sensor::data::FfiDVSEventArray")
    generate!("carla_rust::sensor::data::FfiOpticalFlowImage")
    generate_pod!("carla_rust::rpc::FfiLabelledPoint")
    generate_pod!("carla_rust::rpc::FfiRpcColor")
    generate_pod!("carla_rust::rpc::FfiRpcLightState")
    generate_pod!("carla_rust::rpc::FfiRpcLightGroup")
    generate!("carla_rust::rpc::FfiBoneTransformDataIn")
    generate!("carla_rust::rpc::FfiBoneTransformDataOut")
    generate!("carla_rust::rpc::FfiWalkerBoneControlIn")
    generate!("carla_rust::rpc::FfiWalkerBoneControlOut")
    generate!("carla_rust::rpc::FfiVehicleTelemetryData")

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

    // carla::road::element
    generate_pod!("carla::road::element::LaneMarking_Type")
    generate_pod!("carla::road::element::LaneMarking_Color")
    generate_pod!("carla::road::element::LaneMarking_LaneChange")

    // carla::rpc
    generate!("carla::rpc::OpendriveGenerationParameters")
    generate_pod!("carla::rpc::AttachmentType")
    generate_pod!("carla::rpc::VehicleControl")
    generate_pod!("carla::rpc::VehicleAckermannControl")
    generate_pod!("carla::rpc::AckermannControllerSettings")
    generate_pod!("carla::rpc::WalkerControl")
    generate!("carla::rpc::WalkerBoneControlIn")
    generate!("carla::rpc::WalkerBoneControlOut")
    generate!("carla::rpc::BoneTransformDataIn")
    generate!("carla::rpc::BoneTransformDataOut")
    generate!("carla::rpc::VehicleLightState")
    generate_pod!("carla::rpc::VehicleDoor")
    generate_pod!("carla::rpc::VehicleWheelLocation")
    generate!("carla::rpc::TrafficLightState")
    generate!("carla::rpc::LabelledPoint")
    generate!("carla::rpc::EpisodeSettings")
    generate_pod!("carla::rpc::ActorAttributeType")
    generate_pod!("carla::rpc::GearPhysicsControl")
    generate_pod!("carla::rpc::WheelPhysicsControl")
    generate_pod!("carla::rpc::WeatherParameters")
    generate_pod!("carla::rpc::CityObjectLabel")
    generate_pod!("carla::rpc::ActorState")
    generate_pod!("carla::rpc::LightId")
    generate_pod!("carla::rpc::VehicleFailureState")
    generate_pod!("carla::rpc::WheelTelemetryData")

    // carla::client
    generate!("carla::client::Vehicle")
    generate!("carla::client::Walker")
    generate!("carla::client::WalkerAIController")
    generate!("carla::client::TrafficLight")
    generate!("carla::client::TrafficSign")
    generate!("carla::client::Junction")
    generate!("carla::client::ActorBlueprint")
    generate!("carla::client::ActorList")
    generate!("carla::client::Landmark")
    generate!("carla::client::Light")
    generate!("carla::client::LaneInvasionSensor")
    generate!("carla::client::WorldSnapshot")
    generate_pod!("carla::client::Timestamp")
    generate!("carla::client::Light")

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
    generate!("carla::sensor::data::DVSEvent")
    generate!("carla::sensor::data::DVSEventArray")
    generate!("carla::sensor::data::OpticalFlowImage")

    // carla::traffic_manager
    generate!("carla::traffic_manager::RoadOption")

    // block offending types
    block!("carla::client::Sensor_CallbackFunctionType")
    block!("carla::geom::CubicPolynomial")  // Contains std::array<double, 4> which autocxx doesn't support
    block!("carla::road::element::RoadInfoLaneWidth")  // Uses CubicPolynomial
    block!("carla::road::element::RoadInfoLaneHeight")  // Uses CubicPolynomial
    block!("carla::road::element::RoadInfoLaneBorder")  // Uses CubicPolynomial
    block!("carla::road::element::RoadInfoElevation")  // Uses CubicPolynomial
    block!("carla::road::element::Geometry")  // Uses CubicPolynomial
    block!("carla::road::element::RoadInfoLaneOffset")  // Uses CubicPolynomial
    block!("carla::road::LaneSection")  // Uses CubicPolynomial

    // bad types
    // generate!("carla::client::Sensor")
    // generate!("carla::rpc::MapLayer")
    // generate!("carla::client::Map")
    // generate!("carla::client::World")
    // generate!("carla::client::BlueprintLibrary")
    // generate!("carla::client::LightManager")
    // generate!("carla::traffic_manager::TrafficManager")
    // generate!("carla::sensor::data::Color")
    // generate!("carla::rpc::LightState")
}

pub use ffi::*;
