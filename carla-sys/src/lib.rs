use autocxx::prelude::*;

pub use ffi::*;

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

    safety!(unsafe_ffi)

    // carla_rust
    generate_ns!("carla_rust")

    // carla
    generate!("carla::SharedPtr")
    generate!("carla::time_duration")

    // carla::geom
    generate_ns!("carla::geom")

    // carla::traffic_manager
    generate_ns!("carla::traffic_manager::constants")

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
}
