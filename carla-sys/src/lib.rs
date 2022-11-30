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
    // #include "carla/rpc/MapLayer.h"
    #include "carla/rpc/OpendriveGenerationParameters.h"

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


    safety!(unsafe_ffi)
    generate_ns!("carla::geom")
    generate_ns!("carla_rust")

    generate!("carla::SharedPtr")
    generate!("carla::time_duration")

    generate!("carla::rpc::AttachmentType")
    generate!("carla::rpc::OpendriveGenerationParameters")

    generate!("carla::client::Waypoint")
    generate!("carla::client::Sensor")
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


    // bad types
    // generate!("carla::rpc::MapLayer")
    // generate!("carla::client::Map")
    // generate!("carla::client::World")
    // generate!("carla::client::BlueprintLibrary")
    // generate!("carla::client::LightManager")
}
