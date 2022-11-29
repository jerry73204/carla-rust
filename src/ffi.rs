use autocxx::prelude::*;
pub use ffi::*;

include_cpp! {
    #include "carla/geom/Location.h"
    #include "carla/geom/Rotation.h"
    #include "carla/geom/Transform.h"
    #include "carla/geom/Vector2D.h"
    #include "carla/geom/Vector3D.h"

    #include "carla/client/Client.h"
    #include "carla/client/Actor.h"
    #include "carla/client/Sensor.h"
    #include "carla/client/Vehicle.h"
    #include "carla/client/Walker.h"
    #include "carla/client/TrafficLight.h"
    #include "carla/client/TrafficSign.h"

    safety!(unsafe_ffi)
    generate_ns!("carla::geom")
    generate!("carla::client::Client")
    generate!("carla::client::Actor")
    generate!("carla::client::Sensor")
    generate!("carla::client::Vehicle")
    generate!("carla::client::Walker")
    generate!("carla::client::TrafficLight")
    generate!("carla::client::TrafficSign")
}
