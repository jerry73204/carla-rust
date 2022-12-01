use carla_sys::carla::client::Sensor;
use cxx::SharedPtr;

use super::ActorBase;

pub trait SensorBase : ActorBase {
    fn cxx_sensor(&self) -> SharedPtr<Sensor>;
}
