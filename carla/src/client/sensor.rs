use carla_sys::carla::{client::Sensor, sensor::SensorData};
use cxx::SharedPtr;

use super::ActorBase;

pub trait SensorBase: ActorBase {
    fn cxx_sensor(&self) -> SharedPtr<Sensor>;

    fn listen<F>(callback: F)
    where
        F: FnMut(SensorData) + Send + 'static;
}
