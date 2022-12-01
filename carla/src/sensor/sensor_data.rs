use crate::geom::Transform;
use autocxx::prelude::*;
use carla_sys::carla_rust::sensor::FfiSensorData;
use cxx::SharedPtr;
use nalgebra::Isometry3;

pub trait SensorDataBase {
    fn cxx_sensor_data(&self) -> SharedPtr<FfiSensorData>;

    fn frame(&self) -> usize {
        self.cxx_sensor_data().GetFrame()
    }

    fn timestamp(&self) -> f64 {
        self.cxx_sensor_data().GetTimestamp()
    }

    fn sensor_transform(&self) -> Isometry3<f32> {
        let ptr = self
            .cxx_sensor_data()
            .GetSensorTransform()
            .within_unique_ptr();
        let transform = Transform::from_cxx(ptr);
        transform.to_na()
    }
}

#[derive(Clone)]
pub struct SensorData {
    inner: SharedPtr<FfiSensorData>,
}

impl SensorData {
    pub(crate) fn from_cxx(ptr: SharedPtr<FfiSensorData>) -> Self {
        Self { inner: ptr }
    }
}

impl SensorDataBase for SensorData {
    fn cxx_sensor_data(&self) -> SharedPtr<FfiSensorData> {
        self.inner.clone()
    }
}
