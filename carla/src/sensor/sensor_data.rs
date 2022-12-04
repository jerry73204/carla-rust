use crate::geom::TransformExt;
use carla_sys::carla_rust::sensor::FfiSensorData;
use cxx::SharedPtr;
use derivative::Derivative;
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
        self.cxx_sensor_data().GetSensorTransform().to_na()
    }
}

#[derive(Clone, Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct SensorData {
    #[derivative(Debug = "ignore")]
    pub(crate) inner: SharedPtr<FfiSensorData>,
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
