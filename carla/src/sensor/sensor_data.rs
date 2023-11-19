use crate::geom::TransformExt;
use carla_sys::carla_rust::sensor::FfiSensorData;
use cxx::SharedPtr;
use derivative::Derivative;
use nalgebra::Isometry3;
use static_assertions::assert_impl_all;

/// The base trait for sensor data types.
pub trait SensorDataBase {
    /// Gets the native C++ object.
    fn cxx_sensor_data(&self) -> SharedPtr<FfiSensorData>;

    /// Gets the frame number of the data.
    fn frame(&self) -> usize {
        self.cxx_sensor_data().GetFrame()
    }

    /// Gets the timestamp of the data.
    fn timestamp(&self) -> f64 {
        self.cxx_sensor_data().GetTimestamp()
    }

    /// Gets the transformation of the sensor where the data was
    /// perceived.
    fn sensor_transform(&self) -> Isometry3<f32> {
        self.cxx_sensor_data().GetSensorTransform().to_na()
    }
}

/// The base sensor data type.
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

assert_impl_all!(SensorData: Send, Sync);
