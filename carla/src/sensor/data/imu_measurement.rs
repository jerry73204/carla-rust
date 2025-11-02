use crate::{geom::Vector3D, sensor::SensorData};
use carla_sys::carla_rust::sensor::data::FfiImuMeasurement;
use cxx::SharedPtr;
use derivative::Derivative;
use nalgebra::Vector3;
use static_assertions::assert_impl_all;

#[derive(Clone, Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct ImuMeasurement {
    #[derivative(Debug = "ignore")]
    inner: SharedPtr<FfiImuMeasurement>,
}

impl ImuMeasurement {
    pub fn accelerometer(&self) -> Vector3<f32> {
        // SAFETY: carla::geom::Vector3D and FfiVector3D have identical memory layout
        unsafe {
            let cpp_vec = self.inner.GetAccelerometer();
            Vector3D::from_ffi((&cpp_vec as *const _ as *const crate::geom::FfiVector3D).read())
                .to_na()
        }
    }

    pub fn compass(&self) -> f32 {
        self.inner.GetCompass()
    }

    pub fn gyroscope(&self) -> Vector3<f32> {
        // SAFETY: carla::geom::Vector3D and FfiVector3D have identical memory layout
        unsafe {
            let cpp_vec = self.inner.GetGyroscope();
            Vector3D::from_ffi((&cpp_vec as *const _ as *const crate::geom::FfiVector3D).read())
                .to_na()
        }
    }

    pub(crate) fn from_cxx(ptr: SharedPtr<FfiImuMeasurement>) -> Option<Self> {
        if ptr.is_null() {
            None
        } else {
            Some(Self { inner: ptr })
        }
    }
}

impl TryFrom<SensorData> for ImuMeasurement {
    type Error = SensorData;

    fn try_from(value: SensorData) -> Result<Self, Self::Error> {
        let ptr = value.inner.to_imu_measurement();
        Self::from_cxx(ptr).ok_or(value)
    }
}

assert_impl_all!(ImuMeasurement: Send, Sync);
