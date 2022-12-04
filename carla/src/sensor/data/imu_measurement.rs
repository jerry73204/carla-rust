use crate::geom::Vector3D;
use carla_sys::carla::sensor::data::IMUMeasurement as FfiImuMeasurement;
use cxx::SharedPtr;

#[derive(Clone)]
#[repr(transparent)]
pub struct ImuMeasurement {
    inner: SharedPtr<FfiImuMeasurement>,
}

impl ImuMeasurement {
    pub fn accelerometer(&self) -> Vector3D {
        self.inner.GetAccelerometer()
    }

    pub fn compass(&self) -> f32 {
        self.inner.GetCompass()
    }

    pub fn gyroscope(&self) -> Vector3D {
        self.inner.GetGyroscope()
    }
}
