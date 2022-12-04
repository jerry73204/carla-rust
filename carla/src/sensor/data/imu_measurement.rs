use crate::{geom::Vector3D, sensor::SensorData};
use carla_sys::carla_rust::sensor::data::FfiImuMeasurement;
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
