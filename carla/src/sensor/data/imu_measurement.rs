use crate::{geom::Vector3D, sensor::SensorData};
use carla_sys::carla_rust::sensor::data::FfiImuMeasurement;
use cxx::SharedPtr;
use derivative::Derivative;
use static_assertions::assert_impl_all;

/// IMU (Inertial Measurement Unit) sensor measurement data.
///
/// This type represents data from an IMU sensor, including accelerometer (linear acceleration),
/// gyroscope (angular velocity), and compass (orientation) readings. IMU sensors are useful
/// for vehicle dynamics analysis and state estimation.
///
/// Corresponds to [`carla.IMUMeasurement`] in the Python API.
#[cfg_attr(carla_version_0916, doc = "")]
#[cfg_attr(
    carla_version_0916,
    doc = " [`carla.IMUMeasurement`]: https://carla.readthedocs.io/en/0.9.16/python_api/#carla.IMUMeasurement"
)]
#[cfg_attr(carla_version_0915, doc = "")]
#[cfg_attr(
    carla_version_0915,
    doc = " [`carla.IMUMeasurement`]: https://carla.readthedocs.io/en/0.9.15/python_api/#carla.IMUMeasurement"
)]
#[cfg_attr(carla_version_0914, doc = "")]
#[cfg_attr(
    carla_version_0914,
    doc = " [`carla.IMUMeasurement`]: https://carla.readthedocs.io/en/0.9.14/python_api/#carla.IMUMeasurement"
)]
///
/// # Examples
///
/// ```no_run
/// use carla::{
///     client::{ActorBase, Client},
///     sensor::data::ImuMeasurement,
/// };
///
/// let client = Client::default();
/// let mut world = client.world();
///
/// # let bp_lib = world.blueprint_library();
/// # let imu_bp = bp_lib.filter("sensor.other.imu").get(0).unwrap();
/// # let spawn_points = world.map().recommended_spawn_points();
/// # let imu = world.spawn_actor(&imu_bp, spawn_points.get(0).unwrap()).unwrap();
/// # let sensor: carla::client::Sensor = imu.try_into().unwrap();
///
/// sensor.listen(|sensor_data| {
///     if let Ok(imu_data) = ImuMeasurement::try_from(sensor_data) {
///         let accel = imu_data.accelerometer();
///         let gyro = imu_data.gyroscope();
///         let heading = imu_data.compass();
///
///         println!(
///             "Acceleration: ({:.2}, {:.2}, {:.2}) m/s²",
///             accel.x, accel.y, accel.z
///         );
///         println!(
///             "Angular velocity: ({:.2}, {:.2}, {:.2}) rad/s",
///             gyro.x, gyro.y, gyro.z
///         );
///         println!("Compass heading: {:.2}°", heading);
///     }
/// });
/// ```
#[derive(Clone, Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct ImuMeasurement {
    #[derivative(Debug = "ignore")]
    inner: SharedPtr<FfiImuMeasurement>,
}

impl ImuMeasurement {
    /// Returns the accelerometer reading (linear acceleration) in m/s².
    ///
    /// The vector components represent acceleration in the sensor's local coordinate system:
    /// - x: forward/backward
    /// - y: left/right
    /// - z: up/down
    ///
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.IMUMeasurement.accelerometer](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.IMUMeasurement.accelerometer)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.IMUMeasurement.accelerometer](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.IMUMeasurement.accelerometer)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.IMUMeasurement.accelerometer](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.IMUMeasurement.accelerometer)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    pub fn accelerometer(&self) -> Vector3D {
        // SAFETY: carla::geom::Vector3D and FfiVector3D have identical memory layout
        unsafe {
            let cpp_vec = self.inner.GetAccelerometer();
            Vector3D::from_ffi((&cpp_vec as *const _ as *const crate::geom::FfiVector3D).read())
        }
    }

    /// Returns the compass heading in radians.
    ///
    /// The compass value represents the orientation relative to north (0 radians).
    ///
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.IMUMeasurement.compass](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.IMUMeasurement.compass)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.IMUMeasurement.compass](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.IMUMeasurement.compass)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.IMUMeasurement.compass](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.IMUMeasurement.compass)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    pub fn compass(&self) -> f32 {
        self.inner.GetCompass()
    }

    /// Returns the gyroscope reading (angular velocity) in rad/s.
    ///
    /// The vector components represent rotation rates around the sensor's local axes:
    /// - x: roll rate (rotation around forward axis)
    /// - y: pitch rate (rotation around lateral axis)
    /// - z: yaw rate (rotation around vertical axis)
    ///
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.IMUMeasurement.gyroscope](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.IMUMeasurement.gyroscope)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.IMUMeasurement.gyroscope](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.IMUMeasurement.gyroscope)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.IMUMeasurement.gyroscope](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.IMUMeasurement.gyroscope)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    pub fn gyroscope(&self) -> Vector3D {
        // SAFETY: carla::geom::Vector3D and FfiVector3D have identical memory layout
        unsafe {
            let cpp_vec = self.inner.GetGyroscope();
            Vector3D::from_ffi((&cpp_vec as *const _ as *const crate::geom::FfiVector3D).read())
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
