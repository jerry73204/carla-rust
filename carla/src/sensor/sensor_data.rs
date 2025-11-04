use crate::geom::Transform;
use carla_sys::carla_rust::sensor::FfiSensorData;
use cxx::SharedPtr;
use derivative::Derivative;
use static_assertions::assert_impl_all;

/// Base trait for all sensor data types.
///
/// Provides common methods for accessing metadata about sensor measurements,
/// including frame number, timestamp, and sensor transform.
///
/// All specific sensor data types (Image, LidarMeasurement, etc.) implement this trait.
pub trait SensorDataBase {
    /// Gets the native C++ object.
    fn cxx_sensor_data(&self) -> SharedPtr<FfiSensorData>;

    /// Returns the simulation frame number when this data was captured.
    ///
    /// Frame numbers are sequential and can be used to correlate data across sensors.
    fn frame(&self) -> usize {
        self.cxx_sensor_data().GetFrame()
    }

    /// Returns the simulation time (in seconds) when this data was captured.
    fn timestamp(&self) -> f64 {
        self.cxx_sensor_data().GetTimestamp()
    }

    /// Returns the world transform of the sensor at capture time.
    ///
    /// This is the sensor's position and orientation when the measurement was taken.
    fn sensor_transform(&self) -> Transform {
        Transform::from_ffi(self.cxx_sensor_data().GetSensorTransform())
    }
}

/// Generic sensor data container.
///
/// This is the base type received in sensor callbacks. Use [`TryFrom`] to convert
/// to specific sensor data types like [`data::Image`](crate::sensor::data::Image),
/// [`data::LidarMeasurement`](crate::sensor::data::LidarMeasurement), etc.
///
/// # Examples
///
/// ```no_run
/// use carla::{
///     client::{ActorBase, Client},
///     sensor::{data::Image, SensorDataBase},
/// };
///
/// let client = Client::default();
/// let mut world = client.world();
///
/// // ... spawn sensor ...
/// # let bp_lib = world.blueprint_library();
/// # let camera_bp = bp_lib.filter("sensor.camera.rgb").get(0).unwrap();
/// # let spawn_points = world.map().recommended_spawn_points();
/// # let camera = world.spawn_actor(&camera_bp, &spawn_points.get(0).unwrap()).unwrap();
/// # let sensor: carla::client::Sensor = camera.try_into().unwrap();
///
/// sensor.listen(|data| {
///     // Access common metadata
///     println!("Frame: {}, Time: {:.2}s", data.frame(), data.timestamp());
///
///     // Convert to specific type
///     if let Ok(image) = Image::try_from(data) {
///         println!("Received {}x{} image", image.width(), image.height());
///     }
/// });
/// ```
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
