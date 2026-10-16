use crate::sensor::SensorData;
use carla_sys::carla_rust::sensor::data::FfiCustomV2XEvent;
use cxx::SharedPtr;
use derivative::Derivative;
use static_assertions::assert_impl_all;

/// One message a custom V2X sensor received.
#[derive(Debug, Clone, PartialEq)]
pub struct CustomV2XMessage {
    /// Received power in dBm, as the sensor's path-loss model computed it.
    pub power_dbm: f32,
    /// The ITS station id in the message header.
    pub station_id: i64,
    /// The text the sender passed to [`Sensor::send`](crate::client::Sensor::send).
    pub message: String,
}

/// The messages a `sensor.other.v2x_custom` received in one frame (CARLA 0.9.16).
///
/// The sensor delivers a message only to receivers in range whose received power is
/// above their `receiver_sensitivity`; the power accounts for the scenario's path loss,
/// shadowing and the line of sight between the two antennas.
///
/// Corresponds to [`carla.CustomV2XEvent`] in the Python API.
///
/// [`carla.CustomV2XEvent`]: https://carla.readthedocs.io/en/0.9.16/python_api/#carla.CustomV2XEvent
///
/// # Examples
///
/// ```no_run
/// use carla::{client::Sensor, sensor::data::CustomV2XEvent};
///
/// # fn run(receiver: Sensor) -> carla::Result<()> {
/// receiver.listen(|data| {
///     if let Ok(event) = CustomV2XEvent::try_from(data) {
///         for msg in event.messages() {
///             println!(
///                 "{} dBm from {}: {}",
///                 msg.power_dbm, msg.station_id, msg.message
///             );
///         }
///     }
/// })?;
/// # Ok(())
/// # }
/// ```
#[derive(Clone, Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct CustomV2XEvent {
    #[derivative(Debug = "ignore")]
    inner: SharedPtr<FfiCustomV2XEvent>,
}

impl CustomV2XEvent {
    /// Number of messages received in this frame.
    pub fn len(&self) -> usize {
        self.inner.size()
    }

    /// Whether no message was received.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// The message at `index`, or `None` past the end.
    pub fn get(&self, index: usize) -> Option<CustomV2XMessage> {
        if index >= self.len() {
            return None;
        }
        let text = self.inner.message_at(index);
        Some(CustomV2XMessage {
            power_dbm: self.inner.power_at(index),
            station_id: self.inner.station_id_at(index),
            message: text.to_string_lossy().into_owned(),
        })
    }

    /// All messages received in this frame.
    pub fn messages(&self) -> Vec<CustomV2XMessage> {
        (0..self.len()).filter_map(|i| self.get(i)).collect()
    }

    pub(crate) fn from_cxx(ptr: SharedPtr<FfiCustomV2XEvent>) -> Option<Self> {
        if ptr.is_null() {
            None
        } else {
            Some(Self { inner: ptr })
        }
    }
}

impl TryFrom<SensorData> for CustomV2XEvent {
    type Error = SensorData;

    fn try_from(value: SensorData) -> Result<Self, Self::Error> {
        let ptr = value.inner.to_custom_v2x_event();
        Self::from_cxx(ptr).ok_or(value)
    }
}

assert_impl_all!(CustomV2XEvent: Send, Sync);
