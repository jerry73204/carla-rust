// SAFETY: This module uses unwrap_unchecked() for performance on methods guaranteed
// to never return null. See UNWRAP_REPLACEMENTS.md for detailed C++ code audit.

use crate::sensor::SensorData;
use carla_sys::carla_rust::sensor::data::FfiDVSEventArray;
use cxx::SharedPtr;
use derivative::Derivative;
use std::slice;

// Re-export DVSEvent from carla_sys (opaque type)
pub use carla_sys::carla::sensor::data::DVSEvent;

/// DVS (Dynamic Vision Sensor) event array.
///
/// Contains asynchronous brightness change events from a DVS camera sensor.
/// Unlike traditional cameras that capture frames at fixed intervals, DVS cameras
/// respond only to brightness changes, providing high dynamic range (140dB) and
/// microsecond temporal resolution without motion blur.
///
/// # Event Format
///
/// Each event `(x, y, t, pol)` represents a brightness change at pixel coordinates
/// `(x, y)` at time `t` with polarity `pol`:
/// - `pol = true` (positive): brightness increased
/// - `pol = false` (negative): brightness decreased
///
/// # Examples
///
/// ```no_run
/// use carla::{
///     client::{ActorBase, Client},
///     sensor::{data::DVSEventArray, SensorDataBase},
/// };
///
/// let client = Client::default();
/// let mut world = client.world();
///
/// // Spawn DVS camera sensor
/// let bp_lib = world.blueprint_library();
/// let dvs_bp = bp_lib.find("sensor.camera.dvs").unwrap();
/// let spawn_points = world.map().recommended_spawn_points();
/// let dvs_sensor = world
///     .spawn_actor(&dvs_bp, &spawn_points.get(0).unwrap())
///     .unwrap();
///
/// let sensor: carla::client::Sensor = dvs_sensor.try_into().unwrap();
///
/// sensor.listen(|data| {
///     if let Ok(dvs_data) = DVSEventArray::try_from(data) {
///         println!(
///             "Received {} DVS events ({}x{} FOV: {:.1}°)",
///             dvs_data.len(),
///             dvs_data.width(),
///             dvs_data.height(),
///             dvs_data.fov_angle()
///         );
///
///         // Process events
///         for event in dvs_data.as_slice() {
///             if event.pol {
///                 println!(
///                     "  Positive event at ({}, {}) t={}",
///                     event.x, event.y, event.t
///                 );
///             }
///         }
///     }
/// });
/// ```
#[derive(Clone, Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct DVSEventArray {
    #[derivative(Debug = "ignore")]
    inner: SharedPtr<FfiDVSEventArray>,
}

impl DVSEventArray {
    /// Returns the image height in pixels.
    pub fn height(&self) -> usize {
        self.inner.GetHeight()
    }

    /// Returns the image width in pixels.
    pub fn width(&self) -> usize {
        self.inner.GetWidth()
    }

    /// Returns the number of events in the array.
    pub fn len(&self) -> usize {
        self.inner.size()
    }

    /// Returns `true` if there are no events.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Returns the horizontal field of view in degrees.
    pub fn fov_angle(&self) -> f32 {
        self.inner.GetFOVAngle()
    }

    /// Returns the events as a slice.
    ///
    /// Events are in chronological order by timestamp.
    pub fn as_slice(&self) -> &[DVSEvent] {
        let ptr = self.inner.data();
        let len = self.len();

        debug_assert!(!ptr.is_null(), "DVS event data pointer is null");

        unsafe { slice::from_raw_parts(ptr, len) }
    }

    /// Gets a reference to the event at the given index.
    ///
    /// Returns `None` if the index is out of bounds.
    pub fn get(&self, index: usize) -> Option<&DVSEvent> {
        if index < self.len() {
            Some(self.inner.at(index))
        } else {
            None
        }
    }

    pub(crate) fn from_cxx(ptr: SharedPtr<FfiDVSEventArray>) -> Option<Self> {
        if ptr.is_null() {
            None
        } else {
            Some(Self { inner: ptr })
        }
    }
}

impl TryFrom<SensorData> for DVSEventArray {
    type Error = SensorData;

    fn try_from(value: SensorData) -> Result<Self, Self::Error> {
        let ptr = value.inner.to_dvs_event_array();
        Self::from_cxx(ptr).ok_or(value)
    }
}

// Note: Send + Sync implementation is safe because:
// - SharedPtr<FfiDVSEventArray> manages thread-safe reference counting
// - The underlying CARLA data is immutable after creation
// - No mutable state is exposed through the API
unsafe impl Send for DVSEventArray {}
unsafe impl Sync for DVSEventArray {}
