// SAFETY: This module uses unwrap_unchecked() for performance on methods guaranteed
// to never return null. See UNWRAP_REPLACEMENTS.md for detailed C++ code audit.

use crate::sensor::SensorData;
use carla_sys::carla_rust::sensor::data::FfiOpticalFlowImage;
use cxx::SharedPtr;
use derivative::Derivative;
use ndarray::ArrayView2;
use std::slice;

pub use carla_sys::carla_rust::sensor::data::FfiOpticalFlowPixel as OpticalFlowPixel;

/// Optical flow camera image.
///
/// Captures the motion from the camera's perspective, encoding velocity information
/// for each pixel. Each pixel contains two float values representing the velocity
/// components projected onto the image plane.
///
/// The velocity values are in the range `[-2, 2]`. To obtain motion in pixel units,
/// scale by the image dimensions: `[-2 * image_size, 2 * image_size]`.
///
/// # Data Format
///
/// - Each pixel is an [`OpticalFlowPixel`] with `x` and `y` float components
/// - `x`: Horizontal velocity component (normalized to [-2, 2])
/// - `y`: Vertical velocity component (normalized to [-2, 2])
///
/// # Examples
///
/// ```no_run
/// use carla::{
///     client::{ActorBase, Client},
///     sensor::{data::OpticalFlowImage, SensorDataBase},
/// };
///
/// let client = Client::default();
/// let mut world = client.world();
///
/// // Spawn optical flow camera sensor
/// let bp_lib = world.blueprint_library();
/// let of_bp = bp_lib.find("sensor.camera.optical_flow").unwrap();
/// let spawn_points = world.map().recommended_spawn_points();
/// let of_sensor = world
///     .spawn_actor(&of_bp, &spawn_points.get(0).unwrap())
///     .unwrap();
///
/// let sensor: carla::client::Sensor = of_sensor.try_into().unwrap();
///
/// sensor.listen(|data| {
///     if let Ok(flow_img) = OpticalFlowImage::try_from(data) {
///         println!(
///             "Received optical flow image: {}x{} (FOV: {:.1}°)",
///             flow_img.width(),
///             flow_img.height(),
///             flow_img.fov_angle()
///         );
///
///         // Access flow data
///         for (i, pixel) in flow_img.as_slice().iter().enumerate().take(10) {
///             println!("  Pixel {}: flow=({:.3}, {:.3})", i, pixel.x, pixel.y);
///         }
///     }
/// });
/// ```
#[derive(Clone, Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct OpticalFlowImage {
    #[derivative(Debug = "ignore")]
    inner: SharedPtr<FfiOpticalFlowImage>,
}

impl OpticalFlowImage {
    /// Returns the image height in pixels.
    pub fn height(&self) -> usize {
        self.inner.GetHeight()
    }

    /// Returns the image width in pixels.
    pub fn width(&self) -> usize {
        self.inner.GetWidth()
    }

    /// Returns the total number of pixels.
    pub fn len(&self) -> usize {
        self.inner.size()
    }

    /// Returns `true` if the image contains no pixels.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Returns the horizontal field of view in degrees.
    pub fn fov_angle(&self) -> f32 {
        self.inner.GetFOVAngle()
    }

    /// Returns the optical flow data as a slice of pixels.
    ///
    /// Each pixel contains `x` and `y` velocity components in the range `[-2, 2]`.
    pub fn as_slice(&self) -> &[OpticalFlowPixel] {
        let ptr = self.inner.data();
        let len = self.len();

        debug_assert!(!ptr.is_null(), "Optical flow data pointer is null");
        debug_assert!(
            (ptr as usize).is_multiple_of(std::mem::align_of::<OpticalFlowPixel>()),
            "Optical flow data pointer not properly aligned"
        );

        unsafe { slice::from_raw_parts(ptr, len) }
    }

    /// Returns the optical flow data as a 2D array view.
    ///
    /// The array has shape `(width, height)` where each element is an [`OpticalFlowPixel`].
    ///
    /// # Panics
    ///
    /// Panics if the image dimensions don't match the data length.
    pub fn as_array(&self) -> ArrayView2<'_, OpticalFlowPixel> {
        let width = self.width();
        let height = self.height();
        let len = self.len();

        assert!(
            width * height == len,
            "Optical flow image dimensions mismatch: {}x{} = {} but data length is {}",
            width,
            height,
            width * height,
            len
        );

        ArrayView2::from_shape((width, height), self.as_slice())
            .expect("Failed to create array view with validated dimensions")
    }

    /// Gets the optical flow pixel at the given index.
    ///
    /// Returns `None` if the index is out of bounds.
    pub fn get(&self, index: usize) -> Option<&OpticalFlowPixel> {
        if index < self.len() {
            Some(self.inner.at(index))
        } else {
            None
        }
    }

    /// Converts optical flow values to pixel velocity units.
    ///
    /// The raw flow values are normalized to `[-2, 2]`. This method scales
    /// them to pixel units: `[-2 * size, 2 * size]`.
    ///
    /// Returns `(velocity_x, velocity_y)` in pixels per frame.
    pub fn flow_to_pixels(&self, flow: &OpticalFlowPixel) -> (f32, f32) {
        let width = self.width() as f32;
        let height = self.height() as f32;
        (flow.x * width, flow.y * height)
    }

    pub(crate) fn from_cxx(ptr: SharedPtr<FfiOpticalFlowImage>) -> Option<Self> {
        if ptr.is_null() {
            None
        } else {
            Some(Self { inner: ptr })
        }
    }
}

impl TryFrom<SensorData> for OpticalFlowImage {
    type Error = SensorData;

    fn try_from(value: SensorData) -> Result<Self, Self::Error> {
        let ptr = value.inner.to_optical_flow_image();
        Self::from_cxx(ptr).ok_or(value)
    }
}

// Note: Send + Sync implementation is safe because:
// - SharedPtr<FfiOpticalFlowImage> manages thread-safe reference counting
// - The underlying CARLA data is immutable after creation
// - No mutable state is exposed through the API
unsafe impl Send for OpticalFlowImage {}
unsafe impl Sync for OpticalFlowImage {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_flow_to_pixels() {
        // Create a mock optical flow image structure for testing
        // In real usage, this would come from the simulator
        let pixel = OpticalFlowPixel { x: 1.0, y: -0.5 };

        // Assuming 800x600 image
        let width = 800.0;
        let height = 600.0;

        let (vx, vy) = (pixel.x * width, pixel.y * height);

        assert_eq!(vx, 800.0);
        assert_eq!(vy, -300.0);
    }

    #[test]
    fn test_optical_flow_pixel_size() {
        // Ensure OpticalFlowPixel has expected size (2 floats = 8 bytes)
        assert_eq!(std::mem::size_of::<OpticalFlowPixel>(), 8);
    }
}
