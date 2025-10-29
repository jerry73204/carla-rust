// SAFETY: This module uses unwrap_unchecked() for performance on methods guaranteed
// to never return null. See UNWRAP_REPLACEMENTS.md for detailed C++ code audit.

use super::Color;
use crate::sensor::SensorData;
use carla_sys::carla_rust::sensor::data::FfiImage;
use cxx::SharedPtr;
use derivative::Derivative;
use ndarray::ArrayView2;
use static_assertions::assert_impl_all;
use std::slice;

#[derive(Clone, Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct Image {
    #[derivative(Debug = "ignore")]
    inner: SharedPtr<FfiImage>,
}

impl Image {
    pub fn height(&self) -> usize {
        self.inner.GetHeight()
    }

    pub fn width(&self) -> usize {
        self.inner.GetWidth()
    }

    pub fn len(&self) -> usize {
        self.inner.size()
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    pub fn fov_angle(&self) -> f32 {
        self.inner.GetFOVAngle()
    }

    pub fn as_slice(&self) -> &[Color] {
        let ptr = self.inner.data();
        let len = self.len();

        debug_assert!(!ptr.is_null(), "Image data pointer is null");
        debug_assert!(
            (ptr as usize).is_multiple_of(std::mem::align_of::<Color>()),
            "Image data pointer not properly aligned"
        );

        unsafe { slice::from_raw_parts(ptr, len) }
    }

    /// Returns the raw image data as a byte slice.
    ///
    /// The image data is in BGRA format with 4 bytes per pixel (Blue, Green, Red, Alpha).
    /// This provides zero-copy access to the underlying sensor data, useful for:
    /// - Custom image processing algorithms
    /// - Machine learning pipelines expecting raw bytes
    /// - FFI with external image processing libraries
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use carla::client::{Client, Sensor};
    /// use carla::sensor::data::Image;
    ///
    /// let client = Client::default();
    /// let world = client.world();
    /// # let bp_lib = world.blueprint_library();
    /// # let camera_bp = bp_lib.find("sensor.camera.rgb").unwrap();
    /// # let spawn_points = world.map().recommended_spawn_points();
    /// # let camera_actor = world.spawn_actor(&camera_bp, &spawn_points.get(0).unwrap()).unwrap();
    /// # let camera = Sensor::try_from(camera_actor).unwrap();
    ///
    /// camera.listen(|sensor_data| {
    ///     if let Ok(image) = Image::try_from(sensor_data) {
    ///         let raw_bytes = image.as_raw_bytes();
    ///         // raw_bytes is in BGRA format: [B, G, R, A, B, G, R, A, ...]
    ///         // Each pixel is 4 bytes
    ///         assert_eq!(raw_bytes.len(), image.width() * image.height() * 4);
    ///     }
    /// });
    /// ```
    ///
    /// # Format
    ///
    /// The data is in BGRA format (32 bits per pixel):
    /// - Byte 0: Blue (0-255)
    /// - Byte 1: Green (0-255)
    /// - Byte 2: Red (0-255)
    /// - Byte 3: Alpha (0-255)
    ///
    /// For RGB conversion, swap the B and R channels. For grayscale,
    /// use a weighted sum: `0.299*R + 0.587*G + 0.114*B`.
    pub fn as_raw_bytes(&self) -> &[u8] {
        let ptr = self.inner.data() as *const u8;
        let byte_len = self.len() * std::mem::size_of::<Color>();

        debug_assert!(!ptr.is_null(), "Image data pointer is null");

        unsafe { slice::from_raw_parts(ptr, byte_len) }
    }

    pub fn as_array(&self) -> ArrayView2<'_, Color> {
        let width = self.width();
        let height = self.height();
        let len = self.len();

        // Validate dimensions match the actual data length
        assert!(
            width * height == len,
            "Image dimensions mismatch: {}x{} = {} but data length is {}",
            width,
            height,
            width * height,
            len
        );

        ArrayView2::from_shape((width, height), self.as_slice())
            .expect("Failed to create array view with validated dimensions")
    }

    pub fn get(&self, index: usize) -> Option<&Color> {
        if index < self.inner.size() {
            Some(self.inner.at(index))
        } else {
            None
        }
    }

    /// Saves the image to disk at the specified path.
    ///
    /// The file format is determined by the file extension (e.g., ".png", ".jpg").
    /// CARLA images are in BGRA format and are converted to RGBA before saving.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use carla::client::{Client, Sensor};
    /// use carla::sensor::data::Image;
    ///
    /// let client = Client::default();
    /// let world = client.world();
    /// # let bp_lib = world.blueprint_library();
    /// # let camera_bp = bp_lib.find("sensor.camera.rgb").unwrap();
    /// # let spawn_points = world.map().recommended_spawn_points();
    /// # let camera_actor = world.spawn_actor(&camera_bp, &spawn_points.get(0).unwrap()).unwrap();
    /// # let camera = Sensor::try_from(camera_actor).unwrap();
    ///
    /// camera.listen(|sensor_data| {
    ///     if let Ok(image) = Image::try_from(sensor_data) {
    ///         if let Err(e) = image.save_to_disk("output/frame_001.png") {
    ///             eprintln!("Failed to save image: {}", e);
    ///         }
    ///     }
    /// });
    /// ```
    ///
    /// # Errors
    ///
    /// Returns an error if:
    /// - The file cannot be created or written
    /// - The path is invalid
    /// - The image format is not supported
    pub fn save_to_disk(&self, path: &str) -> std::io::Result<()> {
        use image::{ImageBuffer, Rgba};

        let width = self.width() as u32;
        let height = self.height() as u32;
        let pixels = self.as_slice();

        // Convert BGRA to RGBA
        let rgba_data: Vec<u8> = pixels
            .iter()
            .flat_map(|color| {
                // CARLA uses BGRA, convert to RGBA
                [color.r, color.g, color.b, color.a]
            })
            .collect();

        let img_buffer = ImageBuffer::<Rgba<u8>, Vec<u8>>::from_raw(width, height, rgba_data)
            .ok_or_else(|| {
                std::io::Error::new(
                    std::io::ErrorKind::InvalidData,
                    "Failed to create image buffer from pixel data",
                )
            })?;

        img_buffer
            .save(path)
            .map_err(|e| std::io::Error::other(format!("Failed to save image: {}", e)))
    }

    pub(crate) fn from_cxx(ptr: SharedPtr<FfiImage>) -> Option<Self> {
        if ptr.is_null() {
            None
        } else {
            Some(Self { inner: ptr })
        }
    }
}

impl TryFrom<SensorData> for Image {
    type Error = SensorData;

    fn try_from(value: SensorData) -> Result<Self, Self::Error> {
        let ptr = value.inner.to_image();
        Self::from_cxx(ptr).ok_or(value)
    }
}

assert_impl_all!(Image: Send, Sync);
