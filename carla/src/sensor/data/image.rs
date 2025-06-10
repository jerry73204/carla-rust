use super::Color;
use crate::sensor::{SensorData, SensorDataBase};
use anyhow::{anyhow, Result};
use carla_sys::*;
use ndarray::ArrayView2;
use std::{ptr, slice};

#[derive(Clone, Debug)]
pub struct Image {
    inner: *mut carla_image_data_t,
}

impl Image {
    /// Create an Image from a raw C pointer.
    ///
    /// # Safety
    /// The pointer must be valid and not null.
    pub(crate) fn from_raw_ptr(ptr: *mut carla_image_data_t) -> Result<Self> {
        if ptr.is_null() {
            return Err(anyhow!("Null image data pointer"));
        }
        Ok(Self { inner: ptr })
    }

    pub fn height(&self) -> usize {
        unsafe { carla_image_get_height(self.inner) }
    }

    pub fn width(&self) -> usize {
        unsafe { carla_image_get_width(self.inner) }
    }

    pub fn len(&self) -> usize {
        unsafe { carla_image_get_size(self.inner) }
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    pub fn fov_angle(&self) -> f32 {
        unsafe { carla_image_get_fov_angle(self.inner) }
    }

    pub fn as_slice(&self) -> &[u8] {
        let len = self.len();
        let data = unsafe { carla_image_get_raw_data(self.inner) };
        if data.is_null() {
            &[]
        } else {
            unsafe { slice::from_raw_parts(data, len) }
        }
    }

    pub fn as_array(&self) -> ArrayView2<'_, u8> {
        let height = self.height();
        let width = self.width();
        let channels = if self.len() > 0 && height > 0 && width > 0 {
            self.len() / (height * width)
        } else {
            4 // Default to RGBA
        };

        if channels > 1 {
            ArrayView2::from_shape((height, width * channels), self.as_slice()).unwrap()
        } else {
            ArrayView2::from_shape((height, width), self.as_slice()).unwrap()
        }
    }

    pub fn get(&self, index: usize) -> Option<u8> {
        let slice = self.as_slice();
        slice.get(index).copied()
    }
}

impl SensorDataBase for Image {
    fn raw_ptr(&self) -> *mut carla_sensor_data_t {
        self.inner as *mut carla_sensor_data_t
    }
}

impl TryFrom<SensorData> for Image {
    type Error = SensorData;

    fn try_from(value: SensorData) -> std::result::Result<Self, Self::Error> {
        // Check if the sensor data is actually image data
        let data_type = value.data_type();
        if data_type == carla_sensor_data_type_t_CARLA_SENSOR_DATA_IMAGE {
            let image_ptr = unsafe { carla_sensor_data_as_image(value.inner) };
            if !image_ptr.is_null() {
                return Ok(Image { inner: image_ptr });
            }
        }
        Err(value)
    }
}

impl Drop for Image {
    fn drop(&mut self) {
        // Note: Image data is managed by the sensor data lifecycle
        // We don't need to free it separately
        self.inner = ptr::null_mut();
    }
}

// SAFETY: Image wraps a thread-safe C API
unsafe impl Send for Image {}
unsafe impl Sync for Image {}
