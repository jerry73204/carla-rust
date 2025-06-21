//! Camera sensor implementation.

use crate::{
    actor::{ActorFfi, Sensor, SensorFfi},
    sensor_data::{
        DepthImageData, ImageData, InstanceSegmentationImageData, RGBImageData,
        SemanticSegmentationImageData,
    },
};

/// Camera sensor types supported by CARLA.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CameraType {
    /// RGB camera
    RGB,
    /// Depth camera
    Depth,
    /// Semantic segmentation camera
    SemanticSegmentation,
    /// Instance segmentation camera
    InstanceSegmentation,
    /// Optical flow camera
    OpticalFlow,
    /// Dynamic Vision Sensor (DVS)
    DVS,
}

impl From<carla_sys::CameraType> for CameraType {
    fn from(cxx_type: carla_sys::CameraType) -> Self {
        match cxx_type {
            carla_sys::CameraType::RGB => CameraType::RGB,
            carla_sys::CameraType::Depth => CameraType::Depth,
            carla_sys::CameraType::SemanticSegmentation => CameraType::SemanticSegmentation,
            carla_sys::CameraType::InstanceSegmentation => CameraType::InstanceSegmentation,
            carla_sys::CameraType::OpticalFlow => CameraType::OpticalFlow,
            carla_sys::CameraType::DVS => CameraType::DVS,
        }
    }
}

/// Camera sensor for image data.
#[derive(Debug)]
pub struct Camera(Sensor);

impl Camera {
    /// Get the camera type (RGB, Depth, Semantic, etc.)
    pub fn camera_type(&self) -> CameraType {
        CameraType::from(self.as_sensor_ffi().get_camera_type())
    }

    /// Get the last image data from the camera.
    pub fn last_image_data(&self) -> Option<ImageData> {
        let cxx_data = self.as_sensor_ffi().get_last_image_data();
        if cxx_data.is_empty() {
            None
        } else {
            Some(ImageData::from_cxx(cxx_data))
        }
    }

    /// Get image data into a provided buffer.
    /// Returns true if data was copied successfully.
    pub fn image_data_buffer(&self, buffer: &mut [u8]) -> bool {
        self.as_sensor_ffi().get_image_data_buffer(buffer)
    }

    /// Get the last captured RGB image data.
    ///
    /// Returns None if the camera is not an RGB camera or no data is available.
    pub fn rgb_data(&self) -> Option<RGBImageData> {
        if self.camera_type() != CameraType::RGB {
            return None;
        }

        self.last_image_data().map(RGBImageData::new)
    }

    /// Get the last captured depth data.
    ///
    /// Returns None if the camera is not a depth camera or no data is available.
    pub fn depth_data(&self) -> Option<DepthImageData> {
        if self.camera_type() != CameraType::Depth {
            return None;
        }

        self.last_image_data().map(DepthImageData::new)
    }

    /// Get the last captured semantic segmentation data.
    ///
    /// Returns None if the camera is not a semantic segmentation camera or no data is available.
    pub fn semantic_segmentation_data(&self) -> Option<SemanticSegmentationImageData> {
        if self.camera_type() != CameraType::SemanticSegmentation {
            return None;
        }

        self.last_image_data()
            .map(SemanticSegmentationImageData::new)
    }

    /// Get the last captured instance segmentation data.
    ///
    /// Returns None if the camera is not an instance segmentation camera or no data is available.
    pub fn instance_segmentation_data(&self) -> Option<InstanceSegmentationImageData> {
        if self.camera_type() != CameraType::InstanceSegmentation {
            return None;
        }

        self.last_image_data()
            .map(InstanceSegmentationImageData::new)
    }
}

impl SensorFfi for Camera {
    fn as_sensor_ffi(&self) -> &carla_sys::SensorWrapper {
        self.0.as_sensor_ffi()
    }
}

// Implement conversion traits using the macro
crate::impl_sensor_conversions!(Camera, is_camera);
