//! Camera sensor implementation.

use crate::{
    client::Sensor,
    error::{CarlaResult, SensorError},
    sensor::{
        DepthImageData, ImageData, InstanceSegmentationImageData, RGBImageData,
        SemanticSegmentationImageData,
    },
    traits::ActorT,
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

impl From<carla_cxx::CameraType> for CameraType {
    fn from(cxx_type: carla_cxx::CameraType) -> Self {
        match cxx_type {
            carla_cxx::CameraType::RGB => CameraType::RGB,
            carla_cxx::CameraType::Depth => CameraType::Depth,
            carla_cxx::CameraType::SemanticSegmentation => CameraType::SemanticSegmentation,
            carla_cxx::CameraType::InstanceSegmentation => CameraType::InstanceSegmentation,
            carla_cxx::CameraType::OpticalFlow => CameraType::OpticalFlow,
            carla_cxx::CameraType::DVS => CameraType::DVS,
        }
    }
}

/// Camera sensor that captures images.
pub struct CameraSensor {
    sensor: Sensor,
    camera_type: CameraType,
}

impl CameraSensor {
    /// Create a camera from a sensor.
    pub fn from_sensor(sensor: Sensor) -> CarlaResult<Self> {
        let wrapper = &sensor.inner;

        if !wrapper.is_camera() {
            return Err(crate::error::CarlaError::Actor(
                crate::error::ActorError::InvalidType {
                    expected: "Camera".to_string(),
                    actual: sensor.get_type_id(),
                },
            ));
        }

        let camera_type = CameraType::from(wrapper.get_camera_type());

        Ok(Self {
            sensor,
            camera_type,
        })
    }

    /// Get the camera type.
    pub fn camera_type(&self) -> CameraType {
        self.camera_type
    }

    /// Get the underlying sensor.
    pub fn sensor(&self) -> &Sensor {
        &self.sensor
    }

    /// Check if the camera is currently listening for data.
    pub fn is_listening(&self) -> bool {
        self.sensor.is_listening()
    }

    /// Start listening for camera data.
    pub fn listen<F>(&mut self, callback: F) -> Result<(), SensorError>
    where
        F: Fn(Vec<u8>) + Send + 'static,
    {
        self.sensor.listen(callback)
    }

    /// Stop listening for camera data.
    pub fn stop(&mut self) {
        self.sensor.stop();
    }

    /// Check if new image data is available.
    pub fn has_new_data(&self) -> bool {
        self.sensor.inner.has_new_data()
    }

    /// Get the last captured image data.
    ///
    /// Returns None if no data is available or if the sensor is not listening.
    pub fn get_image_data(&self) -> Option<ImageData> {
        let cxx_data = self.sensor.inner.get_last_image_data();

        if cxx_data.is_empty() {
            None
        } else {
            Some(ImageData::from_cxx(cxx_data))
        }
    }

    /// Get the last captured RGB image data.
    ///
    /// Returns None if the camera is not an RGB camera or no data is available.
    pub fn get_rgb_data(&self) -> Option<RGBImageData> {
        if self.camera_type != CameraType::RGB {
            return None;
        }

        self.get_image_data()
    }

    /// Get the last captured depth data.
    ///
    /// Returns None if the camera is not a depth camera or no data is available.
    pub fn get_depth_data(&self) -> Option<DepthImageData> {
        if self.camera_type != CameraType::Depth {
            return None;
        }

        self.get_image_data().map(DepthImageData::new)
    }

    /// Get the last captured semantic segmentation data.
    ///
    /// Returns None if the camera is not a semantic segmentation camera or no data is available.
    pub fn get_semantic_segmentation_data(&self) -> Option<SemanticSegmentationImageData> {
        if self.camera_type != CameraType::SemanticSegmentation {
            return None;
        }

        self.get_image_data()
            .map(SemanticSegmentationImageData::new)
    }

    /// Get the last captured instance segmentation data.
    ///
    /// Returns None if the camera is not an instance segmentation camera or no data is available.
    pub fn get_instance_segmentation_data(&self) -> Option<InstanceSegmentationImageData> {
        if self.camera_type != CameraType::InstanceSegmentation {
            return None;
        }

        self.get_image_data()
            .map(InstanceSegmentationImageData::new)
    }
}

impl std::fmt::Debug for CameraSensor {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("CameraSensor")
            .field("camera_type", &self.camera_type)
            .field("sensor_id", &self.sensor.id())
            .field("is_listening", &self.is_listening())
            .finish()
    }
}
