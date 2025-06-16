//! Camera sensor implementation.

use crate::{
    actor::{ActorId, Sensor},
    error::CarlaResult,
    geom::{Transform, Vector3D},
    sensor_data::{
        DepthImageData, ImageData, InstanceSegmentationImageData, RGBImageData,
        SemanticSegmentationImageData,
    },
    traits::{ActorT, SensorT},
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

/// Camera sensor for image data.
#[derive(Debug)]
pub struct Camera(Sensor);

impl Camera {
    /// Try to create a Camera from a generic Sensor.
    /// Returns None if the sensor is not a camera.
    pub fn from_sensor(sensor: Sensor) -> Option<Self> {
        if sensor.inner().is_camera() {
            Some(Camera(sensor))
        } else {
            None
        }
    }

    /// Convert back to generic Sensor.
    pub fn into_sensor(self) -> Sensor {
        self.0
    }

    /// Get reference to the underlying sensor.
    pub fn as_sensor(&self) -> &Sensor {
        &self.0
    }

    /// Get the camera type (RGB, Depth, Semantic, etc.)
    pub fn get_camera_type(&self) -> CameraType {
        CameraType::from(self.0.inner().get_camera_type())
    }

    /// Get the last image data from the camera.
    pub fn get_last_image_data(&self) -> Option<ImageData> {
        let cxx_data = self.0.inner().get_last_image_data();
        if cxx_data.is_empty() {
            None
        } else {
            Some(ImageData::from_cxx(cxx_data))
        }
    }

    /// Get image data into a provided buffer.
    /// Returns true if data was copied successfully.
    pub fn get_image_data_buffer(&self, buffer: &mut [u8]) -> bool {
        self.0.inner().get_image_data_buffer(buffer)
    }

    /// Get the last captured RGB image data.
    ///
    /// Returns None if the camera is not an RGB camera or no data is available.
    pub fn get_rgb_data(&self) -> Option<RGBImageData> {
        if self.get_camera_type() != CameraType::RGB {
            return None;
        }

        self.get_last_image_data()
    }

    /// Get the last captured depth data.
    ///
    /// Returns None if the camera is not a depth camera or no data is available.
    pub fn get_depth_data(&self) -> Option<DepthImageData> {
        if self.get_camera_type() != CameraType::Depth {
            return None;
        }

        self.get_last_image_data().map(DepthImageData::new)
    }

    /// Get the last captured semantic segmentation data.
    ///
    /// Returns None if the camera is not a semantic segmentation camera or no data is available.
    pub fn get_semantic_segmentation_data(&self) -> Option<SemanticSegmentationImageData> {
        if self.get_camera_type() != CameraType::SemanticSegmentation {
            return None;
        }

        self.get_last_image_data()
            .map(SemanticSegmentationImageData::new)
    }

    /// Get the last captured instance segmentation data.
    ///
    /// Returns None if the camera is not an instance segmentation camera or no data is available.
    pub fn get_instance_segmentation_data(&self) -> Option<InstanceSegmentationImageData> {
        if self.get_camera_type() != CameraType::InstanceSegmentation {
            return None;
        }

        self.get_last_image_data()
            .map(InstanceSegmentationImageData::new)
    }
}

impl ActorT for Camera {
    fn id(&self) -> ActorId {
        self.0.id()
    }

    fn type_id(&self) -> String {
        self.0.type_id()
    }

    fn transform(&self) -> Transform {
        self.0.transform()
    }

    fn set_transform(&self, transform: &Transform) -> CarlaResult<()> {
        self.0.set_transform(transform)
    }

    fn velocity(&self) -> Vector3D {
        self.0.velocity()
    }

    fn angular_velocity(&self) -> Vector3D {
        self.0.angular_velocity()
    }

    fn acceleration(&self) -> Vector3D {
        self.0.acceleration()
    }

    fn is_alive(&self) -> bool {
        self.0.is_alive()
    }

    fn set_simulate_physics(&self, enabled: bool) -> CarlaResult<()> {
        self.0.set_simulate_physics(enabled)
    }

    fn add_impulse(&self, impulse: &Vector3D) -> CarlaResult<()> {
        self.0.add_impulse(impulse)
    }

    fn add_force(&self, force: &Vector3D) -> CarlaResult<()> {
        self.0.add_force(force)
    }

    fn add_torque(&self, torque: &Vector3D) -> CarlaResult<()> {
        self.0.add_torque(torque)
    }

    fn bounding_box(&self) -> crate::geom::BoundingBox {
        self.0.bounding_box()
    }
}

impl SensorT for Camera {
    fn start_listening(&self) {
        self.0.start_listening()
    }

    fn stop_listening(&self) {
        self.0.stop_listening()
    }

    fn is_listening(&self) -> bool {
        self.0.is_listening()
    }

    fn has_new_data(&self) -> bool {
        self.0.has_new_data()
    }

    fn get_attribute(&self, name: &str) -> Option<String> {
        self.0.get_attribute(name)
    }

    fn enable_recording(&self, filename: &str) -> CarlaResult<()> {
        self.0.enable_recording(filename)
    }

    fn disable_recording(&self) -> CarlaResult<()> {
        self.0.disable_recording()
    }
}
