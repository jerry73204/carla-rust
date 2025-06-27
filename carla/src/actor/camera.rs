//! Camera sensor implementation.

use crate::{
    actor::{Sensor, SensorFfi},
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

// Implement ActorExt trait using the macro
crate::impl_sensor_actor_ext!(Camera);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_camera_type_enum() {
        let camera_types = [
            CameraType::RGB,
            CameraType::Depth,
            CameraType::SemanticSegmentation,
            CameraType::InstanceSegmentation,
            CameraType::OpticalFlow,
            CameraType::DVS,
        ];

        assert_eq!(camera_types.len(), 6);

        // Test Debug output
        assert_eq!(format!("{:?}", CameraType::RGB), "RGB");
        assert_eq!(
            format!("{:?}", CameraType::SemanticSegmentation),
            "SemanticSegmentation"
        );
        assert_eq!(format!("{:?}", CameraType::DVS), "DVS");
    }

    #[test]
    fn test_camera_type_equality() {
        assert_eq!(CameraType::RGB, CameraType::RGB);
        assert_ne!(CameraType::RGB, CameraType::Depth);
        assert_ne!(
            CameraType::SemanticSegmentation,
            CameraType::InstanceSegmentation
        );
    }

    #[test]
    fn test_camera_type_clone() {
        let camera_type = CameraType::RGB;
        let cloned_type = camera_type;
        assert_eq!(camera_type, cloned_type);
    }

    #[test]
    fn test_camera_type_copy() {
        let camera_type = CameraType::Depth;
        let copied_type = camera_type; // Copy trait
        assert_eq!(camera_type, copied_type);
        // Both variables should still be usable
        assert_eq!(camera_type, CameraType::Depth);
        assert_eq!(copied_type, CameraType::Depth);
    }

    #[test]
    fn test_camera_type_pattern_matching() {
        fn get_image_format(camera_type: CameraType) -> &'static str {
            match camera_type {
                CameraType::RGB => "BGRA",
                CameraType::Depth => "F32",
                CameraType::SemanticSegmentation => "R8G8B8A8",
                CameraType::InstanceSegmentation => "R8G8B8A8",
                CameraType::OpticalFlow => "R32G32_SFLOAT",
                CameraType::DVS => "Events",
            }
        }

        assert_eq!(get_image_format(CameraType::RGB), "BGRA");
        assert_eq!(get_image_format(CameraType::Depth), "F32");
        assert_eq!(
            get_image_format(CameraType::SemanticSegmentation),
            "R8G8B8A8"
        );
        assert_eq!(get_image_format(CameraType::OpticalFlow), "R32G32_SFLOAT");
        assert_eq!(get_image_format(CameraType::DVS), "Events");
    }

    #[test]
    fn test_camera_type_is_segmentation() {
        fn is_segmentation_camera(camera_type: CameraType) -> bool {
            matches!(
                camera_type,
                CameraType::SemanticSegmentation | CameraType::InstanceSegmentation
            )
        }

        assert!(is_segmentation_camera(CameraType::SemanticSegmentation));
        assert!(is_segmentation_camera(CameraType::InstanceSegmentation));
        assert!(!is_segmentation_camera(CameraType::RGB));
        assert!(!is_segmentation_camera(CameraType::Depth));
        assert!(!is_segmentation_camera(CameraType::OpticalFlow));
        assert!(!is_segmentation_camera(CameraType::DVS));
    }

    #[test]
    fn test_camera_type_data_format() {
        fn get_expected_channels(camera_type: CameraType) -> u32 {
            match camera_type {
                CameraType::RGB => 4,                  // BGRA
                CameraType::Depth => 1,                // Single depth value
                CameraType::SemanticSegmentation => 4, // RGBA
                CameraType::InstanceSegmentation => 4, // RGBA
                CameraType::OpticalFlow => 2,          // X and Y flow
                CameraType::DVS => 0,                  // Event-based, no fixed channels
            }
        }

        assert_eq!(get_expected_channels(CameraType::RGB), 4);
        assert_eq!(get_expected_channels(CameraType::Depth), 1);
        assert_eq!(get_expected_channels(CameraType::SemanticSegmentation), 4);
        assert_eq!(get_expected_channels(CameraType::InstanceSegmentation), 4);
        assert_eq!(get_expected_channels(CameraType::OpticalFlow), 2);
        assert_eq!(get_expected_channels(CameraType::DVS), 0);
    }

    #[test]
    fn test_camera_type_typical_usage() {
        fn is_color_camera(camera_type: CameraType) -> bool {
            camera_type == CameraType::RGB
        }

        fn is_depth_camera(camera_type: CameraType) -> bool {
            camera_type == CameraType::Depth
        }

        fn is_computer_vision_camera(camera_type: CameraType) -> bool {
            matches!(
                camera_type,
                CameraType::SemanticSegmentation
                    | CameraType::InstanceSegmentation
                    | CameraType::OpticalFlow
            )
        }

        assert!(is_color_camera(CameraType::RGB));
        assert!(!is_color_camera(CameraType::Depth));

        assert!(is_depth_camera(CameraType::Depth));
        assert!(!is_depth_camera(CameraType::RGB));

        assert!(is_computer_vision_camera(CameraType::SemanticSegmentation));
        assert!(is_computer_vision_camera(CameraType::InstanceSegmentation));
        assert!(is_computer_vision_camera(CameraType::OpticalFlow));
        assert!(!is_computer_vision_camera(CameraType::RGB));
        assert!(!is_computer_vision_camera(CameraType::DVS));
    }

    #[test]
    fn test_camera_type_display_names() {
        fn get_display_name(camera_type: CameraType) -> &'static str {
            match camera_type {
                CameraType::RGB => "RGB Camera",
                CameraType::Depth => "Depth Camera",
                CameraType::SemanticSegmentation => "Semantic Segmentation Camera",
                CameraType::InstanceSegmentation => "Instance Segmentation Camera",
                CameraType::OpticalFlow => "Optical Flow Camera",
                CameraType::DVS => "Dynamic Vision Sensor",
            }
        }

        assert_eq!(get_display_name(CameraType::RGB), "RGB Camera");
        assert_eq!(get_display_name(CameraType::Depth), "Depth Camera");
        assert_eq!(
            get_display_name(CameraType::SemanticSegmentation),
            "Semantic Segmentation Camera"
        );
        assert_eq!(get_display_name(CameraType::DVS), "Dynamic Vision Sensor");
    }

    // Note: Testing actual camera functionality would require FFI mocks
    // These tests focus on the enum behavior and type safety
}
