use crate::geom::TransformExt;
use carla_sys::carla_rust::sensor::FfiSensorData;
use cxx::SharedPtr;
use nalgebra::Isometry3;

use super::data::{CollisionEvent, Image, LaneInvasionEvent, ObstacleDetectionEvent};

pub trait SensorDataBase {
    fn cxx_sensor_data(&self) -> SharedPtr<FfiSensorData>;

    fn frame(&self) -> usize {
        self.cxx_sensor_data().GetFrame()
    }

    fn timestamp(&self) -> f64 {
        self.cxx_sensor_data().GetTimestamp()
    }

    fn sensor_transform(&self) -> Isometry3<f32> {
        self.cxx_sensor_data().GetSensorTransform().to_na()
    }
}

#[derive(Clone)]
#[repr(transparent)]
pub struct SensorData {
    inner: SharedPtr<FfiSensorData>,
}

impl SensorData {
    pub fn try_into_image(self) -> Result<Image, Self> {
        let ptr = self.inner.to_image();
        Image::from_cxx(ptr).ok_or(self)
    }

    pub fn try_into_collision_event(self) -> Result<CollisionEvent, Self> {
        let ptr = self.inner.to_collision_event();
        CollisionEvent::from_cxx(ptr).ok_or(self)
    }

    pub fn try_into_lane_invasion_event(self) -> Result<LaneInvasionEvent, Self> {
        let ptr = self.inner.to_lane_invasion_event();
        LaneInvasionEvent::from_cxx(ptr).ok_or(self)
    }

    pub fn try_into_obstacle_detection_event(self) -> Result<ObstacleDetectionEvent, Self> {
        let ptr = self.inner.to_obstacle_detection_event();
        ObstacleDetectionEvent::from_cxx(ptr).ok_or(self)
    }

    pub(crate) fn from_cxx(ptr: SharedPtr<FfiSensorData>) -> Self {
        Self { inner: ptr }
    }
}

impl SensorDataBase for SensorData {
    fn cxx_sensor_data(&self) -> SharedPtr<FfiSensorData> {
        self.inner.clone()
    }
}
