use super::TrafficLight;
use carla_sys::carla_rust::client::FfiTrafficLightList;
use cxx::UniquePtr;
use derivative::Derivative;

/// A list of traffic lights.
#[derive(Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct TrafficLightList {
    #[derivative(Debug = "ignore")]
    inner: UniquePtr<FfiTrafficLightList>,
}

impl TrafficLightList {
    pub fn len(&self) -> usize {
        self.inner.len()
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    pub fn get(&self, index: usize) -> Option<TrafficLight> {
        if index >= self.len() {
            return None;
        }
        let ptr = self.inner.get(index);
        Some(TrafficLight::from_cxx(ptr).unwrap())
    }

    pub fn iter(&self) -> impl Iterator<Item = TrafficLight> + '_ {
        (0..self.len()).map(|index| self.get(index).unwrap())
    }

    pub(crate) fn from_cxx(ptr: UniquePtr<FfiTrafficLightList>) -> Option<Self> {
        if ptr.is_null() {
            None
        } else {
            Some(Self { inner: ptr })
        }
    }
}
