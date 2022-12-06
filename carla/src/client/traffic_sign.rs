use super::{Actor, ActorBase};
use crate::{geom::BoundingBox, road::SignId};
use carla_sys::carla_rust::client::{FfiActor, FfiTrafficSign};
use cxx::SharedPtr;
use derivative::Derivative;
use static_assertions::assert_impl_all;

#[derive(Clone, Derivative)]
#[derivative(Debug)]
pub struct TrafficSign {
    #[derivative(Debug = "ignore")]
    inner: SharedPtr<FfiTrafficSign>,
}

impl TrafficSign {
    pub fn sign_id(&self) -> SignId {
        self.inner.GetSignId().to_string()
    }

    pub fn trigger_volume(&self) -> BoundingBox<f32> {
        BoundingBox::from_native(self.inner.GetTriggerVolume())
    }

    fn from_cxx(ptr: SharedPtr<FfiTrafficSign>) -> Option<Self> {
        if ptr.is_null() {
            None
        } else {
            Some(Self { inner: ptr })
        }
    }
}

impl ActorBase for TrafficSign {
    fn cxx_actor(&self) -> SharedPtr<FfiActor> {
        self.inner.to_actor()
    }
}

impl TryFrom<Actor> for TrafficSign {
    type Error = Actor;

    fn try_from(value: Actor) -> Result<Self, Self::Error> {
        let ptr = value.inner.to_traffic_sign();
        Self::from_cxx(ptr).ok_or(value)
    }
}

assert_impl_all!(TrafficSign: Send, Sync);
