use crate::client::{Sensor, Vehicle};
use carla_sys::carla_rust::client::FfiActor;
use cxx::SharedPtr;
use derivative::Derivative;
use static_assertions::assert_impl_all;

use super::{ActorBase, ActorKind};

#[derive(Clone, Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct Actor {
    #[derivative(Debug = "ignore")]
    pub(crate) inner: SharedPtr<FfiActor>,
}

impl Actor {
    pub fn into_kinds(self) -> ActorKind {
        let me = self;
        let me = match Vehicle::try_from(me) {
            Ok(me) => return me.into(),
            Err(me) => me,
        };
        let me = match Sensor::try_from(me) {
            Ok(me) => return me.into(),
            Err(me) => me,
        };

        me.into()
    }

    pub(crate) fn from_cxx(ptr: SharedPtr<FfiActor>) -> Option<Self> {
        if ptr.is_null() {
            None
        } else {
            Some(Self { inner: ptr })
        }
    }
}

impl ActorBase for Actor {
    fn cxx_actor(&self) -> SharedPtr<FfiActor> {
        self.inner.clone()
    }
}

assert_impl_all!(Actor: Send, Sync);
