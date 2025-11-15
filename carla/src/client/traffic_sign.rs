use super::{Actor, ActorBase};
use crate::{geom::BoundingBox, road::SignId};
use carla_sys::carla_rust::client::{FfiActor, FfiTrafficSign};
use cxx::SharedPtr;
use derivative::Derivative;
use static_assertions::assert_impl_all;

/// Represents a traffic sign in the simulation.
///
/// Traffic signs provide information to drivers about road rules, warnings,
/// and directions. They are static actors in the world.
///
/// Corresponds to [`carla.TrafficSign`](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficSign) in the Python API.
///
/// # Examples
///
/// ```no_run
/// use carla::client::{ActorBase, Client};
///
/// let client = Client::default();
/// let world = client.world();
///
/// // Get all actors in the world
/// let actors = world.actors();
/// for actor in actors.iter() {
///     // Try to convert to traffic sign
///     if let Ok(sign) = carla::client::TrafficSign::try_from(actor.clone()) {
///         println!("Traffic sign at: {:?}", sign.location());
///         println!("Sign ID: {}", sign.sign_id());
///     }
/// }
/// ```
#[derive(Clone, Derivative)]
#[derivative(Debug)]
pub struct TrafficSign {
    #[derivative(Debug = "ignore")]
    inner: SharedPtr<FfiTrafficSign>,
}

impl TrafficSign {
    /// Returns the OpenDRIVE sign ID.
    ///
    /// See [carla.TrafficSign.get_opendrive_id](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficSign) in the Python API.
    pub fn sign_id(&self) -> SignId {
        self.inner.GetSignId().to_string()
    }

    /// Returns the trigger volume bounding box.
    ///
    /// See [carla.TrafficSign.get_trigger_volume](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficSign) in the Python API.
    ///
    /// The trigger volume is the area where the sign affects vehicles.
    pub fn trigger_volume(&self) -> BoundingBox {
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
