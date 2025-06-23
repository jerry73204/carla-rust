//! Traffic sign actor implementation.

use crate::{
    actor::{Actor, ActorFfi},
    error::CarlaResult,
    geom::FromCxx,
};
use carla_sys::{TrafficSignType, TrafficSignWrapper};

/// Traffic sign actor.
#[derive(Debug)]
pub struct TrafficSign {
    /// Internal traffic sign wrapper for FFI calls
    inner: TrafficSignWrapper,
}

impl TrafficSign {
    /// Create a traffic sign from a carla-sys TrafficSignWrapper and actor ID.
    pub(crate) fn from_cxx(traffic_sign_wrapper: TrafficSignWrapper) -> CarlaResult<Self> {
        Ok(Self {
            inner: traffic_sign_wrapper,
        })
    }

    /// Create a traffic sign from an actor by casting.
    pub fn from_actor(actor: Actor) -> Result<Self, Actor> {
        let actor_ref = actor.inner_actor();
        if let Some(traffic_sign_wrapper) = TrafficSignWrapper::from_actor(actor_ref) {
            Ok(Self {
                inner: traffic_sign_wrapper,
            })
        } else {
            Err(actor)
        }
    }

    /// Get the traffic sign type by parsing the sign ID.
    pub fn sign_type(&self) -> TrafficSignType {
        let sign_id = self.inner.get_sign_id();
        TrafficSignType::from_sign_id(&sign_id)
    }

    /// Get the trigger volume for this traffic sign.
    pub fn trigger_volume(&self) -> crate::geom::BoundingBox {
        let simple_bbox = self.inner.get_trigger_volume();
        crate::geom::BoundingBox::from_cxx(simple_bbox)
    }

    /// Convert this traffic sign to a generic Actor.
    ///
    /// This creates a new Actor instance that represents the same traffic sign.
    /// This is useful when you need to work with generic actor functionality.
    pub fn into_actor(self) -> Actor {
        let actor_wrapper = self.inner.as_actor_wrapper();
        Actor::from_cxx(actor_wrapper)
    }

    /// Check if a location is within the sign's influence area.
    pub fn is_in_trigger_volume(&self, location: &crate::geom::Location) -> bool {
        let trigger_volume = self.trigger_volume();
        trigger_volume.contains(location)
    }
}

impl ActorFfi for TrafficSign {
    fn as_actor_ffi(&self) -> carla_sys::ActorWrapper {
        self.inner.as_actor_wrapper()
    }
}

impl Drop for TrafficSign {
    fn drop(&mut self) {
        // Only destroy if the traffic sign is still alive
        if self.inner.is_alive() {
            let _ = self.inner.destroy();
        }
    }
}

#[cfg(test)]
mod tests {
    /// Test traffic light type conversions
    #[test]
    fn test_traffic_light_state_conversions() {
        use crate::actor::TrafficLightState;
        use carla_sys::TrafficLightState as CxxState;

        // Test from_cxx
        assert_eq!(
            TrafficLightState::from_cxx(CxxState::Red),
            TrafficLightState::Red
        );
        assert_eq!(
            TrafficLightState::from_cxx(CxxState::Yellow),
            TrafficLightState::Yellow
        );
        assert_eq!(
            TrafficLightState::from_cxx(CxxState::Green),
            TrafficLightState::Green
        );
        assert_eq!(
            TrafficLightState::from_cxx(CxxState::Off),
            TrafficLightState::Off
        );
        assert_eq!(
            TrafficLightState::from_cxx(CxxState::Unknown),
            TrafficLightState::Unknown
        );

        // Test into_cxx
        assert_eq!(TrafficLightState::Red.into_cxx(), CxxState::Red);
        assert_eq!(TrafficLightState::Yellow.into_cxx(), CxxState::Yellow);
        assert_eq!(TrafficLightState::Green.into_cxx(), CxxState::Green);
        assert_eq!(TrafficLightState::Off.into_cxx(), CxxState::Off);
        assert_eq!(TrafficLightState::Unknown.into_cxx(), CxxState::Unknown);
    }
}
