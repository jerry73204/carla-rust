//! Traffic sign actor implementation.

use crate::{
    actor::{Actor, ActorId},
    error::CarlaResult,
    geom::{FromCxx, ToCxx, Transform, Vector3D},
    traits::ActorT,
};
use carla_cxx::{TrafficSignType, TrafficSignWrapper};

/// Traffic sign actor.
#[derive(Debug)]
pub struct TrafficSign {
    /// Actor ID
    id: ActorId,
    /// Internal traffic sign wrapper for FFI calls
    inner: TrafficSignWrapper,
}

impl TrafficSign {
    /// Create a traffic sign from a carla-cxx TrafficSignWrapper and actor ID.
    pub(crate) fn from_cxx(
        traffic_sign_wrapper: TrafficSignWrapper,
        id: ActorId,
    ) -> CarlaResult<Self> {
        Ok(Self {
            id,
            inner: traffic_sign_wrapper,
        })
    }

    /// Create a traffic sign from an actor by casting.
    pub fn from_actor(actor: Actor) -> CarlaResult<Option<Self>> {
        let actor_ref = actor.get_inner_actor();
        let actor_id = actor.id();
        if let Some(traffic_sign_wrapper) = TrafficSignWrapper::from_actor(actor_ref) {
            Ok(Some(Self {
                id: actor_id,
                inner: traffic_sign_wrapper,
            }))
        } else {
            Ok(None)
        }
    }

    /// Get the traffic sign's actor ID.
    pub fn id(&self) -> ActorId {
        self.id
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

    /// Check if a location is within the sign's influence area.
    pub fn is_in_trigger_volume(&self, location: &crate::geom::Location) -> bool {
        let trigger_volume = self.trigger_volume();
        trigger_volume.contains(location)
    }
}

impl ActorT for TrafficSign {
    fn id(&self) -> ActorId {
        self.id
    }
    fn type_id(&self) -> String {
        self.inner.get_type_id()
    }
    fn transform(&self) -> Transform {
        let cxx_transform = self.inner.get_transform();
        Transform::from(cxx_transform)
    }
    fn set_transform(&self, transform: &Transform) -> CarlaResult<()> {
        let cxx_transform = transform.to_cxx();
        self.inner.set_transform(&cxx_transform);
        Ok(())
    }
    fn velocity(&self) -> Vector3D {
        let vel = self.inner.get_velocity();
        Vector3D::new(vel.x as f32, vel.y as f32, vel.z as f32)
    }
    fn angular_velocity(&self) -> Vector3D {
        let vel = self.inner.get_angular_velocity();
        Vector3D::new(vel.x as f32, vel.y as f32, vel.z as f32)
    }
    fn acceleration(&self) -> Vector3D {
        let acc = self.inner.get_acceleration();
        Vector3D::new(acc.x as f32, acc.y as f32, acc.z as f32)
    }
    fn is_alive(&self) -> bool {
        self.inner.is_alive()
    }
    fn set_simulate_physics(&self, enabled: bool) -> CarlaResult<()> {
        self.inner.set_simulate_physics(enabled);
        Ok(())
    }
    fn add_impulse(&self, impulse: &Vector3D) -> CarlaResult<()> {
        let cxx_impulse = carla_cxx::SimpleVector3D {
            x: impulse.x as f64,
            y: impulse.y as f64,
            z: impulse.z as f64,
        };
        self.inner.add_impulse(&cxx_impulse);
        Ok(())
    }
    fn add_force(&self, force: &Vector3D) -> CarlaResult<()> {
        let cxx_force = carla_cxx::SimpleVector3D {
            x: force.x as f64,
            y: force.y as f64,
            z: force.z as f64,
        };
        self.inner.add_force(&cxx_force);
        Ok(())
    }
    fn add_torque(&self, torque: &Vector3D) -> CarlaResult<()> {
        let cxx_torque = carla_cxx::SimpleVector3D {
            x: torque.x as f64,
            y: torque.y as f64,
            z: torque.z as f64,
        };
        self.inner.add_torque(&cxx_torque);
        Ok(())
    }

    fn bounding_box(&self) -> crate::geom::BoundingBox {
        // TrafficSign doesn't have a direct GetBoundingBox method, need to cast to Actor
        let actor_ptr =
            carla_cxx::ffi::TrafficSign_CastToActor(self.inner.get_inner_traffic_sign());
        if actor_ptr.is_null() {
            panic!("Internal error: Failed to cast TrafficSign to Actor");
        }
        let simple_bbox = carla_cxx::ffi::Actor_GetBoundingBox(&actor_ptr);
        crate::geom::BoundingBox::from_cxx(simple_bbox)
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
