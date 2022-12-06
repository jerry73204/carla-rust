use crate::{
    geom::{Location, LocationExt, Transform, TransformExt, Vector3D, Vector3DExt},
    rpc::ActorId,
};
use carla_sys::carla_rust::client::FfiActor;
use cxx::SharedPtr;
use derivative::Derivative;
use nalgebra::{Isometry3, Translation3, Vector3};
use static_assertions::assert_impl_all;

use super::World;

pub trait ActorBase {
    fn cxx_actor(&self) -> SharedPtr<FfiActor>;

    fn id(&self) -> ActorId {
        self.cxx_actor().GetId()
    }

    fn type_id(&self) -> String {
        self.cxx_actor().GetTypeId().to_string()
    }

    fn display_id(&self) -> String {
        self.cxx_actor().GetDisplayId().to_string()
    }

    fn parent_id(&self) -> ActorId {
        self.cxx_actor().GetParentId()
    }

    fn semantic_tags(&self) -> Vec<u8> {
        self.cxx_actor().GetSemanticTags().iter().cloned().collect()
    }

    fn parent(&self) -> Option<Actor> {
        Actor::from_cxx(self.cxx_actor().GetParent())
    }

    fn world(&self) -> World {
        World::from_cxx(self.cxx_actor().GetWorld()).unwrap()
    }

    fn location(&self) -> Translation3<f32> {
        self.cxx_actor().GetLocation().to_na()
    }

    fn transform(&self) -> Isometry3<f32> {
        self.cxx_actor().GetTransform().to_na()
    }

    fn velocity(&self) -> Vector3<f32> {
        self.cxx_actor().GetVelocity().to_na()
    }

    fn acceleration(&self) -> Vector3<f32> {
        self.cxx_actor().GetAcceleration().to_na()
    }

    fn angular_velocity(&self) -> Vector3<f32> {
        self.cxx_actor().GetAngularVelocity().to_na()
    }

    fn set_location(&self, location: &Translation3<f32>) {
        self.cxx_actor().SetLocation(&Location::from_na(location))
    }

    fn set_transform(&self, transform: &Isometry3<f32>) {
        self.cxx_actor()
            .SetTransform(&Transform::from_na(transform))
    }

    fn set_target_velocity(&self, vector: &Vector3<f32>) {
        self.cxx_actor()
            .SetTargetVelocity(&Vector3D::from_na(vector))
    }

    fn set_target_angular_velocity(&self, vector: &Vector3<f32>) {
        self.cxx_actor()
            .SetTargetAngularVelocity(&Vector3D::from_na(vector))
    }

    fn enable_constant_velocity(&self, vector: &Vector3<f32>) {
        self.cxx_actor()
            .EnableConstantVelocity(&Vector3D::from_na(vector))
    }

    fn disable_constant_velocity(&self) {
        self.cxx_actor().DisableConstantVelocity()
    }

    fn add_impulse(&self, vector: &Vector3<f32>) {
        self.cxx_actor().AddImpulse1(&Vector3D::from_na(vector))
    }

    fn add_impulse_at(&self, vector: &Vector3<f32>, location: &Vector3<f32>) {
        self.cxx_actor()
            .AddImpulse2(&Vector3D::from_na(vector), &Vector3D::from_na(location))
    }

    fn add_force(&self, vector: &Vector3<f32>) {
        self.cxx_actor().AddForce1(&Vector3D::from_na(vector))
    }

    fn add_force_at(&self, vector: &Vector3<f32>, location: &Vector3<f32>) {
        self.cxx_actor()
            .AddForce2(&Vector3D::from_na(vector), &Vector3D::from_na(location))
    }

    fn add_angular_impulse(&self, vector: &Vector3<f32>) {
        self.cxx_actor()
            .AddAngularImpulse(&Vector3D::from_na(vector))
    }

    fn add_torque(&self, vector: &Vector3<f32>) {
        self.cxx_actor().AddTorque(&Vector3D::from_na(vector))
    }

    fn set_simulate_physics(&self, enabled: bool) {
        self.cxx_actor().SetSimulatePhysics(enabled)
    }

    fn set_enable_gravity(&self, enabled: bool) {
        self.cxx_actor().SetEnableGravity(enabled)
    }

    fn is_alive(&self) -> bool {
        self.cxx_actor().IsAlive()
    }

    fn is_dormant(&self) -> bool {
        self.cxx_actor().IsDormant()
    }

    fn is_active(&self) -> bool {
        self.cxx_actor().IsActive()
    }
}

#[derive(Clone, Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct Actor {
    #[derivative(Debug = "ignore")]
    pub(crate) inner: SharedPtr<FfiActor>,
}

impl Actor {
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
