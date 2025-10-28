// SAFETY: This module uses unwrap_unchecked() for performance on methods guaranteed
// to never return null. See UNWRAP_REPLACEMENTS.md for detailed C++ code audit.

use super::{Actor, ActorAttributeValueList, World};
use crate::{
    geom::{Location, LocationExt, Transform, TransformExt, Vector3D, Vector3DExt},
    rpc::ActorId,
};
use autocxx::WithinUniquePtr;
use carla_sys::carla_rust::client::FfiActor;
use cxx::SharedPtr;
use nalgebra::{Isometry3, Translation3, Vector3};

/// This trait defines a basic actor in the simulation.
///
/// `ActorBase` is implemented by all actor types ([`Actor`], [`super::Vehicle`],
/// [`super::Sensor`], `TrafficLight`, `TrafficSign`) and provides common functionality:
/// - Position, velocity, and physics queries
/// - Transform manipulation
/// - Physics simulation control (forces, impulses, gravity)
/// - Metadata (ID, type, parent, attributes)
///
/// # Examples
///
/// ```no_run
/// use carla::client::{ActorBase, Client};
///
/// let client = Client::default();
/// let mut world = client.world();
/// # let bp_lib = world.blueprint_library();
/// # let vehicle_bp = bp_lib.filter("vehicle.*").get(0).unwrap();
/// # let spawn_points = world.map().recommended_spawn_points();
/// # let actor = world.spawn_actor(&vehicle_bp, &spawn_points.get(0).unwrap()).unwrap();
///
/// // All actors have these methods
/// println!("Actor ID: {}", actor.id());
/// println!("Type: {}", actor.type_id());
/// println!("Location: {:?}", actor.location());
/// println!("Velocity: {:?}", actor.velocity());
/// ```
pub trait ActorBase: Clone {
    /// Returns the underlying FFI actor pointer (internal use).
    fn cxx_actor(&self) -> SharedPtr<FfiActor>;

    /// Converts this actor into a generic [`Actor`].
    fn into_actor(self) -> Actor {
        // SAFETY: cxx_actor() returns a valid SharedPtr for all actor implementations
        unsafe { Actor::from_cxx(self.cxx_actor()).unwrap_unchecked() }
    }

    /// Returns the unique actor ID.
    fn id(&self) -> ActorId {
        self.cxx_actor().GetId()
    }

    /// Returns the actor blueprint type ID (e.g., "vehicle.tesla.model3").
    fn type_id(&self) -> String {
        self.cxx_actor().GetTypeId().to_string()
    }

    /// Returns a human-readable display ID.
    fn display_id(&self) -> String {
        self.cxx_actor().GetDisplayId().to_string()
    }

    /// Returns the ID of this actor's parent, or 0 if no parent.
    fn parent_id(&self) -> ActorId {
        self.cxx_actor().GetParentId()
    }

    /// Returns semantic segmentation tags for this actor.
    fn semantic_tags(&self) -> Vec<u8> {
        self.cxx_actor().GetSemanticTags().iter().cloned().collect()
    }

    /// Returns the parent actor, if attached to one.
    fn parent(&self) -> Option<Actor> {
        Actor::from_cxx(self.cxx_actor().GetParent())
    }

    /// Returns the actor's blueprint attributes (color, role_name, etc.).
    fn attributes(&self) -> ActorAttributeValueList<'_> {
        let ptr = self.cxx_actor().GetAttributes().within_unique_ptr();
        unsafe { ActorAttributeValueList::from_cxx(ptr).unwrap_unchecked() }
    }

    /// Returns the world this actor belongs to.
    fn world(&self) -> World {
        unsafe { World::from_cxx(self.cxx_actor().GetWorld()).unwrap_unchecked() }
    }

    /// Returns the actor's current location (position only).
    fn location(&self) -> Translation3<f32> {
        self.cxx_actor().GetLocation().to_na_translation()
    }

    /// Returns the actor's current transform (position and rotation).
    fn transform(&self) -> Isometry3<f32> {
        self.cxx_actor().GetTransform().to_na()
    }

    /// Returns the actor's velocity vector in m/s.
    fn velocity(&self) -> Vector3<f32> {
        self.cxx_actor().GetVelocity().to_na()
    }

    /// Returns the actor's acceleration vector in m/s².
    fn acceleration(&self) -> Vector3<f32> {
        self.cxx_actor().GetAcceleration().to_na()
    }

    /// Returns the actor's angular velocity in radians/s.
    fn angular_velocity(&self) -> Vector3<f32> {
        self.cxx_actor().GetAngularVelocity().to_na()
    }

    /// Teleports the actor to a new location.
    fn set_location(&self, location: &Translation3<f32>) {
        self.cxx_actor()
            .SetLocation(&Location::from_na_translation(location))
    }

    /// Teleports the actor to a new transform (position and rotation).
    fn set_transform(&self, transform: &Isometry3<f32>) {
        self.cxx_actor()
            .SetTransform(&Transform::from_na(transform))
    }

    /// Sets the target velocity for physics simulation (m/s).
    fn set_target_velocity(&self, vector: &Vector3<f32>) {
        self.cxx_actor()
            .SetTargetVelocity(&Vector3D::from_na(vector))
    }

    /// Sets the target angular velocity for physics simulation (rad/s).
    fn set_target_angular_velocity(&self, vector: &Vector3<f32>) {
        self.cxx_actor()
            .SetTargetAngularVelocity(&Vector3D::from_na(vector))
    }

    /// Enables constant velocity mode (actor moves at fixed velocity regardless of physics).
    fn enable_constant_velocity(&self, vector: &Vector3<f32>) {
        self.cxx_actor()
            .EnableConstantVelocity(&Vector3D::from_na(vector))
    }

    /// Disables constant velocity mode.
    fn disable_constant_velocity(&self) {
        self.cxx_actor().DisableConstantVelocity()
    }

    /// Applies an impulse (instantaneous velocity change) to the actor's center of mass.
    fn add_impulse(&self, vector: &Vector3<f32>) {
        self.cxx_actor().AddImpulse1(&Vector3D::from_na(vector))
    }

    /// Applies an impulse at a specific location on the actor.
    fn add_impulse_at(&self, vector: &Vector3<f32>, location: &Vector3<f32>) {
        self.cxx_actor()
            .AddImpulse2(&Vector3D::from_na(vector), &Vector3D::from_na(location))
    }

    /// Applies a continuous force to the actor's center of mass.
    fn add_force(&self, vector: &Vector3<f32>) {
        self.cxx_actor().AddForce1(&Vector3D::from_na(vector))
    }

    /// Applies a continuous force at a specific location on the actor.
    fn add_force_at(&self, vector: &Vector3<f32>, location: &Vector3<f32>) {
        self.cxx_actor()
            .AddForce2(&Vector3D::from_na(vector), &Vector3D::from_na(location))
    }

    /// Applies an angular impulse (instantaneous rotation change).
    fn add_angular_impulse(&self, vector: &Vector3<f32>) {
        self.cxx_actor()
            .AddAngularImpulse(&Vector3D::from_na(vector))
    }

    /// Applies a continuous torque (rotational force).
    fn add_torque(&self, vector: &Vector3<f32>) {
        self.cxx_actor().AddTorque(&Vector3D::from_na(vector))
    }

    /// Enables or disables physics simulation for this actor.
    fn set_simulate_physics(&self, enabled: bool) {
        self.cxx_actor().SetSimulatePhysics(enabled)
    }

    /// Enables or disables gravity for this actor.
    fn set_enable_gravity(&self, enabled: bool) {
        self.cxx_actor().SetEnableGravity(enabled)
    }

    /// Returns whether the actor still exists in the simulation.
    fn is_alive(&self) -> bool {
        self.cxx_actor().IsAlive()
    }

    /// Returns whether the actor is currently dormant (inactive/sleeping).
    fn is_dormant(&self) -> bool {
        self.cxx_actor().IsDormant()
    }

    /// Returns whether the actor is currently active.
    fn is_active(&self) -> bool {
        self.cxx_actor().IsActive()
    }

    /// Destroys this actor and removes it from the simulation.
    ///
    /// Returns `true` if the actor was successfully destroyed, `false` otherwise.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use carla::client::{ActorBase, Client};
    ///
    /// let client = Client::default();
    /// let mut world = client.world();
    /// # let bp_lib = world.blueprint_library();
    /// # let vehicle_bp = bp_lib.filter("vehicle.*").get(0).unwrap();
    /// # let spawn_points = world.map().recommended_spawn_points();
    /// # let actor = world.spawn_actor(&vehicle_bp, &spawn_points.get(0).unwrap()).unwrap();
    ///
    /// // Destroy the actor when done
    /// let success = actor.destroy();
    /// println!("Actor destroyed: {}", success);
    /// ```
    fn destroy(&self) -> bool {
        self.cxx_actor().Destroy()
    }
}
