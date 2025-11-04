// SAFETY: This module uses unwrap_unchecked() for performance on methods guaranteed
// to never return null. See UNWRAP_REPLACEMENTS.md for detailed C++ code audit.

use super::{Actor, ActorAttributeValueList, World};
#[cfg(carla_version_0916)]
use crate::geom::BoundingBox;
use crate::{
    geom::{Location, Transform, Vector3D},
    rpc::ActorId,
};
use autocxx::WithinUniquePtr;
use carla_sys::carla_rust::{client::FfiActor, geom::FfiVector3D};
use cxx::SharedPtr;

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
    fn location(&self) -> Location {
        Location::from_ffi(self.cxx_actor().GetLocation())
    }

    /// Returns the actor's current transform (position and rotation).
    fn transform(&self) -> Transform {
        Transform::from_ffi(self.cxx_actor().GetTransform())
    }

    /// Returns the actor's velocity vector in m/s.
    fn velocity(&self) -> Vector3D {
        // SAFETY: carla::geom::Vector3D and FfiVector3D have identical memory layout
        unsafe {
            let cpp_vec = self.cxx_actor().GetVelocity();
            Vector3D::from_ffi(std::mem::transmute::<
                carla_sys::carla::geom::Vector3D,
                FfiVector3D,
            >(cpp_vec))
        }
    }

    /// Returns the actor's acceleration vector in m/s².
    fn acceleration(&self) -> Vector3D {
        // SAFETY: carla::geom::Vector3D and FfiVector3D have identical memory layout
        unsafe {
            let cpp_vec = self.cxx_actor().GetAcceleration();
            Vector3D::from_ffi(std::mem::transmute::<
                carla_sys::carla::geom::Vector3D,
                FfiVector3D,
            >(cpp_vec))
        }
    }

    /// Returns the actor's angular velocity in radians/s.
    fn angular_velocity(&self) -> Vector3D {
        // SAFETY: carla::geom::Vector3D and FfiVector3D have identical memory layout
        unsafe {
            let cpp_vec = self.cxx_actor().GetAngularVelocity();
            Vector3D::from_ffi(std::mem::transmute::<
                carla_sys::carla::geom::Vector3D,
                FfiVector3D,
            >(cpp_vec))
        }
    }

    /// Teleports the actor to a new location.
    fn set_location(&self, location: &Location) {
        self.cxx_actor().SetLocation(location.as_ffi())
    }

    /// Teleports the actor to a new transform (position and rotation).
    fn set_transform(&self, transform: &Transform) {
        self.cxx_actor().SetTransform(transform.as_ffi())
    }

    /// Sets the target velocity for physics simulation (m/s).
    fn set_target_velocity(&self, vector: &Vector3D) {
        // SAFETY: FfiVector3D and carla::geom::Vector3D have identical memory layout
        unsafe {
            let cpp_vec = &*(vector.as_ffi() as *const FfiVector3D
                as *const carla_sys::carla::geom::Vector3D);
            self.cxx_actor().SetTargetVelocity(cpp_vec)
        }
    }

    /// Sets the target angular velocity for physics simulation (rad/s).
    fn set_target_angular_velocity(&self, vector: &Vector3D) {
        // SAFETY: FfiVector3D and carla::geom::Vector3D have identical memory layout
        unsafe {
            let cpp_vec = &*(vector.as_ffi() as *const FfiVector3D
                as *const carla_sys::carla::geom::Vector3D);
            self.cxx_actor().SetTargetAngularVelocity(cpp_vec)
        }
    }

    /// Enables constant velocity mode (actor moves at fixed velocity regardless of physics).
    fn enable_constant_velocity(&self, vector: &Vector3D) {
        // SAFETY: FfiVector3D and carla::geom::Vector3D have identical memory layout
        unsafe {
            let cpp_vec = &*(vector.as_ffi() as *const FfiVector3D
                as *const carla_sys::carla::geom::Vector3D);
            self.cxx_actor().EnableConstantVelocity(cpp_vec)
        }
    }

    /// Disables constant velocity mode.
    fn disable_constant_velocity(&self) {
        self.cxx_actor().DisableConstantVelocity()
    }

    /// Applies an impulse (instantaneous velocity change) to the actor's center of mass.
    fn add_impulse(&self, vector: &Vector3D) {
        // SAFETY: FfiVector3D and carla::geom::Vector3D have identical memory layout
        unsafe {
            let cpp_vec = &*(vector.as_ffi() as *const FfiVector3D
                as *const carla_sys::carla::geom::Vector3D);
            self.cxx_actor().AddImpulse1(cpp_vec)
        }
    }

    /// Applies an impulse at a specific location on the actor.
    fn add_impulse_at(&self, vector: &Vector3D, location: &Vector3D) {
        // SAFETY: FfiVector3D and carla::geom::Vector3D have identical memory layout
        unsafe {
            let cpp_vec = &*(vector.as_ffi() as *const FfiVector3D
                as *const carla_sys::carla::geom::Vector3D);
            let cpp_loc = &*(location.as_ffi() as *const FfiVector3D
                as *const carla_sys::carla::geom::Vector3D);
            self.cxx_actor().AddImpulse2(cpp_vec, cpp_loc)
        }
    }

    /// Applies a continuous force to the actor's center of mass.
    fn add_force(&self, vector: &Vector3D) {
        // SAFETY: FfiVector3D and carla::geom::Vector3D have identical memory layout
        unsafe {
            let cpp_vec = &*(vector.as_ffi() as *const FfiVector3D
                as *const carla_sys::carla::geom::Vector3D);
            self.cxx_actor().AddForce1(cpp_vec)
        }
    }

    /// Applies a continuous force at a specific location on the actor.
    fn add_force_at(&self, vector: &Vector3D, location: &Vector3D) {
        // SAFETY: FfiVector3D and carla::geom::Vector3D have identical memory layout
        unsafe {
            let cpp_vec = &*(vector.as_ffi() as *const FfiVector3D
                as *const carla_sys::carla::geom::Vector3D);
            let cpp_loc = &*(location.as_ffi() as *const FfiVector3D
                as *const carla_sys::carla::geom::Vector3D);
            self.cxx_actor().AddForce2(cpp_vec, cpp_loc)
        }
    }

    /// Applies an angular impulse (instantaneous rotation change).
    fn add_angular_impulse(&self, vector: &Vector3D) {
        // SAFETY: FfiVector3D and carla::geom::Vector3D have identical memory layout
        unsafe {
            let cpp_vec = &*(vector.as_ffi() as *const FfiVector3D
                as *const carla_sys::carla::geom::Vector3D);
            self.cxx_actor().AddAngularImpulse(cpp_vec)
        }
    }

    /// Applies a continuous torque (rotational force).
    fn add_torque(&self, vector: &Vector3D) {
        // SAFETY: FfiVector3D and carla::geom::Vector3D have identical memory layout
        unsafe {
            let cpp_vec = &*(vector.as_ffi() as *const FfiVector3D
                as *const carla_sys::carla::geom::Vector3D);
            self.cxx_actor().AddTorque(cpp_vec)
        }
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

    /// Returns the actor's bounding box in world coordinates.
    ///
    /// The bounding box contains the actor's extent (half-size along each axis)
    /// and its transform (position and orientation) in world space.
    ///
    /// **Note:** This method is only available in CARLA 0.9.16+. For earlier versions,
    /// use type-specific methods like [`super::Vehicle::bounding_box()`].
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
    /// let bbox = actor.bounding_box();
    /// println!("Bounding box extent: {:?}", bbox.extent);
    /// println!("Bounding box center: {:?}", bbox.transform.location);
    /// ```
    #[cfg(carla_version_0916)]
    fn bounding_box(&self) -> BoundingBox {
        let bbox = self.cxx_actor().GetBoundingBox();
        BoundingBox::from_native(&bbox)
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
