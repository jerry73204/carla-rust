use super::{Actor, ActorAttributeValueList, World};
#[cfg(carla_0100)]
use crate::rpc::{MaterialParameter, TextureColor, TextureFloatColor};
use crate::{
    error::ffi::with_ffi_error,
    geom::{BoundingBox, Location, Transform, Vector3D},
    rpc::{ActorId, ActorState},
};
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
/// # fn main() -> Result<(), Box<dyn std::error::Error>> {
/// use carla::client::{ActorBase, Client};
///
/// let client = Client::connect("localhost", 2000, None)?;
/// let mut world = client.world()?;
/// # let bp_lib = world.blueprint_library()?;
/// # let vehicle_bp = bp_lib.filter("vehicle.*").get(0).unwrap();
/// # let spawn_points = world.map()?.recommended_spawn_points();
/// # let actor = world.spawn_actor(&vehicle_bp, &spawn_points.get(0).unwrap())?;
///
/// // All actors have these methods
/// println!("Actor ID: {}", actor.id());
/// println!("Type: {}", actor.type_id());
/// println!("Location: {:?}", actor.location()?);
/// println!("Velocity: {:?}", actor.velocity()?);
/// # Ok(())
/// # }
/// ```
pub trait ActorBase: Clone {
    /// Returns the underlying FFI actor pointer.
    fn cxx_actor(&self) -> SharedPtr<FfiActor>;

    /// Converts this actor into a generic [`Actor`].
    fn into_actor(self) -> Actor {
        // SAFETY: cxx_actor() returns a valid SharedPtr for all actor implementations
        unsafe { Actor::from_cxx(self.cxx_actor()).unwrap_unchecked() }
    }

    /// Returns the unique actor ID.
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.Actor.id](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Actor.id)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.Actor.id](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.Actor.id)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.Actor.id](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.Actor.id)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    fn id(&self) -> ActorId {
        self.cxx_actor().GetId().into()
    }

    /// Returns the actor blueprint type ID (e.g., "vehicle.tesla.model3").
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.Actor.type_id](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Actor.type_id)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.Actor.type_id](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.Actor.type_id)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.Actor.type_id](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.Actor.type_id)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    fn type_id(&self) -> String {
        self.cxx_actor().GetTypeId().to_string()
    }

    /// Returns a human-readable display ID.
    fn display_id(&self) -> String {
        self.cxx_actor().GetDisplayId().to_string()
    }

    /// Returns the ID of this actor's parent, or 0 if no parent.
    fn parent_id(&self) -> ActorId {
        self.cxx_actor().GetParentId().into()
    }

    /// Returns semantic segmentation tags for this actor.
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.Actor.semantic_tags](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Actor.semantic_tags)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.Actor.semantic_tags](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.Actor.semantic_tags)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.Actor.semantic_tags](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.Actor.semantic_tags)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    fn semantic_tags(&self) -> Vec<u8> {
        self.cxx_actor().GetSemanticTags().iter().cloned().collect()
    }

    /// Returns the parent actor, if attached to one.
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.Actor.get_parent](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Actor.get_parent)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.Actor.get_parent](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.Actor.get_parent)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.Actor.get_parent](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.Actor.get_parent)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    fn parent(&self) -> crate::Result<Option<Actor>> {
        with_ffi_error("parent", |e| Actor::from_cxx(self.cxx_actor().GetParent(e)))
    }

    /// Returns the actor's blueprint attributes (color, role_name, etc.).
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.Actor.attributes](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Actor.attributes)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.Actor.attributes](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.Actor.attributes)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.Actor.attributes](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.Actor.attributes)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    fn attributes(&self) -> crate::Result<ActorAttributeValueList<'_>> {
        let ptr = with_ffi_error("attributes", |e| self.cxx_actor().GetAttributes(e))?;
        Ok(unsafe { ActorAttributeValueList::from_cxx(ptr).unwrap_unchecked() })
    }

    /// Returns the world this actor belongs to.
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.Actor.get_world](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Actor.get_world)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.Actor.get_world](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.Actor.get_world)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.Actor.get_world](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.Actor.get_world)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    fn world(&self) -> crate::Result<World> {
        let world = with_ffi_error("world", |e| self.cxx_actor().GetWorld(e))?;
        Ok(unsafe { World::from_cxx(world).unwrap_unchecked() })
    }

    /// Returns the actor's current location (position only).
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.Actor.get_location](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Actor.get_location)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.Actor.get_location](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.Actor.get_location)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.Actor.get_location](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.Actor.get_location)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    fn location(&self) -> crate::Result<Location> {
        with_ffi_error("location", |e| {
            Location::from_ffi(self.cxx_actor().GetLocation(e))
        })
    }

    /// Returns the actor's current transform (position and rotation).
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.Actor.get_transform](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Actor.get_transform)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.Actor.get_transform](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.Actor.get_transform)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.Actor.get_transform](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.Actor.get_transform)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    fn transform(&self) -> crate::Result<Transform> {
        with_ffi_error("transform", |e| {
            Transform::from_ffi(self.cxx_actor().GetTransform(e))
        })
    }

    /// Returns the actor's velocity vector in m/s.
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.Actor.get_velocity](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Actor.get_velocity)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.Actor.get_velocity](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.Actor.get_velocity)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.Actor.get_velocity](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.Actor.get_velocity)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    fn velocity(&self) -> crate::Result<Vector3D> {
        // SAFETY: carla::geom::Vector3D and FfiVector3D have identical memory layout
        with_ffi_error("velocity", |e| unsafe {
            let cpp_vec = self.cxx_actor().GetVelocity(e);
            Vector3D::from_ffi(std::mem::transmute::<
                carla_sys::carla::geom::Vector3D,
                FfiVector3D,
            >(cpp_vec))
        })
    }

    /// Returns the actor's acceleration vector in m/s².
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.Actor.get_acceleration](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Actor.get_acceleration)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.Actor.get_acceleration](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.Actor.get_acceleration)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.Actor.get_acceleration](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.Actor.get_acceleration)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    fn acceleration(&self) -> crate::Result<Vector3D> {
        // SAFETY: carla::geom::Vector3D and FfiVector3D have identical memory layout
        with_ffi_error("acceleration", |e| unsafe {
            let cpp_vec = self.cxx_actor().GetAcceleration(e);
            Vector3D::from_ffi(std::mem::transmute::<
                carla_sys::carla::geom::Vector3D,
                FfiVector3D,
            >(cpp_vec))
        })
    }

    /// Returns the actor's angular velocity in radians/s.
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.Actor.get_angular_velocity](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Actor.get_angular_velocity)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.Actor.get_angular_velocity](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.Actor.get_angular_velocity)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.Actor.get_angular_velocity](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.Actor.get_angular_velocity)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    fn angular_velocity(&self) -> crate::Result<Vector3D> {
        // SAFETY: carla::geom::Vector3D and FfiVector3D have identical memory layout
        with_ffi_error("angular_velocity", |e| unsafe {
            let cpp_vec = self.cxx_actor().GetAngularVelocity(e);
            Vector3D::from_ffi(std::mem::transmute::<
                carla_sys::carla::geom::Vector3D,
                FfiVector3D,
            >(cpp_vec))
        })
    }

    /// Teleports the actor to a new location.
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.Actor.set_location](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Actor.set_location)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.Actor.set_location](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.Actor.set_location)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.Actor.set_location](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.Actor.set_location)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    fn set_location(&self, location: &Location) -> crate::Result<()> {
        with_ffi_error("set_location", |e| {
            self.cxx_actor().SetLocation(location.as_ffi(), e)
        })
    }

    /// Teleports the actor to a new transform (position and rotation).
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.Actor.set_transform](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Actor.set_transform)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.Actor.set_transform](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.Actor.set_transform)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.Actor.set_transform](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.Actor.set_transform)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    fn set_transform(&self, transform: &Transform) -> crate::Result<()> {
        with_ffi_error("set_transform", |e| {
            self.cxx_actor().SetTransform(transform.as_ffi(), e)
        })
    }

    /// Sets the target velocity for physics simulation (m/s).
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.Actor.set_target_velocity](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Actor.set_target_velocity)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.Actor.set_target_velocity](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.Actor.set_target_velocity)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.Actor.set_target_velocity](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.Actor.set_target_velocity)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    fn set_target_velocity(&self, vector: &Vector3D) -> crate::Result<()> {
        // SAFETY: FfiVector3D and carla::geom::Vector3D have identical memory layout
        with_ffi_error("set_target_velocity", |e| unsafe {
            let cpp_vec = &*(vector.as_ffi() as *const FfiVector3D
                as *const carla_sys::carla::geom::Vector3D);
            self.cxx_actor().SetTargetVelocity(cpp_vec, e)
        })
    }

    /// Sets the target angular velocity for physics simulation (rad/s).
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.Actor.set_target_angular_velocity](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Actor.set_target_angular_velocity)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.Actor.set_target_angular_velocity](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.Actor.set_target_angular_velocity)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.Actor.set_target_angular_velocity](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.Actor.set_target_angular_velocity)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    fn set_target_angular_velocity(&self, vector: &Vector3D) -> crate::Result<()> {
        // SAFETY: FfiVector3D and carla::geom::Vector3D have identical memory layout
        with_ffi_error("set_target_angular_velocity", |e| unsafe {
            let cpp_vec = &*(vector.as_ffi() as *const FfiVector3D
                as *const carla_sys::carla::geom::Vector3D);
            self.cxx_actor().SetTargetAngularVelocity(cpp_vec, e)
        })
    }

    /// Enables constant velocity mode (actor moves at fixed velocity regardless of physics).
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.Actor.enable_constant_velocity](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Actor.enable_constant_velocity)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.Actor.enable_constant_velocity](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.Actor.enable_constant_velocity)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.Actor.enable_constant_velocity](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.Actor.enable_constant_velocity)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    fn enable_constant_velocity(&self, vector: &Vector3D) -> crate::Result<()> {
        // SAFETY: FfiVector3D and carla::geom::Vector3D have identical memory layout
        with_ffi_error("enable_constant_velocity", |e| unsafe {
            let cpp_vec = &*(vector.as_ffi() as *const FfiVector3D
                as *const carla_sys::carla::geom::Vector3D);
            self.cxx_actor().EnableConstantVelocity(cpp_vec, e)
        })
    }

    /// Disables constant velocity mode.
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.Actor.disable_constant_velocity](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Actor.disable_constant_velocity)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.Actor.disable_constant_velocity](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.Actor.disable_constant_velocity)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.Actor.disable_constant_velocity](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.Actor.disable_constant_velocity)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    fn disable_constant_velocity(&self) -> crate::Result<()> {
        with_ffi_error("disable_constant_velocity", |e| {
            self.cxx_actor().DisableConstantVelocity(e)
        })
    }

    /// Applies an impulse (instantaneous velocity change) to the actor's center of mass.
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.Actor.add_impulse](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Actor.add_impulse)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.Actor.add_impulse](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.Actor.add_impulse)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.Actor.add_impulse](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.Actor.add_impulse)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    fn add_impulse(&self, vector: &Vector3D) -> crate::Result<()> {
        // SAFETY: FfiVector3D and carla::geom::Vector3D have identical memory layout
        with_ffi_error("add_impulse", |e| unsafe {
            let cpp_vec = &*(vector.as_ffi() as *const FfiVector3D
                as *const carla_sys::carla::geom::Vector3D);
            self.cxx_actor().AddImpulse1(cpp_vec, e)
        })
    }

    /// Applies an impulse at a specific location on the actor.
    fn add_impulse_at(&self, vector: &Vector3D, location: &Vector3D) -> crate::Result<()> {
        // SAFETY: FfiVector3D and carla::geom::Vector3D have identical memory layout
        with_ffi_error("add_impulse_at", |e| unsafe {
            let cpp_vec = &*(vector.as_ffi() as *const FfiVector3D
                as *const carla_sys::carla::geom::Vector3D);
            let cpp_loc = &*(location.as_ffi() as *const FfiVector3D
                as *const carla_sys::carla::geom::Vector3D);
            self.cxx_actor().AddImpulse2(cpp_vec, cpp_loc, e)
        })
    }

    /// Applies a continuous force to the actor's center of mass.
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.Actor.add_force](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Actor.add_force)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.Actor.add_force](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.Actor.add_force)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.Actor.add_force](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.Actor.add_force)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    fn add_force(&self, vector: &Vector3D) -> crate::Result<()> {
        // SAFETY: FfiVector3D and carla::geom::Vector3D have identical memory layout
        with_ffi_error("add_force", |e| unsafe {
            let cpp_vec = &*(vector.as_ffi() as *const FfiVector3D
                as *const carla_sys::carla::geom::Vector3D);
            self.cxx_actor().AddForce1(cpp_vec, e)
        })
    }

    /// Applies a continuous force at a specific location on the actor.
    fn add_force_at(&self, vector: &Vector3D, location: &Vector3D) -> crate::Result<()> {
        // SAFETY: FfiVector3D and carla::geom::Vector3D have identical memory layout
        with_ffi_error("add_force_at", |e| unsafe {
            let cpp_vec = &*(vector.as_ffi() as *const FfiVector3D
                as *const carla_sys::carla::geom::Vector3D);
            let cpp_loc = &*(location.as_ffi() as *const FfiVector3D
                as *const carla_sys::carla::geom::Vector3D);
            self.cxx_actor().AddForce2(cpp_vec, cpp_loc, e)
        })
    }

    /// Applies an angular impulse (instantaneous rotation change).
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.Actor.add_angular_impulse](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Actor.add_angular_impulse)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.Actor.add_angular_impulse](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.Actor.add_angular_impulse)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.Actor.add_angular_impulse](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.Actor.add_angular_impulse)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    fn add_angular_impulse(&self, vector: &Vector3D) -> crate::Result<()> {
        // SAFETY: FfiVector3D and carla::geom::Vector3D have identical memory layout
        with_ffi_error("add_angular_impulse", |e| unsafe {
            let cpp_vec = &*(vector.as_ffi() as *const FfiVector3D
                as *const carla_sys::carla::geom::Vector3D);
            self.cxx_actor().AddAngularImpulse(cpp_vec, e)
        })
    }

    /// Applies a continuous torque (rotational force).
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.Actor.add_torque](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Actor.add_torque)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.Actor.add_torque](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.Actor.add_torque)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.Actor.add_torque](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.Actor.add_torque)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    fn add_torque(&self, vector: &Vector3D) -> crate::Result<()> {
        // SAFETY: FfiVector3D and carla::geom::Vector3D have identical memory layout
        with_ffi_error("add_torque", |e| unsafe {
            let cpp_vec = &*(vector.as_ffi() as *const FfiVector3D
                as *const carla_sys::carla::geom::Vector3D);
            self.cxx_actor().AddTorque(cpp_vec, e)
        })
    }

    /// Enables or disables physics simulation for this actor.
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.Actor.set_simulate_physics](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Actor.set_simulate_physics)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.Actor.set_simulate_physics](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.Actor.set_simulate_physics)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.Actor.set_simulate_physics](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.Actor.set_simulate_physics)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    fn set_simulate_physics(&self, enabled: bool) -> crate::Result<()> {
        with_ffi_error("set_simulate_physics", |e| {
            self.cxx_actor().SetSimulatePhysics(enabled, e)
        })
    }

    /// Enables or disables gravity for this actor.
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.Actor.set_enable_gravity](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Actor.set_enable_gravity)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.Actor.set_enable_gravity](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.Actor.set_enable_gravity)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.Actor.set_enable_gravity](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.Actor.set_enable_gravity)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    fn set_enable_gravity(&self, enabled: bool) -> crate::Result<()> {
        with_ffi_error("set_enable_gravity", |e| {
            self.cxx_actor().SetEnableGravity(enabled, e)
        })
    }

    /// Returns whether the actor still exists in the simulation.
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.Actor.is_alive](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Actor.is_alive)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.Actor.is_alive](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.Actor.is_alive)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.Actor.is_alive](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.Actor.is_alive)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    fn is_alive(&self) -> crate::Result<bool> {
        with_ffi_error("is_alive", |e| self.cxx_actor().IsAlive(e))
    }

    /// Returns whether the actor is currently dormant (inactive/sleeping).
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.Actor.is_dormant](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Actor.is_dormant)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.Actor.is_dormant](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.Actor.is_dormant)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.Actor.is_dormant](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.Actor.is_dormant)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    fn is_dormant(&self) -> crate::Result<bool> {
        with_ffi_error("is_dormant", |e| self.cxx_actor().IsDormant(e))
    }

    /// Returns whether the actor is currently active.
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.Actor.is_active](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Actor.is_active)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.Actor.is_active](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.Actor.is_active)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.Actor.is_active](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.Actor.is_active)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    fn is_active(&self) -> crate::Result<bool> {
        with_ffi_error("is_active", |e| self.cxx_actor().IsActive(e))
    }

    /// Returns the current lifecycle state of this actor.
    ///
    /// See [`ActorState`] for possible values:
    /// - `Invalid` — Actor is not valid
    /// - `Active` — Actor is active in the simulation
    /// - `Dormant` — Actor is dormant (hybrid mode)
    /// - `PendingKill` — Actor is pending destruction
    fn actor_state(&self) -> crate::Result<ActorState> {
        with_ffi_error("actor_state", |e| {
            let raw = self.cxx_actor().GetActorState(e);
            ActorState::from_u8(raw).unwrap_or(ActorState::Invalid)
        })
    }

    /// Returns the actor's name in the Unreal Engine world.
    ///
    /// **Available in CARLA 0.10.0+**
    #[cfg(carla_0100)]
    fn actor_name(&self) -> crate::Result<String> {
        with_ffi_error("actor_name", |e| {
            self.cxx_actor().GetActorName(e).to_string()
        })
    }

    /// Returns the actor's class name in the Unreal Engine world.
    ///
    /// **Available in CARLA 0.10.0+**
    #[cfg(carla_0100)]
    fn actor_class_name(&self) -> crate::Result<String> {
        with_ffi_error("actor_class_name", |e| {
            self.cxx_actor().GetActorClassName(e).to_string()
        })
    }

    /// Applies a color texture to this actor.
    ///
    /// **Available in CARLA 0.10.0+**
    #[cfg(carla_0100)]
    fn apply_color_texture(
        &self,
        parameter: MaterialParameter,
        texture: &TextureColor,
    ) -> crate::Result<()> {
        with_ffi_error("apply_color_texture", |e| {
            self.cxx_actor()
                .ApplyColorTexture(parameter.as_u8(), &texture.inner, e)
        })
    }

    /// Applies a float color texture to this actor.
    ///
    /// **Available in CARLA 0.10.0+**
    #[cfg(carla_0100)]
    fn apply_float_color_texture(
        &self,
        parameter: MaterialParameter,
        texture: &TextureFloatColor,
    ) -> crate::Result<()> {
        with_ffi_error("apply_float_color_texture", |e| {
            self.cxx_actor()
                .ApplyFloatColorTexture(parameter.as_u8(), &texture.inner, e)
        })
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
    /// # fn main() -> Result<(), Box<dyn std::error::Error>> {
    /// use carla::client::{ActorBase, Client};
    ///
    /// let client = Client::connect("localhost", 2000, None)?;
    /// let mut world = client.world()?;
    /// # let bp_lib = world.blueprint_library()?;
    /// # let vehicle_bp = bp_lib.filter("vehicle.*").get(0).unwrap();
    /// # let spawn_points = world.map()?.recommended_spawn_points();
    /// # let actor = world.spawn_actor(&vehicle_bp, &spawn_points.get(0).unwrap())?;
    ///
    /// let bbox = actor.bounding_box();
    /// println!("Bounding box extent: {:?}", bbox.extent);
    /// println!("Bounding box center: {:?}", bbox.transform.location);
    /// # Ok(())
    /// # }
    /// ```
    fn bounding_box(&self) -> BoundingBox {
        use autocxx::prelude::*;
        // SAFETY: GetBoundingBox returns impl New<Output = carla::geom::BoundingBox>
        // We transmute it to FfiBoundingBox since they have identical layout
        unsafe {
            let bbox_box = self.cxx_actor().GetBoundingBox().within_box();
            let cpp_bbox: &carla_sys::carla::geom::BoundingBox = bbox_box.as_ref().get_ref();
            let ffi_bbox: &carla_sys::carla_rust::geom::FfiBoundingBox =
                std::mem::transmute(cpp_bbox);
            BoundingBox::from_native(ffi_bbox)
        }
    }

    /// Destroys this actor and removes it from the simulation.
    ///
    /// Returns `true` if the actor was successfully destroyed, `false` otherwise.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// # fn main() -> Result<(), Box<dyn std::error::Error>> {
    /// use carla::client::{ActorBase, Client};
    ///
    /// let client = Client::connect("localhost", 2000, None)?;
    /// let mut world = client.world()?;
    /// # let bp_lib = world.blueprint_library()?;
    /// # let vehicle_bp = bp_lib.filter("vehicle.*").get(0).unwrap();
    /// # let spawn_points = world.map()?.recommended_spawn_points();
    /// # let actor = world.spawn_actor(&vehicle_bp, &spawn_points.get(0).unwrap())?;
    ///
    /// // Destroy the actor when done
    /// let success = actor.destroy()?;
    /// println!("Actor destroyed: {}", success);
    /// # Ok(())
    /// # }
    /// ```
    fn destroy(&self) -> crate::Result<bool> {
        with_ffi_error("destroy", |e| self.cxx_actor().Destroy(e))
    }

    /// Enables or disables collision detection for this actor.
    ///
    /// **Available in CARLA 0.9.15+**
    ///
    /// When disabled, the actor will pass through other objects without
    /// generating collision events.
    ///
    /// # Arguments
    ///
    /// * `enabled` - If true, collisions are enabled (default behavior)
    #[cfg(carla_0915)]
    fn set_collisions(&self, enabled: bool) -> crate::Result<()> {
        with_ffi_error("set_collisions", |e| {
            self.cxx_actor().SetCollisions(enabled, e)
        })
    }

    /// Marks this actor as dead.
    ///
    /// **Available in CARLA 0.9.15+**
    ///
    /// Sets the actor state to dead without immediately destroying it.
    /// This is used for ragdoll effects on walkers and similar scenarios.
    #[cfg(carla_0915)]
    fn set_actor_dead(&self) -> crate::Result<()> {
        with_ffi_error("set_actor_dead", |e| self.cxx_actor().SetActorDead(e))
    }
}
